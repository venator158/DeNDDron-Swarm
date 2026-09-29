"""Decentralized, priority-ordered multi-drone assignment of threats.

No zenoh dependency: the agent feeds this class waves, bids and awards and
acts on the decisions it returns, so the protocol is unit-testable.

Protocol (one round per wave of threats):
  1. The dispatcher announces a wave on ``swarm/threats``.
  2. Every free drone bids its cost for *every* threat in the wave on
     ``swarm/bids`` (one message per drone).
  3. ``bid_window_s`` after an agent received the wave (in its own clock) it
     runs ``assign()``: threats in priority order, each taking its ``required``
     cheapest still-unassigned bidders.  The ordering and the (cost, agent_id)
     tie-break make every agent that saw the same bids compute the same result.
  4. Each winner publishes an award on ``swarm/awards``.  If views diverged and
     a threat collects more holders than its level, a holder yields (and
     publishes a withdrawal) once it sees ``level`` holders with better bids.
"""

from dataclasses import dataclass, field
from typing import Dict, List, Optional, Tuple

from threats import Threat, priority_order


def bid_key(cost: float, agent_id: str) -> Tuple[float, str]:
    """Total order on bids: lower cost wins, ties broken by agent_id."""
    return (float(cost), str(agent_id))


def assign(threats: List[Threat], bids: Dict[str, Dict[str, float]]) -> Dict[str, List[str]]:
    """Deterministic priority-greedy assignment.

    ``bids`` maps agent_id -> {threat_id: cost}.  Each agent is assigned to at
    most one threat.  Returns threat_id -> winning agent_ids (may be shorter
    than ``required`` if too few drones bid).
    """
    taken = set()
    result = {}
    for threat in priority_order(threats):
        candidates = sorted(
            bid_key(costs[threat.threat_id], agent)
            for agent, costs in bids.items()
            if agent not in taken and threat.threat_id in costs
        )
        winners = [agent for _, agent in candidates[:threat.required]]
        taken.update(winners)
        result[threat.threat_id] = winners
    return result


@dataclass
class _Wave:
    wave_id: str
    threats: List[Threat]
    closes_at: float
    bids: Dict[str, Dict[str, float]] = field(default_factory=dict)
    closed: bool = False


@dataclass(frozen=True)
class WaveResult:
    wave_id: str
    assignment: Dict[str, List[str]]
    my_threat: Optional[Threat]      # threat this agent must engage, if any
    my_cost: Optional[float]


class AuctionManager:
    def __init__(self, agent_id: str, bid_window_s: float = 1.0, max_history: int = 256):
        if bid_window_s <= 0:
            raise ValueError("bid_window_s must be > 0")
        self.agent_id = agent_id
        self.bid_window_s = float(bid_window_s)
        self.max_history = int(max_history)
        self._waves: Dict[str, _Wave] = {}
        # Bids can arrive before the wave announcement does.
        self._early_bids: Dict[str, Dict[str, Dict[str, float]]] = {}
        # threat_id -> {agent_id: cost} of drones currently holding that threat
        self._holders: Dict[str, Dict[str, float]] = {}
        # Threat this agent currently holds, with its own bid cost.
        self.held: Optional[Threat] = None
        self.held_cost: Optional[float] = None

    # ------------------------------------------------------------------
    def on_wave(self, wave_id: str, threats: List[Threat], now: float) -> bool:
        """Open a wave auction.  Returns True if it is new (caller may bid)."""
        if wave_id in self._waves:
            return False
        wave = _Wave(wave_id, list(threats), now + self.bid_window_s)
        wave.bids.update(self._early_bids.pop(wave_id, {}))
        self._waves[wave_id] = wave
        self._trim()
        return True

    def on_bid(self, wave_id: str, agent_id: str, costs: Dict[str, float]) -> None:
        costs = {str(k): float(v) for k, v in costs.items()}
        wave = self._waves.get(wave_id)
        if wave is None:
            self._early_bids.setdefault(wave_id, {})[agent_id] = costs
        elif not wave.closed:
            wave.bids[agent_id] = costs

    def close_due(self, now: float) -> List[WaveResult]:
        results = []
        for wave in self._waves.values():
            if wave.closed or now < wave.closes_at:
                continue
            wave.closed = True
            assignment = assign(wave.threats, wave.bids)
            mine = None
            for threat in wave.threats:
                if self.agent_id in assignment.get(threat.threat_id, []):
                    mine = threat
            my_cost = wave.bids.get(self.agent_id, {}).get(mine.threat_id) if mine else None
            if mine is not None:
                self.held, self.held_cost = mine, my_cost
                self._holders.setdefault(mine.threat_id, {})[self.agent_id] = my_cost
            results.append(WaveResult(wave.wave_id, assignment, mine, my_cost))
        return results

    def on_award(self, threat_id: str, agent_id: str, cost: float) -> bool:
        """Record another drone's award.  Returns True if *we* must yield our threat."""
        self._holders.setdefault(threat_id, {})[agent_id] = float(cost)
        return self._must_yield()

    def on_withdraw(self, threat_id: str, agent_id: str) -> None:
        self._holders.get(threat_id, {}).pop(agent_id, None)

    def release(self) -> None:
        """Drop the held threat (after yielding or being expended)."""
        if self.held is not None:
            self._holders.get(self.held.threat_id, {}).pop(self.agent_id, None)
        self.held, self.held_cost = None, None

    def has_open_waves(self) -> bool:
        return any(not w.closed for w in self._waves.values())

    # ------------------------------------------------------------------
    def _must_yield(self) -> bool:
        if self.held is None or self.held_cost is None:
            return False
        mine = bid_key(self.held_cost, self.agent_id)
        better = sum(
            1 for agent, cost in self._holders.get(self.held.threat_id, {}).items()
            if agent != self.agent_id and bid_key(cost, agent) < mine
        )
        return better >= self.held.level

    def _trim(self) -> None:
        while len(self._waves) > self.max_history:
            oldest = next(iter(self._waves))
            if not self._waves[oldest].closed:
                break
            del self._waves[oldest]
        while len(self._early_bids) > self.max_history:
            del self._early_bids[next(iter(self._early_bids))]
