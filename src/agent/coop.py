"""Cooperative localization on top of localization.Localizer (LOCALIZATION=coop).

No zenoh dependency, so it is unit-tested.

Every UWB ranging exchange carries the responder's state (its self-estimate, covariance, velocity,
altitude and anchor hops).  This module keeps the drone's neighbour table from those payloads and
decides which peer ranges may update the drone's own estimate.

- Neighbour table: agent -> Neighbour, propagated forward with its velocity, its covariance growing
  with age (accel_sigma).  Entries older than stale_s are dropped.
- Anchoring: anchor_t is the time of the last ship-anchor fix in our chain - our own last accepted
  anchor range, or, through peers, the anchor_t of the peer we used.  Absolute position is only
  observable through the anchors: peer ranges alone leave the swarm free to shift and rotate as a
  whole, so a drone whose chain stops being refreshed drifts (its covariance grows).  A hop count
  was used first, but hops can be passed around a loop (A from B, B from A) and counted to
  infinity with every anchor jammed; anchor_t only gets older along a chain and stops at real
  anchor fixes, so loops are impossible.  hops (0 = ranging anchors) is still reported.
- Peer fusion: a drone ranging the anchors (fix within anchor_fresh_s) does not use peers.  Others
  use a peer's range only if the peer's chain is fresher (its anchor_t later than ours), with the
  peer's uncertainty along the line of sight added to the ranging noise.  The peer's error is a
  slowly varying bias, not fresh noise, so many updates must not average it away (that made
  estimates 7x overconfident in simulation): after each one, our variance along the line of sight
  is floored at the peer's plus the ranging noise.  Consistency is measured (NEES); a fully
  consistent scheme (RDL, Luft et al. 2018, cross-covariances with pairwise exchanges) is the
  planned upgrade, needed to keep relative positions tight when no chain reaches the anchors.
- Peer selection: the max_peers nearest neighbours by the table, plus one rotating slot to discover
  roster members not yet in the table.
"""

import math
from dataclasses import dataclass
from typing import Dict, List, Optional, Sequence, Tuple

import numpy as np

LOST_HOPS = 99


@dataclass
class Neighbour:
    x: float
    y: float
    vx: float
    vy: float
    cov: np.ndarray          # 2x2 position covariance at time t
    z: float
    t: float                 # time of its estimate
    hops: int
    anchor_t: Optional[float]  # time of the last anchor fix in its chain (None: never)

    def at(self, t: float, accel_sigma: float = 0.5) -> Tuple[float, float, np.ndarray]:
        """Position and covariance propagated to time t (constant velocity, growing uncertainty)."""
        dt = max(0.0, t - self.t)
        q = (accel_sigma ** 2) * dt ** 3 / 3.0 + 0.04 * dt          # accel noise + unknown velocity error
        return self.x + self.vx * dt, self.y + self.vy * dt, self.cov + np.eye(2) * q


def state_payload(loc, z: float, t: float, hops: int, anchor_t: Optional[float] = None) -> dict:
    """What our UWB frames carry: our estimate for peers' tables and peer updates."""
    x, y = loc.position()
    vx, vy = loc.velocity()
    P = loc.pos_cov()
    return {"x": round(x, 3), "y": round(y, 3), "vx": round(vx, 3), "vy": round(vy, 3),
            "cov": [round(float(P[0, 0]), 5), round(float(P[0, 1]), 5), round(float(P[1, 1]), 5)],
            "z": round(z, 3), "t": round(t, 3), "hops": hops,
            "anchor_t": None if anchor_t is None else round(anchor_t, 3)}


class Coop:
    def __init__(self, me: str, max_peers: int = 6, stale_s: float = 5.0, anchor_fresh_s: float = 3.0,
                 accel_sigma: float = 0.5):
        self.me = me
        self.max_peers = max_peers
        self.stale_s = stale_s
        self.anchor_fresh_s = anchor_fresh_s
        self.accel_sigma = accel_sigma
        self.table: Dict[str, Neighbour] = {}
        self.last_anchor_t: Optional[float] = None
        self._peer_used: Dict[str, Tuple[float, int, float]] = {}   # peer -> (time used, its hops, its anchor_t)
        self._rr = 0
        self.peer_updates = 0
        self.peer_skipped = 0

    # --- anchoring --------------------------------------------------------
    def anchored(self, t: float) -> bool:
        return self.last_anchor_t is not None and t - self.last_anchor_t <= self.anchor_fresh_s

    def anchor_t(self) -> Optional[float]:
        """Time of the last anchor fix in our chain (our own, or through a peer we used)."""
        best = self.last_anchor_t
        for (_, _, at) in self._peer_used.values():
            if at is not None and (best is None or at > best):
                best = at
        return best

    def hops(self, t: float) -> int:
        if self.anchored(t):
            return 0
        best = self.anchor_t()
        recent = [(at, h) for (tu, h, at) in self._peer_used.values()
                  if t - tu <= self.anchor_fresh_s and at is not None and at == best]
        return min(h for _, h in recent) + 1 if recent else LOST_HOPS

    def on_anchor_update(self, t: float) -> None:
        self.last_anchor_t = t

    # --- neighbour table --------------------------------------------------
    def on_payload(self, peer: str, s: dict) -> None:
        c = s["cov"]
        at = s.get("anchor_t")
        self.table[peer] = Neighbour(float(s["x"]), float(s["y"]), float(s["vx"]), float(s["vy"]),
                                     np.array([[c[0], c[1]], [c[1], c[2]]], dtype=float),
                                     float(s["z"]), float(s["t"]), int(s.get("hops", LOST_HOPS)),
                                     None if at is None else float(at))

    def prune(self, t: float) -> None:
        for k in [k for k, n in self.table.items() if t - n.t > self.stale_s]:
            del self.table[k]

    def neighbours(self, t: float) -> List[Tuple[str, float, float, float, np.ndarray]]:
        """(id, x, y, z, covariance) of every fresh neighbour, propagated to t."""
        out = []
        for k, n in self.table.items():
            if t - n.t <= self.stale_s:
                x, y, P = n.at(t, self.accel_sigma)
                out.append((k, x, y, n.z, P))
        return out

    # --- ranging --------------------------------------------------------------
    def choose_peers(self, own_xy: Tuple[float, float], t: float, roster: Sequence[str]) -> List[str]:
        """Peers to range next: the nearest known ones, plus one rotating unknown roster member."""
        self.prune(t)
        known = sorted(((math.dist(own_xy, (x, y)), k) for k, x, y, _, _ in self.neighbours(t)))
        chosen = [k for _, k in known[: max(0, self.max_peers - 1)]]
        unknown = [r for r in roster if r != self.me and r not in chosen]
        if unknown:
            self._rr = (self._rr + 1) % len(unknown)
            chosen.append(unknown[self._rr])
        return chosen[: self.max_peers]

    def peer_update(self, loc, peer: str, r: float, z: float, t: float, sigma: Optional[float] = None,
                    ref_anchor_t: Optional[float] = "now") -> bool:
        """Use a range to `peer` (its payload already in the table) if its anchor chain is fresher than
        ours was before this ranging round (ref_anchor_t: take anchor_t() once per round, so several
        peers anchored at the same moment can all be used; loops stay impossible because a used
        peer's anchor_t must strictly exceed what we had)."""
        n = self.table.get(peer)
        if n is None or loc.x is None:
            return False
        mine = self.anchor_t() if ref_anchor_t == "now" else ref_anchor_t
        if self.anchored(t) or n.anchor_t is None or (mine is not None and n.anchor_t <= mine):
            self.peer_skipped += 1
            return False
        px, py, P = n.at(t, self.accel_sigma)
        s = loc.noise_sigma() if sigma is None else sigma
        dx, dy, dz = loc.x[0] - px, loc.x[1] - py, z - n.z
        h = math.sqrt(dx * dx + dy * dy + dz * dz)
        if h < 1e-6:
            return False
        u = np.array([dx / h, dy / h])
        s_eff = math.sqrt(s * s + float(u @ P @ u))        # the peer's uncertainty along the line of sight
        ok = loc.update_range((px, py, n.z), r, z, sigma=s_eff, t=t)
        if ok:
            # Floor our variance along the line of sight at the peer's + ranging noise: its error is a
            # persistent bias, which repeated updates must not average away.
            floor = float(u @ P @ u) + s * s
            have = float(u @ loc.P[:2, :2] @ u)
            if have < floor:
                loc.P[:2, :2] += (floor - have) * np.outer(u, u)
            self._peer_used[peer] = (t, n.hops, n.anchor_t)
            self.peer_updates += 1
        return ok
