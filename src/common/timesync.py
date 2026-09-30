"""Clock synchronization: how a node relates its local clock (localclock.py) to ship time.

No zenoh dependency, so it is unit-tested.  Absolute times on the radio are in the ship's
timebase.  A drone keeps its protocol times in a *protocol timebase* and compares them with
proto_time(local):

- inbound(t, sent, local_rx): an absolute ship time from a message (stamped `sent` by the ship
  when the mode needs it), converted to the protocol timebase on receipt;
- to_ship(p): a protocol time back as an estimate of ship time, for stamps the drone sends
  (detonation sync_time, heartbeat time) and for comparing them with other drones' stamps.

Modes (CLOCK_SYNC):

- none:   trust the local clock as ship time (default; with perfect clocks this is exact).
- ttg:    time-to-go.  The ship stamps each message with its send time; the drone anchors every
          absolute time at its own receipt, local_rx + (t - sent).  The protocol timebase is the
          local clock, and every job update re-anchors.  The unknown one-way delay is error.
- master: two-way exchange with the ship (NTP-style, 4 timestamps: t1 drone send, t2 ship
          receive, t3 ship send, t4 drone receive), filtered by a Kalman filter on offset and
          rate (OffsetRateFilter).  The protocol timebase is the estimate of ship time.
- consensus: ship-anchored consensus in the style of ATS (average time synchronization).  Every
          node keeps a virtual clock v = alpha * local + o and beacons (local stamp, alpha, o,
          bound); each node moves its rate and offset towards the aggregate of its neighbours'.
          The ship is a pinned leader (it never adjusts), so drones connected to it, directly or
          through peers, converge to ship time; without it they keep agreeing with each other.
"""

import math
import os
from collections import deque
from typing import Callable, Dict, List, Optional, Sequence, Tuple

MODES = ("none", "ttg", "master", "consensus")


class NoSync:
    """No synchronization: the local clock is taken as ship time."""
    mode = "none"

    def proto_time(self, local: float) -> float:
        return local

    def inbound(self, t_ship: float, sent: Optional[float], local_rx: Optional[float]) -> float:
        return t_ship

    def on_sent(self, sent: Optional[float], local_rx: Optional[float]) -> None:
        """A message stamped with the ship's send time arrived (used by ttg)."""

    def to_ship(self, proto: float) -> float:
        return proto

    def offset(self, local: float) -> Optional[float]:
        """Estimated ship time - local time (s); None if the mode has no estimate."""
        return None

    def error_bound(self, local: float) -> Optional[float]:
        """This node's own bound on |ship time error| (s); None if it has none."""
        return None


class TimeToGo(NoSync):
    """Absolute times are re-anchored on receipt: t -> local_rx + (t - sent)."""
    mode = "ttg"

    def __init__(self):
        self._anchor = None          # sent - local_rx of the latest message: ship - local, delay included

    def on_sent(self, sent, local_rx) -> None:
        """Any stamped message re-anchors, also one that carries no times (e.g. an empty zone list)."""
        if sent is not None and local_rx is not None:
            self._anchor = float(sent) - local_rx

    def inbound(self, t_ship, sent, local_rx):
        if sent is None or local_rx is None:
            return t_ship
        self.on_sent(sent, local_rx)
        return local_rx + (float(t_ship) - float(sent))

    def to_ship(self, proto):
        return proto if self._anchor is None else proto + self._anchor

    def offset(self, local):
        return self._anchor


def exchange(t1: float, t2: float, t3: float, t4: float) -> Tuple[float, float]:
    """(offset, round-trip delay) of one two-way exchange: offset = ship - local at the exchange,
    assuming equal delays both ways (an asymmetry a biases it by a/2, and |bias| <= delay/2)."""
    return ((t2 - t1) + (t3 - t4)) / 2.0, (t4 - t1) - (t3 - t2)


class OffsetRateFilter:
    """Kalman filter on x = (offset, rate) of ship time relative to local time.

    offset(t) = x0 + x1 * (t - t_ref); x1 is the relative rate error (s/s).  An exchange's offset is
    wrong by half the asymmetry of its delays, at most half its round trip, so each sample is weighed
    by R = floor^2 + (delay / 2)^2: long round trips (queueing, a congested start) count for little.
    Judging a sample only against the recent minimum round trip is not enough: at startup every
    round trip was long, and lopsided ones with the same total were trusted fully and corrupted the
    rate for a minute.  Once settled (`settle` samples), a sample more than `gate` standard
    deviations from the prediction is rejected as an outlier; after `relock` rejections in a row
    the filter re-acquires from scratch (a wrong lock otherwise rejects every good sample forever:
    seen at 50 drones, errors grew to 700 ms).  An exchange whose round trip is clearly negative
    (below -neg_tol_s) is impossible, so its stamps are wrong: it is discarded, not clamped to 0,
    which made it the most trusted sample of all.  An asymmetry common to all samples cannot be
    observed; the self-reported error bound includes half the best round trip for it.
    """

    def __init__(self, floor_s: float = 1e-3, q_offset: float = 1e-8, q_rate: float = 1e-12,
                 p0_rate: float = 1e-6, window: int = 32, gate: float = 5.0, settle: int = 8,
                 relock: int = 5, neg_tol_s: float = 0.005):
        self.floor_s, self.q_offset, self.q_rate, self.p0_rate = floor_s, q_offset, q_rate, p0_rate
        self.window, self.gate, self.settle = window, gate, settle
        self.relock, self.neg_tol_s = relock, neg_tol_s
        self.rejected = 0            # gate rejections, all time
        self.invalid = 0             # impossible (negative round trip) exchanges discarded
        self.relocks = 0
        self._run = 0                # gate rejections in a row
        self._since_lock = 0         # samples since the last (re)acquisition
        self.x = None                # [offset at t_ref, rate]
        self.P = None                # 2x2 covariance
        self.t_ref = None
        self.delays = []             # recent round trips
        self.samples = 0

    def _predict(self, t: float):
        dt = t - self.t_ref
        if dt <= 0.0:
            return
        x0, x1 = self.x
        (p00, p01), (p10, p11) = self.P
        self.x = [x0 + x1 * dt, x1]
        # F = [[1, dt], [0, 1]];  Q = diag(q_offset, q_rate) * dt
        self.P = [[p00 + dt * (p01 + p10) + dt * dt * p11 + self.q_offset * dt, p01 + dt * p11],
                  [p10 + dt * p11, p11 + self.q_rate * dt]]
        self.t_ref = t

    def update(self, t_local: float, offset: float, delay: float) -> None:
        if delay < -self.neg_tol_s:
            self.invalid += 1
            return
        delay = max(0.0, delay)
        self.delays = (self.delays + [delay])[-self.window:]
        self.samples += 1
        self._since_lock += 1
        if self.x is None:
            self.x, self.t_ref = [offset, 0.0], t_local
            r0 = self.floor_s ** 2 + (delay / 2.0) ** 2
            self.P = [[r0, 0.0], [0.0, self.p0_rate]]
            return
        self._predict(t_local)
        r = self.floor_s ** 2 + (delay / 2.0) ** 2
        (p00, p01), (p10, p11) = self.P
        s = p00 + r
        innov = offset - self.x[0]
        if self._since_lock > self.settle and innov * innov > self.gate * self.gate * s:
            self.rejected += 1
            self._run += 1
            if self._run >= self.relock:
                # Consistently far from what we believe: our lock is wrong, not the samples.
                self.relocks += 1
                self.x, self.P, self.t_ref, self._run, self._since_lock = None, None, None, 0, 0
                self.delays = []
                self.samples -= 1
                self.update(t_local, offset, delay)
            return
        self._run = 0
        k0, k1 = p00 / s, p10 / s
        self.x = [self.x[0] + k0 * innov, self.x[1] + k1 * innov]
        self.P = [[(1 - k0) * p00, (1 - k0) * p01], [p10 - k1 * p00, p11 - k1 * p01]]

    def offset(self, t_local: float) -> Optional[float]:
        if self.x is None:
            return None
        return self.x[0] + self.x[1] * (t_local - self.t_ref)

    def rate(self) -> Optional[float]:
        return None if self.x is None else self.x[1]

    def error_bound(self, t_local: float) -> Optional[float]:
        """3 sigma of the offset estimate, extrapolated to t_local, plus half the best round trip."""
        if self.x is None:
            return None
        dt = max(0.0, t_local - self.t_ref)
        (p00, p01), (p10, p11) = self.P
        var = p00 + dt * (p01 + p10) + dt * dt * p11 + self.q_offset * dt
        return 3.0 * math.sqrt(max(var, 0.0)) + min(self.delays) / 2.0


class ShipMaster(NoSync):
    """Two-way exchanges with the ship (t1 in the heartbeat, t2/t3 echoed in the roster)."""
    mode = "master"

    def __init__(self, **filter_kw):
        self.filter = OffsetRateFilter(**filter_kw)
        self._last_t1 = None
        self.last_exchange = None            # (t1, t2, t3, t4), for evaluation logging

    def on_exchange(self, t1: float, t2: float, t3: float, t4: float) -> bool:
        """Feed one exchange; a repeat of the last one (no newer heartbeat reached the ship) is ignored."""
        if t1 == self._last_t1:
            return False
        self._last_t1 = t1
        self.last_exchange = (t1, t2, t3, t4)
        offset, delay = exchange(t1, t2, t3, t4)
        self.filter.update(t4, offset, delay)
        return True

    def proto_time(self, local):
        off = self.filter.offset(local)
        return local if off is None else local + off

    def offset(self, local):
        return self.filter.offset(local)

    def error_bound(self, local):
        return self.filter.error_bound(local)


def mean_aggregate(own: float, others: Sequence[float]) -> float:
    """Plain average of our own value and the neighbours' (ATS).  Aggregators get our own value
    separately so resilient variants (trimmed mean, MSR: drop the f most extreme neighbours
    relative to our own value) can be swapped in."""
    return (own + sum(others)) / (1 + len(others))


AGGREGATORS: Dict[str, Callable[[float, Sequence[float]], float]] = {"mean": mean_aggregate}


class _Source:
    """What we know of one neighbour's clock (a peer drone, or the ship as leader)."""

    def __init__(self, leader: bool, window: int):
        self.leader = leader
        # (our receive stamp, its send stamp), raw: the rate fit must not see the delay correction,
        # whose changes tilted the slope (a congested start inflated it for minutes)
        self.samples = deque(maxlen=window)
        self.alpha, self.o = 1.0, 0.0
        self.anchor = None                   # (ship time its chain last touched the ship, hops)
        self.last = None                     # (its send stamp, our receive stamp): echoed back to it
        self.rtts = deque(maxlen=16)         # (our local time, round trip) through echoes / exchanges
        self.hw_hist = deque(maxlen=12)      # (our local t4, its local - ours, round trip) per two-way exchange
        self.echoed_at = -math.inf

    @property
    def last_rx(self) -> float:
        return self.last[1]

    def hw(self, filter_age_s: float = 10.0):
        """(t4, offset) of the exchange to trust: the lowest round trip among the last filter_age_s
        (NTP's clock filter; a lopsided, delayed exchange has a long round trip), else the newest."""
        if not self.hw_hist:
            return None
        newest = self.hw_hist[-1][0]
        recent = [h for h in self.hw_hist if newest - h[0] <= filter_age_s] or [self.hw_hist[-1]]
        t4, theta, _ = min(recent, key=lambda h: h[2])
        return t4, theta

    def delay(self, max_age_s: float = 20.0, newest_n: int = 5) -> float:
        """One-way delay estimate: half the median of the newest_n round trips of the last max_age_s
        (else the newest one; 0 until one is measured).  The median, not the minimum: any bias left in every
        neighbour's estimate makes averaging walk the swarm by ~gain x bias per beacon when the
        ship is not there to anchor it (the minimum under-estimated a 2-10 ms delay by 2.5 ms:
        0.55 ms/s).  The median is unbiased for symmetric delays and ignores queueing spikes; the
        limits drop the long round trips of a congested start within seconds (the ship link is
        measured every second, a peer link about every N beacons)."""
        if not self.rtts:
            return 0.0
        newest = self.rtts[-1][0]
        r = sorted([rtt for t, rtt in self.rtts if newest - t <= max_age_s][-newest_n:]) or [self.rtts[-1][1]]
        return (r[(len(r) - 1) // 2] + r[len(r) // 2]) / 4.0


class Consensus(NoSync):
    """Ship-anchored ATS-style consensus.

    Per neighbour j, its hardware rate relative to ours (eta_j) is a least-squares fit of its raw
    local stamps against ours over the recent beacons: raw oscillators are linear, so the fit is
    clean while virtual clocks are still moving (the ATS trick).  Its virtual clock now, in our
    local time, is v_j = alpha_j * (tau_j + eta_j * (local - sent)) + o_j, and its virtual rate is
    alpha_j * eta_j.  Each step() moves our alpha and our virtual time `gain` of the way towards

        target = aggregate(own, peers)                          without the ship
        target = share * ship + (1 - share) * aggregate(...)    while we hear the ship

    The leader share keeps the swarm's common mode converging to the ship in a few updates; plain
    averaging with the ship as one neighbour among N would take ~N/gain updates.

    Two refinements for startup and late joiners:
    - Sources with an anchor (a chain to the ship, below) outrank free-running ones: while any fresh
      source is anchored, only anchored ones count, so drones that have never heard of the ship
      cannot pull synced ones around.  With none anchored, it is plain mutual consensus.
    - Step, then slew (as NTP does): more than step_s from an anchored target, the clock jumps to
      it (the ship's estimate if heard, else the anchored peers' aggregate).  Slewing 25% per beacon
      took 60-70 s to bring +-3 s offsets under 10 ms.  Smaller corrections move by the gain.

    One-way delay.  Classic ATS ignores it; here it biased every node ~2 delays behind the ship,
    and with the ship lost the whole swarm walked backwards by ~gain x delay per beacon.
    Correcting one-way samples with a per-link delay estimate was not enough either: peer links
    are measured rarely, and after a congested start their stale estimates pulled drones 100 ms
    off the ship.  So offsets come from two-way exchanges: every beacon echoes the last beacon
    heard from one neighbour (rotating), and the neighbour's next beacon completes a four-timestamp
    exchange whose offset ((t2 - t1) + (t3 - t4)) / 2 needs no delay estimate (exact for symmetric
    delays; an asymmetry is still an error of half of it).  Per source, the exchange with the lowest
    round trip of the last 10 s is used (NTP's clock filter), and one with a clearly negative round
    trip is discarded as wrong stamps.  Between exchanges the fitted rate carries it forward.  The ship link uses the heartbeat/roster exchange (as in master mode).  A
    source with no exchange yet is used, one-way, only if no exchanged source is fresh.

    Error bound (heuristic, self-reported).  Every node carries an anchor: the ship time at which
    its chain of sources last touched the ship, and the number of hops.  A newer anchor wins, and
    for the same anchor fewer hops (so a lost ship cannot make hop counts climb around loops, which
    an earlier bound built from neighbours' bounds did: 1.1 s after 5 min).  The bound is
    hops x delay_bound + rate_unc x (time since the anchor) + the residual disagreement.
    """
    mode = "consensus"

    def __init__(self, node_id: str = "", gain: float = 0.5, leader_share: float = 0.5,
                 stale_s: float = 6.0, leader_stale_s: float = 3.0, window: int = 30,
                 min_span_s: float = 10.0, delay_bound_s: float = 0.005, rate_unc: float = 5e-5,
                 step_s: float = 0.05, max_rate_se: float = 1e-4, neg_tol_s: float = 0.005,
                 aggregate="mean"):
        self.node_id = node_id
        self.gain, self.leader_share = gain, leader_share
        self.stale_s, self.leader_stale_s = stale_s, leader_stale_s
        self.window, self.min_span_s = window, min_span_s
        self.delay_bound_s, self.rate_unc, self.step_s = delay_bound_s, rate_unc, step_s
        self.max_rate_se, self.neg_tol_s = max_rate_se, neg_tol_s
        self.invalid = 0                     # impossible (negative round trip) exchanges discarded
        self.steps = 0
        self.aggregate = AGGREGATORS[aggregate] if isinstance(aggregate, str) else aggregate
        self.alpha, self.o = 1.0, 0.0
        self.sources: Dict[str, _Source] = {}
        self.anchor = None                   # (ship time, hops): see the class docstring
        self._resid = 0.0

    # --- clock ----------------------------------------------------------
    def proto_time(self, local):
        return self.alpha * local + self.o

    def offset(self, local):
        return self.proto_time(local) - local

    def error_bound(self, local):
        if self.anchor is None:
            return None
        anchor_t, hops = self.anchor
        return (hops * self.delay_bound_s + self.rate_unc * max(0.0, self.proto_time(local) - anchor_t)
                + self._resid)

    def _take_anchor(self, anchor) -> None:
        if anchor is None:
            return
        t, hops = float(anchor[0]), int(anchor[1])
        if self.anchor is None or t > self.anchor[0] or (t == self.anchor[0] and hops < self.anchor[1]):
            self.anchor = (t, hops)

    def beacon(self, local: float) -> dict:
        """Our beacon, stamped `local` (the send stamp, with jitter), echoing one neighbour's last
        beacon so it can measure the round trip (the neighbour we echoed longest ago)."""
        b = {"tau": round(local, 6), "alpha": self.alpha, "o": self.o,
             "anchor": None if self.anchor is None else [self.anchor[0], self.anchor[1]]}
        peers = [(s.echoed_at, k) for k, s in self.sources.items()
                 if not s.leader and s.last is not None and k in self.fresh(local)]
        if peers:
            _, k = min(peers)
            s = self.sources[k]
            s.echoed_at = local
            b["echo"] = [k, round(s.last[0], 6), round(s.last[1], 6)]
        return b

    # --- neighbours -----------------------------------------------------
    def _source(self, src: str, leader: bool) -> _Source:
        s = self.sources.get(src)
        if s is None:
            s = self.sources[src] = _Source(leader, self.window)
        return s

    def on_beacon(self, src: str, tau: float, alpha: float, o: float, anchor, local_rx: float,
                  leader: bool = False, echo=None) -> None:
        s = self._source(src, leader)
        if echo and echo[0] == self.node_id:
            # Our earlier beacon (sent at echo[1], our clock) reached it at echo[2] (its clock); it
            # sent this one at tau and we got it at local_rx: a round trip, its hold time converted
            # to our clock with the fitted rate.
            t1, t2, t3, t4 = float(echo[1]), float(echo[2]), float(tau), float(local_rx)
            eta = self._eta(s) or 1.0
            self._exchange(s, t4, exchange(t1, t2, t3, t4)[0], (t4 - t1) - (t3 - t2) / eta)
        s.last = (float(tau), float(local_rx))
        s.samples.append((float(local_rx), float(tau)))
        s.alpha, s.o = float(alpha), float(o)
        s.anchor = None if anchor is None else (float(anchor[0]), int(anchor[1]) + 1)

    def _exchange(self, s: _Source, t4: float, offset: float, rtt: float) -> None:
        """Record one two-way exchange.  A clearly negative round trip means wrong stamps: discard it
        (clamped to 0 it would look like the best exchange of all)."""
        if rtt < -self.neg_tol_s:
            self.invalid += 1
            return
        rtt = max(0.0, rtt)
        s.rtts.append((t4, rtt))
        s.hw_hist.append((t4, offset, rtt))

    def on_leader(self, t3: float, t4: float, exchange_=None) -> None:
        """The ship's roster: sent at t3 (ship clock), received at t4 (ours).  exchange_ = (t1, t2)
        when it echoes our heartbeat, giving the link's round trip."""
        s = self._source("ship", True)
        if exchange_ is not None:
            t1, t2 = exchange_
            offset, rtt = exchange(t1, t2, t3, t4)
            self._exchange(s, float(t4), offset, rtt)
        self.on_beacon("ship", t3, 1.0, 0.0, (t3, 0), t4, leader=True)

    @staticmethod
    def _fit(pts):
        """Least-squares line through pts: (slope, residuals, sxx) or None."""
        n = len(pts)
        mx = sum(x for x, _ in pts) / n
        my = sum(y for _, y in pts) / n
        sxx = sum((x - mx) ** 2 for x, _ in pts)
        if sxx <= 0:
            return None
        k = sum((x - mx) * (y - my) for x, y in pts) / sxx
        return k, [y - my - k * (x - mx) for x, y in pts], sxx

    def _eta(self, s: _Source) -> Optional[float]:
        """Its hardware rate relative to ours: a robust line fit of its raw stamps against ours
        (samples beyond 3 x the median absolute residual dropped, then refit), used only when the
        beacons span min_span_s and the slope's standard error is under max_rate_se.  Otherwise
        None (rates are assumed equal: at 500 ppm that is 1 ms per 2 s beacon).  Unchecked, the
        long, uneven round trips of a congested start put rates 1% off, and with the ship lost the
        rate average then walked."""
        pts = list(s.samples)
        if len(pts) < 5 or pts[-1][0] - pts[0][0] < self.min_span_s:
            return None
        f = self._fit(pts)
        if f is None:
            return None
        res = sorted(abs(r) for r in f[1])
        cut = 3.0 * max(1.4826 * res[len(res) // 2], 1e-3)
        pts = [p for p, r in zip(pts, f[1]) if abs(r) <= cut]
        if len(pts) < 5 or pts[-1][0] - pts[0][0] < self.min_span_s:
            return None
        f = self._fit(pts)
        if f is None:
            return None
        k, res, sxx = f
        se = math.sqrt(sum(r * r for r in res) / max(1, len(res) - 2) / sxx)
        return k if se <= self.max_rate_se else None

    def _estimate(self, s: _Source, local: float):
        """(its virtual time now, in our local time; its virtual rate per our local second, or None)."""
        eta = self._eta(s)
        k = eta if eta is not None else 1.0
        hw = s.hw()
        if hw is not None:
            t4, theta = hw                         # its clock read t4 + theta when ours read t4
            tau_now = t4 + theta + k * (local - t4)
        else:
            rx, tau = s.samples[-1]                # one-way, until the first exchange
            tau_now = tau + k * (local - (rx - s.delay()))
        return s.alpha * tau_now + s.o, None if eta is None else s.alpha * eta

    def fresh(self, local: float) -> List[str]:
        return [k for k, s in self.sources.items() if s.last is not None
                and local - s.last_rx <= (self.leader_stale_s if s.leader else self.stale_s)]

    def hears_leader(self, local: float) -> bool:
        return any(self.sources[k].leader for k in self.fresh(local))

    def step(self, local: float) -> None:
        """One consensus update (called before sending our beacon)."""
        fresh = [self.sources[k] for k in self.fresh(local)]
        if not fresh:
            return
        own_v = self.proto_time(local)
        if any(s.hw_hist for s in fresh):
            fresh = [s for s in fresh if s.hw_hist]           # two-way estimates only, once there are any
        est = [(s, *self._estimate(s, local)) for s in fresh]
        anchored = [e for e in est if e[0].anchor is not None]
        used = anchored or est                              # sources with a chain to the ship outrank the rest
        peers = [(v, r) for s, v, r in used if not s.leader]
        leader = [(v, r) for s, v, r in used if s.leader]

        target_v = self.aggregate(own_v, [v for v, _ in peers])
        target_a = self.aggregate(self.alpha, [r for _, r in peers if r is not None])
        if leader:
            lv, lr = leader[0]
            target_v = self.leader_share * lv + (1.0 - self.leader_share) * target_v
            if lr is not None:
                target_a = self.leader_share * lr + (1.0 - self.leader_share) * target_a

        if anchored and abs(target_v - own_v) > self.step_s:
            # Far off an anchored target: step onto the ship's estimate (or the anchored peers').
            new_v = leader[0][0] if leader else sum(v for v, _ in peers) / len(peers)
            self.steps += 1
        else:
            new_v = own_v + self.gain * (target_v - own_v)
        self.alpha += self.gain * (target_a - self.alpha)
        self.o = new_v - self.alpha * local                 # continuous at `local` (except a step)

        for s in fresh:
            self._take_anchor(s.anchor)
        self._resid = max(abs(v - new_v) for _, v, _ in used)


def make(mode: Optional[str] = None, env=None, node_id: str = ""):
    env = os.environ if env is None else env
    mode = (mode or env.get("CLOCK_SYNC") or "none").lower()
    if mode not in MODES:
        raise ValueError(f"CLOCK_SYNC={mode!r}: expected one of {MODES}")
    if mode == "consensus":
        return Consensus(node_id=node_id, gain=float(env.get("CLOCK_GAIN") or 0.5),
                         leader_share=float(env.get("CLOCK_LEADER_SHARE") or 0.5),
                         stale_s=3.0 / float(env.get("CLOCK_BEACON_HZ") or 0.5),
                         step_s=float(env.get("CLOCK_STEP_S") or 0.05),
                         aggregate=env.get("CLOCK_AGGREGATE") or "mean")
    return {"none": NoSync, "ttg": TimeToGo, "master": ShipMaster}[mode]()
