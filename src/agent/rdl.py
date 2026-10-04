"""Recursive decentralized localization (LOCALIZATION=rdl): consistent peer fusion with pairwise
exchanges (Luft, Schubert, Roumeliotis, Burgard 2018).  No zenoh dependency, so it is unit-tested.

coop.py uses a peer's estimate as if it were independent of ours, with a variance floor; that stops
gross overconfidence but not along chains (uwb_short: peer-only drones 16 m off, NEES 50).  RDL keeps
the cross-covariance between drones that have exchanged ranges, in factored form, so a range between
two drones updates both consistently and nobody needs the swarm's joint covariance:

- Each drone i keeps its own estimate (x_i, P_ii, the Localizer's) and, per peer j, a 6x6 factor
  s_ij; the cross-covariance is P_ij = s_ij s_ji^T (j holds s_ji).  Pairs that never exchanged are
  independent (their priors are): no factor, P_ij = 0.
- Own steps touch only own factors: every prediction (x <- F x) and anchor update (x <- (I - K H) x,
  anchors are known landmarks) left-multiplies all of i's factors (Localizer.listener).
- A range from i to j (delayed state): j's payload is its state at its own time tp <= t, with its
  factor s_ji(tp).  The cross-covariance of i's state now and j's at tp is exactly s_ij(t) s_ji(tp)^T,
  so i updates both on the joint 12-state covariance; the range model moves j by its velocity over
  t - tp (plus noise for its unknown acceleration).  i keeps s_ij = P_ij+ and sends j a reply: the
  innovation, its variance S and its covariance g with j's state as sent.  That innovation is
  independent of anything j measured since, so its covariance with j's error now is Phi g (Phi: the
  product of j's steps since that payload, kept per payload by sequence number), and j's update is
  an ordinary, exact one: x += Phi g y / S, P -= (Phi g)(Phi g)^T / S, s_ji = Phi (a reset).  The
  third parties' cross-covariances are scaled by P+ P^-1 on both sides (Luft's approximation; without
  it, or without applying it to the payload snapshots, the filter diverged in simulation).
- One update per snapshot: each payload names the one peer that may update it (`accept`, rotating);
  two updates from the same snapshot would each assume the other did not happen and count j's
  uncertainty twice (that made a drone ranged by four peers diverge).  Two drones accepting each
  other: the smaller id updates the pair.  i waits for j's acknowledgement before using it again.
- Lost replies: past reply_timeout without an acknowledgement, i drops its factor.  A pair that has
  exchanged before but whose factor is gone (lost reply, eviction, a re-lock), or whose factors no
  longer form a valid joint covariance, has an unknown correlation: its next update uses covariance
  intersection on the joint prior (a bound for any correlation), which re-establishes the factor.
- Limit: with a single anchored drone, the formation's rotation about it is unobservable from ranges;
  an EKF (RDL or coop) linearized at a wrong estimate then becomes overconfident (simulation: tens of
  metres off, NEES ~1000).  Two or more anchored drones not in line make it observable.
"""

import math
from collections import OrderedDict, deque
from typing import Dict, List, Optional, Sequence, Tuple

import numpy as np

N = 6
_TRI = np.triu_indices(N)


def _tri(P: np.ndarray) -> List[float]:
    return [float(v) for v in P[_TRI]]


def _untri(v: Sequence[float]) -> np.ndarray:
    P = np.zeros((N, N))
    P[_TRI] = v
    return P + np.triu(P, 1).T


def _sym(P: np.ndarray) -> np.ndarray:
    return 0.5 * (P + P.T)


class _Snapshot:
    __slots__ = ("seq", "t", "Phi")

    def __init__(self, seq: int, t: float):
        self.seq, self.t, self.Phi = seq, t, np.eye(N)


class RDL:
    def __init__(self, me: str, loc, max_peers: int = 6, max_factors: int = 8, accel_sigma: float = 3.0,
                 gate: float = 3.29, reply_timeout: float = 3.0, keep_replies_s: float = 1.5,
                 snapshots: int = 12, stale_s: float = 5.0, imu: bool = False):
        self.me = me
        self.loc = loc
        loc.listener = self
        self.max_peers, self.max_factors = max_peers, max_factors
        # m/s^2: the peer's unknown acceleration over the payload's age (noise accel_sigma tau^2 / 2 on the
        # range).  Calibrated in simulation: 1 left RDL overconfident with IMU prediction (worst-drone NEES
        # 8-22) and 3 made it consistent (1.4-2.7) with smaller errors; it also absorbs what Luft's
        # third-party approximation and the asynchronous exchange leave out.
        self.accel_sigma = accel_sigma
        self.gate2 = gate * gate
        self.reply_timeout, self.keep_replies_s, self.stale_s = reply_timeout, keep_replies_s, stale_s
        self.imu = imu                       # our states 4-5: accelerometer bias (True) or wind (False)
        self.factors: "OrderedDict[str, np.ndarray]" = OrderedDict()   # peer -> s_me,peer (LRU order)
        self.met = set()                     # peers we exchanged with (no factor then = unknown)
        self.table: Dict[str, dict] = {}     # peer -> its latest payload
        self.awaiting: Dict[str, Tuple[int, float]] = {}     # peer -> (our reply id, time sent)
        self.replies_out: Dict[str, Tuple[dict, float]] = {}  # peer -> (reply, time made)
        self.acks: Dict[str, int] = {}       # peer -> id of its last reply we applied
        self.snaps = deque(maxlen=snapshots)
        self.seq = 0
        self._reply_n = 0
        self._rr = 0
        self._acc = -1                       # rotation over peers for `accept`
        self._accept = None                  # the peer our last payload accepted
        self.joint_updates = self.ci_updates = self.replies_applied = self.replies_scaled = 0
        self.replies_lost = self.rejected = self.skipped = 0

    # --- Localizer hooks ------------------------------------------------
    def on_transform(self, M: np.ndarray) -> None:
        for k in self.factors:
            self.factors[k] = M @ self.factors[k]
        for s in self.snaps:
            s.Phi = M @ s.Phi

    def on_reset(self) -> None:
        """Our estimate was re-initialized: every correlation with peers is lost."""
        self.factors.clear()
        self.awaiting.clear()
        self.snaps.clear()

    # --- what our UWB frames carry ---------------------------------------
    def payload(self, t: float, z: float, ranging: Sequence[str] = ()) -> dict:
        """Our state for peers (and a snapshot to map their replies onto later).  `accept` names the one
        peer allowed to update this snapshot (rotating): two updates computed from the same snapshot
        would each assume the other did not happen, and count our uncertainty twice."""
        self.seq += 1
        peers = sorted(k for k in self.table if k != self.me)
        accept = None
        if peers:
            self._acc = (self._acc + 1) % len(peers)
            accept = peers[self._acc]
        self._accept = accept
        self.snaps.append(_Snapshot(self.seq, t))
        loc = self.loc
        return {"seq": self.seq, "t": round(t, 4), "z": round(z, 3), "imu": self.imu,
                "x": [float(v) for v in loc.x], "P": _tri(loc.P),
                "f": {k: [float(v) for v in f.ravel()] for k, f in self.factors.items()},
                "ranging": list(ranging), "acks": dict(self.acks), "accept": accept}

    def replies(self, t: float) -> Dict[str, dict]:
        """Replies to send (each repeated for keep_replies_s, the radio is lossy; peers dedupe)."""
        for k in [k for k, (_, t0) in self.replies_out.items() if t - t0 > self.keep_replies_s]:
            del self.replies_out[k]
        return {k: r for k, (r, _) in self.replies_out.items()}

    def on_payload(self, peer: str, s: dict, t: Optional[float] = None) -> None:
        self.table[peer] = s
        w = self.awaiting.get(peer)
        if w is not None:
            if s.get("acks", {}).get(self.me) == w[0]:
                del self.awaiting[peer]                 # the peer applied our reply: its factor for us is reset
            elif t is not None and t - w[1] > self.reply_timeout:
                del self.awaiting[peer]                 # lost: the pair's correlation is unknown now
                self.factors.pop(peer, None)
                self.replies_lost += 1

    def choose_peers(self, own_xy: Tuple[float, float], t: float, roster: Sequence[str]) -> List[str]:
        """The nearest known peers, plus one rotating roster member to discover."""
        for k in [k for k, s in self.table.items() if t - s["t"] > self.stale_s]:
            del self.table[k]
        known = sorted((math.dist(own_xy, (s["x"][0], s["x"][1])), k) for k, s in self.table.items())
        chosen = [k for _, k in known[: max(0, self.max_peers - 1)]]
        unknown = [r for r in roster if r != self.me and r not in chosen]
        if unknown:
            self._rr = (self._rr + 1) % len(unknown)
            chosen.append(unknown[self._rr])
        return chosen[: self.max_peers]

    # --- a range to a peer -------------------------------------------------
    def peer_update(self, peer: str, r: float, z: float, t: float, sigma: float) -> bool:
        """Use a range r measured at t to `peer` (its payload already in the table).  z: our altitude;
        sigma: the ranging noise.  Returns True if the pair was updated.

        Delayed state: the payload is the peer's state at its own time tp <= t.  The update is on our
        state now and the peer's state at tp, whose cross-covariance is exactly s_ij(t) s_ji(tp)^T
        (each factor carries its owner's steps); the range model moves the peer by its velocity over
        tau = t - tp, with accel_sigma tau^2 / 2 of extra noise for what it did meanwhile.  So the
        correction we send is relative to exactly the state the peer sent, and it maps it to now with
        its own steps since (on_reply)."""
        loc, s = self.loc, self.table.get(peer)
        if s is None or loc.x is None:
            return False
        tau = t - float(s["t"])
        if tau < -0.05 or tau > self.stale_s:
            return False
        tau = max(0.0, tau)
        if s.get("accept") not in (None, self.me):
            self.skipped += 1                           # another peer updates this snapshot
            return False
        if self._accept == peer and peer < self.me:
            self.skipped += 1                           # we accepted each other: the smaller id updates the pair
            return False
        if peer in self.awaiting:
            self.skipped += 1                           # its factor for us is not reset yet
            return False
        xp, Pp = np.asarray(s["x"], float), _untri(s["P"])
        mine, theirs = self.factors.get(peer), s.get("f", {}).get(self.me)
        if mine is not None and theirs is not None:
            C = mine @ np.asarray(theirs, float).reshape(N, N).T
            unknown = False
        elif peer not in self.met:
            C = np.zeros((N, N))                        # never exchanged: independent priors
            unknown = False
        else:
            C, unknown = None, True                     # exchanged, factor lost: correlation unknown
        xm, Pm = loc.x.copy(), loc.P.copy()
        cmd_model = not s.get("imu")                    # command model: ground velocity = v + wind
        vpx = xp[2] + (xp[4] if cmd_model else 0.0)
        vpy = xp[3] + (xp[5] if cmd_model else 0.0)
        dx, dy, dz = xm[0] - (xp[0] + vpx * tau), xm[1] - (xp[1] + vpy * tau), z - float(s["z"])
        h = math.sqrt(dx * dx + dy * dy + dz * dz)
        if h < 1e-6:
            return False
        ux, uy = dx / h, dy / h
        H = np.zeros((1, 2 * N))
        H[0, 0], H[0, 1] = ux, uy
        H[0, N], H[0, N + 1] = -ux, -uy
        H[0, N + 2], H[0, N + 3] = -ux * tau, -uy * tau
        if cmd_model:
            H[0, N + 4], H[0, N + 5] = -ux * tau, -uy * tau
        sigma = math.sqrt(sigma * sigma + (0.5 * self.accel_sigma * tau * tau) ** 2)
        y = r - h
        if not unknown:
            J = np.zeros((2 * N, 2 * N))
            J[:N, :N], J[N:, N:], J[:N, N:], J[N:, :N] = Pm, Pp, C, C.T
            if np.linalg.eigvalsh(_sym(J)).min() <= 0.0:
                unknown = True                          # the factors no longer describe a valid joint
        if unknown:
            # Covariance intersection: blockdiag(Pm / w, Pp / (1 - w)) bounds the joint covariance for any
            # correlation; w chosen to keep our position uncertainty smallest.
            best = None
            for w in (0.2, 0.35, 0.5, 0.65, 0.8):
                J = np.zeros((2 * N, 2 * N))
                J[:N, :N], J[N:, N:] = Pm / w, Pp / (1.0 - w)
                post = self._joint(J, H, y, sigma)
                if post is not None and (best is None or np.trace(post[1][:2, :2]) < np.trace(best[1][:2, :2])):
                    best = post
            post = best
        else:
            post = self._joint(J, H, y, sigma)
        if post is None:
            self.rejected += 1
            return False
        dX, Pj, g, S = post
        Pm_new, C_new = _sym(Pj[:N, :N]), Pj[:N, N:]
        M = Pm_new @ np.linalg.inv(Pm)                  # third parties: s_ik <- P_ii+ P_ii^-1 s_ik
        for k in self.factors:
            if k != peer:
                self.factors[k] = M @ self.factors[k]
        for sn in self.snaps:
            sn.Phi = M @ sn.Phi
        loc.x = xm + dX[:N]
        loc.P = Pm_new
        self._set_factor(peer, C_new)
        self.met.add(peer)
        self._reply_n += 1
        # The reply: the innovation, its variance, and its covariance g with the peer's state as sent.
        self.replies_out[peer] = ({"id": self._reply_n, "seq": int(s["seq"]), "y": float(y), "S": float(S),
                                   "g": [float(v) for v in g]}, t)
        self.awaiting[peer] = (self._reply_n, t)
        if unknown:
            self.ci_updates += 1
        else:
            self.joint_updates += 1
        return True

    def _joint(self, J: np.ndarray, H: np.ndarray, y: float, sigma: float):
        S = float((H @ J @ H.T)[0, 0]) + sigma * sigma
        if y * y > self.gate2 * S:
            return None
        JH = J @ H.T
        K = JH / S
        IKH = np.eye(2 * N) - K @ H
        return K[:, 0] * y, IKH @ J @ IKH.T + (K @ K.T) * sigma * sigma, JH[N:, 0], S

    def _set_factor(self, peer: str, f: np.ndarray) -> None:
        self.factors[peer] = f
        self.factors.move_to_end(peer)
        while len(self.factors) > self.max_factors:
            old, _ = self.factors.popitem(last=False)   # evicted: correlation with it unknown from now
            self.awaiting.pop(old, None)

    # --- a peer's reply to our payload -------------------------------------
    def on_reply(self, sender: str, rep: dict) -> bool:
        """Apply the correction a peer computed for us in a joint update (once per reply)."""
        rid = int(rep["id"])
        if self.acks.get(sender, -1) >= rid or self.loc.x is None:
            return False
        snap = next((sn for sn in self.snaps if sn.seq == int(rep["seq"])), None)
        if snap is None:
            return False                                # too old: the sender times out and drops the pair
        # The innovation is independent of everything we measured since the snapshot, so its covariance
        # with our error now is Phi g: an ordinary update of our current state, exact for any steps since.
        Phi = snap.Phi
        g = Phi @ np.asarray(rep["g"], float)
        S, y = float(rep["S"]), float(rep["y"])
        P = self.loc.P
        dx = g * (y / S)
        dP = np.outer(g, g) / S
        a = 1.0
        while a > 1e-3 and np.linalg.eigvalsh(_sym(P - a * dP)).min() <= 1e-9:
            a *= 0.5                                    # the factors no longer fit: take less
        if a < 1.0:
            self.replies_scaled += 1
        P_new = _sym(P - a * dP)
        M = P_new @ np.linalg.inv(P)
        for k in self.factors:
            if k != sender:
                self.factors[k] = M @ self.factors[k]
        for sn in self.snaps:
            sn.Phi = M @ sn.Phi
        self.loc.x = self.loc.x + a * dx
        self.loc.P = P_new
        self._set_factor(sender, Phi.copy())            # reset: identity at the update, carried by our steps since
        self.awaiting.pop(sender, None)                 # a reply of ours to it is superseded
        self.met.add(sender)
        self.acks[sender] = rid
        self.replies_applied += 1
        return True
