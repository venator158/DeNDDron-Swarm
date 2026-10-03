"""Proximity fuze: when an engaged drone detonates, from unlabelled 3D contacts.

No zenoh dependency, so it is unit-tested.  The simulator's fuze sensor (drone/{id}/fuze) reports
the positions, relative to the drone, of every object within range (threats, other drones, the
ship's hull), without saying which is which, with noise.  The fuze:

1. Tracks the contacts from scan to scan (nearest-neighbour association in world coordinates).
2. Arms during t_engage +- window_s, in the drone's synchronized (protocol) time.
3. Records every object already in range as known: the spatial queue keeps non-job drones more
   than 12 m away, so these are job-mates (or the ship).  The record is taken at the first scan at
   which the job's track puts the threat within range + record_margin_m of *us* (or at arming if
   that is earlier): at the scaled threat speeds (2.5-4.5 m/s) a threat is already 5-9 m away,
   inside the 10 m range, at t_engage - 2 s, and would be taken for a job-mate.  Timing it on the
   threat's distance to the engagement point failed after a manoeuvre: the drone had stopped 3 m
   short of the new point on the threat's side, so the threat was already in range when recorded.
4. Fires on a *new* track, one that appeared after arming, while it lies within gate_m of the
   threat position predicted from the job's track, either at the track's closest approach within
   the kill radius (fire="cpa", the default) or as soon as it is within the kill radius
   (fire="radius").  The closest approach is when the track starts moving away from us, its velocity
   fitted to its positions over the last cpa_fit_s: range itself grows only quadratically there,
   so a range threshold fired 0.2-0.3 s late (0.8 m further on at 2.5 m/s).
5. If the window closes with no detection: fallback "hold" (do not detonate; the drone survives)
   or "timed" (detonate then).

Closing speed (Doppler) is not used: at the simulation's scaled speeds, threats and drones overlap.

Configuration (env): FUZE (1 = on; 0/off = timed detonation at t_engage, as before),
FUZE_WINDOW_S (2), FUZE_GATE_M (5), FUZE_FIRE (cpa|radius), FUZE_FALLBACK (hold|timed),
FUZE_RANGE_M (10, the sensor's range, shared with the simulator).
"""

import math
import os
from dataclasses import dataclass
from typing import Callable, List, Mapping, Optional, Sequence, Tuple

Vec3 = Tuple[float, float, float]


def enabled(env: Mapping[str, str] = None) -> bool:
    env = os.environ if env is None else env
    return (env.get("FUZE") or "1").strip().lower() not in ("0", "off", "false", "no")


@dataclass(frozen=True)
class FuzeConfig:
    window_s: float = 2.0         # armed during t_engage +- this (protocol time)
    gate_m: float = 5.0           # a trigger must be this close to the predicted threat position
    kill_radius_m: float = 8.0
    fire: str = "cpa"             # cpa | radius
    fallback: str = "hold"        # hold | timed, when the window closes with no detection
    max_speed: float = 10.0       # m/s: fastest object associated between scans
    assoc_m: float = 1.0          # association slack beyond max_speed * dt (noise)
    cpa_fit_s: float = 0.3        # track velocity fitted over this long (closest-approach timing)
    track_ttl_s: float = 0.5      # a track unseen this long is dropped
    range_m: float = 10.0         # sensor range
    record_margin_m: float = 2.0  # mates are recorded while the threat is predicted this far beyond range

    @staticmethod
    def from_env(env: Mapping[str, str] = None, kill_radius_m: float = 8.0) -> "FuzeConfig":
        env = os.environ if env is None else env
        cfg = FuzeConfig(window_s=float(env.get("FUZE_WINDOW_S") or 2.0),
                         gate_m=float(env.get("FUZE_GATE_M") or 5.0),
                         kill_radius_m=kill_radius_m,
                         fire=(env.get("FUZE_FIRE") or "cpa").lower(),
                         fallback=(env.get("FUZE_FALLBACK") or "hold").lower(),
                         range_m=float(env.get("FUZE_RANGE_M") or 10.0))
        if cfg.fire not in ("cpa", "radius"):
            raise ValueError(f"FUZE_FIRE={cfg.fire!r}: expected cpa or radius")
        if cfg.fallback not in ("hold", "timed"):
            raise ValueError(f"FUZE_FALLBACK={cfg.fallback!r}: expected hold or timed")
        return cfg


@dataclass
class _Track:
    tid: int
    pos: Vec3                      # world position at the last scan
    t: float                       # protocol time of the last scan
    known: bool = False            # in range when mates were recorded (job-mate, ship)
    range_m: float = math.inf      # distance to us at the last scan
    min_range: float = math.inf
    gated: bool = False            # within the gate of the predicted threat position at the last scan
    hist: list = None              # recent (t, world position), for the velocity fit


@dataclass(frozen=True)
class Decision:
    action: str                    # "fire" | "hold"
    reason: str                    # "fuze" | "fallback_timed" | "fallback_hold"
    trigger: Optional[Vec3] = None  # the trigger's position relative to us (evaluation)
    range_m: Optional[float] = None


class Fuze:
    def __init__(self, cfg: FuzeConfig = FuzeConfig()):
        self.cfg = cfg
        self.tracks: List[_Track] = []
        self.recorded = False
        self.armed = False
        self.done = False
        self._next_id = 0

    def record_now(self, t: float, own: Vec3, t_engage: float, predict) -> bool:
        """Record the objects in range as job-mates now?  Once the threat is predicted within range +
        margin of us (before it can be in range), and at the latest at arming."""
        if t >= t_engage - self.cfg.window_s:
            return True
        p = predict(t) if predict is not None else None
        return p is not None and math.dist(p, own) <= self.cfg.range_m + self.cfg.record_margin_m

    def in_window(self, t: float, t_engage: float) -> bool:
        return t_engage - self.cfg.window_s <= t <= t_engage + self.cfg.window_s

    def chain_ready(self, t: float, t_engage: float, dist_to_slot: float, detonate_radius: float) -> bool:
        """Chain fire: a job-mate's detonation fires us too if we are armed and on station."""
        return not self.done and self.in_window(t, t_engage) and dist_to_slot <= detonate_radius

    def _associate(self, t: float, own: Vec3, contacts: Sequence[Vec3]) -> None:
        self.tracks = [tr for tr in self.tracks if t - tr.t <= self.cfg.track_ttl_s]
        free = list(self.tracks)
        for rel in contacts:
            world = (own[0] + rel[0], own[1] + rel[1], own[2] + rel[2])
            best, best_d = None, math.inf
            for tr in free:
                d = math.dist(tr.pos, world)
                if d <= self.cfg.assoc_m + self.cfg.max_speed * max(0.0, t - tr.t) and d < best_d:
                    best, best_d = tr, d
            if best is None:
                best = _Track(self._next_id, world, t, hist=[])
                self._next_id += 1
                self.tracks.append(best)
            else:
                free.remove(best)
                best.pos, best.t = world, t
            best.hist = [h for h in best.hist if t - h[0] <= self.cfg.cpa_fit_s] + [(t, world)]
            best.range_m = math.dist(world, own)
            best.min_range = min(best.min_range, best.range_m)

    @staticmethod
    def _velocity(hist):
        """Least-squares velocity of a track from its recent positions (None with too few)."""
        if len(hist) < 4:
            return None
        n = len(hist)
        mt = sum(h[0] for h in hist) / n
        stt = sum((h[0] - mt) ** 2 for h in hist)
        if stt <= 0:
            return None
        return tuple(sum((h[0] - mt) * h[1][k] for h in hist) / stt for k in range(3))

    def update(self, t: float, own: Vec3, contacts: Sequence[Vec3], t_engage: float,
               predict: Optional[Callable[[float], Optional[Vec3]]], gate_extra: float = 0.0) -> Optional[Decision]:
        """One fuze scan.  t: its protocol time; own: our world position; contacts: relative
        positions; predict(t): the threat's predicted world position (None: no track, no trigger).
        gate_extra: widens the gate by our own position uncertainty (3 sigma), which shifts every
        contact's world position by the same error."""
        if self.done:
            return None
        self._associate(t, own, contacts)
        if not self.recorded and self.record_now(t, own, t_engage, predict):
            self.recorded = True
            for tr in self.tracks:
                tr.known = True
        if not self.armed and t >= t_engage - self.cfg.window_s:
            self.armed = self.recorded = True
        if not self.armed:
            return None
        if t > t_engage + self.cfg.window_s:
            self.done = True
            fb = self.cfg.fallback
            return Decision("fire" if fb == "timed" else "hold", f"fallback_{fb}")
        p = predict(t) if predict is not None else None
        for tr in self.tracks:
            if tr.known or tr.t != t:
                continue
            tr.gated = p is not None and math.dist(tr.pos, p) <= self.cfg.gate_m + gate_extra
            if not tr.gated:
                continue
            kill = self.cfg.kill_radius_m
            if self.cfg.fire == "radius":
                fire = tr.range_m <= kill
            else:
                v = self._velocity(tr.hist)
                rel = (tr.pos[0] - own[0], tr.pos[1] - own[1], tr.pos[2] - own[2])
                receding = v is not None and rel[0] * v[0] + rel[1] * v[1] + rel[2] * v[2] >= 0.0
                fire = tr.range_m <= kill and receding
            if fire:
                self.done = True
                rel = (tr.pos[0] - own[0], tr.pos[1] - own[1], tr.pos[2] - own[2])
                return Decision("fire", "fuze", rel, tr.range_m)
        return None
