"""Spatial queue: keep drones out of other jobs' blasts, and pick intercept points far out.

No zenoh dependency, so it is unit-tested; drones and the ship share it.

A detonation destroys any drone within the kill radius (8 m).  Every confirmed job therefore
*reserves* a blast: a sphere of CLEARANCE_M around each of its slots (blast_radius) at its
detonation time, +- BLAST_TOL_S for timing error.  Drones not on that job must be outside it
during that window:

- plan_route(): a straight route to the goal with the trapezoidal speed profile.  If the drone
  would be inside a blast during its window, it holds just before the sphere until the window has
  passed, then continues (several holds are possible).  A drone starting inside a sphere whose
  window is still ahead first leaves it radially (`exposed` if it cannot get out in time).  A goal inside another job's blast, occupied
  while that blast goes off, cannot be made safe: the route is `blocked`.
- choose_intercept(): the ship's intercept point is the *earliest* point on the threat's track
  (the farthest from the ship) that `level` drones can reach in time with margin, whose blast is
  at least blast_separation() from every other job's slots (so a job's drones waiting at their
  slots are never inside another job's blast), within range.  None if no such point exists.
"""

import math
from dataclasses import dataclass, field
from typing import Callable, Iterable, List, Optional, Sequence, Tuple

from threats import ETA_MARGIN, eta

Vec3 = Tuple[float, float, float]

CLEARANCE_M = 12.0         # non-job drones are kept this far from a detonation (kill radius 8 m + margin)
SLOT_RADIUS_M = 4.0        # drones' slot circle around the engagement point (multi-drone jobs)
BLAST_TOL_S = 1.5          # detonation time uncertainty: a blast is dangerous for t_engage +- this
HOLD_MARGIN_M = 2.0        # holds stop this far before a blast sphere
INTERCEPT_SLACK_S = 4.0    # time kept free before an intercept (orders, bids, confirmation, errors)
SAMPLE_S = 0.1             # route sampling step inside a blast window


def slot_radius(level: int) -> float:
    return SLOT_RADIUS_M if level > 1 else 0.0


def blast_radius(level: int) -> float:
    """Keep-out radius around a job's engagement point: CLEARANCE_M from every slot."""
    return CLEARANCE_M + slot_radius(level)


def blast_separation(level_a: int, level_b: int) -> float:
    """Minimum distance between two concurrent jobs' engagement points.

    Drones of one job wait at their slots until their own detonation, so the other job's blast
    must stay CLEARANCE_M away from those slots too.
    """
    return CLEARANCE_M + slot_radius(level_a) + slot_radius(level_b)


@dataclass(frozen=True)
class Blast:
    threat_id: str
    center: Vec3
    radius: float
    t: float               # detonation time

    def window(self, tol: float = BLAST_TOL_S) -> Tuple[float, float]:
        return self.t - tol, self.t + tol


@dataclass
class Leg:
    target: Vec3
    release: Optional[float] = None    # hold at target until this time (None: final leg or exit leg)


@dataclass
class Route:
    legs: List[Leg] = field(default_factory=list)
    arrival: float = 0.0               # time at the goal
    travel_s: float = 0.0              # flying time (holds excluded)
    hold_s: float = 0.0                # time spent holding
    blocked: bool = False              # goal is inside another job's blast while it goes off
    exposed: List[str] = field(default_factory=list)   # blasts the drone cannot get clear of in time

    def cost(self, margin: float = ETA_MARGIN) -> float:
        """Bid cost / time needed: flying time with margin, plus holds."""
        return self.travel_s * margin + self.hold_s


def progress(d_total: float, v_max: float, a_max: float, tau: float) -> float:
    """Distance covered after tau seconds of a rest-to-rest trapezoidal move over d_total."""
    if tau <= 0.0 or d_total <= 0.0:
        return 0.0
    t_total = eta(d_total, v_max, a_max)
    if tau >= t_total:
        return d_total
    t_acc = v_max / a_max
    if d_total >= v_max * t_acc:                       # reaches cruise speed
        d_acc = 0.5 * a_max * t_acc * t_acc
        if tau < t_acc:
            return 0.5 * a_max * tau * tau
        if tau < t_total - t_acc:
            return d_acc + v_max * (tau - t_acc)
    else:                                              # triangular profile
        t_acc = t_total / 2.0
        if tau < t_acc:
            return 0.5 * a_max * tau * tau
    rem = t_total - tau
    return d_total - 0.5 * a_max * rem * rem


def _sub(a, b):
    return (a[0] - b[0], a[1] - b[1], a[2] - b[2])


def _along(p, u, s):
    return (p[0] + u[0] * s, p[1] + u[1] * s, p[2] + u[2] * s)


def _entry(p: Vec3, u: Vec3, d: float, c: Vec3, r: float) -> Optional[float]:
    """Distance along p + u*s (0 <= s <= d) where the segment first enters sphere (c, r)."""
    w = _sub(p, c)
    b = u[0] * w[0] + u[1] * w[1] + u[2] * w[2]
    q = w[0] * w[0] + w[1] * w[1] + w[2] * w[2] - r * r
    disc = b * b - q
    if disc < 0.0:
        return None
    s = -b - math.sqrt(disc)
    return s if 0.0 <= s <= d else None


def _first_conflict(pos, goal, t, blasts, v_max, a_max, t_goal, tol):
    """First blast the drone would be inside during its window, flying pos -> goal from time t
    and then waiting at the goal until t_goal.  Returns (blast, segment conflict?) or None."""
    d = math.dist(pos, goal)
    u = _sub(goal, pos)
    u = (u[0] / d, u[1] / d, u[2] / d) if d > 1e-9 else (0.0, 0.0, 0.0)
    t_arr = t + eta(d, v_max, a_max)
    for b in sorted(blasts, key=lambda b: b.t):
        lo, hi = b.window(tol)
        if hi < t:
            continue
        # waiting at the goal until our own detonation
        if (t_goal is None or lo <= t_goal) and hi >= t_arr and math.dist(goal, b.center) < b.radius:
            return b, False
        # flying the segment
        tau = max(lo, t)
        while tau <= min(hi, t_arr):
            if math.dist(_along(pos, u, progress(d, v_max, a_max, tau - t)), b.center) < b.radius:
                return b, True
            tau += SAMPLE_S
    return None


def plan_route(start: Vec3, goal: Vec3, t0: float, blasts: Iterable[Blast], v_max: float, a_max: float,
               t_goal: Optional[float] = None, tol: float = BLAST_TOL_S, max_legs: int = 12) -> Route:
    """Route start -> goal from time t0 that stays out of every blast during its window.

    t_goal: until when the drone will wait at the goal (its own detonation time); a goal inside
    a blast that goes off before then makes the route `blocked`.
    """
    blasts = list(blasts)
    route = Route()
    pos, t = tuple(start), float(t0)
    # Already inside a blast whose window is still ahead: leave it radially first.
    for b in sorted(blasts, key=lambda b: b.t):
        if b.window(tol)[1] >= t and math.dist(pos, b.center) < b.radius:
            w = _sub(pos, b.center)
            n = math.hypot(w[0], w[1])
            ux, uy = (w[0] / n, w[1] / n) if n > 1e-6 else (1.0, 0.0)
            r = b.radius + HOLD_MARGIN_M
            exit_pt = (b.center[0] + ux * r, b.center[1] + uy * r, pos[2])
            dt = eta(math.dist(pos, exit_pt), v_max, a_max)
            if t + dt > b.window(tol)[0]:
                route.exposed.append(b.threat_id)   # too close to get out before it goes off
            route.legs.append(Leg(exit_pt))
            route.travel_s += dt
            pos, t = exit_pt, t + dt
    for _ in range(max_legs):
        hit = _first_conflict(pos, goal, t, blasts, v_max, a_max, t_goal, tol)
        d = math.dist(pos, goal)
        if hit is None:
            dt = eta(d, v_max, a_max)
            route.legs.append(Leg(tuple(goal)))
            route.travel_s += dt
            route.arrival = t + dt
            return route
        b, on_segment = hit
        if not on_segment:
            route.blocked = True               # the goal itself is inside the blast
            route.legs.append(Leg(tuple(goal)))
            route.travel_s += eta(d, v_max, a_max)
            route.arrival = t + eta(d, v_max, a_max)
            return route
        u = _sub(goal, pos)
        u = (u[0] / d, u[1] / d, u[2] / d)
        s_in = _entry(pos, u, d, b.center, b.radius)
        s_hold = max(0.0, (s_in if s_in is not None else 0.0) - HOLD_MARGIN_M)
        release_at = b.window(tol)[1]
        # The hold point must also be clear of every other blast going off while we wait there.
        while True:
            hold_pt = _along(pos, u, s_hold)
            dt = eta(s_hold, v_max, a_max)
            unsafe = [o for o in blasts if o.window(tol)[0] <= release_at and o.window(tol)[1] >= t + dt
                      and math.dist(hold_pt, o.center) < o.radius]
            if not unsafe:
                break
            if s_hold <= 0.0:
                route.blocked = True
                route.arrival = t + eta(d, v_max, a_max)
                return route
            s_hold = max(0.0, s_hold - HOLD_MARGIN_M)
        release = max(t + dt, release_at)
        route.legs.append(Leg(hold_pt, release))
        route.travel_s += dt
        route.hold_s += release - (t + dt)
        pos, t = hold_pt, release
    route.blocked = True                        # too many holds: give up
    route.arrival = t + eta(math.dist(pos, goal), v_max, a_max)
    return route


@dataclass(frozen=True)
class Reservation:
    """A confirmed job's engagement, for separating new intercept points from it."""
    threat_id: str
    point: Vec3
    level: int
    t: float

    def blast(self) -> Blast:
        return Blast(self.threat_id, self.point, blast_radius(self.level), self.t)


@dataclass(frozen=True)
class Intercept:
    t: float
    point: Vec3
    tti_s: float           # time needed by the level-th drone (with margin and holds)


def _seg_dist(p: Vec3, a: Vec3, b: Vec3) -> float:
    ab = _sub(b, a)
    L2 = ab[0] * ab[0] + ab[1] * ab[1] + ab[2] * ab[2]
    if L2 < 1e-12:
        return math.dist(p, a)
    ap = _sub(p, a)
    s = max(0.0, min(1.0, (ap[0] * ab[0] + ap[1] * ab[1] + ap[2] * ab[2]) / L2))
    return math.dist(p, _along(a, ab, s))


def disrupts(blast: Blast, committed: Sequence[Tuple[Vec3, Vec3, float]], now: float, v_max: float,
             a_max: float) -> bool:
    """Would this blast force a hold on (or block) any committed drone (position, slot, own t)?"""
    for pos, goal, t_goal in committed:
        if _seg_dist(blast.center, pos, goal) >= blast.radius:
            continue                            # nowhere near its route or slot
        r = plan_route(pos, goal, now, [blast], v_max, a_max, t_goal=t_goal)
        if r.blocked or r.hold_s > 0 or r.exposed:
            return True
    return False


def choose_intercept(track: Callable[[float], Vec3], now: float, t_latest: float, drones: Sequence[Vec3],
                     level: int, reservations: Sequence[Reservation], v_max: float, a_max: float,
                     max_range: float, z_range: Tuple[float, float], slack: float = INTERCEPT_SLACK_S,
                     margin: float = ETA_MARGIN, step: float = 0.5, verify: int = 3,
                     committed: Sequence[Tuple[Vec3, Vec3, float]] = ()) -> Optional[Intercept]:
    """Earliest time t in [now + slack, t_latest] at which `level` drones can reach track(t) in time.

    A cheap straight-line bound ranks drones; only the best level + `verify` are checked with
    full routes (holds around the reservations' blasts).  New jobs yield to committed ones: a point
    whose blast would force a hold on a drone already flying another job (`committed`: its
    position, slot and detonation time) is skipped.
    """
    if level < 1 or len(drones) < level:
        return None
    blasts = [r.blast() for r in reservations]
    t = now + slack
    while t <= t_latest + 1e-9:
        p = track(t)
        if (math.hypot(p[0], p[1]) <= max_range and z_range[0] <= p[2] <= z_range[1]
                and all(math.dist(p, r.point) >= blast_separation(r.level, level) for r in reservations)):
            bound = sorted((eta(math.dist(d, p), v_max, a_max) * margin, i) for i, d in enumerate(drones))
            if now + slack + bound[level - 1][0] <= t:
                costs = sorted(
                    route.cost(margin) for route in (
                        plan_route(drones[i], p, now, blasts, v_max, a_max, t_goal=t)
                        for _, i in bound[:level + verify]) if not route.blocked)
                if (len(costs) >= level and now + slack + costs[level - 1] <= t
                        and not disrupts(Blast("new", p, blast_radius(level), t), committed, now, v_max, a_max)):
                    return Intercept(t, p, costs[level - 1])
        t += step
    return None
