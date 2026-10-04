"""Spatial queue: keep drones out of other jobs' blasts, and pick intercept points far out.

No zenoh dependency, so it is unit-tested; drones and the ship share it.

A detonation destroys any drone within the kill radius (8 m).  Every confirmed job therefore
*reserves* a blast: a sphere of CLEARANCE_M around each of its slots (blast_radius) at its
detonation time, +- BLAST_TOL_S for timing error (and the fuze window).  Drones not on that job must be outside it
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
import os
from dataclasses import dataclass, field
from typing import Callable, Iterable, List, Optional, Sequence, Tuple

from threats import ETA_MARGIN, eta

Vec3 = Tuple[float, float, float]

CLEARANCE_M = 12.0         # non-job drones are kept this far from a detonation (kill radius 8 m + margin)
SLOT_SPACING_M = 4.0       # vertical spacing of job-mates stacked through the engagement point (owner's decision:
                           # at 3 m, with 0.5 m altitude latching, mates came 2.0-2.5 m apart, inside the 2.5 m
                           # proximity threshold); a three-drone stack spans +-4 m, a two-drone one +-2 m
def _blast_tol() -> float:
    """Detonation time uncertainty: 1.5 s for timed detonation; with the proximity fuze (FUZE, on by
    default) a drone may fire anywhere in its window, t_engage +- FUZE_WINDOW_S, so at least that."""
    fuze_on = (os.environ.get("FUZE") or "1").strip().lower() not in ("0", "off", "false", "no")
    return max(1.5, float(os.environ.get("FUZE_WINDOW_S") or 2.0)) if fuze_on else 1.5


BLAST_TOL_S = _blast_tol()  # a blast is dangerous for t_engage +- this
HOLD_MARGIN_M = 2.0        # holds stop this far before a blast sphere
INTERCEPT_SLACK_S = 4.0    # time kept free before an intercept (orders, bids, confirmation, errors)
SAMPLE_S = 0.1             # route sampling step inside a blast window


def slot_radius(level: int) -> float:
    """How far a job's outermost slot is from its engagement point."""
    return SLOT_SPACING_M * (level - 1) / 2.0 if level > 1 else 0.0


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


def _profile(d: float, v_max: float, a_max: float, v0: float):
    """Straight move over d starting at speed v0 along it, ending at rest, accelerating/braking at
    a_max and cruising at v_max.  Returns (T, t1, tc, v_peak, d1, brake_a): accelerate for t1 to
    v_peak (covering d1), cruise tc, brake to rest.  brake_a is set when the drone cannot even
    stop within d (it brakes harder than a_max and arrives at rest)."""
    v0 = min(max(v0, 0.0), v_max)
    if d <= 0.0:
        return 0.0, 0.0, 0.0, 0.0, 0.0, None
    if v0 > 0.0 and d < v0 * v0 / (2.0 * a_max):
        a = v0 * v0 / (2.0 * d)
        return v0 / a, 0.0, 0.0, v0, 0.0, a
    d_acc = (v_max * v_max - v0 * v0) / (2.0 * a_max)
    d_dec = v_max * v_max / (2.0 * a_max)
    if d_acc + d_dec <= d:
        v_peak, tc = v_max, (d - d_acc - d_dec) / v_max
    else:
        v_peak, tc = math.sqrt(a_max * d + 0.5 * v0 * v0), 0.0
    t1 = (v_peak - v0) / a_max
    d1 = (v_peak * v_peak - v0 * v0) / (2.0 * a_max)
    return t1 + tc + v_peak / a_max, t1, tc, v_peak, d1, None


def travel_time(d: float, v_max: float, a_max: float, v0: float = 0.0) -> float:
    """Time to fly d in a straight line from speed v0 (along it) to rest.  v0 = 0: threats.eta."""
    return _profile(d, v_max, a_max, v0)[0]


def stop_distance(v0: float, a_max: float) -> float:
    return max(0.0, v0) ** 2 / (2.0 * a_max)


def progress(d_total: float, v_max: float, a_max: float, tau: float, v0: float = 0.0) -> float:
    """Distance covered after tau seconds of that move."""
    if tau <= 0.0 or d_total <= 0.0:
        return 0.0
    T, t1, tc, v_peak, d1, brake_a = _profile(d_total, v_max, a_max, v0)
    if tau >= T:
        return d_total
    if brake_a is not None:
        return min(d_total, v_peak * tau - 0.5 * brake_a * tau * tau)
    v0 = min(max(v0, 0.0), v_max)
    if tau < t1:
        return v0 * tau + 0.5 * a_max * tau * tau
    if tau < t1 + tc:
        return d1 + v_peak * (tau - t1)
    rem = T - tau
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


def _first_conflict(pos, goal, t, blasts, v_max, a_max, t_goal, tol, v0=0.0):
    """First blast the drone would be inside during its window, flying pos -> goal from time t
    (at speed v0 along the way) and then waiting at the goal until t_goal.
    Returns (blast, segment conflict?) or None."""
    d = math.dist(pos, goal)
    u = _sub(goal, pos)
    u = (u[0] / d, u[1] / d, u[2] / d) if d > 1e-9 else (0.0, 0.0, 0.0)
    t_arr = t + travel_time(d, v_max, a_max, v0)
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
            if math.dist(_along(pos, u, progress(d, v_max, a_max, tau - t, v0)), b.center) < b.radius:
                return b, True
            tau += SAMPLE_S
    return None


def detour(start: Vec3, goal: Vec3, radius: float, step_deg: float = 30.0) -> List[Vec3]:
    """Waypoints around the ship's no-fly zone (a circle of `radius` at the origin, in the ground
    plane) when the straight path start -> goal would cross it: tangent, arc, tangent, the shorter
    way round.  Arc points are spaced at most step_deg apart, slightly outside the circle so the
    chords between them stay out of it; altitude is interpolated.  [] if the path clears the zone
    (or an end is inside it: nothing sensible to do)."""
    sx, sy, gx, gy = start[0], start[1], goal[0], goal[1]
    ds, dg = math.hypot(sx, sy), math.hypot(gx, gy)
    if ds < radius or dg < radius or _seg_dist((0.0, 0.0, 0.0), (sx, sy, 0.0), (gx, gy, 0.0)) >= radius:
        return []
    a_s, a_g = math.atan2(sy, sx), math.atan2(gy, gx)
    t_s, t_g = math.acos(radius / ds), math.acos(radius / dg)
    best = None
    for sgn in (1.0, -1.0):                       # counter-clockwise, clockwise
        a1, a2 = a_s + sgn * t_s, a_g - sgn * t_g
        sweep = ((a2 - a1) * sgn) % (2.0 * math.pi)
        length = math.sqrt(ds * ds - radius * radius) + radius * sweep + math.sqrt(dg * dg - radius * radius)
        if best is None or length < best[0]:
            best = (length, a1, sweep, sgn)
    _, a1, sweep, sgn = best
    n = max(1, int(math.ceil(math.degrees(sweep) / step_deg)))
    r_out = radius / math.cos(sweep / n / 2.0)      # chords between arc points stay outside the circle
    pts = []
    for k in range(n + 1):
        a = a1 + sgn * sweep * k / n
        f = (k + 1) / (n + 2)
        pts.append((r_out * math.cos(a), r_out * math.sin(a), start[2] + (goal[2] - start[2]) * f))
    return pts


def plan_route(start: Vec3, goal: Vec3, t0: float, blasts: Iterable[Blast], v_max: float, a_max: float,
               t_goal: Optional[float] = None, tol: float = BLAST_TOL_S, max_legs: int = 12,
               v0: float = 0.0, no_fly: Optional[float] = None) -> Route:
    """Route start -> goal from time t0 that stays out of every blast during its window, and around
    the ship's no-fly zone (radius `no_fly`, None: no zone) through detour() waypoints.  Each piece is
    planned by _plan_segment (holds included); time is conservative (each waypoint from rest)."""
    blasts = list(blasts)
    wps = detour(start, goal, no_fly) if no_fly else []
    if not wps:
        return _plan_segment(start, goal, t0, blasts, v_max, a_max, t_goal, tol, max_legs, v0)
    route = Route()
    pos, t, v = tuple(start), float(t0), v0
    for wp in wps + [tuple(goal)]:
        last = wp == tuple(goal)
        seg = _plan_segment(pos, wp, t, blasts, v_max, a_max, t_goal if last else None, tol, max_legs, v)
        route.legs += seg.legs
        route.travel_s += seg.travel_s
        route.hold_s += seg.hold_s
        route.blocked = route.blocked or seg.blocked
        route.exposed += [e for e in seg.exposed if e not in route.exposed]
        route.arrival = seg.arrival
        pos, t, v = wp, seg.arrival, 0.0
    return route


def _plan_segment(start: Vec3, goal: Vec3, t0: float, blasts: Iterable[Blast], v_max: float, a_max: float,
                  t_goal: Optional[float] = None, tol: float = BLAST_TOL_S, max_legs: int = 12,
                  v0: float = 0.0) -> Route:
    """Route start -> goal from time t0 that stays out of every blast during its window.

    t_goal: until when the drone will wait at the goal (its own detonation time); a goal inside
    a blast that goes off before then makes the route `blocked`.
    v0: current speed towards the goal (a moving drone gets there sooner, and needs its stopping
    distance to hold before a blast; a blast it cannot stop short of is `exposed`).
    """
    blasts = list(blasts)
    route = Route()
    pos, t = tuple(start), float(t0)
    for _ in range(max_legs):
        hit = _first_conflict(pos, goal, t, blasts, v_max, a_max, t_goal, tol, v0)
        d = math.dist(pos, goal)
        if hit is None:
            dt = travel_time(d, v_max, a_max, v0)
            route.legs.append(Leg(tuple(goal)))
            route.travel_s += dt
            route.arrival = t + dt
            return route
        b, on_segment = hit
        if on_segment and math.dist(pos, b.center) < b.radius:
            # Inside it now, and the straight path would still be inside when it goes off: leave
            # radially if that can be done before its window opens, otherwise it is unavoidable.
            w = _sub(pos, b.center)
            n = math.hypot(w[0], w[1])
            ux, uy = (w[0] / n, w[1] / n) if n > 1e-6 else (1.0, 0.0)
            r = b.radius + HOLD_MARGIN_M
            exit_pt = (b.center[0] + ux * r, b.center[1] + uy * r, pos[2])
            dt = eta(math.dist(pos, exit_pt), v_max, a_max)
            blasts = [o for o in blasts if o is not b]
            if t + dt > b.window(tol)[0]:
                route.exposed.append(b.threat_id)
                continue
            route.legs.append(Leg(exit_pt, b.window(tol)[1]))     # wait outside until it has gone off
            route.travel_s += dt
            route.hold_s += b.window(tol)[1] - (t + dt)
            pos, t, v0 = exit_pt, b.window(tol)[1], 0.0
            continue
        if not on_segment:
            route.blocked = True               # the goal itself is inside the blast
            route.legs.append(Leg(tuple(goal)))
            route.travel_s += travel_time(d, v_max, a_max, v0)
            route.arrival = t + travel_time(d, v_max, a_max, v0)
            return route
        u = _sub(goal, pos)
        u = (u[0] / d, u[1] / d, u[2] / d)
        s_in = _entry(pos, u, d, b.center, b.radius)
        s_hold = max(0.0, (s_in if s_in is not None else 0.0) - HOLD_MARGIN_M)
        if s_hold < stop_distance(v0, a_max):
            # Moving too fast to stop short of it: unavoidable, carry on (don't turn back into it).
            route.exposed.append(b.threat_id)
            blasts = [o for o in blasts if o is not b]
            continue
        release_at = b.window(tol)[1]
        # The hold point must also be clear of every other blast going off while we wait there.
        while True:
            hold_pt = _along(pos, u, s_hold)
            dt = travel_time(s_hold, v_max, a_max, v0)
            unsafe = [o for o in blasts if o.window(tol)[0] <= release_at and o.window(tol)[1] >= t + dt
                      and math.dist(hold_pt, o.center) < o.radius]
            if not unsafe:
                break
            if s_hold <= 0.0:
                route.blocked = True
                route.arrival = t + travel_time(d, v_max, a_max, v0)
                return route
            s_hold = max(0.0, s_hold - HOLD_MARGIN_M)
        release = max(t + dt, release_at)
        route.legs.append(Leg(hold_pt, release))
        route.travel_s += dt
        route.hold_s += release - (t + dt)
        pos, t, v0 = hold_pt, release, 0.0
    route.blocked = True                        # too many holds: give up
    route.arrival = t + travel_time(math.dist(pos, goal), v_max, a_max, v0)
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
             a_max: float, no_fly: Optional[float] = None) -> bool:
    """Would this blast force a hold on (or block) any committed drone (position, slot, own t)?"""
    for pos, goal, t_goal in committed:
        if _seg_dist(blast.center, pos, goal) >= blast.radius:
            continue                            # nowhere near its route or slot
        r = plan_route(pos, goal, now, [blast], v_max, a_max, t_goal=t_goal, no_fly=no_fly)
        if r.blocked or r.hold_s > 0 or r.exposed:
            return True
    return False


def choose_intercept(track: Callable[[float], Vec3], now: float, t_latest: float, drones: Sequence[Vec3],
                     level: int, reservations: Sequence[Reservation], v_max: float, a_max: float,
                     max_range: float, z_range: Tuple[float, float], slack: float = INTERCEPT_SLACK_S,
                     margin: float = ETA_MARGIN, step: float = 0.5, verify: int = 3,
                     committed: Sequence[Tuple[Vec3, Vec3, float]] = (), min_range: float = 0.0,
                     no_fly: Optional[float] = None) -> Optional[Intercept]:
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
        if (min_range <= math.hypot(p[0], p[1]) <= max_range and z_range[0] <= p[2] <= z_range[1]
                and all(math.dist(p, r.point) >= blast_separation(r.level, level) for r in reservations)):
            bound = sorted((eta(math.dist(d, p), v_max, a_max) * margin, i) for i, d in enumerate(drones))
            if now + slack + bound[level - 1][0] <= t:
                costs = sorted(
                    route.cost(margin) for route in (
                        plan_route(drones[i], p, now, blasts, v_max, a_max, t_goal=t, no_fly=no_fly)
                        for _, i in bound[:level + verify]) if not route.blocked)
                if (len(costs) >= level and now + slack + costs[level - 1] <= t
                        and not disrupts(Blast("new", p, blast_radius(level), t), committed, now, v_max, a_max,
                                         no_fly)):
                    return Intercept(t, p, costs[level - 1])
        t += step
    return None
