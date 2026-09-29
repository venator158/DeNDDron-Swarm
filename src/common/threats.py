"""Threat model and engagement geometry shared by the ship and the drones.

A threat's *level* is both its priority and the number of drones needed to
kill it.  The ship's radar reports each threat as a straight-line track and
derives the closest point of approach (CPA) to the ship at the origin and the
time of it (t_cpa; TCPA = t_cpa - now).

Drones intercept at an *engagement point*: the CPA if the threat stays outside
the defended radius, otherwise the point where the track first crosses the
defended radius (a threat aimed at the ship has its CPA on the ship itself).
The assigned drones wait there, spread over small slots, and detonate at the
engagement time.  All times are simulation seconds.
"""

import math
from dataclasses import dataclass
from typing import Dict, List, Optional, Tuple


@dataclass(frozen=True)
class ThreatType:
    level: int          # drones required, and priority
    speed: float        # m/s
    weight: float       # relative frequency in generated scenarios
    min_z: float
    max_z: float


# Speeds are scaled to the drones' 4 m/s so engagements are feasible in the sim.
DEFAULT_THREAT_TYPES: Dict[str, ThreatType] = {
    "uav":            ThreatType(level=1, speed=2.5, weight=0.5,  min_z=18.0, max_z=28.0),
    "missile":        ThreatType(level=2, speed=3.5, weight=0.35, min_z=14.0, max_z=22.0),
    "cruise_missile": ThreatType(level=3, speed=4.5, weight=0.15, min_z=10.0, max_z=16.0),
}


def parse_threat_types(spec: str) -> Dict[str, ThreatType]:
    """Parse ``"uav:1:2.5:0.5,missile:2:3.5:0.35"`` (type:level:speed:weight).

    Altitude bands are taken from the defaults for known types, else 12-24 m.
    """
    types = {}
    for part in spec.split(","):
        name, level, speed, weight = part.strip().split(":")
        if int(level) < 1 or float(speed) <= 0 or float(weight) < 0:
            raise ValueError(f"bad threat type entry: {part!r}")
        base = DEFAULT_THREAT_TYPES.get(name)
        types[name] = ThreatType(int(level), float(speed), float(weight),
                                 base.min_z if base else 12.0, base.max_z if base else 24.0)
    if not types:
        raise ValueError("empty threat type spec")
    return types


# ---------------------------------------------------------------------------
# Geometry
# ---------------------------------------------------------------------------
Vec3 = Tuple[float, float, float]


def position_at(p0: Vec3, v: Vec3, t0: float, t: float) -> Vec3:
    dt = t - t0
    return (p0[0] + v[0] * dt, p0[1] + v[1] * dt, p0[2] + v[2] * dt)


def closest_point_of_approach(p0: Vec3, v: Vec3, t0: float) -> Tuple[Vec3, float]:
    """CPA of the track to the ship at the origin (horizontal plane). Returns (point, t_cpa)."""
    vv = v[0] * v[0] + v[1] * v[1]
    s = 0.0 if vv < 1e-12 else max(0.0, -(p0[0] * v[0] + p0[1] * v[1]) / vv)
    return position_at(p0, v, t0, t0 + s), t0 + s


def engagement_point(p0: Vec3, v: Vec3, t0: float, defended_radius: float) -> Tuple[Vec3, float]:
    """Where and when the drones should meet the threat. Returns (point, t_engage)."""
    cpa, t_cpa = closest_point_of_approach(p0, v, t0)
    if math.hypot(cpa[0], cpa[1]) >= defended_radius:
        return cpa, t_cpa
    # First crossing of |p0 + v s| = R in the horizontal plane (s >= 0).
    a = v[0] * v[0] + v[1] * v[1]
    b = 2.0 * (p0[0] * v[0] + p0[1] * v[1])
    c = p0[0] * p0[0] + p0[1] * p0[1] - defended_radius * defended_radius
    if c <= 0.0 or a < 1e-12:
        return position_at(p0, v, t0, t0), t0   # already inside the defended radius
    s = (-b - math.sqrt(max(0.0, b * b - 4.0 * a * c))) / (2.0 * a)
    s = max(0.0, s)
    return position_at(p0, v, t0, t0 + s), t0 + s


def slot_point(point: Vec3, slot: int, n_slots: int, radius: float) -> Vec3:
    """Spread n drones on a small horizontal circle around the engagement point."""
    if n_slots <= 1:
        return point
    ang = 2.0 * math.pi * slot / n_slots
    return (point[0] + radius * math.cos(ang), point[1] + radius * math.sin(ang), point[2])


def aim_velocity(p: Vec3, speed: float, miss: float) -> Vec3:
    """Level velocity of `speed` from p toward a point `miss` m beside the ship (at the origin),
    perpendicular to the line of sight: the threat passes the ship at about that distance."""
    bearing = math.atan2(p[1], p[0])
    dx = -math.sin(bearing) * miss - p[0]
    dy = math.cos(bearing) * miss - p[1]
    n = math.hypot(dx, dy)
    return (speed * dx / n, speed * dy / n, 0.0)


# Shared by the ship's feasibility check and the drones' bids so both agree.
ETA_MARGIN = 1.25      # planner detours and imperfect speed profile vs. the ideal trapezoid
ORDER_SLACK_S = 2.0    # bid window + award before a drone starts flying


def eta(distance: float, v_max: float, a_max: float) -> float:
    """Time to fly `distance` from rest to rest with a trapezoidal speed profile."""
    distance = max(0.0, distance)
    if distance >= v_max * v_max / a_max:
        return distance / v_max + v_max / a_max
    return 2.0 * math.sqrt(distance / a_max)


def time_to_intercept(etas: List[float], level: int) -> Optional[float]:
    """Time until `level` drones can be at the point: the level-th smallest ETA."""
    if level < 1 or len(etas) < level:
        return None
    return sorted(etas)[level - 1]


# ---------------------------------------------------------------------------
# Engagement orders (what the ship sends the swarm)
# ---------------------------------------------------------------------------
@dataclass(frozen=True)
class Threat:
    threat_id: str
    type: str
    level: int       # total drones this threat needs; also its priority
    required: int    # drones still to assign in this order (< level on re-announcement)
    location: dict   # engagement point {x, y, z}
    t_engage: float = 0.0   # sim time to detonate at the engagement point

    def to_dict(self) -> dict:
        return {
            "threat_id": self.threat_id,
            "type": self.type,
            "level": self.level,
            "required": self.required,
            "location": dict(self.location),
            "t_engage": self.t_engage,
        }

    @staticmethod
    def from_dict(d: dict) -> "Threat":
        loc = d["location"]
        level = int(d["level"])
        return Threat(
            threat_id=str(d["threat_id"]),
            type=str(d.get("type", "unknown")),
            level=level,
            required=int(d.get("required", level)),
            location={"x": float(loc["x"]), "y": float(loc["y"]), "z": float(loc["z"])},
            t_engage=float(d.get("t_engage", 0.0)),
        )


def priority_order(threats: List[Threat]) -> List[Threat]:
    """Highest level first; threat_id breaks ties so every agent agrees."""
    return sorted(threats, key=lambda t: (-t.level, t.threat_id))
