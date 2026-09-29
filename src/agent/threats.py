"""Threat model shared by the agents and the threat dispatcher.

A threat's *level* is both its priority and the number of drones it needs:
a level-3 cruise missile is engaged before a level-1 UAV and takes three
drones.  Drones are expendable, so every engagement permanently consumes
``level`` drones.
"""

import math
import random
from dataclasses import dataclass
from typing import Dict, List, Tuple

# type -> (level, relative frequency when generating scenarios)
DEFAULT_THREAT_MIX: Dict[str, Tuple[int, float]] = {
    "uav": (1, 0.5),
    "missile": (2, 0.35),
    "cruise_missile": (3, 0.15),
}


@dataclass(frozen=True)
class Threat:
    threat_id: str
    type: str
    level: int      # total drones this threat needs; also its priority
    required: int   # drones still to assign in this wave (< level on re-announcement)
    location: dict

    def to_dict(self) -> dict:
        return {
            "threat_id": self.threat_id,
            "type": self.type,
            "level": self.level,
            "required": self.required,
            "location": dict(self.location),
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
        )


def priority_order(threats: List[Threat]) -> List[Threat]:
    """Highest level first; threat_id breaks ties so every agent agrees."""
    return sorted(threats, key=lambda t: (-t.level, t.threat_id))


def parse_mix(spec: str) -> Dict[str, Tuple[int, float]]:
    """Parse ``"uav:1:0.5,missile:2:0.35"`` (type:level:weight)."""
    mix = {}
    for part in spec.split(","):
        name, level, weight = part.strip().split(":")
        if int(level) < 1 or float(weight) < 0:
            raise ValueError(f"bad threat mix entry: {part!r}")
        mix[name] = (int(level), float(weight))
    if not mix:
        raise ValueError("empty threat mix")
    return mix


def admit(candidates: List[Threat], free_drones: int) -> Tuple[List[Threat], List[Threat]]:
    """Choose which threats to engage so the sum of levels never exceeds free drones.

    Threats are considered in priority order; one that does not fit is skipped
    and smaller ones after it may still be admitted.
    Returns ``(admitted, unengaged)``.
    """
    admitted, unengaged = [], []
    budget = max(0, int(free_drones))
    for t in priority_order(candidates):
        if t.required <= budget:
            admitted.append(t)
            budget -= t.required
        else:
            unengaged.append(t)
    return admitted, unengaged


def random_threats(rng: random.Random, mix: Dict[str, Tuple[int, float]], count: int, id_start: int,
                   min_r: float, max_r: float, min_z: float, max_z: float) -> List[Threat]:
    """Generate ``count`` threats at random points in a ring around the ship."""
    names = list(mix)
    weights = [mix[n][1] for n in names]
    out = []
    for i in range(count):
        kind = rng.choices(names, weights=weights)[0]
        level = mix[kind][0]
        r = rng.uniform(min_r, max_r)
        theta = rng.uniform(0.0, 2.0 * math.pi)
        out.append(Threat(
            threat_id=f"T{id_start + i}",
            type=kind,
            level=level,
            required=level,
            location={"x": r * math.cos(theta), "y": r * math.sin(theta), "z": rng.uniform(min_z, max_z)},
        ))
    return out
