"""Hardware record (hardware/hardware.json): device classes and the simulation parameters derived
from them.  No zenoh dependency, so it is unit-tested.

The launcher copies the record into config/swarm_runtime.json under "hardware"; drones and the
ship read it from there (load()), the simulator reads the same section.  Speeds and accelerations
are scaled for the simulation by simulation.speed_scale; sensor noise, range and rate are used as
recorded.
"""

import json
import os
from pathlib import Path
from typing import Mapping, Optional

_up = Path(__file__).resolve().parents
RECORD = _up[2] / "hardware" / "hardware.json" if len(_up) > 2 else None   # the repo copy (not in images)


def load(runtime_config: Optional[Mapping] = None, path: Optional[str] = None) -> Optional[dict]:
    """The hardware record: from the runtime config's "hardware" section if present, else the file
    (None if neither exists, e.g. an old runtime config inside a container)."""
    if runtime_config and isinstance(runtime_config.get("hardware"), dict):
        return runtime_config["hardware"]
    f = path or os.environ.get("HARDWARE_RECORD") or RECORD
    return json.loads(Path(f).read_text()) if f and Path(f).exists() else None


def kinematics(hw: Mapping) -> dict:
    """Simulated airframe limits: real class performance x speed_scale."""
    s = float(hw["simulation"]["speed_scale"])
    a = hw["airframe"]
    return {
        "max_velocity": round(a["max_speed_mps"] * s, 6),
        "max_acceleration": round(a["max_accel_mps2"] * s, 6),
        "max_climb": round(a["max_climb_mps"] * s, 6),
        "max_descent": round(a["max_descent_mps"] * s, 6),
    }
