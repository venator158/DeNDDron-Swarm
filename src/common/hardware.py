"""Hardware record (hardware/hardware.json): device classes and the simulation parameters derived
from them.  No zenoh dependency, so it is unit-tested.

The launcher copies the record into config/swarm_runtime.json under "hardware"; drones and the
ship read it from there (load()), the simulator reads the same section.  Speeds and accelerations
are scaled for the simulation by simulation.speed_scale; sensor noise, range and rate are used as
recorded.
"""

import json
import math
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


G = 9.80665


def imu_errors(hw: Mapping, scaling: Optional[str] = None) -> dict:
    """Simulated horizontal IMU errors: the flight controller's accelerometer, and its attitude (tilt)
    error, which leaks gravity into the horizontal axes as an equivalent accelerometer bias.

    The simulation stretches time: distances are real, speeds are x speed_scale (s), so a mission
    phase takes 1/s times longer in sim seconds.  An error that grows with time is scaled so that a
    blackout drifts as far in the simulation as the same phase would in reality (IMU_ERROR_SCALING
    "dilated", the default, owner's decision): biases (m/s^2) x s^2, noise densities
    (m/s^2/sqrt(Hz)) x s^1.5, correlation times / s; the scale factor is dimensionless.  "real" uses
    the record's figures unscaled: a stress setting (at s = 0.1, ~100x the drift per mission phase).
    The simulator applies the same rule (GazeboSimulator.cpp, configure_imu).

    Returns sim units: accel_bias (m/s^2, 1 sigma per axis), accel_bias_tau (s), tilt_bias (m/s^2,
    g sin tilt), tilt_tau (s), noise_density (m/s^2/sqrt(Hz)), scale_factor, output_hz, scaling.
    """
    scaling = (scaling or os.environ.get("IMU_ERROR_SCALING") or "dilated").strip().lower()
    s = float(hw["simulation"]["speed_scale"]) if scaling == "dilated" else 1.0
    imu, ahrs = hw["imu"], hw.get("ahrs", {})
    tilt = math.radians(float(ahrs.get("tilt_sigma_deg", 0.0)))
    return {
        "accel_bias": float(imu.get("accel_bias_mg", 0.0)) * 1e-3 * G * s * s,
        "accel_bias_tau": float(imu.get("accel_bias_tau_s", 300.0)) / s,
        "tilt_bias": G * math.sin(tilt) * s * s,
        "tilt_tau": float(ahrs.get("tilt_tau_s", 20.0)) / s,
        "noise_density": float(imu.get("accel_noise_ug_rthz", 0.0)) * 1e-6 * G * s ** 1.5,
        "scale_factor": float(imu.get("accel_scale_factor", 0.0)),
        "output_hz": float(imu.get("output_hz", 50.0)),
        "scaling": scaling,
    }


def gnss_errors(hw: Mapping) -> dict:
    """Simulated GNSS errors (metres, per horizontal axis, 1 sigma) and the drone-relative-to-ship
    figure the drones assume.  Positions are not speed-scaled; correlation times follow the
    simulation's time stretch (/ speed_scale), as the IMU's do.

    Returns common (sigma, tau), receiver (sigma, tau), white, rate_hz, min_sats, heading_sigma (rad),
    rel_sigma (the relative fix's per-axis noise without the heading term: both receivers' own parts
    and white noise; the common part cancels) and rel_bias_sigma (its slowly varying part)."""
    s = float(hw["simulation"]["speed_scale"])
    g, sg = hw["gnss"], hw.get("ship_gnss", {})
    rec, white = float(g["receiver_sigma_m"]), float(g["white_sigma_m"])
    return {
        "common_sigma": float(g["common_sigma_m"]), "common_tau": float(g["common_tau_s"]) / s,
        "receiver_sigma": rec, "receiver_tau": float(g["receiver_tau_s"]) / s,
        "white_sigma": white, "rate_hz": float(g.get("rate_hz", 5.0)), "min_sats": int(g.get("min_sats", 6)),
        "heading_sigma": math.radians(float(sg.get("heading_sigma_deg", 0.0))),
        "rel_bias_sigma": math.sqrt(2.0) * rec,
        "rel_sigma": math.sqrt(2.0 * rec * rec + 2.0 * white * white),
    }
