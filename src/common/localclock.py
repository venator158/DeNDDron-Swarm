"""Per-node hardware clocks: a drifting, offset oscillator on top of simulated truth.

No zenoh dependency, so it is unit-tested; drones and the ship share it.

Three notions of time are kept apart:

- **truth**: simulator time (sensor-frame sim_time, sim/clock).  Only physics (control-loop dt,
  sensor freshness) and evaluation logging may read it.
- **local**: this node's hardware clock, local = (1 + rho) * truth + theta.  rho is the rate error
  (drift, in ppm), theta the offset at truth 0.
- **synchronized**: the sync layer's estimate of ship time (timesync.py).  Every absolute time the
  protocol exchanges (t_engage, zones, track times, detonation stamps) is in the ship's timebase.

Timestamp jitter (Gaussian, sd jitter_s) applies only to stamp(), which sync exchanges use; read()
is noise-free, so protocol decisions such as "now >= t_engage" never flicker.

Configuration (env, per node; the ship uses prefix SHIP_CLOCK_ instead of CLOCK_):
  CLOCK_DRIFT_PPM, CLOCK_DRIFT_SPREAD_PPM   drift = fixed + uniform(-spread, +spread)
  CLOCK_OFFSET_S, CLOCK_OFFSET_SPREAD_S     offset = fixed + uniform(-spread, +spread)
  CLOCK_JITTER_S                            sd of exchange timestamp noise
  CLOCK_SEED                                the per-node draw depends only on (seed, node id)
All default to 0: a perfect clock, local == truth, which is the behaviour before this module.
"""

import hashlib
import os
import random
from typing import Mapping, Optional


def node_rng(seed, node_id: str) -> random.Random:
    """RNG for one node, reproducible from (seed, node id).  Python's hash() is salted per process,
    so a digest is used instead."""
    digest = hashlib.sha256(f"{seed}:{node_id}".encode()).digest()
    return random.Random(int.from_bytes(digest[:8], "big"))


class LocalClock:
    def __init__(self, drift_ppm: float = 0.0, offset_s: float = 0.0, jitter_s: float = 0.0,
                 rng: Optional[random.Random] = None):
        self.drift_ppm = float(drift_ppm)
        self.offset_s = float(offset_s)
        self.jitter_s = float(jitter_s)
        self.rate = 1.0 + self.drift_ppm * 1e-6
        self._rng = rng or random.Random(0)

    @property
    def perfect(self) -> bool:
        return self.drift_ppm == 0.0 and self.offset_s == 0.0 and self.jitter_s == 0.0

    def read(self, truth: float) -> float:
        """Local clock reading at a truth time (noise-free)."""
        return self.rate * truth + self.offset_s

    def stamp(self, truth: float) -> float:
        """Timestamp for a sync exchange: the reading plus jitter."""
        t = self.read(truth)
        return t + self._rng.gauss(0.0, self.jitter_s) if self.jitter_s > 0.0 else t

    def to_truth(self, local: float) -> float:
        """Truth time of a local reading.  Evaluation only: a node cannot know its own error."""
        return (local - self.offset_s) / self.rate

    def describe(self) -> dict:
        return {"drift_ppm": round(self.drift_ppm, 3), "offset_s": round(self.offset_s, 6),
                "jitter_s": self.jitter_s}


def from_env(node_id: str, prefix: str = "CLOCK_", env: Mapping[str, str] = None) -> LocalClock:
    """This node's clock, drawn reproducibly from CLOCK_SEED and the node id."""
    env = os.environ if env is None else env

    def num(name):
        return float(env.get(prefix + name) or 0.0)

    rng = node_rng(env.get("CLOCK_SEED") or 0, node_id)
    drift = num("DRIFT_PPM") + rng.uniform(-1.0, 1.0) * num("DRIFT_SPREAD_PPM")
    offset = num("OFFSET_S") + rng.uniform(-1.0, 1.0) * num("OFFSET_SPREAD_S")
    return LocalClock(drift, offset, num("JITTER_S"), rng)
