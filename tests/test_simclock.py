import importlib
import os
import unittest
from unittest import mock

import simclock


class FakeTime:
    def __init__(self):
        self.t = 1000.0

    def monotonic(self):
        return self.t


class TestSimClock(unittest.TestCase):
    def setUp(self):
        self.clock = FakeTime()
        with mock.patch.dict(os.environ, {"SIM_RTF": "3"}):
            importlib.reload(simclock)
        self.patch = mock.patch.object(simclock.time, "monotonic", self.clock.monotonic)
        self.patch.start()

    def tearDown(self):
        self.patch.stop()
        importlib.reload(simclock)

    def feed(self, wall_s, sim_rate, sim_start=0.0, step=0.02):
        """Advance wall time by wall_s while the simulator runs at sim_rate."""
        sim = sim_start
        for _ in range(int(wall_s / step)):
            self.clock.t += step
            sim += step * sim_rate
            simclock.observe(sim)
        return sim

    def test_runs_at_target_before_any_observation(self):
        a = simclock.now()
        self.clock.t += 2.0
        self.assertAlmostEqual(simclock.now() - a, 6.0)

    def test_first_observation_does_not_jump(self):
        a = simclock.now()
        self.clock.t += 0.1
        simclock.observe(12.5)
        self.assertAlmostEqual(simclock.now() - a, 0.3)

    def test_follows_a_simulator_slower_than_target(self):
        # Target 3x, but the simulator only manages 1.7x (as at 25 drones).
        sim = self.feed(5.0, 1.7)
        a, sim_a = simclock.now(), sim
        sim = self.feed(10.0, 1.7, sim_start=sim)
        self.assertAlmostEqual(simclock.now() - a, sim - sim_a, delta=0.1)
        self.assertAlmostEqual(simclock.rate(), 1.7, delta=0.05)

    def test_keeps_running_when_observations_stop(self):
        self.feed(5.0, 2.0)
        a = simclock.now()
        self.clock.t += 3.0                       # onboard link silent
        self.assertAlmostEqual(simclock.now() - a, 6.0, delta=0.2)

    def test_never_goes_backwards(self):
        sim = self.feed(3.0, 3.0)
        self.clock.t += 1.0
        before = simclock.now()                   # extrapolated to sim + 3
        simclock.observe(sim + 0.5)               # simulator was actually slower
        self.assertGreaterEqual(simclock.now(), before)

    def test_restart_keeps_clock_continuous(self):
        self.feed(3.0, 3.0, sim_start=500.0)
        a = simclock.now()
        self.clock.t += 0.02
        simclock.observe(0.0)                     # world reset
        self.assertAlmostEqual(simclock.now() - a, 0.06, delta=0.01)

    def test_truth_now_extrapolates_without_shift(self):
        self.assertIsNone(simclock.truth_now())
        sim = self.feed(2.0, 3.0, sim_start=50.0)
        self.assertAlmostEqual(simclock.truth_now(), sim, delta=1e-6)
        self.clock.t += 0.01
        self.assertAlmostEqual(simclock.truth_now(), sim + 0.03, delta=0.002)


if __name__ == "__main__":
    unittest.main()
