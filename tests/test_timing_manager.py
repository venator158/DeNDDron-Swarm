import unittest
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src" / "agent"))

from timing import TimingManager, TimingState

class FakeClock:
    def __init__(self):
        self.now = 0.0
    def __call__(self):
        return self.now
    def advance(self, dt):
        self.now += dt


class TestTimingManager(unittest.TestCase):
    def setUp(self):
        self.clock = FakeClock()
        self.t = TimingManager(clock=self.clock, max_control_dt=0.2, sensor_timeout_s=0.5)

    def test_control_states(self):
        self.assertEqual(self.t.step(10.0).state, TimingState.FIRST_FRAME)
        self.assertEqual(self.t.step(10.05).state, TimingState.NORMAL)
        self.assertAlmostEqual(self.t.step(10.10).sim_dt, 0.05)
        self.assertEqual(self.t.step(10.10).state, TimingState.PAUSED_ZERO_DT)
        self.assertEqual(self.t.step(10.40).state, TimingState.LARGE_DT)
        self.assertAlmostEqual(self.t.step(10.40).sim_dt, 0.0)

    def test_large_dt_is_clamped(self):
        self.t.step(1.0)
        snap = self.t.step(1.8)
        self.assertEqual(snap.state, TimingState.LARGE_DT)
        self.assertEqual(snap.sim_dt, 0.2)
        self.assertAlmostEqual(self.t.step(1.9).sim_dt, 0.1)

    def test_missing_time(self):
        snap = self.t.step(None)
        self.assertEqual(snap.state, TimingState.MISSING_TIME)
        self.assertEqual(snap.sim_dt, 0.02)
        snap = self.t.step(float('nan'))
        self.assertEqual(snap.state, TimingState.MISSING_TIME)

    def test_sensor_ordering_and_reset(self):
        self.assertEqual(self.t.record_sensor(10.0)[1], TimingState.FIRST_FRAME)
        accepted, state = self.t.record_sensor(9.0)
        self.assertFalse(accepted)
        self.assertEqual(state, TimingState.OUT_OF_ORDER)
        accepted, state = self.t.record_sensor(0.1)
        self.assertTrue(accepted)
        self.assertEqual(state, TimingState.TIME_RESET)
        self.assertEqual(self.t.step(0.1).state, TimingState.TIME_RESET)

    def test_wall_clock_freshness_is_independent_of_sim_time(self):
        self.t.record_sensor(10.0)
        self.clock.advance(0.49)
        self.assertTrue(self.t.sensor_is_fresh())
        self.clock.advance(0.02)
        self.assertFalse(self.t.sensor_is_fresh())
        # Simulation can be paused while wall-clock freshness expires.
        self.assertEqual(self.t.step(10.0).state, TimingState.FIRST_FRAME)


if __name__ == '__main__':
    unittest.main()
