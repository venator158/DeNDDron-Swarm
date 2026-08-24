import unittest
import time
import sys
import os

# Add src/agent to path for imports
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "src", "agent")))


class TestTimeHandling(unittest.TestCase):

    def test_timing_state_classification(self):

        def classify_time_step(current_time, last_sim_time, max_control_dt=0.2):
            if current_time is None:
                return "MISSING_TIME", 0.02
            if last_sim_time is None:
                return "FIRST_FRAME", 0.02
            if current_time == last_sim_time:
                return "PAUSED_ZERO_DT", 0.0
            if current_time < last_sim_time:
                if current_time < 0.5:
                    return "TIME_RESET", 0.02
                return "OUT_OF_ORDER", 0.0
            
            raw_dt = current_time - last_sim_time
            if raw_dt > max_control_dt:
                return "LARGE_DT", max_control_dt
            return "NORMAL", raw_dt

        # Test 1: First frame
        state, dt = classify_time_step(10.0, None)
        self.assertEqual(state, "FIRST_FRAME")
        self.assertEqual(dt, 0.02)

        # Test 2: Normal step
        state, dt = classify_time_step(10.05, 10.0)
        self.assertEqual(state, "NORMAL")
        self.assertAlmostEqual(dt, 0.05)

        # Test 3: Paused simulation
        state, dt = classify_time_step(10.05, 10.05)
        self.assertEqual(state, "PAUSED_ZERO_DT")
        self.assertEqual(dt, 0.0)

        # Test 4: Simulation reset
        state, dt = classify_time_step(0.01, 105.0)
        self.assertEqual(state, "TIME_RESET")
        self.assertEqual(dt, 0.02)

        # Test 5: Out of order frame
        state, dt = classify_time_step(10.02, 10.05)
        self.assertEqual(state, "OUT_OF_ORDER")
        self.assertEqual(dt, 0.0)

        # Test 6: Large dt jump
        state, dt = classify_time_step(12.5, 10.0, max_control_dt=0.2)
        self.assertEqual(state, "LARGE_DT")
        self.assertEqual(dt, 0.2)

        # Test 7: Missing sim time
        state, dt = classify_time_step(None, 10.0)
        self.assertEqual(state, "MISSING_TIME")
        self.assertEqual(dt, 0.02)


if __name__ == "__main__":
    unittest.main()
