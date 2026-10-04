"""Hardware record (hardware/hardware.json) and the truth boundary."""
import re
import unittest
from pathlib import Path

import hardware

REPO = Path(__file__).resolve().parents[1]


class TestHardwareRecord(unittest.TestCase):
    def test_every_class_documented(self):
        hw = hardware.load()
        for key in ("airframe", "warhead", "uwb", "uwb_anchors", "radar", "imu", "ahrs", "barometer", "compass",
                    "gnss", "ship_gnss"):
            self.assertIn("class", hw[key], key)
            self.assertIn("simulated", hw[key], key)

    def test_kinematics_reproduce_the_original_drones(self):
        # speed_scale 0.1 x a 40 m/s, 10 m/s^2 airframe = the original 4 m/s, 1 m/s^2
        k = hardware.kinematics(hardware.load())
        self.assertEqual((k["max_velocity"], k["max_acceleration"]), (4.0, 1.0))

    def test_imu_errors_follow_the_time_stretch(self):
        hw = hardware.load()
        real, sim = hardware.imu_errors(hw, "real"), hardware.imu_errors(hw, "dilated")
        s = hw["simulation"]["speed_scale"]
        self.assertAlmostEqual(real["accel_bias"], 1e-3 * hardware.G)            # 1 mg
        self.assertAlmostEqual(real["tilt_bias"], 0.0856, places=3)              # g sin(0.5 deg)
        self.assertAlmostEqual(sim["tilt_bias"], real["tilt_bias"] * s * s)
        self.assertAlmostEqual(sim["tilt_tau"], real["tilt_tau"] / s)
        self.assertAlmostEqual(sim["noise_density"], real["noise_density"] * s ** 1.5)
        # the same drift over the same mission phase: 1/2 b t^2 with t_sim = t_real / s
        t_real = 6.0
        self.assertAlmostEqual(0.5 * sim["tilt_bias"] * (t_real / s) ** 2, 0.5 * real["tilt_bias"] * t_real ** 2)

    def test_gnss_errors(self):
        hw = hardware.load()
        g = hardware.gnss_errors(hw)
        s = hw["simulation"]["speed_scale"]
        self.assertAlmostEqual(g["receiver_tau"], hw["gnss"]["receiver_tau_s"] / s)     # time-stretched
        self.assertAlmostEqual(g["receiver_sigma"], hw["gnss"]["receiver_sigma_m"])     # metres: not scaled
        self.assertLess(g["rel_sigma"], g["common_sigma"])                               # the common part cancels

    def test_runtime_config_section_wins(self):
        self.assertEqual(hardware.load({"hardware": {"x": 1}}), {"x": 1})
        self.assertIsNone(hardware.load({}, path="/nonexistent/hardware.json"))


class TestTruthBoundary(unittest.TestCase):
    def test_drones_never_read_truth_streams(self):
        # sim/truth (true poses) is for physics and evaluation only.
        for f in (REPO / "src" / "agent").glob("*.py"):
            self.assertIsNone(re.search(r"sim/truth", f.read_text()), f.name)


if __name__ == "__main__":
    unittest.main()
