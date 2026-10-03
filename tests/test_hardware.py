"""Hardware record (hardware/hardware.json) and the truth boundary."""
import re
import unittest
from pathlib import Path

import hardware

REPO = Path(__file__).resolve().parents[1]


class TestHardwareRecord(unittest.TestCase):
    def test_every_class_documented(self):
        hw = hardware.load()
        for key in ("airframe", "warhead", "uwb", "uwb_anchors", "radar", "imu", "barometer", "compass"):
            self.assertIn("class", hw[key], key)
            self.assertIn("simulated", hw[key], key)

    def test_kinematics_reproduce_the_original_drones(self):
        # speed_scale 0.1 x a 40 m/s, 10 m/s^2 airframe = the original 4 m/s, 1 m/s^2
        k = hardware.kinematics(hardware.load())
        self.assertEqual((k["max_velocity"], k["max_acceleration"]), (4.0, 1.0))

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
