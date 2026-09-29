import random
import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src" / "agent"))

from threats import DEFAULT_THREAT_MIX, Threat, admit, parse_mix, priority_order, random_threats

LOC = {"x": 0.0, "y": 30.0, "z": 20.0}


def T(tid, level, kind="t"):
    return Threat(tid, kind, level, level, LOC)


class TestAdmit(unittest.TestCase):
    def test_sum_of_levels_never_exceeds_free_drones(self):
        rng = random.Random(0)
        for _ in range(500):
            cands = random_threats(rng, DEFAULT_THREAT_MIX, rng.randint(1, 8), 1, 25, 45, 14, 26)
            free = rng.randint(0, 10)
            admitted, unengaged = admit(cands, free)
            self.assertLessEqual(sum(t.level for t in admitted), free)
            self.assertEqual(len(admitted) + len(unengaged), len(cands))

    def test_highest_priority_admitted_first(self):
        admitted, unengaged = admit([T("uav", 1), T("cm", 3), T("msl", 2)], free_drones=3)
        self.assertEqual([t.threat_id for t in admitted], ["cm"])
        self.assertEqual({t.threat_id for t in unengaged}, {"uav", "msl"})

    def test_smaller_threat_fills_leftover_budget(self):
        admitted, _ = admit([T("cm", 3), T("msl", 2), T("uav", 1)], free_drones=4)
        self.assertEqual([t.threat_id for t in admitted], ["cm", "uav"])

    def test_no_drones_admits_nothing(self):
        admitted, unengaged = admit([T("uav", 1)], free_drones=0)
        self.assertEqual(admitted, [])
        self.assertEqual(len(unengaged), 1)

    def test_negative_budget_treated_as_zero(self):
        self.assertEqual(admit([T("uav", 1)], free_drones=-3)[0], [])


class TestThreatModel(unittest.TestCase):
    def test_priority_order(self):
        order = priority_order([T("b", 1), T("a", 1), T("z", 3)])
        self.assertEqual([t.threat_id for t in order], ["z", "a", "b"])

    def test_dict_roundtrip(self):
        t = Threat("T1", "missile", 2, 1, LOC)
        self.assertEqual(Threat.from_dict(t.to_dict()), t)

    def test_required_defaults_to_level(self):
        d = T("T1", 3).to_dict()
        del d["required"]
        self.assertEqual(Threat.from_dict(d).required, 3)

    def test_parse_mix(self):
        self.assertEqual(parse_mix("uav:1:0.7, missile:2:0.3"), {"uav": (1, 0.7), "missile": (2, 0.3)})
        with self.assertRaises(ValueError):
            parse_mix("uav:0:1")

    def test_random_threats_use_mix_levels_and_unique_ids(self):
        ts = random_threats(random.Random(1), DEFAULT_THREAT_MIX, 20, 5, 25, 45, 14, 26)
        self.assertEqual(len({t.threat_id for t in ts}), 20)
        for t in ts:
            self.assertEqual(t.level, DEFAULT_THREAT_MIX[t.type][0])
            self.assertEqual(t.required, t.level)


if __name__ == "__main__":
    unittest.main()
