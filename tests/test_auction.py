import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src" / "agent"))

from auction import AuctionManager, assign
from threats import Threat

LOC = {"x": 1.0, "y": 2.0, "z": 10.0}


def T(tid, level, required=None, kind="t"):
    return Threat(tid, kind, level, level if required is None else required, LOC)


def run_wave(threats, bids, window=1.0):
    """Every agent hears every bid; return {agent_id: WaveResult}."""
    mgrs = {a: AuctionManager(a, bid_window_s=window) for a in bids}
    for m in mgrs.values():
        m.on_wave("W1", threats, now=0.0)
    for a, costs in bids.items():
        for m in mgrs.values():
            m.on_bid("W1", a, costs)
    return {a: m.close_due(now=window)[0] for a, m in mgrs.items()}


class TestAssign(unittest.TestCase):
    def test_level_is_number_of_drones(self):
        bids = {f"d{i}": {"M": float(i)} for i in range(1, 6)}
        self.assertEqual(assign([T("M", 3)], bids), {"M": ["d1", "d2", "d3"]})

    def test_higher_level_threat_gets_first_pick(self):
        # d1 is closest to both; the level-2 missile must get it, not the level-1 UAV.
        bids = {
            "d1": {"UAV": 1.0, "MSL": 1.0},
            "d2": {"UAV": 2.0, "MSL": 9.0},
            "d3": {"UAV": 3.0, "MSL": 8.0},
        }
        out = assign([T("UAV", 1), T("MSL", 2)], bids)
        self.assertEqual(out["MSL"], ["d1", "d3"])
        self.assertEqual(out["UAV"], ["d2"])

    def test_each_drone_assigned_at_most_once(self):
        bids = {f"d{i}": {"A": float(i), "B": float(i), "C": float(i)} for i in range(1, 7)}
        out = assign([T("A", 3), T("B", 2), T("C", 1)], bids)
        flat = [a for ws in out.values() for a in ws]
        self.assertEqual(len(flat), len(set(flat)))
        self.assertEqual(sum(len(ws) for ws in out.values()), 6)

    def test_threat_that_cannot_be_fully_covered_is_declined(self):
        bids = {"d1": {"M": 1.0}, "d2": {"M": 2.0}}
        self.assertEqual(assign([T("M", 3)], bids), {"M": []})

    def test_declined_threat_leaves_drones_for_lower_priority(self):
        # Level-3 threat cannot be covered by 2 drones; the level-1 threat still gets one.
        bids = {"d1": {"A": 1.0, "B": 1.0}, "d2": {"A": 2.0, "B": 2.0}}
        out = assign([T("A", 1), T("B", 3)], bids)
        self.assertEqual(out["B"], [])
        self.assertEqual(out["A"], ["d1"])

    def test_exactly_enough_drones_is_accepted(self):
        bids = {"d1": {"M": 1.0}, "d2": {"M": 2.0}}
        self.assertEqual(assign([T("M", 2)], bids), {"M": ["d1", "d2"]})

    def test_tie_broken_by_agent_id(self):
        bids = {"d3": {"A": 5.0}, "d1": {"A": 5.0}, "d2": {"A": 5.0}}
        self.assertEqual(assign([T("A", 2)], bids)["A"], ["d1", "d2"])

    def test_equal_level_threats_ordered_by_id(self):
        bids = {"d1": {"X": 1.0, "Y": 1.0}}
        self.assertEqual(assign([T("Y", 1), T("X", 1)], bids), {"X": ["d1"], "Y": []})

    def test_reannounced_threat_uses_required_not_level(self):
        bids = {f"d{i}": {"M": float(i)} for i in range(1, 5)}
        self.assertEqual(assign([T("M", 3, required=1)], bids)["M"], ["d1"])


class TestAuctionManager(unittest.TestCase):
    def test_all_agents_compute_same_assignment(self):
        threats = [T("UAV", 1), T("MSL", 2)]
        bids = {
            "d1": {"UAV": 4.0, "MSL": 1.0},
            "d2": {"UAV": 1.0, "MSL": 7.0},
            "d3": {"UAV": 6.0, "MSL": 2.0},
            "d4": {"UAV": 9.0, "MSL": 9.0},
        }
        res = run_wave(threats, bids)
        self.assertEqual(len({str(r.assignment) for r in res.values()}), 1)
        mine = {a: (r.my_threat.threat_id if r.my_threat else None) for a, r in res.items()}
        self.assertEqual(mine, {"d1": "MSL", "d3": "MSL", "d2": "UAV", "d4": None})

    def test_not_closed_before_window(self):
        m = AuctionManager("d1", bid_window_s=1.0)
        m.on_wave("W1", [T("A", 1)], now=10.0)
        m.on_bid("W1", "d1", {"A": 3.0})
        self.assertEqual(m.close_due(now=10.5), [])
        self.assertTrue(m.has_open_waves())
        self.assertEqual(len(m.close_due(now=11.0)), 1)
        self.assertFalse(m.has_open_waves())

    def test_duplicate_wave_ignored(self):
        m = AuctionManager("d1")
        self.assertTrue(m.on_wave("W1", [T("A", 1)], now=0.0))
        self.assertFalse(m.on_wave("W1", [T("A", 1)], now=0.5))

    def test_bid_before_wave_is_kept(self):
        m = AuctionManager("d1")
        m.on_bid("W1", "d2", {"A": 2.0})
        m.on_wave("W1", [T("A", 1)], now=0.0)
        m.on_bid("W1", "d1", {"A": 9.0})
        r = m.close_due(now=1.0)[0]
        self.assertEqual(r.assignment["A"], ["d2"])
        self.assertIsNone(r.my_threat)

    def test_late_bid_ignored(self):
        m = AuctionManager("d1")
        m.on_wave("W1", [T("A", 1)], now=0.0)
        m.on_bid("W1", "d1", {"A": 9.0})
        m.close_due(now=1.0)
        m.on_bid("W1", "d2", {"A": 1.0})
        self.assertEqual(m.close_due(now=2.0), [])
        self.assertEqual(m.held.threat_id, "A")

    def test_no_bids_assigns_nobody(self):
        m = AuctionManager("d1")
        m.on_wave("W1", [T("A", 2)], now=0.0)
        r = m.close_due(now=1.0)[0]
        self.assertEqual(r.assignment, {"A": []})
        self.assertIsNone(m.held)

    def test_overfilled_threat_worst_holder_yields(self):
        # d3 missed d1's and d2's better bids, so it believes {d3, d4} cover the level-2 threat.
        m3 = AuctionManager("d3")
        m3.on_wave("W1", [T("M", 2)], now=0.0)
        m3.on_bid("W1", "d3", {"M": 9.0})
        m3.on_bid("W1", "d4", {"M": 10.0})
        self.assertEqual(m3.close_due(now=1.0)[0].my_threat.threat_id, "M")
        self.assertFalse(m3.on_award("M", "d1", 1.0))   # one better holder: still needed
        self.assertTrue(m3.on_award("M", "d2", 2.0))    # level reached by better bids: yield

    def test_better_holder_does_not_yield_to_worse_awards(self):
        m = AuctionManager("d1")
        m.on_wave("W1", [T("M", 2)], now=0.0)
        m.on_bid("W1", "d1", {"M": 1.0})
        m.on_bid("W1", "d2", {"M": 5.0})
        self.assertEqual(m.close_due(now=1.0)[0].my_threat.threat_id, "M")
        self.assertFalse(m.on_award("M", "d2", 5.0))
        self.assertFalse(m.on_award("M", "d3", 6.0))

    def test_withdrawn_holder_no_longer_counts(self):
        m = AuctionManager("d3")
        m.on_wave("W1", [T("M", 2)], now=0.0)
        m.on_bid("W1", "d3", {"M": 9.0})
        m.on_bid("W1", "d4", {"M": 10.0})
        self.assertEqual(m.close_due(now=1.0)[0].my_threat.threat_id, "M")
        m.on_award("M", "d1", 1.0)
        m.on_withdraw("M", "d1")
        self.assertFalse(m.on_award("M", "d2", 2.0))

    def test_invalid_window_rejected(self):
        with self.assertRaises(ValueError):
            AuctionManager("d1", bid_window_s=0)


if __name__ == "__main__":
    unittest.main()
