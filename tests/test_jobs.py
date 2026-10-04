import unittest

from jobs import arbitrate, heartbeat_answer


class TestArbitrate(unittest.TestCase):
    def test_best_bid_confirmed_other_rejected(self):
        # Two drones both believe they won a level-1 threat (they could not hear each other).
        accepted, rejected = arbitrate({"drone_3": 22.6, "drone_8": 6.0}, {}, need=1, n_slots=1)
        self.assertEqual(accepted, {"drone_8": 0})
        self.assertEqual(rejected, ["drone_3"])

    def test_level_two_gets_distinct_slots(self):
        accepted, rejected = arbitrate({"a": 5.0, "b": 3.0, "c": 9.0}, {}, need=2, n_slots=2)
        self.assertEqual(accepted, {"a": 0, "b": 1})
        self.assertEqual(rejected, ["c"])

    def test_slots_match_drones_tentative_convention(self):
        # Drones take slot = rank by id among the winners while tentative; the ship must agree,
        # or confirmed drones swap slots across the formation (seen: 17-24 m misses at 50 drones).
        pending = {"drone_9": 12.0, "drone_13": 14.7, "drone_5": 10.1}
        winners = sorted(pending)
        accepted, _ = arbitrate(pending, {}, need=3, n_slots=3)
        self.assertEqual(accepted, {d: winners.index(d) for d in winners})

    def test_fills_only_free_slots(self):
        # Re-announced level-2 threat: one drone already confirmed in slot 0.
        accepted, rejected = arbitrate({"x": 4.0}, {"a": 0}, need=1, n_slots=2)
        self.assertEqual(accepted, {"x": 1})
        self.assertEqual(rejected, [])

    def test_nothing_needed_rejects_all(self):
        accepted, rejected = arbitrate({"x": 1.0, "y": 2.0}, {"a": 0}, need=0, n_slots=1)
        self.assertEqual(accepted, {})
        self.assertEqual(sorted(rejected), ["x", "y"])

    def test_already_confirmed_is_neither(self):
        accepted, rejected = arbitrate({"a": 1.0}, {"a": 0}, need=0, n_slots=1)
        self.assertEqual((accepted, rejected), ({}, []))

    def test_tie_broken_by_id(self):
        accepted, _ = arbitrate({"d2": 5.0, "d1": 5.0}, {}, need=1, n_slots=1)
        self.assertEqual(accepted, {"d1": 0})


class TestHeartbeatAnswer(unittest.TestCase):
    """The ship's answer to a heartbeat that names a job (lost ACKs, awards and job updates)."""

    def answer(self, status="approved", holder=False, pending=False, rejected=False, hb_confirmed=False):
        return heartbeat_answer(status, holder, pending, rejected, hb_confirmed)

    def test_holder_that_missed_its_ack_gets_it_again(self):
        self.assertEqual(self.answer(holder=True), "ack")

    def test_holder_that_knows_needs_nothing(self):
        self.assertEqual(self.answer(holder=True, hb_confirmed=True), "none")

    def test_lost_award_is_taken_from_the_heartbeat(self):
        self.assertEqual(self.answer(), "pending")

    def test_pending_award_waits_for_arbitration(self):
        self.assertEqual(self.answer(pending=True), "none")

    def test_refused_award_gets_its_nack_again(self):
        self.assertEqual(self.answer(rejected=True), "nack")

    def test_dropped_drone_that_believes_it_holds_gets_the_job_update(self):
        self.assertEqual(self.answer(hb_confirmed=True), "job")

    def test_closed_job_releases_any_drone(self):
        for status in ("destroyed", "leaked", "failed", "impact"):
            for holder in (False, True):
                self.assertEqual(self.answer(status=status, holder=holder, hb_confirmed=holder), "job")


if __name__ == "__main__":
    unittest.main()
