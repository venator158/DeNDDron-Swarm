import random
import unittest

from threat_queue import ThreatQueue


class TestThreatQueue(unittest.TestCase):
    def test_orders_by_time_of_cpa(self):
        q = ThreatQueue()
        for tid, t in [("T1", 50.0), ("T2", 20.0), ("T3", 35.0)]:
            q.push(tid, t)
        self.assertEqual(q.ordered(), ["T2", "T3", "T1"])
        self.assertEqual(q.peek(), "T2")

    def test_pop_returns_most_urgent(self):
        q = ThreatQueue()
        rng = random.Random(0)
        times = {f"T{i}": rng.uniform(0, 100) for i in range(50)}
        for tid, t in times.items():
            q.push(tid, t)
        popped = [q.pop() for _ in range(50)]
        self.assertEqual(popped, sorted(times, key=times.get))
        self.assertIsNone(q.pop())

    def test_lazy_removal(self):
        q = ThreatQueue()
        q.push("T1", 10.0)
        q.push("T2", 20.0)
        q.remove("T1")
        self.assertNotIn("T1", q)
        self.assertEqual(len(q), 1)
        self.assertEqual(q.peek(), "T2")
        self.assertEqual(q.ordered(), ["T2"])

    def test_equal_times_keep_detection_order(self):
        q = ThreatQueue()
        q.push("T9", 5.0)
        q.push("T1", 5.0)
        self.assertEqual(q.ordered(), ["T9", "T1"])

    def test_duplicate_push_rejected(self):
        q = ThreatQueue()
        q.push("T1", 1.0)
        with self.assertRaises(KeyError):
            q.push("T1", 2.0)


if __name__ == "__main__":
    unittest.main()
