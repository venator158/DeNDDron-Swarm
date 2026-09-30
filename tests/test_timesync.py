import random
import unittest

import timesync
from localclock import LocalClock
from timesync import OffsetRateFilter, ShipMaster, TimeToGo, exchange


def run_master(drone, ship, seconds=120.0, delay=lambda rng: rng.uniform(0.005, 0.05),
               back=None, period=1.0, seed=1):
    """Simulate exchanges: drone sends at truth t, ship answers `period / 2` later (the roster)."""
    rng = random.Random(seed)
    sync = ShipMaster()
    back = back or delay
    t = 1.0
    while t < seconds:
        d_up, d_down = delay(rng), back(rng)
        t1 = drone.stamp(t)
        t2 = ship.stamp(t + d_up)
        t3 = ship.stamp(t + d_up + period / 2)
        t4 = drone.stamp(t + d_up + period / 2 + d_down)
        sync.on_exchange(t1, t2, t3, t4)
        t += period
    return sync, t


def true_error(sync, drone, ship, truth):
    return sync.to_ship(sync.proto_time(drone.read(truth))) - ship.read(truth)


class TestExchange(unittest.TestCase):
    def test_symmetric_exchange_is_exact(self):
        drone, ship = LocalClock(offset_s=-2.5), LocalClock()
        t1 = drone.read(10.0)
        t2, t3 = ship.read(10.02), ship.read(10.5)
        t4 = drone.read(10.52)
        off, delay = exchange(t1, t2, t3, t4)
        self.assertAlmostEqual(off, 2.5)
        self.assertAlmostEqual(delay, 0.04)

    def test_asymmetry_biases_by_half(self):
        drone, ship = LocalClock(), LocalClock()
        off, delay = exchange(drone.read(0), ship.read(0.03), ship.read(0.5), drone.read(0.51))
        self.assertAlmostEqual(off, 0.01)            # (0.03 - 0.01) / 2
        self.assertLessEqual(abs(off), delay / 2 + 1e-12)


class TestShipMaster(unittest.TestCase):
    def test_converges_with_drift_and_offset(self):
        drone, ship = LocalClock(drift_ppm=500, offset_s=3.0), LocalClock(drift_ppm=-20, offset_s=0.7)
        sync, t = run_master(drone, ship)
        err = true_error(sync, drone, ship, t)
        self.assertLess(abs(err), 0.005)
        self.assertAlmostEqual(sync.filter.rate(), (ship.rate - drone.rate) / drone.rate, delta=5e-5)
        self.assertGreaterEqual(sync.error_bound(drone.read(t)), abs(err))

    def test_holdover_uses_the_rate(self):
        drone, ship = LocalClock(drift_ppm=500), LocalClock()
        sync, t = run_master(drone, ship)
        # 60 s without exchanges: the rate estimate keeps the error small (the offset alone would be 30 ms off)
        self.assertLess(abs(true_error(sync, drone, ship, t + 60.0)), 0.005)
        self.assertGreater(sync.error_bound(drone.read(t + 60.0)), sync.error_bound(drone.read(t)))

    def test_jitter_and_delay_outliers(self):
        rng_clock = random.Random(3)
        drone = LocalClock(drift_ppm=100, offset_s=-1.0, jitter_s=0.002, rng=rng_clock)
        ship = LocalClock(jitter_s=0.002, rng=rng_clock)
        spiky = lambda rng: 0.01 + (rng.uniform(0.2, 0.8) if rng.random() < 0.2 else rng.uniform(0, 0.01))
        sync, t = run_master(drone, ship, seconds=200.0, delay=spiky)
        self.assertLess(abs(true_error(sync, drone, ship, t)), 0.01)

    def test_constant_asymmetry_is_within_the_bound(self):
        drone, ship = LocalClock(offset_s=1.0), LocalClock()
        sync, t = run_master(drone, ship, delay=lambda r: 0.04, back=lambda r: 0.01)
        err = true_error(sync, drone, ship, t)
        self.assertAlmostEqual(err, 0.015, delta=0.002)           # (0.04 - 0.01) / 2
        self.assertGreaterEqual(sync.error_bound(drone.read(t)), abs(err))

    def test_startup_congestion_does_not_corrupt_the_rate(self):
        # Seen live: at startup every round trip is long, and some are lopsided with the same total
        # (0.25 s up, 0.01 s back: 0.12 s of bias).  Judged only against the recent minimum round
        # trip, they were fully trusted, and the rate estimate went wrong for a minute.
        drone, ship = LocalClock(drift_ppm=329, offset_s=0.9), LocalClock()
        state = {"n": 0}

        def up(rng):
            state["n"] += 1
            if state["n"] <= 3:
                return 0.13                                   # congested start, symmetric
            if state["n"] <= 6:
                return 0.25                                   # lopsided, same round trip
            return rng.uniform(0.005, 0.02)

        def down(rng):
            if state["n"] <= 3:
                return 0.13
            return 0.01 if state["n"] <= 6 else rng.uniform(0.005, 0.02)

        sync, t = run_master(drone, ship, seconds=40.0, delay=up, back=down)
        self.assertLess(abs(true_error(sync, drone, ship, t)), 0.005)
        self.assertLess(abs(true_error(sync, drone, ship, t + 30.0)), 0.01)   # rate still right

    def test_repeated_exchange_ignored(self):
        sync = ShipMaster()
        self.assertTrue(sync.on_exchange(1.0, 1.1, 1.5, 1.6))
        self.assertFalse(sync.on_exchange(1.0, 1.1, 1.5, 1.6))
        self.assertEqual(sync.filter.samples, 1)

    def test_before_any_exchange_local_is_used(self):
        sync = ShipMaster()
        self.assertEqual(sync.proto_time(12.0), 12.0)
        self.assertIsNone(sync.error_bound(12.0))


class TestTimeToGo(unittest.TestCase):
    def test_anchor_on_receipt(self):
        drone, ship = LocalClock(offset_s=5.0), LocalClock()
        sync = TimeToGo()
        delay = 0.05
        sent = ship.read(100.0)
        local_rx = drone.read(100.0 + delay)
        t_local = sync.inbound(ship.read(130.0), sent, local_rx)   # ship's t_engage at truth 130
        # the drone fires when its local clock reads t_local: late by exactly the one-way delay
        self.assertAlmostEqual(drone.to_truth(t_local), 130.0 + delay)
        self.assertEqual(sync.proto_time(7.0), 7.0)
        self.assertAlmostEqual(sync.to_ship(local_rx), sent)

    def test_any_stamped_message_reanchors(self):
        sync = TimeToGo()
        self.assertEqual(sync.to_ship(10.0), 10.0)          # no anchor yet: the local clock
        sync.on_sent(105.0, 100.0)                           # e.g. an empty zone list
        self.assertEqual(sync.to_ship(101.0), 106.0)
        sync.on_sent(None, 101.0)
        self.assertEqual(sync.to_ship(101.0), 106.0)

    def test_without_stamp_is_identity(self):
        self.assertEqual(TimeToGo().inbound(42.0, None, 1.0), 42.0)


class TestMake(unittest.TestCase):
    def test_modes(self):
        self.assertEqual(timesync.make("none").mode, "none")
        self.assertEqual(timesync.make("ttg").mode, "ttg")
        self.assertEqual(timesync.make("MASTER").mode, "master")
        with self.assertRaises(ValueError):
            timesync.make("gps")

    def test_filter_first_sample(self):
        f = OffsetRateFilter()
        f.update(10.0, 2.0, 0.02)
        self.assertEqual(f.offset(10.0), 2.0)
        self.assertEqual(f.rate(), 0.0)


if __name__ == "__main__":
    unittest.main()
