"""Ship-anchored consensus (timesync.Consensus) on a simulated network."""
import random
import unittest

from localclock import LocalClock
from timesync import AGGREGATORS, Consensus, make


class Net:
    """Ship (pinned leader, roster beacons at 1 Hz) + drones (beacons at beacon_hz), event-stepped.

    links(t) -> set of (sender, receiver) pairs that can hear each other at truth time t.
    """

    def __init__(self, n=6, seed=1, drift_ppm=500, offset_s=3.0, beacon_hz=0.5, delay=(0.002, 0.01),
                 links=None, **kw):
        rng = random.Random(seed)
        self.rng, self.delay, self.beacon_hz = rng, delay, beacon_hz
        self.ship = LocalClock()
        self.ids = [f"d{i}" for i in range(n)]
        self.clocks = {d: LocalClock(rng.uniform(-drift_ppm, drift_ppm), rng.uniform(-offset_s, offset_s))
                       for d in self.ids}
        self.sync = {d: Consensus(node_id=d, **kw) for d in self.ids}
        self.phase = {d: rng.uniform(0, 1 / beacon_hz) for d in self.ids}
        self.links = links or (lambda t: None)       # None = everyone hears everyone
        self.t = 0.0

    def hears(self, a, b, t):
        l = self.links(t)
        return l is None or (a, b) in l or (b, a) in l

    def run(self, until, dt=0.05):
        while self.t < until - 1e-9:
            t = self.t
            if int((t + dt) * 1.0) > int(t * 1.0):                        # ship roster, 1 Hz
                t3 = self.ship.read(t)
                for d in self.ids:
                    if self.hears("ship", d, t):
                        # it echoes the drone's heartbeat sent 0.5 s ago (t1 -> t2)
                        t1 = self.clocks[d].read(t - 0.5)
                        t2 = self.ship.read(t - 0.5 + self.rng.uniform(*self.delay))
                        t4 = self.clocks[d].read(t + self.rng.uniform(*self.delay))
                        self.sync[d].on_leader(t3, t4, (t1, t2))
            period = 1 / self.beacon_hz
            for d in self.ids:
                k = self.phase[d]
                if int((t + dt - k) / period) > int((t - k) / period) and t >= k:
                    local = self.clocks[d].read(t)
                    self.sync[d].step(local)
                    b = self.sync[d].beacon(local)
                    for e in self.ids:
                        if e != d and self.hears(d, e, t):
                            rx = self.clocks[e].read(t + self.rng.uniform(*self.delay))
                            self.sync[e].on_beacon(d, b["tau"], b["alpha"], b["o"], b["anchor"], rx,
                                                   echo=b.get("echo"))
            self.t += dt
        return self

    def errors(self, t=None):
        """Each drone's error against ship time at truth t."""
        t = self.t if t is None else t
        return {d: self.sync[d].proto_time(self.clocks[d].read(t)) - self.ship.read(t) for d in self.ids}

    def spread(self, t=None):
        e = list(self.errors(t).values())
        return max(e) - min(e)


class TestConsensus(unittest.TestCase):
    def test_converges_to_ship_time(self):
        net = Net().run(90.0)
        worst = max(abs(e) for e in net.errors().values())
        self.assertLess(worst, 0.005)                        # delays are 2-10 ms, compensated
        for d in net.ids:                                    # own bounds cover the error
            b = net.sync[d].error_bound(net.clocks[d].read(net.t))
            self.assertGreaterEqual(b, abs(net.errors()[d]))

    def test_rates_are_learned(self):
        net = Net().run(120.0)
        for d in net.ids:
            virtual_rate = net.sync[d].alpha * net.clocks[d].rate      # ship seconds per truth second
            self.assertAlmostEqual(virtual_rate, 1.0, delta=5e-5)   # 2-10 ms delay noise over a 30-60 s fit

    def test_through_peers_only(self):
        # A line: ship - d0 - d1 - d2 - d3; only d0 hears the ship.
        chain = {("ship", "d0"), ("d0", "d1"), ("d1", "d2"), ("d2", "d3")}
        net = Net(n=4, links=lambda t: chain).run(240.0)
        self.assertLess(max(abs(e) for e in net.errors().values()), 0.01)

    def test_ship_lost_drones_keep_agreeing(self):
        cut_at = 120.0
        peers = {(f"d{i}", f"d{j}") for i in range(6) for j in range(6) if i != j}
        net = Net(links=lambda t: None if t < cut_at else peers)
        net.run(cut_at)
        before = max(abs(e) for e in net.errors().values())
        net.run(cut_at + 180.0)
        self.assertLess(net.spread(), 0.005)                 # still agree with each other
        after = max(abs(e) for e in net.errors().values())
        self.assertLess(after, before + 0.02)                # 3 min without the ship: rates learned, no walk
        for d in net.ids:                                    # bound grows with time only, and covers the error
            b = net.sync[d].error_bound(net.clocks[d].read(net.t))
            self.assertGreaterEqual(b, abs(net.errors()[d]))
            self.assertLess(b, 0.05)
        for d in net.ids:
            self.assertFalse(net.sync[d].hears_leader(net.clocks[d].read(net.t)))

    def test_without_ship_from_the_start_they_agree(self):
        net = Net(links=lambda t: {(f"d{i}", f"d{j}") for i in range(6) for j in range(6) if i != j}).run(120.0)
        self.assertLess(net.spread(), 0.005)

    def test_continuous_no_large_jumps_after_convergence(self):
        net = Net().run(90.0)
        d, t0 = net.ids[0], net.t
        before = net.sync[d].proto_time(net.clocks[d].read(t0))
        net.run(t0 + 2.0)
        after = net.sync[d].proto_time(net.clocks[d].read(net.t))
        self.assertAlmostEqual(after - before, net.t - t0, delta=0.002)

    def test_link_delay_is_measured_through_echoes(self):
        net = Net(delay=(0.02, 0.02)).run(60.0)
        s = net.sync["d0"]
        for k, src in s.sources.items():
            self.assertTrue(src.rtts, k)
            self.assertAlmostEqual(src.delay(), 0.02, delta=0.002)

    def test_startup_is_fast(self):
        # +-3 s offsets: stepping onto the ship's estimate, then slewing (was 60-70 s live)
        net = Net().run(15.0)
        self.assertLess(max(abs(e) for e in net.errors().values()), 0.01)
        self.assertTrue(all(net.sync[d].steps >= 1 for d in net.ids))

    def test_late_unsynced_joiner_does_not_disturb(self):
        # d5 hears nobody for 60 s, then joins hearing only peers (not the ship)
        others = {(f"d{i}", f"d{j}") for i in range(5) for j in range(5) if i != j}
        with_ship = others | {("ship", f"d{i}") for i in range(5)}
        joined = with_ship | {("d5", f"d{i}") for i in range(5)}
        net = Net(links=lambda t: with_ship if t < 60 else joined)
        net.run(60.0)
        net.run(64.0)
        synced = [abs(e) for d, e in net.errors().items() if d != "d5"]
        self.assertLess(max(synced), 0.01)                   # its +-3 s clock pulled nobody
        net.run(100.0)
        self.assertLess(abs(net.errors()["d5"]), 0.01)       # and it synced through its peers

    def test_congested_start_then_ship_lost(self):
        # Seen live: long, uneven delays at startup put rates 1% off (the swarm then ran away without
        # the ship) and left stale delay estimates that held drones 100 ms off for a minute.
        class Congested(Net):
            def run(self, until, dt=0.05):
                while self.t < until - 1e-9:
                    self.delay = (0.05, 0.3) if self.t < 20 else (0.002, 0.01)
                    super().run(self.t + dt, dt)
                return self
        peers = {(f"d{i}", f"d{j}") for i in range(8) for j in range(8) if i != j}
        net = Congested(n=8, links=lambda t: None if t < 80 or t >= 170 else peers)
        net.run(45.0)
        self.assertLess(max(abs(e) for e in net.errors().values()), 0.01)      # 25 s after the congestion
        net.run(170.0)                                                           # 90 s without the ship
        self.assertLess(max(abs(e) for e in net.errors().values()), 0.005)
        for d in net.ids:
            self.assertLess(abs(net.sync[d].alpha * net.clocks[d].rate - 1.0), 1e-4)

    def test_bad_ship_exchanges_are_filtered(self):
        # Seen at 100 drones: drones 190 ms off while claiming a 5 ms bound, after bad exchanges.
        # A negative round trip (wrong stamps) is discarded; a lopsided, delayed one loses to the
        # lowest round trip of the last 10 s.
        drone, ship = LocalClock(drift_ppm=200, offset_s=2.0), LocalClock()
        c = Consensus(node_id="d0")
        errs = []
        for k in range(60):
            t = 1.0 + k
            up, down = 0.005, 0.005
            if k in (30, 31):
                up = 0.4                                   # lopsided queueing spike
            t1, t2 = drone.read(t - 0.5), ship.read(t - 0.5 + up)
            t3, t4 = ship.read(t), drone.read(t + down)
            if k == 40:
                t4 -= 0.1                                  # corrupt stamp: negative round trip
            c.on_leader(t3, t4, (t1, t2))
            c.step(drone.read(t + 0.2))
            errs.append(c.proto_time(drone.read(t + 0.3)) - ship.read(t + 0.3))
        self.assertLess(max(abs(e) for e in errs[10:]), 0.01)
        self.assertEqual(c.invalid, 1)

    def test_anchor_hops_through_peers(self):
        chain = {("ship", "d0"), ("d0", "d1"), ("d1", "d2"), ("d2", "d3")}
        net = Net(n=4, links=lambda t: chain).run(60.0)
        self.assertEqual([net.sync[d].anchor[1] for d in net.ids], [1, 2, 3, 4])

    def test_aggregator_is_pluggable(self):
        calls = []

        def median(own, others):
            calls.append(len(others))
            vals = sorted([own, *others])
            return vals[len(vals) // 2]
        net = Net(aggregate=median).run(60.0)
        self.assertTrue(calls)
        self.assertLess(max(abs(e) for e in net.errors().values()), 0.05)
        self.assertIn("mean", AGGREGATORS)

    def test_make_from_env(self):
        c = make(env={"CLOCK_SYNC": "consensus", "CLOCK_GAIN": "0.3", "CLOCK_LEADER_SHARE": "0.7",
                      "CLOCK_BEACON_HZ": "1"}, node_id="drone_4")
        self.assertEqual((c.mode, c.gain, c.leader_share, c.stale_s, c.node_id),
                         ("consensus", 0.3, 0.7, 3.0, "drone_4"))


if __name__ == "__main__":
    unittest.main()
