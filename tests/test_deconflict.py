import math
import unittest

from deconflict import (BLAST_TOL_S, Blast, Reservation, blast_radius, blast_separation, choose_intercept,
                        plan_route, progress, stop_distance, travel_time)
from threats import eta, position_at

V, A = 4.0, 1.0


def fly(route, start, t0, t_end, step=0.05):
    """Positions (t, p) along a route: fly each leg rest-to-rest, hold until its release."""
    out, pos, t = [], tuple(start), t0
    for leg in route.legs:
        d = math.dist(pos, leg.target)
        dur = eta(d, V, A)
        u = [(leg.target[i] - pos[i]) / d for i in range(3)] if d > 1e-9 else [0.0, 0.0, 0.0]
        tau = 0.0
        while tau < dur:
            s = progress(d, V, A, tau)
            out.append((t + tau, tuple(pos[i] + u[i] * s for i in range(3))))
            tau += step
        t, pos = t + dur, leg.target
        if leg.release is not None:
            while t < leg.release:
                out.append((t, pos))
                t += step
    while t <= t_end:
        out.append((t, pos))
        t += step
    return out


def inside_during_window(samples, blast):
    lo, hi = blast.window()
    return [t for t, p in samples if lo <= t <= hi and math.dist(p, blast.center) < blast.radius]


class TestProgress(unittest.TestCase):
    def test_endpoints_and_monotonic(self):
        for d in (2.0, 16.0, 60.0):          # triangular and trapezoidal profiles
            T = eta(d, V, A)
            self.assertEqual(progress(d, V, A, 0.0), 0.0)
            self.assertAlmostEqual(progress(d, V, A, T), d)
            self.assertAlmostEqual(progress(d, V, A, T / 2), d / 2, places=6)   # symmetric
            xs = [progress(d, V, A, T * k / 50) for k in range(51)]
            self.assertEqual(xs, sorted(xs))


class TestMovingStart(unittest.TestCase):
    def test_from_rest_matches_eta(self):
        for d in (2.0, 16.0, 60.0):
            self.assertAlmostEqual(travel_time(d, V, A), eta(d, V, A))

    def test_moving_start_is_faster_by_the_acceleration_saved(self):
        # Cruising at 4 m/s over a long leg: no acceleration phase, 2 s sooner than from rest.
        self.assertAlmostEqual(eta(60, V, A) - travel_time(60, V, A, v0=4.0), 2.0)

    def test_progress_consistent_with_moving_start(self):
        for d, v0 in ((60.0, 4.0), (60.0, 2.0), (10.0, 3.0), (5.0, 4.0)):   # incl. braking-only
            T = travel_time(d, V, A, v0)
            self.assertAlmostEqual(progress(d, V, A, T, v0), d, places=6)
            xs = [progress(d, V, A, T * k / 60, v0) for k in range(61)]
            self.assertEqual(xs, sorted(xs))
            self.assertAlmostEqual(xs[1], v0 * T / 60, delta=0.05 * d)   # starts at v0

    def test_fast_drone_cannot_hold_short_of_a_near_blast(self):
        # Blast sphere starts 6 m ahead; at 4 m/s the stopping distance is 8 m.
        self.assertEqual(stop_distance(4.0, A), 8.0)
        b = Blast("T9", (20.0, 0.0, 20.0), 12.0, t=4.0)
        r = plan_route((0, 0, 20), (60, 0, 20), 0.0, [b], V, A, t_goal=100.0, v0=4.0)
        self.assertEqual(r.exposed, ["T9"])
        self.assertEqual(r.hold_s, 0.0)             # carries on instead of turning back into it

    def test_slow_drone_still_holds(self):
        b = Blast("T9", (30.0, 0.0, 20.0), 12.0, t=8.0)
        r = plan_route((0, 0, 20), (60, 0, 20), 0.0, [b], V, A, t_goal=100.0, v0=1.0)
        self.assertEqual(r.exposed, [])
        self.assertGreater(r.hold_s, 0.0)


class TestPlanRoute(unittest.TestCase):
    def test_no_blasts_is_straight(self):
        r = plan_route((0, 0, 20), (60, 0, 20), 10.0, [], V, A)
        self.assertEqual(len(r.legs), 1)
        self.assertAlmostEqual(r.arrival, 10.0 + eta(60, V, A))
        self.assertEqual(r.hold_s, 0.0)
        self.assertFalse(r.blocked)

    def test_holds_before_blast_on_path(self):
        # Another job detonates at x=30 while we would be flying through it.
        b = Blast("T9", (30.0, 0.0, 20.0), blast_radius(1), t=8.0)
        r = plan_route((0, 0, 20), (60, 0, 20), 0.0, [b], V, A, t_goal=100.0)
        self.assertFalse(r.blocked)
        self.assertEqual(len(r.legs), 2)
        self.assertAlmostEqual(r.legs[0].release, 8.0 + BLAST_TOL_S)
        self.assertLess(r.legs[0].target[0], 30.0 - blast_radius(1))
        self.assertGreater(r.hold_s, 0.0)
        self.assertEqual(inside_during_window(fly(r, (0, 0, 20), 0.0, 40.0), b), [])

    def test_no_hold_when_blast_is_long_after_passing(self):
        b = Blast("T9", (30.0, 0.0, 20.0), blast_radius(1), t=60.0)
        r = plan_route((0, 0, 20), (60, 0, 20), 0.0, [b], V, A, t_goal=100.0)
        self.assertEqual(len(r.legs), 1)
        self.assertEqual(r.hold_s, 0.0)

    def test_goal_inside_blast_before_own_detonation_is_blocked(self):
        b = Blast("T9", (60.0, 5.0, 20.0), blast_radius(2), t=30.0)
        r = plan_route((0, 0, 20), (60, 0, 20), 0.0, [b], V, A, t_goal=40.0)
        self.assertTrue(r.blocked)

    def test_goal_inside_blast_after_own_detonation_is_fine(self):
        b = Blast("T9", (60.0, 5.0, 20.0), blast_radius(2), t=50.0)
        r = plan_route((0, 0, 20), (60, 0, 20), 0.0, [b], V, A, t_goal=40.0)
        self.assertFalse(r.blocked)

    def test_starting_inside_a_blast_exits_first(self):
        b = Blast("T9", (0.0, 3.0, 20.0), blast_radius(1), t=10.0)
        r = plan_route((0, 0, 20), (0, -60, 20), 0.0, [b], V, A, t_goal=100.0)
        self.assertFalse(r.blocked)
        self.assertEqual(r.exposed, [])
        self.assertGreaterEqual(math.dist(r.legs[0].target, b.center), b.radius)
        self.assertEqual(inside_during_window(fly(r, (0, 0, 20), 0.0, 40.0), b), [])

    def test_too_close_to_escape_is_reported(self):
        # Heading through the centre; getting clear sideways (11 m) takes 6.6 s, the window opens at 4.5 s.
        b = Blast("T9", (0.0, 3.0, 20.0), blast_radius(1), t=4.5 + BLAST_TOL_S)   # window opens at 4.5 s
        r = plan_route((0, 0, 20), (0, 60, 20), 0.0, [b], V, A, t_goal=100.0)
        self.assertEqual(r.exposed, ["T9"])

    def test_no_exit_when_straight_path_leaves_in_time(self):
        # Inside now, but flying away: out of the sphere (9 m, 4.2 s) before the window opens (4.5 s).
        b = Blast("T9", (0.0, 3.0, 20.0), blast_radius(1), t=4.5 + BLAST_TOL_S)   # window opens at 4.5 s
        r = plan_route((0, 0, 20), (0, -60, 20), 0.0, [b], V, A, t_goal=100.0)
        self.assertEqual((r.exposed, len(r.legs), r.hold_s), ([], 1, 0.0))

    def test_two_blasts_on_path(self):
        b1 = Blast("T8", (25.0, 0.0, 20.0), blast_radius(1), t=7.0)
        b2 = Blast("T9", (55.0, 0.0, 20.0), blast_radius(1), t=22.0)
        r = plan_route((0, 0, 20), (90, 0, 20), 0.0, [b1, b2], V, A, t_goal=100.0)
        self.assertFalse(r.blocked)
        samples = fly(r, (0, 0, 20), 0.0, 60.0)
        self.assertEqual(inside_during_window(samples, b1), [])
        self.assertEqual(inside_during_window(samples, b2), [])


class TestChooseIntercept(unittest.TestCase):
    # Threat inbound along -x at 3.5 m/s from x=180, drones parked near the ship.
    p0, v = (180.0, 10.0, 18.0), (-3.5, 0.0, 0.0)

    def track(self, t):
        return position_at(self.p0, self.v, 0.0, t)

    def choose(self, drones, level=1, reservations=(), t_latest=45.0, max_range=150.0):
        return choose_intercept(self.track, 0.0, t_latest, drones, level, list(reservations), V, A,
                                max_range=max_range, z_range=(5.0, 40.0))

    def test_earliest_feasible_and_farther_than_waiting(self):
        drones = [(40.0, 0.0, 20.0), (35.0, 20.0, 20.0), (-40.0, 0.0, 20.0)]
        ic = self.choose(drones)
        self.assertIsNotNone(ic)
        # 0.5 s earlier must be infeasible for the best drone
        best = min(eta(math.dist(d, self.track(ic.t - 0.5)), V, A) * 1.25 for d in drones)
        self.assertGreater(4.0 + best, ic.t - 0.5)
        self.assertGreater(math.hypot(ic.point[0], ic.point[1]), 80.0)   # well outside the 45 m ring

    def test_level_needs_that_many_drones(self):
        drones = [(40.0, 0.0, 20.0), (-40.0, 0.0, 20.0)]
        one, two = self.choose(drones, level=1), self.choose(drones, level=2)
        self.assertIsNotNone(one)
        self.assertTrue(two is None or two.t > one.t)

    def test_keeps_away_from_other_jobs(self):
        drones = [(40.0, 0.0, 20.0), (35.0, 20.0, 20.0)]
        free = self.choose(drones)
        res = Reservation("T1", free.point, 2, free.t + 5.0)
        ic = self.choose(drones, reservations=[res])
        self.assertIsNotNone(ic)
        self.assertGreaterEqual(math.dist(ic.point, res.point), blast_separation(2, 1))

    def test_yields_to_committed_drones(self):
        drones = [(40.0, 0.0, 20.0), (35.0, 20.0, 20.0)]
        free = self.choose(drones)
        # A drone of another job flies straight through that point around that time.
        px, py, pz = free.point
        start, goal = (px, py - 100.0, pz), (px, py + 60.0, pz)     # passes the point ~25-26 s from now
        committed = [(start, goal, free.t + 40)]
        ic = choose_intercept(self.track, 0.0, 45.0, drones, 1, [], V, A, max_range=150.0, z_range=(5.0, 40.0),
                              committed=committed)
        self.assertIsNotNone(ic)
        self.assertNotAlmostEqual(ic.t, free.t)

    def test_none_when_out_of_range_or_time(self):
        self.assertIsNone(self.choose([(-150.0, 0.0, 20.0)], t_latest=10.0))
        self.assertIsNone(self.choose([(40.0, 0.0, 20.0)], max_range=5.0))


if __name__ == "__main__":
    unittest.main()
