"""Cooperative localization (src/agent/coop.py) on a small simulated swarm."""
import math
import random
import unittest

import numpy as np

from coop import LOST_HOPS, Coop, state_payload
from localization import Localizer, nees

ANCHORS = [(15.0, 5.0, 8.0), (15.0, -5.0, 8.0), (-15.0, 5.0, 8.0), (-15.0, -5.0, 8.0)]


def swarm(seconds=90.0, anchor_range=150.0, use_peers=True, sigma=0.1, seed=2, jam=None):
    """Drones circling at various ranges; the last one is beyond anchor range (200 m).
    Returns {id: (errors, nees)} and the coop objects."""
    rng = random.Random(seed)
    dt = 0.02
    centres = {"d1": (80, 0), "d2": (110, 40), "d3": (120, -40), "d4": (140, 10), "far": (200, 0)}
    z = {k: 15.0 + 3 * i for i, k in enumerate(centres)}
    truth = {k: np.array(c, float) for k, c in centres.items()}
    vel = {k: np.zeros(2) for k in centres}
    loc, coop = {}, {}
    for k, c in centres.items():
        loc[k] = Localizer(range_sigma=sigma)
        loc[k].init_prior(c[0] + rng.gauss(0, 2), c[1] + rng.gauss(0, 2), sigma=3.0)
        coop[k] = Coop(k)
    out = {k: ([], []) for k in centres}
    t, nxt = 0.0, 0.0
    while t < seconds:
        t += dt
        for k, c in centres.items():
            # slow circles of 10 m radius around the station
            w = 0.2
            target = np.array(c) + 10 * np.array([math.cos(w * t), math.sin(w * t)])
            cmd = (target - truth[k]) * 0.8
            vel[k] = vel[k] + 0.35 * (cmd - vel[k]) + np.array([rng.gauss(0, 0.1), rng.gauss(0, 0.1)]) * dt
            truth[k] = truth[k] + vel[k] * dt
            loc[k].predict(dt, (float(cmd[0]), float(cmd[1])))
        if t >= nxt:
            nxt += 0.5
            for k in centres:                         # anchors
                p3 = (truth[k][0], truth[k][1], z[k])
                if jam and jam[0] <= t < jam[1]:
                    continue
                for a in ANCHORS:
                    d = math.dist(p3, a)
                    if d <= anchor_range and loc[k].update_range(a, d + rng.gauss(0, sigma), z[k], t=t):
                        coop[k].on_anchor_update(t)
            payloads = {k: state_payload(loc[k], z[k], t, coop[k].hops(t), coop[k].anchor_t()) for k in centres}
            if use_peers:
                for k in centres:                     # peer ranges, each with the peer's payload
                    ref = coop[k].anchor_t()          # our chain before this round
                    for j in centres:
                        if j == k:
                            continue
                        coop[k].on_payload(j, payloads[j])
                        r = math.dist((truth[k][0], truth[k][1], z[k]), (truth[j][0], truth[j][1], z[j]))
                        coop[k].peer_update(loc[k], j, r + rng.gauss(0, sigma), z[k], t, ref_anchor_t=ref)
            if t > 15:
                for k in centres:
                    e = np.array(loc[k].position()) - truth[k]
                    out[k][0].append(float(np.linalg.norm(e)))
                    out[k][1].append(nees(e, loc[k].pos_cov()))
    return out, coop


class TestCoop(unittest.TestCase):
    def test_far_drone_localized_through_anchored_peers(self):
        out, coop = swarm()
        errs, ns = out["far"]
        self.assertLess(max(errs), 3.0)
        self.assertEqual(coop["far"].hops(90.0), 1)
        self.assertGreater(coop["far"].peer_updates, 50)
        self.assertLess(sum(ns) / len(ns), 4.0)              # consistent (2 ideal), not overconfident
        for k in ("d1", "d2", "d3", "d4"):                   # anchored drones ignore peers
            self.assertEqual(coop[k].hops(90.0), 0)
            self.assertEqual(coop[k].peer_updates, 0)

    def test_without_peers_the_far_drone_drifts(self):
        with_p, _ = swarm()
        without, _ = swarm(use_peers=False)
        self.assertGreater(max(without["far"][0]), 2 * max(with_p["far"][0]))

    def test_all_anchors_jammed_nobody_is_anchored(self):
        out, coop = swarm(jam=(40.0, 91.0))
        for k in coop:
            self.assertEqual(coop[k].hops(90.0), LOST_HOPS)  # no chain to the ship: absolute error unbounded
            self.assertLess(coop[k].anchor_t(), 40.5)         # and no loop keeps a stale chain "fresh"

    def test_peer_selection(self):
        c = Coop("me", max_peers=3)
        for i, (x, y) in enumerate([(5, 0), (50, 0), (10, 0), (100, 0)]):
            c.on_payload(f"p{i}", {"x": x, "y": y, "vx": 0, "vy": 0, "cov": [1, 0, 1], "z": 20, "t": 0.0, "hops": 0})
        chosen = c.choose_peers((0.0, 0.0), 0.0, ["me", "p0", "p1", "p2", "p3", "new"])
        self.assertEqual(chosen[:2], ["p0", "p2"])               # the two nearest ...
        self.assertEqual(len(chosen), 3)                         # ... plus one rotating slot
        self.assertNotIn("me", chosen)


if __name__ == "__main__":
    unittest.main()
