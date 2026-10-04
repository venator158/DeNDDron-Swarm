"""Recursive decentralized localization (src/agent/rdl.py) on a small simulated swarm.

The exchange runs as it does live: every 0.5 s round each drone ranges the anchors in reach, then its
chosen peers using the payloads they sent the round before (0.5 s old), and replies reach their
peer one round later; `loss` drops payload ranges and replies independently.
"""
import math
import random
import unittest

import numpy as np

from coop import Coop, state_payload
from localization import Localizer, nees
from rdl import RDL

ANCHORS = [(15.0, 5.0, 8.0), (15.0, -5.0, 8.0), (-15.0, 5.0, 8.0), (-15.0, -5.0, 8.0)]
CENTRES = {"d1": (80, 0), "d2": (110, 40), "d3": (120, -40), "d4": (140, 10), "far": (200, 0)}


def swarm(mode="rdl", seconds=90.0, anchor_range=150.0, sigma=0.1, seed=2, jam=None, loss=0.0,
          centres=CENTRES, radius=10.0, imu=None):
    """Drones circling their stations; with anchor_range 150 m the last one (200 m) reaches no anchor.
    mode: rdl | coop | none.  Returns ({id: (errors, nees)}, {pair: relative errors}, filters)."""
    rng = random.Random(seed)
    dt = 0.02
    ids = list(centres)
    z = {k: 15.0 + 3 * i for i, k in enumerate(ids)}
    truth = {k: np.array(c, float) for k, c in centres.items()}
    vel = {k: np.zeros(2) for k in ids}
    loc, rdl, coop = {}, {}, {}
    bias = {}
    for k, c in centres.items():
        loc[k] = Localizer(range_sigma=sigma, imu=imu)
        loc[k].init_prior(c[0] + rng.gauss(0, 2), c[1] + rng.gauss(0, 2), sigma=3.0)
        if imu is not None:                             # the flight controller's accelerometer + tilt bias
            bias[k] = np.array([rng.gauss(0, imu["tilt_bias"]), rng.gauss(0, imu["tilt_bias"])])
        if mode == "rdl":
            rdl[k] = RDL(k, loc[k], imu=imu is not None)
        elif mode == "coop":
            coop[k] = Coop(k)
    out = {k: ([], []) for k in ids}
    rel = {}
    payloads, inbox = {}, {k: [] for k in ids}
    t, nxt = 0.0, 0.0
    while t < seconds:
        t += dt
        for k, c in centres.items():
            w = 0.2
            target = np.array(c) + radius * np.array([math.cos(w * t), math.sin(w * t)])
            cmd = (target - truth[k]) * 0.8
            v_old = vel[k]
            vel[k] = vel[k] + 0.35 * (cmd - vel[k]) + np.array([rng.gauss(0, 0.1), rng.gauss(0, 0.1)]) * dt
            if imu is None:
                truth[k] = truth[k] + vel[k] * dt
                loc[k].predict(dt, (float(cmd[0]), float(cmd[1])))
            else:
                u = rng.random()                        # the velocity changes somewhere in the frame
                truth[k] = truth[k] + ((1 - u) * v_old + u * vel[k]) * dt
                dv = (vel[k] - v_old) + bias[k] * dt + np.array([rng.gauss(0, imu["noise_density"] * math.sqrt(dt))
                                                                for _ in range(2)])
                loc[k].predict_imu(dt, (float(dv[0]), float(dv[1])), dt)
        if t < nxt:
            continue
        nxt += 0.5
        jammed = jam and jam[0] <= t < jam[1]
        for k in ids:                                   # anchors
            if jammed:
                continue
            p3 = (truth[k][0], truth[k][1], z[k])
            for a in ANCHORS:
                d = math.dist(p3, a)
                if d <= anchor_range and loc[k].update_range(a, d + rng.gauss(0, sigma), z[k], t=t) and coop:
                    coop[k].on_anchor_update(t)
        if mode == "rdl":
            for k in ids:                               # replies sent last round
                for sender, rep in inbox[k]:
                    rdl[k].on_reply(sender, rep)
                inbox[k] = []
            chosen = {k: [j for j in ids if j != k] for k in ids}
            if not jammed:
                for k in ids:                           # ranges with last round's payloads
                    for j in chosen[k]:
                        if j not in payloads or rng.random() < loss:
                            continue
                        rdl[k].on_payload(j, payloads[j], t)
                        r = math.dist((truth[k][0], truth[k][1], z[k]), (truth[j][0], truth[j][1], z[j]))
                        rdl[k].peer_update(j, r + rng.gauss(0, sigma), z[k], t, sigma)
            for k in ids:
                for j, rep in rdl[k].replies(t).items():
                    if rng.random() >= loss:
                        inbox[j].append((k, rep))
            payloads = {k: rdl[k].payload(t, z[k], chosen[k]) for k in ids}
        elif mode == "coop":
            states = {k: state_payload(loc[k], z[k], t, coop[k].hops(t), coop[k].anchor_t()) for k in ids}
            if not jammed:
                for k in ids:
                    ref = coop[k].anchor_t()
                    for j in ids:
                        if j == k or rng.random() < loss:
                            continue
                        coop[k].on_payload(j, states[j])
                        r = math.dist((truth[k][0], truth[k][1], z[k]), (truth[j][0], truth[j][1], z[j]))
                        coop[k].peer_update(loc[k], j, r + rng.gauss(0, sigma), z[k], t, ref_anchor_t=ref)
        if t > 15:
            for k in ids:
                e = np.array(loc[k].position()) - truth[k]
                out[k][0].append(float(np.linalg.norm(e)))
                out[k][1].append(nees(e, loc[k].pos_cov()))
            for i, a in enumerate(ids):
                for b in ids[i + 1:]:
                    est = np.array(loc[a].position()) - np.array(loc[b].position())
                    rel.setdefault((a, b), []).append(float(np.linalg.norm(est - (truth[a] - truth[b]))))
    return out, rel, (rdl or coop or loc)


def mean(v):
    return sum(v) / len(v)


# The uwb_short layout as it is live: stations 50-130 m out, three drones within the 80 m anchor range
LIVE = {"d1": (60, 0), "d2": (55, 45), "d3": (50, -50), "d4": (110, 10), "far": (130, -30)}


class TestExchange(unittest.TestCase):
    def test_one_exchange_matches_the_centralized_update(self):
        # Two drones, one range: the factors must give exactly the joint posterior a central filter computes.
        a, b = Localizer(), Localizer()
        a.init_prior(80.0, 0.0, sigma=3.0)
        b.init_prior(110.0, 40.0, sigma=3.0)
        ra, rb = RDL("a", a), RDL("b", b)
        pb = rb.payload(0.0, 18.0, ["a"])               # b's state as sent
        Pa, Pb, xa, xb = a.P.copy(), b.P.copy(), a.x.copy(), b.x.copy()
        ra.on_payload("b", pb, 0.0)
        r = math.dist((81.0, 1.0, 15.0), (108.0, 41.0, 18.0))
        self.assertTrue(ra.peer_update("b", r, 15.0, 0.0, 0.1))
        rb.on_reply("a", ra.replies(0.0)["b"])
        # central EKF on [a; b]
        d = np.array([xa[0] - xb[0], xa[1] - xb[1], 15.0 - 18.0])
        h = float(np.linalg.norm(d))
        H = np.zeros((1, 12))
        H[0, :2], H[0, 6:8] = d[:2] / h, -d[:2] / h
        J = np.zeros((12, 12))
        J[:6, :6], J[6:, 6:] = Pa, Pb
        S = float((H @ J @ H.T)[0, 0]) + 0.01
        K = J @ H.T / S
        Jp = (np.eye(12) - K @ H) @ J @ (np.eye(12) - K @ H).T + K @ K.T * 0.01
        X = np.concatenate([xa, xb]) + K[:, 0] * (r - h)
        np.testing.assert_allclose(a.x, X[:6], atol=1e-9)
        np.testing.assert_allclose(b.x, X[6:], atol=1e-9)
        np.testing.assert_allclose(a.P, Jp[:6, :6], atol=1e-9)
        np.testing.assert_allclose(b.P, Jp[6:, 6:], atol=1e-9)
        np.testing.assert_allclose(ra.factors["b"] @ rb.factors["a"].T, Jp[:6, 6:], atol=1e-9)
        self.assertEqual(rb.acks, {"a": 1})

    def test_a_reply_is_applied_once(self):
        a, b = Localizer(), Localizer()
        a.init_prior(80.0, 0.0, sigma=3.0)
        b.init_prior(110.0, 40.0, sigma=3.0)
        ra, rb = RDL("a", a), RDL("b", b)
        ra.on_payload("b", rb.payload(0.0, 18.0, []), 0.0)
        ra.peer_update("b", 50.0, 15.0, 0.0, 0.1)
        rep = ra.replies(0.0)["b"]
        self.assertTrue(rb.on_reply("a", rep))
        self.assertFalse(rb.on_reply("a", rep))         # repeats (the radio is lossy) are ignored


class TestSwarm(unittest.TestCase):
    def test_far_drone_consistent_through_peers(self):
        out, _, filt = swarm()
        errs, ns = out["far"]
        self.assertLess(max(errs), 2.0)
        self.assertLess(mean(ns), 4.0)
        for k, (e, n) in out.items():
            self.assertLess(mean(n), 4.0, k)
        coop, _, _ = swarm(mode="coop")
        self.assertLess(max(max(e) for e, _ in out.values()), max(max(e) for e, _ in coop.values()))

    def test_short_anchor_range_live_layout(self):
        for loss in (0.0, 0.3):
            out, _, _ = swarm(anchor_range=80.0, centres=LIVE, loss=loss, seed=3)
            for k, (e, n) in out.items():
                self.assertLess(max(e), 2.0, (k, loss))
                self.assertLess(mean(n), 4.0, (k, loss))

    def test_all_anchors_jammed_relative_positions_stay_tight(self):
        out, rel, _ = swarm(jam=(40.0, 91.0))
        _, rel_coop, _ = swarm(mode="coop", jam=(40.0, 91.0))
        self.assertLess(max(max(v) for v in rel.values()), max(max(v) for v in rel_coop.values()))
        for k, (e, n) in out.items():
            self.assertLess(mean(n), 4.0, k)            # absolute error grows, honestly

    def test_exchange_loss(self):
        out, _, filt = swarm(loss=0.3)
        self.assertGreater(sum(f.replies_lost for f in filt.values()) + sum(f.ci_updates for f in filt.values()), 0)
        for k, (e, n) in out.items():
            self.assertLess(max(e), 2.0, k)
            self.assertLess(mean(n), 4.0, k)


if __name__ == "__main__":
    unittest.main()
