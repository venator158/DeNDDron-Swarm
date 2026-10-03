"""Ship C2 node: radar picture, operator dashboard, roster and kill assessment.

The ship is the swarm's only sensor for threats (a deliberate single point of
failure: drones carry no long-range sensors).  It

- simulates radar tracks of incoming threats and reports each with its closest
  point of approach (CPA) and time to it (TCPA), queued in a min-heap by TCPA;
- lets the operator approve an engagement only when enough free drones can
  reach the engagement point in time: TTI(level) < time until engagement;
- sends approved engagements to the swarm, which assigns drones itself;
- keeps the roster (drones it hears heartbeats from) and publishes it;
- assesses kills from detonations observed on the onboard (physics) bus;
- aggregates instrumentation from drone telemetry and its own radio traffic;
- serves the operator dashboard over HTTP.

Links: onboard bus (sim clock, detonations, threat tracks for Gazebo) and the
peer-to-peer radio (everything to and from the drones).
"""

import argparse
import json
import logging
import math
import os
import random
import signal
import threading
import time
from collections import defaultdict, deque
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path

from links import open_onboard
import localclock
import simclock
from radio_process import RadioProcess
from deconflict import Reservation, blast_radius, choose_intercept, plan_route
from jobs import arbitrate
from threat_queue import ThreatQueue
from threats import (DEFAULT_THREAT_TYPES, ORDER_SLACK_S, Threat, aim_velocity, closest_point_of_approach,
                     engagement_point, parse_threat_types, position_at)

logging.basicConfig(level=logging.INFO, format="%(asctime)s [Ship] %(message)s", datefmt="%H:%M:%S")
log = logging.getLogger("Ship")

HERE = Path(__file__).resolve().parent
MEMBER_TIMEOUT_S = 3.0     # seconds (simclock) without a heartbeat before a drone leaves the roster
ROSTER_HZ = 1.0
RETRY_AFTER_S = 3.0        # seconds (simclock) before an under-assigned engagement is re-announced
MAX_ANNOUNCES = 3
SHIP_RADIUS = 16.0
ACK_WINDOW_S = 0.3         # collect competing awards for a threat this long before confirming
JOB_HZ = 2.0               # job topic updates per (sim) second
JOB_CLOSE_REPEATS = 3      # "closed" job updates sent after a threat is resolved (lossy radio)
HB_MISMATCH_S = 2.0        # a confirmed drone whose heartbeat shows another job this long is dropped
INTERCEPT_REPLAN_S = 0.5   # intercept search results are reused this long (dashboard polls several times/s)
# Slack before a re-planned intercept (after a manoeuvre) for the job's own drones.  A new order needs
# INTERCEPT_SLACK_S (4 s: order, bids, confirmation); drones already on the job only need the job update.
# With 4 s, a late turn (threat ~100 m out, drones on station) pushed 1 re-plan in 8 back to the legacy
# point (45 m) and 1 in 4 inside 60 m; with 1 s, 0 and ~1 in 70 (200 simulated turns).
MANEUVER_REPLAN_SLACK_S = 1.0


def _env(name, default, cast):
    return cast(os.environ.get(name, default))


def _mean(xs):
    xs = [x for x in xs if x is not None]
    return round(sum(xs) / len(xs), 1) if xs else None


def _stats(xs, nd=3):
    """(mean, sd, max |x|) of the non-None values, or Nones."""
    xs = [x for x in xs if x is not None]
    if not xs:
        return None, None, None
    m = sum(xs) / len(xs)
    sd = math.sqrt(sum((x - m) ** 2 for x in xs) / (len(xs) - 1)) if len(xs) > 1 else 0.0
    return round(m, nd), round(sd, nd), round(max(abs(x) for x in xs), nd)


def _xyz(p):
    return {"x": round(p[0], 2), "y": round(p[1], 2), "z": round(p[2], 2)}


class Track:
    """One radar track plus its engagement bookkeeping.

    The threat moves in truth (true_p0, true_v, true_t0): Gazebo markers and kill assessment use
    that.  The radar measures it in ship time, so C2 sees p0, v, t0 (and CPA, engagement point)
    in the ship's timebase: t0 = clock.read(true_t0), v = true_v / clock.rate.  The positions are
    the same; only the timebase differs, and with a perfect ship clock the two are identical.
    """

    def __init__(self, threat_id, kind, level, p0, v, t0_truth, defended_radius, clock):
        self.threat_id, self.type, self.level = threat_id, kind, level
        self.defended_radius = defended_radius
        self.clock = clock
        self.retrack(p0, v, t0_truth)
        self.status = "tracking"        # tracking | approved | destroyed | leaked | impact | failed
        self.confirmed = {}             # drone -> slot: engagements the ship has ACKed (the job's holders)
        self.pending = {}               # drone -> (bid cost, order id): awards awaiting arbitration
        self.pending_since = None
        self.rejected = set()           # (drone, order id) refused: repeats of that award get a NACK again
        self.mismatch = {}              # confirmed drone -> simclock time its heartbeat started disagreeing
        self.job_seq = 0
        self.close_repeats = 0
        self.maneuver_at = None         # sim time of the planned heading change, if any
        self.maneuvers = 0
        self.intercept_range = None     # distance from the ship of the chosen intercept point
        self.legacy_range = None        # ... of the legacy (defended radius / CPA) point, for comparison
        self.holds = {}                 # drone -> planned hold time (s) reported with its award
        self.detonated = {}             # drone -> miss distance (m)
        self.friendly_fire = []         # drones destroyed by this threat's detonations
        self.det_eval = []              # per detonation: timing evaluation against truth (_on_detonation)
        self.intruded = 0               # detonations with non-job drones within CLEARANCE_M
        self.hits = set()               # detonations within kill radius
        self.missed = set()             # drones that aborted
        self.no_detection = set()       # drones whose fuze window closed with no detection (hold fallback)
        self.announces = 0
        self.orders = []                # order (wave) ids sent for this threat
        self.last_announce = None
        self.approved_wall = None       # simclock time of approval
        self.award_events = set()       # (drone, status, order) already logged: awards are sent several times
        self.first_award_ms = None
        self.full_award_ms = None

    def retrack(self, p0, v, t0_truth):
        """New track segment (detection or manoeuvre, in truth): recompute the ship-time view, CPA
        and the fallback engagement point.

        The legacy point (defended radius crossing, or CPA) is the latest acceptable intercept; the
        ship normally engages earlier and farther out (choose_intercept) and overwrites point/t_engage.
        """
        self.true_p0, self.true_v, self.true_t0 = p0, v, t0_truth
        r = self.clock.rate
        self.p0, self.v, self.t0 = p0, (v[0] / r, v[1] / r, v[2] / r), self.clock.read(t0_truth)
        self.cpa, self.t_cpa = closest_point_of_approach(self.p0, self.v, self.t0)
        self.cpa_dist = math.hypot(self.cpa[0], self.cpa[1])
        self.legacy_point, self.legacy_t = engagement_point(self.p0, self.v, self.t0, self.defended_radius)
        self.point, self.t_engage = self.legacy_point, self.legacy_t
        self.plan_cache = None          # (ship time, Intercept or None)

    def position(self, t):
        """Position at ship time t."""
        return position_at(self.p0, self.v, self.t0, t)

    def true_position(self, t_truth):
        return position_at(self.true_p0, self.true_v, self.true_t0, t_truth)

    def true_closest_approach(self, p):
        """(truth time, distance) of the threat's true closest approach to point p (this segment)."""
        v, d = self.true_v, [p[i] - self.true_p0[i] for i in range(3)]
        vv = v[0] * v[0] + v[1] * v[1] + v[2] * v[2]
        t = self.true_t0 + (0.0 if vv < 1e-12 else (d[0] * v[0] + d[1] * v[1] + d[2] * v[2]) / vv)
        return t, math.dist(self.true_position(t), p)

    @property
    def holders(self):
        return set(self.confirmed)

    @property
    def active(self):
        return self.status in ("tracking", "approved")

    def track_msg(self, truth=False):
        """The track as C2 knows it (ship time), or in truth for the simulator's markers."""
        p0, v, t0 = (self.true_p0, self.true_v, self.true_t0) if truth else (self.p0, self.v, self.t0)
        return {"threat_id": self.threat_id, "type": self.type, "level": self.level,
                "status": "active" if self.active else self.status, "t0": t0,
                "p0": _xyz(p0), "v": _xyz(v)}


class Ship:
    def __init__(self, args):
        self.args = args
        self.types = parse_threat_types(args.threat_types) if args.threat_types else DEFAULT_THREAT_TYPES
        self.rng = random.Random(args.seed)
        self.maneuver_rng = random.Random(args.seed + 1)   # separate, so the scenario is unchanged
        self.lock = threading.RLock()

        # Truth (sim/clock) is for the radar's world and evaluation; C2 runs on ship_time, the ship's
        # own clock (SHIP_CLOCK_*, perfect by default), which is the protocol's timebase.
        self.clock = localclock.from_env("ship", prefix="SHIP_CLOCK_")
        # Sync mode (timesync.py): ttg stamps orders/jobs/zones with their send time; master echoes
        # each drone's heartbeat stamp (t1) with receive/send stamps (t2, t3) in the roster.
        self.sync_mode = (os.environ.get("CLOCK_SYNC") or "none").lower()
        self.sync_evals = []             # (drone, |true error| s, own bound s) from clock telemetry
        self.metrics = {}                # latest metrics-node summary (true separations)
        # Auto-approve (dashboard toggle, /api/auto_approve, AUTO_APPROVE): the ship approves feasible
        # threats itself, most urgent first, once each has been feasible for a reaction delay.
        self.auto_approve = (os.environ.get("AUTO_APPROVE") or "0").strip().lower() in ("1", "on", "true", "yes")
        self.auto_reaction_s = float(os.environ.get("AUTO_APPROVE_REACTION_S") or 3.0)
        self._feasible_since = {}        # threat_id -> ship time it became (continuously) feasible
        self.truth_time = None
        self.ship_time = None
        self.tracks = {}                 # threat_id -> Track
        self.queue = ThreatQueue()       # active tracks by t_cpa
        self.next_detection = None
        self.detected = 0
        self.order_seq = 0

        self.drones = {}                 # id -> {"hb": heartbeat, "seen": wall, "telemetry": {...}}
        self.expended = set()
        self.events = deque(maxlen=200)
        self.rx_counts = defaultdict(int)
        self.rx_bytes = defaultdict(int)
        self.rx_rates = {}
        self._rx_window_start = simclock.now()
        self.log_file = open(args.log_path, "a", buffering=1) if args.log_path else None

        v_max, a_max = self._kinematics()
        self.v_max, self.a_max = v_max, a_max
        # Ship no-fly zone (NO_FLY_RADIUS_M, 0: off): intercept points stay 5 m outside it, the
        # fallback point moves out to that radius, and routes go around it (3 m margin).
        self.no_fly = float(args.no_fly or 0.0)
        self.engage_radius = max(args.defended_radius, self.no_fly + 5.0) if self.no_fly > 0 else args.defended_radius
        self.no_fly_route = self.no_fly + 3.0 if self.no_fly > 0 else None

        self.radio = RadioProcess()          # own process: a radio failure must not freeze C2
        self.onboard = open_onboard(args.sim_bus)
        self.pub_tracks = self.onboard.declare_publisher("sim/threat_tracks")
        self.pub_orders = self.radio.declare_publisher("swarm/threats")
        self.pub_roster = self.radio.declare_publisher("ship/roster")
        self.pub_status = self.radio.declare_publisher("ship/threat_status")
        self.pub_zones = self.radio.declare_publisher("ship/zones")
        self.subs = [
            self.onboard.declare_subscriber("sim/clock", self._on_clock),
            self.onboard.declare_subscriber("sim/detonation", self._on_detonation),
            self.onboard.declare_subscriber("sim/damage", self._on_damage),
            self.onboard.declare_subscriber("sim/clock_eval", self._on_clock_eval),
            self.onboard.declare_subscriber("swarm/metrics/summary", self._on_metrics),
            self.radio.declare_subscriber("swarm/heartbeat/*", self._on_heartbeat),
            self.radio.declare_subscriber("swarm/heartbeat_relay/*", self._on_relayed_heartbeat),
            self.radio.declare_subscriber("swarm/telemetry/*", self._on_telemetry),
            self.radio.declare_subscriber("swarm/awards", self._on_award),
            self.radio.declare_subscriber("swarm/bids", self._on_bid),
        ]

    def _kinematics(self):
        try:
            cfg = json.loads(Path(self.args.runtime_config).read_text())["defaults"]["kinematics"]
            self.z_range = (float(cfg.get("min_z", 5.0)), float(cfg.get("max_z", 45.0)))
            return float(cfg["max_velocity"]), float(cfg["max_acceleration"])
        except Exception:
            self.z_range = (5.0, 45.0)
            return 4.0, 1.0

    # ------------------------------------------------------------------ util
    def event(self, kind, **fields):
        e = {"wall": time.strftime("%H:%M:%S"), "sim_time": self.truth_time, "ship_time": self.ship_time,
             "kind": kind, **fields}
        self.events.append(e)
        if self.log_file:
            self.log_file.write(json.dumps(e) + "\n")
        log.info("%s %s", kind, " ".join(f"{k}={v}" for k, v in fields.items()))

    def log_only(self, kind, **fields):
        """Evaluation record: to the log file, not the dashboard's event list."""
        if self.log_file:
            self.log_file.write(json.dumps({"sim_time": self.truth_time, "ship_time": self.ship_time,
                                            "kind": kind, **fields}) + "\n")

    def _stamp(self):
        """Ship-clock timestamp for sync exchanges: extrapolated between sim/clock updates (10 Hz),
        with the clock's jitter."""
        t = simclock.truth_now()
        return self.clock.stamp(t if t is not None else (self.truth_time or 0.0))

    def _stamped(self, msg):
        """With ttg, stamp a message carrying absolute times with its send time."""
        if self.sync_mode == "ttg":
            msg["sent"] = round(self._stamp(), 6)
        return msg

    def _count_rx(self, topic, sample):
        self.rx_counts[topic] += 1
        self.rx_bytes[topic] += len(bytes(sample.payload))

    @staticmethod
    def _parse(sample):
        return json.loads(bytes(sample.payload).decode("utf-8"))

    # ------------------------------------------------------------- callbacks
    def _on_clock(self, sample):
        truth = float(self._parse(sample)["sim_time"])
        with self.lock:
            self.truth_time, self.ship_time = truth, self.clock.read(truth)
        simclock.observe(truth)

    def _on_heartbeat(self, sample):
        t2 = self._stamp()
        self._count_rx("swarm/heartbeat", sample)
        hb = self._parse(sample)
        with self.lock:
            d = self.drones.setdefault(hb["agent_id"], {"telemetry": {}})
            d["hb"], d["seen"], d["relayed"] = hb, simclock.now(), False
            if hb.get("t1") is not None:
                d["sync"] = (hb["t1"], t2)
            if hb.get("state") == "expended":
                self.expended.add(hb["agent_id"])

    def _on_relayed_heartbeat(self, sample):
        """A peer forwarded the heartbeat of a drone we could not hear directly."""
        self._count_rx("swarm/heartbeat_relay", sample)
        hb = self._parse(sample)
        with self.lock:
            d = self.drones.setdefault(hb["agent_id"], {"telemetry": {}})
            if "seen" in d and simclock.now() - d["seen"] < 1.0 and not d.get("relayed"):
                return   # heard directly anyway
            d["hb"], d["seen"], d["relayed"] = hb, simclock.now(), True
            if hb.get("t1") is not None:
                # The relaying drone held it for relay_resid (transparent clock): take that off t2.
                d["sync"] = (hb["t1"], self._stamp() - float(hb.get("relay_resid") or 0.0))

    def _on_telemetry(self, sample):
        self._count_rx("swarm/telemetry", sample)
        t = self._parse(sample)
        with self.lock:
            self.drones.setdefault(t["agent_id"], {"telemetry": {}})["telemetry"] = t

    def _on_metrics(self, sample):
        """The metrics node's summary (true positions): collisions, close calls, minimum separation."""
        m = self._parse(sample)
        with self.lock:
            self.metrics = {k: v for k, v in m.items()
                            if k in ("total_collisions", "close_calls", "min_separation_m", "min_ship_range_m")
                            or k.startswith("loc_")}

    def _on_clock_eval(self, sample):
        """True sync error (evaluation only, onboard bus): the drone's estimate of ship time against
        the ship's clock at the same truth instant.  Measured during radio cuts too."""
        c = self._parse(sample)
        if c.get("truth") is None or c.get("ship_est") is None:
            return
        err = float(c["ship_est"]) - self.clock.read(float(c["truth"]))
        bound = c.get("bound")
        with self.lock:
            d = self.drones.setdefault(c["agent_id"], {"telemetry": {}})
            d["clock_err_ms"] = round(err * 1000.0, 2)
            d["clock_bound_ms"] = None if bound is None else round(bound * 1000.0, 2)
            self.log_only("clock_eval", drone=c["agent_id"], mode=c.get("mode"), err_s=round(err, 6),
                          bound_s=bound, offset_s=c.get("offset"), truth=c["truth"], hops=c.get("hops"),
                          ship=c.get("ship"), **{k: c[k] for k in ("exch", "rtt_min", "samples", "rejected",
                                                                   "invalid", "relocks", "steps", "radio_q")
                                                          if k in c})
            if c["agent_id"] not in self.expended:
                self.sync_evals.append((c["agent_id"], abs(err), bound))

    def _on_bid(self, sample):
        self._count_rx("swarm/bids", sample)

    def _on_award(self, sample):
        self._count_rx("swarm/awards", sample)
        a = self._parse(sample)
        with self.lock:
            tr = self.tracks.get(a["threat_id"])
            if tr is None:
                return
            agent, status, wave = a["agent_id"], a.get("status", "engaged"), a.get("wave_id")
            first = (agent, status, wave) not in tr.award_events
            tr.award_events.add((agent, status, wave))
            if status == "engaged":
                if agent in tr.detonated or agent in self.expended:
                    return   # late repeat of an award already acted on
                if agent in tr.confirmed:
                    self._send_ack(tr, agent, True, wave)     # repeat: the ACK may have been lost
                    return
                if (agent, wave) in tr.rejected or tr.status != "approved":
                    self._send_ack(tr, agent, False, wave)
                    return
                if not tr.pending:
                    tr.pending_since = simclock.now()
                tr.pending[agent] = (float(a.get("cost") or 0.0), wave)
                tr.holds[agent] = float(a.get("hold_s") or 0.0)
                if tr.approved_wall is not None and tr.first_award_ms is None:
                    tr.first_award_ms = round((simclock.now() - tr.approved_wall) * 1000)
                if first:
                    self.event("award", threat=tr.threat_id, drone=agent, cost=a.get("cost"), order=wave)
            else:
                tr.confirmed.pop(agent, None)
                tr.pending.pop(agent, None)
                tr.mismatch.pop(agent, None)
                if status == "missed":
                    tr.missed.add(agent)
                elif status == "no_detection":
                    tr.no_detection.add(agent)
                if first:
                    self.event(status, threat=tr.threat_id, drone=agent)

    # ----------------------------------------------------------------- jobs
    def _send_ack(self, tr, agent, accepted, wave=None):
        msg = {"threat_id": tr.threat_id, "agent_id": agent, "wave_id": wave, "accepted": accepted,
               "job": f"ship/jobs/{tr.threat_id}"}
        if accepted:
            msg.update(self._job_msg(tr))
        self.radio.put(f"ship/ack/{agent}", json.dumps(self._stamped(msg)))

    def _job_msg(self, tr):
        return {"threat_id": tr.threat_id, "status": "active" if tr.active else tr.status, "seq": tr.job_seq,
                "type": tr.type, "level": tr.level, "point": _xyz(tr.point), "t_engage": tr.t_engage,
                "cpa": _xyz(tr.cpa), "t_cpa": tr.t_cpa, "track": tr.track_msg(),
                "holders": dict(tr.confirmed), "n_slots": tr.level, "intruders": self._intruders(tr)}

    @staticmethod
    def zone_radius(tr):
        """Keep-out radius around a job's engagement point: clearance from every slot."""
        return blast_radius(tr.level)

    def _reservations(self, exclude=None):
        return [Reservation(t.threat_id, t.point, t.level, t.t_engage) for t in self.tracks.values()
                if t.status == "approved" and t.threat_id != exclude]

    def _pose(self, drone):
        p = self.drones.get(drone, {}).get("hb", {}).get("pose")
        return (p["x"], p["y"], p["z"]) if p else None

    def plan_intercept(self, tr, now, drones=None, level=None, slack=None):
        """Earliest reachable intercept for this track (cached briefly for the free-drone case).
        slack: time kept free before it (default INTERCEPT_SLACK_S, for a new order)."""
        cacheable = drones is None
        if cacheable and tr.plan_cache and abs(tr.plan_cache[0] - now) < INTERCEPT_REPLAN_S:
            return tr.plan_cache[1]
        if drones is None:
            drones = [p for p in (self._pose(d) for d in self.free_drones()) if p]
        # New jobs yield to committed ones: avoid blasts that would force a hold on a drone already
        # flying another job; only if no such point exists, accept holds (drones route around).
        committed = [(pos, t.point, t.t_engage) for t in self.tracks.values()
                     if t.status == "approved" and t.threat_id != tr.threat_id
                     for pos in (self._pose(d) for d in t.confirmed) if pos]
        args = (tr.position, now, tr.legacy_t, drones, level or tr.level,
                self._reservations(exclude=tr.threat_id), self.v_max, self.a_max)
        kw = dict(max_range=self.args.intercept_range, z_range=self.z_range,
                  min_range=self.engage_radius if self.no_fly > 0 else 0.0, no_fly=self.no_fly_route)
        if slack is not None:
            kw["slack"] = slack
        ic = choose_intercept(*args, committed=committed, **kw) or choose_intercept(*args, **kw)
        if cacheable:
            tr.plan_cache = (now, ic)
        return ic

    def _intruders(self, tr):
        """Drones not on this job inside its zone (heartbeat positions, up to 0.5 s old)."""
        on_job = set(tr.confirmed) | set(tr.pending) | set(tr.detonated)
        out = []
        for d in self.members():
            hb = self.drones[d]["hb"]
            p = hb.get("pose")
            if d in on_job or not p:
                continue
            if math.dist((p["x"], p["y"], p["z"]), tr.point) < self.zone_radius(tr):
                out.append({"id": d, "pose": p, "sigma": hb.get("pos_sigma")})
        return out

    def _publish_zones(self):
        zones = [{"threat_id": tr.threat_id, "point": _xyz(tr.point), "radius": self.zone_radius(tr),
                  "t_engage": tr.t_engage} for tr in self.tracks.values() if tr.status == "approved"]
        self.pub_zones.put(json.dumps(self._stamped({"zones": zones})))

    def _publish_job(self, tr):
        tr.job_seq += 1
        self.radio.put(f"ship/jobs/{tr.threat_id}", json.dumps(self._stamped(self._job_msg(tr))))

    def _arbitrate(self):
        """Confirm the best pending awards per threat once the collection window has passed."""
        for tr in self.tracks.values():
            if not tr.pending or simclock.now() - tr.pending_since < ACK_WINDOW_S:
                continue
            need = tr.level - len(tr.hits) - len(tr.confirmed) if tr.status == "approved" else 0
            # The ship sees every job: re-check each candidate's route around the other jobs' blasts
            # from its reported position before confirming it.
            now = self.ship_time or 0.0
            blasts = [r.blast() for r in self._reservations(exclude=tr.threat_id)]
            unfit = set()
            for d in tr.pending:
                pos = self._pose(d)
                if pos is None:
                    continue
                route = plan_route(pos, tr.point, now, blasts, self.v_max, self.a_max, t_goal=tr.t_engage,
                                   no_fly=self.no_fly_route)
                if route.blocked or now + route.cost() > tr.t_engage:
                    unfit.add(d)
            accepted, rejected = arbitrate({d: c for d, (c, _) in tr.pending.items() if d not in unfit},
                                           tr.confirmed, need, tr.level)
            rejected += sorted(unfit)
            for agent, slot in accepted.items():
                tr.confirmed[agent] = slot
                self._send_ack(tr, agent, True, tr.pending[agent][1])
                self.event("confirmed", threat=tr.threat_id, drone=agent, slot=slot)
            for agent in rejected:
                wave = tr.pending[agent][1]
                tr.rejected.add((agent, wave))
                self._send_ack(tr, agent, False, wave)
                self.event("rejected", threat=tr.threat_id, drone=agent, order=wave)
            tr.pending.clear()
            if (tr.approved_wall is not None and tr.full_award_ms is None
                    and len(tr.confirmed) + len(tr.hits) >= tr.level):
                tr.full_award_ms = round((simclock.now() - tr.approved_wall) * 1000)
            if accepted:
                self._publish_job(tr)

    def _publish_jobs(self):
        for tr in self.tracks.values():
            if tr.status == "approved":
                self._publish_job(tr)
            elif tr.close_repeats > 0:
                tr.close_repeats -= 1
                self._publish_job(tr)

    def _reconcile(self):
        """Drop confirmed drones whose heartbeats show they left the job (their withdrawal was lost)."""
        members = set(self.members())
        now = simclock.now()
        for tr in self.tracks.values():
            if tr.status != "approved":
                continue
            for agent in list(tr.confirmed):
                hb = self.drones.get(agent, {}).get("hb", {})
                if agent not in members or hb.get("threat_id") == tr.threat_id:
                    tr.mismatch.pop(agent, None)      # unheard drones keep their job (link loss)
                    continue
                since = tr.mismatch.setdefault(agent, now)
                if now - since >= HB_MISMATCH_S:
                    del tr.confirmed[agent]
                    del tr.mismatch[agent]
                    self.event("dropped", threat=tr.threat_id, drone=agent, reason="heartbeat shows no job")

    def _maneuver(self, now):
        """Threats with a planned manoeuvre turn once, re-aiming past the ship."""
        for tr in self.tracks.values():
            if not tr.active or tr.maneuver_at is None or now < tr.maneuver_at:
                continue
            tr.maneuver_at = None
            p = tr.true_position(self.truth_time)          # the threat turns in truth
            speed = math.hypot(tr.true_v[0], tr.true_v[1])
            miss = self.maneuver_rng.uniform(-self.args.max_miss, self.args.max_miss)
            old_point, old_t = tr.point, tr.t_engage
            tr.retrack(p, aim_velocity(p, speed, miss), self.truth_time)
            if tr.status == "approved":
                # Re-plan the intercept for the drones already on the job (legacy point if none fits).
                crew = [q for q in (self._pose(d) for d in set(tr.confirmed) | set(tr.pending)) if q]
                ic = (self.plan_intercept(tr, now, drones=crew, level=max(1, len(crew)), slack=MANEUVER_REPLAN_SLACK_S)
                      if crew else None)
                if ic is not None:
                    tr.point, tr.t_engage = ic.point, ic.t
            tr.maneuvers += 1
            self.queue.update(tr.threat_id, tr.t_cpa)
            self.pub_tracks.put(json.dumps(tr.track_msg(truth=True)))
            self.event("maneuver", threat=tr.threat_id, point_shift_m=round(math.dist(old_point, tr.point), 1),
                       t_engage_shift_s=round(tr.t_engage - old_t, 1),
                       range_m=[round(math.hypot(old_point[0], old_point[1]), 1),
                                round(math.hypot(tr.point[0], tr.point[1]), 1)],
                       replanned=tr.status == "approved" and tr.point != tr.legacy_point)
            if tr.status == "approved":
                self._publish_job(tr)

    def _on_damage(self, sample):
        """A drone destroyed by a detonation of another job (physics, observed like detonations)."""
        d = self._parse(sample)
        with self.lock:
            victim = d["agent_id"]
            self.expended.add(victim)
            by = self.tracks.get(d.get("by_threat"))
            if by is not None:
                by.friendly_fire.append(victim)
            for tr in self.tracks.values():            # its own job, if any, loses it
                tr.confirmed.pop(victim, None)
                tr.pending.pop(victim, None)
            self.event("friendly_fire", drone=victim, by=d.get("by"), threat=d.get("by_threat"),
                       job=d.get("job"), distance_m=d.get("distance"))

    def _on_detonation(self, sample):
        """Kill assessment.  The radar observes the blast physically, so this uses truth (the
        drone's sensor-frame time at detonation), whatever any clock says."""
        d = self._parse(sample)
        with self.lock:
            self.expended.add(d["agent_id"])
            tr = self.tracks.get(d["threat_id"])
            if tr is None:
                return
            truth = float(d["truth_time"])
            here = (d["x"], d["y"], d["z"])
            miss = math.dist(tr.true_position(truth), here)
            self._evaluate_detonation(tr, d, truth, here, miss)
            tr.detonated[d["agent_id"]] = round(miss, 2)
            tr.confirmed.pop(d["agent_id"], None)
            if d.get("intruders"):
                tr.intruded += 1
                self.event("detonation_risk", threat=tr.threat_id, drone=d["agent_id"], intruders=d["intruders"])
            if miss <= self.args.kill_radius:
                tr.hits.add(d["agent_id"])
            self.event("detonation", threat=tr.threat_id, drone=d["agent_id"], miss_m=round(miss, 2),
                       hit=miss <= self.args.kill_radius)
            if tr.active and len(tr.hits) >= tr.level:
                self._close(tr, "destroyed")

    def _evaluate_detonation(self, tr, d, truth, here, miss):
        """Timing against truth (evaluation only).  Primary: the geometric error, detonation truth
        time minus the moment the threat truly passes closest to the detonation point; it holds at
        any threat speed.  Secondary: the error against the ordered t_engage (ship time, mapped to
        truth through the ship's clock)."""
        ideal_t, ideal_miss = tr.true_closest_approach(here)
        ordered = tr.t_engage                   # the ship's latest order (ship time)
        rec = {"threat": tr.threat_id, "drone": d["agent_id"], "truth_t": round(truth, 3),
               "local_t": d.get("local_time"), "sync_t": d.get("sync_time"), "t_engage": ordered,
               "ideal_t": round(ideal_t, 3), "timing_err_s": round(truth - ideal_t, 3),
               "ordered_err_s": round(truth - self.clock.to_truth(ordered), 3),
               "miss_m": round(miss, 2), "ideal_miss_m": round(ideal_miss, 2), "chain": bool(d.get("chain")),
               "threat_speed": round(math.hypot(*tr.true_v), 2),
               "reason": d.get("reason", "timed"), "chain_delay_s": d.get("chain_delay_s"), "chain_by": d.get("chain_by"),
               "range_m": round(math.hypot(here[0], here[1]), 1)}      # blast's distance from the ship
        if d.get("est"):
            # where the drone believed it was (its localization error at the blast; evaluation only)
            rec["est_err_m"] = round(math.dist(tuple(d["est"]), here), 2)
        if d.get("trigger"):
            # Ground-truth label of what the fuze fired on (evaluation only): was the contact the threat?
            trig = tuple(here[k] + float(d["trigger"][k]) for k in range(3))
            rec["trigger_to_threat_m"] = round(math.dist(trig, tr.true_position(truth)), 2)
            rec["trigger_is_threat"] = rec["trigger_to_threat_m"] <= 1.0
        tr.det_eval.append(rec)
        self.log_only("detonation_eval", **rec)

    # ------------------------------------------------------------- radar
    def _maybe_detect(self, now):
        a = self.args
        if a.max_threats and self.detected >= a.max_threats:
            return
        if self.next_detection is None:
            self.next_detection = now + a.first_detection
        if now < self.next_detection:
            return
        self.next_detection = now + self.rng.uniform(0.6, 1.4) * a.detection_interval
        self.detected += 1

        names = list(self.types)
        kind = self.rng.choices(names, weights=[self.types[n].weight for n in names])[0]
        tt = self.types[kind]
        bearing = self.rng.uniform(0.0, 2.0 * math.pi)
        rng_m = self.rng.uniform(a.detect_min, a.detect_max)
        z = self.rng.uniform(tt.min_z, tt.max_z)
        p0 = (rng_m * math.cos(bearing), rng_m * math.sin(bearing), z)
        # Aim somewhere near the ship: miss distance up to max_miss, sideways to the line of sight.
        miss = self.rng.uniform(-a.max_miss, a.max_miss)
        v = aim_velocity(p0, tt.speed, miss)

        tr = Track(f"T{self.detected}", kind, tt.level, p0, v, self.truth_time, self.engage_radius, self.clock)
        if a.maneuver_p > 0 and self.maneuver_rng.random() < a.maneuver_p:
            tr.maneuver_at = now + self.maneuver_rng.uniform(0.3, 0.6) * (tr.t_engage - now)
        self.tracks[tr.threat_id] = tr
        self.queue.push(tr.threat_id, tr.t_cpa)
        self.pub_tracks.put(json.dumps(tr.track_msg(truth=True)))
        self.event("detected", threat=tr.threat_id, type=kind, level=tt.level,
                   tcpa_s=round(tr.t_cpa - now, 1), cpa_m=round(tr.cpa_dist, 1),
                   t_to_engage_s=round(tr.t_engage - now, 1))

    def _close(self, tr, status):
        was_job = tr.status == "approved"
        tr.status = status
        tr.pending.clear()
        if was_job:   # tell the job's drones it is over (surviving ones become free)
            tr.close_repeats = JOB_CLOSE_REPEATS
            self._publish_job(tr)
        self.queue.remove(tr.threat_id)
        self.pub_tracks.put(json.dumps(tr.track_msg(truth=True)))
        self.pub_status.put(json.dumps({"threat_id": tr.threat_id, "status": status}))
        self.event(status, threat=tr.threat_id, type=tr.type, level=tr.level, hits=len(tr.hits),
                   detonations=len(tr.detonated))

    def _age_tracks(self, now):
        for tr in list(self.tracks.values()):
            if not tr.active:
                continue
            if (tr.status == "approved" and now > tr.t_engage + 1.0 and not tr.confirmed and not tr.pending
                    and len(tr.hits) < tr.level):
                self._close(tr, "failed")      # engagement time passed without a kill
            elif now >= tr.t_cpa:
                self._close(tr, "impact" if tr.cpa_dist <= SHIP_RADIUS else "leaked")

    # ------------------------------------------------------- swarm picture
    def members(self):
        now = simclock.now()
        return sorted(d for d, v in self.drones.items()
                      if "seen" in v and now - v["seen"] <= MEMBER_TIMEOUT_S and d not in self.expended)

    def free_drones(self):
        return [d for d in self.members() if self.drones[d]["hb"].get("state") == "idle"]

    def feasibility(self, tr, now):
        """Is there an intercept point on the track that `level` free drones can reach in time?"""
        free = len([d for d in self.free_drones() if self._pose(d)])
        ic = self.plan_intercept(tr, now) if free >= tr.level else None
        if free < tr.level:
            reason = f"needs {tr.level} free drones, {free} available"
        elif ic is None:
            reason = "no reachable intercept point before the defended radius"
        else:
            reason = "feasible"
        return {"tti_s": None if ic is None else round(ic.tti_s, 1),
                "available_s": None if ic is None else round(ic.t - now, 1),
                "intercept_range_m": None if ic is None else round(math.hypot(ic.point[0], ic.point[1]), 1),
                "free": free, "feasible": ic is not None, "reason": reason}

    # ------------------------------------------------------------- orders
    def set_auto_approve(self, enabled, reaction_s=None):
        with self.lock:
            if reaction_s is not None:
                self.auto_reaction_s = max(0.0, float(reaction_s))
            if bool(enabled) != self.auto_approve:
                self.auto_approve = bool(enabled)
                self._feasible_since.clear()
                self.event("auto_approve", enabled=self.auto_approve, reaction_s=self.auto_reaction_s)
            return {"enabled": self.auto_approve, "reaction_s": self.auto_reaction_s}

    def _auto_approve(self, now):
        """Approve the most urgent threat that has been feasible for auto_reaction_s (ship time), one
        per call, like the stand-in operator (tools/comms/operator_bot.py) but on the ship."""
        if not self.auto_approve:
            return
        for tid in self.queue.ordered():                # min-heap order: smallest TCPA first
            tr = self.tracks[tid]
            if tr.status != "tracking":
                continue
            if not self.feasibility(tr, now)["feasible"]:
                self._feasible_since.pop(tid, None)
                continue
            since = self._feasible_since.setdefault(tid, now)
            if now - since >= self.auto_reaction_s:
                self._feasible_since.pop(tid, None)
                self.approve(tid, by="auto")
                return

    def approve(self, threat_id, by="operator"):
        with self.lock:
            tr = self.tracks.get(threat_id)
            now = self.ship_time
            if tr is None or now is None:
                return False, "unknown threat"
            if tr.status != "tracking":
                return False, f"threat is {tr.status}"
            f = self.feasibility(tr, now)
            ic = tr.plan_cache[1] if f["feasible"] else None
            if ic is None:
                self.event("approval_rejected", threat=threat_id, reason=f["reason"])
                return False, f["reason"]
            tr.point, tr.t_engage = ic.point, ic.t      # engage as far out as the drones can reach
            tr.intercept_range = round(math.hypot(ic.point[0], ic.point[1]), 1)
            tr.legacy_range = round(math.hypot(tr.legacy_point[0], tr.legacy_point[1]), 1)
            tr.status = "approved"
            tr.approved_wall = simclock.now()
            self._announce(tr, tr.level)
            self._publish_zones()             # idle drones start clearing the zone right away
            self.event("approved", threat=threat_id, by=by, tti_s=f["tti_s"], available_s=f["available_s"],
                       intercept_range_m=tr.intercept_range, legacy_range_m=tr.legacy_range)
            return True, "approved"

    def _announce(self, tr, required):
        self.order_seq += 1
        tr.orders.append(f"O{self.order_seq}")
        order = Threat(tr.threat_id, tr.type, tr.level, required, _xyz(tr.point), tr.t_engage)
        self.pub_orders.put(json.dumps(self._stamped({"wave_id": f"O{self.order_seq}", "threats": [order.to_dict()]})))
        tr.announces += 1
        tr.last_announce = simclock.now()

    def _engaged_by_heartbeat(self, tr):
        """Roster members whose latest heartbeat says they are engaging this threat."""
        return {d for d in self.members()
                if self.drones[d]["hb"].get("threat_id") == tr.threat_id
                and self.drones[d]["hb"].get("state") in ("engaging", "holding", "on_station")}

    def _retry_underassigned(self, now):
        for tr in self.tracks.values():
            if tr.status != "approved" or tr.announces >= MAX_ANNOUNCES:
                continue
            if simclock.now() - tr.last_announce < RETRY_AFTER_S:
                continue
            # A lost award must not trigger a re-announce (and a second drone) when the
            # drone's heartbeats show it engaging, so count those too.
            assigned = len(set(tr.confirmed) | set(tr.pending) | set(tr.detonated) | self._engaged_by_heartbeat(tr))
            missing = tr.level - assigned
            if missing <= 0:
                continue
            if tr.t_engage - now <= ORDER_SLACK_S:
                continue
            self._announce(tr, missing)
            self.event("reannounced", threat=tr.threat_id, missing=missing)

    # --------------------------------------------------------------- loop
    def run(self):
        last_roster = last_job = 0.0
        while True:
            with self.lock:
                now = self.ship_time
                if now is not None:
                    if self.args.max_threats != 0:
                        self._maybe_detect(now)
                    self._maneuver(now)
                    self._age_tracks(now)
                    self._arbitrate()
                    self._reconcile()
                    self._retry_underassigned(now)
                    self._auto_approve(now)
                wall = simclock.now()
                if wall - last_job >= 1.0 / JOB_HZ:
                    last_job = wall
                    self._publish_jobs()
                if wall - last_roster >= 1.0 / ROSTER_HZ:
                    last_roster = wall
                    members = self.members()
                    relayed = [d for d in members if self.drones[d].get("relayed")]
                    roster = {"time": now, "count": len(members), "members": members, "relayed": relayed}
                    if self.sync_mode in ("master", "consensus"):
                        # Two-way exchange: echo each drone's latest t1 with our receive stamp t2 and
                        # this send stamp t3 (one broadcast for all drones, no extra messages).  With
                        # consensus, t3 is also the pinned leader's clock beacon.
                        t3 = round(self._stamp(), 6)
                        roster["sync"] = {d: [self.drones[d]["sync"][0], round(self.drones[d]["sync"][1], 6), t3]
                                          for d in members if self.drones[d].get("sync")}
                        if self.sync_mode == "consensus":
                            roster["beacon"] = t3
                    self.pub_roster.put(json.dumps(roster))
                    self._publish_zones()
                    span = wall - self._rx_window_start
                    if span >= 1.0:
                        self.rx_rates = {k: {"msgs_per_s": round(self.rx_counts[k] / span, 1),
                                             "bytes_per_s": round(self.rx_bytes[k] / span)} for k in self.rx_counts}
                        self.rx_counts.clear()
                        self.rx_bytes.clear()
                        self._rx_window_start = wall
            simclock.sleep(0.05)

    # ------------------------------------------------------------ snapshot
    def _agreement(self, tr):
        """Share of drones that computed the same assignment, over this threat's orders (None if unknown)."""
        agreeing = total = 0
        for order in tr.orders:
            counts = defaultdict(int)
            for d in self.drones.values():
                h = d.get("telemetry", {}).get("assignments", {}).get(order)
                if h:
                    counts[h] += 1
            if counts:
                agreeing += max(counts.values())
                total += sum(counts.values())
        return None if total == 0 else round(agreeing / total, 3)

    def summary(self):
        """Run-level outcome figures for experiments (GET /api/summary)."""
        s = self.snapshot(all_threats=True)   # the dashboard's snapshot lists only 10 closed threats
        engaged = [t for t in s["threats"] if t["announces"] > 0]
        lat = [t["full_award_ms"] for t in engaged if t["full_award_ms"] is not None]
        agree = [t["agreement"] for t in engaged if t["agreement"] is not None]
        counts = s["threat_counts"]
        with self.lock:
            evals = [e for t in self.tracks.values() for e in t.det_eval]
        timing = _stats([e["timing_err_s"] for e in evals])
        chain_delays = _stats([e["chain_delay_s"] for e in evals if e.get("chain_delay_s") is not None], 4)
        reasons = defaultdict(int)
        for e in evals:
            reasons[e.get("reason", "timed")] += 1
        ordered = _stats([e["ordered_err_s"] for e in evals])
        miss = _stats([e["miss_m"] for e in evals], 2)
        with self.lock:
            sync_err = _stats([e for _, e, _ in self.sync_evals], 4)
            bounds = [b for _, _, b in self.sync_evals if b is not None]
            covered = [e <= b for _, e, b in self.sync_evals if b is not None]
        return {
            "sim_time": s["sim_time"],
            "threats_detected": len(self.tracks),
            "approved": len(engaged),
            "destroyed": counts.get("destroyed", 0),
            "failed": counts.get("failed", 0),
            "leaked_or_impact": counts.get("leaked", 0) + counts.get("impact", 0),
            "still_active": counts.get("tracking", 0) + counts.get("approved", 0),
            "kill_ratio": round(counts.get("destroyed", 0) / len(engaged), 3) if engaged else None,
            "award_latency_ms_mean": round(sum(lat) / len(lat)) if lat else None,
            "award_latency_ms_max": max(lat) if lat else None,
            "never_fully_assigned": sum(1 for t in engaged if t["full_award_ms"] is None),
            "reannounces": sum(max(0, t["announces"] - 1) for t in engaged),
            "over_assigned": sum(1 for t in engaged if len(t["detonated"]) > t["level"]),
            "rejected_awards": sum(t["rejected"] for t in engaged),
            "friendly_fire": sum(len(t["friendly_fire"]) for t in s["threats"]),
            "intercept_range_m_mean": _mean([t["intercept_range_m"] for t in engaged]),
            "legacy_range_m_mean": _mean([t["legacy_range_m"] for t in engaged]),
            "drones_with_holds": sum(sum(1 for h in t["holds"].values() if h > 0) for t in engaged),
            "hold_s_max": max([h for t in engaged for h in t["holds"].values()], default=0.0),
            "detonations_with_intruders": sum(t["intruded"] for t in s["threats"]),
            "maneuvers": sum(t["maneuvers"] for t in s["threats"]),
            "missed_slots": sum(len(t["missed"]) for t in engaged),
            "agreement_mean": round(sum(agree) / len(agree), 3) if agree else None,
            "drones_expended": s["roster"]["expended"],
            "radio_rx_at_ship": s["radio_rx_at_ship"],
            # Detonation timing against truth (_evaluate_detonation): geometric error (primary),
            # error against the ordered t_engage, and miss distance.
            "detonations": len(evals),
            "det_timing_err_s_mean": timing[0], "det_timing_err_s_sd": timing[1], "det_timing_err_s_absmax": timing[2],
            "det_ordered_err_s_mean": ordered[0], "det_ordered_err_s_sd": ordered[1],
            "det_ordered_err_s_absmax": ordered[2],
            "det_miss_m_mean": miss[0], "det_miss_m_sd": miss[1], "det_miss_m_max": miss[2],
            "chain_fires": sum(1 for e in evals if e["chain"]),
            # True separations between drones (metrics node, sim/truth).
            "collisions": self.metrics.get("total_collisions"), "close_calls": self.metrics.get("close_calls"),
            "min_separation_m": self.metrics.get("min_separation_m"),
            "min_ship_range_m": self.metrics.get("min_ship_range_m"),         # closest any drone came to the ship
            **{k: v for k, v in self.metrics.items() if k.startswith("loc_")},   # localization error vs truth
            # Proximity fuze: what fired each detonation, false triggers (ground truth), holds.
            "det_reasons": dict(reasons),
            "fuze_false_triggers": sum(1 for e in evals if e.get("trigger_is_threat") is False),
            "fuze_no_detection": sum(len(t["no_detection"]) for t in s["threats"]),
            "chain_delay_s_mean": chain_delays[0], "chain_delay_s_max": chain_delays[2],
            # Clock sync (drone telemetry, 1 Hz per live drone): |true error| and the drones' own bounds.
            "sync_mode": self.sync_mode,
            "sync_err_s_mean": sync_err[0], "sync_err_s_sd": sync_err[1], "sync_err_s_max": sync_err[2],
            "sync_bound_s_mean": round(sum(bounds) / len(bounds), 4) if bounds else None,
            "sync_bound_coverage": round(sum(covered) / len(covered), 3) if covered else None,
            "ship_clock": self.clock.describe(),
        }

    def snapshot(self, all_threats=False):
        with self.lock:
            now = self.ship_time
            wall = simclock.now()
            threats = []
            order = self.queue.ordered()
            closed = [t for t in self.tracks.values() if not t.active]
            recent = sorted(closed, key=lambda t: t.t0, reverse=True)
            for tid in order + [t.threat_id for t in (recent if all_threats else recent[:10])]:
                tr = self.tracks[tid]
                row = {
                    "threat_id": tid, "type": tr.type, "level": tr.level, "status": tr.status,
                    "pos": _xyz(position_at(tr.p0, tr.v, tr.t0, now)) if now is not None else _xyz(tr.p0),
                    "v": _xyz(tr.v), "cpa": _xyz(tr.cpa), "cpa_dist": round(tr.cpa_dist, 1),
                    "tcpa_s": None if now is None else round(tr.t_cpa - now, 1),
                    "point": _xyz(tr.point),
                    "t_to_engage_s": None if now is None else round(tr.t_engage - now, 1),
                    "holders": sorted(tr.holders), "hits": sorted(tr.hits), "detonated": tr.detonated,
                    "missed": sorted(tr.missed), "no_detection": sorted(tr.no_detection), "announces": tr.announces,
                    "pending": sorted(tr.pending), "rejected": len(tr.rejected), "maneuvers": tr.maneuvers,
                    "friendly_fire": list(tr.friendly_fire), "intruded": tr.intruded,
                    "intercept_range_m": tr.intercept_range, "legacy_range_m": tr.legacy_range,
                    "holds": dict(tr.holds),
                    "agreement": self._agreement(tr),
                    "first_award_ms": tr.first_award_ms, "full_award_ms": tr.full_award_ms,
                }
                if tr.status == "tracking" and now is not None:
                    row["feasibility"] = self.feasibility(tr, now)
                threats.append(row)

            drones = []
            for d, v in sorted(self.drones.items(), key=lambda kv: int(kv[0].split("_")[-1])
                               if kv[0].split("_")[-1].isdigit() else 0):
                hb = v.get("hb", {})
                drones.append({"id": d, "state": "expended" if d in self.expended else hb.get("state"),
                               "link": hb.get("link"), "relayed": v.get("relayed", False),
                               "pose": hb.get("pose"), "threat_id": hb.get("threat_id"),
                               "hb_age_s": None if "seen" not in v else round(wall - v["seen"], 1),
                               "clock_err_ms": v.get("clock_err_ms"), "clock_bound_ms": v.get("clock_bound_ms"),
                               "telemetry": v.get("telemetry", {})})
            members = self.members()
            counts = defaultdict(int)
            for t in self.tracks.values():
                counts[t.status] += 1
            return {
                "sim_time": self.truth_time, "ship_time": now, "ship_clock": self.clock.describe(),
                "auto_approve": {"enabled": self.auto_approve, "reaction_s": self.auto_reaction_s},
                "params": {"defended_radius": self.args.defended_radius, "kill_radius": self.args.kill_radius,
                           "ship_radius": SHIP_RADIUS, "v_max": self.v_max, "a_max": self.a_max},
                "roster": {"count": len(members), "free": len(self.free_drones()), "members": members,
                           "expended": len(self.expended)},
                "threats": threats, "threat_counts": dict(counts),
                "drones": drones, "radio_rx_at_ship": self.rx_rates,
                "events": list(self.events)[-40:],
            }


# ---------------------------------------------------------------------- HTTP
def make_handler(ship):
    page = (HERE / "dashboard.html").read_bytes()

    class Handler(BaseHTTPRequestHandler):
        def log_message(self, *args):
            pass

        def _json(self, code, body):
            data = json.dumps(body).encode()
            self.send_response(code)
            self.send_header("Content-Type", "application/json")
            self.send_header("Content-Length", str(len(data)))
            self.end_headers()
            self.wfile.write(data)

        def do_GET(self):
            if self.path in ("/", "/index.html"):
                self.send_response(200)
                self.send_header("Content-Type", "text/html; charset=utf-8")
                self.send_header("Content-Length", str(len(page)))
                self.end_headers()
                self.wfile.write(page)
            elif self.path == "/api/summary":
                self._json(200, ship.summary())
            elif self.path == "/api/state":
                self._json(200, ship.snapshot())
            elif self.path == "/api/stream":
                self.send_response(200)
                self.send_header("Content-Type", "text/event-stream")
                self.send_header("Cache-Control", "no-cache")
                self.end_headers()
                try:
                    while True:
                        self.wfile.write(f"data: {json.dumps(ship.snapshot())}\n\n".encode())
                        self.wfile.flush()
                        time.sleep(0.25)
                except (BrokenPipeError, ConnectionResetError):
                    pass
            else:
                self._json(404, {"error": "not found"})

        def do_POST(self):
            if self.path == "/api/auto_approve":
                try:
                    body = json.loads(self.rfile.read(int(self.headers.get("Content-Length", 0))) or b"{}")
                    return self._json(200, {"ok": True, **ship.set_auto_approve(bool(body["enabled"]),
                                                                                body.get("reaction_s"))})
                except Exception as e:
                    return self._json(400, {"ok": False, "message": f"bad request: {e}"})
            if self.path != "/api/approve":
                return self._json(404, {"error": "not found"})
            try:
                body = json.loads(self.rfile.read(int(self.headers.get("Content-Length", 0))) or b"{}")
                ok, msg = ship.approve(str(body["threat_id"]))
            except Exception as e:
                ok, msg = False, f"bad request: {e}"
            self._json(200 if ok else 409, {"ok": ok, "message": msg})

    return Handler


def main():
    types_default = ",".join(f"{k}:{t.level}:{t.speed}:{t.weight}" for k, t in DEFAULT_THREAT_TYPES.items())
    ap = argparse.ArgumentParser(description="DeNDDron ship C2 and operator dashboard")
    ap.add_argument("--sim-bus", default=os.environ.get("SIM_BUS"))
    ap.add_argument("--runtime-config", default=os.environ.get("SWARM_RUNTIME_CONFIG", "/app/config/swarm_runtime.json"))
    ap.add_argument("--http-port", type=int, default=_env("DASHBOARD_PORT", 8080, int))
    ap.add_argument("--seed", type=int, default=_env("THREAT_SEED", 42, int))
    ap.add_argument("--max-threats", type=int, default=_env("MAX_THREATS", 0, int),
                    help="threats to generate (0 = radar simulation off)")
    ap.add_argument("--first-detection", type=float, default=_env("THREAT_FIRST_S", 20.0, float),
                    help="sim seconds after start before the first detection")
    ap.add_argument("--detection-interval", type=float, default=_env("THREAT_INTERVAL_S", 30.0, float),
                    help="mean sim seconds between detections")
    ap.add_argument("--threat-types", default=os.environ.get("THREAT_TYPES") or types_default,
                    help="type:level:speed:weight,...")
    ap.add_argument("--detect-min", type=float, default=_env("DETECT_MIN_M", 150.0, float))
    ap.add_argument("--detect-max", type=float, default=_env("DETECT_MAX_M", 190.0, float))
    ap.add_argument("--max-miss", type=float, default=_env("MAX_MISS_M", 30.0, float),
                    help="largest lateral miss distance of generated tracks")
    ap.add_argument("--intercept-range", type=float, default=_env("INTERCEPT_MAX_RANGE_M", 140.0, float),
                    help="farthest intercept point from the ship (drones' reach: spawn ring + service radius)")
    ap.add_argument("--maneuver-p", type=float, default=_env("THREAT_MANEUVER_P", 0.0, float),
                    help="probability that a threat turns once, re-aiming past the ship")
    ap.add_argument("--defended-radius", type=float, default=_env("DEFENDED_RADIUS_M", 45.0, float))
    ap.add_argument("--kill-radius", type=float, default=_env("KILL_RADIUS_M", 8.0, float))
    ap.add_argument("--no-fly", type=float, default=_env("NO_FLY_RADIUS_M", 0.0, float),
                    help="ship no-fly zone radius (m, 0: off)")
    ap.add_argument("--log-path", default=os.environ.get("SHIP_LOG", "/state/ship_log.jsonl"))
    args = ap.parse_args()

    try:
        Path(args.log_path).parent.mkdir(parents=True, exist_ok=True)
    except OSError:
        args.log_path = None
    ship = Ship(args)
    server = ThreadingHTTPServer(("0.0.0.0", args.http_port), make_handler(ship))
    server.daemon_threads = True
    threading.Thread(target=server.serve_forever, daemon=True).start()
    log.info("dashboard on http://localhost:%d  (radar: %s)", args.http_port,
             f"{args.max_threats} threats" if args.max_threats else "off")
    # Running as PID 1 in the container: SIGTERM is ignored unless handled.
    signal.signal(signal.SIGTERM, signal.default_int_handler)
    try:
        ship.run()
    except KeyboardInterrupt:
        log.info("shutting down")
    finally:
        server.shutdown()
        ship.radio.close()
        ship.onboard.close()
        if ship.log_file:
            ship.log_file.close()


if __name__ == "__main__":
    main()
