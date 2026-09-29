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
import simclock
from radio_process import RadioProcess
from jobs import arbitrate
from threat_queue import ThreatQueue
from threats import (DEFAULT_THREAT_TYPES, ETA_MARGIN, ORDER_SLACK_S, Threat, aim_velocity,
                     closest_point_of_approach, engagement_point, eta, parse_threat_types, position_at,
                     time_to_intercept)

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


def _env(name, default, cast):
    return cast(os.environ.get(name, default))


def _xyz(p):
    return {"x": round(p[0], 2), "y": round(p[1], 2), "z": round(p[2], 2)}


class Track:
    """One radar track plus its engagement bookkeeping."""

    def __init__(self, threat_id, kind, level, p0, v, t0, defended_radius):
        self.threat_id, self.type, self.level = threat_id, kind, level
        self.defended_radius = defended_radius
        self.retrack(p0, v, t0)
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
        self.detonated = {}             # drone -> miss distance (m)
        self.hits = set()               # detonations within kill radius
        self.missed = set()             # drones that aborted
        self.announces = 0
        self.orders = []                # order (wave) ids sent for this threat
        self.last_announce = None
        self.approved_wall = None       # simclock time of approval
        self.award_events = set()       # (drone, status, order) already logged: awards are sent several times
        self.first_award_ms = None
        self.full_award_ms = None

    def retrack(self, p0, v, t0):
        """New track segment (detection or manoeuvre): recompute CPA and the engagement point."""
        self.p0, self.v, self.t0 = p0, v, t0
        self.cpa, self.t_cpa = closest_point_of_approach(p0, v, t0)
        self.cpa_dist = math.hypot(self.cpa[0], self.cpa[1])
        self.point, self.t_engage = engagement_point(p0, v, t0, self.defended_radius)

    @property
    def holders(self):
        return set(self.confirmed)

    @property
    def active(self):
        return self.status in ("tracking", "approved")

    def track_msg(self):
        return {"threat_id": self.threat_id, "type": self.type, "level": self.level,
                "status": "active" if self.active else self.status, "t0": self.t0,
                "p0": _xyz(self.p0), "v": _xyz(self.v)}


class Ship:
    def __init__(self, args):
        self.args = args
        self.types = parse_threat_types(args.threat_types) if args.threat_types else DEFAULT_THREAT_TYPES
        self.rng = random.Random(args.seed)
        self.maneuver_rng = random.Random(args.seed + 1)   # separate, so the scenario is unchanged
        self.lock = threading.RLock()

        self.sim_time = None
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

        self.radio = RadioProcess()          # own process: a radio failure must not freeze C2
        self.onboard = open_onboard(args.sim_bus)
        self.pub_tracks = self.onboard.declare_publisher("sim/threat_tracks")
        self.pub_orders = self.radio.declare_publisher("swarm/threats")
        self.pub_roster = self.radio.declare_publisher("ship/roster")
        self.pub_status = self.radio.declare_publisher("ship/threat_status")
        self.subs = [
            self.onboard.declare_subscriber("sim/clock", self._on_clock),
            self.onboard.declare_subscriber("sim/detonation", self._on_detonation),
            self.radio.declare_subscriber("swarm/heartbeat/*", self._on_heartbeat),
            self.radio.declare_subscriber("swarm/heartbeat_relay/*", self._on_relayed_heartbeat),
            self.radio.declare_subscriber("swarm/telemetry/*", self._on_telemetry),
            self.radio.declare_subscriber("swarm/awards", self._on_award),
            self.radio.declare_subscriber("swarm/bids", self._on_bid),
        ]

    def _kinematics(self):
        try:
            cfg = json.loads(Path(self.args.runtime_config).read_text())["defaults"]["kinematics"]
            return float(cfg["max_velocity"]), float(cfg["max_acceleration"])
        except Exception:
            return 4.0, 1.0

    # ------------------------------------------------------------------ util
    def event(self, kind, **fields):
        e = {"wall": time.strftime("%H:%M:%S"), "sim_time": self.sim_time, "kind": kind, **fields}
        self.events.append(e)
        if self.log_file:
            self.log_file.write(json.dumps(e) + "\n")
        log.info("%s %s", kind, " ".join(f"{k}={v}" for k, v in fields.items()))

    def _count_rx(self, topic, sample):
        self.rx_counts[topic] += 1
        self.rx_bytes[topic] += len(bytes(sample.payload))

    @staticmethod
    def _parse(sample):
        return json.loads(bytes(sample.payload).decode("utf-8"))

    # ------------------------------------------------------------- callbacks
    def _on_clock(self, sample):
        with self.lock:
            self.sim_time = float(self._parse(sample)["sim_time"])

    def _on_heartbeat(self, sample):
        self._count_rx("swarm/heartbeat", sample)
        hb = self._parse(sample)
        with self.lock:
            d = self.drones.setdefault(hb["agent_id"], {"telemetry": {}})
            d["hb"], d["seen"], d["relayed"] = hb, simclock.now(), False
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

    def _on_telemetry(self, sample):
        self._count_rx("swarm/telemetry", sample)
        t = self._parse(sample)
        with self.lock:
            self.drones.setdefault(t["agent_id"], {"telemetry": {}})["telemetry"] = t

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
                if first:
                    self.event(status, threat=tr.threat_id, drone=agent)

    # ----------------------------------------------------------------- jobs
    def _send_ack(self, tr, agent, accepted, wave=None):
        msg = {"threat_id": tr.threat_id, "agent_id": agent, "wave_id": wave, "accepted": accepted,
               "job": f"ship/jobs/{tr.threat_id}"}
        if accepted:
            msg.update(self._job_msg(tr))
        self.radio.put(f"ship/ack/{agent}", json.dumps(msg))

    def _job_msg(self, tr):
        return {"threat_id": tr.threat_id, "status": "active" if tr.active else tr.status, "seq": tr.job_seq,
                "type": tr.type, "level": tr.level, "point": _xyz(tr.point), "t_engage": tr.t_engage,
                "cpa": _xyz(tr.cpa), "t_cpa": tr.t_cpa, "track": tr.track_msg(),
                "holders": dict(tr.confirmed), "n_slots": tr.level}

    def _publish_job(self, tr):
        tr.job_seq += 1
        self.radio.put(f"ship/jobs/{tr.threat_id}", json.dumps(self._job_msg(tr)))

    def _arbitrate(self):
        """Confirm the best pending awards per threat once the collection window has passed."""
        for tr in self.tracks.values():
            if not tr.pending or simclock.now() - tr.pending_since < ACK_WINDOW_S:
                continue
            need = tr.level - len(tr.hits) - len(tr.confirmed) if tr.status == "approved" else 0
            accepted, rejected = arbitrate({d: c for d, (c, _) in tr.pending.items()}, tr.confirmed,
                                           need, tr.level)
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
            p = position_at(tr.p0, tr.v, tr.t0, now)
            speed = math.hypot(tr.v[0], tr.v[1])
            miss = self.maneuver_rng.uniform(-self.args.max_miss, self.args.max_miss)
            old_point, old_t = tr.point, tr.t_engage
            tr.retrack(p, aim_velocity(p, speed, miss), now)
            tr.maneuvers += 1
            self.queue.update(tr.threat_id, tr.t_cpa)
            self.pub_tracks.put(json.dumps(tr.track_msg()))
            self.event("maneuver", threat=tr.threat_id, point_shift_m=round(math.dist(old_point, tr.point), 1),
                       t_engage_shift_s=round(tr.t_engage - old_t, 1))
            if tr.status == "approved":
                self._publish_job(tr)

    def _on_detonation(self, sample):
        d = self._parse(sample)
        with self.lock:
            self.expended.add(d["agent_id"])
            tr = self.tracks.get(d["threat_id"])
            if tr is None:
                return
            tp = position_at(tr.p0, tr.v, tr.t0, float(d["sim_time"]))
            miss = math.dist(tp, (d["x"], d["y"], d["z"]))
            tr.detonated[d["agent_id"]] = round(miss, 2)
            tr.confirmed.pop(d["agent_id"], None)
            if miss <= self.args.kill_radius:
                tr.hits.add(d["agent_id"])
            self.event("detonation", threat=tr.threat_id, drone=d["agent_id"], miss_m=round(miss, 2),
                       hit=miss <= self.args.kill_radius)
            if tr.active and len(tr.hits) >= tr.level:
                self._close(tr, "destroyed")

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

        tr = Track(f"T{self.detected}", kind, tt.level, p0, v, now, a.defended_radius)
        if a.maneuver_p > 0 and self.maneuver_rng.random() < a.maneuver_p:
            tr.maneuver_at = now + self.maneuver_rng.uniform(0.3, 0.6) * (tr.t_engage - now)
        self.tracks[tr.threat_id] = tr
        self.queue.push(tr.threat_id, tr.t_cpa)
        self.pub_tracks.put(json.dumps(tr.track_msg()))
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
        self.pub_tracks.put(json.dumps(tr.track_msg()))
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
        """Can `level` free drones reach the engagement point before t_engage?"""
        etas = []
        for d in self.free_drones():
            p = self.drones[d]["hb"].get("pose")
            if p:
                dist = math.dist((p["x"], p["y"], p["z"]), tr.point)
                etas.append(eta(dist, self.v_max, self.a_max) * ETA_MARGIN)
        tti = time_to_intercept(etas, tr.level)
        available = tr.t_engage - now
        ok = tti is not None and tti + ORDER_SLACK_S < available
        if tti is None:
            reason = f"needs {tr.level} free drones, {len(etas)} available"
        elif not ok:
            reason = f"TTI {tti:.1f}s + {ORDER_SLACK_S:.0f}s slack >= {available:.1f}s until engagement"
        else:
            reason = "feasible"
        return {"tti_s": None if tti is None else round(tti, 1), "available_s": round(available, 1),
                "free": len(etas), "feasible": ok, "reason": reason}

    # ------------------------------------------------------------- orders
    def approve(self, threat_id):
        with self.lock:
            tr = self.tracks.get(threat_id)
            now = self.sim_time
            if tr is None or now is None:
                return False, "unknown threat"
            if tr.status != "tracking":
                return False, f"threat is {tr.status}"
            f = self.feasibility(tr, now)
            if not f["feasible"]:
                self.event("approval_rejected", threat=threat_id, reason=f["reason"])
                return False, f["reason"]
            tr.status = "approved"
            tr.approved_wall = simclock.now()
            self._announce(tr, tr.level)
            self.event("approved", threat=threat_id, tti_s=f["tti_s"], available_s=f["available_s"])
            return True, "approved"

    def _announce(self, tr, required):
        self.order_seq += 1
        tr.orders.append(f"O{self.order_seq}")
        order = Threat(tr.threat_id, tr.type, tr.level, required, _xyz(tr.point), tr.t_engage)
        self.pub_orders.put(json.dumps({"wave_id": f"O{self.order_seq}", "threats": [order.to_dict()]}))
        tr.announces += 1
        tr.last_announce = simclock.now()

    def _engaged_by_heartbeat(self, tr):
        """Roster members whose latest heartbeat says they are engaging this threat."""
        return {d for d in self.members()
                if self.drones[d]["hb"].get("threat_id") == tr.threat_id
                and self.drones[d]["hb"].get("state") in ("engaging", "on_station")}

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
                now = self.sim_time
                if now is not None:
                    if self.args.max_threats != 0:
                        self._maybe_detect(now)
                    self._maneuver(now)
                    self._age_tracks(now)
                    self._arbitrate()
                    self._reconcile()
                    self._retry_underassigned(now)
                wall = simclock.now()
                if wall - last_job >= 1.0 / JOB_HZ:
                    last_job = wall
                    self._publish_jobs()
                if wall - last_roster >= 1.0 / ROSTER_HZ:
                    last_roster = wall
                    members = self.members()
                    self.pub_roster.put(json.dumps({"sim_time": now, "count": len(members), "members": members}))
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
        s = self.snapshot()
        engaged = [t for t in s["threats"] if t["announces"] > 0]
        lat = [t["full_award_ms"] for t in engaged if t["full_award_ms"] is not None]
        agree = [t["agreement"] for t in engaged if t["agreement"] is not None]
        counts = s["threat_counts"]
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
            "maneuvers": sum(t["maneuvers"] for t in s["threats"]),
            "missed_slots": sum(len(t["missed"]) for t in engaged),
            "agreement_mean": round(sum(agree) / len(agree), 3) if agree else None,
            "drones_expended": s["roster"]["expended"],
            "radio_rx_at_ship": s["radio_rx_at_ship"],
        }

    def snapshot(self):
        with self.lock:
            now = self.sim_time
            wall = simclock.now()
            threats = []
            order = self.queue.ordered()
            closed = [t for t in self.tracks.values() if not t.active]
            for tid in order + [t.threat_id for t in sorted(closed, key=lambda t: t.t0, reverse=True)[:10]]:
                tr = self.tracks[tid]
                row = {
                    "threat_id": tid, "type": tr.type, "level": tr.level, "status": tr.status,
                    "pos": _xyz(position_at(tr.p0, tr.v, tr.t0, now)) if now is not None else _xyz(tr.p0),
                    "v": _xyz(tr.v), "cpa": _xyz(tr.cpa), "cpa_dist": round(tr.cpa_dist, 1),
                    "tcpa_s": None if now is None else round(tr.t_cpa - now, 1),
                    "point": _xyz(tr.point),
                    "t_to_engage_s": None if now is None else round(tr.t_engage - now, 1),
                    "holders": sorted(tr.holders), "hits": sorted(tr.hits), "detonated": tr.detonated,
                    "missed": sorted(tr.missed), "announces": tr.announces,
                    "pending": sorted(tr.pending), "rejected": len(tr.rejected), "maneuvers": tr.maneuvers,
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
                               "telemetry": v.get("telemetry", {})})
            members = self.members()
            counts = defaultdict(int)
            for t in self.tracks.values():
                counts[t.status] += 1
            return {
                "sim_time": now,
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
    ap.add_argument("--maneuver-p", type=float, default=_env("THREAT_MANEUVER_P", 0.0, float),
                    help="probability that a threat turns once, re-aiming past the ship")
    ap.add_argument("--defended-radius", type=float, default=_env("DEFENDED_RADIUS_M", 45.0, float))
    ap.add_argument("--kill-radius", type=float, default=_env("KILL_RADIUS_M", 8.0, float))
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
