"""
MetricsNode — Reliable swarm telemetry collector.

Subscribes to:
  drone/*/sensors        → pose + velocity per agent (50 Hz from Gazebo bridge)
  swarm/agents/join      → agent spawn events
  swarm/agents/despawn   → agent despawn events

Tracks per-agent:
  - Distance travelled
  - Current speed
  - Time alive
  - Proximity collisions with other drones

Publishes:
  swarm/metrics/summary  → JSON summary every 5 s

Writes:
  /state/metrics_log.json  → full log on shutdown (or every 60 s)
"""

import zenoh
import json
import math
import os
import signal
import threading
import time
import logging
from pathlib import Path

# ---------------------------------------------------------------------------
logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s [MetricsNode] %(message)s",
    datefmt="%H:%M:%S"
)
log = logging.getLogger("MetricsNode")

# ---------------------------------------------------------------------------
# Constants
# ---------------------------------------------------------------------------
COLLISION_RADIUS_M  = 2.5    # drones closer than this = proximity collision
CLOSE_CALL_M        = 5.0    # pairs closer than this count as close calls; also the hash cell size
DEDUP_WINDOW_S      = 3.0    # min seconds between same-pair collision events (suppress sustained-contact spam)
PUBLISH_INTERVAL_S  = 5.0    # how often to publish swarm/metrics/summary
SAVE_INTERVAL_S     = 60.0   # how often to auto-save log to disk
COLLISION_CHECK_HZ  = float(os.environ.get("COLLISION_CHECK_HZ", "20.0"))
LOG_PATH            = Path(os.environ.get("METRICS_LOG_PATH", "/state/metrics_log.json"))


# ---------------------------------------------------------------------------
class AgentState:
    """Mutable state for one drone tracked by the metrics node."""

    def __init__(self, agent_id: str):
        self.agent_id        = agent_id
        self.alive           = True
        self.spawn_time      = time.monotonic()
        self.death_time      = None

        # Pose
        self.x = self.y = self.z = 0.0
        self.vx = self.vy = self.vz = 0.0
        self.speed = 0.0
        self._prev_x = self._prev_y = self._prev_z = None
        self.first_pose_received = False

        # Accumulators
        self.distance_m     = 0.0     # total distance flown
        self.collision_count = 0

        # Per-pair dedup: partner_id -> last collision monotonic time
        self._last_col_time: dict = {}

    # ------------------------------------------------------------------
    def update_pose(self, x, y, z, vx, vy, vz):
        self.x, self.y, self.z = x, y, z
        self.vx, self.vy, self.vz = vx, vy, vz
        self.speed = math.sqrt(vx * vx + vy * vy + vz * vz)

        if self.first_pose_received and self._prev_x is not None:
            dx = x - self._prev_x
            dy = y - self._prev_y
            dz = z - self._prev_z
            self.distance_m += math.sqrt(dx * dx + dy * dy + dz * dz)

        self._prev_x, self._prev_y, self._prev_z = x, y, z
        self.first_pose_received = True

    def time_alive_s(self) -> float:
        end = self.death_time if self.death_time else time.monotonic()
        return end - self.spawn_time

    def to_dict(self) -> dict:
        return {
            "agent_id":        self.agent_id,
            "alive":           self.alive,
            "time_alive_s":    round(self.time_alive_s(), 1),
            "position":        {"x": round(self.x, 2), "y": round(self.y, 2), "z": round(self.z, 2)},
            "speed_mps":       round(self.speed, 3),
            "distance_m":      round(self.distance_m, 2),
            "collision_count": self.collision_count,
        }


# ---------------------------------------------------------------------------
class MetricsNode:

    def __init__(self, session: zenoh.Session):
        self.session = session
        self._lock   = threading.Lock()

        self.running = True
        self._agents: dict = {}
        self._total_spawned    = 0
        self._total_despawned  = 0
        self._total_collisions = 0
        self._close_calls      = 0
        self._min_separation   = None   # closest true distance between two live drones (pairs < CLOSE_CALL_M)
        self._min_ship_range   = None   # closest any live drone came to the ship's centre (ground plane)
        self._start_time       = time.monotonic()
        self._events: list = []

        # Zenoh declarations — NOTE: Zenoh uses * (not +) for single-level wildcard
        # True poses come from the simulator's evaluation stream (sim/truth): drones' own sensor
        # frames no longer carry x,y once they localize themselves.
        self._sub_truth = session.declare_subscriber("sim/truth", self._on_truth)
        # Localization evaluation: drones report their x,y estimate and covariance (drone/{id}/loc);
        # compared with sim/truth extrapolated to the report's sim time.
        self._truth = {}                 # agent -> (sim_time, x, y, vx, vy)
        self._loc_errs = []              # recent position errors (m), for percentiles
        self._loc_level = {}             # drone -> highest error level logged (m)
        self._loc_stats = {"n": 0, "sum": 0.0, "max": 0.0, "nees_sum": 0.0, "within95": 0, "relocks": {}}
        self._sub_loc = session.declare_subscriber("drone/*/loc", self._on_loc)
        self._sub_join = session.declare_subscriber(
            "swarm/agents/join", self._on_join
        )
        self._sub_despawn = session.declare_subscriber(
            "swarm/agents/despawn", self._on_despawn
        )
        self._pub_summary = session.declare_publisher("swarm/metrics/summary")

        log.info("Subscribed to sim/truth, swarm/agents/join, swarm/agents/despawn")

    # -----------------------------------------------------------------------
    # Zenoh callbacks (called from Zenoh threads — must be fast and safe)
    # -----------------------------------------------------------------------

    def _on_truth(self, sample):
        """sim/truth: {sim_time, drones{id: [x, y, z, vx, vy, vz]}} for every live drone (10 Hz).  A drone
        missing from it has been despawned by the simulator (detonated or destroyed)."""
        try:
            payload = json.loads(bytes(sample.payload).decode("utf-8"))
            drones = payload.get("drones", {})
            t = float(payload.get("sim_time", 0.0))
            with self._lock:
                for agent_id, (x, y, z, vx, vy, vz) in drones.items():
                    self._truth[agent_id] = (t, x, y, vx, vy)
                    r = math.hypot(x, y)
                    if self._min_ship_range is None or r < self._min_ship_range:
                        self._min_ship_range = r
                for agent_id, (x, y, z, vx, vy, vz) in drones.items():
                    if agent_id not in self._agents:
                        self._agents[agent_id] = AgentState(agent_id)
                        self._total_spawned += 1
                        log.info(f"Auto-registered agent {agent_id} from sim/truth")
                    agent = self._agents[agent_id]
                    if agent.alive:
                        agent.update_pose(x, y, z, vx, vy, vz)
                for agent_id, agent in self._agents.items():
                    if agent.alive and agent.first_pose_received and agent_id not in drones:
                        agent.alive = False
                        agent.death_time = time.monotonic()
                        self._total_despawned += 1
                        self._events.append({"type": "despawn", "agent_id": agent_id, "reason": "simulator",
                                             "time": time.time()})
                        log.info(f"Agent gone from sim/truth: {agent_id}")
        except Exception as e:
            log.warning(f"Error in _on_truth: {e}")

    def _on_loc(self, sample):
        """A drone's localization report: error against truth, and consistency (NEES, 2 dof)."""
        try:
            agent_id = str(sample.key_expr).split("/")[1]
            m = json.loads(bytes(sample.payload).decode("utf-8"))
            with self._lock:
                tr = self._truth.get(agent_id)
                if tr is None or abs(m["sim_time"] - tr[0]) > 0.5:
                    return
                dt = m["sim_time"] - tr[0]
                ex = m["est"][0] - (tr[1] + tr[3] * dt)
                ey = m["est"][1] - (tr[2] + tr[4] * dt)
                err = math.hypot(ex, ey)
                a, b, c = m["cov"]
                det = a * c - b * b
                nees = (c * ex * ex - 2 * b * ex * ey + a * ey * ey) / det if det > 1e-12 else float("inf")
                st = self._loc_stats
                st["n"] += 1
                st["sum"] += err
                st["max"] = max(st["max"], err)
                st["nees_sum"] += min(nees, 1e6)
                st["within95"] += nees <= 5.99
                st["relocks"][agent_id] = m.get("relocks", 0)
                # Log each drone's error the first time it crosses 5, 10, 30, 100 m (diagnosis)
                level = max([x for x in (5.0, 10.0, 30.0, 100.0) if err >= x], default=0.0)
                if level > self._loc_level.get(agent_id, 0.0):
                    self._loc_level[agent_id] = level
                    log.info(f"loc error {agent_id} >= {level:.0f} m at t={m['sim_time']:.1f}: {err:.1f} m, "
                             f"claimed sigma {math.sqrt(max(a, c)):.2f} m, status {m.get('status')}, "
                             f"hops {m.get('hops')}, relocks {m.get('relocks')}, noise {m.get('noise_sigma')}, "
                             f"flags {m.get('flags')}, excluded {m.get('excluded')}, "
                             f"est ({m['est'][0]:.1f}, {m['est'][1]:.1f}), truth ({tr[1]:.1f}, {tr[2]:.1f})")
                self._loc_errs.append(err)
                if len(self._loc_errs) > 20000:
                    del self._loc_errs[:5000]
        except Exception as e:
            log.warning(f"Error in _on_loc: {e}")

    def _loc_summary(self) -> dict:
        st = self._loc_stats
        if not st["n"]:
            return {}
        errs = sorted(self._loc_errs)
        return {"loc_reports": st["n"], "loc_err_mean_m": round(st["sum"] / st["n"], 3),
                "loc_err_p95_m": round(errs[int(0.95 * (len(errs) - 1))], 3), "loc_err_max_m": round(st["max"], 3),
                "loc_nees_mean": round(st["nees_sum"] / st["n"], 2),
                "loc_within95": round(st["within95"] / st["n"], 3),
                "loc_relocks": sum(st["relocks"].values())}

    def _on_join(self, sample):
        try:
            payload  = json.loads(bytes(sample.payload).decode("utf-8"))
            agent_id = payload.get("agent_id", "unknown")

            with self._lock:
                if agent_id not in self._agents:
                    self._agents[agent_id] = AgentState(agent_id)
                    self._total_spawned += 1
                else:
                    # Re-join: reset
                    self._agents[agent_id].alive      = True
                    self._agents[agent_id].spawn_time = time.monotonic()
                    self._agents[agent_id].death_time = None

                self._events.append({
                    "type":     "spawn",
                    "agent_id": agent_id,
                    "time":     time.time()
                })

            log.info(f"Agent joined: {agent_id}  (total spawned: {self._total_spawned})")

        except Exception as e:
            log.warning(f"Error in _on_join: {e}")

    def _on_despawn(self, sample):
        try:
            payload  = json.loads(bytes(sample.payload).decode("utf-8"))
            agent_id = payload.get("agent_id", "unknown")
            reason   = payload.get("reason", "unknown")

            with self._lock:
                if agent_id in self._agents:
                    self._agents[agent_id].alive      = False
                    self._agents[agent_id].death_time = time.monotonic()
                self._total_despawned += 1
                self._events.append({
                    "type":     "despawn",
                    "agent_id": agent_id,
                    "reason":   reason,
                    "time":     time.time()
                })

            log.info(f"Agent despawned: {agent_id}  reason={reason}")

        except Exception as e:
            log.warning(f"Error in _on_despawn: {e}")

    # -----------------------------------------------------------------------
    # Asynchronous Collision Detection (Spatial Hashing)
    # -----------------------------------------------------------------------

    def check_collisions_spatial_hash(self):
        """
        Asynchronously builds a 3D spatial hash from a state snapshot and checks
        candidate pairs for proximity collisions.
        """
        # Step 1: Copy state snapshot under lock
        snapshots = {}
        with self._lock:
            for aid, agent in self._agents.items():
                if agent.alive and agent.first_pose_received:
                    snapshots[aid] = (agent.x, agent.y, agent.z)

        if len(snapshots) < 2:
            return 0, 0

        # Step 2: Build spatial hash grid
        cell_size = CLOSE_CALL_M
        grid = {}
        for aid, (x, y, z) in snapshots.items():
            cell_key = (
                math.floor(x / cell_size),
                math.floor(y / cell_size),
                math.floor(z / cell_size)
            )
            if cell_key not in grid:
                grid[cell_key] = []
            grid[cell_key].append(aid)

        # Step 3: Gather unique candidate pairs from 3D neighbor cells (3x3x3 neighborhood)
        candidate_pairs = set()
        for cell_key, members in grid.items():
            cx, cy, cz = cell_key
            for dx in (-1, 0, 1):
                for dy in (-1, 0, 1):
                    for dz in (-1, 0, 1):
                        neighbor_key = (cx + dx, cy + dy, cz + dz)
                        if neighbor_key in grid:
                            for a1 in members:
                                for a2 in grid[neighbor_key]:
                                    if a1 < a2:
                                        candidate_pairs.add((a1, a2))

        # Step 4: Perform exact distance check on candidate pairs
        now = time.monotonic()
        for a1_id, a2_id in candidate_pairs:
            p1 = snapshots[a1_id]
            p2 = snapshots[a2_id]
            dx = p1[0] - p2[0]
            dy = p1[1] - p2[1]
            dz = p1[2] - p2[2]
            dist = math.sqrt(dx * dx + dy * dy + dz * dz)
            if dist < CLOSE_CALL_M:
                with self._lock:
                    a1, a2 = self._agents.get(a1_id), self._agents.get(a2_id)
                    if a1 and a2 and a1.alive and a2.alive:
                        if self._min_separation is None or dist < self._min_separation:
                            self._min_separation = dist
                        key = ("close", a2_id)
                        if now - a1._last_col_time.get(key, 0.0) >= DEDUP_WINDOW_S:
                            a1._last_col_time[key] = now
                            self._close_calls += 1

            if dist < COLLISION_RADIUS_M:
                with self._lock:
                    agent1 = self._agents.get(a1_id)
                    agent2 = self._agents.get(a2_id)
                    if not agent1 or not agent2 or not agent1.alive or not agent2.alive:
                        continue

                    last1 = agent1._last_col_time.get(a2_id, 0.0)
                    last2 = agent2._last_col_time.get(a1_id, 0.0)
                    last = max(last1, last2)

                    if now - last >= DEDUP_WINDOW_S:
                        agent1._last_col_time[a2_id] = now
                        agent2._last_col_time[a1_id] = now
                        agent1.collision_count += 1
                        agent2.collision_count += 1
                        self._total_collisions += 1
                        self._events.append({
                            "type":     "collision",
                            "agents":   [a1_id, a2_id],
                            "dist_m":   round(dist, 3),
                            "time":     time.time()
                        })
                        log.warning(
                            f"PROXIMITY COLLISION: {a1_id} <-> {a2_id}  "
                            f"dist={dist:.2f}m  total={self._total_collisions}"
                        )

        return len(candidate_pairs), len(candidate_pairs)

    # -----------------------------------------------------------------------
    # Reporting
    # -----------------------------------------------------------------------

    def _build_summary(self) -> dict:
        with self._lock:
            elapsed  = time.monotonic() - self._start_time
            alive    = [a for a in self._agents.values() if a.alive]
            avg_dist = (sum(a.distance_m for a in alive) / len(alive)) if alive else 0.0
            avg_spd  = (sum(a.speed      for a in alive) / len(alive)) if alive else 0.0

            return {
                "timestamp":         time.time(),
                "elapsed_s":         round(elapsed, 1),
                "active_agents":     len(alive),
                "total_spawned":     self._total_spawned,
                "total_despawned":   self._total_despawned,
                "total_collisions":  self._total_collisions,
                "close_calls":       self._close_calls,
                "min_separation_m":  None if self._min_separation is None else round(self._min_separation, 2),
                "min_ship_range_m":  None if self._min_ship_range is None else round(self._min_ship_range, 2),
                **self._loc_summary(),
                "avg_distance_m":    round(avg_dist, 2),
                "avg_speed_mps":     round(avg_spd, 3),
                "agents":            [a.to_dict() for a in sorted(
                                          self._agents.values(),
                                          key=lambda a: a.agent_id)],
                "recent_events":     list(self._events[-20:]),
            }
    def _get_agent_color(self, agent_id: str) -> str:
        # Match colors from GazeboSimulator.cpp
        colors = [
            "\033[91m",         # 0: Vivid Red
            "\033[92m",         # 1: Vivid Green
            "\033[94m",         # 2: Vivid Blue
            "\033[93m",         # 3: Yellow
            "\033[96m",         # 4: Cyan
            "\033[95m",         # 5: Magenta
            "\033[38;5;214m",   # 6: Orange
            "\033[38;5;129m",   # 7: Purple
            "\033[38;5;48m",    # 8: Spring Green
        ]
        color_idx = 0
        try:
            # Parse trailing number (e.g. 'drone_3' -> 3)
            num_str = "".join(filter(str.isdigit, agent_id))
            if num_str:
                color_idx = (int(num_str) - 1) % len(colors)
            else:
                color_idx = hash(agent_id) % len(colors)
        except Exception:
            color_idx = hash(agent_id) % len(colors)
        return colors[color_idx]

    def print_dashboard(self):
        s = self._build_summary()
        print(
            f"\n{'─'*60}\n"
            f"  DeNDDron Metrics  |  t+{s['elapsed_s']:.0f}s\n"
            f"{'─'*60}\n"
            f"  Active agents   : {s['active_agents']}\n"
            f"  Total spawned   : {s['total_spawned']}\n"
            f"  Total despawned : {s['total_despawned']}\n"
            f"  Collisions      : {s['total_collisions']}\n"
            f"  Avg distance    : {s['avg_distance_m']:.1f} m\n"
            f"  Avg speed       : {s['avg_speed_mps']:.2f} m/s\n"
            f"{'─'*60}",
            flush=True
        )
        for a in s["agents"]:
            status = "ALIVE" if a["alive"] else "DEAD "
            color = self._get_agent_color(a['agent_id'])
            reset = "\033[0m"
            print(
                f"  [{status}] {color}{a['agent_id']:12s}{reset}  "
                f"pos=({a['position']['x']:6.1f},{a['position']['y']:6.1f},{a['position']['z']:5.1f})  "
                f"spd={a['speed_mps']:.2f}m/s  "
                f"dist={a['distance_m']:.1f}m  "
                f"cols={a['collision_count']}",
                flush=True
            )

    def publish_summary(self):
        try:
            summary = self._build_summary()
            self._pub_summary.put(json.dumps(summary))
        except Exception as e:
            log.debug(f"publish_summary error: {e}")

    def save_log(self):
        try:
            LOG_PATH.parent.mkdir(parents=True, exist_ok=True)
            with LOG_PATH.open("w") as f:
                json.dump(self._build_summary(), f, indent=2)
            log.info(f"Metrics log saved → {LOG_PATH}")
        except Exception as e:
            log.warning(f"Failed to save log: {e}")

    def _collision_loop(self):
        interval = 1.0 / max(0.1, COLLISION_CHECK_HZ)
        while self.running:
            try:
                self.check_collisions_spatial_hash()
            except Exception as e:
                log.warning(f"Error in collision loop: {e}")
            time.sleep(interval)

    def run(self):
        last_publish = 0.0
        last_save    = 0.0
        last_print   = 0.0

        self.running = True
        worker = threading.Thread(target=self._collision_loop, daemon=True)
        worker.start()

        log.info(f"MetricsNode running. Collecting swarm telemetry (spatial hash at {COLLISION_CHECK_HZ} Hz)...")
        try:
            while True:
                now = time.monotonic()

                if now - last_publish >= PUBLISH_INTERVAL_S:
                    self.publish_summary()
                    last_publish = now

                if now - last_print >= PUBLISH_INTERVAL_S:
                    self.print_dashboard()
                    last_print = now

                if now - last_save >= SAVE_INTERVAL_S:
                    self.save_log()
                    last_save = now

                time.sleep(0.5)

        except KeyboardInterrupt:
            pass
        finally:
            self.running = False
            log.info("Shutting down — saving final metrics log...")
            self.save_log()


# ---------------------------------------------------------------------------
def main():
    # Running as PID 1 in the container: SIGTERM is ignored unless handled.
    # Route it through the KeyboardInterrupt path so the final log is saved.
    signal.signal(signal.SIGTERM, signal.default_int_handler)
    router = os.environ.get("ZENOH_ROUTER_IP", None)

    log.info("Connecting to Zenoh...")
    conf = zenoh.Config()
    conf.insert_json5("mode", '"client"')

    if router:
        conf.insert_json5("connect/endpoints", f'["{router}"]')
        log.info(f"Using router: {router}")
    else:
        log.warning("ZENOH_ROUTER_IP not set — relying on scouting (may be unreliable)")

    # Retry connection loop (Docker startup ordering)
    session = None
    for attempt in range(10):
        try:
            session = zenoh.open(conf)
            break
        except Exception as e:
            log.warning(f"Zenoh connect attempt {attempt+1}/10 failed: {e}. Retrying in 2s...")
            time.sleep(2)

    if session is None:
        log.error("Could not connect to Zenoh after 10 attempts. Exiting.")
        raise SystemExit(1)

    log.info("Connected to Zenoh.")
    node = MetricsNode(session)
    node.run()
    session.close()


if __name__ == "__main__":
    main()
