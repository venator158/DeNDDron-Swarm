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
        self._start_time       = time.monotonic()
        self._events: list = []

        # Zenoh declarations — NOTE: Zenoh uses * (not +) for single-level wildcard
        self._sub_sensors = session.declare_subscriber(
            "drone/*/sensors", self._on_sensors
        )
        self._sub_join = session.declare_subscriber(
            "swarm/agents/join", self._on_join
        )
        self._sub_despawn = session.declare_subscriber(
            "swarm/agents/despawn", self._on_despawn
        )
        self._pub_summary = session.declare_publisher("swarm/metrics/summary")

        log.info("Subscribed to drone/*/sensors, swarm/agents/join, swarm/agents/despawn")

    # -----------------------------------------------------------------------
    # Zenoh callbacks (called from Zenoh threads — must be fast and safe)
    # -----------------------------------------------------------------------

    def _on_sensors(self, sample):
        try:
            # Key is "drone/{agent_id}/sensors"
            key_str  = str(sample.key_expr)
            parts    = key_str.split("/")
            if len(parts) < 3:
                return
            agent_id = parts[1]

            payload = json.loads(bytes(sample.payload).decode("utf-8"))
            pose    = payload.get("pose", {})
            x  = pose.get("x",  0.0);  y  = pose.get("y",  0.0);  z  = pose.get("z",  0.0)
            vx = pose.get("vx", 0.0);  vy = pose.get("vy", 0.0);  vz = pose.get("vz", 0.0)

            with self._lock:
                if agent_id not in self._agents:
                    # First contact — register the agent
                    self._agents[agent_id] = AgentState(agent_id)
                    self._total_spawned += 1
                    log.info(f"Auto-registered agent {agent_id} from sensor data")

                agent = self._agents[agent_id]
                agent.update_pose(x, y, z, vx, vy, vz)

                # Proximity collision detection against all other alive agents
                self._check_collisions_locked(agent_id)

        except Exception as e:
            log.warning(f"Error in _on_sensors: {e}")

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
        cell_size = COLLISION_RADIUS_M
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
