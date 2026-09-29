import hashlib
import json
import math
import time
from collections import OrderedDict
import threading
import numpy as np
import logging
import os
from voxel_map import VoxelMap
from path_planning import APFStrategy, ORCAStrategy
from timing import TimingManager, TimingState
from auction import AuctionManager
from telemetry import Telemetry
from threats import ETA_MARGIN, ORDER_SLACK_S, Threat, eta, slot_point
from links import open_onboard
from radio_process import RadioProcess
import simclock

logging.basicConfig(level=logging.INFO)
logger = logging.getLogger("DenddronAgent")

class DenddronAgent:
    HEARTBEAT_HZ = 2.0
    TELEMETRY_HZ = 1.0
    LINK_TIMEOUT_S = 3.0       # no radio traffic for this long = disconnected from the swarm
    AWARD_RESENDS = 2          # extra copies of each award/withdrawal, one per heartbeat tick (lossy radio)
    ACK_TIMEOUT_S = 3.0        # sim s: a won job the ship has not confirmed by then is abandoned
    RETARGET_MIN_M = 0.5       # job updates that move our slot less than this do not change the goal
    SLOT_RADIUS = 4.0          # m, spacing of drones around an engagement point
    DETONATE_RADIUS = 8.0      # m, must be this close to the slot at t_engage to detonate (= kill radius)
    KILL_RADIUS = 8.0          # m, a detonation destroys any drone this close (friendly fire included)
    CLEARANCE_M = 12.0         # m, non-job drones are kept this far from a detonation (ship's zones)
    ZONE_MARGIN_M = 2.0        # m, idle drones evade to this far beyond a zone's edge
    ZONE_HOT_S = 8.0           # s before t_engage from which moving drones may not enter another job's zone
    # Experiment switch: ZONE_KEEPOUT=0 turns prevention off (friendly fire and the final check stay on).
    KEEPOUT = os.environ.get("ZONE_KEEPOUT", "1") != "0"

    def __init__(self, agent_id: str, sim_bus_locator: str = None):
        self.agent_id = agent_id

        # Onboard bus: this drone's sensors/actuators (simulator).  Radio: the swarm and the ship.
        # The radio runs in its own process so a radio failure cannot freeze flight control.
        logger.info(f"[{self.agent_id}] Opening onboard bus ({sim_bus_locator}) and radio (peer-to-peer, own process)...")
        self.radio = RadioProcess()
        self.onboard = open_onboard(sim_bus_locator)
        
        # --- Internal State ---
        self.running = True
        self.state_lock = threading.RLock()
        self.voxel_map = VoxelMap()
        self.current_pose = None
        self.current_goal = None
        self.current_job = None
        self.step_count = 0
        self.sensor_frame_count = 0
        self.current_time = None  # Gazebo sim-time (seconds), NOT Unix wall-clock
        self.latest_voxel_summary = {
            "hits": 0,
            "placed": 0,
            "visible": 0,
            "nearest": []
        }

        # --- Runtime config loading ---
        config_path = os.environ.get(
            "SWARM_RUNTIME_CONFIG",
            os.path.join(os.path.dirname(__file__), "..", "..", "config", "swarm_runtime.json")
        )
        if not os.path.exists(config_path):
            raise FileNotFoundError(f"[{self.agent_id}] Runtime config not found: {config_path}")

        try:
            with open(config_path, "r", encoding="utf-8") as f:
                runtime_cfg = json.load(f)
        except Exception as e:
            raise RuntimeError(f"[{self.agent_id}] Failed to load runtime config: {e}") from e

        def _require_keys(section_name: str, values: dict, required_keys: list):
            missing = [k for k in required_keys if k not in values]
            if missing:
                raise ValueError(
                    f"[{self.agent_id}] Missing required config keys in {section_name}: {missing}"
                )

        global_cfg = runtime_cfg.get("defaults", {})
        my_cfg = runtime_cfg.get("agents", {}).get(self.agent_id, {})

        if not global_cfg:
            raise ValueError(f"[{self.agent_id}] Missing required config section: defaults")

        # --- Goal State ---
        # Latched flag: once True, the drone stops forever until goal changes.
        # This prevents jitter from re-triggering APF after arrival.
        self._goal_reached = False
        self._prev_to_goal_vec = None
        goal_cfg = dict(global_cfg.get("goal_control", {}))
        goal_cfg.update(my_cfg.get("goal_control", {}))
        _require_keys(
            "defaults.goal_control",
            goal_cfg,
            ["tolerance", "stop_radius", "tolerance_xy", "tolerance_z", "settle_ticks"],
        )
        self.GOAL_TOLERANCE = float(goal_cfg["tolerance"])
        self.GOAL_STOP_RADIUS = float(goal_cfg["stop_radius"])
        self.GOAL_TOLERANCE_XY = float(goal_cfg["tolerance_xy"])
        self.GOAL_TOLERANCE_Z = float(goal_cfg["tolerance_z"])
        self.GOAL_SETTLE_TICKS = int(goal_cfg["settle_ticks"])
        self._goal_hold_ticks = 0

        # --- Dynamic Drone Kinematics ---
        self.last_velocity = np.zeros(3)
        self.spawn_pose = {"x": 0.0, "y": 0.0, "z": 0.0}
        self.kinematics = dict(global_cfg.get("kinematics", {}))
        self.kinematics.update(my_cfg.get("kinematics", {}))
        _require_keys(
            "defaults.kinematics",
            self.kinematics,
            ["max_velocity", "max_acceleration", "max_z", "min_z", "max_service_radius"],
        )
        logger.info(f"[{self.agent_id}] Loaded kinematic limits: {self.kinematics}")

        # --- Agent-specific spawn/goal ---
        if "spawn" in my_cfg:
            self.spawn_pose = my_cfg["spawn"]

        if "goal" in my_cfg:
            self.current_goal = my_cfg["goal"]
            logger.info(f"[{self.agent_id}] Assigned static goal from runtime config: {self.current_goal}")

        # --- Path Planner: strategy selection ---
        planner_cfg = dict(global_cfg.get("path_planning", {}))
        planner_cfg.update(my_cfg.get("path_planning", {}))
        planner_cfg["min_z"] = self.kinematics["min_z"]
        planner_cfg["max_z"] = self.kinematics["max_z"]
        planner_cfg["max_velocity"] = self.kinematics["max_velocity"]

        algorithm = planner_cfg.get("algorithm", "apf").lower()
        if algorithm == "orca":
            logger.info(f"[{self.agent_id}] Using ORCA path planning strategy")
            self.path_planner = ORCAStrategy()
            _require_keys(
                "defaults.path_planning",
                planner_cfg,
                [
                    "algorithm",
                    "step_size",
                    "goal_tolerance",
                    "braking_radius",
                    "ship_keepout_radius",
                    "velocity_smoothing",
                ],
            )
        else:
            logger.info(f"[{self.agent_id}] Using APF path planning strategy")
            self.path_planner = APFStrategy()
            _require_keys(
                "defaults.path_planning",
                planner_cfg,
                [
                    "algorithm",
                    "attractive_gain",
                    "repulsive_gain",
                    "influence_radius",
                    "step_size",
                    "goal_tolerance",
                    "braking_radius",
                    "ship_keepout_radius",
                    "ship_influence_radius",
                    "apf_exponential_decay",
                    "apf_inverse_square_scale",
                    "apf_stuck_growth_rate",
                    "min_movement",
                    "stuck_threshold",
                    "velocity_smoothing",
                ],
            )
        self.planner_cfg = planner_cfg
        self.path_planner.configure(planner_cfg)

        self.max_control_dt = float(planner_cfg.get("max_control_dt", 0.2))
        self.sensor_timeout_s = float(planner_cfg.get("sensor_timeout_s", 0.5))
        self.control_rate_hz = 50.0   # per simulated second (the loop runs SIM_RTF times faster in wall time)
        self.timing = TimingManager(
            max_control_dt=self.max_control_dt,
            sensor_timeout_s=self.sensor_timeout_s,
            control_period_s=1.0 / self.control_rate_hz,
        )
        
        # --- Threat engagement (priority-ordered wave auction) ---
        auction_cfg = global_cfg.get("auction", {})
        self.auction = AuctionManager(
            self.agent_id, bid_window_s=float(auction_cfg.get("bid_window_s", 1.0))
        )
        self._auction_lock = threading.Lock()
        self._pending_wave_id = None   # wave we bid in and whose result we await
        self.engaged_threat = None     # Threat we are flying to intercept
        self.engagement = None         # {"threat", "slot_point", "t_engage"} once assigned
        # Drones are expendable: once destroyed the agent stays down for good.
        self.destroyed = False

        # --- Radio link and membership ---
        self.telemetry = Telemetry(1.0 / (self.control_rate_hz * simclock.RTF))
        self._last_radio_rx = None     # simclock time of the last message from the ship or a peer
        self.peers = {}                # peer drone id -> simclock time any message from it was last heard
        self._peer_help = {}           # peer drone id -> (heartbeat payload, simclock time) it asked us to relay
        self._started = simclock.now()
        self._roster_time = None       # simclock time of the last roster
        self._assign_hashes = OrderedDict()   # order id -> short hash of the assignment we computed
        self.roster = []               # drone ids the ship last reported hearing
        self.roster_relayed = set()    # of those, the ones it hears only through peer relays
        self._missing_from_roster = 0
        self._link_was_up = False
        self._award_resends = {}       # threat_id -> [award payload, copies left]; newest status wins
        # Engagement: set on winning an auction (tentative), confirmed by the ship's ACK or by appearing
        # in the job topic's holders, updated from the job topic.  Replaced whole, under _eng_lock.
        self._eng_lock = threading.RLock()
        self._job_sub = None           # subscription to ship/jobs/{threat_id} while engaged
        self._award_lock = threading.Lock()

        # --- Onboard bus (simulator) ---
        self.pub_cmd_vel    = self.onboard.declare_publisher(f"swarm/{self.agent_id}/cmd_vel")
        self.pub_detonation = self.onboard.declare_publisher("sim/detonation")
        self.sub_sensors = self.onboard.declare_subscriber(
            f"drone/{self.agent_id}/sensors",
            self._on_sensor_data
        )

        # --- Radio (peer-to-peer) ---
        self.pub_bids      = self.radio.declare_publisher("swarm/bids")
        self.pub_awards    = self.radio.declare_publisher("swarm/awards")
        self.pub_heartbeat = self.radio.declare_publisher(f"swarm/heartbeat/{self.agent_id}")
        self.pub_telemetry = self.radio.declare_publisher(f"swarm/telemetry/{self.agent_id}")
        self.pub_relay     = self.radio
        self.sub_threats   = self.radio.declare_subscriber("swarm/threats", self._on_threat_wave)
        self.sub_bids      = self.radio.declare_subscriber("swarm/bids", self._on_bid_received)
        self.sub_awards    = self.radio.declare_subscriber("swarm/awards", self._on_award_received)
        self.sub_roster    = self.radio.declare_subscriber("ship/roster", self._on_roster)
        # Heartbeats go to the ship only (drones do not subscribe to them), so radio load grows with N,
        # not N^2.  A drone the ship cannot hear also publishes on heartbeat_help, which every drone
        # hears, and drones in the roster relay it (_relay_unheard_peers).
        self.pub_help      = self.radio.declare_publisher(f"swarm/heartbeat_help/{self.agent_id}")
        self.sub_help      = self.radio.declare_subscriber("swarm/heartbeat_help/*", self._on_peer_help)
        self.sub_ack       = self.radio.declare_subscriber(f"ship/ack/{self.agent_id}", self._on_ack)
        # Engagement zones: keep clear of other jobs' detonations (_service_zones, _keepout).
        self.zones = {}                # threat_id -> (zone, simclock time received)
        self.sub_zones     = self.radio.declare_subscriber("ship/zones", self._on_zones)
        # Physics: detonations of other jobs within KILL_RADIUS destroy this drone (friendly fire).
        self.sub_blasts    = self.onboard.declare_subscriber("sim/detonation", self._on_blast)
        self.pub_damage    = self.onboard.declare_publisher("sim/damage")

        # Start background threads
        self.control_thread = threading.Thread(target=self._reflex_control_loop)
        self.control_thread.start()
        self.heartbeat_thread = threading.Thread(target=self._heartbeat_loop, daemon=True)
        self.heartbeat_thread.start()

        self._announce_join()

    def _radio_put(self, publisher, topic: str, payload: dict):
        data = json.dumps(payload)
        publisher.put(data)
        self.telemetry.tx(topic, len(data))

    def _announce_join(self):
        """Tell the simulator to spawn this drone."""
        pub_join = self.onboard.declare_publisher("swarm/agents/join")
        payload = {"agent_id": self.agent_id, "type": "quadrotor"}
        pub_join.put(json.dumps(payload))
        logger.info(f"[{self.agent_id}] Joined the swarm.")
        pub_join.undeclare()

    def set_goal(self, goal: dict):
        """
        Assign a new goal. Always resets the goal-reached latch so the drone
        will start moving again. Call this instead of writing current_goal directly.
        """
        with self.state_lock:
            self.current_goal = goal
            self._goal_reached = False
            self._goal_hold_ticks = 0
            self._prev_to_goal_vec = None
            self.last_velocity = np.zeros(3)   # reset momentum so it doesn't coast
        logger.info(f"[{self.agent_id}] New goal set: {goal}")

    # ==========================================
    # PILLAR 1: EYES (Obstacle Perception)
    # ==========================================
    def _on_sensor_data(self, sample):
        if self.destroyed:
            return
        try:
            payload = json.loads(bytes(sample.payload).decode('utf-8'))

            sim_time = payload.get("sim_time", None)
            pose = payload.get("pose", {})
            lidar_data = payload.get("lidar")   # present on every 5th frame (10 Hz)

            current_pose = {
                "x": pose.get("x", 0.0),
                "y": pose.get("y", 0.0),
                "z": pose.get("z", 0.0),
                "yaw": pose.get("yaw", 0.0)
            }

            accepted, timing_state = self.timing.record_sensor(sim_time)
            if not accepted:
                logger.warning(
                    f"[{self.agent_id}] Dropping sensor frame: timing state={timing_state.value}, "
                    f"sim_time={sim_time!r}"
                )
                return

            with self.state_lock:
                self.current_time = float(sim_time) if sim_time is not None else None
                simclock.observe(self.current_time)
                self.current_pose = current_pose

                if timing_state == TimingState.TIME_RESET:
                    self.last_velocity = np.zeros(3)
                    self._goal_hold_ticks = 0
                    self._prev_to_goal_vec = None

            # Perception feeds only the planner, which runs only with a goal: a drone holding
            # station sends zero velocity and never reads the map.  Voxels expire after 0.5 s, so
            # there is nothing to keep fresh either; the first scan after tasking (<= 0.1 s)
            # rebuilds the map.  Skipping it while idle was ~75% of an idle drone's CPU.
            if lidar_data and self.current_goal is not None:
                t0 = time.perf_counter()
                self.voxel_map.cleanup_stale_data(max_age=0.5, current_time=self.current_time)
                self._process_lidar(lidar_data, current_pose)
                self.telemetry.perception(time.perf_counter() - t0)

        except Exception as e:
            logger.debug(f"[{self.agent_id}] Error processing sensor data: {e}")

    def _process_lidar(self, scan: dict, pose: dict):
        """scan: {"angle_step", "ranges": [...], "hits": [0/1, ...]}; ray i points at i * angle_step."""
        ranges = scan.get("ranges") or []
        if not ranges:
            return
        angle_step = float(scan.get("angle_step", 2.0 * np.pi / len(ranges)))
        hits = scan.get("hits") or [0] * len(ranges)
        lidar_rays = [
            {"angle": i * angle_step, "distance": r, "intensity": 0.9 if h else 0.2}
            for i, (r, h) in enumerate(zip(ranges, hits))
        ]

        agent_x = pose.get("x", 0.0)
        agent_y = pose.get("y", 0.0)
        agent_z = pose.get("z", 0.0)
        yaw     = pose.get("yaw", 0.0)
        lidar_hits     = 0
        placed_occupied = 0

        rays_data = []
        for ray in lidar_rays:
            try:
                angle    = ray.get("angle", 0.0)
                distance = ray.get("distance", 0.0)
                intensity = ray.get("intensity", 0.0)

                if distance <= 0 or distance > 50:
                    continue

                world_angle = yaw + angle
                ray_x = agent_x + distance * np.cos(world_angle)
                ray_y = agent_y + distance * np.sin(world_angle)
                ray_z = agent_z

                is_hit = intensity > 0.5
                if is_hit:
                    lidar_hits += 1
                    placed_occupied += 1

                rays_data.append((agent_x, agent_y, agent_z, ray_x, ray_y, ray_z, is_hit))

            except Exception as e:
                logger.debug(f"[{self.agent_id}] Error processing ray: {e}")

        if rays_data:
            self.voxel_map.batch_raytrace(rays_data, current_time=self.current_time)

        visible_obstacles = self.voxel_map.get_nearby_obstacles(agent_x, agent_y, agent_z, radius=12.0)
        nearest = []
        if visible_obstacles:
            curr = np.array([agent_x, agent_y, agent_z])
            with_dist = sorted(
                [(float(np.linalg.norm(curr - np.array([o["x"], o["y"], o["z"]]))), o)
                 for o in visible_obstacles],
                key=lambda t: t[0]
            )
            nearest = [
                {"d": round(d, 2), "x": round(o["x"], 2), "y": round(o["y"], 2), "z": round(o["z"], 2)}
                for d, o in with_dist[:3]
            ]

        self.latest_voxel_summary = {
            "hits": lidar_hits,
            "placed": placed_occupied,
            "visible": len(visible_obstacles),
            "nearest": nearest,
        }

        self.sensor_frame_count += 1
        if self.sensor_frame_count % 50 == 0:
            stats = self.voxel_map.get_stats()
            logger.info(
                f"[{self.agent_id}] Voxel detection: hits={lidar_hits}, "
                f"placed_occupied={placed_occupied}, visible={len(visible_obstacles)}, "
                f"nearest={nearest}, map_stats={stats}"
            )

        logger.debug(f"[{self.agent_id}] Voxel map: {self.voxel_map.get_stats()}")

    # ==========================================
    # PILLAR 2: REFLEXES (APF & Navigation)
    # ==========================================
    def _reflex_control_loop(self):
        """
        Runs at 50 Hz. Computes APF velocity and publishes cmd_vel.
        Explicitly handles simulation time vs wall time across explicit timing states:
        FIRST_FRAME, NORMAL, PAUSED_ZERO_DT, OUT_OF_ORDER, TIME_RESET, LARGE_DT, MISSING_TIME.
        """
        rate_hz   = self.control_rate_hz * simclock.RTF
        sleep_time = 1.0 / rate_hz
        next_tick = time.monotonic()

        def _sleep_to_next_tick():
            nonlocal next_tick, last_tick
            now = time.monotonic()
            self.telemetry.loop_tick(now - last_tick, now - tick_start)
            last_tick = now
            next_tick += sleep_time
            remaining = next_tick - time.monotonic()
            if remaining > 0.0:
                time.sleep(remaining)
            else:
                next_tick = time.monotonic()

        _zero_cmd = {
            "linear":  {"x": 0.0, "y": 0.0, "z": 0.0},
            "angular": {"x": 0.0, "y": 0.0, "z": 0.0}
        }

        last_tick = time.monotonic()
        while self.running:
            tick_start = time.monotonic()
            self._service_auctions()
            self._check_engagement()
            self._service_zones()
            if self.destroyed:
                break
            with self.state_lock:
                current_time = self.current_time
                current_goal = dict(self.current_goal) if self.current_goal is not None else None
                current_pose = dict(self.current_pose) if self.current_pose is not None else None
                goal_latched = self._goal_reached

            timing = self.timing.step(current_time)
            timing_state = timing.state
            dt = timing.sim_dt
            sensor_age = timing.sensor_wall_age
            if np.isfinite(sensor_age):
                self.telemetry.sensor_age(sensor_age)

            if timing_state == TimingState.TIME_RESET:
                with self.state_lock:
                    self.last_velocity = np.zeros(3)
                    self._goal_hold_ticks = 0
                    self._prev_to_goal_vec = None
                if hasattr(self, "planner_cfg"):
                    self.path_planner.configure(self.planner_cfg)
                logger.info(
                    f"[{self.agent_id}] Timing state: TIME_RESET — "
                    f"sim_time={current_time!r}"
                )
            elif timing_state == TimingState.OUT_OF_ORDER:
                logger.warning(
                    f"[{self.agent_id}] Timing state: OUT_OF_ORDER — "
                    f"sim_time={current_time!r}; skipping control integration"
                )

            # Communication freshness is wall-clock based.  A paused simulator
            # may legitimately have zero simulation dt, but if the simulator is
            # advancing while sensor messages stop arriving, fail safe.
            if (
                timing_state != TimingState.PAUSED_ZERO_DT
                and not self.timing.sensor_is_fresh()
            ):
                safe_cmd = _zero_cmd
                self.pub_cmd_vel.put(json.dumps(safe_cmd))
                with self.state_lock:
                    self.last_velocity = np.zeros(3)

                self.step_count += 1
                if self.step_count % 100 == 0:
                    logger.warning(
                        f"[{self.agent_id}] Sensor stream stale "
                        f"({sensor_age*1000.0:.0f} ms); holding zero cmd"
                    )
                _sleep_to_next_tick()
                continue

            safe_cmd = None   # will be set below before every publish

            if current_goal is not None and current_pose is not None:

                curr_pos = np.array([
                    current_pose["x"],
                    current_pose["y"],
                    current_pose["z"]
                ])
                goal_pos = np.array([
                    current_goal["x"],
                    current_goal["y"],
                    current_goal["z"]
                ])
                dist_to_goal = np.linalg.norm(goal_pos - curr_pos)
                dist_xy = np.linalg.norm(goal_pos[:2] - curr_pos[:2])
                dist_z = abs(goal_pos[2] - curr_pos[2])
                current_speed = np.linalg.norm(self.last_velocity)
                max_decel = max(0.1, self.kinematics["max_acceleration"])
                stopping_distance = (current_speed * current_speed) / (2.0 * max_decel)

                # Detect crossing through the goal sphere between control ticks.
                to_goal_vec = goal_pos - curr_pos
                crossed_goal = False
                if self._prev_to_goal_vec is not None:
                    crossed_goal = (
                        np.dot(self._prev_to_goal_vec, to_goal_vec) < 0.0
                        and dist_to_goal <= max(self.GOAL_STOP_RADIUS * 2.0, self.GOAL_TOLERANCE * 2.0)
                    )
                self._prev_to_goal_vec = to_goal_vec.copy()

                # ── ARRIVAL LATCH ────────────────────────────────────────────
                # Latch on first arrival; stay latched until set_goal() is called.
                in_goal_region = (
                    (dist_to_goal <= self.GOAL_STOP_RADIUS) or
                    (dist_xy <= self.GOAL_TOLERANCE_XY and dist_z <= self.GOAL_TOLERANCE_Z) or
                    (dist_to_goal <= max(self.GOAL_TOLERANCE, stopping_distance + 0.2) and current_speed <= 0.8) or
                    crossed_goal
                )
                if in_goal_region:
                    self._goal_hold_ticks += 1
                else:
                    self._goal_hold_ticks = 0

                # Immediate hard latch once stop radius or crossing condition is met.
                if dist_to_goal <= self.GOAL_STOP_RADIUS or crossed_goal:
                    with self.state_lock:
                        self._goal_reached = True
                    self._goal_hold_ticks = self.GOAL_SETTLE_TICKS

                if self._goal_hold_ticks >= self.GOAL_SETTLE_TICKS:
                    with self.state_lock:
                        self._goal_reached = True

                if goal_latched:
                    self._goal_reached = True

                if self._goal_reached:
                    safe_cmd = _zero_cmd
                    self.pub_cmd_vel.put(json.dumps(safe_cmd))
                    with self.state_lock:
                        self.last_velocity = np.zeros(3)

                    self.step_count += 1
                    if self.step_count % 100 == 0:
                        logger.info(
                            f"[{self.agent_id}] Goal reached — holding position. "
                            f"dist={dist_to_goal:.2f}m"
                        )
                    _sleep_to_next_tick()
                    continue   # skip all APF work below

                # ── ACTIVE NAVIGATION ────────────────────────────────────────

                t_plan = time.perf_counter()
                cmd = self.path_planner.compute_velocity(
                    current_pose=current_pose,
                    goal_pose=current_goal,
                    voxel_map=self.voxel_map
                )
                self.telemetry.planner(time.perf_counter() - t_plan)

                # Hard halt on planner-level arrival detection.
                if self.path_planner.is_goal_reached():
                    with self.state_lock:
                        self._goal_reached = True
                        self._goal_hold_ticks = self.GOAL_SETTLE_TICKS
                    safe_cmd = _zero_cmd
                    self.pub_cmd_vel.put(json.dumps(safe_cmd))
                    with self.state_lock:
                        self.last_velocity = np.zeros(3)
                    _sleep_to_next_tick()
                    continue

                raw_v = np.array([
                    cmd["linear"].get("x", 0.0),
                    cmd["linear"].get("y", 0.0),
                    cmd["linear"].get("z", 0.0)
                ])

                # ── A. FLOOR SAFETY FIRST ────────────────────────────────────
                curr_z = current_pose["z"]
                min_z = self.kinematics["min_z"]
                max_z = self.kinematics["max_z"]

                # If below minimum, force upward
                if curr_z < min_z:
                    raw_v[2] = 1.0  # climb hard
                # If at minimum and trying to descend, block descent
                elif curr_z <= min_z + 0.5 and raw_v[2] < 0:
                    raw_v[2] = 0.0
                # If at max altitude, block ascent
                elif curr_z >= max_z and raw_v[2] > 0:
                    raw_v[2] = 0.0

                # ── B. Emergency damping near obstacles ────────────────────────
                nearby = self.voxel_map.get_nearby_obstacles(
                    curr_pos[0], curr_pos[1], curr_pos[2], radius=6.0
                )
                if nearby:
                    dists = []
                    for obs in nearby:
                        dist = np.linalg.norm(curr_pos - np.array([obs["x"], obs["y"], obs["z"]]))
                        if dist > 0.0:
                            dists.append(dist)
                    if dists:
                        min_obs_dist = min(dists)
                        if min_obs_dist < 1.2:
                            raw_v[:2] *= 0.65
                        elif min_obs_dist < 2.0:
                            raw_v[:2] *= 0.80
                        elif min_obs_dist < 3.0:
                            raw_v[:2] *= 0.90

                # ── B2. Keep out of other jobs' engagement zones ─────────────
                raw_v = self._keepout(raw_v, curr_pos, current_goal, current_time)

                # ── C. Service radius containment ────────────────────────────
                spawn_xy = np.array([self.spawn_pose["x"], self.spawn_pose["y"]])
                curr_xy  = np.array([current_pose["x"], current_pose["y"]])
                dist_2d  = np.linalg.norm(curr_xy - spawn_xy)
                if dist_2d >= self.kinematics["max_service_radius"]:
                    out_vec    = (curr_xy - spawn_xy) / max(dist_2d, 0.001)
                    v_xy       = np.array([raw_v[0], raw_v[1]])
                    v_outward  = np.dot(v_xy, out_vec)
                    if v_outward > 0:
                        v_xy  -= v_outward * out_vec
                        raw_v[0], raw_v[1] = v_xy[0], v_xy[1]

                # ── D. Goal-approach braking envelope ───────────────────────
                # Bound commanded speed so the drone can still stop inside the tolerance.
                brake_margin = max(0.0, dist_to_goal - self.GOAL_TOLERANCE)
                allowed_speed = np.sqrt(max(0.0, 2.0 * max_decel * brake_margin))
                raw_speed = np.linalg.norm(raw_v)
                if raw_speed > allowed_speed:
                    if allowed_speed <= 1e-6:
                        raw_v = np.zeros(3)
                    else:
                        raw_v = (raw_v / raw_speed) * allowed_speed

                # ── E. Acceleration clamping ─────────────────────────────────
                dv     = raw_v - self.last_velocity
                dv_mag = np.linalg.norm(dv)
                max_dv = self.kinematics["max_acceleration"] * dt
                if dv_mag > max_dv:
                    dv = (dv / dv_mag) * max_dv
                new_v = self.last_velocity + dv

                # ── F. Absolute velocity cap ─────────────────────────────────
                speed = np.linalg.norm(new_v)
                if speed > self.kinematics["max_velocity"]:
                    new_v = (new_v / speed) * self.kinematics["max_velocity"]

                # Final floor safety check
                if curr_z < min_z:
                    new_v[2] = max(new_v[2], 0.5)

                # ── G. Update momentum ───────────────────────────────────────
                with self.state_lock:
                    self.last_velocity = new_v

                safe_cmd = {
                    "linear":  {"x": float(new_v[0]), "y": float(new_v[1]), "z": float(new_v[2])},
                    "angular": {"x": 0.0, "y": 0.0, "z": 0.0}
                }
                self.pub_cmd_vel.put(json.dumps(safe_cmd))

            elif current_goal is None and current_pose is not None:
                # No tasking: hold station.  The simulator low-pass filters
                # commands, so zero must be sent continuously to stop a drift.
                self.pub_cmd_vel.put(json.dumps(_zero_cmd))
                with self.state_lock:
                    self.last_velocity = np.zeros(3)

            self.step_count += 1
            if self.step_count % 100 == 0 and current_goal is not None and safe_cmd is not None:
                logger.info(
                    f"[{self.agent_id}] CMD_VEL: {safe_cmd['linear']} | "
                    f"goal_reached={self._goal_reached} | "
                    f"sensor_age_ms={(sensor_age*1000.0):.0f} | "
                    f"visible_voxels={self.latest_voxel_summary['visible']} "
                    f"hits={self.latest_voxel_summary['hits']} "
                    f"nearest={self.latest_voxel_summary['nearest']}"
                )

            _sleep_to_next_tick()

    # ==========================================
    # PILLAR 3: THREAT HANDLER
    # ==========================================
    def _auction_now(self) -> float:
        """Sim-time when available (auction windows must track the simulator), else wall time."""
        with self.state_lock:
            t = self.current_time
        return float(t) if t is not None else simclock.now()

    def _link_up(self) -> bool:
        """Heard the ship or another drone recently."""
        return self._last_radio_rx is not None and simclock.now() - self._last_radio_rx <= self.LINK_TIMEOUT_S

    def _radio_heard(self):
        self._last_radio_rx = simclock.now()

    def _is_free(self) -> bool:
        """Available for tasking: alive, localized, not engaged and not awaiting a wave result."""
        with self.state_lock:
            has_pose = self.current_pose is not None
        return (
            has_pose
            and not self.destroyed
            and self.engaged_threat is None
            and self._pending_wave_id is None
        )

    @staticmethod
    def _parse(sample) -> dict:
        return json.loads(bytes(sample.payload).decode("utf-8"))

    def _on_threat_wave(self, sample):
        self._radio_heard()
        self.telemetry.rx("swarm/threats")
        try:
            msg = self._parse(sample)
            wave_id = str(msg["wave_id"])
            threats = [Threat.from_dict(t) for t in msg["threats"]]
        except Exception as e:
            logger.warning(f"[{self.agent_id}] Ignoring malformed threat order: {e}")
            return

        logger.info(
            f"[{self.agent_id}] Order {wave_id}: "
            + ", ".join(f"{t.threat_id}({t.type}, level {t.level}, need {t.required}, "
                        f"t_engage {t.t_engage:.1f})" for t in threats)
        )
        with self._auction_lock:
            if not self.auction.on_wave(wave_id, threats, self._auction_now()):
                return
            # A drone already tasked or waiting on another wave sits this one out,
            # otherwise it could be assigned to two threats.
            if not self._is_free():
                return
            costs = self._calculate_costs(threats)
            if not costs:
                return
            self._pending_wave_id = wave_id
            self.auction.on_bid(wave_id, self.agent_id, costs)
        self._propose_bid(wave_id, costs)

    def _calculate_costs(self, threats):
        """Bid our ETA (s) to each engagement point we can reach before its engagement time."""
        with self.state_lock:
            pose = dict(self.current_pose) if self.current_pose is not None else None
            now = self.current_time
        if pose is None or now is None:
            return {}
        v_max = float(self.kinematics["max_velocity"])
        a_max = float(self.kinematics["max_acceleration"])
        slack = max(ORDER_SLACK_S, self.auction.bid_window_s + 1.0)
        costs = {}
        for t in threats:
            d = float(np.linalg.norm([
                t.location["x"] - pose["x"], t.location["y"] - pose["y"], t.location["z"] - pose["z"],
            ]))
            e = eta(d, v_max, a_max) * ETA_MARGIN
            if now + slack + e <= t.t_engage:
                costs[t.threat_id] = round(e, 2)
        return costs

    # ==========================================
    # PILLAR 4: CONSENSUS MECHANISM
    # ==========================================
    def _propose_bid(self, wave_id, costs):
        self._radio_put(self.pub_bids, "swarm/bids", {"agent_id": self.agent_id, "wave_id": wave_id, "costs": costs})
        logger.info(f"[{self.agent_id}] Bid in {wave_id} (ETA s): "
                    + ", ".join(f"{k}={v:.1f}" for k, v in costs.items()))

    def _on_bid_received(self, sample):
        self._radio_heard()
        self.telemetry.rx("swarm/bids")
        try:
            bid = self._parse(sample)
            self._peer_heard(str(bid["agent_id"]))
            if bid["agent_id"] == self.agent_id:
                return  # own bid already recorded when placed
            with self._auction_lock:
                self.auction.on_bid(str(bid["wave_id"]), str(bid["agent_id"]), bid["costs"])
        except Exception as e:
            logger.warning(f"[{self.agent_id}] Ignoring malformed bid: {e}")

    def _service_auctions(self):
        """Close wave auctions whose window elapsed; called every control tick."""
        with self._auction_lock:
            if not self.auction.has_open_waves():
                return
            results = self.auction.close_due(self._auction_now())
            for r in results:
                if r.wave_id == self._pending_wave_id:
                    self._pending_wave_id = None
        for r in results:
            summary = ", ".join(f"{tid}->{ws}" for tid, ws in r.assignment.items())
            # Short fingerprint of what we concluded, reported in telemetry: the ship compares
            # fingerprints across drones to measure whether they agreed on the assignment.
            digest = hashlib.sha1(json.dumps(r.assignment, sort_keys=True).encode()).hexdigest()[:8]
            self._assign_hashes[r.wave_id] = digest
            while len(self._assign_hashes) > 10:
                self._assign_hashes.popitem(last=False)
            logger.info(f"[{self.agent_id}] Order {r.wave_id} assignment: {summary}")
            if r.my_threat is None or self.destroyed:
                continue
            t = r.my_threat
            if not self._link_up():
                # Computed from whatever bids reached us while cut off: likely a
                # divergent view, and no one would hear our award. Don't act on it.
                logger.warning(f"[{self.agent_id}] Radio down at auction close; not engaging {t.threat_id}")
                with self._auction_lock:
                    self.auction.release()
                continue
            # Tentative until the ship confirms: start flying now (no time to lose), using our own
            # view of the slots; the ship's ACK assigns the final slot.
            winners = sorted(r.assignment[t.threat_id])
            slot = winners.index(self.agent_id)
            loc = t.location
            with self.state_lock:
                now = self.current_time
            with self._eng_lock:
                self.engaged_threat = t
                self._set_engagement({
                    "threat": t, "wave_id": r.wave_id, "confirmed": False, "since": now, "seq": -1,
                    "point": (loc["x"], loc["y"], loc["z"]), "t_engage": t.t_engage,
                    "slot": slot, "n_slots": len(winners),
                })
                self._job_sub = self.radio.declare_subscriber(f"ship/jobs/{t.threat_id}", self._on_job)
            sp = self.engagement["slot_point"]
            logger.info(f"[{self.agent_id}] ENGAGING {t.threat_id} ({t.type}, level {t.level}), awaiting ACK: "
                        f"slot {slot + 1}/{len(winners)} at ({sp['x']:.1f}, {sp['y']:.1f}, {sp['z']:.1f}), "
                        f"ETA {r.my_cost:.1f}s, detonate at t={t.t_engage:.1f}")
            self._publish_award({
                "threat_id": t.threat_id, "agent_id": self.agent_id, "cost": r.my_cost, "wave_id": r.wave_id,
                "status": "engaged", "slot": slot, "t_engage": t.t_engage,
            })

    # ---- job confirmation and updates from the ship ----
    def _set_engagement(self, eng):
        """Install an engagement (a new dict) and fly to its slot if the slot moved."""
        sp = slot_point(eng["point"], eng["slot"], eng["n_slots"], self.SLOT_RADIUS)
        eng["slot_point"] = {"x": sp[0], "y": sp[1], "z": sp[2]}
        old = self.engagement
        self.engagement = eng
        if old is None or math.dist(sp, tuple(old["slot_point"][k] for k in "xyz")) >= self.RETARGET_MIN_M:
            self.set_goal(dict(eng["slot_point"]))

    def _apply_job(self, job: dict, confirm: bool) -> None:
        """Update our engagement from a job message (ACK or job topic).  Caller holds _eng_lock."""
        eng = self.engagement
        me = job.get("holders", {}).get(self.agent_id)
        new = dict(eng)
        p = job["point"]
        new.update(point=(p["x"], p["y"], p["z"]), t_engage=float(job["t_engage"]),
                   seq=int(job.get("seq", eng["seq"])), intruders=job.get("intruders", []))
        if me is not None:
            new.update(slot=int(me), n_slots=int(job.get("n_slots", eng["n_slots"])))
        if confirm and not eng["confirmed"]:
            new["confirmed"] = True
            logger.info(f"[{self.agent_id}] Job {eng['threat'].threat_id} CONFIRMED by ship: "
                        f"slot {new['slot'] + 1}/{new['n_slots']}, detonate at t={new['t_engage']:.1f}")
        elif abs(new["t_engage"] - eng["t_engage"]) > 0.05 or math.dist(new["point"], eng["point"]) > 0.05:
            logger.info(f"[{self.agent_id}] Job {eng['threat'].threat_id} updated: point moved "
                        f"{math.dist(new['point'], eng['point']):.1f} m, t_engage {new['t_engage']:.1f}")
        self._set_engagement(new)

    def _on_ack(self, sample):
        self._radio_heard()
        self.telemetry.rx("ship/ack")
        try:
            ack = self._parse(sample)
            threat_id, accepted = str(ack["threat_id"]), bool(ack["accepted"])
        except Exception as e:
            logger.warning(f"[{self.agent_id}] Ignoring malformed ACK: {e}")
            return
        with self._eng_lock:
            eng = self.engagement
            if eng is None or eng["threat"].threat_id != threat_id:
                if accepted and not self.destroyed:
                    # Confirmed for a job we already gave up (e.g. ACK timeout): tell the ship.
                    self._publish_award({"threat_id": threat_id, "agent_id": self.agent_id, "status": "withdrawn"})
                return
            if accepted:
                self._apply_job(ack, confirm=True)
            elif ack.get("wave_id") == eng["wave_id"] and not eng["confirmed"]:
                logger.warning(f"[{self.agent_id}] Job {threat_id} REJECTED by ship (better bids confirmed)")
                self._abandon(threat_id, "withdrawn")

    def _on_job(self, sample):
        self._radio_heard()
        self.telemetry.rx("ship/jobs")
        try:
            job = self._parse(sample)
            threat_id = str(job["threat_id"])
        except Exception as e:
            logger.warning(f"[{self.agent_id}] Ignoring malformed job update: {e}")
            return
        with self._eng_lock:
            eng = self.engagement
            if eng is None or eng["threat"].threat_id != threat_id or int(job.get("seq", 0)) < eng["seq"]:
                return
            if job.get("status") != "active":
                logger.info(f"[{self.agent_id}] Job {threat_id} closed by ship ({job.get('status')}); free again")
                self._abandon(threat_id, "released")
            elif self.agent_id in job.get("holders", {}):
                self._apply_job(job, confirm=True)
            elif eng["confirmed"]:
                logger.warning(f"[{self.agent_id}] Job {threat_id}: no longer a holder; released by ship")
                self._abandon(threat_id, "withdrawn")
            else:
                self._apply_job(job, confirm=False)   # not confirmed yet: still follow the target

    def _abandon(self, threat_id: str, status: str):
        """Leave a job and become free.  Caller holds _eng_lock."""
        with self._auction_lock:
            self.auction.release()
        self._disengage(threat_id, status)

    def _on_award_received(self, sample):
        self._radio_heard()
        self.telemetry.rx("swarm/awards")
        try:
            award = self._parse(sample)
            threat_id, agent_id = str(award["threat_id"]), str(award["agent_id"])
            status = award.get("status", "engaged")
            cost = float(award.get("cost") or 0.0)
        except Exception as e:
            logger.warning(f"[{self.agent_id}] Ignoring malformed award: {e}")
            return
        self._peer_heard(agent_id)
        if agent_id == self.agent_id:
            return
        with self._eng_lock:
            with self._auction_lock:
                if status == "engaged":
                    must_yield = self.auction.on_award(threat_id, agent_id, cost)
                else:
                    self.auction.on_withdraw(threat_id, agent_id)
                    must_yield = False
            eng = self.engagement
            # Once the ship has confirmed us, only the ship can release us (it hears every drone).
            if must_yield and eng is not None and eng["threat"].threat_id == threat_id and not eng["confirmed"]:
                logger.warning(f"[{self.agent_id}] Yielding {threat_id}: enough drones with better bids engaged")
                self._abandon(threat_id, "withdrawn")

    # ---- engagement zones and friendly fire ----
    def _on_zones(self, sample):
        self._radio_heard()
        self.telemetry.rx("ship/zones")
        try:
            zones = self._parse(sample)["zones"]
        except Exception as e:
            logger.warning(f"[{self.agent_id}] Ignoring malformed zones: {e}")
            return
        now = simclock.now()
        self.zones = {str(z["threat_id"]): (z, now) for z in zones}

    def _foreign_zones(self):
        """Live zones of jobs other than ours: (center, radius, t_engage, threat_id)."""
        now = simclock.now()
        eng = self.engagement
        mine = eng["threat"].threat_id if eng else None
        out = []
        for tid, (z, t) in list(self.zones.items()):
            if tid == mine or now - t > 3.0:       # ship publishes at 1 Hz; stale after 3 s
                continue
            p = z["point"]
            out.append(((p["x"], p["y"], p["z"]), float(z["radius"]), float(z["t_engage"]), tid))
        return out

    def _service_zones(self):
        """An idle drone inside another job's zone moves radially out of it and holds there."""
        if not self.KEEPOUT or self.destroyed or self.engagement is not None or self._pending_wave_id is not None:
            return
        with self.state_lock:
            pose = dict(self.current_pose) if self.current_pose is not None else None
            goal = self.current_goal
        if pose is None:
            return
        here = (pose["x"], pose["y"], pose["z"])
        for center, radius, _, tid in self._foreign_zones():
            if math.dist(here, center) >= radius:
                continue
            if goal is not None and math.dist((goal["x"], goal["y"], goal["z"]), center) >= radius:
                return                              # already evading to a point outside
            dx, dy = here[0] - center[0], here[1] - center[1]
            d = math.hypot(dx, dy)
            ux, uy = (dx / d, dy / d) if d > 1e-3 else (1.0, 0.0)
            r = radius + self.ZONE_MARGIN_M
            evade = {"x": center[0] + ux * r, "y": center[1] + uy * r, "z": here[2]}
            logger.info(f"[{self.agent_id}] Inside the engagement zone of {tid}; moving "
                        f"{r - d:.1f} m out to ({evade['x']:.1f}, {evade['y']:.1f})")
            self.set_goal(evade)
            return

    def _keepout(self, v, pos, goal, now):
        """Remove velocity into another job's zone in its last ZONE_HOT_S before detonation.

        A drone whose own goal lies inside that zone keeps going: its target comes first.
        """
        if now is None or not self.KEEPOUT:
            return v
        for center, radius, t_engage, _ in self._foreign_zones():
            if not (now >= t_engage - self.ZONE_HOT_S and now <= t_engage + 1.0):
                continue
            if goal is not None and math.dist((goal["x"], goal["y"], goal["z"]), center) < radius:
                continue
            out = np.asarray(pos, dtype=float) - np.asarray(center, dtype=float)
            d = float(np.linalg.norm(out))
            if d >= radius + 3.0 or d < 1e-6:
                continue
            n = out / d
            inward = -float(np.dot(v, n))
            if inward > 0:
                v = v + inward * n                  # slide along the boundary instead of entering
            if d < radius:
                v = v + n * min(2.0, radius - d)    # inside: push out
        return v

    def _on_blast(self, sample):
        """A detonation from another job within KILL_RADIUS destroys this drone (friendly fire)."""
        if self.destroyed:
            return
        try:
            b = self._parse(sample)
        except Exception:
            return
        if b.get("agent_id") == self.agent_id:
            return
        eng = self.engagement
        if eng is not None and eng["threat"].threat_id == b.get("threat_id"):
            return                                   # job-mates detonate together by design
        with self.state_lock:
            pose = dict(self.current_pose) if self.current_pose is not None else None
        if pose is None:
            return
        dist = math.dist((pose["x"], pose["y"], pose["z"]), (b["x"], b["y"], b["z"]))
        if dist > self.KILL_RADIUS:
            return
        threat_id = eng["threat"].threat_id if eng else None
        logger.warning(f"[{self.agent_id}] DESTROYED by friendly fire: {b['agent_id']} detonated on "
                       f"{b.get('threat_id')} {dist:.1f} m away" + (f"; job {threat_id} lost" if threat_id else ""))
        self.pub_damage.put(json.dumps({"agent_id": self.agent_id, "cause": "friendly_fire", "by": b["agent_id"],
                                        "by_threat": b.get("threat_id"), "job": threat_id,
                                        "distance": round(dist, 2), "sim_time": b.get("sim_time")}))
        self.pub_cmd_vel.put(json.dumps({"linear": {"x": 0.0, "y": 0.0, "z": 0.0},
                                         "angular": {"x": 0.0, "y": 0.0, "z": 0.0}}))
        pub_leave = self.onboard.declare_publisher("swarm/agents/despawn")
        pub_leave.put(json.dumps({"agent_id": self.agent_id, "reason": "friendly_fire"}))
        pub_leave.undeclare()
        with self._auction_lock:
            self.auction.release()
        with self._eng_lock:
            self.engaged_threat = None
            self.engagement = None
            self._drop_job_sub()
        self.destroyed = True
        self._send_heartbeat()
        threading.Timer(1.0, self.radio.close).start()

    def _drop_job_sub(self):
        if self._job_sub is not None:
            self._job_sub.undeclare()
            self._job_sub = None

    def _disengage(self, threat_id: str, status: str):
        with self._eng_lock:
            self.engaged_threat = None
            self.engagement = None
            self._drop_job_sub()
        self.set_goal(None)   # hold position
        self._publish_award({"threat_id": threat_id, "agent_id": self.agent_id, "status": status})

    def _publish_award(self, award: dict):
        """Send an award (or withdrawal) now and again on the next AWARD_RESENDS heartbeat ticks.

        A lost award makes the ship re-announce the threat and a second drone engage it, so
        awards are repeated; receivers treat repeats idempotently.  A newer status for the
        same threat replaces the pending copies, so a stale "engaged" never follows a withdrawal.
        """
        with self._award_lock:
            self._award_resends[award["threat_id"]] = [award, self.AWARD_RESENDS]
        self._radio_put(self.pub_awards, "swarm/awards", award)

    def _resend_awards(self):
        with self._award_lock:
            due = [entry[0] for entry in self._award_resends.values()]
            for tid in list(self._award_resends):
                self._award_resends[tid][1] -= 1
                if self._award_resends[tid][1] <= 0:
                    del self._award_resends[tid]
        for award in due:
            self._radio_put(self.pub_awards, "swarm/awards", award)

    def _check_engagement(self):
        """At the allocated time: detonate if on station, otherwise abort and hold.

        Runs on local state only, so an engaged drone completes its engagement
        even with the radio down.
        """
        eng = self.engagement
        if eng is None or self.destroyed:
            return
        with self.state_lock:
            now = self.current_time
            pose = dict(self.current_pose) if self.current_pose is not None else None
        if now is None or pose is None:
            return
        if not eng["confirmed"]:
            if eng["since"] is None or now - eng["since"] > self.ACK_TIMEOUT_S or now >= eng["t_engage"]:
                logger.warning(f"[{self.agent_id}] No ACK from the ship for {eng['threat'].threat_id} within "
                               f"{self.ACK_TIMEOUT_S:.0f}s; abandoning the job")
                with self._eng_lock:
                    if self.engagement is eng:
                        self._abandon(eng["threat"].threat_id, "withdrawn")
            return
        if now < eng["t_engage"]:
            return
        threat = eng["threat"]
        sp = eng["slot_point"]
        miss = float(np.linalg.norm([pose["x"] - sp["x"], pose["y"] - sp["y"], pose["z"] - sp["z"]]))
        if miss > self.DETONATE_RADIUS:
            logger.warning(f"[{self.agent_id}] MISSED {threat.threat_id}: {miss:.1f} m from slot at "
                           f"t_engage; aborting and holding position")
            with self._auction_lock:
                self.auction.release()
            self._disengage(threat.threat_id, "missed")
            return

        # Final check: non-job drones within CLEARANCE_M (positions from the ship's job topic).  The
        # target comes first: detonate anyway, at the risk of friendly fire.
        here = (pose["x"], pose["y"], pose["z"])
        intruders = [i["id"] for i in eng.get("intruders", [])
                     if math.dist(here, (i["pose"]["x"], i["pose"]["y"], i["pose"]["z"])) <= self.CLEARANCE_M]
        if intruders:
            logger.warning(f"[{self.agent_id}] Detonating on {threat.threat_id} with non-job drones within "
                           f"{self.CLEARANCE_M:.0f} m: {intruders} (risk of friendly fire)")
        # Physical event: goes on the onboard bus (the ship's radar observes it there).
        self.pub_detonation.put(json.dumps({
            "agent_id": self.agent_id, "threat_id": threat.threat_id, "sim_time": now,
            "x": pose["x"], "y": pose["y"], "z": pose["z"], "intruders": intruders,
        }))
        self.pub_cmd_vel.put(json.dumps({
            "linear": {"x": 0.0, "y": 0.0, "z": 0.0}, "angular": {"x": 0.0, "y": 0.0, "z": 0.0},
        }))
        pub_leave = self.onboard.declare_publisher("swarm/agents/despawn")
        pub_leave.put(json.dumps({"agent_id": self.agent_id, "reason": "detonated", "threat_id": threat.threat_id}))
        pub_leave.undeclare()
        with self._auction_lock:
            self.auction.release()
        with self._eng_lock:
            self.engaged_threat = None
            self.engagement = None
            self._drop_job_sub()
        self.destroyed = True
        self._send_heartbeat()   # final "expended" heartbeat
        # An expended drone has nothing left to say: close the radio (it cost ~1.6% of a core and
        # kept the drone in the radio mesh).  Delayed so the queued final heartbeat goes out first.
        threading.Timer(1.0, self.radio.close).start()
        logger.info(f"[{self.agent_id}] DETONATED on {threat.threat_id} ({threat.type}) at t={now:.2f}, "
                    f"{miss:.1f} m from slot; drone expended")

    # ==========================================
    # MEMBERSHIP: heartbeat, roster, link state
    # ==========================================
    def _state_name(self) -> str:
        if self.destroyed:
            return "expended"
        if self.engagement is not None:
            with self.state_lock:
                return "on_station" if self._goal_reached else "engaging"
        if self._pending_wave_id is not None:
            return "bidding"
        return "idle"

    def _needs_help(self) -> bool:
        """The ship does not seem to hear us: not in its recent rosters, or no roster for a while."""
        if self.destroyed:
            return False
        if self._roster_time is None:
            return simclock.now() - self._started > 2 * self.LINK_TIMEOUT_S
        return simclock.now() - self._roster_time > self.LINK_TIMEOUT_S or self._missing_from_roster >= 3

    def _send_heartbeat(self):
        with self.state_lock:
            pose = dict(self.current_pose) if self.current_pose is not None else None
            now = self.current_time
        eng = self.engagement
        hb = {
            "agent_id": self.agent_id,
            "sim_time": now,
            "state": self._state_name(),
            "link": self._link_up(),
            "pose": None if pose is None else {k: round(pose[k], 2) for k in ("x", "y", "z")},
            "threat_id": eng["threat"].threat_id if eng else None,
            "t_engage": eng["t_engage"] if eng else None,
            "confirmed": eng["confirmed"] if eng else None,
        }
        self._radio_put(self.pub_heartbeat, "swarm/heartbeat", hb)
        if self._needs_help():
            self._radio_put(self.pub_help, "swarm/heartbeat_help", hb)

    def _heartbeat_loop(self):
        period = 1.0 / self.HEARTBEAT_HZ
        ticks_per_telemetry = max(1, int(round(self.HEARTBEAT_HZ / self.TELEMETRY_HZ)))
        tick = 0
        while self.running and not self.destroyed:
            try:
                self._send_heartbeat()
                self._resend_awards()
                tick += 1
                if tick % ticks_per_telemetry == 0:
                    self._relay_unheard_peers()
                    snap = self.telemetry.snapshot()
                    snap.update({"agent_id": self.agent_id, "peers_heard": len(self._fresh_peers()),
                                 "assignments": dict(self._assign_hashes),
                                 "voxels": self.voxel_map.get_stats().get("total_voxels", 0)})
                    self._radio_put(self.pub_telemetry, "swarm/telemetry", snap)
                link = self._link_up()
                if link != self._link_was_up:
                    logger.warning(f"[{self.agent_id}] Radio link {'UP' if link else 'LOST'}"
                                   + ("" if link else (" - continuing engagement autonomously"
                                                      if self.engagement else " - holding position")))
                    self._link_was_up = link
            except Exception as e:
                logger.warning(f"[{self.agent_id}] heartbeat error: {e}")
            simclock.sleep(period)

    def _fresh_peers(self):
        now = simclock.now()
        return [p for p, t in self.peers.items() if now - t <= self.LINK_TIMEOUT_S]

    def _peer_heard(self, peer: str):
        if peer != self.agent_id:
            self.peers[peer] = simclock.now()

    def _on_peer_help(self, sample):
        peer = str(sample.key_expr).rsplit("/", 1)[-1]
        if peer == self.agent_id:
            return
        self.telemetry.rx("swarm/heartbeat_help")
        self._peer_heard(peer)
        self._peer_help[peer] = (bytes(sample.payload), simclock.now())
        self._radio_heard()

    def _relay_unheard_peers(self):
        """Forward heartbeats of peers that asked for help and are not in the roster (once per second).

        Zenoh peers do not relay for each other, so a drone whose link to the ship
        is lost while its links to other drones still work would drop out of the
        roster.  Such a drone publishes on swarm/heartbeat_help (_needs_help); only
        drones that are themselves in a fresh roster relay.
        """
        if self.destroyed or self._roster_time is None or simclock.now() - self._roster_time > self.LINK_TIMEOUT_S:
            return
        if self.agent_id not in self.roster:
            return
        now = simclock.now()
        for peer, (payload, t) in list(self._peer_help.items()):
            if now - t > self.LINK_TIMEOUT_S:
                del self._peer_help[peer]
            elif peer not in self.roster or peer in self.roster_relayed:
                self.pub_relay.put(f"swarm/heartbeat_relay/{peer}", payload)
                self.telemetry.tx("swarm/heartbeat_relay", len(payload))

    def _on_roster(self, sample):
        self._radio_heard()
        self.telemetry.rx("ship/roster")
        try:
            msg = self._parse(sample)
            self.roster = list(msg.get("members", []))
            self.roster_relayed = set(msg.get("relayed", []))
            self._roster_time = simclock.now()
        except Exception as e:
            logger.warning(f"[{self.agent_id}] Ignoring malformed roster: {e}")
            return
        if (self.agent_id in self.roster and self.agent_id not in self.roster_relayed) or self.destroyed:
            self._missing_from_roster = 0
            return
        # The ship does not hear us directly although we hear it.  Heartbeats keep going out, also on
        # heartbeat_help, and peers relay them (_relay_unheard_peers) - also while the roster lists us
        # as relayed, otherwise the relay would stop as soon as it worked.
        self._missing_from_roster += 1
        if self._missing_from_roster in (3, 30) or self._missing_from_roster % 300 == 0:
            how = "relayed by peers" if self.agent_id in self.roster else "not in the roster"
            logger.warning(f"[{self.agent_id}] Ship has not heard us directly for {self._missing_from_roster} "
                           f"rosters ({how}); still heartbeating")

    def shutdown(self):
        self.running = False
        if self.control_thread.is_alive():
            self.control_thread.join()
        if not self.destroyed:
            pub_leave = self.onboard.declare_publisher("swarm/agents/despawn")
            pub_leave.put(json.dumps({"agent_id": self.agent_id, "reason": "shutdown"}))
            pub_leave.undeclare()
        self.radio.close()
        self.onboard.close()
