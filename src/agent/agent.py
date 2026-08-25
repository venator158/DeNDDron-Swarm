import zenoh
import json
import time
import threading
import numpy as np
import logging
import os
from voxel_map import VoxelMap
from path_planning import APFStrategy, ORCAStrategy
from timing import TimingManager, TimingState

logging.basicConfig(level=logging.INFO)
logger = logging.getLogger("DenddronAgent")

class DenddronAgent:
    def __init__(self, agent_id: str, router_locator: str = None):
        self.agent_id = agent_id
        
        # Zenoh Configuration
        conf = zenoh.Config()
        if router_locator:
            conf.insert_json5("connect/endpoints", f'["{router_locator}"]')
            
        logger.info(f"[{self.agent_id}] Connecting to Zenoh session...")
        self.session = zenoh.open(conf)
        
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
        self.sensor_wall_time = time.monotonic()
        self.latest_visible_obstacles = []
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

        self.sensor_wall_time = time.monotonic()
        self.sensor_sim_time = None
        self.max_control_dt = float(planner_cfg.get("max_control_dt", 0.2))
        self.sensor_timeout_s = float(planner_cfg.get("sensor_timeout_s", 0.5))
        self.control_rate_hz = 50.0
        self.timing = TimingManager(
            max_control_dt=self.max_control_dt,
            sensor_timeout_s=self.sensor_timeout_s,
            control_period_s=1.0 / self.control_rate_hz,
        )
        
        # --- Publishers ---
        self.pub_cmd_vel = self.session.declare_publisher(f"swarm/{self.agent_id}/cmd_vel")
        self.pub_bids    = self.session.declare_publisher("swarm/bids")

        # --- Subscribers ---
        self.sub_sensors = self.session.declare_subscriber(
            f"drone/{self.agent_id}/sensors",
            self._on_sensor_data
        )
        self.sub_jobs = self.session.declare_subscriber(
            "swarm/jobs",
            self._on_job_received
        )
        self.sub_bids = self.session.declare_subscriber(
            "swarm/bids",
            self._on_bid_received
        )

        # Start background threads
        self.control_thread = threading.Thread(target=self._reflex_control_loop)
        self.control_thread.start()
        
        self._announce_join()

    def _announce_join(self):
        """Tell the simulator to spawn this drone."""
        pub_join = self.session.declare_publisher("swarm/agents/join")
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
        try:
            payload = json.loads(bytes(sample.payload).decode('utf-8'))

            sim_time = payload.get("sim_time", None)
            pose = payload.get("pose", {})
            lidar_data = payload.get("lidar", [])

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
                if sim_time is not None:
                    self.current_time = float(sim_time)
                    self.sensor_sim_time = float(sim_time)
                else:
                    self.current_time = None
                    self.sensor_sim_time = None
                self.current_pose = current_pose
                self.sensor_wall_time = time.monotonic()

                if timing_state == TimingState.TIME_RESET:
                    self.last_velocity = np.zeros(3)
                    self._goal_hold_ticks = 0
                    self._prev_to_goal_vec = None

            self.voxel_map.cleanup_stale_data(max_age=0.5, current_time=self.current_time)
            self._process_lidar(lidar_data, current_pose)

        except Exception as e:
            logger.debug(f"[{self.agent_id}] Error processing sensor data: {e}")

    def _process_lidar(self, lidar_rays: list, pose: dict):
        if not lidar_rays:
            return

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

        self.latest_visible_obstacles = visible_obstacles
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
        rate_hz   = self.control_rate_hz
        sleep_time = 1.0 / rate_hz
        next_tick = time.monotonic()

        def _sleep_to_next_tick():
            nonlocal next_tick
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

        while self.running:
            with self.state_lock:
                current_time = self.current_time
                current_goal = dict(self.current_goal) if self.current_goal is not None else None
                current_pose = dict(self.current_pose) if self.current_pose is not None else None
                goal_latched = self._goal_reached

            timing = self.timing.step(current_time)
            timing_state = timing.state
            dt = timing.sim_dt
            sensor_age = timing.sensor_wall_age()

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

                cmd = self.path_planner.compute_velocity(
                    current_pose=current_pose,
                    goal_pose=current_goal,
                    voxel_map=self.voxel_map
                )

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
    # PILLAR 3: JOB HANDLER
    # ==========================================
    def _on_job_received(self, sample):
        job_data = json.loads(sample.payload.decode('utf-8'))
        job_id   = job_data.get("job_id")
        target_location = job_data.get("location")
        logger.info(f"[{self.agent_id}] Received new job {job_id} at {target_location}")
        cost = self._calculate_job_cost(target_location)
        self._propose_bid(job_id, cost)

    def _calculate_job_cost(self, location):
        return np.random.uniform(1.0, 100.0)

    # ==========================================
    # PILLAR 4: CONSENSUS MECHANISM
    # ==========================================
    def _propose_bid(self, job_id, cost):
        bid = {"agent_id": self.agent_id, "job_id": job_id, "cost": cost}
        self.pub_bids.put(json.dumps(bid))
        logger.info(f"[{self.agent_id}] Placed bid for job {job_id} with cost {cost:.2f}")

    def _on_bid_received(self, sample):
        bid_data = json.loads(sample.payload.decode('utf-8'))
        # If someone else has a lower cost for the same job, drop our bid.
        # If we win, call self.set_goal(job.location) — NOT self.current_goal directly.
        pass

    def shutdown(self):
        self.running = False
        if self.control_thread.is_alive():
            self.control_thread.join()
        
        pub_leave = self.session.declare_publisher("swarm/agents/despawn")
        pub_leave.put(json.dumps({"agent_id": self.agent_id, "reason": "shutdown"}))
        pub_leave.undeclare()
        self.session.close()