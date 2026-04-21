import zenoh
import json
import yaml
import time
import threading
import numpy as np
import logging
import os
from voxel_map import VoxelMap
from path_planning import APFStrategy

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
        self.voxel_map = VoxelMap()
        self.current_pose = None
        self.current_goal = None
        self.current_job = None
        self.step_count = 0
        self.sensor_frame_count = 0
        self.current_time = time.time()
        self.latest_visible_obstacles = []
        self.latest_voxel_summary = {
            "hits": 0,
            "placed": 0,
            "visible": 0,
            "nearest": []
        }

        # --- Goal State ---
        # Latched flag: once True, the drone stops forever until goal changes.
        # This prevents jitter from re-triggering APF after arrival.
        self._goal_reached = False
        self.GOAL_TOLERANCE = 1.5      # metres — declare arrived within this radius
        self.GOAL_STOP_RADIUS = 3.0    # metres — begin hard braking inside this radius
        self.GOAL_TOLERANCE_XY = 2.0
        self.GOAL_TOLERANCE_Z = 1.5
        self.GOAL_SETTLE_TICKS = 8
        self._goal_hold_ticks = 0

        # --- Dynamic Drone Kinematics ---
        self.last_velocity = np.zeros(3)
        self.spawn_pose = {"x": 0.0, "y": 0.0, "z": 0.0}
        self.kinematics = {
            "max_velocity": 2.0,
            "max_acceleration": 1.5,
            "max_z": 50.0,
            "min_z": 1.0,
            "max_service_radius": 100.0
        }

        # --- Job Simulation / Config Loading ---
        config_path = os.environ.get(
            "SWARM_RUNTIME_CONFIG",
            os.path.join(os.path.dirname(__file__), "..", "..", "config", "swarm_runtime.json")
        )
        if os.path.exists(config_path):
            with open(config_path, "r") as f:
                try:
                    cfg = json.load(f)
                    agents_cfg = cfg.get("agents", {})
                    my_cfg = agents_cfg.get(self.agent_id, {})
                    
                    if "spawn" in my_cfg:
                        self.spawn_pose = my_cfg["spawn"]

                    if "goal" in my_cfg:
                        self.current_goal = my_cfg["goal"]
                        logger.info(f"[{self.agent_id}] Assigned static goal from runtime config: {self.current_goal}")
                        
                    if "kinematics" in my_cfg:
                        self.kinematics.update(my_cfg["kinematics"])
                        logger.info(f"[{self.agent_id}] Loaded kinematic limits: {self.kinematics}")
                        
                except Exception as e:
                    logger.error(f"[{self.agent_id}] Failed to load static goal from config: {e}")

        # Initialize Reflex Strategy (using APF)
        self.path_planner = APFStrategy()
        planner_cfg = {
            "goal_tolerance": self.GOAL_TOLERANCE,
            "braking_radius": self.GOAL_STOP_RADIUS * 2.0,
            "min_z": self.kinematics["min_z"],
            "max_z": self.kinematics["max_z"],
        }
        if os.path.exists(config_path):
            try:
                with open(config_path, "r") as f:
                    cfg = json.load(f)
                    my_cfg = cfg.get("agents", {}).get(self.agent_id, {})
                    planner_cfg.update(my_cfg.get("path_planning", {}))
            except Exception:
                pass
        self.path_planner.configure(planner_cfg)
        
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
        self.current_goal = goal
        self._goal_reached = False
        self._goal_hold_ticks = 0
        self.last_velocity = np.zeros(3)   # reset momentum so it doesn't coast
        logger.info(f"[{self.agent_id}] New goal set: {goal}")

    # ==========================================
    # PILLAR 1: EYES (Obstacle Perception)
    # ==========================================
    def _on_sensor_data(self, sample):
        try:
            payload = json.loads(bytes(sample.payload).decode('utf-8'))

            self.current_time = payload.get("sim_time", time.time())
            pose = payload.get("pose", {})
            lidar_data = payload.get("lidar", [])

            self.current_pose = {
                "x": pose.get("x", 0.0),
                "y": pose.get("y", 0.0),
                "z": pose.get("z", 0.0),
                "yaw": pose.get("yaw", 0.0)
            }

            self.voxel_map.cleanup_stale_data(max_age=0.5, current_time=self.current_time)
            self._process_lidar(lidar_data, self.current_pose)

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

                self.voxel_map.raytrace(
                    agent_x, agent_y, agent_z,
                    ray_x, ray_y, ray_z,
                    current_time=self.current_time,
                    mark_endpoint_occupied=is_hit
                )

            except Exception as e:
                logger.debug(f"[{self.agent_id}] Error processing ray: {e}")

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

        Goal-reached logic:
          - Once the drone is within GOAL_TOLERANCE, _goal_reached is latched True.
          - While latched, we publish zero velocity every tick (keeps Gazebo's
            integrator from drifting) but skip all APF computation entirely.
          - The latch is only cleared by set_goal(), so new jobs can restart motion.
        """
        rate_hz   = 50.0
        sleep_time = 1.0 / rate_hz
        last_sim_time = self.current_time

        _zero_cmd = {
            "linear":  {"x": 0.0, "y": 0.0, "z": 0.0},
            "angular": {"x": 0.0, "y": 0.0, "z": 0.0}
        }

        while self.running:
            dt = self.current_time - last_sim_time
            if dt <= 0:
                dt = sleep_time
            last_sim_time = self.current_time

            safe_cmd = None   # will be set below before every publish

            if self.current_goal is not None and self.current_pose is not None:

                curr_pos = np.array([
                    self.current_pose["x"],
                    self.current_pose["y"],
                    self.current_pose["z"]
                ])
                goal_pos = np.array([
                    self.current_goal["x"],
                    self.current_goal["y"],
                    self.current_goal["z"]
                ])
                dist_to_goal = np.linalg.norm(goal_pos - curr_pos)
                dist_xy = np.linalg.norm(goal_pos[:2] - curr_pos[:2])
                dist_z = abs(goal_pos[2] - curr_pos[2])
                current_speed = np.linalg.norm(self.last_velocity)
                max_decel = max(0.1, self.kinematics["max_acceleration"])
                stopping_distance = (current_speed * current_speed) / (2.0 * max_decel)

                # ── ARRIVAL LATCH ────────────────────────────────────────────
                # Latch on first arrival; stay latched until set_goal() is called.
                in_goal_region = (
                    (dist_xy <= self.GOAL_TOLERANCE_XY and dist_z <= self.GOAL_TOLERANCE_Z) or
                    (dist_to_goal <= max(self.GOAL_TOLERANCE, stopping_distance + 0.2) and current_speed <= 0.8)
                )
                if in_goal_region:
                    self._goal_hold_ticks += 1
                else:
                    self._goal_hold_ticks = 0

                if self._goal_hold_ticks >= self.GOAL_SETTLE_TICKS:
                    self._goal_reached = True

                if self._goal_reached:
                    safe_cmd = _zero_cmd
                    self.pub_cmd_vel.put(json.dumps(safe_cmd))
                    self.last_velocity = np.zeros(3)

                    self.step_count += 1
                    if self.step_count % 100 == 0:
                        logger.info(
                            f"[{self.agent_id}] Goal reached — holding position. "
                            f"dist={dist_to_goal:.2f}m"
                        )
                    time.sleep(sleep_time)
                    continue   # skip all APF work below

                # ── ACTIVE NAVIGATION ────────────────────────────────────────

                cmd = self.path_planner.compute_velocity(
                    current_pose=self.current_pose,
                    goal_pose=self.current_goal,
                    voxel_map=self.voxel_map
                )

                raw_v = np.array([
                    cmd["linear"].get("x", 0.0),
                    cmd["linear"].get("y", 0.0),
                    cmd["linear"].get("z", 0.0)
                ])

                # ── A. FLOOR SAFETY FIRST ────────────────────────────────────
                curr_z = self.current_pose["z"]
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
                    dists = [
                        np.linalg.norm(curr_pos - np.array([o["x"], o["y"], o["z"]]))
                        for o in nearby
                        if np.linalg.norm(curr_pos - np.array([o["x"], o["y"], o["z"]])) > 1.2
                    ]
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
                curr_xy  = np.array([self.current_pose["x"], self.current_pose["y"]])
                dist_2d  = np.linalg.norm(curr_xy - spawn_xy)
                if dist_2d >= self.kinematics["max_service_radius"]:
                    out_vec    = (curr_xy - spawn_xy) / max(dist_2d, 0.001)
                    v_xy       = np.array([raw_v[0], raw_v[1]])
                    v_outward  = np.dot(v_xy, out_vec)
                    if v_outward > 0:
                        v_xy  -= v_outward * out_vec
                        raw_v[0], raw_v[1] = v_xy[0], v_xy[1]

                # ── D. Acceleration clamping ─────────────────────────────────
                dv     = raw_v - self.last_velocity
                dv_mag = np.linalg.norm(dv)
                max_dv = self.kinematics["max_acceleration"] * dt
                if dv_mag > max_dv:
                    dv = (dv / dv_mag) * max_dv
                new_v = self.last_velocity + dv

                # ── E. Absolute velocity cap ─────────────────────────────────
                speed = np.linalg.norm(new_v)
                if speed > self.kinematics["max_velocity"]:
                    new_v = (new_v / speed) * self.kinematics["max_velocity"]

                # Final floor safety check
                if curr_z < min_z:
                    new_v[2] = max(new_v[2], 0.5)

                # ── F. Update momentum ───────────────────────────────────────
                self.last_velocity = new_v

                safe_cmd = {
                    "linear":  {"x": float(new_v[0]), "y": float(new_v[1]), "z": float(new_v[2])},
                    "angular": {"x": 0.0, "y": 0.0, "z": 0.0}
                }
                self.pub_cmd_vel.put(json.dumps(safe_cmd))

            self.step_count += 1
            if self.step_count % 100 == 0 and self.current_goal is not None and safe_cmd is not None:
                logger.info(
                    f"[{self.agent_id}] CMD_VEL: {safe_cmd['linear']} | "
                    f"goal_reached={self._goal_reached} | "
                    f"visible_voxels={self.latest_voxel_summary['visible']} "
                    f"hits={self.latest_voxel_summary['hits']} "
                    f"nearest={self.latest_voxel_summary['nearest']}"
                )

            time.sleep(sleep_time)

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