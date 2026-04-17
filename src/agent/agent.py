import zenoh
import json
import time
import threading
import numpy as np
import logging
from voxel_map import VoxelMap

logging.basicConfig(level=logging.INFO)
logger = logging.getLogger("DenddronAgent")

class DenddronAgent:
    def __init__(self, agent_id: str, router_locator: str = None):
        self.agent_id = agent_id
        
        # Zenoh Configuration
        conf = zenoh.Config()
        if router_locator:
            # E.g., "tcp/zenoh_router:7447"
            conf.insert_json5("connect/endpoints", f'["{router_locator}"]')
            
        logger.info(f"[{self.agent_id}] Connecting to Zenoh session...")
        self.session = zenoh.open(conf)
        
        # --- Internal State ---
        self.running = True
        self.voxel_map = VoxelMap()  # Probabilistic occupancy grid
        self.current_pose = None  # Current agent position and orientation
        self.current_goal = None
        self.current_job = None
        self.step_count = 0  # For periodic exports
        
        # --- PILLAR 2: Reflexes (Publishers) ---
        self.pub_cmd_vel = self.session.declare_publisher(f"drone/{self.agent_id}/cmd_vel")
        
        # --- PILLAR 4: Consensus (Publishers) ---
        self.pub_bids = self.session.declare_publisher("swarm/bids")

        # --- PILLAR 1: Eyes (Subscribers) ---
        self.sub_sensors = self.session.declare_subscriber(
            f"drone/{self.agent_id}/sensors", 
            self._on_sensor_data
        )

        # --- PILLAR 3: Job Handler (Subscribers) ---
        self.sub_jobs = self.session.declare_subscriber(
            "swarm/jobs", 
            self._on_job_received
        )
        
        # --- PILLAR 4: Consensus (Subscribers) ---
        self.sub_bids = self.session.declare_subscriber(
            "swarm/bids", 
            self._on_bid_received
        )

        # Start background threads
        self.control_thread = threading.Thread(target=self._reflex_control_loop)
        self.control_thread.start()
        
        # Announce Join
        self._announce_join()

    def _announce_join(self):
        """Tell the simulator to spawn this drone."""
        pub_join = self.session.declare_publisher("swarm/agents/join")
        payload = {"agent_id": self.agent_id, "type": "quadrotor"}
        pub_join.put(json.dumps(payload))
        logger.info(f"[{self.agent_id}] Joined the swarm.")
        pub_join.undeclare()

    # ==========================================
    # PILLAR 1: EYES (Obstacle Perception)
    # ==========================================
    def _on_sensor_data(self, sample):
        """
        Receives sensor data (pose + LiDAR) from Gazebo.
        Processes LiDAR rays and updates the voxel map for collision avoidance.
        """
        try:
            payload = json.loads(bytes(sample.payload).decode('utf-8'))

            # Extract pose and LiDAR data
            pose = payload.get("pose", {})
            lidar_data = payload.get("lidar", [])

            # Store current pose for transformation
            self.current_pose = {
                "x": pose.get("x", 0.0),
                "y": pose.get("y", 0.0),
                "z": pose.get("z", 0.0),
                "yaw": pose.get("yaw", 0.0)
            }

            # Process LiDAR rays
            self._process_lidar(lidar_data, self.current_pose)

        except Exception as e:
            logger.debug(f"[{self.agent_id}] Error processing sensor data: {e}")

    def _process_lidar(self, lidar_rays: list, pose: dict):
        """
        Process LiDAR rays: transform to world frame and update voxel map.

        Args:
            lidar_rays: List of ray measurements with (angle, distance, intensity, ray_id)
            pose: Agent pose (x, y, z, yaw)
        """
        if not lidar_rays:
            return

        agent_x = pose.get("x", 0.0)
        agent_y = pose.get("y", 0.0)
        agent_z = pose.get("z", 0.0)
        yaw = pose.get("yaw", 0.0)

        for ray in lidar_rays:
            try:
                angle = ray.get("angle", 0.0)
                distance = ray.get("distance", 0.0)

                # Ignore invalid distances
                if distance <= 0 or distance > 100:
                    continue

                # Transform ray from drone frame to world frame
                # Ray in drone frame: horizontal angle = angle, vertical angle = 0 (horizontal plane)
                # This simulates a horizontal 2D LiDAR

                # World coordinates of ray endpoint
                world_angle = yaw + angle  # Robot yaw + ray angle
                ray_x = agent_x + distance * np.cos(world_angle)
                ray_y = agent_y + distance * np.sin(world_angle)
                ray_z = agent_z  # Assume horizontal LiDAR at drone center height

                # Raytrace: mark free space along ray, occupied at endpoint
                self.voxel_map.raytrace(agent_x, agent_y, agent_z,
                                       ray_x, ray_y, ray_z)

            except Exception as e:
                logger.debug(f"[{self.agent_id}] Error processing ray: {e}")

        # Log voxel map stats periodically (light logging)
        logger.debug(f"[{self.agent_id}] Voxel map: {self.voxel_map.get_stats()}")

    def _export_voxel_map(self):
        """Export voxel map to JSON file for visualization."""
        try:
            import os
            snapshot = {
                "timestamp": time.time(),
                "agent_id": self.agent_id,
                "voxel_map": self.voxel_map.export_to_dict(),
                "stats": self.voxel_map.get_stats()
            }

            output_path = f"/tmp/voxel_agent_{self.agent_id}.json"
            with open(output_path, 'w') as f:
                json.dump(snapshot, f)

            logger.debug(f"[{self.agent_id}] Exported voxel map to {output_path}")
        except Exception as e:
            logger.debug(f"[{self.agent_id}] Failed to export voxel map: {e}")

    # ==========================================
    # PILLAR 2: REFLEXES (APF & Navigation)
    # ==========================================
    def _reflex_control_loop(self):
        """
        Runs at a constant frequency (e.g., 50Hz).
        Calculates Artificial Potential Fields (APF) based on `self.voxel_map`
        and `self.current_goal`, then outputs to cmd_vel.
        """
        rate_hz = 50.0
        sleep_time = 1.0 / rate_hz

        while self.running:
            if self.current_goal is not None:
                # 1. Calculate attractive force towards goal
                # 2. Calculate repulsive force from self.voxel_map (Eyes)
                # 3. Sum forces to get velocity vector

                # Placeholder logic:
                cmd = {
                    "linear": {"x": 0.0, "y": 0.0, "z": 0.0},
                    "angular": {"x": 0.0, "y": 0.0, "z": 0.0}
                }

                self.pub_cmd_vel.put(json.dumps(cmd))

            # Export voxel map every 5 seconds (250 steps at 50Hz)
            self.step_count += 1
            if self.step_count % 250 == 0:
                self._export_voxel_map()

            time.sleep(sleep_time)

    # ==========================================
    # PILLAR 3: JOB HANDLER
    # ==========================================
    def _on_job_received(self, sample):
        """
        Listens to the job topic and decides whether to take the job or not.
        """
        job_data = json.loads(sample.payload.decode('utf-8'))
        job_id = job_data.get("job_id")
        target_location = job_data.get("location")
        
        logger.info(f"[{self.agent_id}] Received new job {job_id} at {target_location}")
        
        # Calculate cost/distance to job
        cost = self._calculate_job_cost(target_location)
        
        # Trigger Pillar 4 (Consensus)
        self._propose_bid(job_id, cost)

    def _calculate_job_cost(self, location):
        """Placeholder for cost calculation (e.g., distance to target)"""
        return np.random.uniform(1.0, 100.0)

    # ==========================================
    # PILLAR 4: CONSENSUS MECHANISM
    # ==========================================
    def _propose_bid(self, job_id, cost):
        """Broadcasts willingness to take a job."""
        bid = {
            "agent_id": self.agent_id,
            "job_id": job_id,
            "cost": cost
        }
        self.pub_bids.put(json.dumps(bid))
        logger.info(f"[{self.agent_id}] Placed bid for job {job_id} with cost {cost:.2f}")

    def _on_bid_received(self, sample):
        """
        Evaluates bids from other agents against own bids.
        Comes to a conclusion about job assignments in the swarm.
        """
        bid_data = json.loads(sample.payload.decode('utf-8'))
        # If someone else has a lower cost for the same job, drop our bid.
        # If we win, self.current_goal = job.location
        pass

    def shutdown(self):
        self.running = False
        if self.control_thread.is_alive():
            self.control_thread.join()
        
        # Publish despawn
        pub_leave = self.session.declare_publisher("swarm/agents/despawn")
        pub_leave.put(json.dumps({"agent_id": self.agent_id, "reason": "shutdown"}))
        pub_leave.undeclare()
        
        self.session.close()
