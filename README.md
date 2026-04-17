# DeNDDron-Swarm
Codebase for DeNDDron Swarm

## Gazebo Simulation

To build and run:
```bash
bash scripts/run_swarm.sh 3
```

This starts the Gazebo simulator, the Zenoh router, and 3 agent containers. The agents read their spawn positions from `config/swarm_runtime.json` and appear in the Gazebo world as they join.

The launcher reuses existing images by default (no forced rebuild). To rebuild images when needed:
```bash
bash scripts/run_swarm.sh 3 --build
```

This launcher does three things in order:
1. Runs `scripts/generate_swarm_config.py` to create `config/swarm_runtime.json` (spawn map).
2. Writes `.swarm.env` (runtime docker env with `AGENT_COUNT`).
3. Starts Docker Compose with `--scale agent=<N>`.

Spawn positions are randomized on each run, bounded so agents are neither too near nor too far from the spawn center.
By default, they spawn in a ring around the ship at the origin.

You can change count by passing a different number:
```bash
bash scripts/run_swarm.sh 6
```

### Watch Agents Spawn in Gazebo

1. Allow the Docker container to open a GUI window on your host:
	```bash
	xhost +local:docker
	```

2. Start the swarm from the repo root:
	```bash
	bash scripts/run_swarm.sh 3
	```

3. If the Gazebo window does not appear automatically, open the client from another terminal:
	```bash
	docker exec -it gazebo_simulator gzclient
	```

4. Watch the drones appear in the world while the agents connect over Zenoh and initialize their voxel maps.

5. When you are done, close the simulator and clean up the containers:
	```bash
	docker compose down
	```

To customize random spawn limits:
```bash
python3 scripts/generate_swarm_config.py --agents 6 --x 0 --y 0 --z 20 --min-radius 30 --max-radius 45 --min-separation 8
docker compose --env-file .swarm.env up --scale agent=6
```

### Recent Updates
- **Agent Initialization**: Python agents now correctly spawn in the Gazebo 3D environment upon joining the Zenoh network.
- **Spawn Coordinates**: Agents are dynamically configured with a larger visual radius and explicitly spawn at clear locations outside the main ship obstacle to ensure high visibility.
- **Sensor Communications**: Set up the foundational bi-directional Zenoh topics mapping between the C++ Gazebo bridge and Python Agents (`drone/{agent_id}/sensors` and `cmd_vel`).

## Next Steps
- **Predetermined Paths**: Implement logic in the Python agent node to command agents along predetermined paths.
- **Velocity Commands**: Broadcast the path-following movement data from the agents to the Gazebo bridge as `cmd_vel` payload to actually move the drone models.
- **LiDAR Data Logging**: Start logging and actively recording the simulated 16-ray LiDAR sensor data being received by each agent from the simulated Gazebo environment.
- **Obstacle Detection**: Process the recorded LiDAR telemetry on the agent side to actively detect obstacles (like the ship or other drones) and build a spatial map.
