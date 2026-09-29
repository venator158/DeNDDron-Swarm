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

For Metrics, Open another terminal and run the following command
```bash
docker compose logs -f metrics_node
```

## Threat Engagement (Naval Defence Scenario)

Drones hold station around the ship until a threat is dispatched. Each threat has a **level**, which is both its priority and the number of drones it needs (default: `uav` = 1, `missile` = 2, `cruise_missile` = 3). Drones are expendable: a drone that reaches its threat intercepts it and is despawned for good.

```bash
# 8 drones, 5 waves of up to 4 threats, one wave every 20 s
bash scripts/run_swarm.sh 8 --threats 5 --threats-per-wave 4 --threat-interval 20
docker compose logs -f threat_dispatcher
```

Override the mix with `THREAT_MIX="type:level:weight,..."`, e.g. `THREAT_MIX="uav:1:0.3,missile:2:0.3,cruise_missile:3:0.4"`.

- **Dispatcher** (`src/agent/threat_dispatcher.py`): admits threats in priority order only while the sum of their levels fits the free drones, so drones committed never exceed drones alive. Threats that don't fit are reported as unengaged. Under-assigned threats are re-announced.
- **Allocation** (`src/agent/auction.py`): fully decentralized. Each free drone bids its distance to every threat in a wave; after a short window every drone computes the same priority-greedy assignment (highest level first, each threat takes its `level` cheapest drones, ties broken by drone ID). If views diverge and a threat gets too many drones, the worse bidders yield.
- **Topics**: `swarm/threats` (waves), `swarm/bids`, `swarm/awards` (engaged / withdrawn), `swarm/intercepts`, `swarm/agents/despawn`.

### Recent Updates
- **Agent Initialization**: Python agents now correctly spawn in the Gazebo 3D environment upon joining the Zenoh network.
- **Spawn Coordinates**: Agents are dynamically configured with a larger visual radius and explicitly spawn at clear locations outside the main ship obstacle to ensure high visibility.
- **Sensor Communications**: Set up the foundational bi-directional Zenoh topics mapping between the C++ Gazebo bridge and Python Agents (`drone/{agent_id}/sensors` and `cmd_vel`).

## Next Steps
- **Predetermined Paths**: Implement logic in the Python agent node to command agents along predetermined paths.
- **Velocity Commands**: Broadcast the path-following movement data from the agents to the Gazebo bridge as `cmd_vel` payload to actually move the drone models.
- **LiDAR Data Logging**: Start logging and actively recording the simulated 16-ray LiDAR sensor data being received by each agent from the simulated Gazebo environment.
- **Obstacle Detection**: Process the recorded LiDAR telemetry on the agent side to actively detect obstacles (like the ship or other drones) and build a spatial map.
