# DeNDDron-Swarm
Codebase for DeNDDron Swarm

## Gazebo Simulation

To build and run:
```bash
docker compose build gazebo_simulator && docker compose up -d gazebo_simulator && docker compose restart agent_1 agent_2 agent_3
```
To view the Gazebo simulation GUI while the containers are running, execute:
```bash
docker exec -it gazebo_simulator gzclient
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
