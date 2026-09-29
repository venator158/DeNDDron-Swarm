# DeNDDron Swarm

**De**centralized **N**aval **D**efence **Dron**e swarm. Drones hold station around a ship and, when threats appear, decide among themselves which drones intercept which threats. There is no central controller. Drones are expendable: each one is destroyed when it intercepts a threat.

The simulation runs in Gazebo. Every drone runs as its own Python agent in its own container, and all components talk over Zenoh.

This README is the project's only documentation. Keep it up to date when behaviour changes.

## Contents
- [Quick start](#quick-start)
- [Threat engagement](#threat-engagement)
- [Architecture](#architecture)
- [Agent internals](#agent-internals)
- [Configuration](#configuration)
- [Testing](#testing)
- [Repository layout](#repository-layout)
- [Known limitations](#known-limitations)

## Quick start

Requirements: Docker with Compose v2, and an X11 display if you want the Gazebo window.

```bash
# 3 drones flying to static goals (basic navigation check)
bash scripts/run_swarm.sh 3

# Naval defence scenario: 8 drones, 5 waves of up to 4 threats, one wave every 20 s
bash scripts/run_swarm.sh 8 --threats 5 --threats-per-wave 4 --threat-interval 20
```

The first run builds the images, which takes several minutes. Later runs reuse them. Add `--build` after changing code.

| Option | Default | Meaning |
|---|---|---|
| `N` (first argument) | 3 | number of drones |
| `--algorithm orca\|apf` | `orca` | path planner |
| `--seed S` | 42 | seed for spawn layout and threat generation |
| `--threats WAVES` | 0 (off) | enable the threat dispatcher with this many waves |
| `--threats-per-wave K` | 4 | at most K threats per wave (the actual number is random, from 1 to K) |
| `--threat-interval S` | 15 | wall-clock seconds between waves |
| `--build` | off | rebuild the Docker images |
| `THREAT_MIX` env | `uav:1:0.5,missile:2:0.35,cruise_missile:3:0.15` | threat types as `type:level:weight` |

The launcher does four things:
1. It generates `config/swarm_runtime.json`, which holds spawn points and planner/kinematics defaults.
2. It writes `.swarm.env`.
3. It resets the drone-ID registry.
4. It runs `docker compose up --scale agent=N`.

In threat mode, drones get no static goals.

### Watching the simulation

```bash
xhost +local:docker                            # let containers open windows on your display
bash scripts/run_swarm.sh 8 --threats 5        # in one terminal
docker exec -it gazebo_simulator gzclient      # in another: opens the Gazebo window
```

In the window, threats show up as glowing spheres when they are dispatched: yellow for a UAV, orange for a missile, red for a cruise missile, larger for higher levels. Each sphere disappears once the threat is neutralized. Drones vanish when they are expended. Unengaged threats are never dispatched, so they are not drawn. Set `THREAT_INITIAL_DELAY_S` (default 25) to get more time to open the window before the first wave.

To follow individual services:

```bash
docker compose logs -f threat_dispatcher       # waves, admissions, intercepts, final summary
docker compose logs -f metrics_node            # live dashboard: positions, speed, distance, collisions
docker compose logs -f agent                   # all drones
docker compose down                            # stop everything
```

## Threat engagement

### Threat levels
Every threat has a **level**. The level is both its priority and the number of drones needed to intercept it.

| Type | Level (drones needed) |
|---|---|
| `uav` | 1 |
| `missile` | 2 |
| `cruise_missile` | 3 |

### Rules
1. **Never commit more drones than exist.** The dispatcher only engages threats while the sum of their levels fits within the free drones:

   `committed ≤ alive`

   Here, *committed* means drones already engaged plus drones still owed to open threats, and *alive* counts drones that have not been expended. Threats are considered highest level first. A threat that doesn't fit is reported as **unengaged** and never dispatched.
2. **All or nothing.** The swarm itself also refuses a threat it cannot fully cover. If fewer free drones bid than the threat needs, no drone engages it, and those drones stay free for lower-priority threats.
3. **Expendable drones.** A drone that reaches its threat intercepts it and is despawned permanently. The simulator removes its model and never respawns it.

### Allocation protocol
The protocol is decentralized. It lives in `src/agent/auction.py`.

1. The dispatcher publishes a **wave** of threats on `swarm/threats`.
2. Each free drone publishes one bid on `swarm/bids`. The bid lists its cost, the straight-line distance, for every threat in the wave.
3. After a short bid window (`auction.bid_window_s`, default 1 s of sim time), every drone independently runs the same deterministic assignment:
   - threats are taken highest level first, with ties broken by threat ID;
   - each threat takes its `level` cheapest unassigned drones, with ties broken by drone ID, or no drones at all if it cannot be fully covered;
   - each drone is assigned to at most one threat.

   Drones that received the same bids therefore compute the same answer without talking to each other.
4. Each assigned drone publishes an award (`status: engaged`) on `swarm/awards` and flies to the threat.
5. **Conflict repair.** Drones may see different bids, for example after a lost message. If a threat ends up with more drones than its level, a drone backs off (`status: withdrawn`) once it sees `level` drones with better bids.
6. **Re-announcement.** If a threat is left short of drones, the dispatcher re-announces it for just the missing number. After `--max-announces` attempts it abandons the threat.
7. **Interception.** On arrival, the drone publishes to `swarm/intercepts` and `swarm/agents/despawn`, then shuts down its control loop. The process stays up so the container does not restart and respawn it.

A drone that is already engaged, or that is waiting for another wave's result, does not bid.

### Dispatcher summary
At the end of a run the dispatcher prints a summary like this:

```
SUMMARY (all waves resolved): 5 waves, 11 threats generated
  engaged=5 neutralized=5 abandoned=0 unengaged(no budget)=6
  drones expended=8 alive at end=0 peak committed=3
  invariant (committed <= alive) violations=0, threats with more drones than level=0
```

The dispatcher exits with code 0 only if there were no invariant violations, no over-assigned threats and no abandoned threats.

## Architecture

```
                         Zenoh router (tcp/udp 7447)
   ┌────────────────────────────┼─────────────────────────────────┐
   │                            │                                 │
Gazebo simulator (C++)     Agent × N (Python)             Threat dispatcher (Python, optional)
 - integrates drone motion  - voxel map from lidar         - generates waves, enforces the budget
 - 32-ray planar lidar      - APF / ORCA planner           - tracks liveness, awards, intercepts
 - spawns / despawns models - 50 Hz control loop          Metrics node (Python)
                            - bids, engages, intercepts    - distance, speed, collisions
```

The simulator (`sim/GazeboSimulator.cpp`) runs next to `gzserver` (world: `sim/ocean.world`). It does its own kinematic integration from the velocity commands, and uses Gazebo for visualisation. The ship is modelled as a cylinder at the origin: radius 16 m for the lidar, and a 17.5 m keep-out in the physics.

### Zenoh topics

| Topic | Publisher → Subscriber | Payload |
|---|---|---|
| `swarm/agents/join` | agent → simulator, metrics | `{agent_id, type}` — spawn this drone |
| `swarm/agents/despawn` | agent → simulator, metrics | `{agent_id, reason, threat_id?}` — remove this drone |
| `drone/{id}/sensors` | simulator → agent, metrics, dispatcher | `{sim_time, pose{x,y,z,yaw,vx,vy,vz}, lidar[{angle,distance,intensity}]}` at 50 Hz |
| `swarm/{id}/cmd_vel` | agent → simulator | `{linear{x,y,z}, angular{x,y,z}}` |
| `swarm/threats` | dispatcher → agents | `{wave_id, threats[{threat_id, type, level, required, location}]}` |
| `swarm/bids` | agent → agents | `{agent_id, wave_id, costs{threat_id: cost}}` |
| `swarm/awards` | agent → agents, dispatcher | `{threat_id, agent_id, cost, status: engaged\|withdrawn}` |
| `swarm/intercepts` | agent → dispatcher | `{threat_id, agent_id, sim_time}` |
| `swarm/metrics/summary` | metrics → anyone | swarm summary every 5 s |

### Drone IDs
Agent replicas are identical containers. On startup, each claims the lowest free `drone_N` from `config/swarm_runtime.json`. The claim is recorded in `config/agent_registry.json` under a file lock, keyed by container hostname.

## Agent internals

Each agent (`src/agent/agent.py`) has four parts.

1. **Eyes: `voxel_map.py`.**
   - A sparse 3D occupancy grid (0.5 m voxels) in world coordinates.
   - Each lidar sweep is ray-traced in one batch: cells along each ray are marked free, and the endpoint is marked occupied if the ray hit something.
   - Voxels expire after 0.5 s, so moving drones don't leave trails.
   - Rays longer than 50 m, or with non-positive distance, are ignored.
2. **Reflexes: `path_planning.py` and the 50 Hz control loop.**
   - **APF:** attraction to the goal plus exponentially decaying repulsion from nearby voxels and the ship. The repulsion is soft-saturated, and there is stuck detection with growing attraction.
   - **ORCA:** one velocity half-plane per nearby voxel, plus one for the ship, solved by iterative projection. It works in the XY plane with a vertical filter; altitude is handled separately.
   - After planning, the loop applies:
     - a floor/ceiling guard,
     - damping near obstacles,
     - service-radius containment,
     - a braking envelope near the goal,
     - acceleration and speed limits.

     Arrival latches after `goal_control.settle_ticks`. An untasked drone keeps sending zero velocity so that it holds position.
3. **Timing: `timing.py`.**
   - Simulation time drives the physics integration. Wall-clock time drives sensor freshness.
   - It detects these states: first frame, normal, paused, out of order, time reset, large dt and missing time.
   - If sensor data is stale, the drone commands zero velocity.
4. **Threat handling and consensus: `auction.py` and `threats.py`.** See [Allocation protocol](#allocation-protocol).

The **metrics node** (`src/metrics/main.py`):
- tracks each drone's distance, speed, time alive and proximity collisions (under 2.5 m, checked with a spatial hash);
- prints a dashboard;
- publishes `swarm/metrics/summary`;
- saves `/state/metrics_log.json`.

## Configuration

`config/swarm_runtime.json` is generated on every launch and is not tracked in git. To change the defaults, edit `GLOBAL_DEFAULTS` in `scripts/generate_swarm_config.py`. Per-drone overrides can go under `agents.<id>` in the runtime file.

| Section | Keys |
|---|---|
| `defaults.path_planning` | `algorithm`, gains/radii for APF, `step_size`, `goal_tolerance`, `braking_radius`, `ship_keepout_radius`, `velocity_smoothing`; optional `time_horizon_obst`, `agent_radius`, `max_control_dt`, `sensor_timeout_s` |
| `defaults.kinematics` | `max_velocity` (4 m/s), `max_acceleration` (1 m/s²), `min_z`, `max_z`, `max_service_radius` |
| `defaults.goal_control` | `tolerance`, `stop_radius`, `tolerance_xy`, `tolerance_z`, `settle_ticks` |
| `defaults.auction` (optional) | `bid_window_s` (1.0) |
| `agents.drone_N` | `spawn{x,y,z}`, optional `goal{x,y,z}` and per-drone overrides |

To generate a layout by hand:

```bash
python3 scripts/generate_swarm_config.py --agents 6 --min-radius 30 --max-radius 45 --min-separation 8 --seed 1 [--no-goals]
```

## Testing

```bash
python3 -m pytest tests -q                 # unit tests, ~2 s
python3 tests/run_all_validations.py       # validation suite, ~75 s; writes pre_consensus_validation_report.md (git-ignored)
```

| File | Covers |
|---|---|
| `test_auction.py` | deterministic assignment, priority order, all-or-nothing, tie-breaks, early/late bids, conflict yield |
| `test_threats.py` | budget admission (`sum(levels) ≤ free`), priority order, threat serialization, mix parsing |
| `test_apf.py`, `test_orca_vertical_filter.py` | planner force bounds, ORCA vertical envelope |
| `test_voxel_map_batch.py`, `benchmark_voxelmap.py` | batched ray-trace correctness and throughput |
| `test_timing_manager.py`, `test_time_handling.py` | timing states |
| `test_metrics_spatial_hash.py` | spatial-hash collision check matches brute force |
| `validate_*.py` | planner scenarios, timing, metrics overhead, worker-thread lifecycle |

The Gazebo end-to-end check is running the swarm itself: the dispatcher's exit code and summary report the result.

## Repository layout

```
config/                 generated runtime files (swarm_runtime.json, agent_registry.json) — not tracked
docker/                 base (zenoh-c/cpp), gazebo, agent, metrics images
scripts/run_swarm.sh    launcher
scripts/generate_swarm_config.py
sim/                    GazeboSimulator (C++ bridge + kinematics), simulator_main.cpp, ocean.world
src/agent/              agent, planners, voxel map, timing, auction, threats, threat_dispatcher
src/metrics/main.py     metrics node
tests/                  unit tests and validation scripts
```

## Known limitations
- **Threats don't move.** They are fixed points; there is no intercept geometry and no time-to-impact.
- **Assignments are greedy.** They are made in priority order, not globally optimal. Overlapping waves are not jointly optimized, and a drone waiting on one wave's result skips another.
- **Only partly exercised live.** The retry and back-off paths are covered by unit tests. Live runs so far had no message loss, so these paths never actually ran.
- **2D perception.** The lidar is planar, and voxels are placed at the drone's own altitude.
- **ORCA is simplified.** It resolves constraints by greedy projection rather than a full linear program, and it avoids other drones only through lidar voxels.
- **Untested at scale.** Performance above about 12 drones hasn't been measured.
- **No shutdown handling.** Agents don't handle SIGTERM, so `docker compose down` kills them (exit code 137).
- **No security.** Zenoh traffic is unauthenticated and unencrypted.
