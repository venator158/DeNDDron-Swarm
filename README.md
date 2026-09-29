# DeNDDron Swarm

**De**centralized **N**aval **D**efence **Dron**e swarm. Expendable drones hold station around a ship. The ship's radar reports incoming threats, and the operator approves each interception on a live dashboard. The drones then decide among themselves which of them engage. The assigned drones fly to the engagement point, wait there, and detonate at the allocated time.

The simulation runs in Gazebo. Every drone is its own Python agent in its own container. The drones and the ship talk peer-to-peer over Zenoh, with no central router.

This README is the project's only documentation. Keep it up to date when behaviour changes.

## Contents
- [Quick start](#quick-start)
- [Operator workflow](#operator-workflow)
- [Architecture](#architecture)
- [Engagement protocol](#engagement-protocol)
- [Degraded communications](#degraded-communications)
- [Instrumentation](#instrumentation)
- [Implementation map](#implementation-map)
- [Agent internals](#agent-internals)
- [Configuration](#configuration)
- [Testing](#testing)
- [Repository layout](#repository-layout)
- [Known limitations](#known-limitations)

## Quick start

Requirements: Docker with Compose v2, and an X11 display for the Gazebo window.

```bash
xhost +local:docker                                   # let containers open windows
bash scripts/run_swarm.sh 8 --threats 6               # 8 drones, radar generates 6 threats
# operator dashboard:  http://localhost:8080
docker exec -it gazebo_simulator gzclient             # optional: 3D view
docker compose down                                   # stop (every service shuts down cleanly)
```

The first run builds the images, which takes several minutes. Add `--build` after changing code.

| Option | Default | Meaning |
|---|---|---|
| `N` (first argument) | 3 | number of drones |
| `--threats K` | 0 (radar off) | threats the ship's radar generates; drones get no static goals |
| `--threat-interval S` | 30 | mean sim seconds between detections |
| `--first-threat S` | 20 | sim seconds before the first detection |
| `--algorithm orca\|apf` | `orca` | path planner |
| `--seed S` | 42 | spawn layout and threat scenario |
| `--build` | off | rebuild the images |

Radar environment variables (ship): `THREAT_TYPES` (`type:level:speed:weight,...`), `DETECT_MIN_M`/`DETECT_MAX_M` (150/190), `MAX_MISS_M` (30), `DEFENDED_RADIUS_M` (45), `KILL_RADIUS_M` (8).

With no `--threats`, drones fly to the static goals in `config/swarm_runtime.json`. That is a basic navigation check.

## Operator workflow

The dashboard is at `http://localhost:8080`. It is served by the ship and updates 4 times a second.

1. **Detection.** When the radar picks up a threat, the threat appears on the tactical map and in the **threat queue**. The queue is a min-heap ordered by TCPA (time to closest point of approach), so the most urgent threat is always on top. Each row shows:
   - type and level;
   - TCPA, and CPA distance from the ship;
   - time until the engagement point is reached;
   - **TTI(L)**: how long the `L` nearest free drones need to get there.
2. **Feasibility.** A threat can be approved only if both hold:
   - at least `level` drones are free;
   - `TTI(level) + 2 s slack < time until engagement`.

   Otherwise the Approve button is disabled and the reason is shown.
3. **Approval.** The operator selects the threat and approves it. The ship re-checks feasibility at that moment and sends the engagement order to the swarm.
4. **Engagement.** The swarm assigns drones itself. The map draws each engaged drone's line to its engagement point. The event log records: detected, approved, engaged, detonation (with miss distance), and then destroyed, leaked or impact.

Threat levels are both priority and the number of drones needed:

| Type | Level | Speed |
|---|---|---|
| `uav` | 1 | 2.5 m/s |
| `missile` | 2 | 3.5 m/s |
| `cruise_missile` | 3 | 4.5 m/s |

Speeds are scaled to the drones' 4 m/s.

### Engagement geometry
The ship reports each threat as a straight-line track with its CPA and TCPA. The drones intercept at the **engagement point**:
- the CPA, if the threat passes outside the defended radius (45 m);
- otherwise, the point where the track first crosses the defended radius. A threat aimed at the ship has its CPA on the ship itself, so intercepting there would be too late.

The assigned drones take up slots 4 m apart around that point and **detonate at the allocated time**, the moment the threat arrives. The ship assesses kills with its radar: a detonation within 8 m of the threat's true position counts as a hit. A threat is destroyed once it has `level` hits.

## Architecture

```
                 sim_net (Zenoh router "sim_bus")              radio_net (peer-to-peer, no router)
                 = each drone's own sensors/actuators          = all communications
  ┌──────────────────┐   sensors 50 Hz, lidar 10 Hz   ┌──────────┐   heartbeats, orders,   ┌──────────┐
  │ Gazebo simulator │ ─────────────────────────────▶ │ drone ×N │ ◀────bids, awards─────▶ │ drone ×N │
  │ (C++ bridge)     │ ◀──── cmd_vel, detonation ──── │ (Python) │                         └──────────┘
  │                  │ ── sim clock ──┐                └──────────┘                              ▲
  │                  │ ◀─ tracks ──┐  │                      ▲ roster, orders    heartbeats,     │
  └──────────────────┘             │  ▼                      │                   telemetry       │
                                 ┌─────────────────────────────┐                                 │
                                 │ ship: radar, C2, dashboard  │ ────────────────────────────────┘
                                 └─────────────────────────────┘   metrics node: sim_net observer
```

- **Two links per node** (`src/common/links.py`).
  - The *onboard bus* goes through a Zenoh router on `sim_net`. It stands in for a drone's own wiring (sensors, actuators, detonation), and for the ship's radar truth. It is not communications.
  - The *radio* runs peer-to-peer on `radio_net` (Zenoh 1.10.1). Peers find each other by multicast scouting on the radio interface and connect directly. Every drone and the ship run the radio **in a separate OS process** (`src/common/radio_process.py`), so a radio failure cannot freeze flight control or C2.
  - Degrading `radio_net` degrades only the communications. The physics keeps working.
- **Simulator** (`sim/GazeboSimulator.cpp`).
  - Integrates the drones' motion from `cmd_vel`.
  - Publishes pose at 50 Hz and a compact 32-ray planar lidar at 10 Hz.
  - Publishes the sim clock at 10 Hz.
  - Moves threat models along the ship's tracks, nose along the direction of flight: a fixed-wing UAV (yellow), a finned missile (orange), a longer winged cruise missile (red). Drones are quadcopters in their swarm colour. All shapes are visual-only primitives, so they add no physics load.
  - Despawns drones for good when they detonate.
  - Gazebo reports sim time only every 0.2 s, so the bridge extrapolates between updates using the observed real-time factor.
- **Ship** (`src/ship/ship.py`): radar simulation, threat queue, roster, feasibility, orders, kill assessment, instrumentation aggregation, and the dashboard (`dashboard.html`).
- **Drones** (`src/agent/`): perception, planning, the 50 Hz control loop, decentralized allocation, heartbeat and telemetry.

### Topics

| Topic | Link | Direction | Payload |
|---|---|---|---|
| `drone/{id}/sensors` | onboard | sim → drone, metrics | `{sim_time, pose, lidar?{angle_step, ranges[], hits[]}}` |
| `swarm/{id}/cmd_vel` | onboard | drone → sim | `{linear, angular}` |
| `swarm/agents/join`, `swarm/agents/despawn` | onboard | drone → sim, metrics | spawn / remove this drone |
| `sim/clock` | onboard | sim → ship | `{sim_time}` at 10 Hz |
| `sim/detonation` | onboard | drone → ship | `{agent_id, threat_id, sim_time, x, y, z}` (physical event, observed by radar) |
| `sim/threat_tracks` | onboard | ship → sim | `{threat_id, type, level, status, t0, p0, v}` for Gazebo markers |
| `swarm/heartbeat/{id}` | radio | drone → all | `{state, link, pose, threat_id, t_engage}` at 2 Hz |
| `swarm/heartbeat_relay/{id}` | radio | drone → ship | a peer's heartbeat, forwarded when the ship's roster lacks that peer |
| `swarm/telemetry/{id}` | radio | drone → ship | instrumentation, 1 Hz |
| `ship/roster` | radio | ship → drones | `{count, members[]}` at 1 Hz: drones the ship hears |
| `swarm/threats` | radio | ship → drones | engagement order `{wave_id, threats[{threat_id, type, level, required, location, t_engage}]}` |
| `swarm/bids` | radio | drone → all | `{agent_id, wave_id, costs{threat_id: ETA s}}` |
| `swarm/awards` | radio | drone → all, ship | `{threat_id, agent_id, cost, status: engaged\|withdrawn\|missed, slot, t_engage}` |
| `ship/threat_status` | radio | ship → all | `{threat_id, status}` |

## Engagement protocol

The allocation is decentralized (`src/agent/auction.py`).

1. The ship publishes an engagement order on `swarm/threats`. It carries the engagement point, the detonation time, and `required` drones.
2. Each **free** drone (idle, localized, not waiting on another order) bids its ETA to the point. The ETA uses a trapezoidal speed profile at 4 m/s and 1 m/s², times a 1.25 margin. A drone only bids if it can arrive before the detonation time.
3. After a 1 s bid window (sim time), every drone runs the same deterministic assignment:
   - threats are taken highest level first;
   - each threat gets its `required` fastest drones, ties broken by drone ID;
   - **all or nothing**: a threat that cannot get every drone it needs gets none, and those drones stay free.
4. Each winner publishes an award, takes its slot (slot index = rank among the winners, sorted by ID), and flies there.
5. **Conflict repair.** If views diverged and a threat collects more than `level` drones, the drones with worse bids withdraw. If a threat is left short, the ship re-announces it for the missing drones, at most 3 times and only while there is still time.
6. At the detonation time, a drone within 8 m of its slot (the kill radius) detonates:
   - it publishes `sim/detonation`;
   - it despawns;
   - its container stays up but idle, so it is never respawned.

   A drone that is not in position aborts, publishes `missed`, and holds position.

Decision latency from approval to the last award is about 0.8–1.2 s. Almost all of it is the 1 s bid window.

### Membership and link loss
- Each drone heartbeats at 2 Hz. The ship publishes the roster (drones heard in the last 3 s) at 1 Hz. The dashboard's drone count and free count come from it.
- A drone considers its **radio link up** while it hears the ship or any peer within 3 s.
- **Link lost, no job:** the drone holds position and does not bid.
- **Link lost, engaged:** the drone continues to its slot and detonates at the allocated time. That needs only local state. The detonation is observed through the onboard bus, standing in for the ship's radar.
- **Link lost at auction close:** the drone ignores the result, because it was computed from whatever bids reached it.
- **Not in the ship's roster although it hears the ship:** the drone keeps heartbeating and logs a warning. Any drone that is itself in the roster and hears that peer forwards the peer's heartbeat on `swarm/heartbeat_relay/{id}` once a second. The ship accepts it as membership, and the dashboard marks the drone *relayed*. Zenoh 1.10 peers do not relay for each other, so this is done at the application level.

## Degraded communications

Radio loss is emulated two ways: 100% packet loss on the radio interface (`tc netem` through Pumba, which is closest to jamming because the interface stays up), and removing the interface (`docker network disconnect`). Both gave the same results. The tools are in `tools/comms/` (see below).

| Finding | Evidence |
|---|---|
| Zenoh tries to reconnect to a lost peer with a 10 s connect timeout. During those attempts, other peers lost **all** traffic from healthy peers for ~8–10 s. | radio probe: 2 of 3 healthy peers heard nothing for 8.4 s (1.0.4) / 9.8 s (1.10.1) |
| **Fix 1:** Zenoh 1.10.1 with a 1 s open/accept timeout (`RADIO_OPEN_TIMEOUT_MS`) | radio probe: worst gap 0.05 s on every healthy peer. (On 1.0.4, `linkstate` routing had the same effect, but it no longer exists in 1.10.) |
| The process whose radio is lost freezes **all** its Zenoh sessions once for ~10 s: every timeout setting, both Zenoh versions, both loss methods | onboard probe: 10.1–10.4 s gap in the jammed drone's 50 Hz onboard stream |
| **Fix 2:** the radio in its own OS process (`RadioProcess`) | onboard probe: jammed drone's worst onboard gap 25 ms (was 10.4 s) |
| Full system, before the fixes: the ship froze ~10 s when an engaged drone was cut, and cut drones missed their slots | runs 3/4: ship clock stopped for 10 s; cut drone 31.7 m / 5.0 m off its slot at detonation time, so it aborted |
| **Full system after the fixes:** a drone jammed right after engaging kept flying on its onboard sensors and destroyed its threat on time. The ship never froze. | run 5: detonation at t = 76.45 s (allocated 76.3 s), 2.86 m from the threat; 0 frozen ship-clock samples |
| **Still open:** while a peer is being jammed, some *healthy* drones' radio receive stalls ~7 s. Their sending keeps working, and the ship keeps hearing them. | run 5: up to 4 healthy drones reported link down for ~7 s, then recovered. Not reproduced in the isolated probes, so it depends on something the full system has. |

### Tools (`tools/comms/`)

| Script | What it does |
|---|---|
| `radio_probe.sh [routing] [netem\|disconnect]` | 4 radio-only peers; cuts one; reports per healthy peer the seconds it heard nobody, the worst gap and the worst process stall |
| `onboard_probe.sh [routing] [netem\|disconnect]` | drone-like processes with both links; cuts one radio; reports the worst onboard gaps of the cut drone and a healthy one. Set `RADIO_PROCESS=1` to use the separate radio process. |
| `cut_radio.sh <container> <network> <subnet> [netem\|disconnect] [duration]` | cuts one container's radio (used by the others) |
| `chaos.py <compose log> [disconnect\|netem]` | during a live run: cuts an idle drone, then the first drone that engages, then restores the idle one |
| `operator_bot.py [reaction_s] [duration_s]` | stand-in operator: approves feasible threats through the dashboard API, most urgent first |

All probes accept `IMAGE=...` to test another Zenoh build, and they read `RADIO_LEASE_MS` / `RADIO_OPEN_TIMEOUT_MS`.

## Instrumentation

Every drone sends `swarm/telemetry/{id}` once a second, covering the last second. The dashboard's *Swarm instrumentation* table shows it:

| Field | Meaning |
|---|---|
| `cpu_pct` | process CPU |
| `loop_hz`, `loop_p50_ms`, `loop_p99_ms`, `loop_work_p99_ms`, `overruns` | 50 Hz control loop timing. An overrun is a tick longer than 1.5 × 20 ms. |
| `sensor_age_p50_ms`, `sensor_age_max_ms` | age of the newest onboard sensor frame, sampled every tick |
| `perception_p99_ms`, `planner_p99_ms` | lidar processing per scan; planner per tick |
| `rx_per_s`, `tx_per_s`, `tx_bytes_per_s` | radio messages per topic, and bytes sent |
| `peers_heard`, `voxels` | peers heard in the last 3 s; voxel map size |

The ship adds per-topic radio receive rates, per-threat decision latency (approval → first and last award), and an event log. Events also go to `/state/ship_log.jsonl` in the `swarm_state` volume, for offline analysis. The metrics node logs positions, distance flown and collisions (under 2.5 m).

Measured at 8 drones: about 1.5% CPU per drone, loop p99 about 20–26 ms, perception p99 about 3–12 ms. Radio to the ship is about 170 B/s of heartbeats and 220 B/s of telemetry per drone.

## Implementation map

Where each implemented feature lives.

| Area | Feature | Files |
|---|---|---|
| **Launch** | swarm launcher: drones, threat scenario, rebuilds | `scripts/run_swarm.sh` |
| | spawn layout and planner/kinematics defaults (`swarm_runtime.json`) | `scripts/generate_swarm_config.py` |
| | services, the two networks, healthchecks, clean shutdown | `docker-compose.yml`, `docker/*.Dockerfile` |
| **Simulator** | Gazebo bridge: kinematic integration from `cmd_vel`, spawn/despawn | `sim/GazeboSimulator.cpp/.hpp`, `sim/simulator_main.cpp` |
| | sim clock extrapolated between Gazebo's 5 Hz stats; pose 50 Hz, lidar 10 Hz | `GazeboSimulator.cpp` (`estimated_sim_time`, `step`) |
| | planar 32-ray lidar (ship + other drones), compact scan | `GazeboSimulator.cpp` (`simulate_lidar`) |
| | threat models (by type) moved along the ship's tracks | `GazeboSimulator.cpp` (`generate_threat_sdf`, `on_threat_track`, `move_threat_markers`) |
| | quadcopter drone model; model deletion (despawn, destroyed threats) | `GazeboSimulator.cpp` (`generate_drone_sdf`, `delete_model`) |
| | world: ocean, lighting, frigate (~31 m, sized to the simulator's 16 m ship radius) | `sim/ocean.world` |
| **Links** | onboard bus (router) and radio (peer-to-peer) session config | `src/common/links.py` |
| | radio in a separate OS process | `src/common/radio_process.py` |
| **Threat model** | threat types and levels, CPA/TCPA, engagement point, slots, ETA, TTI | `src/common/threats.py` |
| **Ship C2** | radar simulation (track generation) | `src/ship/ship.py` (`_maybe_detect`, `Track`) |
| | threat min-heap by TCPA | `src/ship/threat_queue.py` |
| | roster from heartbeats (+ relayed) | `ship.py` (`_on_heartbeat`, `_on_relayed_heartbeat`, `members`) |
| | feasibility (TTI vs. time to engagement), operator approval, orders, re-announcement | `ship.py` (`feasibility`, `approve`, `_announce`, `_retry_underassigned`) |
| | kill assessment, leak/impact/failed | `ship.py` (`_on_detonation`, `_age_tracks`) |
| | instrumentation aggregation, event log (`/state/ship_log.jsonl`), HTTP/SSE API | `ship.py` (`snapshot`, `make_handler`) |
| | operator dashboard (map, heap queue, approval, swarm table, radio, events) | `src/ship/dashboard.html` |
| **Drone** | agent lifecycle, ID allocation, SIGTERM | `src/agent/main.py` |
| | perception: voxel map, occupied index, batched ray tracing, expiry | `src/agent/voxel_map.py`, `agent.py` (`_process_lidar`) |
| | planners (APF, ORCA) | `src/agent/path_planning.py` |
| | 50 Hz control loop, safety envelope, arrival latch | `agent.py` (`_reflex_control_loop`) |
| | sim-time / wall-time handling | `src/agent/timing.py` |
| | bidding (ETA), engagement, slots, detonation / abort | `agent.py` (`_on_threat_wave`, `_calculate_costs`, `_service_auctions`, `_check_engagement`) |
| | decentralized all-or-nothing priority assignment, conflict yield | `src/agent/auction.py` |
| | heartbeat, roster check, link state, peer relay | `agent.py` (`_heartbeat_loop`, `_on_roster`, `_relay_unheard_peers`) |
| | telemetry (loop, sensors, perception, planner, CPU, radio) | `src/agent/telemetry.py` |
| **Metrics** | positions, distance, proximity collisions (spatial hash) | `src/metrics/main.py` |
| **Tests / tools** | unit tests and validation scripts | `tests/` (see [Testing](#testing)) |
| | degraded-comms probes, radio cut, chaos, stand-in operator | `tools/comms/` |

## Agent internals

`src/agent/agent.py`:

1. **Eyes: `voxel_map.py`.**
   - A sparse 3D occupancy grid with 0.5 m voxels.
   - Each lidar scan is ray-traced in one vectorized batch.
   - Occupied voxels are indexed separately, so obstacle queries touch only those (about 0.01 ms).
   - Voxels expire after 0.5 s, cleaned up incrementally batch by batch.
2. **Reflexes: `path_planning.py` and the 50 Hz loop.**
   - Planning uses APF or ORCA against the voxels and the ship.
   - The loop then applies:
     - a floor/ceiling guard,
     - damping near obstacles,
     - service-radius containment,
     - a braking envelope,
     - acceleration and speed limits.

     Arrival latches and the drone holds position.
   - An untasked drone keeps sending zero velocity, so it holds.
3. **Timing: `timing.py`.** Simulation time drives the physics integration. Wall-clock time drives sensor freshness. If sensor data is stale, the drone commands zero velocity.
4. **Engagement: `auction.py` + `src/common/threats.py`.** Covers bidding, assignment, slots, and the detonation time. See [Engagement protocol](#engagement-protocol).
5. **Telemetry: `telemetry.py`.**

## Configuration

`config/swarm_runtime.json` is generated on every launch and is not tracked in git. To change the defaults, edit `GLOBAL_DEFAULTS` in `scripts/generate_swarm_config.py`.

| Section | Keys |
|---|---|
| `defaults.path_planning` | `algorithm`, APF gains and radii, `step_size`, `goal_tolerance`, `braking_radius`, `ship_keepout_radius`, `velocity_smoothing`; optional `time_horizon_obst`, `agent_radius`, `max_control_dt`, `sensor_timeout_s` |
| `defaults.kinematics` | `max_velocity` (4 m/s), `max_acceleration` (1 m/s²), `min_z`, `max_z`, `max_service_radius` — also used by the ship's TTI |
| `defaults.goal_control` | `tolerance`, `stop_radius`, `tolerance_xy`, `tolerance_z`, `settle_ticks` |
| `defaults.auction` (optional) | `bid_window_s` (1.0) |
| `agents.drone_N` | `spawn{x,y,z}`, optional `goal{x,y,z}` and per-drone overrides |

Radio tuning (env): `RADIO_OPEN_TIMEOUT_MS` (1000), `RADIO_LEASE_MS` (2000), `RADIO_SUBNET` (`172.21.0.0/16`).

Drone IDs: agent replicas are identical containers. Each claims the lowest free `drone_N` via `config/agent_registry.json`, under a file lock.

## Testing

```bash
python3 -m pytest tests -q                 # unit tests, ~1 s
python3 tests/run_all_validations.py       # validation suite, ~75 s; writes pre_consensus_validation_report.md (git-ignored)
```

| File | Covers |
|---|---|
| `test_auction.py` | deterministic priority assignment, all-or-nothing, tie-breaks, early/late bids, conflict yield |
| `test_threats.py` | CPA/TCPA, engagement point (CPA vs. defended-radius crossing), slots, ETA, TTI, serialization |
| `test_threat_queue.py` | min-heap ordering by TCPA, lazy removal |
| `test_voxel_map_batch.py` | batched ray tracing, occupied index vs. brute force, incremental expiry, clock reset |
| `test_apf.py`, `test_orca_vertical_filter.py` | planner force bounds, ORCA vertical envelope |
| `test_timing_manager.py`, `test_time_handling.py` | timing states |
| `test_metrics_spatial_hash.py` | spatial-hash collision check matches brute force |
| `benchmark_voxelmap.py`, `validate_*.py` | throughput, planner scenarios, timing, metrics overhead, worker lifecycle |

## Repository layout

```
config/                 generated runtime files — not tracked
docker/                 base (zenoh-c/cpp), gazebo, agent, ship, metrics images
scripts/run_swarm.sh    launcher;  scripts/generate_swarm_config.py
sim/                    GazeboSimulator (C++ bridge, kinematics, markers), simulator_main.cpp, ocean.world
src/common/             threat model and geometry (threats.py), Zenoh links (links.py), radio process (radio_process.py)
src/agent/              drone: agent, auction, planners, voxel map, timing, telemetry
src/ship/               ship C2 (ship.py), threat min-heap (threat_queue.py), dashboard.html
src/metrics/main.py     metrics node (collisions, distance)
tests/                  unit tests and validation scripts
tools/comms/            degraded-comms probes, radio cut helper, chaos script, stand-in operator
```

## Known limitations
- **Healthy drones can lose radio receive for ~7 s while a peer is jammed.** See [Degraded communications](#degraded-communications).
- **The ship is a single point of failure, by design.** It is the only threat sensor and the only source of engagement orders.
- **Shared simulation clock.** Detonation times use the simulator's clock, which every node shares. A distributed clock is future work.
- **Idealized threats.** They fly straight lines at constant speed, and a detonation within the kill radius always kills (no kill probability).
- **Greedy assignment.** Orders are assigned per order, highest level first, not globally optimized. A drone waiting on one order's result skips any other order that arrives before that result.
- **Planar perception.** The lidar is 2D, and voxels are placed at the drone's own altitude. ORCA uses greedy projection, not a full linear program.
- **No security.** Radio traffic is unauthenticated: a forged order or bid would be acted on.
