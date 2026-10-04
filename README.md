# DeNDDron Swarm

**De**centralized **N**aval **D**efence **Dron**e swarm. Expendable drones hold station around a ship. The ship's radar reports incoming threats, and the operator approves each interception on a live dashboard. The drones then decide among themselves which of them engage. The assigned drones fly to the engagement point, wait there, and detonate when their proximity fuze sees the threat pass (or, with the fuze off, at the allocated time).

Each drone works out its own position from ultra-wideband (UWB) ranges to anchors on the ship and to its neighbours. It sees threats and other drones with a mmWave radar. It never leaves a 50 m no-fly zone around the ship. The simulator owns the physics: a drone only asks to detonate, and the simulator decides what the blast hits.

The simulation runs in Gazebo. Every drone is its own Python agent in its own container. The drones and the ship talk peer-to-peer over Zenoh, with no central router.

This README is the project's only documentation. Keep it up to date when behaviour changes.

## Contents
- [Quick start](#quick-start)
- [Operator workflow](#operator-workflow)
- [Architecture](#architecture)
- [Engagement protocol](#engagement-protocol)
- [Localization and perception](#localization-and-perception)
- [Proximity fuze](#proximity-fuze)
- [Distributed clock](#distributed-clock)
- [Degraded communications](#degraded-communications) (and [sensing degradation](#sensing-and-ship-link-sweep))
- [Instrumentation](#instrumentation)
- [Implementation map](#implementation-map)
- [Agent internals](#agent-internals)
- [Configuration](#configuration)
- [Testing](#testing)
- [Repository layout](#repository-layout)
- [Next steps](#next-steps)
- [Known limitations](#known-limitations)

## Quick start

Requirements: Docker with Compose v2, and an X11 display for the Gazebo window.

**More than ~30 drones:** raise the host's ARP table limit first. The radio is a full mesh, so every node needs an ARP entry for every other node, and Linux keeps one ARP table for all containers on the host. At the default `gc_thresh3 = 1024`, the mesh stops at about 32 nodes: later drones cannot reach anyone, and the kernel log fills with `neighbor table overflow`. `run_swarm.sh` warns when the swarm is too big for the limit.

```bash
sudo sysctl -w net.ipv4.neigh.default.gc_thresh1=4096 net.ipv4.neigh.default.gc_thresh2=8192 net.ipv4.neigh.default.gc_thresh3=16384
```

Put the same three settings in `/etc/sysctl.d/` to keep them after a reboot. 16384 covers about 125 drones.

```bash
xhost +local:docker                                   # let containers open windows
bash scripts/run_swarm.sh 8 --threats 6               # 8 drones, radar generates 6 threats
# operator dashboard:  http://localhost:8080
docker exec -it gazebo_simulator gzclient             # optional: 3D view
docker compose down                                   # stop (every service shuts down cleanly)
```

The first run builds the images, which takes several minutes. Add `--build` after changing code; the sweeps below do not rebuild.

**What runs by default.** Every default is the current architecture:
- **Localization:** cooperative UWB (`--localization coop`). Drones are never told their x, y; they range the ship's four UWB anchors, and their neighbours when out of anchor reach.
- **Perception:** the mmWave radar (`--perception radar`), for obstacle avoidance and the proximity fuze.
- **Ship no-fly zone:** 50 m (`--no-fly 50`), with stations at 55–95 m. Routes go around the zone, and intercepts stay outside it.
- **Proximity fuze:** on, with the `hold` fallback.
- **Calm air**, and perfect clocks (`--clock-sync none`).

The legacy setup is opt-in, for comparison runs only: the simulator's true x, y, a planar lidar with a voxel map, and stations at 30–45 m with no zone (`--localization truth --perception lidar --no-fly 0`).

**Common runs:**

```bash
# the standard check: 8 drones, 4 threats, 3x real time, rebuild; then the stand-in operator approves
bash scripts/run_swarm.sh 8 --threats 4 --rtf 3 --build
python3 tools/comms/operator_bot.py 1 400 0.5          # reaction s, duration s, poll s (wall time)

# no operator at all: the ship approves every feasible threat itself
bash scripts/run_swarm.sh 12 --threats 6 --rtf 3 --maneuver-p 0.5 --auto-approve

# wind (real m/s, scaled like the airframe) and a UWB jammer near (40, 0) from t = 30 s
bash scripts/run_swarm.sh 8 --threats 4 --rtf 3 --wind 5,0 --gust 1.5
UWB_JAM=anchors:30:9999:40:0:30 bash scripts/run_swarm.sh 8 --threats 4 --rtf 3

# anchors only (no peer ranging), or the legacy setup for comparison
bash scripts/run_swarm.sh 8 --threats 4 --rtf 3 --localization anchors
bash scripts/run_swarm.sh 8 --threats 4 --rtf 3 --localization truth --perception lidar --no-fly 0
```

| Option | Default | Meaning |
|---|---|---|
| `N` (first argument) | 3 | number of drones |
| `--threats K` | 0 (radar off) | threats the ship's radar generates; drones get no static goals |
| `--threat-interval S` | 30 | mean sim seconds between detections |
| `--first-threat S` | 20 | sim seconds before the first detection |
| `--algorithm apf\|orca` | `apf` | path planner (APF with goal-proximity repulsion fade; ORCA dead-ends in crowds, see [Spatial queue](#spatial-queue)) |
| `--seed S` | 42 | spawn layout and threat scenario |
| `--maneuver-p P` | 0 | probability that a threat turns once mid-flight (`THREAT_MANEUVER_P`) |
| `--localization coop\|anchors\|truth` | `coop` | how drones know x, y: UWB anchors + peers, anchors only, or the simulator's truth (legacy) (see [Localization and perception](#localization-and-perception)) |
| `--perception radar\|lidar` | `radar` | obstacle sensing: mmWave radar, or the legacy lidar + voxel map |
| `--no-fly R` | 50 | ship no-fly zone radius; stations at R+5 … R+45 m; `0` turns it off (stations at 30–45 m) |
| `--wind X,Y`, `--gust S` | calm | steady wind and gust strength, real m/s (× `speed_scale`); drones cannot sense either |
| `--auto-approve [S]` | off | the ship approves every feasible threat itself after S sim seconds (default 3); also a dashboard toggle |
| `--instance K` | 0 | run an independent swarm next to others (see [Several swarms at once](#several-swarms-at-once)) |
| `--rtf K` | 1 | run the simulation K times faster than real time (see [Faster than real time](#faster-than-real-time)) |
| `--radio-qos default\|tuned` | `default` | Zenoh QoS profile for the radio (`RADIO_QOS`) |
| `--clock-drift PPM`, `--clock-drift-spread PPM` | 0, 0 | drones' clock drift: fixed + uniform ±spread per drone (see [Distributed clock](#distributed-clock)) |
| `--clock-offset S`, `--clock-offset-spread S` | 0, 0 | drones' clock offset, the same way |
| `--clock-jitter S`, `--clock-seed N`, `--clock-sync MODE` | 0, 0, `none` | timestamp noise on sync exchanges, seed of the per-drone draw, sync mode |
| `--fuze on\|off` | `on` | proximity fuze; `off` = timed detonation at the allocated time (see [Proximity fuze](#proximity-fuze)) |
| `--fuze-fallback hold\|timed`, `--fuze-fire cpa\|radius`, `--fuze-window S` | `hold`, `cpa`, 2 | what happens when the window closes with no detection; firing rule; window half-width |
| `--fuze-noise M`, `--fuze-latency S` | 0.1, 0 | fuze sensor noise (per axis) and latency |
| `--ship-clock-drift PPM`, `--ship-clock-offset S` | 0, 0 | the ship's own oscillator |
| `--build` | off | rebuild the images |

Radar environment variables (ship): `THREAT_TYPES` (`type:level:speed:weight,...`), `DETECT_MIN_M`/`DETECT_MAX_M` (150/190), `MAX_MISS_M` (30), `DEFENDED_RADIUS_M` (45), `KILL_RADIUS_M` (8).

With no `--threats`, drones fly to the static goals in `config/swarm_runtime.json`. That is a basic navigation check.

### Faster than real time

`--rtf K` (`SIM_RTF`) runs the whole system K times faster, so experiments finish sooner. Speeding up Gazebo alone would change the results, because the swarm's protocol timers run on the computer's clock. So everything scales together:

- **Gazebo** steps its 1 ms physics at 1000·K steps per second.
- **The bridge** ticks every 20/K ms, so per simulated second poses stay at 50 Hz, radar at 20 Hz and UWB at 2 Hz.
- **Drones** run their control loop at 50·K Hz. The simulator filters each velocity command, so the command rate per simulated second must stay the same for the flight dynamics to match.
- **Protocol timers** (heartbeats, link and roster timeouts, re-announce delay, decision latency, radio rates) use `src/common/simclock.py`. It follows the simulator's actual clock: drones feed it the sim time from their sensor frames, and the ship from `sim/clock`. Between updates it runs at the measured sim speed, so timers stay correct when Gazebo falls behind the target. At 25 drones, Gazebo reached 1.9× against a 2× target, and the ship still received exactly the expected 75 messages per simulated second.
- **Radio impairments** must be scaled by hand: delay ÷ K and rate × K. Loss is unchanged. `degradation_sweep.py --rtf K` does this for you.

Compute metrics (loop timing, CPU) stay in real time. What doesn't scale:
- the computer's own processing latencies;
- Zenoh's internal timers;
- netem impairments, which are scaled by the *target* K, so a simulator that falls behind makes delays slightly longer in simulated terms.

Keep K small, and check `rtf_measured` in sweep results.

Measured at K = 3 with 8 drones (legacy lidar setup):
- Gazebo reaches 2.98× real time.
- Each drone uses about 15% of a CPU core; 8 drones plus Gazebo use about 2.3 of 6 cores.
- A sweep run takes 87 s instead of 201 s.
- Results match real-time runs: decision latency 1030 vs 1015 ms with no impairment, and 1440 vs 1445 ms at 200 ms delay.

### Unattended runs and sweeps

`tools/comms/degradation_sweep.py` runs complete experiments without anyone at the dashboard. For each run it:
1. launches the swarm (`run_swarm.sh`);
2. waits for every drone to join;
3. starts the stand-in operator (`operator_bot.py`), which approves every feasible threat, most urgent first;
4. waits until every threat is resolved;
5. saves `/api/summary`, the dashboard state and the logs, and tears the swarm down.

```bash
# one scenario, 3 runs: 12 drones, 6 threats, every other threat manoeuvres, 3x real time
python3 tools/comms/degradation_sweep.py --rtf 3 --drones 12 --threats 6 --maneuver-p 0.5 \
    --profiles default --conditions baseline= --repeats 3

# the same under radio impairments, 2 runs at a time on separate instances
python3 tools/comms/degradation_sweep.py --rtf 3 --drones 12 --threats 6 --maneuver-p 0.5 \
    --profiles default --conditions baseline= loss30="loss 30%" delay200="delay 200ms 50ms" --parallel 2

# any other swarm setting through --env, e.g. clocks and the fuze
python3 tools/comms/degradation_sweep.py --rtf 3 --drones 8 --threats 4 --profiles default --conditions baseline= \
    --env CLOCK_DRIFT_SPREAD_PPM=500 CLOCK_OFFSET_SPREAD_S=1.5 CLOCK_SYNC=consensus FUZE=0

# localization and environment: wind with gusts, then the legacy setup on the same scenario
python3 tools/comms/degradation_sweep.py --rtf 3 --drones 8 --threats 4 --profiles default --conditions baseline= \
    --env WIND_MPS=5,0 GUST_SIGMA_MPS=1.5
python3 tools/comms/degradation_sweep.py --rtf 3 --drones 8 --threats 4 --profiles default --conditions baseline= \
    --env LOCALIZATION=truth PERCEPTION=lidar NO_FLY_RADIUS_M=0

# scaling: 50 drones at 1x, the ship approving by itself
python3 tools/comms/scaling_sweep.py --sizes 50:1 --auto-approve
```

The sweeps run whatever images exist: rebuild first (`bash scripts/run_swarm.sh 1 --build`, then `docker compose down`, or `docker compose build`) after changing code.

| Option | Default | Meaning |
|---|---|---|
| `--rtf K` | 1 | simulation speed-up; operator reaction, timeouts and radio impairments are scaled to it |
| `--drones N` | 8 | number of drones |
| `--threats K` | 4 | threats the radar generates |
| `--maneuver-p P` | 0 | probability that a threat turns once mid-flight |
| `--interval S`, `--first S`, `--seed S` | 30, 25, 42 | mean sim seconds between detections, before the first, scenario seed |
| `--repeats N` | 1 | runs per cell |
| `--conditions NAME=NETEM ...` | built-in matrix | radio impairments (`NAME=` for none) |
| `--profiles ...` | `default tuned` | radio QoS profiles |
| `--env KEY=VALUE ...` | | any other swarm environment (clocks, fuze, localization, perception, no-fly zone, wind, threats) |
| `--reaction S`, `--timeout S` | 3, 360 | operator reaction and time allowed per run, sim seconds |
| `--parallel P`, `--instance K` | 1, 0 | runs at once on separate instances; which instance for a single run |
| `--auto-approve` | off | the ship approves threats itself instead of `operator_bot.py` |
| `--out DIR` | `results/sweep_<time>` | results: `results.csv`, `results.md` (mean ± sd per cell), and per run `swarm.log`, `operator.log`, `summary.json`, `state.json` |

`scaling_sweep.py` takes the same `--maneuver-p` and `--env`, with swarm sizes as `--sizes N:RTF ...` and N/2 threats per size.

**50 drones, watched in Gazebo.** The full-size unattended test: 50 drones, 25 threats detected every 4.8 s (the per-drone load of the scaling runs), half of them turning once, at real time (50 drones plus the Gazebo window is about what a 6-core host sustains at 1×). Raise the [ARP limit](#quick-start) first.

```bash
xhost +local:docker                                   # let the container open a window
python3 tools/comms/degradation_sweep.py --rtf 1 --drones 50 --threats 25 --interval 4.8 --maneuver-p 0.5 \
    --timeout 480 --profiles default --conditions baseline= --out results/n50_gui &
# once the simulator container is up (about 10 s after launch):
docker exec -d gazebo_simulator gzclient              # the 3D view; it closes when the sweep tears the swarm down
```

The dashboard at `http://localhost:8080` shows the same run live; results land in `results/n50_gui/`.

For a single interactive run with the stand-in operator instead of a person: start the swarm, then the bot (or launch with `--auto-approve` and skip the bot).

```bash
bash scripts/run_swarm.sh 12 --threats 6 --rtf 3 --maneuver-p 0.5
python3 tools/comms/operator_bot.py 1 400 0.5      # reaction s, duration s, poll s (wall time)
```

The same 50-drone scenario as a plain launch, approved by the ship, with the Gazebo window:

```bash
bash scripts/run_swarm.sh 50 --threats 25 --threat-interval 4.8 --maneuver-p 0.5 --auto-approve
docker exec -d gazebo_simulator gzclient
```

### Several swarms at once

`--instance K` runs an independent swarm next to others on the same host, for example sweep cells in parallel on a cluster node. `scripts/swarm_instance.py` maps K to the instance's settings; instance 0 is the default and keeps the original names, ports and subnets.

| | Instance 0 | Instance K ≥ 1 |
|---|---|---|
| Compose project | `denddron-swarm` | `denddron-swarm-K` |
| Containers | `ship`, `gazebo_simulator`, ... | `iK-ship`, `iK-gazebo_simulator`, ... |
| Dashboard / Gazebo ports | 8080 / 11345 | 8080+K / 11345+K |
| Sim / radio subnets | 172.20.0.0/16, 172.21.0.0/16 | 10.(210+K).0.0/17, 10.(210+K).128.0/17 |
| Config, env file | `config/`, `.swarm.env` | `config/instances/K/`, `.swarm.K.env` |

- **Images are shared**, so an instance never rebuilds. K goes up to 30.
- **Subnet clashes:** if the host already routes the 10.21x ranges, set `SWARM_SUBNET_BASE`, or set `SIM_SUBNET`, `RADIO_SUBNET` and `SIM_BUS_IP` directly.
- **Stopping an instance:** `docker compose -p denddron-swarm-K down`.

```bash
bash scripts/run_swarm.sh 8 --threats 4 --instance 2            # dashboard on :8082
python3 tools/comms/degradation_sweep.py --parallel 3 --rtf 3    # 3 runs at a time, instances 1-3
python3 tools/comms/scaling_sweep.py --sizes 8:3 16:3 --parallel 2
```

The sweeps, `degrade_radio.sh` (`SWARM_INSTANCE=K`) and `operator_bot.py` (`DASHBOARD_URL`) all work per instance. The ARP table is shared by every instance on a host, so the limit must cover the sum of all meshes; the sweeps warn when it does not. Plan CPU at about 8% of a core per drone per 1× of sim speed, plus about 0.3 core for each instance's Gazebo.

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
The ship reports each threat as a straight-line track with its CPA and TCPA. The drones intercept at the **engagement point**, the **earliest point on the track** (so the farthest from the ship) that satisfies all of these (see [Spatial queue](#spatial-queue)):
- enough free drones can reach it in time, with a 1.25× ETA margin plus 4 s of slack;
- it is within 140 m of the ship (`INTERCEPT_MAX_RANGE_M`);
- it is at least 55 m from the ship: the no-fly zone plus 5 m;
- every drone can reach it on a route around the no-fly zone;
- it is kept clear of other jobs.

The latest acceptable point, used as a fallback, is the old rule, with the defended radius raised to the zone + 5 m (55 m; 45 m with `--no-fly 0`):
- the CPA, if the threat passes outside the defended radius;
- otherwise, the point where the track first crosses the defended radius.

The assigned drones take up slots **stacked vertically** through that point (within ±3 m). Around the allocated time, the moment the threat should arrive, their **proximity fuze** arms, and each drone detonates as the threat passes closest; its job-mates fire with it (chain fire). With `--fuze off`, they detonate at the allocated time instead. Vertical stacking matters: obstacle avoidance is planar (radar contacts become points at the drone's own altitude, as the lidar's were), so job-mates stacked 3 m apart don't repel each other off their slots (a horizontal ring did). The ship assesses kills with its radar: a detonation within 8 m of the threat's true position counts as a hit. A threat is destroyed once it has `level` hits.

## Architecture

```
                 sim_net (Zenoh router "sim_bus")              radio_net (peer-to-peer, no router)
                 = each drone's own sensors/actuators          = all communications
  ┌──────────────────┐  altitude 50 Hz, radar 20 Hz,  ┌──────────┐   heartbeats, orders,   ┌──────────┐
  │ Gazebo simulator │ ───── UWB ranges 2 Hz ───────▶ │ drone ×N │ ◀────bids, awards─────▶ │ drone ×N │
  │ (C++ bridge,     │ ◀── cmd_vel, detonate, job ─── │ (Python) │                         └──────────┘
  │ physics, truth)  │ ── sim clock ──┐                └──────────┘                              ▲
  │                  │ ◀─ tracks ──┐  │                      ▲ roster, orders,   heartbeats,     │
  └──────────────────┘             │  ▼                      │ jobs, zones       telemetry       │
        │ sim/truth            ┌─────────────────────────────┐                                   │
        ▼ (evaluation only)    │ ship: radar, C2, dashboard  │ ──────────────────────────────────┘
  metrics node                 └─────────────────────────────┘
```

- **Two links per node** (`src/common/links.py`).
  - The *onboard bus* goes through a Zenoh router on `sim_net`. It stands in for a drone's own wiring (sensors, actuators, detonation), and for the ship's radar truth. It is not communications.
  - The *radio* runs peer-to-peer on `radio_net` (Zenoh 1.10.1): data over UDP (best-effort), and Zenoh's control messages (declarations) over a QUIC link next to it, which retransmits them (see [Degraded communications](#degraded-communications)). Peers find each other by multicast scouting on the radio interface and connect directly. Drones also connect to the ship's fixed radio address (`.2` of the radio subnet, port 7450) as a meeting point, and Zenoh gossip introduces the rest. Every drone and the ship run the radio **in a separate OS process** (`src/common/radio_process.py`), so a radio failure cannot freeze flight control or C2. A watchdog pings the radio process every second, and restarts it if it stops answering for 5 s (subscriptions are re-declared). At ~50 peers starting together, a drone's Zenoh session occasionally hung inside a call and the drone never joined.
  - Degrading `radio_net` degrades only the communications. The physics keeps working.
- **Simulator** (`sim/GazeboSimulator.cpp`).
  - Integrates the drones' motion from `cmd_vel` through a first-order command response, plus wind and gusts the drones cannot sense.
  - Publishes each drone's pose frame at 50 Hz **without x, y or horizontal velocity**: only altitude and attitude (barometer, IMU, compass). With the legacy `--localization truth` the frame carries the full pose.
  - Publishes UWB ranges (`drone/{id}/uwb`, 2 Hz): to the ship's four anchors and to the peers each drone asked for, with each peer's state attached, and with noise, a maximum range, dropouts, channel capacity and optional jamming.
  - Publishes each drone's mmWave radar scan (`drone/{id}/radar`, 20 Hz): unlabelled relative positions of every threat, drone and the ship's hull within 30 m, with range and angle noise. It feeds obstacle avoidance and the proximity fuze. With the legacy `--perception lidar`, it publishes a 32-ray planar lidar at 10 Hz and a 10 m fuze sensor (`drone/{id}/fuze`) instead.
  - Publishes the sim clock at 10 Hz.
  - Moves threat models along the ship's tracks, nose along the direction of flight: a fixed-wing UAV (yellow), a finned missile (orange), a longer winged cruise missile (red). Drones are quadcopters in their swarm colour. All shapes are visual-only primitives, so they add no physics load.
  - Owns the physics: a drone only *asks* to detonate (`drone/{id}/detonate`); the simulator places the blast at the drone's true position, destroys every other drone within the warhead's kill radius (except the same job's), despawns them for good, and publishes `sim/truth` for evaluation. Kill radius and every other device parameter come from the [hardware record](#hardware-record).
  - Gazebo reports sim time only every 0.2 s, so the bridge extrapolates between updates using the observed real-time factor.
- **Ship** (`src/ship/ship.py`): radar simulation, threat queue, roster, feasibility, orders, kill assessment, instrumentation aggregation, and the dashboard (`dashboard.html`).
- **Drones** (`src/agent/`): localization (UWB EKF and cooperative fusion), radar perception and the fuze, planning, the 50 Hz control loop, decentralized allocation, heartbeat and telemetry.

### Drone pipeline

One drone, from sensors to detonation (`src/agent/agent.py` unless noted):

| Stage | What happens | Where |
|---|---|---|
| **Localization: predict** | every 50 Hz pose frame moves the EKF forward to the frame's sim time with the last commanded velocity; the estimate replaces x, y in the pose everything else uses | `_on_sensor_data`, `_loc_predict`; `localization.py` (`Localizer.predict`) |
| **Localization: correct** | anchor ranges first, then (coop) peer ranges from drones whose anchor chain is fresher; gating and re-lock | `_on_uwb`; `localization.py` (`update_range`, `relock_from`); `coop.py` (`peer_update`) |
| **Localization: share** | 2 Hz: our state for peers' tables and the peers to range next | `_send_uwb_tx`; `coop.py` (`state_payload`, `choose_peers`) |
| **Perception** | radar contacts → planar obstacle points (while flying) and, when engaged, the fuze | `_on_contacts`; `radar_obstacles.py` |
| **Assignment** | order → ETA bids around blasts and the no-fly zone → deterministic auction → ship confirmation → job topic | `_on_threat_wave`, `_calculate_costs`, `_service_auctions`, `_apply_job`; `auction.py`, `jobs.py` |
| **Route** | trapezoidal-profile legs around the no-fly zone, with holds outside other jobs' blast windows; re-planned every second and on new zones; legs advance as each latches | `_service_route`, `_on_zones`; `deconflict.py` (`plan_route`, `detour`) |
| **Reflex (50 Hz)** | arrival latch (0.5 m in altitude), APF velocity with a no-fly barrier widened by 2σ, safety envelope (floor/ceiling, obstacle damping, service radius, braking split horizontal/vertical, acceleration and speed caps); idle drones hold station against wind | `_reflex_control_loop`; `path_planning.py` |
| **Fuze** | records mates, tracks contacts, gates on the predicted threat (+3σ), fires at closest approach inside `t_engage` ± 2 s; mates chain-fire | `_on_contacts`, `_fuze_decision`, `_on_mate_blast`, `_check_engagement`; `fuze.py` |
| **Detonation** | the drone asks (`drone/{id}/detonate`, with its estimated position); the simulator places the blast at its true position and applies friendly fire | `_detonate_now`; `GazeboSimulator.cpp` (`apply_detonations`) |
| **Clocks** | every protocol time (`t_engage`, zones, holds) is ship time: the drone's hardware clock read through its sync mode | `_proto_now`, `_inbound_job`; `localclock.py`, `timesync.py` |

### Topics

| Topic | Link | Direction | Payload |
|---|---|---|---|
| `drone/{id}/sensors` | onboard | sim → drone, metrics | `{sim_time, pose{z, yaw, ...}}` at 50 Hz: altitude and attitude only (legacy `truth`: the full pose; legacy `lidar`: plus `lidar{angle_step, ranges[], hits[]}` every 0.1 s) |
| `drone/{id}/radar` | onboard | sim → drone | `{sim_time, objects[[dx, dy, dz], ...]}` from the hardware record's mmWave radar (30 m, 20 Hz, range/angle noise): every threat, other drone and the ship's hull, relative and unlabelled; feeds obstacle avoidance and the fuze |
| `drone/{id}/fuze` | onboard | sim → drone | legacy `PERCEPTION=lidar` only: the same contact format at `FUZE_HZ` (50) within `FUZE_RANGE_M` (10) |
| `drone/{id}/uwb` | onboard | sim → drone | `{sim_time, anchors[[k, range]], peers[[id, range, state]]}`: UWB ranges to the ship's anchors (and requested peers, with each peer's state); not with `truth` |
| `drone/{id}/uwb_tx` | onboard | drone → sim | `{state{x, y, vx, vy, cov, z, t, hops, anchor_t}, peers[]}`: the state our UWB frames carry and the peers to range (`coop`) |
| `drone/{id}/loc` | onboard | drone → metrics | `{sim_time, est, cov, status, ...}` at 2 Hz: the drone's estimate, compared with `sim/truth` (evaluation only) |
| `swarm/{id}/cmd_vel` | onboard | drone → sim | `{linear, angular}` |
| `swarm/agents/join`, `swarm/agents/despawn` | onboard | drone → sim, metrics | spawn / remove this drone |
| `sim/clock` | onboard | sim → ship | `{sim_time}` at 10 Hz |
| `drone/{id}/detonate` | onboard | drone → sim | `{threat_id, truth_time, local_time, sync_time, t_engage, reason, chain, chain_delay_s, chain_by, trigger, est, intruders[]}`: the drone asks to detonate; `est` is where it believes it is (evaluation) |
| `drone/{id}/job` | onboard | drone → sim | `{job}`: the drone's current or last job, so the simulator exempts it from that job's blasts |
| `sim/detonation` | onboard | sim → ship, drones | the request plus `{agent_id, x, y, z, blast_sim_time}`: the blast, placed by the simulator at the drone's **true** position (physical event, observed by radar) |
| `sim/damage`, `drone/{id}/destroyed` | onboard | sim → ship / the victim | `{agent_id, cause: friendly_fire, by, by_threat, job, distance, truth_time}`: the simulator destroys every other drone within the warhead's kill radius, except drones on the same job |
| `sim/truth` | onboard | sim → metrics, evaluation | `{sim_time, drones{id: [x, y, z, vx, vy, vz]}}` at 10 Hz: true poses; **drones never subscribe** (`test_hardware.py` checks) |
| `ship/zones` | radio | ship → drones | `{zones[{threat_id, point, radius, t_engage}]}` at 1 Hz: every job's reserved blast |
| `sim/threat_tracks` | onboard | ship → sim | `{threat_id, type, level, status, t0, p0, v}` for Gazebo markers, in **truth** (the only track message not in ship time) |
| `sim/clock_eval` | onboard | drone → ship | `{agent_id, mode, truth, ship_est, offset, bound, hops?, ship?}` at 1 Hz, only with a sync mode or an imperfect clock; evaluation only (true sync error) |
| `swarm/heartbeat/{id}` | radio | drone → ship | `{time, state, link, pose, threat_id, t_engage, confirmed, wave_id?, cost?, hold_s?, t1?}` at 2 Hz (drones do not subscribe); `wave_id`, `cost`, `hold_s` (the award) only while unconfirmed; `t1` (local send stamp) only with `CLOCK_SYNC=master` or `consensus` |
| `swarm/heartbeat_help/{id}` | radio | drone → drones | the same heartbeat, only while the ship does not hear this drone directly |
| `swarm/heartbeat_relay/{id}` | radio | drone → ship | a peer's help heartbeat, forwarded by drones in the roster; with `master` or `consensus`, plus `relay_resid` (how long the relay held it) |
| `swarm/telemetry/{id}` | radio | drone → ship | instrumentation, 1 Hz |
| `swarm/clock/{id}` | radio | drone → drones | `{agent_id, tau, alpha, o, anchor, echo?}` at `CLOCK_BEACON_HZ` (0.5), only with `CLOCK_SYNC=consensus` |
| `ship/roster` | radio | ship → drones | `{time, count, members[], relayed[], sync?{drone: [t1, t2, t3]}, beacon?}` at 1 Hz (`sync` with `master` or `consensus`, `beacon` with `consensus`): drones the ship hears, and which of them only through relays |
| `swarm/threats` | radio | ship → drones | engagement order `{wave_id, threats[{threat_id, type, level, required, location, t_engage}]}` |
| `swarm/bids` | radio | drone → all | `{agent_id, wave_id, costs{threat_id: ETA s}}` |
| `swarm/awards` | radio | drone → all, ship | `{threat_id, agent_id, wave_id, cost, status: engaged\|withdrawn\|missed\|released\|no_detection, slot, t_engage}` |
| `ship/ack/{id}` | radio | ship → drone | the drone's inbox, subscribed once at startup: ACK/NACK `{threat_id, wave_id, accepted}` (plus the job state when accepted), and job updates `{threat_id, status, seq, point, t_engage, cpa, t_cpa, track, holders{drone: slot}, n_slots, intruders}` at 2 Hz to each of the job's drones, and in answer to heartbeats |
| `ship/threat_status` | radio | ship → all | `{threat_id, status}` |

Every absolute time on the radio (`t_engage`, `t_cpa`, `track.t0`, `time`, `sync_time`) is in the **ship's timebase** (see [Distributed clock](#distributed-clock)). With `CLOCK_SYNC=ttg`, orders, ACKs, jobs and zones also carry `sent`, the ship's send time.

## Engagement protocol

The allocation is decentralized (`src/agent/auction.py`).

1. The ship publishes an engagement order on `swarm/threats`. It carries the engagement point, the detonation time, and `required` drones. Each order is sent 3 times, 0.15 s apart (`ORDER_COPIES`); drones ignore repeats. Often only one or two drones can reach the intercept in time, and a lost order to them lost the threat, because a re-announcement comes 3 s later.
2. Each **free** drone (idle, localized, not waiting on another order) bids its ETA to the point, sending the bid 3 times, 0.2 s apart (receivers keep the latest per drone). The ETA uses a trapezoidal speed profile at 4 m/s and 1 m/s², times a 1.25 margin. A drone only bids if it can arrive before the detonation time.
3. After a 1 s bid window (sim time), every drone runs the same deterministic assignment:
   - threats are taken highest level first;
   - each threat gets its `required` fastest drones, ties broken by drone ID;
   - **all or nothing**: a threat that cannot get every drone it needs gets none, and those drones stay free.
4. Each winner is only **tentatively** engaged. It publishes an award (with the order ID) and starts flying toward its slot. The drones carry no seeker, so an engaged drone depends on the ship for the target's position; it is not fire-and-forget.
5. **The ship confirms** (`src/common/jobs.py`). It collects the awards for a threat for 0.3 s, then confirms the best bids up to the number of drones still needed, and gives each a slot. Slots go to the confirmed drones in drone-ID order, the same convention a tentative drone uses for itself, so confirmation doesn't move a drone that is already flying. Assigning slots by bid order moved 17 of 41 confirmed drones across the formation at 50 drones, causing misses of up to 24 m. It sends each one an ACK in the drone's **inbox**, `ship/ack/{drone}`; the rest get a NACK and become free again. The ship hears every drone, so this also settles conflicts between drones that could not hear each other.
   - The radio is best-effort, so each award and withdrawal is sent 3 times (now and on the next two heartbeat ticks, 0.5 s apart). A newer status for the same threat replaces the pending copies. The ship ACKs every copy it gets from a confirmed drone.
   - **Heartbeats carry the job** (`threat_id`, `confirmed`, and while unconfirmed the order ID and bid), and the ship answers each one that disagrees with its own view (`heartbeat_answer` in `jobs.py`): an unconfirmed holder gets its ACK again, a drone whose award never arrived is taken into arbitration from its heartbeat, a refused one gets its NACK again, and a drone holding a job it no longer has gets the job update that releases it.
   - An unconfirmed drone **keeps flying** to its slot and asking (every heartbeat) until a confirmation could no longer help: the detonation time has come, or a straight flight at full speed would arrive after it. Then it withdraws and becomes free. Two drones can briefly head for one slot until the ship arbitrates. A drone never detonates without confirmation.
   - Until confirmed, a drone still yields to better awards from its peers. Once confirmed, only the ship can release it.
6. **Job updates.** While a job is active, the ship sends a job update at 2 Hz to the inbox of each of the job's drones (pending or confirmed). It carries the latest engagement point and detonation time, the expected CPA, the track, and the confirmed drones with their slots. Being listed there also counts as an ACK, which covers a lost ACK.
   - **Why an inbox, not a topic per job.** Over the UDP radio, a subscriber declaration can be lost, and Zenoh doesn't retry it. A drone used to subscribe to `ship/jobs/{threat}` when it won. Under 30 % loss, about 30 % of those subscriptions heard nothing for the whole engagement, so confirmed drones heard nothing and gave up (`tools/comms/declare_probe.sh`: 12 of 40 late subscriptions silent for 3 s; of the lost ones, about half recovered after 5–19 s and the rest not within 40 s). The inbox is declared once at startup. The radio is peer-to-peer, so per-drone sends cost the same as one shared topic. The root cause, lost control messages, was later fixed in the radio itself (a QUIC control plane, see [Degraded communications](#degraded-communications)); the inbox stays because it needs no declaration at the moment of winning.
   - Drones re-aim on every update. Threats can manoeuvre (`--maneuver-p P`: with probability P a threat turns once, re-aiming past the ship), and the update reaches the drones within one publish.
   - After a manoeuvre the ship re-picks the intercept point for the job's drones from where they are, keeping only 1 s of slack (`MANEUVER_REPLAN_SLACK_S`): they need the job update, not a new order. With a new order's 4 s, a late turn (threat ~100 m out, drones already on station ~85 m out) found no point in 1 case in 8 and fell back to the legacy point near the ship, and put 1 in 4 inside 60 m. Drones then flew 25–40 m back towards the ship. With 1 s: none and ~1 in 70 (200 simulated turns). The `maneuver` event logs the point's distance from the ship before and after, and whether the re-plan found one (`replanned`).
   - Measured, every threat manoeuvring (`--maneuver-p 1`, `--rtf 3`), 9 threats in two runs: all destroyed by the fuze, the points stayed within 6 m of their approved distance (62–99 m), and blasts went off 65–98 m from the ship. Before the fix, the same scenario engaged two of three threats at 53 and 60 m instead of 85 m. Misses are 1.9–3.6 m: the drone now moves sideways onto the new track, so its ~2.8 m arrival tolerance ends up across the track, where the fuze cannot make up for it.
   - When the threat is resolved, the job says so (3 times), and drones that have not detonated become free again.
   - A confirmed drone that is not listed any more has been released, and becomes free. The ship drops a confirmed drone whose heartbeat shows no job for 2 s, which covers a lost withdrawal.
7. **Re-announcement.** If a threat is left short, the ship re-announces it for the missing drones, at most 3 times and only while there is still time. It counts confirmed, pending and detonated drones, and drones whose latest heartbeat says they are engaged, so a lost award never sends a second drone.
8. **Detonation** (see [Proximity fuze](#proximity-fuze)). A confirmed drone's fuze arms at the detonation time ± 2 s and fires as the threat passes; a job-mate's detonation fires it too (chain fire). With `--fuze off`, a confirmed drone within 8 m of its slot (the kill radius) detonates at the detonation time. Either way it:
   - publishes `sim/detonation`;
   - despawns;
   - stays up as an idle container, so it is never respawned.

   A drone that is not within 8 m of its slot at the detonation time aborts, publishes `missed`, and holds position. With the fuze, a drone whose window closes with no detection holds and publishes `no_detection` (`--fuze-fallback timed`: detonates then instead).

Decision latency, from approval until every drone the threat needs is confirmed, is about 1.4 s: the 1 s bid window, the 0.3 s confirmation window, and radio round trips.

Measured (8 drones, `--rtf 3`):
- **Ship-link loss** (inbox, heartbeat answers, order copies; 15 drones, 8 threats, one run each, 2026-10-04): `ship_loss30` 8/8 (before: 4/8, then 3/8 and 4/8 in two diagnosis runs), every engagement confirmed within 1.2 s, no re-announcement; `ship_outage` 8/8; `combined` 8/8 (before: 4/8). In the `ship_loss30` run, 4 drones whose award was lost were confirmed from their heartbeats.
- **30% loss:** 6 runs, 4/4 destroyed with exactly 4 drones in every run. The ship settled 5 conflicts, where two drones both believed they had won.
- **Manoeuvres:** 3 two-drone threats, each turning once, moved their engagement points by 18, 10 and 22 m. The first two were destroyed. For the third, the point moved 22 m with only 1.7 s more time, so its drones could not reach it; they aborted and stayed alive instead of detonating 25 m away.
- **The same manoeuvre scenario in real time** (with the Gazebo window) gave an identical result: same drones, slots, point shifts and miss distances as the `--rtf 3` run.

## Localization and perception

Drones localize themselves instead of receiving their true x,y, and avoid each other with a mmWave radar instead of lidar and a voxel map. This is the default (`LOCALIZATION=coop`, `PERCEPTION=radar`, `NO_FLY_RADIUS_M=50`). The original behaviour (`truth`, `lidar`, `0`) is kept as a legacy option for comparison runs, and only then are the voxel map and the lidar loaded. Altitude and attitude are always given (barometer, IMU and compass, taken as perfect); localization is in the ground plane. Every sensor parameter comes from the [hardware record](#hardware-record).

**Perception (`PERCEPTION=radar`, default).** The simulator turns the contact sensor into the record's 60 GHz mmWave radar (30 m, 20 Hz, 5 cm range and 2° angle noise, 360° from four boards; `drone/{id}/radar`) and stops computing lidar. The radar's contacts feed both the proximity fuze and obstacle avoidance. For avoidance, `radar_obstacles.py` builds exactly the obstacle points the planar lidar produced: the same 32 rays, each neighbour a 3 m circle at the drone's altitude, checked against a Python port of the simulator's lidar. The planners are unchanged, and nothing is ray-traced into a voxel map. Avoidance uses relative positions, so it does not depend on localization.

**Localization (`LOCALIZATION=coop`, default, or `anchors`).** The simulator withholds x, y and the horizontal velocity, and gives UWB ranges instead (`drone/{id}/uwb`). Four anchors sit on the ship at the hull corners (±15 × ±5 m, 8 m up). The ranges have the record's characteristics: 10 cm noise, 250 m range, 2 % dropouts, and channel airtime. Each drone runs an EKF (`localization.py`) on [x, y, vx, vy, wind x, wind y]:
- **prediction** with the velocity it commands, through the airframe's command response (the simulator's filter; on hardware, the identified response), plus the wind it cannot sense, which the anchors make observable;
- **updates** from 3D anchor ranges with its altitude known, with an innovation gate and re-initialization after 5 rejections in a row;
- **robust anchor updates** (`UWB_FILTER=robust`, default; `basic` is the plain filter), for an environment worse than the record:
  - *adaptive noise:* the range noise is estimated from the last 40 anchor innovations (the median, which outliers don't pull), floored at the record's 0.1 m and capped at 2 m;
  - *Huber update:* an innovation beyond 2.5σ is down-weighted instead of rejected. A blocked path only ever lengthens a range, so a range too long beyond the 3.29σ gate is still rejected, and one too short only beyond 8σ;
  - *safe re-locks:* a re-lock fix from 3 or more anchors must pass a residual test (χ², 99 %) at the estimated noise. With 4 anchors, leaving one out is tried too, so one blocked anchor cannot pull the fix. Localization is 2D, so 3 anchors leave one range to check;
  - peer ranges keep the plain gate: a peer's error is its estimate's, not a blocked path;
- **initialization** from the launch position, or by least squares from three or more anchors.

**Cooperation (`coop`, `coop.py`).** Each ranging exchange carries the responder's state: estimate, covariance, velocity, altitude, and the time of the last anchor fix in its chain. Each drone keeps a neighbour table from those states and ranges its 6 nearest peers (plus a rotating slot for discovery). A drone that ranges the anchors itself never uses peers. Others use a peer only if its chain reached an anchor more recently than theirs did, adding the peer's uncertainty along the line of sight. They also never let their own variance along that line drop below the peer's plus the ranging noise: the peer's error is a persistent bias, not fresh noise. Earlier versions failed in two ways that tests reproduce:
- updating as if the peer's error were fresh noise made estimates 7× overconfident;
- counting hops instead of times let a jammed swarm pass "anchored" around in a loop.

Absolute position is only observable through the anchors: with every anchor jammed, no chain is fresher than another, nobody uses peers, and each drone dead-reckons with its learned wind while its covariance grows.

**Environment.** The record's `environment` entry adds wind and gusts that push the drones' true positions (`--wind 5,0 --gust 1.5`, real m/s × `speed_scale`; default calm). Wind made two more things necessary:
- the wind state in the filter: without it, a 0.5 m/s wind made it 600× overconfident and re-lock 290 times in one run;
- station keeping: idle drones fly back after drifting 1 m, and a drone latched on its goal re-approaches after drifting 1 m beyond where it latched. Neither triggers in calm air.

**Ship no-fly zone (`--no-fly R`, `NO_FLY_RADIUS_M`, default 50; 0 turns it off).** Shrapnel safety: drones stay out of a circle of radius R around the ship.
- **Stations** move to R+5 … R+45 m (55–95 m for 50).
- **Planners:** APF repels from the zone's boundary, widened by 2σ of the drone's own position uncertainty; ORCA uses the boundary as its hard constraint.
- **Routes** (`deconflict.plan_route`, used for bids, ETAs, the ship's feasibility and confirmation) go around the zone instead of through it: tangent, arc and tangent waypoints at R + 3 m, the shorter way round (`detour`).
- **Intercept points** stay at least R + 5 m out, and the fallback point (the old 45 m defended-radius crossing) moves out to that radius.

**Uncertainty in the gates.**
- **Fuze:** a contact must lie within the gate + 3σ of the predicted threat position, because the drone's own error shifts every contact.
- **Final intruder check:** clearance + 2·(own σ + the intruder's σ); heartbeats carry `pos_sigma`.

**Evaluation.** Drones report their estimate and covariance on `drone/{id}/loc`. The metrics node compares them with `sim/truth` and reports, through `/api/summary` and the sweeps:
- position error: `loc_err_mean_m`, `loc_err_p95_m`, `loc_err_max_m`;
- consistency: `loc_nees_mean` (2 for a consistent filter) and `loc_within95` (share of errors inside the 95 % ellipse);
- `loc_relocks`;
- true separations: `collisions` (< 2.5 m), `close_calls` (< 5 m) and `min_separation_m`;
- `min_ship_range_m`: the closest any drone came to the ship.

Measured, 8 drones, 4 level-1 threats, `--rtf 3` (wind 5 m/s with 1.5 m/s gusts, real; the jammer blocks the anchors for drones within 30 m of (40, 0) from t = 30 s):

| Localization, perception | Conditions | Destroyed | Miss mean / max | Position error mean / p95 / max | NEES | Within 95 % |
|---|---|---|---|---|---|---|
| truth, lidar | calm | 4/4 | 0.28 / 0.50 m | – | – | – |
| anchors, radar | calm | 4/4 | 0.35 / 0.47 m | 0.03 / 0.08 / 0.43 m | 3.1 | 92 % |
| coop, radar | calm | 4/4 | 0.34 / 0.51 m | 0.03 / 0.07 / 0.43 m | 3.2 | 90 % |
| truth, radar | wind | 4/4 | 2.58 / 4.24 m | – | – | – |
| anchors, radar | wind | 4/4 | 2.88 / 3.85 m | 0.17 / 0.38 / 0.74 m | 3.1 | 86 % |
| anchors, radar | wind + jammer | 4/4 | 2.92 / 4.60 m | 1.55 / **7.4 / 11.3 m** | 2.3 | 90 % |
| coop, radar | wind + jammer | 4/4 | 2.76 / 3.55 m | **0.16 / 0.36 / 0.76 m** | 2.5 | 90 % |

Perception alone, truth localization (`scaling_sweep.py`, sampled over the run):

| | 8 drones lidar | 8 drones radar | 50 drones lidar | 50 drones radar |
|---|---|---|---|---|
| Destroyed | 4/4 | 4/4 | 25/25 | 25/25 |
| CPU per drone | 6.6 % | **4.6 %** | 4.8 % | **3.6 %** |
| Host CPU | 0.78 cores | 0.66 | 2.18 | **1.75** |
| Control-loop p99 | 12.6 ms | **7.6 ms** | 41 ms | 38 ms |
| Collisions / close calls / closest | 0 / 0 / – | 0 / 0 / – | 0 / 3 / 3.55 m | 0 / 8 / 3.14 m |

- The radar saves 25–30 % of drone CPU. The saving is smaller at 50 drones because most of them are idle, and idle drones already skipped lidar processing.
- Close calls are mostly job-mates stacked 3 m apart on purpose.
- Anchors alone hold position error to centimetres in calm air and to sub-metre in wind.
- Cooperation keeps jammed drones within 0.76 m, where anchors alone let them drift by up to 11 m.
- Misses in wind are a station-keeping limit, the same with truth: drones latch up to 3 m from their slot, downwind, and the fuze cannot make up a cross-track offset.

The full architecture, now the default (`LOCALIZATION=coop PERCEPTION=radar NO_FLY_RADIUS_M=50`), 4 level-1 threats, `--rtf 3`:

| Drones, conditions | Destroyed | Miss mean | Intercept range mean | Closest to the ship | Position error p95 | NEES |
|---|---|---|---|---|---|---|
| 8, legacy (truth, lidar, no zone) | 4/4 | 0.27 m | 92 m | – | – | – |
| 8, calm | 4/4 | 1.15–1.22 m | 104 m | 58.7 m | 0.46–0.53 m | 1.6 |
| 8, wind | 4/4 | 2.46 m | 103 m | – | 0.57 m | 3.0 |
| 4 (intercepts on the far side: routes around the zone), calm | 4/4 | 2.47 m | 74 m | **54.9 m** | 0.53 m | 1.5 |

None of these runs had collisions, friendly fire or holds. No drone entered the zone.

At 50 drones (25 threats, 1×, `scaling_sweep.py`), after two fixes this comparison found:
- goals latch only within 0.5 m of their altitude, with vertical braking separate from horizontal. Stacked job-mates had settled 1.5 m apart within the 3 m stop radius;
- route legs advance once their goal latches. A drone had held 2.9 m from a detour waypoint, outside the 1.5 m leg threshold, and missed its slot by 8.7 m.

| | Legacy (truth, lidar, no zone) | Full architecture (now the default) |
|---|---|---|
| Destroyed | 23/25 | 24/25 |
| Intercept range mean | 86 m | 100 m |
| Closest to the ship | 28 m | 55 m |
| Proximity < 2.5 m / closest pair | 0 / 4.7 m | 3 / 2.2 m (two stacked job-mates, 3 m apart by design) |
| Position error p95, NEES | – | 0.42 m, 1.5 |
| Drone CPU, host CPU | 4.7 %, 2.16 cores | 4.2 %, 1.96 cores |

- **Legacy, T19 and T23:** the losses are chain fire, not localization. In a three-drone stack, one chain-fired mate missed by 8.4–9.5 m: the mates are spread along the track, and chain fire sets them all off when the first one's fuze fires.
- **Full architecture, T22:** the loss was a late three-drone detection that was never feasible with stations and intercepts outside the zone. Misses are larger than with the legacy setup. Stations are farther out, so drones fly longer and arrive with their position known to within about 0.5 m rather than exactly, and the fuze cannot remove a cross-track offset.

## Proximity fuze

Timed detonation depends on every drone's clock and on the drone sitting exactly on the threat's path. The proximity fuze (`src/agent/fuze.py`, pure and unit-tested, driven by `agent.py`'s `_on_contacts`) detonates when the threat actually passes. It is on by default; `--fuze off` (`FUZE=0`) restores timed detonation.

- **Sensor.** By default the mmWave radar (`drone/{id}/radar`): unlabelled 3D positions, relative to the drone, of every object within 30 m (threats, other drones, the nearest point of the ship's hull), at 20 Hz with 5 cm range and 2° angle noise (see [Localization and perception](#localization-and-perception)). The drone's own position uncertainty shifts every contact, so the gate below is widened by 3σ. With the legacy `--perception lidar`, a dedicated fuze sensor (`drone/{id}/fuze`) is used instead: `FUZE_RANGE_M` (10 m), Gaussian noise `FUZE_NOISE_M` (0.1 m per axis), `FUZE_HZ` (50), latency `FUZE_LATENCY_S` (0). What each contact really was is known only to the evaluation.
- **Tracking.** Contacts are associated from scan to scan (nearest neighbour in world coordinates).
- **Job-mates.** Objects already in range are recorded as known: the spatial queue keeps non-job drones more than 12 m away, so these are job-mates (or the ship). The record is taken at the first scan at which the job's track puts the threat within range + 2 m of the drone itself, or at arming if that is earlier. At the simulation's threat speeds (2.5–4.5 m/s), the threat is already 5–9 m away when the window opens, inside the 10 m range, and would otherwise be taken for a job-mate. A first version timed the record on the threat's distance to the engagement point; after a manoeuvre a drone had stopped 3 m short of the new point on the threat's side, took the threat for a mate, and held.
- **Arming.** The fuze fires only during `t_engage ± FUZE_WINDOW_S` (2 s), in the drone's synchronized time.
- **Trigger.** A *new* track that lies within `FUZE_GATE_M` (5 m) of the threat position predicted from the job's track (job updates, so a manoeuvre updates it) fires the fuze:
  - `FUZE_FIRE=cpa` (default): at its closest approach, if that is within the kill radius (8 m). The closest approach is when the track starts moving away, its velocity fitted to its positions over the last 0.3 s. Range alone grows only quadratically there; a range threshold fired 0.2–0.3 s late.
  - `FUZE_FIRE=radius`: as soon as it is within the kill radius.
  - Closing speed (Doppler) is not used to tell threats from drones: at the simulation's scaled speeds they overlap.
- **Fallback** when the window closes with no detection (`FUZE_FALLBACK`): `hold` (default) does not detonate; the drone reports `no_detection`, holds position and becomes free. `timed` detonates at the window's close. If no scans arrive at all (sensor off), the same fallback applies half a second after the window.
- **Chain fire** (`_on_blast`). A detonation from the same job fires this drone at once if its fuze is armed and it is within 8 m of its slot; otherwise it does not fire, and survives. The detonation topic stands in for a job-selective trigger, and its delivery delay is logged (`chain_delay_s`), not hidden.
- **Evaluation.** Each `detonation_eval` also records the reason (`fuze`, `chain`, `timed`, `fallback_timed`), the chain delay and who fired first, and, for fuze fires, how far the contact that fired it was from the true threat (`trigger_to_threat_m`, `trigger_is_threat` within 1 m). `/api/summary` adds `det_reasons`, `chain_fires`, `chain_delay_s_*`, `fuze_false_triggers` and `fuze_no_detection`.

Measured with the legacy setup and the 10 m fuze sensor (`--rtf 3`, one run each; timing error is the geometric one, see [Distributed clock](#distributed-clock)). With the current defaults, 8 drones and 4 uav: 4/4 by the fuze, misses 1.2 / 2.4 m (mean / max), timing error +0.06 ± 0.09 s:

| Scenario | Fuze | Destroyed | What fired | Timing error (mean ± sd) | Miss mean / max | Other |
|---|---|---|---|---|---|---|
| 8 drones, 4 uav (level 1) | off | 4/4 | 4 timed | −1.13 ± 0.007 s | 2.84 / 2.86 m | |
| same | **on** | 4/4 | 4 fuze | **+0.001 ± 0.088 s** | **0.52 / 1.35 m** | no false triggers, no holds |
| 10 drones, 4 two-drone missiles | off | 4/4 | 8 timed | −0.41 ± 0.49 s | 3.96 / 4.42 m | |
| same | **on** | 4/4 | 4 fuze + 4 chain | −0.37 ± 0.47 s | 3.80 / 6.09 m | chain delay 1.5–32 ms (mean 14 ms); fuze triggers 0.07–0.25 m from the true threat |
| 8 drones, ±1.5 s clock offsets, no sync | off | 4/4 | 4 timed | −1.40 ± 0.53 s | 3.52 / 5.28 m | |
| same | **on** | 3/4 | 3 fuze | +0.05 ± 0.05 s | **0.25 / 0.38 m** | 1 held (no detection) |

- **The fuze removes the constant 2.8 m miss.** Drones stop about 2.8 m short of the point, so a timed blast comes 1.1 s early. The fuze fires when the threat passes, whatever the timing.
- **Two-drone jobs**: the fuze-fired drone's miss equals its best possible (3.2–4.2 m, set by the ±3 m vertical stack). The chain-fired mate goes off with it, not at its own closest approach: one fired 1.5 s before its own and missed by 6.1 m, still within the kill radius.
- **Clocks.** When the fuze fires, its aim no longer depends on the clock (0.25 m misses with ±1.5 s offsets and no sync). But its *window* does: one drone whose clock ran 1 s ahead closed its window at `t_engage` + 1.0 s, just before the threat arrived at + 1.1 s, and held. Because threats arrive about 1.1 s late, the effective margin on the late side is about W − 1.1 = 0.9 s of clock error, not 2 s. With any sync mode this does not arise; centring the window on the predicted arrival would widen the margin (not implemented).

## Distributed clock

Engagement times are absolute (the fuze's window, and detonation itself with the fuze off), so every drone must agree with the ship on what time it is. The simulation keeps three notions of time strictly apart:

- **Truth**: simulator time (the sensor frames' `sim_time`, the ship's `sim/clock`). Only physics (control-loop dt, sensor freshness), the ship's radar world (threat motion, manoeuvres, leaks) and evaluation logging read it.
- **Local clock**: every node has its own oscillator (`src/common/localclock.py`), `local = (1 + ρ)·truth + θ`. The drift ρ (ppm) and offset θ are fixed values plus an optional uniform spread per node, drawn reproducibly from `CLOCK_SEED` and the node ID. Timestamp jitter applies only to sync exchanges, never to protocol decisions. The ship has its own oscillator too (`SHIP_CLOCK_*`, perfect by default).
- **Synchronized time**: the node's estimate of **ship time** (`src/common/timesync.py`, mode `CLOCK_SYNC`). All protocol-level absolute times go through it: the ship's `t_engage`, zones and track times are in its own timebase, and a drone compares them with its synchronized time (`_proto_now` in `agent.py`).

The ship's radar measures in ship time: a track's `t0` is the ship's clock reading, and its speed is scaled by 1/(1+ρ_ship). The positions are exact, only the timebase differs. The threat itself moves in truth, so the Gazebo markers (`sim/threat_tracks`) get the truth track.

Durations (heartbeats, timeouts, bid window, ACK timeout, re-plan interval) stay on `simclock`, which follows the simulator's rate; over a few seconds, drift of even 500 ppm is under a few milliseconds.

Protocol decisions read the clock at the latest sensor frame (50 Hz), as before, so with the defaults (no drift, no offset, `CLOCK_SYNC=none`) behaviour is exactly the same as with the shared simulation clock.

| Sync mode (`CLOCK_SYNC`) | How | Radio cost |
|---|---|---|
| `none` (default) | each drone trusts its local clock as ship time | none |
| `ttg` (time-to-go) | the ship stamps every message that carries absolute times with its send time (`sent`); the drone anchors each time on receipt, `local_rx + (t − sent)`, and keeps its protocol times on its local clock. Every job update (2 Hz) and zone update (1 Hz) re-anchors. The unknown one-way delay becomes error. | one field per message |
| `master` (ship master) | two-way exchange, NTP-style: the drone's heartbeat carries its send stamp t1; the ship stamps its receipt t2, and its 1 Hz roster echoes `[t1, t2, t3]` for every drone, t3 being the roster's send stamp; the drone stamps the roster's arrival t4. offset = ((t2 − t1) + (t3 − t4)) / 2. A Kalman filter on offset and rate weighs each sample by its round trip (an exchange is off by at most half of it), rejects outliers once settled, and the rate estimate carries the clock through gaps. | no new messages: ~12 B per heartbeat, ~45 B per drone in each roster |
| `consensus` (ship-anchored, ATS-style) | every node keeps a virtual clock `v = α·local + o` and moves its rate α and its virtual time toward the aggregate of its neighbours'; the ship is a pinned leader that never adjusts, so drones connected to it, directly or through peers, converge to ship time, and without it they keep agreeing with each other. Details below. | drone beacons at `CLOCK_BEACON_HZ` (0.5), ~130 B each, received by every drone: O(N²) swarm-wide (measured below); the ship's beacon rides in its roster |

**Relayed drones (`master`).** The ship hears a drone that needs help through a peer, which forwards its heartbeat once a second, so the heartbeat can sit up to 1 s at the relay. The relay adds how long it held it (`relay_resid`, a PTP-style transparent clock) and the ship subtracts that from t2. The roster still reaches the drone directly. A drone that hears no roster keeps its last offset and rate.

### Ship-anchored consensus (`CLOCK_SYNC=consensus`)

`timesync.Consensus`, driven by `agent.py` (`_send_clock_beacon`, `_on_clock_beacon`, `_on_roster`):

- **Beacons.** Each drone beacons on `swarm/clock/{id}` every 1/`CLOCK_BEACON_HZ` s: `{tau, alpha, o, anchor, echo}`, where `tau` is its local send stamp. The ship's beacon is its roster: `beacon` (its send stamp) and the per-drone `sync` echoes of `master` mode.
- **Offsets from two-way exchanges.** Every beacon echoes the last beacon heard from one neighbour, rotating through them: `[neighbour, its send stamp, our receive stamp]`. The neighbour's next beacon completes a four-timestamp exchange, whose offset `((t2 − t1) + (t3 − t4)) / 2` needs no delay estimate: it is exact for symmetric delays, and an asymmetry costs half of it. The ship link uses the heartbeat/roster exchange every second. Until a neighbour has an exchange it is used one-way, and only if nothing better is fresh.
- **Rates, ATS-style.** A neighbour's hardware rate relative to ours is a robust line fit of its raw stamps against ours: outliers beyond 3× the median residual are dropped, and the fit is used only if its standard error is under 100 ppm. Raw oscillators are linear, so the fit stays clean while virtual clocks move.
- **Update.** Before each beacon, a drone moves α and its virtual time `CLOCK_GAIN` (0.5) of the way toward `target = share·ship + (1 − share)·aggregate(own, neighbours)` while it hears the ship (`CLOCK_LEADER_SHARE`, 0.5), and toward `aggregate(own, neighbours)` otherwise. Plain averaging with the ship as one neighbour among N would take about N/gain updates to move the swarm's common offset.
- **Pluggable aggregation.** `aggregate(own, others)` comes from `timesync.AGGREGATORS` (`CLOCK_AGGREGATE`, only `mean` so far). It gets its own value separately so a trimmed mean or MSR (drop the f most extreme neighbours relative to one's own value) can be swapped in for the security work.
- **Anchors and error bound.** Every node carries an anchor: the ship time at which its chain of sources last touched the ship, and the hop count. A newer anchor wins, and for the same anchor fewer hops. The self-reported bound is `hops × 5 ms + 50 ppm × (time since the anchor) + the largest disagreement with the sources used`. Without the ship it grows with time only; a bound built from neighbours' bounds counted to infinity around loops (1.1 s after 5 min).
- **Startup and late joiners.** While any fresh source is anchored, only anchored sources count, so drones that never heard of the ship cannot drag synced ones. More than `CLOCK_STEP_S` (50 ms) from an anchored target, the clock steps onto it (the ship's estimate if heard); smaller corrections slew by the gain, as NTP does.

Every drone reports its hop count and whether it hears the ship in telemetry (`clock.hops`, `clock.ship`).

What the live runs and `test_consensus.py` found, and what was fixed:
- **One-way delay** made every drone lag about 2 delays behind the ship. With the ship lost, the whole swarm walked backwards by gain × delay per beacon, about 1.25 ms/s. Per-link delay estimates reduced but did not remove it: the minimum round trip under-estimates the mean, and peer links are measured rarely, so after a congested start stale estimates held drones about 100 ms off. The fix is the two-way offsets.
- **Rates from a congested start** were 1% off: long, uneven round trips, and a delay correction changing across the fit window, tilted the slope. With the ship lost, the rate average then ran away (0.8 s in 90 s). The fix is fitting on raw stamps, with outlier rejection and the standard-error check.
- **Slow start**: slewing at 25% per beacon took 60–70 s to bring ±3 s offsets under 10 ms. The fix is stepping.

**Exchange timestamps** come from `simclock.truth_now()` (sim time extrapolated between frames) through the node's clock with its jitter: sensor frames are 20 ms apart, too coarse for an exchange.

Measured (8 drones, 4 level-1 threats, `--rtf 3`; skewed = ±500 ppm drift and ±3 s offset per drone, `CLOCK_SEED=1`; sync error is the true error of each drone's telemetry report after its first 20 s):

| Clocks, sync | Destroyed | Detonation error vs ordered time (max \|·\|) | Miss (mean, max) | Sync error (mean, max per drone) | Own bound covers true error |
|---|---|---|---|---|---|
| perfect, `none` | 4/4 | 18 ms | 2.83 m, 2.86 m | – | – |
| skewed, `none` | 4/4 | 1.96 s | 4.23 m, 7.74 m | = clock error (0.2–3 s) | – |
| skewed, `ttg` | 4/4 | 25 ms | 2.82 m, 2.84 m | 2.7–3.2 ms, ≤ 18 ms | no bound |
| skewed, `master` | 4/4 | 17 ms | 2.82 m, 2.85 m | 0.5–1.4 ms, ≤ 4.8 ms | 82% |
| perfect, `master` | 4/4 | 22 ms | 2.82 m | 0.6–1.3 ms, ≤ 2.9 ms | 83% |
| skewed, `consensus` | 4/4 | 24 ms | 2.82 m, 2.85 m | 0.6–2.7 ms, ≤ 5.0 ms | 99.8% |
| skewed, `consensus`, ship radio jammed for 90 s | 2/4 (the jam blocked the operator and the ship's orders) | 18 ms | 2.82 m, 2.83 m | during the jam 0.5–2.8 ms, ≤ 5.9 ms | 99.7% (100% during the jam) |

- All three modes remove the clock error from detonation timing. What remains is the 20 ms sensor-frame step: decisions run on frames.
- `consensus` reached ~4 ms within 10 s of the first reports and 1.4 ms within 20 s (steps onto the ship's estimate, then slews). With the ship's radio jammed (100% loss) for 90 s of sim time, drones kept agreeing with each other within 1–4 ms and with ship time within 0.5–2.8 ms on average; the bound grew from ~8 to ~13 ms and covered every report; on restoration they re-anchored within 10 s. The evaluation stream (`sim/clock_eval`, onboard) kept measuring through the jam.
- **Radio cost of `consensus`** (`scaling_sweep.py`, `--rtf 3`, sampled over the run; all threats destroyed, no friendly fire):

  | Drones | Msgs/s received per drone, `none` → `consensus` | Bytes/s sent per drone | Drone CPU |
  |---|---|---|---|
  | 8 | 2.3 → 5.3 | 851 → 1117 | 9.1% → 9.1% |
  | 16 | 2.2 → 8.5 | 923 → 1141 | 9.2% → 9.3% |

  Each drone sends 0.5 beacons/s (about 130–260 B/s with the echo) and receives 0.5 × (N − 1). At 50 drones that is about 24.5 more messages per second per drone, about 5× today's load. This is the O(N²) pattern that made heartbeats ship-only. Lower `CLOCK_BEACON_HZ` trades it against convergence and holdover.

- Both `ttg` and `master` remove the clock error from detonation timing. What remains is the 20 ms sensor-frame step: decisions run on frames.
- `ttg` is off by the one-way delay of each message, a few ms at this radio load. Every job update re-anchors, so drift never accumulates while a job is live.
- `master` gives the same result with ±500 ppm and ±3 s as with perfect clocks. The ~1 ms left (median −1.0 ms, 5–95% −2.0 to −0.2 ms) is systematic: path asymmetry between heartbeat and roster, plus the simulation's own floor (each process extrapolates truth from onboard messages that arrive a few ms late at `--rtf 3`). That bias is why the drones' own bound (median about 2.6 ms) covers only 82% of reports.
- **Startup.** The first exchanges come in a congested start, when round trips are long. A version that weighed samples only against the recent minimum round trip trusted lopsided ones and lost the rate for a minute (errors to 129 ms). Samples are now weighed by their absolute round trip and gated once the filter has settled; `test_timesync.py` covers this case.
- **Radio cost of `master`**: the same number of messages. Heartbeat bytes at the ship rose from 1445 to 1636 B/s for 8 drones, about 24 B/s per drone, and each roster grows by about 45 B per drone. With a sync mode, telemetry carries a few tens of bytes more per drone per second (`clock`).

### Stress test: 50 and 100 drones

Setup: `scaling_sweep.py` at 1× (N drones, N/2 threats), skewed = ±500 ppm and ±3 s per drone. Results are in `results/stress_*` (git-ignored); sync error is each drone's true error after its first 20 s.

| Drones, sync | Destroyed | Error vs ordered time (mean) | Miss mean / max | Sync error p50 / p95 / max | Bound covers | Sim speed | Host CPU | Msgs/s received per drone |
|---|---|---|---|---|---|---|---|---|
| 50, `none` (perfect clocks) | 25/25 | 12 ms | 3.71 / 6.89 m | – | – | 0.98× | 2.4 cores | 3.8 |
| 50, `ttg` | 25/25 | 14 ms | 3.65 / 5.42 m | 4.9 / 14.5 / 236 ms | – | 0.95× | 2.2 | 3.6 |
| 50, `master` (before fix) | 25/25 | 35 ms | 3.66 / 6.96 m | 2.6 / 230 / 425 ms | 27% | 0.99× | 2.0 | 4.1 |
| 50, `master` (fixed) | 25/25 | 11 ms | 3.69 m | 2.2 / 4.5 / 27 ms | 36% | 0.97× | 2.5 | 4.2 |
| 50, `consensus` (fixed) | 25/25 | 14 ms | 3.74 m | 2.2 / 6.2 / 22 ms | 99% | 0.98× | 2.5 | 24.5 |
| 100, `none` (perfect clocks) | 50/50 | 17 ms | 3.81 / 7.65 m | – | – | 0.65× | 4.3 | 8.5 |
| 100, `consensus` (before fix) | 50/50 | 23 ms | 3.87 / 6.07 m | 7.8 / 23.5 / 192 ms | 91% | 0.55× | 4.6 | 45.5 |
| 100, `consensus` (fixed) | 31 of 41 detected (run cut short, see below) | 29 ms | 3.77 m | 5.6 / 20.2 / 51 ms | 89% | 0.25× | 5.3 | 50.7 |

No run had friendly fire, leaks or missed slots.

- **The bug the stress test found.** At 50 drones, each process's timestamps (truth extrapolated from onboard messages that arrive tens of ms late) occasionally gave an exchange an impossible, negative round trip. Both filters clamped it to 0, so it looked like the best exchange of all.
  - In `master` a single one threw the rate 5,000 ppm off, and the outlier gate then rejected every good sample for the rest of the run: 5 of 50 drones drifted to 425–720 ms (125 of 133 samples rejected). The clamped zeros also made the bound too tight.
  - Fixes: exchanges with a round trip below −5 ms are discarded (about 5% of them at 50 drones); after 5 gate rejections in a row the `master` filter re-acquires; consensus uses, per source, the exchange with the lowest round trip of the last 10 s (NTP's clock filter) instead of the newest.
  - `test_timesync.py` and `test_consensus.py` reproduce all three cases, and they fail without the fix.
- **The bound in `master`** still covers only about a third of reports at 50 drones. The filter's statistics give 1–2 ms, but the simulation's timestamps carry 2–4 ms of error at this load that no exchange can see. An explicit timestamp-uncertainty term, as a real node would take from its datasheet, is the proposed fix; it is not implemented yet.
- **100 drones is the limit of this host** (6 cores). The same configuration ran at 0.55× once and 0.25× the next time (load average 63). The protocol clock follows the simulator, so results stay valid, but the fixed 100-drone consensus run ran out of wall time at sim t = 136 s with 10 threats still in flight. It lost none: 31 of the 39 approved were destroyed, and nothing failed or leaked.
- **Consensus radio load** reached 45–51 messages per second per drone at 100 drones, 6× the load without it. Each link is also measured only about every 2·(N − 1) s (one echo per beacon, rotating), so at 100 drones a peer's two-way offset can be minutes old. Median error rose from 2.2 ms at 50 drones to 5.6–7.8 ms at 100. Beaconing to a subset of neighbours would bound both; it is not implemented.

**What each drone reports.** With a sync mode or an imperfect clock, radio telemetry carries what a real drone could report: `clock{mode, offset, bound}` (the estimated offset ship − local and the drone's own error bound; with `consensus` also `hops` and `ship`). For evaluation only, the drone also publishes a (truth, estimate) pair taken at the same instant on the onboard bus (`sim/clock_eval`, 1 Hz), so the error is measured during radio cuts too. The ship turns that pair into the true error (the drone cannot know it), shows offset, bound and true error in the dashboard's instrumentation table, logs a `clock_eval` record per report, and adds `sync_err_s_*`, `sync_bound_s_mean` and `sync_bound_coverage` (how often the true error was within the bound) to `/api/summary`.

**Kill assessment uses truth.** The radar observes a blast physically, whatever any clock says: a detonation carries the drone's sensor-frame sim time (`truth_time`) along with its local and synchronized stamps.

**Evaluation.** For every detonation the ship logs a `detonation_eval` record to `/state/ship_log.jsonl`:
- truth time, the drone's local and synchronized time, and the ordered `t_engage`;
- `ideal_t`: when the threat truly passes closest to the detonation point, and `timing_err_s = truth − ideal_t`. This geometric error is the primary figure: it stays meaningful at any threat speed, whatever the ordered time was;
- `ordered_err_s`: the error against the ordered `t_engage` (mapped to truth through the ship's clock);
- `miss_m` (distance to the threat at detonation), `ideal_miss_m` (the best possible from that point), `chain`, and the threat's speed.

`/api/summary` aggregates them (`det_timing_err_s_*`, `det_ordered_err_s_*`, `det_miss_m_*`), and the sweeps report them. Sweeps take any swarm environment with `--env KEY=VALUE`.

Measured (8 drones, 4 level-1 threats, `--rtf 3`, one run each):

| Clocks | Destroyed | Error vs ordered time (mean, max \|·\|) | Geometric timing error (mean ± sd) | Miss (mean, max) |
|---|---|---|---|---|
| perfect (default) | 4/4 | +0.007 s, 0.017 s | −1.13 ± 0.02 s | 2.85 m, 2.89 m |
| ±500 ppm, ±3 s offset, `CLOCK_SYNC=none` | 4/4 | −0.55 s, 1.96 s | −1.69 ± 1.06 s | 4.23 m, 7.74 m |

- With perfect clocks, the error against the ordered time is only the 20 ms sensor-frame step. The outcome matches the shared-clock runs (2.84 m misses).
- With skewed clocks, each drone's error is exactly minus its own clock error at that moment (offset plus drift × elapsed time). The worst miss, 7.74 m, came from a clock 1.97 s ahead, just inside the 8 m kill radius.
- **The −1.13 s geometric error with perfect clocks is not a clock error.** Drones latch on arrival about 2.8 m short of the engagement point, on the side they came from. The threat reaches that spot 1.1 s after `t_engage` (2.8 m at 2.5 m/s), so a timed detonation there misses by 2.8 m, although 0.2–0.3 m was possible from the same spot. This is where the constant 2.84 m misses come from.

### Membership and link loss
- Each drone heartbeats at 2 Hz, to the ship only: drones do not subscribe to each other's heartbeats, so radio load grows with the number of drones, not its square. The ship publishes the roster (drones heard in the last 3 s) at 1 Hz. The dashboard's drone count and free count come from it.
- A drone considers its **radio link up** while it hears the ship or any peer (orders, bids, awards, help) within 3 s.
- **Link lost, no job:** the drone holds position and does not bid.
- **Link lost, confirmed job:** the drone continues to the last engagement point and time the job topic gave it, and detonates then. It gets no more target updates, so a manoeuvre after the loss makes it miss. The detonation is observed through the onboard bus, standing in for the ship's radar.
- **Link lost before confirmation:** no ACK arrives, so the drone abandons the job after 3 s.
- **Link lost at auction close:** the drone ignores the result, because it was computed from whatever bids reached it.
- **The ship does not hear a drone directly** (the drone is missing from, or listed as relayed in, 3 rosters, or has heard no roster for 3 s): the drone also publishes its heartbeat on `swarm/heartbeat_help/{id}`, which every drone hears. Drones in the roster forward it on `swarm/heartbeat_relay/{id}` once a second. The ship accepts it as membership, and the dashboard marks the drone *relayed*. Relaying continues while the roster lists the drone as relayed, so it does not stop as soon as it works. Zenoh 1.10 peers do not relay for each other, so this is done at the application level.
  - Measured (8 drones, ship deaf to one drone): out of the roster after ~3 s, back in (relayed) by 6 s, relayed without a gap for the next 20 s, and back to direct when the link was restored.

## Degraded communications

Radio loss is emulated two ways: 100% packet loss on the radio interface (`tc netem` through Pumba, which is closest to jamming because the interface stays up), and removing the interface (`docker network disconnect`). Both gave the same results. The tools are in `tools/comms/` (see below).

| Finding | Evidence |
|---|---|
| **Over TCP radio links**, jamming one peer stalls other peers' traffic for ~10 s: a random subset of healthy peers hear nobody | radio probe, 9 peers, repeated runs: 4–7 of 8 healthy peers silent for 9.8–10.4 s. Not changed by the connect/accept timeout, the interest timeout, the lease, or more send threads. (Earlier single clean runs with a 1 s connect timeout were luck, not a fix.) |
| Over TCP, the jammed node also freezes **all** its Zenoh sessions once for ~10 s, including the onboard link | onboard probe: 10.1–10.4 s gap in the jammed drone's 50 Hz onboard stream (Zenoh 1.0.4 and 1.10.1) |
| **Fix: radio over UDP** (`RADIO_PROTO=udp`, default). Both effects disappear. Mixing TCP and UDP brings the stall back. | radio probe, 9 peers, 2 runs: all healthy peers worst gap ≤ 0.06 s; onboard probe with the radio in-process: jammed drone worst gap 25 ms; `tcp,udp`: 7 of 8 peers silent 9.8 s |
| Defence in depth: the radio runs in its own OS process (`RadioProcess`), so no radio problem can reach flight control or C2 | onboard probe over TCP: jammed drone's worst onboard gap 25 ms (was 10.4 s) |
| Full system before the fixes: the ship froze ~10 s when an engaged drone was cut; cut drones missed their slots | runs 3/4: ship clock stopped 10 s; cut drone 31.7 m / 5.0 m off its slot at detonation time, so it aborted |
| **Full system, UDP radio:** jamming an idle drone and then an engaged one affects only those two. No healthy drone lost its link; the ship's roster tracked the jams exactly (8 → 7 → 6). The jammed engaged drone destroyed its threat on time. | run 6, sampled every second (81 samples): 0 samples with a healthy drone down; detonation at t = 76.29 s (allocated 76.3 s), 2.84 m from the threat |
| **Lost control messages over UDP.** Zenoh sends each control message once. When a node first names a key expression to a peer it declares a numeric ID for it, and later messages use the ID. If that declaration is lost, every later message naming the key is undecodable at that peer (its log: `Unknown wire expr`), so a subscription made under loss never hears that peer, and re-declaring the same key, or a wider one built on it, cannot repair it. | `declare_probe.sh`, 2 peers, 30 % loss on the subscriber's side: 11–15 of 40 new subscriptions never heard anything; re-declaring every 0.5 s (with or without a pause) or adding `K/**`, `K$*` forms later: no change; Zenoh trace on the publisher shows `Unknown wire expr` for exactly the dead keys |
| **Fix: QUIC control plane** (`RADIO_PROTO=quic,udp`, default). Each pair of nodes also opens a QUIC link (TLS, simulation-only certificate in `src/common/radio_tls/`; listen port + 1). Zenoh sends reliable messages, its control messages among them, over QUIC, and best-effort ones over UDP; every publication is marked best-effort, so data stays on UDP and is never retransmitted late. QUIC does not bring back the TCP stall. | `mesh_probe.sh`, 8 peers, 30 % loss everywhere: new subscriptions dead after 3 s 33 % → 0.1 % (5 min: 0.3 %), heartbeat delivery unchanged (0.69: data still best-effort); jamming probe, 9 peers, `quic,udp`, 2 runs: every healthy peer's worst gap 0.05 s. At 50 % loss QUIC helps less (dead 58 % → 45 %). |
| **Sessions closed by the lease.** With a 2 s lease, keep-alives (every 0.5 s) lost in a row closed sessions, and a reopened session declares everything again under loss. | `mesh_probe.sh`, 8 peers: 30 % loss 9 flaps in 60 s; 50 % loss 41–54 flaps, a third of the sessions down at the end, 25–40 of 56 peer pairs deaf for good. **6 s lease** (`RADIO_LEASE_MS`, now the default): 0 flaps at 30 % and 50 %. |
| Trade-off: data is best-effort, so orders, bids and awards can be lost under packet loss. Orders and bids are sent 3 times, awards 3 times, and recovery relies on heartbeat answers, re-announcement and conflict repair, measured below. | [Radio degradation sweep](#radio-degradation-sweep) |

### Radio degradation sweep

Setup: `degradation_sweep.py --rtf 3 --repeats 3`. Each run uses 8 drones and the same seeded scenario: 4 level-1 threats, one every ~30 s, approved by the stand-in operator. The impairment is applied to the radio of every drone and the ship. Times are simulated milliseconds, and each cell is the mean of 3 runs.

| Condition | Destroyed (of 4) | Drones used | Decision latency | Re-announces | Agreement |
|---|---|---|---|---|---|
| none | 4 | 4 | 1030 ms | 0 | 1.00 |
| 10% loss | 4 | 4 | 1050 ms | 0 | 0.87–0.97 |
| 30% loss | 4 | 4 (default) / 4.3 (tuned) | 1080 ms | 0 | 0.78–0.82 |
| 200 ± 50 ms delay | 4 | 4 | 1440 ms | 0 | 0.96–1.00 |
| 64 kbit/s | 4 | 4 | 1360 ms | 0 | 1.00 |
| **24 kbit/s** | **2.7–3** | 2.7–3 | **17,500–19,200 ms** | 5–6 | 0.71 |

The loss rows are from the run with the award fixes below. The other rows come from the full sweep, which ran before the last of those fixes; that fix affects only conflicts between drones, which only the loss rows showed. All rows predate ship confirmation (see [Engagement protocol](#engagement-protocol)), which adds about 0.35 s to decision latency.

- **Wasted drones.** Before these fixes, one run per cell spent up to 7 drones on 4 threats at 30% loss, with 3 re-announces. Three fixes brought it to 4, with no re-announces:
  - drones send each award 3 times;
  - before re-announcing, the ship also counts drones whose heartbeats say they are engaged;
  - a drone that has already heard a better award for a threat does not take it when its own auction closes.
- **Remaining case, now fixed.** Once in 6 runs at 30% loss, two drones engaged the same threat. The worse drone never heard the better one's bid or any of its 3 award copies, although the ship heard both. That suggests the direct link between those two drones was down, which conflict repair between drones cannot fix. Ship confirmation fixes it: in 6 more runs at 30% loss, the ship settled 5 such conflicts and every run used exactly 4 drones.
- **Bandwidth.** 64 kbit/s is fine. At 24 kbit/s, routine traffic alone overfills the link:
  - each drone sends heartbeats (to every peer and the ship) and telemetry, and those outgoing bytes are more than 24 kbit/s;
  - the backlog builds in the kernel's first-in-first-out queue, not in Zenoh's, so Zenoh priorities (`tuned`) cannot move orders ahead;
  - decisions take ~18 s, threats are re-announced, and one threat leaks because it could not be approved in time.

  The fix is less background traffic, not QoS.
- **`tuned` vs `default`.** No meaningful difference in any condition, so `default` stays the default.
- **Impairment method.** `degrade_radio.sh` impairs only UDP. The earlier version impaired all traffic on the radio interface, including the dashboard's TCP connection (Docker forwards `:8080` to the ship's radio address). At 64 kbit/s one dashboard request then took 1.2 s instead of 1 ms, which is why the earlier tuned 64 kbit/s run failed.

### Sensing and ship-link sweep

`tools/comms/sensing_sweep.py` runs the seeded scenario under degraded sensing and a degraded link to the ship. There is no GNSS degradation, because the drones don't use GNSS: their position comes from UWB.

Every impairment makes the **environment** worse than the [hardware record](#hardware-record) says. The drones keep assuming the record: for example, their filters still expect 0.1 m UWB noise. That is how a real degradation reaches them.

```bash
python3 tools/comms/sensing_sweep.py --list                      # the conditions and their settings
docker compose build                                             # after code changes: the sweep runs existing images
python3 tools/comms/sensing_sweep.py                             # every condition once: 15 drones, 8 threats, --rtf 3
python3 tools/comms/sensing_sweep.py --conditions baseline uwb_nlos ship_outage --repeats 3
python3 tools/comms/sensing_sweep.py --custom wet="UWB_NLOS_P=0.2 RADAR_CLUTTER=1" --conditions baseline wet
python3 tools/comms/sensing_sweep.py --drones 30 --threats 15 --parallel 2 --repeats 3  # larger, 2 swarms at once
```

| Condition | Impairment |
|---|---|
| `baseline` | none |
| `uwb_noise` | UWB range noise 0.32 m (record 0.1 m): `UWB_EXTRA_SIGMA_M=0.3` |
| `uwb_nlos` | 10 % of UWB ranges non-line-of-sight, with a positive bias (exponential, mean 1 m): `UWB_NLOS_P=0.1 UWB_NLOS_BIAS_M=1` |
| `uwb_dropout` | half of all UWB exchanges lost: `UWB_DROPOUT_P=0.5` |
| `uwb_short` | UWB range 80 m (record 250 m), so far stations and intercepts depend on peers: `UWB_MAX_RANGE_M=80` |
| `uwb_jam_local` | anchors jammed within 40 m of (80, 0) from t = 60 s: `UWB_JAM=anchors:60:100000:80:0:40` |
| `uwb_jam_all` | all UWB, anchors and peers, jammed for t = 60–120 s, so everyone dead-reckons: `UWB_JAM=all:60:120` |
| `ship_loss30` | 30 % loss on the ship's link, both ways, for the whole run |
| `ship_outage` | the ship's link cut for t = 90–120 s |
| `radar_noise` | radar noise 0.3 m, 6° (record 0.05 m, 2°): `RADAR_RANGE_SIGMA_M`, `RADAR_AZ_SIGMA_DEG`, `RADAR_EL_SIGMA_DEG` |
| `radar_miss` | radar misses 30 % of contacts per scan: `RADAR_MISS_P=0.3` |
| `radar_clutter` | 2 false contacts per scan, uniform within range: `RADAR_CLUTTER=2` |
| `radar_latency` | radar scans 0.1 s late: `RADAR_LATENCY_S=0.1` |
| `combined` | UWB noise + NLOS, radar noise + clutter, 30 % loss on the ship link |

**How the ship link is impaired.** The tool shapes the ship's own UDP traffic, and the drones' UDP traffic addressed to the ship (`degrade_radio.sh` with `DST_IP`). Drone-to-drone links and the dashboard are untouched, so a timed outage is applied and lifted by watching the dashboard's sim time; it is logged in `impairments.log`. Netem loss is not scaled by `--rtf`.

**Options:** the same as `degradation_sweep.py` (`--rtf`, `--drones`, `--threats`, `--maneuver-p`, `--interval`, `--first`, `--seed`, `--repeats`, `--parallel`, `--auto-approve`, `--env`, `--out`). The defaults are 15 drones, 8 threats and `--rtf 3`.

**Results** go to `results/sensing_<time>/`:
- `results.csv` (every column of `degradation_sweep.py`);
- `results.md`: the key metrics per run, then mean ± sd per condition. These include kills, misses, detonation reasons, fuze holds and false triggers, missed slots, position error p95/max, NEES and minimum separation;
- per run: `swarm.log`, `operator.log`, `summary.json`, `state.json` and `impairments.log`.

**First results** (2026-10-03: one run per condition, 15 drones, 8 threats, `--rtf 3`, seed 42; single runs, so treat differences of one threat as noise):

| Condition | Destroyed | Miss mean / max | Position error p95 / max | NEES | What went wrong |
|---|---|---|---|---|---|
| baseline | 8/8 | 2.39 / 4.84 m | 0.42 / 1.16 m | 1.5 | – |
| `uwb_noise` | 8/8 | 2.82 / 6.63 m | **2.84 / 18.6 m** | **24.8** | the filter still assumes 0.1 m: overconfident, 86 re-locks, 1 fuze false trigger, closest pair 3.9 m |
| `uwb_nlos` | 8/8 | 2.35 / 4.98 m | 0.52 / **14.0 m** | 2.5 | the gate rejects most biased ranges; 3 re-locks, one bad excursion |
| `uwb_dropout` | 8/8 | 2.33 / 5.09 m | 0.58 / 1.85 m | 1.5 | – (consistent) |
| `uwb_short` | **5/8** | 3.02 / 5.0 m | **16.3 / 36.1 m** | **49.8** | beyond 80 m only peers localize, and the fusion is badly overconfident; 3 drones held with no detection |
| `uwb_jam_local` | 8/8 | 2.40 / 4.81 m | 0.38 / 1.43 m | 1.5 | – |
| `uwb_jam_all` | 8/8 | 2.42 / 4.85 m | 2.55 / 7.1 m | 1.1 | dead reckoning for 60 s; error grows but stays honest |
| `ship_loss30` | **4/8** | 2.31 / 3.51 m | 0.42 / 1.16 m | 1.5 | ACKs lost: confirmed drones abandoned after 3 s, re-announcements found no drone in time (3 never fully assigned, 8 re-announcements) |
| `ship_outage` | 8/8 | 2.31 / 5.15 m | 0.42 / 1.13 m | 1.5 | no decision fell in the window this run; in a smoke run of the same condition, one threat was approved too late and lost |
| `radar_noise` | 8/8 | **4.94** / 6.98 m | 0.43 / 1.14 m | 1.5 | misses doubled; 2 fuze false triggers |
| `radar_miss` | 7/7 approved | 2.65 / 5.02 m | 0.43 / 1.45 m | 1.5 | one threat never approved (the operator found it infeasible), so it leaked |
| `radar_clutter` | 8/8 | 2.40 / 5.06 m | 0.41 / 1.33 m | 1.5 | – (clutter never gated in) |
| `radar_latency` | 8/8 | 2.51 / 5.21 m | 0.41 / 1.45 m | 1.5 | – |
| `combined` | **4/8** | 3.28 / 5.29 m | 1.27 / 7.3 m | 10.4 | the ship-link losses as in `ship_loss30`, plus overconfidence from the UWB noise |

What this points at:
- **Ship-link loss was the biggest threat to the kill rate.** At 30 % loss, confirmed drones never heard their confirmation and gave up after the old 3 s timeout, and lost orders left threats without bidders. **Fixed** (2026-10-04, see [Engagement protocol](#engagement-protocol)): `ship_loss30`, `ship_outage` and `combined` now destroy 8/8.
- **The UWB filters trusted the record's noise figure.** When real noise was 3× worse, they became overconfident and re-locked repeatedly. **Fixed** by the robust filter (2026-10-04, `UWB_FILTER=robust`): `uwb_noise` NEES 24.8 → 2.0–2.1, re-locks 86 → 0–2, worst error 18.6 → 9.6–10.2 m; `uwb_nlos` worst error 14.0 → 2.7 m, NEES 2.5 → 1.6.
- **Peer-only localization beyond anchor range is not safe yet** (`uwb_short`): errors reached tens of metres while the filters claimed sub-metre accuracy. This is the open recursive decentralized localization (RDL) item in [Known limitations](#known-limitations).
- **The radar is robust** to misses, clutter and 0.1 s latency, but noisier ranging doubles the misses.

### Spatial queue

A detonation destroys any drone within 8 m (friendly fire), and the drones have no seeker. The rule: at detonation, every drone not on that job must be more than **12 m** away. `src/common/deconflict.py` is shared by the ship and the drones, and is unit-tested. It enforces the rule in space and time:

- **Reservations.** Each confirmed job reserves its blast: 12 m around each slot, at its detonation time ±1.5 s, or ± the fuze window (2 s) with the fuze on: the drone may fire anywhere in it.
- **Intercept choice (ship).** The ship picks the earliest reachable point on the track that is separated from every other job's slots. It also avoids points whose blast would force a hold on a drone already flying another job, so new jobs yield to committed ones. Only if no such point exists are holds accepted.
- **Routes with holds (drones).** A drone plans a straight route with its real speed profile, starting from its current speed. If it would be inside another job's blast during that blast's window, it holds just outside, beyond its stopping distance, until the blast has passed. Bids include hold time, and the ship re-checks each drone's route before confirming it. Drones re-plan every second and whenever the zones change.
- **Giving a job back.** A drone gives its job back only if its slot is inside another job's blast, or it really can't arrive in time. It never does so within 10 s of detonation, because it would be left inside the zone as an outsider.
- **Idle drones** inside a zone move out of it.
- **Final check.** At detonation, a drone counts non-job drones within 12 m. If there are any, it detonates anyway: the target comes first.
- **Friendly fire is modelled.** A detonation of another job within 8 m destroys a drone (`sim/damage`). A job's own blasts never hurt its drones, also after a drone has aborted or given the job back (`_last_job`; this replaced a special case for aborts). `ZONE_KEEPOUT=0` turns prevention off for experiments.

Results, 50 drones, 25 threats, real time, APF planner:

| | Before (CPA/defended-radius intercepts) | Now |
|---|---|---|
| Intercept distance from the ship | 45 m | 86 m |
| Threats destroyed | 23/25 | **25/25, 25/25** (two runs) |
| Friendly-fire kills | 3 (first zones version) | 0 |
| Detonations with non-job drones within 12 m | 5 | 0 |

With 8 drones: 4/4 destroyed at 92 m. Manoeuvring missiles (each turns once, points shift 7–27 m): 3/3 destroyed; before, the largest shift was a miss.

What it took, from traced flights:
- **Planner.** ORCA's greedy solver dead-ends at zero velocity when boxed in (e.g. by a drone parked on the route), so APF is now the default. APF's repulsion fades out over the last 10 m to the goal; otherwise neighbours near a goal push the drone away forever.
- **Vertical slots.** Drones in a horizontal ring pushed each other 6–10 m off their slots.
- **Speed-aware planning.** Routes that assumed a standing start put hold points inside the stopping distance of a cruising drone.

### Scaling

Setup: `scaling_sweep.py`. The table was measured with the legacy setup (truth, lidar, no zone); the current defaults are profiled after it. N drones face N/2 threats (default type mix), detected every 240/N sim seconds, so the load per drone stays about the same. Each size runs at the fastest speed the host could sustain (6 cores, 15 GB). Needs the [ARP limit](#quick-start) raised above ~30 drones.

| Drones | Speed (target → achieved) | Host CPU | Drone CPU per 1× | Gazebo CPU | Worst loop (p99) | Oldest sensor frame | Msgs/s received per drone | Msgs/s at ship | Outcome |
|---|---|---|---|---|---|---|---|---|---|
| 8 | 3 → 2.95× | 1.9 cores | 7.4% | 33% | 22 ms | 30 ms | 13 | 21 | 4/4 destroyed, 4 drones |
| 16 | 3 → 2.72× | 3.5 cores | 8.1% | 37% | 33 ms | 47 ms | 27 | 38 | 8/8 destroyed, 10 drones |
| 25 | 2 → 1.9× | 3.4 cores | 7.3% | 33% | 42 ms | 54 ms | 44 | 60 | 12/12 destroyed, 16 drones |
| 50 | 1 → 0.87× | 3.8 cores | 9.4% | 26% | 137 ms | 378 ms | 86 | 123 | 22/25 destroyed, 38 drones |
| 50, now | 1 → 0.98× | 1.9 cores | 3.9% | 30% | 55 ms | 110 ms | 6 | 108 | 23/25 destroyed, 39 drones |
| 50, QUIC radio (2026-10-04) | 1 → 0.97× | 2.0 cores | 4.3% | 31% | 50 ms | 171 ms | 4.3 | 106 | 24/24 approved destroyed, 38 drones; all 50 joined within 30 s. T22 (level 3) was not approved: too few free drones could reach it in time |

All sizes: no re-announces, no conflicts, full agreement on assignments, decision latency ~1.4 s. The 16-drone run predates the sim-following protocol clock. "50, now" is after the heartbeat, idle-perception, expended-radio and slot fixes below.

- **Radio traffic grew with the square of the swarm** while every drone heard every heartbeat: messages per drone doubled when the swarm doubled, about 4,300/s swarm-wide at 50 drones. Heartbeats now go to the ship only. At 50 drones that cut the messages each drone receives from 86 to 4.4 per second. It also cut host CPU from 3.8 to 3.1 cores, worst loop time from 137 to 86 ms and oldest sensor frame from 378 to 172 ms, and 23 of 25 threats were destroyed (was 22).
- **Profile (py-spy, 50 drones).** An active drone used 4.7–5.7% of a core in its main process and ~1.5% in its radio process. An expended drone still used 1.6%, because its radio kept running. Of the main process's work, 75–80% was lidar perception: ray tracing every 10 Hz scan into the voxel map, whether the drone was idle or engaged. The control loop was 12–16%; the auction, heartbeats and telemetry barely registered.
- **Two changes from the profile:**
  - Idle drones skip lidar processing. Only the planner reads the map, and it runs only with a goal; voxels expire after 0.5 s, and the first scan after tasking rebuilds the map.
  - Expended drones close their radio 1 s after detonating.

  At 50 drones, CPU per drone fell from 6.9% to 3.9%. Host CPU fell from 3.1 to 1.9 cores, control-loop overruns from 3,064 to 40, and the simulator reached 0.98× real time.
- **Profile, full architecture (radar, cooperative UWB, no-fly zone; 50 drones, 25 threats, 1×).** The whole swarm used 1.8 cores: drones 1.31, Gazebo 0.31, the simulator's Zenoh router 0.11, ship 0.04, metrics node 0.01. Per drone, from per-thread CPU over 33 s mid-run:

  | Drone state | Total | Control loop (50 Hz) | Zenoh callbacks and radio relay | Zenoh native threads | Radio process | Heartbeat |
  |---|---|---|---|---|---|---|
  | idle (30 drones) | 3.6 % (3.2–4.0) | 0.8 | 0.8 | 0.5 | 1.3 | 0.1 |
  | tasked (engaging, on station, bidding) | 3.4–4.1 % | 0.8–1.2 | 0.6–0.7 | 0.5 | 1.3–1.6 | 0.1 |
  | expended | 0.04 % | – | – | – | – | – |

  - **Almost all of it is fixed runtime overhead.** It goes to the two Zenoh sessions (onboard, and the radio in its own process) and to waking the 50 Hz control loop and the callback threads, so a tasked drone costs about the same as an idle one.
  - **The algorithms barely register.** py-spy, counting only threads that were not blocked, saw Python code running for about 0.5% of a core per drone. Localization, coop fusion, radar obstacles, fuze and planner were each under 0.1% of a core.
  - **Where the next saving is:** fewer idle wake-ups (a slower control loop or radio process while idle). Algorithmic work has nothing left worth optimizing.

  How it was measured: `utime + stime` of every thread in `/proc/*/task/*/stat` inside each container, read twice 30 s apart (`docker exec`); py-spy from an image built `FROM denddron-swarm-agent` with `pip install py-spy`, run with `--pid=container:<id> --cap-add SYS_PTRACE` in blocking mode. In `--nonblocking` mode py-spy cannot tell blocked threads from busy ones and reports every waiting thread as busy. Host `perf` needs `kernel.perf_event_paranoid` ≤ 2, so Gazebo and the native Zenoh threads were not profiled below the thread level.
- **The two remaining misses are geometry, not load.** In every 50-drone run the same drones miss the same threats by the same distances: drone_40 on T7 by 8.3–8.7 m, drone_48 on T22 by 10.4–11 m. That was true before and after the CPU fixes, so it isn't CPU starvation, as first assumed. It's probably an optimistic ETA bid or crowding on the way; it still needs investigating.

### Tools (`tools/comms/`)

| Script | What it does |
|---|---|
| `radio_probe.sh [routing] [netem\|disconnect]` | 4 radio-only peers; cuts one; reports per healthy peer the seconds it heard nobody, the worst gap and the worst process stall |
| `onboard_probe.sh [routing] [netem\|disconnect]` | drone-like processes with both links; cuts one radio; reports the worst onboard gaps of the cut drone and a healthy one. Set `RADIO_PROCESS=1` to use the separate radio process. |
| `cut_radio.sh <container> <network> <subnet> [netem\|disconnect] [duration]` | cuts one container's radio (used by the others) |
| `chaos.py <compose log> [disconnect\|netem]` | during a live run: cuts an idle drone, then the first drone that engages, then restores the idle one |
| `operator_bot.py [reaction_s] [duration_s] [poll_s]` | stand-in operator: approves feasible threats through the dashboard API, most urgent first |
| `degrade_radio.sh apply "<netem args>" \| clear [container...]` | impairs the radio (UDP only) of every drone and the ship: loss, delay, rate, combinable; `DST_IP=<ip>` impairs only traffic to that address |
| `declare_probe.sh late_sub\|fresh_pub\|both [netem]` | two radio peers, one side impaired: how many late subscriber declarations are lost (and for how long), and whether the first put on a new publisher is delivered as often as later ones. Env `TRIALS`, `WATCH_S`. |
| `mesh_probe.sh [netem]` | `PEERS` radio peers (8), every one impaired: session flaps, peers still connected at the end, heartbeat delivery, new subscriptions that heard a peer nothing within `WATCH_S` (3), and startup subscriptions gone deaf. Env `DURATION`, `DEAD_LIST`. |
| `sensing_sweep.py [--conditions NAME ...] [--custom NAME="K=V ..."] [--list] [--rtf K] [--drones N] [--threats K] [--repeats N]` | the seeded scenario under UWB, radar and ship-link degradation (see [Sensing and ship-link sweep](#sensing-and-ship-link-sweep)) |
| `degradation_sweep.py [--rtf K] [--drones N] [--threats K] [--maneuver-p P] [--repeats N] [--profiles ...] [--conditions NAME=NETEM ...] [--env KEY=VALUE ...]` | the seeded scenario under each impairment and QoS profile, with the stand-in operator; writes `results/<sweep>/results.{csv,md}` with mean ± sd per cell (see [Unattended runs and sweeps](#unattended-runs-and-sweeps)) |

All probes accept `IMAGE=...` to test another Zenoh build and pass through the radio settings (`RADIO_PROTO`, `RADIO_PROCESS`, `RADIO_LEASE_MS`, `RADIO_OPEN_TIMEOUT_MS`, `RADIO_ZENOH_CONFIG`, `RADIO_DEBUG`). `radio_probe.sh` takes `PEERS=N`.

## Instrumentation

Every drone sends `swarm/telemetry/{id}` once a second, covering the last second. The dashboard's *Swarm instrumentation* table shows it:

| Field | Meaning |
|---|---|
| `cpu_pct` | process CPU |
| `loop_hz`, `loop_p50_ms`, `loop_p99_ms`, `loop_work_p99_ms`, `overruns` | control loop timing, real time (50·`SIM_RTF` Hz). An overrun is a tick longer than 1.5 × the period. |
| `sensor_age_p50_ms`, `sensor_age_max_ms` | age of the newest onboard sensor frame, sampled every tick |
| `perception_p99_ms`, `planner_p99_ms` | lidar or radar processing per scan; planner per tick |
| `rx_per_s`, `tx_per_s`, `tx_bytes_per_s` | radio messages per topic, and bytes sent, per simulated second |
| `peers_heard`, `voxels` | peers heard from (bids, awards, help, clock beacons) in the last 3 s; voxel map size (radar: obstacle points held) |
| `clock` | clock sync state (only with a sync mode or an imperfect clock): mode, estimated offset, own error bound; with `consensus` also hops to the ship and whether it hears the ship. The ship adds the true error from `sim/clock_eval` (`clock_err_ms`, see [Distributed clock](#distributed-clock)) |

The ship adds per-topic radio receive rates, per-threat decision latency (approval → first and last award), and an event log. Events also go to `/state/ship_log.jsonl` in the `swarm_state` volume, for offline analysis, with each detonation's timing evaluation (`detonation_eval`, see [Distributed clock](#distributed-clock)). Every event carries truth (`sim_time`) and ship time (`ship_time`). The metrics node logs positions, distance flown and collisions (under 2.5 m).

Measured at 8 drones: about 1.5% CPU per drone, loop p99 about 20–26 ms, perception p99 about 3–12 ms. Radio to the ship is about 170 B/s of heartbeats and 220 B/s of telemetry per drone.

## Implementation map

Where each implemented feature lives.

| Area | Feature | Files |
|---|---|---|
| **Launch** | swarm launcher: drones, threat scenario, rebuilds | `scripts/run_swarm.sh` |
| | spawn layout and planner/kinematics defaults (`swarm_runtime.json`) | `scripts/generate_swarm_config.py` |
| | services, the two networks, healthchecks, clean shutdown | `docker-compose.yml`, `docker/*.Dockerfile` |
| **Hardware** | device-class record and derived simulation parameters | `hardware/hardware.json`, `src/common/hardware.py` |
| **Simulator** | physics: blasts at true positions, friendly fire, `sim/truth` | `GazeboSimulator.cpp` (`on_detonate`, `on_job`, `apply_detonations`, `publish_truth`) |
| | Gazebo bridge: kinematic integration from `cmd_vel`, spawn/despawn | `sim/GazeboSimulator.cpp/.hpp`, `sim/simulator_main.cpp` |
| | sim clock extrapolated between Gazebo's 5 Hz stats; pose 50 Hz, radar 20 Hz, UWB 2 Hz (per sim second, at any `SIM_RTF`) | `GazeboSimulator.cpp` (`estimated_sim_time`, `step`) |
| | legacy planar 32-ray lidar (ship + other drones), compact scan (`PERCEPTION=lidar`) | `GazeboSimulator.cpp` (`simulate_lidar`) |
| | threat models (by type) moved along the ship's tracks | `GazeboSimulator.cpp` (`generate_threat_sdf`, `on_threat_track`, `move_threat_markers`) |
| | quadcopter drone model; model deletion (despawn, destroyed threats) | `GazeboSimulator.cpp` (`generate_drone_sdf`, `delete_model`) |
| | world: ocean, lighting, frigate (~31 m, sized to the simulator's 16 m ship radius) | `sim/ocean.world` |
| **Links** | onboard bus (router) and radio (peer-to-peer) session config; radio QUIC control plane + UDP data, lease | `src/common/links.py`, `src/common/radio_tls/` |
| | radio in a separate OS process | `src/common/radio_process.py` |
| **Threat model** | threat types and levels, CPA/TCPA, engagement point, slots, ETA, TTI | `src/common/threats.py` |
| **Ship C2** | radar simulation (track generation) | `src/ship/ship.py` (`_maybe_detect`, `Track`) |
| | threat min-heap by TCPA | `src/ship/threat_queue.py` |
| | roster from heartbeats (+ relayed) | `ship.py` (`_on_heartbeat`, `_on_relayed_heartbeat`, `members`) |
| | feasibility (TTI vs. time to engagement), operator approval, orders, re-announcement | `ship.py` (`feasibility`, `approve`, `_announce`, `_retry_underassigned`) |
| | job confirmation (ACK/NACK, slots), job updates to the drones' inboxes, answers to heartbeats, heartbeat reconciliation, order copies, threat manoeuvres | `src/common/jobs.py` (`arbitrate`, `heartbeat_answer`), `ship.py` (`_arbitrate`, `_publish_jobs`, `_answer_heartbeat`, `_reconcile`, `_send_order_copies`, `_maneuver`) |
| | kill assessment (truth), detonation timing evaluation, leak/impact/failed | `ship.py` (`_on_detonation`, `_evaluate_detonation`, `_age_tracks`) |
| | radar track in truth (world) and ship time (C2) | `ship.py` (`Track`) |
| | instrumentation aggregation, event log (`/state/ship_log.jsonl`), HTTP/SSE API | `ship.py` (`snapshot`, `make_handler`) |
| | operator dashboard (map, heap queue, approval, swarm table, radio, events) | `src/ship/dashboard.html` |
| **Drone** | agent lifecycle, ID allocation, SIGTERM | `src/agent/main.py` |
| | legacy perception (`PERCEPTION=lidar`): voxel map, occupied index, batched ray tracing, expiry; loaded only in that mode | `src/agent/voxel_map.py`, `agent.py` (`_process_lidar`) |
| | planners (APF, ORCA) | `src/agent/path_planning.py` |
| | 50 Hz control loop, safety envelope, arrival latch | `agent.py` (`_reflex_control_loop`) |
| | sim-time / wall-time handling | `src/agent/timing.py` |
| | protocol clock scaled by the real-time factor (`SIM_RTF`); `truth_now` for exchange stamps | `src/common/simclock.py` |
| | bidding (ETA), engagement, slots, detonation / abort | `agent.py` (`_on_threat_wave`, `_calculate_costs`, `_service_auctions`, `_check_engagement`) |
| | inbox (ACKs and job updates), keep flying until confirmation can no longer help, release; bids sent 3 times | `agent.py` (`_on_inbox`, `_on_ack`, `_on_job`, `_apply_job`, `_check_engagement`, `_abandon`, `_propose_bid`) |
| | decentralized all-or-nothing priority assignment, conflict yield | `src/agent/auction.py` |
| | heartbeat, roster check, link state, peer relay | `agent.py` (`_heartbeat_loop`, `_on_roster`, `_relay_unheard_peers`) |
| | telemetry (loop, sensors, perception, planner, CPU, radio) | `src/agent/telemetry.py` |
| **Clocks** | per-node hardware clock: drift, offset, exchange jitter, reproducible per-node draw | `src/common/localclock.py` |
| | sync modes (`none`, `ttg`, `master`, `consensus`): exchange filter, ship-anchored consensus, pluggable aggregators, error bounds | `src/common/timesync.py` |
| | drone: protocol time, inbound conversion, exchange stamps, relay holding time, clock beacons, clock telemetry and evaluation | `agent.py` (`_proto_now`, `_inbound_job`, `_stamp`, `_on_roster`, `_relay_unheard_peers`, `_send_clock_beacon`, `_on_clock_beacon`, `_clock_report`) |
| | ship: `sent` stamps, exchange stamps and leader beacon in heartbeats and roster, true sync error | `ship.py` (`_stamped`, `_on_heartbeat`, `run`, `_on_clock_eval`) |
| **Metrics** | positions, distance, proximity collisions (spatial hash); per-drone log when a localization error crosses 5/10/30/100 m | `src/metrics/main.py` |
| **Localization** | UWB ranging (anchors, peers with state payloads, noise, range, dropouts, airtime, jamming), x,y withheld; wind | `GazeboSimulator.cpp` (`configure_localization`, `publish_uwb`, `on_uwb_tx`, `configure_environment`) |
| | EKF (position, velocity, wind), gating, re-lock, multilateration; robust mode: adaptive noise, Huber update, residual-checked re-locks | `src/agent/localization.py` |
| | cooperative localization: neighbour table, anchor-time chains, peer fusion | `src/agent/coop.py` |
| | drone integration, station keeping against wind | `agent.py` (`_loc_predict`, `_on_uwb`, `_send_uwb_tx`, `_reflex_control_loop`) |
| | evaluation: errors, NEES, separations | `src/metrics/main.py` (`_on_loc`, `_loc_summary`) |
| **Perception** | mmWave radar (default; lidar off) | `GazeboSimulator.cpp` (`configure_fuze`, `publish_fuze`) |
| | radar contacts -> planner obstacles | `src/agent/radar_obstacles.py`, `agent.py` (`_on_contacts`) |
| **No-fly zone** | routes around the ship's zone, intercepts outside it; planner barrier with 2σ margin; stations outside | `deconflict.py` (`detour`, `plan_route`, `choose_intercept`), `path_planning.py`, `ship.py` (`engage_radius`), `generate_swarm_config.py` (`--no-fly`) |
| **Fuze** | fuze sensor: unlabelled contacts with noise and latency | `GazeboSimulator.cpp` (`configure_fuze`, `publish_fuze`, `flush_fuze`) |
| | fuze logic: tracking, mate record, arming, gate, firing rules, fallback | `src/agent/fuze.py` |
| | drone: fuze scans, detonation (once), chain fire, fallback | `agent.py` (`_on_contacts`, `_fuze_decision`, `_detonate`, `_on_mate_blast`, `_check_engagement`) |
| | ship: reason, trigger label, chain delay, no-detection count | `ship.py` (`_evaluate_detonation`, `_on_award`, `summary`) |
| **Tests / tools** | unit tests and validation scripts | `tests/` (see [Testing](#testing)) |
| | degraded-comms probes, radio cut, chaos, stand-in operator, radio degradation, sweep | `tools/comms/` |

## Agent internals

`src/agent/agent.py`:

1. **Position: `localization.py`, `coop.py`.** An EKF on position, velocity and wind, predicted with the commanded velocity and corrected by UWB ranges to the ship's anchors and, out of their reach, to peers with fresher anchor chains. Its uncertainty (σ) widens the no-fly barrier, the fuze gate and the intruder check. See [Localization and perception](#localization-and-perception).
2. **Eyes: `radar_obstacles.py`.**
   - Radar contacts become planar obstacle points: 32 rays, each contact a 3 m circle at the drone's altitude, ship returns dropped.
   - Updated only while the drone has a goal; points expire.
   - The legacy lidar mode uses `voxel_map.py` instead: a sparse 0.5 m occupancy grid, ray-traced in vectorized batches, with expiry.
3. **Reflexes: `path_planning.py` and the 50 Hz loop.**
   - Planning uses APF (default) or ORCA against the obstacle points, the ship and the no-fly zone.
   - The loop then applies:
     - a floor/ceiling guard,
     - damping near obstacles,
     - service-radius containment,
     - a braking envelope,
     - acceleration and speed limits.

     Arrival latches (within 0.5 m of the goal's altitude) and the drone holds position, re-approaching if wind pushes it more than 1 m beyond where it latched.
   - An untasked drone holds station: zero velocity, and back to its hold point after drifting 1 m.
4. **Timing: `timing.py`.** Simulation time drives the physics integration. Wall-clock time drives sensor freshness. If sensor data is stale, the drone commands zero velocity.
5. **Engagement: `auction.py` + `src/common/threats.py` + `fuze.py`.** Covers bidding, assignment, slots, the detonation time and the proximity fuze. See [Engagement protocol](#engagement-protocol).
6. **Telemetry: `telemetry.py`.**

## Configuration

### Hardware record

`hardware/hardware.json` records the class of every device the swarm uses, not exact models, with its typical real performance:
- airframe (speed, acceleration, climb and descent rates);
- warhead (kill radius);
- UWB radio and the ship's UWB anchors;
- mmWave radar;
- IMU, barometer and compass;
- the C2 radio.

Simulation parameters are derived from it (`src/common/hardware.py`):
- **airframe speeds and accelerations** are the real figures × `simulation.speed_scale` (0.1 reproduces the 4 m/s, 1 m/s² drones);
- **sensor noise, range and rate** are used as recorded.

Devices marked `"simulated": false` are recorded for hardware deployment but not modelled; for example, altitude, attitude and heading are taken as perfect. The launcher copies the record into `config/swarm_runtime.json` under `hardware`, where the simulator, drones and ship read it; `HARDWARE_RECORD=<file>` points the launcher at another one. When a sensor or airframe model is added or changed, its class and parameters go into the record first.

`config/swarm_runtime.json` is generated on every launch and is not tracked in git. To change the defaults, edit `GLOBAL_DEFAULTS` in `scripts/generate_swarm_config.py`.

| Section | Keys |
|---|---|
| `defaults.path_planning` | `algorithm`, APF gains and radii, `step_size`, `goal_tolerance`, `braking_radius`, `ship_keepout_radius`, `velocity_smoothing`; optional `time_horizon_obst`, `agent_radius`, `max_control_dt`, `sensor_timeout_s` |
| `defaults.kinematics` | `max_velocity` (4 m/s), `max_acceleration` (1 m/s²), `min_z`, `max_z`, `max_service_radius` — also used by the ship's TTI |
| `defaults.goal_control` | `tolerance`, `stop_radius`, `tolerance_xy`, `tolerance_z`, `settle_ticks` |
| `defaults.auction` (optional) | `bid_window_s` (1.0) |
| `agents.drone_N` | `spawn{x,y,z}`, optional `goal{x,y,z}` and per-drone overrides |

Simulation speed (env): `SIM_RTF` (1), set by `--rtf`.

Ship no-fly zone (env): `NO_FLY_RADIUS_M` (50; 0 = off, stations at 30–45 m; `--no-fly R`, which also moves stations to R+5 … R+45 m).

Localization and perception (env): `PERCEPTION` (`radar`, or legacy `lidar`; `--perception`), `LOCALIZATION` (`coop`, `anchors`, or legacy `truth`; `--localization`), `UWB_FILTER` (`robust`, or the plain `basic`), `UWB_JAM` (`what:t0:t1[:x:y:r]`, what = `anchors`|`peers`|`all`; a jammer at (x, y) affects drones within r m), `UWB_SEED`, `WIND_MPS` (`x,y` real m/s, `--wind`), `GUST_SIGMA_MPS` (`--gust`). Sensing degradation (simulator only; empty = the record, which the drones keep assuming): `UWB_EXTRA_SIGMA_M`, `UWB_NLOS_P`, `UWB_NLOS_BIAS_M`, `UWB_DROPOUT_P`, `UWB_MAX_RANGE_M`, `RADAR_RANGE_SIGMA_M`, `RADAR_AZ_SIGMA_DEG`, `RADAR_EL_SIGMA_DEG`, `RADAR_MISS_P`, `RADAR_CLUTTER`, `RADAR_LATENCY_S` (see [Sensing and ship-link sweep](#sensing-and-ship-link-sweep)). Sensor and environment parameters are in the [hardware record](#hardware-record).

Proximity fuze (env): `FUZE` (1; 0 = timed detonation), `FUZE_WINDOW_S` (2), `FUZE_GATE_M` (5), `FUZE_FIRE` (`cpa`), `FUZE_FALLBACK` (`hold`), and for the simulator's sensor `FUZE_RANGE_M` (10, drones too), `FUZE_HZ` (50), `FUZE_NOISE_M` (0.1), `FUZE_LATENCY_S` (0), `FUZE_SEED` (0). See [Proximity fuze](#proximity-fuze).

Clocks (env, all default 0 / `none` = perfect shared clock): drones `CLOCK_DRIFT_PPM`, `CLOCK_DRIFT_SPREAD_PPM`, `CLOCK_OFFSET_S`, `CLOCK_OFFSET_SPREAD_S`, `CLOCK_JITTER_S`; ship `SHIP_CLOCK_DRIFT_PPM`, `SHIP_CLOCK_OFFSET_S`, `SHIP_CLOCK_JITTER_S`; both `CLOCK_SEED`, `CLOCK_SYNC`. Consensus (drones): `CLOCK_BEACON_HZ` (0.5, `--clock-beacon-hz`), `CLOCK_GAIN` (0.5), `CLOCK_LEADER_SHARE` (0.5), `CLOCK_STEP_S` (0.05), `CLOCK_AGGREGATE` (`mean`). See [Distributed clock](#distributed-clock).

Diagnostics (env): `CONFIRM_TRACE` (0; 1 = log every award, ACK, job update and skipped order with its sim time, drones and ship), `RADIO_PEER_LOG` (0; 1 = log the radio's Zenoh id and every peer session that opens or closes).

Radio tuning (env): `RADIO_QOS` (`default`; `tuned` = per-topic priorities, see `QOS_PROFILES` in `links.py`), `RADIO_PROTO` (`quic,udp`; `udp` = the old UDP-only radio), `RADIO_LEASE_MS` (6000), `RADIO_OPEN_TIMEOUT_MS` (1000), `RADIO_SUBNET` (`172.21.0.0/16`), `RADIO_ZENOH_CONFIG` (JSON object of extra Zenoh settings, for experiments).

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
| `test_jobs.py` | ship confirmation: best bids confirmed, distinct slots in drone-ID order, held slots kept; the ship's answer to each heartbeat (ACK again, NACK again, award from a heartbeat, release a dropped or closed job) |
| `test_threat_queue.py` | min-heap ordering by TCPA, lazy removal |
| `test_hardware.py` | hardware record complete, kinematics reproduce the original drones, runtime config wins; drones never read `sim/truth` |
| `test_localclock.py` | perfect default, drift/offset model, jitter only in exchange stamps, reproducible per-node draw |
| `test_deconflict.py` | spatial queue: speed profiles, routes with holds and exits around blasts, earliest-feasible intercepts, manoeuvre re-plan slack; routes around the no-fly zone (legs clear, shorter way, timing) |
| `test_localization.py` | EKF from anchor ranges: multilateration, tracking a moving drone consistently (NEES), outlier gating, dropouts, re-lock after a corrupted estimate, uncertainty growth, wind learned, outage dead-reckoned with the wind; robust filter: adaptive noise at 3× the record's noise (with and without NLOS), re-lock leaving out a blocked anchor, refused re-lock when three anchors disagree |
| `test_coop.py` | cooperative localization: a drone beyond anchor range localized through anchored peers (consistent), drift without peers, no stale "anchored" loop with every anchor jammed, peer selection |
| `test_radar_obstacles.py` | radar contacts give exactly the lidar's obstacle points (50 random layouts vs a port of the simulator's lidar), ship contacts dropped, staleness |
| `test_fuze.py` | proximity fuze: fires at closest approach (three threat speeds), threat in range before arming not taken for a mate, mates never trigger, hold and timed fallbacks, closest approach beyond the kill radius, contacts off the predicted track ignored, radius mode, arming window, stale track, chain readiness, config |
| `test_ship_track.py` | radar track: truth vs ship-time view, true closest approach; ship time attributes set at start (needs zenoh installed) |
| `test_timesync.py` | exchange arithmetic and asymmetry bias; ship-master filter: convergence under drift and offset, holdover on the rate, jitter and delay spikes, bound covers a constant asymmetry, congested start, corrupt exchange (negative round trip) discarded, re-acquisition after a wrong lock; time-to-go anchoring |
| `test_consensus.py` | ship-anchored consensus on a simulated network: converges to ship time with ±500 ppm / ±3 s, learns rates, works through a chain of peers, keeps agreeing and holds ship time with the ship lost (bound grows with time only), congested start then ship loss, fast start by stepping, late unsynced joiner, echoes measure link delay, anchors count hops, bad ship exchanges filtered, pluggable aggregator |
| `test_voxel_map_batch.py` | legacy voxel map: batched ray tracing, occupied index vs. brute force, incremental expiry, clock reset |
| `test_apf.py`, `test_orca_vertical_filter.py` | planner force bounds, ORCA vertical envelope |
| `test_timing_manager.py`, `test_time_handling.py` | timing states |
| `test_simclock.py` | protocol clock follows the measured sim rate, never goes backwards, survives restarts; `truth_now` extrapolation |
| `test_metrics_spatial_hash.py` | spatial-hash collision check matches brute force |
| `benchmark_voxelmap.py`, `validate_*.py` | throughput, planner scenarios, timing, metrics overhead, worker lifecycle |

## Repository layout

```
config/                 generated runtime files — not tracked
docker/                 base (zenoh-c/cpp), gazebo, agent, ship, metrics images
scripts/run_swarm.sh    launcher;  scripts/generate_swarm_config.py
sim/                    GazeboSimulator (C++ bridge, kinematics, markers), simulator_main.cpp, ocean.world
src/common/             threat model and geometry (threats.py), Zenoh links (links.py), radio process (radio_process.py),
                        spatial queue (deconflict.py), jobs (jobs.py), clocks (simclock.py, localclock.py, timesync.py)
src/agent/              drone: agent, localization (EKF) and coop, radar obstacles, fuze, auction, planners, timing,
                        telemetry; legacy voxel map
hardware/hardware.json  device-class record: every simulation parameter comes from it
src/ship/               ship C2 (ship.py), threat min-heap (threat_queue.py), dashboard.html
src/metrics/main.py     metrics node (truth vs estimates, separations, distance)
tests/                  unit tests and validation scripts
tools/comms/            degraded-comms probes, radio cut helper, chaos script, stand-in operator
```

## Next steps

The plan, written 2026-10-04, in build order: each item depends only on those above it. Each item is switchable and gets its own branch off `main`. During development each item gets a light check (a few sweep conditions, one run each); the full [sensing sweep](#sensing-and-ship-link-sweep) with repeats is kept for system testing once all of them are in place. Items marked **decision** still need the project owner's call.

### 1. Robust job confirmation (ship-link loss) — done

Done on branch `robust-confirmation` (2026-10-04); see [Engagement protocol](#engagement-protocol). Light check, one run each, 15 drones, 8 threats: `ship_loss30` 8/8 (was 4/8), `ship_outage` 8/8, `combined` 8/8 (was 4/8), `baseline` 8/8; whole radio at 10 % and 30 % loss with 8 drones and 4 threats, 4/4 each with no re-announcement.

**What the diagnosis found** (`CONFIRM_TRACE=1`, `tools/comms/declare_probe.sh`):
- **Lost subscriber declarations, not lost messages.** The drone subscribed to `ship/jobs/{threat}` when it won. Over UDP, Zenoh sends that declaration once, so at 30 % loss about 30 % of these subscriptions heard nothing for longer than the whole engagement. In the traced runs, two confirmed drones got none of 7 job updates while the others got 58–75 %. Fix: job updates go to the drone's inbox, subscribed at startup. The cause underneath, lost Zenoh control messages, is fixed in item 1b.
- **Lost orders to the one drone that can make it.** Intercepts are often reachable by only one or two drones, with about 4 s of slack. When the order to that drone was lost, nobody bid, and the re-announcement 3 s later was too late for anyone (best margin −0.8 s). This lost 3 of the 5 threats in one diagnosis run. Fix: each order is sent 3 times.
- **The fixed 3 s ACK timeout** dropped the remaining drones. Fix: heartbeats carry the award, the ship answers them, and an unconfirmed drone keeps flying until confirmation can no longer help (owner's decision).
- Batching in Zenoh is not the cause: the first put on a new publisher is delivered as often as later ones (62 % vs 66 % at 30 % loss).

### 1b. Radio robustness (whole-radio loss) — done

Found while measuring item 1, done on branch `radio-robustness` (2026-10-04); see [Degraded communications](#degraded-communications).
- **Lost control messages:** a QUIC link carries Zenoh's control messages; data stays best-effort over UDP.
- **Sessions closed by lost keep-alives:** the lease is 6 s instead of 2 s.
- **Level-2 threats lost to lost bids** (each winner must hear the other's bid): bids are sent 3 times.

Light check, 30 % loss on every link, 15 drones, 8 threats, one run each:

| Radio | Destroyed | Re-announces | Decision latency mean / max | Assignment agreement |
|---|---|---|---|---|
| UDP, 2 s lease (item 1 only) | 5/8 | 6 | 1.66 / 2.83 s | 0.72 |
| QUIC control plane, 6 s lease | 8/8 | 1 | 1.93 / 4.64 s | split auctions (T1: 9 vs 6) |
| + bids sent 3 times | **8/8** | **0** | **1.49 / 1.88 s** | 104 of 105 drone views agree |

Also 8/8 for `baseline`, `ship_loss30` and `combined` (sensing sweep, one run each).

### 2. Robust UWB filtering — done

Done on branch `robust-uwb` (2026-10-04), the default (`UWB_FILTER=robust`; `basic` is the plain filter); see [Localization and perception](#localization-and-perception). Owner's decision: re-lock from 3 or more anchors with a residual check.

Unit tests (`test_localization.py`, 3 seeds each; plain vs robust): 3× the record's noise NEES 23 → 2.0, re-locks 14 → 0; 3× noise with 10 % NLOS NEES 28 → 2.1, worst error 13.9 → 3.8 m; a re-lock with one blocked anchor of four leaves it out (plain: pulled over 1 m off); three disagreeing anchors refuse a re-lock.

Light check (sensing sweep, 15 drones, 8 threats, one run each; plain filter from the [first results](#sensing-and-ship-link-sweep)):

| Condition | Error p95 / max | NEES | Re-locks | Destroyed |
|---|---|---|---|---|
| `baseline` | 0.42 / 1.16 → 0.42 / 1.32 m | 1.5 → 1.4 | 0 → 0 | 8/8 |
| `uwb_noise` | 2.84 / 18.6 → 1.13–1.17 / 9.6–10.2 m | 24.8 → 2.0–2.1 | 86 → 0–2 | 8/8 |
| `uwb_nlos` | 0.52 / 14.0 → 0.50 / 2.7 m | 2.5 → 1.6 | 3 → 0 | 8/8 |
| `uwb_jam_all` | 2.55 / 7.1 → 2.13 / 5.1 m | 1.1 → 1.1 | 0 → 0 | 8/8 |
| `combined` | 1.27 / 7.3 → 0.91 / 7.2 m | 10.4 → 1.9 | – | 8/8 (with items 1, 1b) |
| `uwb_short` | 16.3 / 36.1 → 15–16 / 25 m | 50 → 40–63 | 0 | 5/8 → 7/8, 8/8 |

`uwb_short` stays bad: beyond anchor range only peers localize, which is item 4 (RDL). Applying the robust update to peer ranges too made it much worse (NEES 399, one drone 74 m off), so peers keep the plain gate. One `uwb_short` run with two swarms side by side had one drone 123 m off; a run alone did not repeat it. The remaining `uwb_noise` excursions (~10 m) are not yet traced to a drone (the metrics node now logs every drone whose error crosses 5, 10, 30 and 100 m).

### 3. IMU dead reckoning

**Why.** Between UWB fixes the EKF predicts with the commanded-velocity response model, which on hardware is only approximately known, and the simulator applies exactly that model. An IMU measures the motion that actually happened, wind included. It is the layer under everything else:
- it buys time in a UWB (and GNSS) blackout: the error grows from the IMU's bias, not from an unknown wind and response;
- it is an independent reference to detect UWB drift (non-line-of-sight bias, a bad re-lock) and GNSS spoofing: a spoofer or a biased range can move the fix, but not the measured acceleration;
- it gives RDL (item 4) and the GNSS comparator (item 5) a motion model whose uncertainty is known.

**Class.** The record has a flight-controller MEMS IMU (BMI088-class, `simulated: false`). Its accelerometer bias after calibration is ~1 mg (0.01 m/s²); uncompensated, that is 0.5·b·t² of error: ~4.5 m after 30 s, ~18 m after 60 s. An industrial navigation-grade MEMS IMU (ADIS1650x-class, ~0.1 mg in-run bias) gives ~10× less. Both go in the record with noise density, bias instability, bias random walk, scale factor and rate, and the sweep shows how long each holds a position. Attitude stays perfect (as now), so a tilt error enters only as an equivalent accelerometer bias.

**Steps:**
1. **Hardware record.** Complete the `imu` class (above) and set it `simulated: true`; add the navigation-grade class as an alternative (`IMU_CLASS`).
2. **Simulator.** Publish `drone/{id}/imu` as delta-velocity increments pre-integrated to 50 Hz, as a real IMU FIFO is read, not 400 Hz messages (50 drones × 400 Hz would load the bus). Errors: white noise, a bias (Gauss–Markov), scale factor; impairment knob `IMU_EXTRA_BIAS_MPS2`. Like UWB noise, not speed-scaled: at `speed_scale` 0.1 a blackout lasts longer in sim seconds than it would in the real world, which makes it a conservative test.
3. **Drone.** The EKF predicts with the IMU (state: position, velocity, accelerometer bias in x,y) in place of the command model and its wind state; the command model stays as the fallback when the IMU is off (`NAV_PREDICT=imu|cmd`). The bias is learned while UWB is good and held through an outage.
4. **Drift and spoof check.** A windowed innovation test between the IMU-propagated track and each absolute source (anchors, peers, GNSS), flagged in telemetry and heartbeats. Item 5 uses it for spoof detection.
5. **Light check:** `baseline`, `uwb_jam_all` (dead-reckoning error after 60 s vs the bias figure), wind with gusts, `uwb_nlos`.

**Decided (2026-10-04):** the flight-controller class (BMI088-class) is the default and the only class modelled. The goal is cheap drones deployed by the dozens, using the IMU their flight controller already has; a navigation-grade IMU would mean custom hardware. The record keeps the navigation-grade class only as a reference figure.

### 4. Recursive decentralized localization (RDL)

**Problem.** Peer fusion today uses each peer's estimate as if it were independent of ours. That stops the 7× overconfidence seen first (by flooring our variance at the peer's), but not overconfidence along chains. With UWB range cut to 80 m, drones localized only through peers were 16 m off (p95) while claiming sub-metre accuracy (NEES 50).

**Method.** RDL (Luft et al., 2018) keeps the cross-covariance between every pair of drones that have exchanged ranges, in factored form: each drone stores its share σᵢⱼ for each peer j. A pairwise range update needs only the two drones involved. It produces a consistent joint update of both states, and updates the two drones' factors, while every other factor stays valid. Propagation multiplies only the drone's own factors by its transition matrix. No drone needs the global covariance, and it tolerates asynchronous, lossy exchanges.

**Steps:**
1. **`src/agent/rdl.py`**, pure and unit-tested on the 6-state EKF (position, velocity, and wind or, with the IMU, accelerometer bias):
   - factor storage per peer, capped at the UWB `max_peers` plus recent partners, oldest dropped (a dropped factor is treated as unknown correlation: covariance intersection for that pair);
   - the joint 12-state range update;
   - anchor updates (anchors are uncorrelated landmarks, so only the drone's own factors change).

   Tests reuse `test_coop.py`'s swarm:
   - consistency (NEES ≈ 2) for the far drone;
   - with every anchor jammed, relative positions stay tight while absolute error grows honestly;
   - the `uwb_short` geometry;
   - exchange loss.
2. **Exchange.** A range update changes both drones, so the initiator sends the responder its new state and factor in a reply. The simulator carries it in the next UWB frame, as `uwb_tx` payloads are carried now, with the same dropout and capacity model. A lost reply is detected by sequence number, and that pair falls back to covariance intersection.
3. **Integration** behind `LOCALIZATION=rdl`, with `coop` the default until RDL beats it in the sweep (`uwb_short`, `uwb_jam_all`, `uwb_jam_local`, `combined`; NEES, p95 and max error, kills).
4. **Cost.** State per drone grows with the peer count: 6 × 6 per factor, so about 1.5 kB at 6 peers. The UWB payload grows by one 6 × 6 factor per exchange (~150 B in float16), more than the record's 64 B. The payload may be expanded (DW3000-class radios carry frames up to 1023 B) if that brings a benefit: measure both and record the difference and the airtime.

It builds on the robust update (item 2) and the IMU motion model (item 3). It is the larger job: about the size of the cooperative localization work.

### 5. GNSS as comparator and fallback

GNSS is not used today: position comes from the ship's UWB anchors, which also work where GNSS is jammed or spoofed. Where it is available, it can check UWB and, when UWB fails, stand in for it.

**Steps:**
1. **Hardware record.** Add a `gnss` class: a multi-band, multi-constellation receiver with typical 1.5–3 m horizontal σ standalone, 5–10 Hz, and fix quality and satellite count. No RTK: the ship is a moving base in the real world, so its corrections don't apply. Add a ship `gnss` and heading entry, since positions are needed relative to the ship.
2. **Simulator.**
   - Publish `drone/{id}/gnss` with the record's noise and a slowly varying bias (Gauss–Markov, like real receiver errors).
   - Impairment knobs as for UWB: `GNSS_JAM` (outage windows, local or global) and `GNSS_SPOOF` (an offset that ramps in slowly, the hard case to detect). Like UWB noise, these are not speed-scaled.
3. **Drone.**
   - **Comparator (on by default):** check GNSS against the UWB estimate with a windowed normalized-innovation test. While UWB is good, GNSS is not fused into the position estimate, so a spoofer cannot pull the drone. Disagreement is flagged in telemetry and heartbeats.
   - **On a detected spoof:** the drone reports it to the ship, then ignores GNSS and relies on UWB alone.
   - The ship-relative conversion uses the ship's own GNSS position and heading.
4. **Sweep conditions:**
   - `gnss_fallback_uwb_short`: UWB to 80 m with GNSS on, the case where peer chains failed;
   - `gnss_jam`;
   - `gnss_spoof_ramp`;
   - `gnss_spoof` with `uwb_jam_all`, the worst case.

**Decided (2026-10-04):** GNSS is a comparator for UWB, never fused while UWB is available; it is on by default and ignored once a spoof is detected (after reporting to the ship); no RTK.

**Fallback (decided 2026-10-04):** when UWB fails (no anchor chain, and RDL's uncertainty shows the estimate is no longer good), GNSS stands in, fused with an inflated noise; the IMU check (item 3) and the comparator stay active so a spoof or a jump still rejects it. This needs RDL first, so the drone knows how uncertain its UWB estimate really is, and the IMU, so it can tell a spoof from drift.

### 6. Already-open items

The chain-fire and stack-spacing decisions are deferred until items 1–5 are done (2026-10-04).

- **Chain fire for spread-out mates** (**decision**): fire only if the mate's own radar sees the threat inside the kill radius, otherwise wait for its own fuze.
- **Three-drone stack spacing** (**decision**): spacing about 4 m, or a lower proximity threshold.
- **Speed retune and 1 km detection:** `speed_scale` 0.2–0.4, threats faster, detection ~1 km out.
- **Continuous position hold** in wind, instead of latch and re-approach.
- **Motion-model mismatch:** drones use an approximate command response, not the simulator's exact one (largely replaced by the IMU prediction, item 3).
- **Fuze window** centred on the predicted arrival rather than `t_engage`.
- **Clocks:**
  - a timestamp-uncertainty term in the sync error bounds;
  - a trimmed-mean or MSR aggregator for consensus;
  - the planned clock sweep (drift, offset, sync mode, fuze, radio, threat speed).
- **Fewer idle wake-ups** (the [CPU profile](#scaling)): a slower control loop and radio for idle drones.

## Known limitations
- **Localization is optimistic.** The drones' motion model is the simulator's exact command response, and wind is the only disturbance. On hardware the response must be identified and is only approximately known. By default, ultra-wideband (UWB) ranges have no blocked-path or multipath errors, and the radar has no clutter or false alarms. The [sensing sweep](#sensing-and-ship-link-sweep) adds them as impairments: a positive non-line-of-sight bias, missed detections and uniform clutter. These are simple models, not a propagation or radar-scene simulation.
- **No consistent fusion between unanchored drones.** Peers are used only along fresh anchor chains. With every anchor jammed, drones dead-reckon independently (their uncertainty grows honestly), and peer ranges keep nobody's estimate tight. Recursive decentralized localization (RDL), which tracks cross-covariances with pairwise exchanges, is the planned fix.
- **Degraded sensing** (see the [sensing sweep](#sensing-and-ship-link-sweep)):
  - **Peer chains beyond anchor range:** with UWB range cut to 80 m, drones localized only through peers were tens of metres off while claiming sub-metre accuracy (NEES 50).
- **Station keeping in wind.** Drones latch up to 3 m from their goal, then drift downwind until they re-approach: 2.6–2.9 m misses in a 0.5 m/s wind, with true positions too. Continuous position hold would remove most of it.
- **Not yet done:** the planned speed retune (`speed_scale` 0.2–0.4, threats faster) and 1 km threat detection.
- **Routes around the no-fly zone are timed conservatively:** each detour waypoint is planned from rest, so ETAs over-estimate.
- **Chain-fire delay.** A real mate-to-mate trigger such as a barometric shock travels at about the speed of sound: ~17 ms across a 6 m stack, which lets a fast threat escape. A barometric trigger would also respond to unrelated blasts. The simulation uses the detonation topic as an idealized, job-selective trigger (measured delivery 1.5–32 ms at `--rtf 3`).
- **Closing-speed discrimination.** Closing speed would separate threats from drones only at real speeds, not at the simulation's scaled ones, so the fuze does not use it.
- **Fuze window and clocks.** The window is centred on `t_engage`, but threats arrive ~1.1 s late (drones stop short), so without clock sync a clock ~1 s ahead closes the window too early (see [Proximity fuze](#proximity-fuze)).
- **Chain fire fires spread-out mates early.** Job-mates end up several metres apart along the threat's track, and chain fire sets them all off when the first one's fuze fires, up to 2 s before the threat reaches the last. At 50 drones this cost two three-drone kills in one run (misses of 8.4 and 9.5 m, where each drone's own fuze would have missed by 1–3 m).
- **Stack spacing vs proximity.** Three-drone stacks are 3 m apart vertically; with 0.5 m altitude latching, stacked mates can be 2.0–2.5 m apart, under the metrics node's 2.5 m proximity threshold.
- **Late job-mates.** A job-mate that enters fuze range after the mates were recorded (still flying in) is a new track; if it passes within the gate of the predicted threat position it could trigger the fuze. The evaluation labels every trigger; none was false in the runs so far.
- **Best-effort radio.** Data runs over UDP, so messages can be lost under packet loss; Zenoh's control messages go over QUIC and are retransmitted. At 50 % loss QUIC's own recovery slows down and new subscriptions are often still dead after 10 s. See [Degraded communications](#degraded-communications).
- **Shared radio certificate.** Every node uses the same simulation-only TLS key for the QUIC link; hardware needs a key per node (and see "No security").
- **The ship is a single point of failure, by design.** It is the only threat sensor and the only source of engagement orders.
- **Clock synchronization.** `ttg` and `master` need the ship's messages; a drone that stops hearing the ship keeps its last estimate (with `master`, its last offset and rate). `consensus` keeps drones agreeing without the ship, but:
  - a path asymmetry (one direction slower than the other) is an error of half of it that no exchange can observe, and without the ship to anchor them drones then drift together by about gain × that per beacon;
  - beacons go from every drone to every drone, so their radio load grows with the square of the swarm;
  - the aggregation trusts every neighbour: a single drone lying about its clock pulls everyone (a trimmed-mean or MSR aggregator is the planned fix);
  - every drone beacons to every drone and measures each link only every ~2·(N − 1) s: at 100 drones, ~50 beacons/s per drone and offsets minutes old (see [Stress test](#stress-test-50-and-100-drones)).
- **Self-reported clock bounds** don't include timestamping uncertainty, so with `master` they cover only a third of reports at 50 drones.
- **100 drones exceeds a 6-core host** at 1×: the simulator runs at 0.25–0.65× real time.
- **Idealized threats.** They fly straight lines at constant speed, apart from at most one optional turn, and a detonation within the kill radius always kills (no kill probability).
- **Re-planning after a manoeuvre uses the job's own drones only.** The ship re-picks the intercept point for them, falling back to the legacy point (closest approach or 45 m crossing) if none fits, which with 1 s of slack was never needed in the runs so far. It doesn't bring in closer free drones.
- **Greedy assignment.** Orders are assigned per order, highest level first, not globally optimized. A drone waiting on one order's result skips any other order that arrives before that result.
- **Planar perception.** Radar contacts (like the legacy lidar) become obstacle points at the drone's own altitude, so avoidance is 2D; the fuze uses the full 3D contacts. ORCA uses greedy projection, not a full linear program.
- **No security.** Radio traffic is unauthenticated: a forged order or bid would be acted on.
