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

The first run builds the images, which takes several minutes. Add `--build` after changing code.

| Option | Default | Meaning |
|---|---|---|
| `N` (first argument) | 3 | number of drones |
| `--threats K` | 0 (radar off) | threats the ship's radar generates; drones get no static goals |
| `--threat-interval S` | 30 | mean sim seconds between detections |
| `--first-threat S` | 20 | sim seconds before the first detection |
| `--algorithm orca\|apf` | `orca` | path planner |
| `--seed S` | 42 | spawn layout and threat scenario |
| `--maneuver-p P` | 0 | probability that a threat turns once mid-flight (`THREAT_MANEUVER_P`) |
| `--rtf K` | 1 | run the simulation K times faster than real time (see [Faster than real time](#faster-than-real-time)) |
| `--radio-qos default\|tuned` | `default` | Zenoh QoS profile for the radio (`RADIO_QOS`) |
| `--build` | off | rebuild the images |

Radar environment variables (ship): `THREAT_TYPES` (`type:level:speed:weight,...`), `DETECT_MIN_M`/`DETECT_MAX_M` (150/190), `MAX_MISS_M` (30), `DEFENDED_RADIUS_M` (45), `KILL_RADIUS_M` (8).

With no `--threats`, drones fly to the static goals in `config/swarm_runtime.json`. That is a basic navigation check.

### Faster than real time

`--rtf K` (`SIM_RTF`) runs the whole system K times faster, so experiments finish sooner. Speeding up Gazebo alone would change the results, because the swarm's protocol timers run on the computer's clock. So everything scales together:

- **Gazebo** steps its 1 ms physics at 1000·K steps per second.
- **The bridge** ticks every 20/K ms, so poses stay at 50 Hz and lidar at 10 Hz per simulated second.
- **Drones** run their control loop at 50·K Hz. The simulator filters each velocity command, so the command rate per simulated second must stay the same for the flight dynamics to match.
- **Protocol timers** (heartbeats, link and roster timeouts, re-announce delay, decision latency, radio rates) use `src/common/simclock.py`. It follows the simulator's actual clock: drones feed it the sim time from their sensor frames, and the ship from `sim/clock`. Between updates it runs at the measured sim speed, so timers stay correct when Gazebo falls behind the target. At 25 drones, Gazebo reached 1.9× against a 2× target, and the ship still received exactly the expected 75 messages per simulated second.
- **Radio impairments** must be scaled by hand: delay ÷ K and rate × K. Loss is unchanged. `degradation_sweep.py --rtf K` does this for you.

Compute metrics (loop timing, CPU) stay in real time. What doesn't scale:
- the computer's own processing latencies;
- Zenoh's internal timers;
- netem impairments, which are scaled by the *target* K, so a simulator that falls behind makes delays slightly longer in simulated terms.

Keep K small, and check `rtf_measured` in sweep results.

Measured at K = 3 with 8 drones:
- Gazebo reaches 2.98× real time.
- Each drone uses about 15% of a CPU core; 8 drones plus Gazebo use about 2.3 of 6 cores.
- A sweep run takes 87 s instead of 201 s.
- Results match real-time runs: decision latency 1030 vs 1015 ms with no impairment, and 1440 vs 1445 ms at 200 ms delay.

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
  - The *radio* runs peer-to-peer over UDP on `radio_net` (Zenoh 1.10.1). Peers find each other by multicast scouting on the radio interface and connect directly. Every drone and the ship run the radio **in a separate OS process** (`src/common/radio_process.py`), so a radio failure cannot freeze flight control or C2.
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
| `swarm/heartbeat/{id}` | radio | drone → ship | `{state, link, pose, threat_id, t_engage, confirmed}` at 2 Hz (drones do not subscribe) |
| `swarm/heartbeat_help/{id}` | radio | drone → drones | the same heartbeat, only while the ship does not hear this drone directly |
| `swarm/heartbeat_relay/{id}` | radio | drone → ship | a peer's help heartbeat, forwarded by drones in the roster |
| `swarm/telemetry/{id}` | radio | drone → ship | instrumentation, 1 Hz |
| `ship/roster` | radio | ship → drones | `{count, members[], relayed[]}` at 1 Hz: drones the ship hears, and which of them only through relays |
| `swarm/threats` | radio | ship → drones | engagement order `{wave_id, threats[{threat_id, type, level, required, location, t_engage}]}` |
| `swarm/bids` | radio | drone → all | `{agent_id, wave_id, costs{threat_id: ETA s}}` |
| `swarm/awards` | radio | drone → all, ship | `{threat_id, agent_id, wave_id, cost, status: engaged\|withdrawn\|missed\|released, slot, t_engage}` |
| `ship/ack/{id}` | radio | ship → drone | `{threat_id, wave_id, accepted, job}`, plus the job state when accepted |
| `ship/jobs/{threat_id}` | radio | ship → the job's drones | `{status, seq, point, t_engage, cpa, t_cpa, track, holders{drone: slot}, n_slots}` at 2 Hz |
| `ship/threat_status` | radio | ship → all | `{threat_id, status}` |

## Engagement protocol

The allocation is decentralized (`src/agent/auction.py`).

1. The ship publishes an engagement order on `swarm/threats`. It carries the engagement point, the detonation time, and `required` drones.
2. Each **free** drone (idle, localized, not waiting on another order) bids its ETA to the point. The ETA uses a trapezoidal speed profile at 4 m/s and 1 m/s², times a 1.25 margin. A drone only bids if it can arrive before the detonation time.
3. After a 1 s bid window (sim time), every drone runs the same deterministic assignment:
   - threats are taken highest level first;
   - each threat gets its `required` fastest drones, ties broken by drone ID;
   - **all or nothing**: a threat that cannot get every drone it needs gets none, and those drones stay free.
4. Each winner is only **tentatively** engaged. It publishes an award (with the order ID), starts flying toward its slot, and subscribes to the job topic `ship/jobs/{threat}`. The drones carry no seeker, so an engaged drone depends on the ship for the target's position; it is not fire-and-forget.
5. **The ship confirms** (`src/common/jobs.py`). It collects the awards for a threat for 0.3 s, then confirms the best bids up to the number of drones still needed, and gives each a slot. It sends each one an ACK on `ship/ack/{drone}`; the rest get a NACK and become free again. The ship hears every drone, so this also settles conflicts between drones that could not hear each other.
   - The radio is best-effort, so each award and withdrawal is sent 3 times (now and on the next two heartbeat ticks, 0.5 s apart). A newer status for the same threat replaces the pending copies. The ship ACKs every copy it gets from a confirmed drone.
   - Until confirmed, a drone still yields to better awards from its peers. Once confirmed, only the ship can release it.
   - No ACK within 3 s (sim) means the drone abandons the job, withdraws and becomes free. A drone never detonates without confirmation.
6. **Job topic.** While a job is active, the ship publishes `ship/jobs/{threat}` at 2 Hz. It carries the latest engagement point and detonation time, the expected CPA, the track, and the confirmed drones with their slots. Being listed there also counts as an ACK, which covers a lost ACK.
   - Drones re-aim on every update. Threats can manoeuvre (`--maneuver-p P`: with probability P a threat turns once, re-aiming past the ship), and the update reaches the drones within one publish.
   - When the threat is resolved, the job says so (3 times), and drones that have not detonated become free again.
   - A confirmed drone that is not listed any more has been released, and becomes free. The ship drops a confirmed drone whose heartbeat shows no job for 2 s, which covers a lost withdrawal.
7. **Re-announcement.** If a threat is left short, the ship re-announces it for the missing drones, at most 3 times and only while there is still time. It counts confirmed, pending and detonated drones, and drones whose latest heartbeat says they are engaged, so a lost award never sends a second drone.
8. At the detonation time, a confirmed drone within 8 m of its slot (the kill radius) detonates:
   - it publishes `sim/detonation`;
   - it despawns;
   - its container stays up but idle, so it is never respawned.

   A drone that is not in position aborts, publishes `missed`, and holds position.

Decision latency, from approval until every drone the threat needs is confirmed, is about 1.4 s: the 1 s bid window, the 0.3 s confirmation window, and radio round trips.

Measured (8 drones, `--rtf 3`):
- **30% loss:** 6 runs, 4/4 destroyed with exactly 4 drones in every run. The ship settled 5 conflicts, where two drones both believed they had won.
- **Manoeuvres:** 3 two-drone threats, each turning once, moved their engagement points by 18, 10 and 22 m. The first two were destroyed. For the third, the point moved 22 m with only 1.7 s more time, so its drones could not reach it; they aborted and stayed alive instead of detonating 25 m away.
- **The same manoeuvre scenario in real time** (with the Gazebo window) gave an identical result: same drones, slots, point shifts and miss distances as the `--rtf 3` run.

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
| Trade-off: over UDP every message is best-effort, so orders, bids and awards can be lost under packet loss. Recovery relies on re-announcement and conflict repair, measured below. | [Radio degradation sweep](#radio-degradation-sweep) |

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

### Scaling

Setup: `scaling_sweep.py`. N drones face N/2 threats (default type mix), detected every 240/N sim seconds, so the load per drone stays about the same. Each size runs at the fastest speed the host could sustain (6 cores, 15 GB). Needs the [ARP limit](#quick-start) raised above ~30 drones.

| Drones | Speed (target → achieved) | Host CPU | Drone CPU per 1× | Gazebo CPU | Worst loop (p99) | Oldest sensor frame | Msgs/s received per drone | Msgs/s at ship | Outcome |
|---|---|---|---|---|---|---|---|---|---|
| 8 | 3 → 2.95× | 1.9 cores | 7.4% | 33% | 22 ms | 30 ms | 13 | 21 | 4/4 destroyed, 4 drones |
| 16 | 3 → 2.72× | 3.5 cores | 8.1% | 37% | 33 ms | 47 ms | 27 | 38 | 8/8 destroyed, 10 drones |
| 25 | 2 → 1.9× | 3.4 cores | 7.3% | 33% | 42 ms | 54 ms | 44 | 60 | 12/12 destroyed, 16 drones |
| 50 | 1 → 0.87× | 3.8 cores | 9.4% | 26% | 137 ms | 378 ms | 86 | 123 | 22/25 destroyed, 38 drones |

All sizes: no re-announces, no conflicts, full agreement on assignments, decision latency ~1.4 s. The 16-drone run predates the sim-following protocol clock.

- **Radio traffic grew with the square of the swarm** while every drone heard every heartbeat: messages per drone doubled when the swarm doubled, about 4,300/s swarm-wide at 50 drones. Heartbeats now go to the ship only. At 50 drones that cut the messages each drone receives from 86 to 4.4 per second. It also cut host CPU from 3.8 to 3.1 cores, worst loop time from 137 to 86 ms and oldest sensor frame from 378 to 172 ms, and 23 of 25 threats were destroyed (was 22).
- **CPU per drone is roughly constant** at 7–9% of a core per 1× of sim speed (the radio process included), rising slightly with the heartbeats received. 50 drones need about 4–5 cores at real time.
- **At 50 drones this host is the limit.** Drones are starved of CPU: loop times reach 137 ms (target 20 ms) and sensor frames 378 ms. The 3 failed threats were each one drone of a multi-drone threat arriving 8.3, 12.9 and 10.8 m from its slot, just outside the 8 m kill radius; the protocol itself had no errors. A bigger host, or fewer processes per drone, is needed for clean 50-drone runs.

### Tools (`tools/comms/`)

| Script | What it does |
|---|---|
| `radio_probe.sh [routing] [netem\|disconnect]` | 4 radio-only peers; cuts one; reports per healthy peer the seconds it heard nobody, the worst gap and the worst process stall |
| `onboard_probe.sh [routing] [netem\|disconnect]` | drone-like processes with both links; cuts one radio; reports the worst onboard gaps of the cut drone and a healthy one. Set `RADIO_PROCESS=1` to use the separate radio process. |
| `cut_radio.sh <container> <network> <subnet> [netem\|disconnect] [duration]` | cuts one container's radio (used by the others) |
| `chaos.py <compose log> [disconnect\|netem]` | during a live run: cuts an idle drone, then the first drone that engages, then restores the idle one |
| `operator_bot.py [reaction_s] [duration_s] [poll_s]` | stand-in operator: approves feasible threats through the dashboard API, most urgent first |
| `degrade_radio.sh apply "<netem args>" \| clear [container...]` | impairs the radio (UDP only) of every drone and the ship: loss, delay, rate, combinable |
| `degradation_sweep.py [--rtf K] [--repeats N] [--profiles ...] [--conditions NAME=NETEM ...]` | the seeded scenario under each impairment and QoS profile; writes `results/<sweep>/results.{csv,md}` with mean ± sd per cell |

All probes accept `IMAGE=...` to test another Zenoh build and pass through the radio settings (`RADIO_PROTO`, `RADIO_PROCESS`, `RADIO_LEASE_MS`, `RADIO_OPEN_TIMEOUT_MS`, `RADIO_ZENOH_CONFIG`, `RADIO_DEBUG`). `radio_probe.sh` takes `PEERS=N`.

## Instrumentation

Every drone sends `swarm/telemetry/{id}` once a second, covering the last second. The dashboard's *Swarm instrumentation* table shows it:

| Field | Meaning |
|---|---|
| `cpu_pct` | process CPU |
| `loop_hz`, `loop_p50_ms`, `loop_p99_ms`, `loop_work_p99_ms`, `overruns` | control loop timing, real time (50·`SIM_RTF` Hz). An overrun is a tick longer than 1.5 × the period. |
| `sensor_age_p50_ms`, `sensor_age_max_ms` | age of the newest onboard sensor frame, sampled every tick |
| `perception_p99_ms`, `planner_p99_ms` | lidar processing per scan; planner per tick |
| `rx_per_s`, `tx_per_s`, `tx_bytes_per_s` | radio messages per topic, and bytes sent, per simulated second |
| `peers_heard`, `voxels` | peers heard from (bids, awards, help) in the last 3 s; voxel map size |

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
| | sim clock extrapolated between Gazebo's 5 Hz stats; pose 50 Hz, lidar 10 Hz (per sim second, at any `SIM_RTF`) | `GazeboSimulator.cpp` (`estimated_sim_time`, `step`) |
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
| | job confirmation (ACK/NACK, slots), job topic, heartbeat reconciliation, threat manoeuvres | `src/common/jobs.py`, `ship.py` (`_arbitrate`, `_publish_jobs`, `_reconcile`, `_maneuver`) |
| | kill assessment, leak/impact/failed | `ship.py` (`_on_detonation`, `_age_tracks`) |
| | instrumentation aggregation, event log (`/state/ship_log.jsonl`), HTTP/SSE API | `ship.py` (`snapshot`, `make_handler`) |
| | operator dashboard (map, heap queue, approval, swarm table, radio, events) | `src/ship/dashboard.html` |
| **Drone** | agent lifecycle, ID allocation, SIGTERM | `src/agent/main.py` |
| | perception: voxel map, occupied index, batched ray tracing, expiry | `src/agent/voxel_map.py`, `agent.py` (`_process_lidar`) |
| | planners (APF, ORCA) | `src/agent/path_planning.py` |
| | 50 Hz control loop, safety envelope, arrival latch | `agent.py` (`_reflex_control_loop`) |
| | sim-time / wall-time handling | `src/agent/timing.py` |
| | protocol clock scaled by the real-time factor (`SIM_RTF`) | `src/common/simclock.py` |
| | bidding (ETA), engagement, slots, detonation / abort | `agent.py` (`_on_threat_wave`, `_calculate_costs`, `_service_auctions`, `_check_engagement`) |
| | ACK wait and timeout, job topic updates, release | `agent.py` (`_on_ack`, `_on_job`, `_apply_job`, `_abandon`) |
| | decentralized all-or-nothing priority assignment, conflict yield | `src/agent/auction.py` |
| | heartbeat, roster check, link state, peer relay | `agent.py` (`_heartbeat_loop`, `_on_roster`, `_relay_unheard_peers`) |
| | telemetry (loop, sensors, perception, planner, CPU, radio) | `src/agent/telemetry.py` |
| **Metrics** | positions, distance, proximity collisions (spatial hash) | `src/metrics/main.py` |
| **Tests / tools** | unit tests and validation scripts | `tests/` (see [Testing](#testing)) |
| | degraded-comms probes, radio cut, chaos, stand-in operator, radio degradation, sweep | `tools/comms/` |

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

Simulation speed (env): `SIM_RTF` (1), set by `--rtf`.

Radio tuning (env): `RADIO_QOS` (`default`; `tuned` = per-topic priorities, see `QOS_PROFILES` in `links.py`), `RADIO_PROTO` (`udp`), `RADIO_LEASE_MS` (2000), `RADIO_OPEN_TIMEOUT_MS` (1000), `RADIO_SUBNET` (`172.21.0.0/16`), `RADIO_ZENOH_CONFIG` (JSON object of extra Zenoh settings, for experiments).

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
- **Best-effort radio.** The radio runs over UDP, so messages can be lost under packet loss; see [Degraded communications](#degraded-communications).
- **The ship is a single point of failure, by design.** It is the only threat sensor and the only source of engagement orders.
- **Shared simulation clock.** Detonation times use the simulator's clock, which every node shares. A distributed clock is future work.
- **Idealized threats.** They fly straight lines at constant speed, apart from at most one optional turn, and a detonation within the kill radius always kills (no kill probability).
- **No re-planning after a manoeuvre.** The job's drones follow the new point, but the ship does not re-check whether they can still make it, or bring in closer free drones.
- **Greedy assignment.** Orders are assigned per order, highest level first, not globally optimized. A drone waiting on one order's result skips any other order that arrives before that result.
- **Planar perception.** The lidar is 2D, and voxels are placed at the drone's own altitude. ORCA uses greedy projection, not a full linear program.
- **No security.** Radio traffic is unauthenticated: a forged order or bid would be acted on.
