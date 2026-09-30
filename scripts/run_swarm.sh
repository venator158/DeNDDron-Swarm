#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT_DIR"

# Defaults
AGENT_COUNT=3
SEED=42
ALGORITHM="apf"
COMPOSE_ARGS=()
BUILD_FLAG=""
MAX_THREATS=0
THREAT_INTERVAL_S=30
THREAT_FIRST_S=20
INSTANCE="${SWARM_INSTANCE:-0}"

# Parse positional argument for AGENT_COUNT if provided as the very first argument (legacy support)
if [[ $# -gt 0 && ! "$1" =~ ^- ]]; then
  AGENT_COUNT="$1"
  shift
fi

while [[ $# -gt 0 ]]; do
  case "$1" in
    --seed)
      SEED="$2"
      shift 2
      ;;
    --algorithm)
      ALGORITHM="$2"
      shift 2
      ;;
    --threats)
      MAX_THREATS="$2"
      shift 2
      ;;
    --threat-interval)
      THREAT_INTERVAL_S="$2"
      shift 2
      ;;
    --first-threat)
      THREAT_FIRST_S="$2"
      shift 2
      ;;
    --radio-qos)
      export RADIO_QOS="$2"
      shift 2
      ;;
    --maneuver-p)
      # Probability that a threat turns once mid-flight (the ship updates the job topic).
      export THREAT_MANEUVER_P="$2"
      shift 2
      ;;
    --instance)
      # Run as instance K (0 = default): own project, names, ports, subnets and config,
      # so several swarms can run side by side (scripts/swarm_instance.py).
      INSTANCE="$2"
      shift 2
      ;;
    --rtf)
      # Real-time factor: run the simulation (and the swarm's protocol clocks) this many times faster.
      export SIM_RTF="$2"
      shift 2
      ;;
    --clock-drift|--clock-drift-spread|--clock-offset|--clock-offset-spread|--clock-jitter|--clock-seed|--clock-sync|--ship-clock-drift|--ship-clock-offset)
      # Distributed clock (README "Distributed clock"): drift in ppm, offsets/jitter in s, sync mode.
      case "$1" in
        --clock-drift) export CLOCK_DRIFT_PPM="$2" ;;
        --clock-drift-spread) export CLOCK_DRIFT_SPREAD_PPM="$2" ;;
        --clock-offset) export CLOCK_OFFSET_S="$2" ;;
        --clock-offset-spread) export CLOCK_OFFSET_SPREAD_S="$2" ;;
        --clock-jitter) export CLOCK_JITTER_S="$2" ;;
        --clock-seed) export CLOCK_SEED="$2" ;;
        --clock-sync) export CLOCK_SYNC="$2" ;;
        --ship-clock-drift) export SHIP_CLOCK_DRIFT_PPM="$2" ;;
        --ship-clock-offset) export SHIP_CLOCK_OFFSET_S="$2" ;;
      esac
      shift 2
      ;;
    --build)
      BUILD_FLAG="--build"
      COMPOSE_ARGS+=("$1")
      shift
      ;;
    *)
      COMPOSE_ARGS+=("$1")
      shift
      ;;
  esac
done

# The radio is a full mesh, so every node needs an ARP entry for every other node, and Linux keeps
# one ARP table for all containers on the host. Past gc_thresh3 new nodes cannot reach anyone
# (at the default 1024 the mesh stops at ~32 drones; kernel log: "neighbor table overflow").
ARP_MAX=$(cat /proc/sys/net/ipv4/neigh/default/gc_thresh3 2>/dev/null || echo 0)
ARP_NEED=$(( (AGENT_COUNT + 1) * AGENT_COUNT + 2 * (AGENT_COUNT + 4) ))
if [[ "$ARP_MAX" -gt 0 && "$ARP_NEED" -gt "$ARP_MAX" ]]; then
  echo "WARNING: ${AGENT_COUNT} drones need ~${ARP_NEED} ARP entries; this host allows ${ARP_MAX}." >&2
  echo "         Drones beyond ~32 will not join the radio. Raise the limit (needs sudo):" >&2
  echo "         sudo sysctl -w net.ipv4.neigh.default.gc_thresh1=4096 net.ipv4.neigh.default.gc_thresh2=8192 net.ipv4.neigh.default.gc_thresh3=16384" >&2
fi

GEN_ARGS=(--agents "$AGENT_COUNT" --seed "$SEED" --algorithm "$ALGORITHM")
# In the defence scenario drones hold station until tasked, so no static goals.
if [[ "$MAX_THREATS" -gt 0 ]]; then
  GEN_ARGS+=(--no-goals)
fi
eval "$(python3 scripts/swarm_instance.py "$INSTANCE")"
mkdir -p "$SWARM_CONFIG_DIR"
GEN_ARGS+=(--output "$SWARM_CONFIG_DIR/swarm_runtime.json" --env-output "$ENV_FILE")
python3 scripts/generate_swarm_config.py "${GEN_ARGS[@]}"
# Instance settings go into the env file too, so `docker compose --env-file <it> down` targets this instance.
python3 scripts/swarm_instance.py "$INSTANCE" | grep -v '^ENV_FILE=' >> "$ENV_FILE"

# Reset per-container ID assignment so replicas get clean drone_1..drone_N IDs.
rm -f "$SWARM_CONFIG_DIR/agent_registry.json" "$SWARM_CONFIG_DIR/agent_registry.json.lock"

if [[ ! -f "$ENV_FILE" ]]; then
  echo "Failed to generate $ENV_FILE" >&2
  exit 1
fi

# shellcheck disable=SC1091
source "$ENV_FILE"

echo "Starting swarm with AGENT_COUNT=${AGENT_COUNT}, SEED=${SEED}, ALGORITHM=${ALGORITHM}"

if [[ -n "$BUILD_FLAG" ]]; then
  echo "Building base image first to satisfy Docker build dependencies..."
  docker compose build denddron_base
fi

export MAX_THREATS THREAT_INTERVAL_S THREAT_FIRST_S THREAT_SEED="$SEED"
if [[ "$MAX_THREATS" -gt 0 ]]; then
  echo "Ship radar: ${MAX_THREATS} threats, first after ${THREAT_FIRST_S}s, then every ~${THREAT_INTERVAL_S}s (sim time)"
fi
if [[ -n "${SIM_RTF:-}" && "${SIM_RTF}" != "1" ]]; then
  echo "Simulation runs at ${SIM_RTF}x real time"
fi
if [[ -n "${CLOCK_DRIFT_PPM:-}${CLOCK_DRIFT_SPREAD_PPM:-}${CLOCK_OFFSET_S:-}${CLOCK_OFFSET_SPREAD_S:-}${CLOCK_JITTER_S:-}${SHIP_CLOCK_DRIFT_PPM:-}${SHIP_CLOCK_OFFSET_S:-}" || "${CLOCK_SYNC:-none}" != "none" ]]; then
  echo "Clocks: drift ${CLOCK_DRIFT_PPM:-0}±${CLOCK_DRIFT_SPREAD_PPM:-0} ppm, offset ${CLOCK_OFFSET_S:-0}±${CLOCK_OFFSET_SPREAD_S:-0} s, jitter ${CLOCK_JITTER_S:-0} s, ship ${SHIP_CLOCK_DRIFT_PPM:-0} ppm / ${SHIP_CLOCK_OFFSET_S:-0} s, sync ${CLOCK_SYNC:-none}, seed ${CLOCK_SEED:-0}"
fi
if [[ "$INSTANCE" != "0" ]]; then
  echo "Instance ${INSTANCE}: project ${COMPOSE_PROJECT_NAME}, env file ${ENV_FILE}"
fi
echo "Operator dashboard: http://localhost:${DASHBOARD_PORT}"

compose_cmd=(docker compose --env-file "$ENV_FILE" up --scale agent="${AGENT_COUNT}")
compose_cmd+=("${COMPOSE_ARGS[@]}")

"${compose_cmd[@]}"
