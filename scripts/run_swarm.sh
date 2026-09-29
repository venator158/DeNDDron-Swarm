#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT_DIR"

# Defaults
AGENT_COUNT=3
SEED=42
ALGORITHM="orca"
COMPOSE_ARGS=()
BUILD_FLAG=""
MAX_THREATS=0
THREAT_INTERVAL_S=30
THREAT_FIRST_S=20

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

GEN_ARGS=(--agents "$AGENT_COUNT" --seed "$SEED" --algorithm "$ALGORITHM")
# In the defence scenario drones hold station until tasked, so no static goals.
if [[ "$MAX_THREATS" -gt 0 ]]; then
  GEN_ARGS+=(--no-goals)
fi
python3 scripts/generate_swarm_config.py "${GEN_ARGS[@]}"

# Reset per-container ID assignment so replicas get clean drone_1..drone_N IDs.
rm -f config/agent_registry.json config/agent_registry.json.lock

if [[ ! -f .swarm.env ]]; then
  echo "Failed to generate .swarm.env" >&2
  exit 1
fi

# shellcheck disable=SC1091
source .swarm.env

echo "Starting swarm with AGENT_COUNT=${AGENT_COUNT}, SEED=${SEED}, ALGORITHM=${ALGORITHM}"

if [[ -n "$BUILD_FLAG" ]]; then
  echo "Building base image first to satisfy Docker build dependencies..."
  docker compose build denddron_base
fi

export MAX_THREATS THREAT_INTERVAL_S THREAT_FIRST_S THREAT_SEED="$SEED"
if [[ "$MAX_THREATS" -gt 0 ]]; then
  echo "Ship radar: ${MAX_THREATS} threats, first after ${THREAT_FIRST_S}s, then every ~${THREAT_INTERVAL_S}s (sim time)"
fi
echo "Operator dashboard: http://localhost:8080"

compose_cmd=(docker compose --env-file .swarm.env up --scale agent="${AGENT_COUNT}")
compose_cmd+=("${COMPOSE_ARGS[@]}")

"${compose_cmd[@]}"
