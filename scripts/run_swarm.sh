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
THREAT_WAVES=0
THREAT_INTERVAL_S=15
THREATS_PER_WAVE=4

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
      THREAT_WAVES="$2"
      shift 2
      ;;
    --threat-interval)
      THREAT_INTERVAL_S="$2"
      shift 2
      ;;
    --threats-per-wave)
      THREATS_PER_WAVE="$2"
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
if [[ "$THREAT_WAVES" -gt 0 ]]; then
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

if [[ "$THREAT_WAVES" -gt 0 ]]; then
  export COMPOSE_PROFILES=threats THREAT_WAVES THREAT_INTERVAL_S THREATS_PER_WAVE THREAT_SEED="$SEED"
  echo "Threat dispatcher enabled: ${THREAT_WAVES} waves of up to ${THREATS_PER_WAVE} threats every ${THREAT_INTERVAL_S}s"
fi

compose_cmd=(docker compose --env-file .swarm.env up --scale agent="${AGENT_COUNT}")
compose_cmd+=("${COMPOSE_ARGS[@]}")

"${compose_cmd[@]}"
