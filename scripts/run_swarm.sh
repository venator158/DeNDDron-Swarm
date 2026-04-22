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

python3 scripts/generate_swarm_config.py --agents "$AGENT_COUNT" --seed "$SEED" --algorithm "$ALGORITHM"

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

compose_cmd=(docker compose --env-file .swarm.env up --scale agent="${AGENT_COUNT}")
compose_cmd+=("${COMPOSE_ARGS[@]}")

"${compose_cmd[@]}"
