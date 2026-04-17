#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT_DIR"

AGENT_COUNT="${1:-3}"
shift || true

python3 scripts/generate_swarm_config.py --agents "$AGENT_COUNT"

# Reset per-container ID assignment so replicas get clean drone_1..drone_N IDs.
rm -f config/agent_registry.json config/agent_registry.json.lock

if [[ ! -f .swarm.env ]]; then
  echo "Failed to generate .swarm.env" >&2
  exit 1
fi

# shellcheck disable=SC1091
source .swarm.env

echo "Starting swarm with AGENT_COUNT=${AGENT_COUNT}"

BUILD_FLAG=""
if [[ "${SWARM_BUILD:-0}" == "1" ]]; then
  BUILD_FLAG="--build"
fi

for arg in "$@"; do
  if [[ "$arg" == "--build" ]]; then
    BUILD_FLAG="--build"
    break
  fi
done

compose_cmd=(docker compose --env-file .swarm.env up --scale agent="${AGENT_COUNT}")
if [[ -n "$BUILD_FLAG" ]]; then
  compose_cmd+=("$BUILD_FLAG")
fi
compose_cmd+=("$@")

"${compose_cmd[@]}"
