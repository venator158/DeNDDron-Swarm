#!/usr/bin/env bash
# CLAIM: attribute the legacy-vs-current difference to its three causes.
#
# Why: the earlier legacy run changed three settings at once (LOCALIZATION=truth,
# PERCEPTION=lidar, NO_FLY_RADIUS_M=0), so its result could not be pinned on any one
# of them.  Here each setting is changed alone, from the current defaults, with the
# full legacy and full current setups as the two ends.
# Read: det_miss_m_mean/max, min_ship_range_m, intercept_range_m_mean, destroyed,
#       award_latency_ms_mean, collisions, close_calls, drone_cpu_pct.
#
#   bash experiments/legacy_split.sh
#   REPEATS=5 bash experiments/legacy_split.sh

source "$(dirname "${BASH_SOURCE[0]}")/_common.sh"
EXPERIMENT="legacy_split"
CLAIM="one setting at a time between current and legacy architecture"
check_images

# arm name -> --env settings (empty = current defaults)
ARMS=(
  "current|"
  "loc_truth|LOCALIZATION=truth"
  "perc_lidar|PERCEPTION=lidar"
  "nofly_off|NO_FLY_RADIUS_M=0"
  "legacy|LOCALIZATION=truth PERCEPTION=lidar NO_FLY_RADIUS_M=0"
)

for arm in "${ARMS[@]}"; do
  name="${arm%%|*}"; envs="${arm#*|}"
  env_args=(); [[ -n "$envs" ]] && env_args=(--env $envs)
  run_sweep "results/legacy_split/$name" \
    python3 tools/comms/degradation_sweep.py \
      --rtf 3 --drones 8 --threats 4 --seed "$SEED" \
      --profiles default --conditions baseline= \
      --repeats "$REPEATS" --parallel "$PARALLEL" \
      "${env_args[@]}" \
      --out "results/legacy_split/$name"
done
echo "LEGACY_SPLIT DONE"
