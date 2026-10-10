#!/usr/bin/env bash
# QUESTION: with the proximity fuze on, what does an unsynchronized skewed clock cost,
#           and which sync mode fixes it?
# RESULT (2026-10-10, 3 runs, 8 drones): `none` 3/4 in every run (the fuze arms for
#           +-2 s in SYNCHRONIZED time, so a drone 3 s out never detects); ttg, master
#           and consensus 4/4 in every run.  Only consensus's error bound is honest
#           (covers 98.5-99.8% vs master 82-92%; ttg has none).
# Read: destroyed, fuze_no_detection, sync_err_s_mean/max, sync_bound_coverage,
#       drone_rx_msgs_per_s (the O(N^2) beacon cost of consensus).
#
#   bash experiments/clock_sync.sh
#   PERFECT=1 bash experiments/clock_sync.sh   # control arm: perfect clocks

source "$(dirname "${BASH_SOURCE[0]}")/_common.sh"
EXPERIMENT="clock_sync"
CLAIM="none/ttg/master/consensus under skewed clocks, fuze on"
check_images

# "Skewed" = each drone draws a drift in +-500 ppm and an offset in +-3 s,
# reproducibly from CLOCK_SEED and its own id.
if [[ "${PERFECT:-0}" == "1" ]]; then
  SKEW=(); TAG="perfect"
else
  SKEW=(CLOCK_DRIFT_SPREAD_PPM=500 CLOCK_OFFSET_SPREAD_S=3 CLOCK_SEED=1); TAG="skewed"
fi

for mode in none ttg master consensus; do
  run_sweep "results/clock_sync/${TAG}_$mode" \
    python3 tools/comms/degradation_sweep.py \
      --rtf 3 --drones 8 --threats 4 --seed "$SEED" \
      --profiles default --conditions baseline= \
      --repeats "$REPEATS" --parallel "$PARALLEL" \
      --env "${SKEW[@]}" "CLOCK_SYNC=$mode" \
      --out "results/clock_sync/${TAG}_$mode"
done
