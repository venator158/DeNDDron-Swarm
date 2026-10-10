#!/usr/bin/env bash
# QUESTION: do the two 8-drone stress findings hold at 50 drones?  Where is the
#           bandwidth cliff when the swarm (and its background traffic) is 6x larger,
#           and does an unsynchronized skewed clock still cost kills in a crowd?
# RESULT (2026-10-10, 3 runs, 50 drones, ~25 threats): no impairment 23-24/24.
#           256 kbit/s 3-5/25, 128 kbit/s 2-4/25, 64 kbit/s 0-6: the cliff is far above
#           256 kbit/s (8 drones: ~40).  Skewed + none 8-9/25 (8 drones: 3/4);
#           consensus 24/24 every run at ~5x the messages per drone.
#
# Why: at 8 drones the cliff sits between 48 and 32 kbit/s, just below the 36.4 kbit/s
# of routine traffic, and skewed clocks without sync lose 1 of 4 threats in every run
# (experiments/radio_degradation.sh, experiments/clock_sync.sh).  Both are measured on
# a small swarm; background traffic grows with N, so the cliff should move up.
# Read: destroyed, failed, never_fully_assigned, reannounces, fuze_no_detection,
#       sync_err_s_mean, close_calls, rtf_measured.
#
#   bash experiments/scale_stress.sh                 # radio and clock arms
#   ARMS=radio bash experiments/scale_stress.sh
#   ARMS=clock bash experiments/scale_stress.sh
#
# 50 drones at real time (50:1, as every 50-drone result in the README), one swarm at
# a time: a 50-drone swarm takes ~5-6 GB, and 100 drones once crashed a 15 GB host.

source "$(dirname "${BASH_SOURCE[0]}")/_common.sh"
EXPERIMENT="scale_stress"
CLAIM="radio and clock stress at 50 drones"
check_images
need_arp 50
ARMS="${ARMS:-all}"
PARALLEL=1

# --- radio: baseline, then bandwidth steps down to the 8-drone safe point ----
# Real time, so netem is applied as written.
if [[ "$ARMS" == "all" || "$ARMS" == "radio" ]]; then
  run_sweep "results/scale_stress/radio" \
    python3 tools/comms/scaling_sweep.py \
      --sizes 50:1 --auto-approve --seed "$SEED" \
      --conditions baseline= bw256k="rate 256kbit" bw128k="rate 128kbit" bw64k="rate 64kbit" \
      --repeats "$REPEATS" \
      --out "results/scale_stress/radio"
fi

# --- clocks: skewed as in clock_sync.sh, no sync vs consensus -----------------
SKEW=(CLOCK_DRIFT_SPREAD_PPM=500 CLOCK_OFFSET_SPREAD_S=3 CLOCK_SEED=1)
if [[ "$ARMS" == "all" || "$ARMS" == "clock" ]]; then
  for sync in none consensus; do
    run_sweep "results/scale_stress/skewed_$sync" \
      python3 tools/comms/scaling_sweep.py \
        --sizes 50:1 --auto-approve --seed "$SEED" \
        --repeats "$REPEATS" \
        --env "${SKEW[@]}" "CLOCK_SYNC=$sync" \
        --out "results/scale_stress/skewed_$sync"
  done
fi
