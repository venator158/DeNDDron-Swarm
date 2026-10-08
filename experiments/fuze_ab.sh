#!/usr/bin/env bash
# CLAIM: the proximity fuze roughly halves miss distance, and makes clock sync mandatory.
#
# Why: every figure attributed to the fuze so far comes from before/after comparisons
# across runs that also differed in other ways.  This is the controlled version.
# Read: det_miss_m_mean/max, det_timing_err_s_mean, fuze_no_detection, destroyed.
#
#   bash experiments/fuze_ab.sh              # 3 repeats per arm
#   REPEATS=5 bash experiments/fuze_ab.sh
#   ARMS=small bash experiments/fuze_ab.sh   # skip the 50-drone half (~35 min -> ~10)

source "$(dirname "${BASH_SOURCE[0]}")/_common.sh"
EXPERIMENT="fuze_ab"
CLAIM="proximity fuze vs timed detonation, everything else held"
check_images
ARMS="${ARMS:-all}"

# --- 8 drones, --rtf 3: the accuracy claim, cheap to repeat ------------------
for fuze in 1 0; do
  run_sweep "results/fuze_ab/n8_fuze$fuze" \
    python3 tools/comms/degradation_sweep.py \
      --rtf 3 --drones 8 --threats 4 --seed "$SEED" \
      --profiles default --conditions baseline= \
      --repeats "$REPEATS" --parallel "$PARALLEL" \
      --env "FUZE=$fuze" \
      --out "results/fuze_ab/n8_fuze$fuze"
done

# --- 50 drones, real time: does the accuracy gain survive a crowd? -----------
if [[ "$ARMS" == "all" ]]; then
  need_arp 50
  for fuze in 1 0; do
    run_sweep "results/fuze_ab/n50_fuze$fuze" \
      python3 tools/comms/scaling_sweep.py \
        --sizes 50:1 --auto-approve --seed "$SEED" \
        --repeats "$REPEATS" \
        --env "FUZE=$fuze" \
        --out "results/fuze_ab/n50_fuze$fuze"
  done
fi

# --- firing rule: closest approach vs a plain range threshold ----------------
# The cpa detector is claimed to beat radius by 0.2-0.3 s (about 0.8 m at 2.5 m/s).
for rule in cpa radius; do
  run_sweep "results/fuze_ab/fire_$rule" \
    python3 tools/comms/degradation_sweep.py \
      --rtf 3 --drones 8 --threats 4 --seed "$SEED" \
      --profiles default --conditions baseline= \
      --repeats "$REPEATS" --parallel "$PARALLEL" \
      --env FUZE=1 "FUZE_FIRE=$rule" \
      --out "results/fuze_ab/fire_$rule"
done
