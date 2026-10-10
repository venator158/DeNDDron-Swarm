#!/usr/bin/env bash
# QUESTION: does the protocol hold its kill rate under packet loss and delay, and at
#           what bandwidth does it break?
# RESULT (2026-10-10, 3 runs, 8 drones): loss up to 30% and 200 ms delay cost latency
#           but no threats; 64 and 48 kbit/s 4/4.  Cliff below that: 32 kbit/s 2/4,
#           24 kbit/s 1/4 (was 2.7-3), identical in every run.  Background telemetry
#           alone is 36.4 kbit/s at the ship.
# Read: destroyed, award_latency_ms_mean, reannounces, never_fully_assigned, agreement_mean.
#
#   bash experiments/radio_degradation.sh
#   CELLS=bandwidth bash experiments/radio_degradation.sh    # just the cliff

source "$(dirname "${BASH_SOURCE[0]}")/_common.sh"
EXPERIMENT="radio_degradation"
CLAIM="kill rate vs radio impairment; locate the bandwidth cliff"
check_images
CELLS="${CELLS:-all}"

# netem is applied to every drone's radio and the ship's.  --rtf 3 scales it:
# times are divided by 3 and rates multiplied by 3, so each run is equivalent
# to a real-time run with the nominal impairment.  Loss is a ratio and is left alone.
case "$CELLS" in
  bandwidth) CONDS=(bw48k="rate 48kbit" bw32k="rate 32kbit" bw24k="rate 24kbit") ;;
  loss)      CONDS=(loss10="loss 10%" loss30="loss 30%" loss50="loss 50%") ;;
  *)         CONDS=(baseline= loss10="loss 10%" loss30="loss 30%" \
                    delay200="delay 200ms 50ms" bw64k="rate 64kbit" \
                    bw48k="rate 48kbit" bw32k="rate 32kbit" bw24k="rate 24kbit") ;;
esac

run_sweep "results/radio_degradation/$CELLS" \
  python3 tools/comms/degradation_sweep.py \
    --rtf 3 --drones 8 --threats 4 --seed "$SEED" \
    --profiles default --conditions "${CONDS[@]}" \
    --repeats "$REPEATS" --parallel "$PARALLEL" \
    --out "results/radio_degradation/$CELLS"
