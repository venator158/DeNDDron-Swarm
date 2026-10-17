#!/usr/bin/env bash
# QUESTION: do the README's sensing and ship-link results hold up when repeated?
#           They are single runs from 2026-10-03/04 (uwb_short: tens of metres of error
#           while claiming sub-metre accuracy; ship_loss30 4/8 before the fix).
# RESULT (2026-10-17, 3 runs): defaults: 18 of 19 conditions 8/8 every run, incl.
#           ship_loss30 and combined (were 4/8); radar_noise 7/8 every run (worst miss
#           8.4 m, 2-3 fuze false triggers).  gnss0: every UWB condition 8/8; worst
#           single-moment errors 16 m (uwb_noise) and 22 m (uwb_nlos) cost nothing.
#
# Two arms:
#   defaults  every sensing_sweep.py condition with the shipped configuration (GNSS on).
#   gnss0     the UWB conditions with GNSS off.  GNSS takes over when UWB fails, so with it
#             on these cells measure the fallback, not the localization; the 2026-10-03
#             UWB results predate GNSS.  uwb_short with GNSS=0 is already repeated by
#             experiments/rdl_vs_coop.sh (the coop arm), so it is left out here.
# Read: destroyed, det_miss_m_mean/max, loc_err_p95_m, loc_err_max_m, loc_nees_mean,
#       loc_relocks, fuze_false_triggers, min_separation_m.
#
#   bash experiments/sensing.sh                  # both arms
#   ARMS=defaults bash experiments/sensing.sh
#   ARMS=gnss0 bash experiments/sensing.sh
#   PARALLEL=2 bash experiments/sensing.sh       # two 15-drone swarms at once (~3 GB)

source "$(dirname "${BASH_SOURCE[0]}")/_common.sh"
EXPERIMENT="sensing"
CLAIM="sensing and ship-link degradation, repeated"
check_images
need_arp 15
ARMS="${ARMS:-all}"

if [[ "$ARMS" == "all" || "$ARMS" == "defaults" ]]; then
  run_sweep "results/sensing/defaults" \
    python3 tools/comms/sensing_sweep.py \
      --rtf 3 --drones 15 --threats 8 --seed "$SEED" \
      --repeats "$REPEATS" --parallel "$PARALLEL" \
      --out "results/sensing/defaults"
fi

if [[ "$ARMS" == "all" || "$ARMS" == "gnss0" ]]; then
  run_sweep "results/sensing/gnss0" \
    python3 tools/comms/sensing_sweep.py \
      --rtf 3 --drones 15 --threats 8 --seed "$SEED" \
      --conditions baseline uwb_noise uwb_nlos uwb_dropout uwb_jam_local uwb_jam_all \
      --repeats "$REPEATS" --parallel "$PARALLEL" \
      --env GNSS=0 \
      --out "results/sensing/gnss0"
fi
