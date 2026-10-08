#!/usr/bin/env bash
# CLAIM: tracking cross-covariances (RDL, Luft et al. 2018) keeps peer-only
#        localization consistent where coop becomes overconfident.
#
# Why: rdl.py is unit-tested but has never run live.  uwb_short is the condition it
# was built for: with 80 m UWB range the anchors are out of reach, and coop reached
# 16 m error while claiming sub-metre accuracy (NEES ~50).
# Read: loc_nees_mean (2.0 is honest), loc_err_p95_m/max_m, loc_relocks, destroyed.
#
#   bash experiments/rdl_vs_coop.sh
#   CONDITIONS="uwb_short uwb_jam_all" bash experiments/rdl_vs_coop.sh

source "$(dirname "${BASH_SOURCE[0]}")/_common.sh"
EXPERIMENT="rdl_vs_coop"
CLAIM="consistent peer fusion (rdl) vs variance-floored peer fusion (coop)"
check_images
need_arp 15
CONDITIONS="${CONDITIONS:-uwb_short}"
# GNSS fallback (on by default since 2026-10-04) steps in when the anchors are out of
# range, so with it on, coop is never left on peers alone and the failure RDL was built
# for does not occur.  GNSS=0 isolates peer fusion; GNSS=1 is the as-deployed default.
GNSS="${GNSS:-0}"

for mode in coop rdl anchors; do
  run_sweep "results/rdl_vs_coop/gnss${GNSS}_$mode" \
    python3 tools/comms/sensing_sweep.py \
      --rtf 3 --drones 15 --threats 8 --seed "$SEED" \
      --conditions $CONDITIONS \
      --repeats "$REPEATS" --parallel "$PARALLEL" \
      --env "LOCALIZATION=$mode" "GNSS=$GNSS" \
      --out "results/rdl_vs_coop/gnss${GNSS}_$mode"
done

# anchors = the floor (no peer fusion at all): if coop and rdl do not beat it
# under uwb_short, peer fusion is not earning its complexity.
