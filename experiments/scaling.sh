#!/usr/bin/env bash
# CLAIM: swarm size is limited by drone density on the approach, not by computation.
#        CPU scales linearly; separation degrades superlinearly, pushing the worst
#        miss onto the hard 8 m kill radius.
#
# Why: close calls rose 9.3 -> 48.8 between 50 and 75 drones (5.2x for a 1.5x swarm)
# while host CPU went 5.3 -> 8.2 cores (linear).  At 75 drones, 3 of 4 runs lost a threat.
# Read: destroyed, close_calls, min_separation_m, det_miss_m_max, host_cpu_pct, rtf_measured.
#
#   bash experiments/scaling.sh                 # 8 16 50 75
#   SIZES="50:1 75:1" bash experiments/scaling.sh
#   SYNC=consensus bash experiments/scaling.sh  # the O(N^2) beacon cost at scale
#
# NOTE 100 drones needs ~7 GB for drones alone and made a 15 GB laptop unusable.
# Run it only on a machine with headroom, and not while you need the desktop.

source "$(dirname "${BASH_SOURCE[0]}")/_common.sh"
EXPERIMENT="scaling"
CLAIM="what scales and what does not, from 8 to 75 drones"
check_images

SIZES="${SIZES:-8:3 16:3 50:1 75:1}"
SYNC="${SYNC:-none}"
MAX=$(echo "$SIZES" | tr ' ' '\n' | cut -d: -f1 | sort -n | tail -1)
need_arp "$MAX"

# Real time (:1) for the large sizes on purpose: CPU, loop timing and messages/s stay
# on the real clock, so they are NOT comparable across --rtf values.
run_sweep "results/scaling/sync_$SYNC" \
  python3 tools/comms/scaling_sweep.py \
    --sizes $SIZES --auto-approve --seed "$SEED" \
    --repeats "$REPEATS" \
    --env "CLOCK_SYNC=$SYNC" \
    --out "results/scaling/sync_$SYNC"
