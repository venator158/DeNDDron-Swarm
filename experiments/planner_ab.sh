#!/usr/bin/env bash
# CLAIM: ORCA's greedy projection dead-ends at zero velocity in crowds; APF with
#        goal-proximity repulsion fade does not.
#
# Why: this is asserted in the README with no measurement behind it.  The failure is
# predicted to appear in dense swarms, so it is run at two sizes.
# Read: missed_slots, destroyed, min_separation_m, close_calls, det_miss_m_mean.
#
# NOTE the planner is a native sweep flag (--algorithm), not an --env pass-through:
# ALGORITHM is only a shell variable inside run_swarm.sh.
#
#   bash experiments/planner_ab.sh
#   ARMS=small bash experiments/planner_ab.sh   # 8 drones only

source "$(dirname "${BASH_SOURCE[0]}")/_common.sh"
EXPERIMENT="planner_ab"
CLAIM="APF vs ORCA, same scenario, same safety envelope"
check_images
ARMS="${ARMS:-all}"

for algo in apf orca; do
  run_sweep "results/planner_ab/n8_$algo" \
    python3 tools/comms/degradation_sweep.py \
      --rtf 3 --drones 8 --threats 4 --seed "$SEED" \
      --profiles default --conditions baseline= \
      --repeats "$REPEATS" --parallel "$PARALLEL" \
      --algorithm "$algo" \
      --out "results/planner_ab/n8_$algo"
done

if [[ "$ARMS" == "all" ]]; then
  need_arp 50
  for algo in apf orca; do
    run_sweep "results/planner_ab/n50_$algo" \
      python3 tools/comms/scaling_sweep.py \
        --sizes 50:1 --auto-approve --seed "$SEED" \
        --repeats "$REPEATS" \
        --algorithm "$algo" \
        --out "results/planner_ab/n50_$algo"
  done
fi
