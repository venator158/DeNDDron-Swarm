#!/usr/bin/env bash
# Radio mesh probe: 4 radio-only peers (a b c d); cut d's radio and report, for each healthy peer,
# how long it heard nothing from the other healthy peers and the worst process stall.
#
#   tools/comms/radio_probe.sh [routing] [cut]
#     routing: linkstate (default) | peer_to_peer
#     cut:     netem (default, 100% packet loss, interface stays up) | disconnect (interface removed)
#   env: IMAGE (default denddron-swarm-agent), RADIO_LEASE_MS (default 2000), PEERS (default 4, max 26;
#        the last peer is the one cut)
set -u
HERE=$(cd "$(dirname "$0")" && pwd); REPO=$(cd "$HERE/../.." && pwd)
ROUTING=${1:-linkstate}; CUT=${2:-netem}; IMAGE=${IMAGE:-denddron-swarm-agent}
NAMES=($(echo {a..z} | cut -d' ' -f1-${PEERS:-4})); CUT_PEER=${NAMES[-1]}; HEALTHY=("${NAMES[@]:0:${#NAMES[@]}-1}")
cleanup() { for n in {a..z}; do docker rm -f probe_$n >/dev/null 2>&1; done; docker network rm probe_radio >/dev/null 2>&1; }
cleanup; trap cleanup EXIT
docker network create --subnet 172.31.0.0/16 probe_radio >/dev/null
for n in "${NAMES[@]}"; do
  docker run -d --name probe_$n --network probe_radio -e NAME=$n -e DURATION=45 -e SUBNET=172.31.0.0/16 \
    -e RADIO_ROUTING=$ROUTING -e RADIO_LEASE_MS=${RADIO_LEASE_MS:-2000} -e RADIO_OPEN_TIMEOUT_MS=${RADIO_OPEN_TIMEOUT_MS:-} -e RADIO_PROCESS=${RADIO_PROCESS:-} -e RADIO_DEBUG=${RADIO_DEBUG:-} -e RADIO_ZENOH_CONFIG=${RADIO_ZENOH_CONFIG:-} -e RADIO_PROTO=${RADIO_PROTO:-} -e PYTHONPATH=/common \
    -v "$REPO/src/common":/common:ro -v "$HERE":/t:ro -w /t "$IMAGE" python -u radio_probe.py >/dev/null
done
until [[ $(docker logs probe_a 2>&1 | grep -c rx_gap) -ge 3 ]]; do
  docker ps -a --filter name=probe_a --format '{{.Status}}' | grep -q Exited && { docker logs probe_a 2>&1 | tail -5; exit 1; }
  sleep 1
done
sleep 3
"$HERE/cut_radio.sh" probe_$CUT_PEER probe_radio 172.31.0.0/16 "$CUT"
for n in "${HEALTHY[@]}"; do docker wait probe_$n >/dev/null; done
[[ -n "${RADIO_DEBUG:-}" ]] && for n in "${HEALTHY[@]}"; do echo "-- $n child:"; docker logs probe_$n 2>&1 | grep "radio child" | awk '{print}' | sort -t= -k3 -n | tail -2; done
echo "== peers=${#NAMES[@]} routing=$ROUTING cut=$CUT image=$IMAGE"
for n in "${HEALTHY[@]}"; do
  docker logs probe_$n 2>&1 | python3 -c "
import sys, re, ast
worst, silent, stall = 0.0, 0, 0.0
for line in sys.stdin:
    m = re.search(r'stall_max=\s*([\d.]+)ms rx_gap_max=(\{.*\})', line)
    if not m: continue
    g = {k: v for k, v in ast.literal_eval(m.group(2)).items() if k != '$CUT_PEER'}
    stall = max(stall, float(m.group(1)))
    if not g: silent += 1
    else: worst = max(worst, max(g.values()))
print(f'  $n: seconds hearing no healthy peer={silent}, worst gap={worst:.2f}s, worst process stall={stall:.0f}ms')"
done
