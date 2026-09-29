#!/usr/bin/env bash
# Onboard-stall probe: 3 drone-like processes (x y z) with an onboard link (router, 50 Hz stream) and a
# radio link. Cut x's radio and report the worst onboard receive gaps of x (cut) and y (healthy).
#
#   tools/comms/onboard_probe.sh [routing] [cut]      (same arguments/env as radio_probe.sh)
set -u
HERE=$(cd "$(dirname "$0")" && pwd); REPO=$(cd "$HERE/../.." && pwd)
ROUTING=${1:-linkstate}; CUT=${2:-netem}; IMAGE=${IMAGE:-denddron-swarm-agent}
cleanup() {
  docker rm -f probe_router probe_pub probe_x probe_y probe_z >/dev/null 2>&1
  docker network rm probe_sim probe_radio >/dev/null 2>&1
}
cleanup; trap cleanup EXIT
docker network create --subnet 172.30.0.0/16 probe_sim >/dev/null
docker network create --subnet 172.31.0.0/16 probe_radio >/dev/null
docker run -d --name probe_router --network probe_sim eclipse/zenoh:latest --listen tcp/0.0.0.0:7447 --no-multicast-scouting >/dev/null
sleep 2
run() {  # name role
  docker create --name probe_$1 --network probe_sim -e NAME=$1 -e ROLE=$2 -e RADIO_ROUTING=$ROUTING \
    -e RADIO_LEASE_MS=${RADIO_LEASE_MS:-2000} -e RADIO_OPEN_TIMEOUT_MS=${RADIO_OPEN_TIMEOUT_MS:-} -e RADIO_PROCESS=${RADIO_PROCESS:-} -e PYTHONPATH=/common \
    -v "$REPO/src/common":/common:ro -v "$HERE":/t:ro -w /t "$IMAGE" python -u onboard_probe.py >/dev/null
}
run pub pub; docker start probe_pub >/dev/null
for n in x y z; do run $n drone; docker network connect probe_radio probe_$n; docker start probe_$n >/dev/null; done
until [[ $(docker logs probe_x 2>&1 | grep -c onboard_gap) -ge 5 ]]; do
  docker ps -a --filter name=probe_x --format '{{.Status}}' | grep -q Exited && { docker logs probe_x 2>&1 | tail -5; exit 1; }
  sleep 1
done
"$HERE/cut_radio.sh" probe_x probe_radio 172.31.0.0/16 "$CUT"
for n in x y z; do docker wait probe_$n >/dev/null; done
echo "== routing=$ROUTING cut=$CUT image=$IMAGE"
for n in x y; do
  top=$(docker logs probe_$n 2>&1 | grep -oE "onboard_gap_max= *[0-9.]+" | grep -oE "[0-9.]+$" | sort -n | tail -3 | tr "\n" " ")
  echo "  $n$([[ $n == x ]] && echo ' (radio cut)' || echo ' (healthy)'): top-3 worst onboard gaps (ms): $top"
done
