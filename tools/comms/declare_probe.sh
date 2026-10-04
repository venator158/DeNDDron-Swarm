#!/usr/bin/env bash
# Zenoh declaration probe (declare_probe.py): two peers on a scratch network, one side's UDP impaired.
#   declare_probe.sh late_sub  [netem]   impair SUB's egress (declarations), default "loss 30%"
#   declare_probe.sh fresh_pub [netem]   impair PUB's egress (data)
#   declare_probe.sh both      [netem]   impair both (as the ship link in the sensing sweep)
# Uses the agent image (its Zenoh version); env TRIALS, WATCH_S.
set -eu
TEST=$1; NETEM=${2:-loss 30%}
HERE=$(cd "$(dirname "$0")" && pwd); REPO=$(cd "$HERE/../.." && pwd)
NET=denddron_declare_probe; SUBNET=172.31.0.0/16; PUB_IP=172.31.0.2
IMAGE=${IMAGE:-denddron-swarm-agent}
PTEST=$TEST; [[ "$TEST" == both ]] && PTEST=late_sub
cleanup() { docker rm -f dprobe_pub dprobe_sub >/dev/null 2>&1 || true; docker network rm $NET >/dev/null 2>&1 || true; }
trap cleanup EXIT
cleanup
docker network create --subnet $SUBNET $NET >/dev/null
common=(--network $NET -v "$HERE/declare_probe.py:/app/declare_probe.py:ro" -e RADIO_SUBNET=$SUBNET
        -e TEST=$PTEST -e TRIALS="${TRIALS:-40}" -e WATCH_S="${WATCH_S:-3}" --entrypoint python "$IMAGE")
docker run -d --name dprobe_pub --ip $PUB_IP -e ROLE=pub -e RADIO_LISTEN_PORT=7450 "${common[@]}" -u /app/declare_probe.py >/dev/null
docker run -d --name dprobe_sub -e ROLE=sub -e RADIO_CONNECT=udp/$PUB_IP:7450 "${common[@]}" -u /app/declare_probe.py >/dev/null
sleep 5   # sessions up (no impairment yet)
impair() {
  docker run -i --rm --net "container:$1" --cap-add NET_ADMIN gaiadocker/iproute2 -batch - <<TC
qdisc add dev eth0 root handle 1: prio bands 3 priomap 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0
qdisc add dev eth0 parent 1:3 handle 30: netem $NETEM
filter add dev eth0 parent 1: protocol ip prio 1 u32 match ip protocol 17 0xff flowid 1:3
TC
}
case $TEST in
  late_sub)  impair dprobe_sub ;;
  fresh_pub) impair dprobe_pub ;;
  both)      impair dprobe_sub; impair dprobe_pub ;;
esac
docker wait dprobe_sub >/dev/null
docker logs dprobe_sub 2>&1 | grep '^{'
