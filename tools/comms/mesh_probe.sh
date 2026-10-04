#!/usr/bin/env bash
# Radio mesh under loss (mesh_probe.py): PEERS radio peers, every one's UDP impaired after START_S.
#   mesh_probe.sh [netem]        default "loss 30%"
# env: PEERS (8), DURATION (60), IMAGE, and the radio settings (RADIO_PROTO, RADIO_LEASE_MS,
#      RADIO_OPEN_TIMEOUT_MS, RADIO_ZENOH_CONFIG, RADIO_PROCESS).
# Prints per peer: session flaps, live sessions at the end, heartbeat delivery, and late subscriptions
# that heard a sender nothing within WATCH_S (of late_trials = subscriptions x senders); then totals.
set -eu
NETEM=${1:-loss 30%}
HERE=$(cd "$(dirname "$0")" && pwd); REPO=$(cd "$HERE/../.." && pwd)
NET=denddron_mesh_probe; SUBNET=172.31.0.0/16; IMAGE=${IMAGE:-denddron-swarm-agent}
N=${PEERS:-8}; NAMES=$(seq -s, -f "p%g" 1 "$N")
cleanup() { for i in $(seq 1 "$N"); do docker rm -f mprobe_p$i >/dev/null 2>&1 || true; done
            docker network rm $NET >/dev/null 2>&1 || true; }
trap cleanup EXIT
cleanup
docker network create --subnet $SUBNET $NET >/dev/null
for i in $(seq 1 "$N"); do
  ip=172.31.0.$((i + 1))
  extra=(); [[ $i -eq 1 ]] && extra=(-e RADIO_LISTEN_PORT=7450) || extra=(-e RADIO_CONNECT=udp/172.31.0.2:7450)
  docker run -d --name mprobe_p$i --network $NET --ip $ip -e NAME=p$i -e PEER_NAMES=$NAMES -e RADIO_SUBNET=$SUBNET \
    -e DURATION="${DURATION:-60}" -e START_S=8 -e WATCH_S="${WATCH_S:-3}" -e DEAD_LIST="${DEAD_LIST:-}" -e RADIO_PROTO="${RADIO_PROTO:-}" -e RADIO_LEASE_MS="${RADIO_LEASE_MS:-}" \
    -e RADIO_OPEN_TIMEOUT_MS="${RADIO_OPEN_TIMEOUT_MS:-}" -e RADIO_ZENOH_CONFIG="${RADIO_ZENOH_CONFIG:-}" \
    -e RADIO_PROCESS="${RADIO_PROCESS:-}" -e RUST_LOG="${RUST_LOG:-error}" -e PYTHONPATH=/common "${extra[@]}" \
    -v "$REPO/src/common":/common:ro -v "$HERE":/t:ro -w /t --entrypoint python "$IMAGE" -u mesh_probe.py >/dev/null
done
sleep 6
for i in $(seq 1 "$N"); do
  docker run -i --rm --net "container:mprobe_p$i" --cap-add NET_ADMIN gaiadocker/iproute2 -batch - <<TC
qdisc add dev eth0 root handle 1: prio bands 3 priomap 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0
qdisc add dev eth0 parent 1:3 handle 30: netem $NETEM
filter add dev eth0 parent 1: protocol ip prio 1 u32 match ip protocol 17 0xff flowid 1:3
TC
done
for i in $(seq 1 "$N"); do docker wait mprobe_p$i >/dev/null; done
for i in $(seq 1 "$N"); do docker logs mprobe_p$i 2>&1 | grep '^{' || docker logs mprobe_p$i 2>&1 | tail -3; done | tee /dev/stderr | python3 -c "
import sys, json
rs = [json.loads(l) for l in sys.stdin if l.startswith('{')]
print(f'== peers={len(rs)} flaps={sum(r[\"flaps\"] for r in rs)} sessions_end={sum(r[\"sessions_end\"] for r in rs)}/{len(rs)*(len(rs)-1)} '
      f'hb_delivery={sum(r[\"hb_delivery\"] for r in rs)/max(1,len(rs)):.3f} '
      f'late_dead={sum(r[\"late_dead\"] for r in rs)}/{sum(r[\"late_trials\"] for r in rs)} '
      f'hb_dead_pairs={sum(r[\"hb_dead_pairs\"] for r in rs)}')"
