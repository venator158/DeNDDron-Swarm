#!/usr/bin/env bash
# Degrade (or restore) the radio of running swarm containers with tc netem on each radio interface.
# Only UDP is impaired: the radio is Zenoh over UDP, while TCP on the same interface (Docker forwards
# the dashboard port, :8080, to the ship's radio address) must pass untouched. A prio root sends all
# traffic to an unimpaired band and a filter moves IPv4 UDP to the band with netem. The onboard
# (simulator) network is not touched.
#
#   degrade_radio.sh apply "<netem args>" [container...]
#   degrade_radio.sh clear [container...]
#
#   netem args, e.g.:  "loss 10%"   "delay 200ms 50ms"   "rate 64kbit"   "loss 5% delay 100ms rate 256kbit"
#   containers default to every drone and the ship of the running swarm.
#   env: SWARM_INSTANCE (default 0) picks the swarm; RADIO_SUBNET overrides its radio subnet.
set -eu
MODE=$1; shift
ARGS=""
if [[ "$MODE" == apply ]]; then ARGS=$1; shift; fi
OVERRIDE_SUBNET=${RADIO_SUBNET:-}
eval "$(python3 "$(dirname "$0")/../../scripts/swarm_instance.py" "${SWARM_INSTANCE:-0}")"
SUBNET=${OVERRIDE_SUBNET:-$RADIO_SUBNET}
if [[ $# -gt 0 ]]; then
  CONTAINERS=("$@")
else
  mapfile -t CONTAINERS < <(docker ps --format '{{.Names}}' \
    | grep -E "^${COMPOSE_PROJECT_NAME}-agent-[0-9]+\$|^${NAME_PREFIX}ship\$" | sort)
fi

iface_of() {   # radio interface name inside container $1
  docker exec "$1" python -c "
import ipaddress, os, socket, fcntl, struct
net = ipaddress.ip_network('$SUBNET')
for n in sorted(os.listdir('/sys/class/net')):
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        ip = socket.inet_ntoa(fcntl.ioctl(s.fileno(), 0x8915, struct.pack('256s', n[:15].encode()))[20:24])
    except OSError:
        continue
    if ipaddress.ip_address(ip) in net:
        print(n); break"
}

for c in "${CONTAINERS[@]}"; do
  IF=$(iface_of "$c")
  if [[ "$MODE" == apply ]]; then
    docker run --rm --net "container:$c" --cap-add NET_ADMIN gaiadocker/iproute2 \
      qdisc del dev "$IF" root 2>/dev/null || true
    docker run -i --rm --net "container:$c" --cap-add NET_ADMIN gaiadocker/iproute2 -batch - <<TC
qdisc add dev $IF root handle 1: prio bands 3 priomap 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0
qdisc add dev $IF parent 1:3 handle 30: netem $ARGS
filter add dev $IF parent 1: protocol ip prio 1 u32 match ip protocol 17 0xff flowid 1:3
TC
  else
    docker run --rm --net "container:$c" --cap-add NET_ADMIN gaiadocker/iproute2 \
      qdisc del dev "$IF" root 2>/dev/null || true
  fi
done
echo "$(date +%T) radio $MODE${ARGS:+ ($ARGS)} on ${#CONTAINERS[@]} containers"
