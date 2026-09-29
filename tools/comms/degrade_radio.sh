#!/usr/bin/env bash
# Degrade (or restore) the radio of running swarm containers with tc netem, one qdisc per radio
# interface, so impairments can be combined. The onboard (simulator) network is not touched.
#
#   degrade_radio.sh apply "<netem args>" [container...]
#   degrade_radio.sh clear [container...]
#
#   netem args, e.g.:  "loss 10%"   "delay 200ms 50ms"   "rate 64kbit"   "loss 5% delay 100ms rate 256kbit"
#   containers default to every drone and the ship of the running swarm.
#   env: RADIO_SUBNET (default 172.21.0.0/16)
set -eu
MODE=$1; shift
ARGS=""
if [[ "$MODE" == apply ]]; then ARGS=$1; shift; fi
SUBNET=${RADIO_SUBNET:-172.21.0.0/16}
if [[ $# -gt 0 ]]; then
  CONTAINERS=("$@")
else
  mapfile -t CONTAINERS < <(docker ps --format '{{.Names}}' | grep -E '^denddron-swarm-agent-[0-9]+$|^ship$' | sort)
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
    # shellcheck disable=SC2086
    docker run --rm --net "container:$c" --cap-add NET_ADMIN gaiadocker/iproute2 \
      qdisc replace dev "$IF" root netem $ARGS
  else
    docker run --rm --net "container:$c" --cap-add NET_ADMIN gaiadocker/iproute2 \
      qdisc del dev "$IF" root 2>/dev/null || true
  fi
done
echo "$(date +%T) radio $MODE${ARGS:+ ($ARGS)} on ${#CONTAINERS[@]} containers"
