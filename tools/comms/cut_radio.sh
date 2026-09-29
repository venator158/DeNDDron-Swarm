#!/usr/bin/env bash
# Cut a container's radio.
#   cut_radio.sh <container> <docker network> <radio subnet> [netem|disconnect] [duration]
#   netem:      100% packet loss on the radio interface for <duration> (default 30s); the interface stays up,
#               like jamming. Uses the gaiaadm/pumba and gaiadocker/iproute2 images.
#   disconnect: remove the container from the network (docker network connect to restore).
set -eu
C=$1; NET=$2; SUBNET=$3; MODE=${4:-netem}; DUR=${5:-30s}
if [[ "$MODE" == netem ]]; then
  IFACE=$(docker exec "$C" python -c "
import ipaddress, os, socket, fcntl, struct
net = ipaddress.ip_network('$SUBNET')
for n in sorted(os.listdir('/sys/class/net')):
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        ip = socket.inet_ntoa(fcntl.ioctl(s.fileno(), 0x8915, struct.pack('256s', n[:15].encode()))[20:24])
    except OSError:
        continue
    if ipaddress.ip_address(ip) in net:
        print(n); break")
  docker run -d --rm -v /var/run/docker.sock:/var/run/docker.sock gaiaadm/pumba netem --duration "$DUR" \
    --tc-image gaiadocker/iproute2 --interface "$IFACE" loss --percent 100 "$C" >/dev/null
  echo "$(date +%T) radio of $C jammed ($IFACE, 100% loss for $DUR)"
else
  docker network disconnect "$NET" "$C"
  echo "$(date +%T) radio of $C disconnected"
fi
