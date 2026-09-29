"""Zenoh session factories for the two links every node can have.

- **Onboard bus** (client -> Zenoh router on the sim network): the simulator
  stand-in for a drone's own sensors and actuators, plus the ship's radar
  truth.  It is not "communications" and must keep working while the radio
  is degraded.
- **Radio** (peer-to-peer on the radio network, no router): everything the
  swarm and the ship say to each other.  Peers find each other by multicast
  scouting on the radio interface and connect directly.  Degrading this
  network (tc netem on the radio interface) degrades only swarm comms.
  Nodes run it in a separate process (radio_process.py).
"""

import fcntl
import ipaddress
import json
import os
import socket
import struct

import zenoh

SIOCGIFADDR = 0x8915


def _iface_ipv4(name: str):
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        packed = fcntl.ioctl(s.fileno(), SIOCGIFADDR, struct.pack("256s", name[:15].encode()))
        return socket.inet_ntoa(packed[20:24])
    except OSError:
        return None
    finally:
        s.close()


def find_interface(subnet: str):
    """Return (interface name, IPv4) of the local interface inside ``subnet``."""
    net = ipaddress.ip_network(subnet)
    for name in sorted(os.listdir("/sys/class/net")):
        ip = _iface_ipv4(name)
        if ip and ipaddress.ip_address(ip) in net:
            return name, ip
    raise RuntimeError(f"no interface in radio subnet {subnet}")


def open_radio(subnet: str = None, routing_mode: str = None, lease_ms: int = None) -> zenoh.Session:
    """Peer-mode session bound to the radio network only.

    Agents and the ship open this inside a RadioProcess (radio_process.py), so that
    no radio problem can freeze their onboard link.

    - protocol (RADIO_PROTO, udp): see the comment below.
    - lease (RADIO_LEASE_MS, 2 s): how soon a lost peer is noticed.
    - open/accept timeout (RADIO_OPEN_TIMEOUT_MS, 1 s): bounds connection attempts.
    - routing_mode (RADIO_ROUTING) only exists before Zenoh 1.1; newer versions
      always route peer-to-peer and ignore it.
    """
    subnet = subnet or os.environ.get("RADIO_SUBNET", "172.21.0.0/16")
    routing_mode = routing_mode or os.environ.get("RADIO_ROUTING", "linkstate")
    lease_ms = lease_ms or int(os.environ.get("RADIO_LEASE_MS") or 2000)
    open_ms = int(os.environ.get("RADIO_OPEN_TIMEOUT_MS") or 1000)
    iface, ip = find_interface(subnet)
    conf = zenoh.Config()
    conf.insert_json5("mode", '"peer"')
    # Listen only on the radio address so every peer link runs over the radio network.
    # RADIO_PROTO: "udp" (default), "tcp", or "tcp,udp". With any TCP link, jamming one peer stalled
    # other peers' traffic (and the jammed node's other Zenoh sessions) for ~10 s inside Zenoh's TCP
    # link handling; over UDP that does not happen. UDP means best-effort delivery: the protocol
    # recovers from lost orders/bids/awards by re-announcement and conflict repair.
    protos = [p.strip() for p in (os.environ.get("RADIO_PROTO") or "udp").split(",") if p.strip()]
    conf.insert_json5("listen/endpoints", json.dumps([f"{p}/{ip}:0" for p in protos]))
    if len(protos) > 1:
        conf.insert_json5("transport/unicast/max_links", str(len(protos)))
    conf.insert_json5("scouting/multicast/enabled", "true")
    conf.insert_json5("scouting/multicast/interface", json.dumps(iface))
    conf.insert_json5("transport/link/tx/lease", str(int(lease_ms)))
    for key, value in (("routing/peer/mode", json.dumps(routing_mode)),
                       ("transport/unicast/open_timeout", str(open_ms)),
                       ("transport/unicast/accept_timeout", str(open_ms))):
        try:
            conf.insert_json5(key, value)
        except zenoh.ZError:
            pass   # key not supported by this Zenoh version
    # Extra Zenoh settings for experiments, e.g. RADIO_ZENOH_CONFIG='{"routing/interests/timeout": 1000}'.
    # Unknown keys fail loudly here, so a typo cannot silently change nothing.
    for key, value in json.loads(os.environ.get("RADIO_ZENOH_CONFIG") or "{}").items():
        conf.insert_json5(key, json.dumps(value))
    return zenoh.open(conf)


def open_onboard(router_locator: str = None) -> zenoh.Session:
    """Client session to the simulator's router (onboard sensors/actuators)."""
    conf = zenoh.Config()
    conf.insert_json5("mode", '"client"')
    conf.insert_json5("scouting/multicast/enabled", "false")
    if router_locator:
        conf.insert_json5("connect/endpoints", json.dumps([router_locator]))
    return zenoh.open(conf)
