"""Zenoh session factories for the two links every node can have.

- **Onboard bus** (client -> Zenoh router on the sim network): the simulator
  stand-in for a drone's own sensors and actuators, plus the ship's radar
  truth.  It is not "communications" and must keep working while the radio
  is degraded.
- **Radio** (peer-to-peer on the radio network, no router): everything the
  swarm and the ship say to each other.  Peers find each other by multicast
  scouting on the radio interface and connect directly.  Degrading this
  network (tc netem on the radio interface) degrades only swarm comms.
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

    Routing defaults to linkstate: peers relay for each other (multi-hop), and
    losing one peer does not black out traffic between the others, which
    peer_to_peer mode does in Zenoh 1.0.4 when the lost peer's lease expires.
    A short lease makes peers notice a lost peer sooner.

    Known issue (Zenoh 1.0.4): in the process whose radio is lost, the onboard
    session stops delivering for ~10 s once, independent of the lease.
    """
    subnet = subnet or os.environ.get("RADIO_SUBNET", "172.21.0.0/16")
    routing_mode = routing_mode or os.environ.get("RADIO_ROUTING", "linkstate")
    lease_ms = lease_ms or int(os.environ.get("RADIO_LEASE_MS", "2000"))
    iface, ip = find_interface(subnet)
    conf = zenoh.Config()
    conf.insert_json5("mode", '"peer"')
    # Listen only on the radio address so every peer link runs over the radio network.
    conf.insert_json5("listen/endpoints", json.dumps([f"tcp/{ip}:0"]))
    conf.insert_json5("scouting/multicast/enabled", "true")
    conf.insert_json5("scouting/multicast/interface", json.dumps(iface))
    conf.insert_json5("routing/peer/mode", json.dumps(routing_mode))
    conf.insert_json5("transport/link/tx/lease", str(int(lease_ms)))
    return zenoh.open(conf)


def open_onboard(router_locator: str = None) -> zenoh.Session:
    """Client session to the simulator's router (onboard sensors/actuators)."""
    conf = zenoh.Config()
    conf.insert_json5("mode", '"client"')
    conf.insert_json5("scouting/multicast/enabled", "false")
    if router_locator:
        conf.insert_json5("connect/endpoints", json.dumps([router_locator]))
    return zenoh.open(conf)
