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
import time

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


# Per-topic Zenoh QoS for the radio (RADIO_QOS). Priorities give each class its own send queue,
# so command-and-control traffic is not stuck behind telemetry when the radio is congested.
# Over UDP every message is best-effort on the wire; "block" only means the sender waits for
# queue space instead of dropping locally.
QOS_PROFILES = {
    "default": [],   # Zenoh defaults for everything
    "tuned": [
        {"key_exprs": ["swarm/threats", "swarm/awards", "ship/threat_status",      # orders, awards, jobs
                       "ship/ack/**"],
         "config": {"priority": "real_time", "express": True, "congestion_control": "block"}},
        {"key_exprs": ["swarm/bids"],
         "config": {"priority": "interactive_high", "express": True, "congestion_control": "block"}},
        {"key_exprs": ["ship/roster", "ship/zones"],
         "config": {"priority": "data_high", "express": True, "congestion_control": "drop"}},
        {"key_exprs": ["swarm/heartbeat/**", "swarm/heartbeat_relay/**", "swarm/heartbeat_help/**"],
         "config": {"priority": "data_low", "congestion_control": "drop", "reliability": "best_effort"}},
        {"key_exprs": ["swarm/telemetry/**"],
         "config": {"priority": "background", "congestion_control": "drop", "reliability": "best_effort"}},
    ],
}


QUIC_PORT_OFFSET = 1   # a node's QUIC link listens on its UDP port + this (both are UDP sockets)


def _port(proto: str, port: int) -> int:
    return port + QUIC_PORT_OFFSET if proto == "quic" and port else port


def open_radio(subnet: str = None, routing_mode: str = None, lease_ms: int = None) -> zenoh.Session:
    """Peer-mode session bound to the radio network only.

    Agents and the ship open this inside a RadioProcess (radio_process.py), so that
    no radio problem can freeze their onboard link.

    - protocol (RADIO_PROTO, quic,udp): see the comments below.
    - QoS profile (RADIO_QOS, default): per-topic priority/congestion rules, QOS_PROFILES.
    - lease (RADIO_LEASE_MS, 6 s): how soon a lost peer's session closes.  At 2 s, 30 % loss lost
      enough keep-alives in a row to close and reopen sessions several times a run (50 % loss: 54
      times in 2 min, 8 peers), and a reopened session must declare everything again; at 6 s, none.
    - open/accept timeout (RADIO_OPEN_TIMEOUT_MS, 1 s): bounds connection attempts.
    - routing_mode (RADIO_ROUTING) only exists before Zenoh 1.1; newer versions
      always route peer-to-peer and ignore it.
    """
    subnet = subnet or os.environ.get("RADIO_SUBNET", "172.21.0.0/16")
    routing_mode = routing_mode or os.environ.get("RADIO_ROUTING", "linkstate")
    lease_ms = lease_ms or int(os.environ.get("RADIO_LEASE_MS") or 6000)
    open_ms = int(os.environ.get("RADIO_OPEN_TIMEOUT_MS") or 1000)
    iface, ip = find_interface(subnet)
    conf = zenoh.Config()
    conf.insert_json5("mode", '"peer"')
    # Listen only on the radio address so every peer link runs over the radio network.
    # RADIO_PROTO: "quic,udp" (default), "udp", "tcp", or "tcp,udp". With any TCP link, jamming one
    # peer stalled other peers' traffic (and the jammed node's other Zenoh sessions) for ~10 s inside
    # Zenoh's TCP link handling; over UDP and QUIC that does not happen (radio_probe.sh). Data is
    # best-effort over UDP: the protocol recovers from lost orders/bids/awards by repeats,
    # re-announcement and conflict repair. QUIC carries Zenoh's control messages (below).
    protos = [p.strip() for p in (os.environ.get("RADIO_PROTO") or "quic,udp").split(",") if p.strip()]
    port = int(os.environ.get("RADIO_LISTEN_PORT") or 0)       # the ship listens on a fixed port
    conf.insert_json5("listen/endpoints", json.dumps([f"{p}/{ip}:{_port(p, port)}" for p in protos]))
    # Meeting point: also connect to the ship's fixed radio address (RADIO_CONNECT), retrying until
    # it is up.  Multicast discovery alone occasionally missed a node when ~50 start at once, and
    # a drone with no radio session is silent; peer gossip introduces the others once connected.
    connect = [e.strip() for e in (os.environ.get("RADIO_CONNECT") or "").split(",") if e.strip()]
    if "quic" in protos:   # the meeting point's QUIC link listens next to its UDP one
        connect += [f"quic/{host}:{_port('quic', int(p))}" for host, p in
                    (e[len("udp/"):].rsplit(":", 1) for e in connect if e.startswith("udp/"))]
    if connect:
        conf.insert_json5("connect/endpoints", json.dumps(connect))
        conf.insert_json5("connect/exit_on_failure", "false")
        conf.insert_json5("connect/retry", json.dumps({"period_init_ms": 500, "period_max_ms": 2000,
                                                       "period_increase_factor": 2}))
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
    if "quic" in protos:
        # Reliable control plane.  Over UDP, Zenoh sends its control messages once: a lost key
        # expression declaration makes every later message that names it undecodable at that peer
        # ("Unknown wire expr"), so a subscription or a publisher's data to that peer stays dead
        # (tools/comms/declare_probe.sh, mesh_probe.sh).  A QUIC link next to the UDP one carries
        # the reliable messages (declarations, interests); publications are best-effort (below),
        # so data keeps using UDP and is never retransmitted late.
        tls = os.path.join(os.path.dirname(os.path.abspath(__file__)), "radio_tls")
        for key, value in (("root_ca_certificate", "ca.pem"), ("listen_certificate", "radio.pem"),
                           ("listen_private_key", "radio.key")):
            conf.insert_json5(f"transport/link/tls/{key}", json.dumps(os.path.join(tls, value)))
        conf.insert_json5("transport/link/tls/verify_name_on_connect", "false")
    qos = os.environ.get("RADIO_QOS") or "default"
    publication = list(QOS_PROFILES[qos])
    if "quic" in protos and "udp" in protos:
        publication.append({"key_exprs": ["**"], "config": {"reliability": "best_effort"}})
    if publication:
        conf.insert_json5("qos/publication", json.dumps(publication))
    # Extra Zenoh settings for experiments, e.g. RADIO_ZENOH_CONFIG='{"routing/interests/timeout": 1000}'.
    # Unknown keys fail loudly here, so a typo cannot silently change nothing.
    for key, value in json.loads(os.environ.get("RADIO_ZENOH_CONFIG") or "{}").items():
        conf.insert_json5(key, json.dumps(value))
    return zenoh.open(conf)


def open_onboard(router_locator: str = None, attempts: int = 30) -> zenoh.Session:
    """Client session to the simulator's router (onboard sensors/actuators).

    Retries with backoff: when many containers start at once the router can be slow to accept,
    and crashing instead would put the container in Docker's restart loop, which adds load.
    """
    delay = 0.5
    for attempt in range(1, attempts + 1):
        conf = zenoh.Config()
        conf.insert_json5("mode", '"client"')
        conf.insert_json5("scouting/multicast/enabled", "false")
        if router_locator:
            conf.insert_json5("connect/endpoints", json.dumps([router_locator]))
        try:
            return zenoh.open(conf)
        except zenoh.ZError as e:
            if attempt == attempts:
                raise
            print(f"[links] onboard bus {router_locator}: {e}; retry {attempt}/{attempts - 1} in {delay:.1f}s",
                  flush=True)
            time.sleep(delay)
            delay = min(delay * 2, 5.0)
