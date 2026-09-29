"""Settings for one of several swarm instances running side by side on a host.

Instance 0 is the default and keeps the original names, ports and subnets, so existing commands
work unchanged.  Instance k >= 1 gets its own Compose project, container names, host ports,
subnets, config directory and env file, so several simulations can run at once (e.g. sweep
cells in parallel on a cluster node).  Built images are shared by all instances.

    python3 scripts/swarm_instance.py 2            # shell assignments (eval in bash)
    python3 scripts/swarm_instance.py 2 --json

Subnets for k >= 1: sim 10.(SWARM_SUBNET_BASE+k).0.0/17, radio 10.(SWARM_SUBNET_BASE+k).128.0/17,
SWARM_SUBNET_BASE defaulting to 210.  Override SIM_SUBNET / RADIO_SUBNET / SIM_BUS_IP to pick
others (e.g. where the host already routes those ranges).
"""

import argparse
import ipaddress
import json
import os
import shlex
from pathlib import Path

REPO = Path(__file__).resolve().parents[1]
MAX_INSTANCES = 30


def instance(k: int) -> dict:
    if not 0 <= k <= MAX_INSTANCES:
        raise ValueError(f"instance must be 0..{MAX_INSTANCES}")
    if k == 0:
        cfg = {
            "COMPOSE_PROJECT_NAME": "denddron-swarm",
            "NAME_PREFIX": "",
            "DASHBOARD_PORT": "8080",
            "GAZEBO_PORT": "11345",
            "SIM_SUBNET": "172.20.0.0/16",
            "RADIO_SUBNET": "172.21.0.0/16",
            "SWARM_CONFIG_DIR": "./config",
            "ENV_FILE": ".swarm.env",
        }
    else:
        second = int(os.environ.get("SWARM_SUBNET_BASE", "210")) + k
        cfg = {
            "COMPOSE_PROJECT_NAME": f"denddron-swarm-{k}",
            "NAME_PREFIX": f"i{k}-",
            "DASHBOARD_PORT": str(8080 + k),
            "GAZEBO_PORT": str(11345 + k),
            "SIM_SUBNET": f"10.{second}.0.0/17",
            "RADIO_SUBNET": f"10.{second}.128.0/17",
            "SWARM_CONFIG_DIR": f"./config/instances/{k}",
            "ENV_FILE": f".swarm.{k}.env",
        }
    for key in ("SIM_SUBNET", "RADIO_SUBNET"):
        if os.environ.get(key) and k != 0:
            cfg[key] = os.environ[key]
    # The sim bus router has a fixed address (.2 of the sim subnet): everything connects by IP.
    cfg["SIM_BUS_IP"] = os.environ.get("SIM_BUS_IP") if k and os.environ.get("SIM_BUS_IP") else \
        str(ipaddress.ip_network(cfg["SIM_SUBNET"]).network_address + 2)
    cfg["SWARM_INSTANCE"] = str(k)
    return cfg


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("k", type=int, nargs="?", default=int(os.environ.get("SWARM_INSTANCE", "0")))
    ap.add_argument("--json", action="store_true")
    args = ap.parse_args()
    cfg = instance(args.k)
    if args.json:
        print(json.dumps(cfg))
    else:
        print("\n".join(f"{k}={shlex.quote(v)}" for k, v in cfg.items()))


if __name__ == "__main__":
    main()
