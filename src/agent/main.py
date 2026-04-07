import argparse
import fcntl
import json
import os
import socket
import time
from pathlib import Path

from agent import DenddronAgent


def _sorted_agent_ids(cfg: dict) -> list:
    agents = cfg.get("agents", {})

    def sort_key(agent_id: str):
        num = ""
        for ch in reversed(agent_id):
            if ch.isdigit():
                num = ch + num
            elif num:
                break
        return int(num) if num else 10**9

    return sorted(agents.keys(), key=sort_key)


def allocate_agent_id(runtime_cfg_path: str, registry_path: str) -> str:
    cfg_path = Path(runtime_cfg_path)
    if not cfg_path.exists():
        return os.getenv("AGENT_ID", "drone_0")

    try:
        cfg = json.loads(cfg_path.read_text(encoding="utf-8"))
        available_ids = _sorted_agent_ids(cfg)
    except Exception:
        return os.getenv("AGENT_ID", "drone_0")

    if not available_ids:
        return os.getenv("AGENT_ID", "drone_0")

    host_key = socket.gethostname()
    reg_path = Path(registry_path)
    reg_path.parent.mkdir(parents=True, exist_ok=True)
    lock_path = reg_path.with_suffix(reg_path.suffix + ".lock")

    with lock_path.open("a+", encoding="utf-8") as lock_file:
        fcntl.flock(lock_file.fileno(), fcntl.LOCK_EX)

        if reg_path.exists():
            try:
                registry = json.loads(reg_path.read_text(encoding="utf-8"))
            except Exception:
                registry = {"assigned": {}}
        else:
            registry = {"assigned": {}}

        assigned = registry.get("assigned", {})
        existing = assigned.get(host_key)
        if existing in available_ids:
            chosen = existing
        else:
            used = {v for v in assigned.values() if v in available_ids}
            chosen = None
            for candidate in available_ids:
                if candidate not in used:
                    chosen = candidate
                    break

            if chosen is None:
                chosen = os.getenv("AGENT_ID", available_ids[0])

            assigned[host_key] = chosen
            registry["assigned"] = assigned
            reg_path.write_text(json.dumps(registry, indent=2) + "\n", encoding="utf-8")

        fcntl.flock(lock_file.fileno(), fcntl.LOCK_UN)

    return chosen


def main():
    parser = argparse.ArgumentParser(description="DeNDDron Swarm Python Agent")
    parser.add_argument(
        "--agent-id",
        type=str,
        default=os.getenv("AGENT_ID", None),
        help="Unique ID for this drone",
    )
    parser.add_argument(
        "--zenoh-router",
        type=str,
        default=os.getenv("ZENOH_ROUTER_IP", None),
        help="Locator for Zenoh router (e.g. tcp/zenoh_router:7447)",
    )
    parser.add_argument(
        "--runtime-config",
        type=str,
        default=os.getenv("SWARM_RUNTIME_CONFIG", "/app/config/swarm_runtime.json"),
        help="Runtime config path used for deterministic ID allocation",
    )
    parser.add_argument(
        "--id-registry",
        type=str,
        default=os.getenv("AGENT_ID_REGISTRY", "/app/config/agent_registry.json"),
        help="Shared ID registry path",
    )

    args = parser.parse_args()

    agent_id = args.agent_id if args.agent_id else allocate_agent_id(args.runtime_config, args.id_registry)
    agent = DenddronAgent(agent_id=agent_id, router_locator=args.zenoh_router)

    try:
        while True:
            time.sleep(1.0)
    except KeyboardInterrupt:
        print(f"\n[{agent_id}] Shutting down...")
        agent.shutdown()


if __name__ == "__main__":
    main()
