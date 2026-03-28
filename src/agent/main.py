import os
import time
import argparse
from agent import DenddronAgent

def main():
    parser = argparse.ArgumentParser(description="DeNDDron Swarm Python Agent")
    parser.add_argument("--agent-id", type=str, default=os.getenv("AGENT_ID", "drone_0"), help="Unique ID for this drone")
    parser.add_argument("--zenoh-router", type=str, default=os.getenv("ZENOH_ROUTER_IP", None), help="Locator for Zenoh router (e.g. tcp/zenoh_router:7447)")
    
    args = parser.parse_args()
    
    agent = DenddronAgent(agent_id=args.agent_id, router_locator=args.zenoh_router)
    
    try:
        # Keep main thread alive while background threads do the work
        while True:
            time.sleep(1.0)
    except KeyboardInterrupt:
        print(f"\n[{args.agent_id}] Shutting down...")
        agent.shutdown()

if __name__ == "__main__":
    main()
