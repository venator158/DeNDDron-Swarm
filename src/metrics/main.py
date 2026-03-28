import zenoh
import json
import time

def on_collision_data(sample):
    try:
        payload = sample.payload.decode('utf-8')
        data = json.loads(payload)
        print(f"[Metrics] Collision: {data}", flush=True)
    except Exception as e:
        print(f"[Metrics] Parse error: {e}", flush=True)

def main():
    print("[MetricsNode] Connecting to Zenoh...", flush=True)
    conf = zenoh.Config()
    conf.insert_json5("mode", '"client"')
    zenoh.init_log_from_env_or("error")
    
    session = zenoh.open(conf)
    
    print("[MetricsNode] Active.", flush=True)
    sub = session.declare_subscriber("swarm/metrics/collisions", on_collision_data)
    
    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        pass
    finally:
        session.close()

if __name__ == "__main__":
    main()
