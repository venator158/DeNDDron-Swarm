"""Radio mesh probe (run by radio_probe.sh): each peer publishes at 20 Hz on the radio network and reports,
per second, the slowest put(), the longest process stall (GIL/runtime freeze), and the longest receive gap
from every other peer. Radio settings come from links.py env vars (RADIO_LEASE_MS, RADIO_OPEN_TIMEOUT_MS).
RADIO_PROCESS=1 runs the radio in its own process (src/common/radio_process.py), as the agents do."""
import json
import os
import threading
import time

from links import open_radio


def main():
    me = os.environ["NAME"]
    duration = float(os.environ.get("DURATION", "40"))
    subnet = os.environ.get("SUBNET", "172.31.0.0/16")
    if os.environ.get("RADIO_PROCESS") == "1":
        from radio_process import RadioProcess
        s = RadioProcess(subnet)
    else:
        s = open_radio(subnet)
    last_rx, max_gap = {}, {}
    lock = threading.Lock()

    def on_msg(smp):
        peer = str(smp.key_expr).rsplit("/", 1)[-1]
        if peer == me:
            return
        now = time.monotonic()
        with lock:
            if peer in last_rx:
                max_gap[peer] = max(max_gap.get(peer, 0.0), now - last_rx[peer])
            last_rx[peer] = now

    s.declare_subscriber("probe/*", on_msg)
    pub = s.declare_publisher(f"probe/{me}")
    stall = {"max": 0.0}

    def ticker():   # a pure-Python thread: gaps here mean the whole process (GIL) stalled
        last = time.monotonic()
        while True:
            time.sleep(0.005)
            now = time.monotonic()
            stall["max"] = max(stall["max"], now - last)
            last = now

    threading.Thread(target=ticker, daemon=True).start()
    t0 = time.monotonic()
    sec, put_max = 0, 0.0
    while time.monotonic() - t0 < duration:
        a = time.monotonic()
        pub.put(json.dumps({"from": me, "t": a}))
        put_max = max(put_max, time.monotonic() - a)
        time.sleep(0.05)
        el = int(time.monotonic() - t0)
        if el != sec:
            with lock:
                gaps = {p: round(g, 2) for p, g in sorted(max_gap.items())}
                max_gap.clear()
            print(f"{me} t={el:3d}s put_max={put_max * 1000:7.1f}ms stall_max={stall['max'] * 1000:7.1f}ms "
                  f"rx_gap_max={gaps}", flush=True)
            put_max, stall["max"], sec = 0.0, 0.0, el
    os._exit(0)


if __name__ == "__main__":
    main()
