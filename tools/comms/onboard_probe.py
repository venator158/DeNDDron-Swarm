"""Onboard-stall probe (run by onboard_probe.sh): a drone-like process with both links. ROLE=pub publishes
a 50 Hz stream on the onboard router; ROLE=drone subscribes to it and also joins the radio. Reports the
worst onboard receive gap per second, so a radio failure that freezes the whole process shows up.
RADIO_PROCESS=1 runs the radio in its own process (src/common/radio_process.py), as the agents do."""
import json
import os
import time

from links import open_onboard, open_radio


def main():
    me, role = os.environ["NAME"], os.environ["ROLE"]
    if role == "pub":
        s = open_onboard("tcp/probe_router:7447")
        p = s.declare_publisher("onb/tick")
        while True:
            p.put(json.dumps({"t": time.monotonic()}))
            time.sleep(0.02)

    if os.environ.get("RADIO_PROCESS") == "1":
        from radio_process import RadioProcess
        radio = RadioProcess("172.31.0.0/16")
    else:
        radio = open_radio("172.31.0.0/16")
    onb = open_onboard("tcp/probe_router:7447")
    last, gap = [None], [0.0]

    def on_tick(_):
        now = time.monotonic()
        if last[0] is not None:
            gap[0] = max(gap[0], now - last[0])
        last[0] = now

    onb.declare_subscriber("onb/tick", on_tick)
    radio.declare_subscriber("probe/*", lambda s: None)
    rpub = radio.declare_publisher(f"probe/{me}")
    t0, sec = time.monotonic(), 0
    while time.monotonic() - t0 < 45:
        rpub.put("x")
        time.sleep(0.05)
        el = int(time.monotonic() - t0)
        if el != sec:
            print(f"{me} t={el:3d}s onboard_gap_max={gap[0] * 1000:7.1f}ms", flush=True)
            gap[0], sec = 0.0, el
    os._exit(0)


if __name__ == "__main__":
    main()
