"""Onboard-stall probe (run by onboard_probe.sh): a drone-like process with both links. ROLE=pub publishes
a 50 Hz stream on the onboard router; ROLE=drone subscribes to it and also joins the radio. Reports the
worst onboard receive gap per second, so a radio failure that freezes the whole process shows up."""
import json, os, threading, time
from links import open_onboard, open_radio
me, role = os.environ["NAME"], os.environ["ROLE"]
if role == "pub":
    s = open_onboard("tcp/probe_router:7447"); p = s.declare_publisher("onb/tick")
    while True:
        p.put(json.dumps({"t": time.monotonic()})); time.sleep(0.02)
onb = open_onboard("tcp/probe_router:7447"); radio = open_radio("172.31.0.0/16")
last, gap = [None], [0.0]
def on_tick(_):
    now = time.monotonic()
    if last[0] is not None: gap[0] = max(gap[0], now - last[0])
    last[0] = now
sub = onb.declare_subscriber("onb/tick", on_tick)
rsub = radio.declare_subscriber("probe/*", lambda s: None)
rpub = radio.declare_publisher(f"probe/{me}")
t0 = time.monotonic(); sec = 0
while time.monotonic() - t0 < 45:
    rpub.put("x"); time.sleep(0.05)
    el = int(time.monotonic() - t0)
    if el != sec:
        print(f"{me} t={el:3d}s onboard_gap_max={gap[0]*1000:7.1f}ms", flush=True); gap[0] = 0.0; sec = el
os._exit(0)
