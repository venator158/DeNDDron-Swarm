"""Live radio-loss test for a running swarm (see README "Degraded communications").

Cuts one idle drone's radio once the roster is full, then the radio of the first drone that engages a
threat, and restores the idle one 45 s later. Only the radio network is touched, so the drones' onboard
links (sensors/actuators) stay up.

    bash scripts/run_swarm.sh 8 --threats 4 > run.log 2>&1 &
    python3 tools/comms/operator_bot.py &          # approves threats
    python3 tools/comms/chaos.py run.log [disconnect|netem]
"""
import json
import re
import subprocess
import sys
import time
import urllib.request
from pathlib import Path

LOG = sys.argv[1]
MODE = sys.argv[2] if len(sys.argv) > 2 else "disconnect"
NET = "denddron-swarm_radio_net"
SUBNET = "172.21.0.0/16"
CUT = str(Path(__file__).with_name("cut_radio.sh"))


def sh(*cmd):
    r = subprocess.run(cmd, capture_output=True, text=True)
    return (r.stdout + r.stderr).strip()


def container_of(drone):
    # log lines look like: "agent-3  | INFO:DenddronAgent:[drone_4] ..."
    for line in open(LOG, errors="ignore"):
        m = re.match(r"(agent-\d+)\s+\|.*\[" + re.escape(drone) + r"\]", line)
        if m:
            return "denddron-swarm-" + m.group(1)
    return None


def state():
    return json.load(urllib.request.urlopen("http://localhost:8080/api/state", timeout=2))


def log(msg):
    print(f"{time.strftime('%H:%M:%S')} {msg}", flush=True)


# 1) wait for a full roster, then cut an idle drone
while True:
    try:
        s = state()
        if s["roster"]["count"] >= 8 and s["sim_time"] and s["sim_time"] > 10:
            break
    except Exception:
        pass
    time.sleep(1)
idle = "drone_1"
c_idle = container_of(idle)
log(f"CUT radio of idle {idle} ({c_idle}): {sh(CUT, c_idle, NET, SUBNET, MODE, '45s')}")
t_cut_idle = time.time()

# 2) cut the first drone that engages
engaged = None
seen = set()
reconnected = False
while True:
    if not reconnected and time.time() - t_cut_idle > 45:
        if MODE == "disconnect":
            log(f"RECONNECT radio of {idle}: {sh('docker', 'network', 'connect', NET, c_idle) or 'ok'}")
        else:
            log(f"radio of {idle} restored (netem period over)")
        reconnected = True
    if engaged is None:
        for line in open(LOG, errors="ignore"):
            m = re.search(r"\[(drone_\d+)\] ENGAGING (T\d+)", line)
            if m and m.group(1) != idle and m.group(1) not in seen:
                engaged, threat = m.group(1), m.group(2)
                seen.add(engaged)
                c = container_of(engaged)
                log(f"CUT radio of ENGAGED {engaged} on {threat} ({c}): {sh(CUT, c, NET, SUBNET, MODE, '120s')}")
                break
    if engaged and reconnected:
        break
    time.sleep(0.5)
log("chaos script done")
