"""Stand-in operator for unattended runs: through the dashboard API, approves every feasible threat,
most urgent (smallest TCPA) first, after a human-like reaction delay.

    python3 tools/comms/operator_bot.py [reaction_s=3] [duration_s=400] [poll_s=1]

All three are wall seconds; the sweep divides them by the sim's real-time factor.
"""
import json
import os
import sys
import time
import urllib.request

URL = os.environ.get("DASHBOARD_URL", "http://localhost:8080")   # set per swarm instance by the sweeps
REACTION_S = float(sys.argv[1]) if len(sys.argv) > 1 else 3.0
DURATION_S = float(sys.argv[2]) if len(sys.argv) > 2 else 400.0
POLL_S = float(sys.argv[3]) if len(sys.argv) > 3 else 1.0

first_feasible = {}
done = set()
t_end = time.time() + DURATION_S
while time.time() < t_end:
    try:
        s = json.load(urllib.request.urlopen(URL + "/api/state", timeout=2))
    except Exception as e:
        print("state error", e, flush=True)
        time.sleep(1)
        continue
    for t in s["threats"]:          # already in min-heap (TCPA) order
        tid, f = t["threat_id"], t.get("feasibility")
        if t["status"] != "tracking" or tid in done or not f:
            continue
        if not f["feasible"]:
            first_feasible.pop(tid, None)
            continue
        first_feasible.setdefault(tid, time.time())
        if time.time() - first_feasible[tid] >= REACTION_S:
            req = urllib.request.Request(URL + "/api/approve", data=json.dumps({"threat_id": tid}).encode(),
                                         headers={"Content-Type": "application/json"}, method="POST")
            try:
                resp = json.load(urllib.request.urlopen(req, timeout=2))
            except urllib.error.HTTPError as e:
                resp = json.load(e)
            print(f"t={s['sim_time']:.1f} approve {tid} ({t['type']} L{t['level']}, TCPA {t['tcpa_s']}s, "
                  f"TTI {f['tti_s']}s / {f['available_s']}s): {resp}", flush=True)
            done.add(tid)
            break   # one approval per poll, most urgent first
    time.sleep(POLL_S)
