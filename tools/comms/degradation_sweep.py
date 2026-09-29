"""Degraded-radio sweep: run the same threat scenario under each radio impairment and QoS profile.

For every (profile, condition) pair: start the swarm with RADIO_QOS=profile, wait for the full roster,
impair every radio interface (tc netem via degrade_radio.sh), let a stand-in operator approve feasible
threats, wait until every threat is resolved, collect the ship's /api/summary, restore and tear down.
The scenario is seeded, so every run faces the same threats.

    python3 tools/comms/degradation_sweep.py                      # default matrix
    python3 tools/comms/degradation_sweep.py --profiles tuned --conditions baseline= loss10="loss 10%"

Results: <out>/results.csv, <out>/results.md, <out>/<run>/{swarm.log,summary.json,state.json}
"""

import argparse
import csv
import json
import os
import subprocess
import sys
import time
import urllib.request
from pathlib import Path

HERE = Path(__file__).resolve().parent
REPO = HERE.parents[1]
API = "http://localhost:8080"

DEFAULT_CONDITIONS = {
    "baseline": "",
    "loss10": "loss 10%",
    "loss30": "loss 30%",
    "delay200": "delay 200ms 50ms",
    "bw64k": "rate 64kbit",
}
COLUMNS = ["profile", "condition", "netem", "approved", "destroyed", "failed", "leaked_or_impact", "kill_ratio",
           "award_latency_ms_mean", "award_latency_ms_max", "never_fully_assigned", "reannounces",
           "over_assigned", "missed_slots", "agreement_mean", "drones_expended", "hb_rx_per_s_at_ship",
           "wall_s"]


def log(msg):
    print(f"{time.strftime('%H:%M:%S')} {msg}", flush=True)


def get(path, timeout=2):
    return json.load(urllib.request.urlopen(API + path, timeout=timeout))


def wait_for(cond, timeout, step=2.0):
    end = time.time() + timeout
    while time.time() < end:
        try:
            if cond():
                return True
        except Exception:
            pass
        time.sleep(step)
    return False


def compose_down():
    subprocess.run(["docker", "compose", "--env-file", ".swarm.env", "down"], cwd=REPO,
                   stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)


def run_one(args, profile, cond, netem, outdir):
    rundir = outdir / f"{profile}_{cond}"
    rundir.mkdir(parents=True, exist_ok=True)
    env = dict(os.environ, RADIO_QOS=profile)
    t0 = time.time()
    log(f"=== {profile} / {cond} ({netem or 'no impairment'})")
    swarm_log = open(rundir / "swarm.log", "w")
    swarm = subprocess.Popen(
        ["bash", "scripts/run_swarm.sh", str(args.drones), "--threats", str(args.threats),
         "--threat-interval", str(args.interval), "--first-threat", str(args.first), "--seed", str(args.seed)],
        cwd=REPO, env=env, stdout=swarm_log, stderr=subprocess.STDOUT)
    bot = None
    try:
        if not wait_for(lambda: get("/api/state")["roster"]["count"] >= args.drones, 300):
            raise RuntimeError("swarm did not come up")
        if netem:
            subprocess.run([str(HERE / "degrade_radio.sh"), "apply", netem], check=True)
        bot = subprocess.Popen([sys.executable, str(HERE / "operator_bot.py"), "3", str(args.timeout)],
                               stdout=open(rundir / "operator.log", "w"), stderr=subprocess.STDOUT)

        def resolved():
            s = get("/api/summary")
            return s["threats_detected"] >= args.threats and s["still_active"] == 0
        if not wait_for(resolved, args.timeout, step=3.0):
            log("timeout: not every threat resolved")
        summary = get("/api/summary", timeout=5)
        (rundir / "summary.json").write_text(json.dumps(summary, indent=2))
        (rundir / "state.json").write_text(json.dumps(get("/api/state", timeout=5), indent=2))
    finally:
        if bot:
            bot.terminate()
        if netem:
            subprocess.run([str(HERE / "degrade_radio.sh"), "clear"], stdout=subprocess.DEVNULL)
        compose_down()
        swarm.wait(timeout=60)
        swarm_log.close()
    row = {k: summary.get(k) for k in COLUMNS if k in summary}
    row.update(profile=profile, condition=cond, netem=netem or "-",
               hb_rx_per_s_at_ship=summary.get("radio_rx_at_ship", {}).get("swarm/heartbeat", {}).get("msgs_per_s"),
               wall_s=round(time.time() - t0))
    log("    " + " ".join(f"{k}={row.get(k)}" for k in COLUMNS[3:]))
    return row


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--profiles", nargs="+", default=["default", "tuned"])
    ap.add_argument("--conditions", nargs="+", metavar="NAME=NETEM",
                    help="e.g. loss10='loss 10%%' (empty netem = no impairment); default: built-in matrix")
    ap.add_argument("--drones", type=int, default=8)
    ap.add_argument("--threats", type=int, default=4)
    ap.add_argument("--interval", type=float, default=30)
    ap.add_argument("--first", type=float, default=25)
    ap.add_argument("--seed", type=int, default=42)
    ap.add_argument("--timeout", type=float, default=360, help="wall seconds per run after the swarm is up")
    ap.add_argument("--out", default=str(REPO / "results" / time.strftime("sweep_%Y%m%d_%H%M%S")))
    args = ap.parse_args()

    conditions = DEFAULT_CONDITIONS if not args.conditions else dict(
        (c.split("=", 1) + [""])[:2] for c in args.conditions)
    outdir = Path(args.out)
    outdir.mkdir(parents=True, exist_ok=True)
    compose_down()
    rows = []
    for cond, netem in conditions.items():
        for profile in args.profiles:
            try:
                rows.append(run_one(args, profile, cond, netem, outdir))
            except Exception as e:
                log(f"run failed: {e}")
                rows.append({"profile": profile, "condition": cond, "netem": netem or "-"})
            with open(outdir / "results.csv", "w", newline="") as f:
                w = csv.DictWriter(f, fieldnames=COLUMNS)
                w.writeheader()
                w.writerows(rows)
    md = ["| " + " | ".join(COLUMNS) + " |", "|" + "---|" * len(COLUMNS)]
    md += ["| " + " | ".join(str(r.get(k, "")) for k in COLUMNS) + " |" for r in rows]
    (outdir / "results.md").write_text("\n".join(md) + "\n")
    print("\n".join(md))
    log(f"results in {outdir}")


if __name__ == "__main__":
    main()
