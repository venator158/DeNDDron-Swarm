"""Degraded-radio sweep: run the same threat scenario under each radio impairment and QoS profile.

For every (condition, profile, repeat): start the swarm with RADIO_QOS=profile, wait for the full
roster, impair every radio interface (tc netem via degrade_radio.sh), let a stand-in operator approve
feasible threats, wait until every threat is resolved, collect the ship's /api/summary, restore and
tear down. The scenario is seeded, so every run faces the same threats.

--rtf K runs the simulation K times faster than real time. The swarm's protocol clocks scale with it
(src/common/simclock.py), and so do the impairments: netem delays are divided by K and rates
multiplied by K, so each run is equivalent to a real-time run with the nominal impairment. Loss is
unchanged. Timings in the results are in simulated time.

    python3 tools/comms/degradation_sweep.py                                  # default matrix, 1 run each
    python3 tools/comms/degradation_sweep.py --rtf 3 --repeats 3
    python3 tools/comms/degradation_sweep.py --profiles tuned --conditions baseline= loss10="loss 10%"

Results: <out>/results.csv (one row per run), <out>/results.md (runs, then mean ± sd per cell),
<out>/<run>/{swarm.log,operator.log,summary.json,state.json}
"""

import argparse
import csv
import json
import os
import re
import statistics
import subprocess
import sys
import threading
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
    "bw24k": "rate 24kbit",
}
COLUMNS = ["profile", "condition", "netem", "rep", "rtf", "drones", "threats", "approved", "destroyed", "failed", "leaked_or_impact",
           "kill_ratio", "award_latency_ms_mean", "award_latency_ms_max", "never_fully_assigned", "reannounces",
           "over_assigned", "missed_slots", "rejected_awards", "agreement_mean", "drones_expended",
           "hb_rx_per_s_at_ship", "wall_s"]
# Resource columns, sampled during the run (Sampler); see scaling_sweep.py.
RESOURCE_COLUMNS = ["rtf_measured", "startup_s", "host_cpu_pct", "drone_cpu_pct", "drone_mem_mb", "gazebo_cpu_pct",
                    "sim_bus_cpu_pct", "ship_cpu_pct", "loop_p99_ms_max", "overruns", "sensor_age_max_ms",
                    "drone_rx_msgs_per_s", "drone_tx_bytes_per_s", "ship_rx_msgs_per_s"]
AGGREGATE = ["destroyed", "kill_ratio", "award_latency_ms_mean", "reannounces", "over_assigned", "missed_slots",
             "agreement_mean", "drones_expended"]

_TIME = re.compile(r"^(\d+(?:\.\d+)?)(us|usec|ms|msec|s|sec)$")
_RATE = re.compile(r"^(\d+(?:\.\d+)?)(bit|kbit|mbit|gbit|bps|kbps|mbps|gbps)$")
_US_PER = {"us": 1, "usec": 1, "ms": 1e3, "msec": 1e3, "s": 1e6, "sec": 1e6}


def scale_netem(netem, rtf):
    """Netem args for a sim running rtf times faster: times / rtf, rates * rtf (loss is unchanged)."""
    out = []
    for tok in netem.split():
        if m := _TIME.match(tok):
            tok = f"{round(float(m[1]) * _US_PER[m[2]] / rtf)}us"
        elif m := _RATE.match(tok):
            tok = f"{float(m[1]) * rtf:g}{m[2]}"
        out.append(tok)
    return " ".join(out)


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


class Sampler(threading.Thread):
    """Samples container CPU/memory (docker stats) and drone telemetry (/api/state) during a run.

    Drones that have detonated sit idle, so per-drone figures average only drones still in play,
    matched to containers through config/agent_registry.json (container hostname -> drone id).
    """

    def __init__(self, period_s=10.0):
        super().__init__(daemon=True)
        self.period_s, self.samples, self._halt = period_s, [], threading.Event()

    def stop(self):
        self._halt.set()
        self.join(timeout=30)

    def run(self):
        while not self._halt.wait(self.period_s):
            try:
                self.samples.append(self._sample())
            except Exception as e:
                log(f"    sampler: {e}")

    @staticmethod
    def _sample():
        wall = time.time()
        state = get("/api/state", timeout=5)
        out = subprocess.run(["docker", "stats", "--no-stream", "--format", "{{.ID}}\t{{.Name}}\t{{.CPUPerc}}\t{{.MemUsage}}"],
                             capture_output=True, text=True, timeout=60).stdout
        try:
            registry = json.loads((REPO / "config" / "agent_registry.json").read_text())["assigned"]
        except Exception:
            registry = {}
        live = {d["id"] for d in state["drones"] if d["state"] not in ("expended", None)}
        containers = {}
        for line in out.splitlines():
            cid, name, cpu, mem = line.split("\t")
            used = mem.split("/")[0].strip()
            mb = float(re.sub(r"[^0-9.]", "", used)) * (1024 if "GiB" in used else 1 if "MiB" in used else 1 / 1024)
            containers[name] = (registry.get(cid), float(cpu.rstrip("%") or 0), mb)
        agents = [(d, c, m) for n, (d, c, m) in containers.items() if "-agent-" in n]
        tel = [d["telemetry"] for d in state["drones"] if d["id"] in live and d.get("telemetry")]
        return {
            "wall": wall, "sim": state["sim_time"],
            "host_cpu": sum(c for _, c, _ in containers.values()),
            "drone_cpu": [c for d, c, _ in agents if d in live],
            "drone_mem": [m for _, _, m in agents],
            **{k: containers.get(k, (None, None))[1] for k in ("gazebo_simulator", "sim_bus", "ship")},
            "loop_p99": [t.get("loop_p99_ms") for t in tel if t.get("loop_p99_ms") is not None],
            "overruns": sum(t.get("overruns") or 0 for t in tel),
            "sensor_age": [t.get("sensor_age_max_ms") for t in tel if t.get("sensor_age_max_ms") is not None],
            "rx": [sum((t.get("rx_per_s") or {}).values()) for t in tel],
            "tx_bytes": [t.get("tx_bytes_per_s") or 0 for t in tel],
            "ship_rx": sum(v.get("msgs_per_s", 0) for v in state.get("radio_rx_at_ship", {}).values()),
        }

    def summary(self):
        s = self.samples
        if len(s) < 2:
            return {}
        mean = lambda xs: round(statistics.mean(xs), 1) if xs else None
        flat = lambda k: [x for smp in s for x in smp[k]]
        return {
            "rtf_measured": round((s[-1]["sim"] - s[0]["sim"]) / (s[-1]["wall"] - s[0]["wall"]), 2),
            "host_cpu_pct": mean([x["host_cpu"] for x in s]),
            "drone_cpu_pct": mean(flat("drone_cpu")),
            "drone_mem_mb": mean(flat("drone_mem")),
            "gazebo_cpu_pct": mean([x["gazebo_simulator"] for x in s if x["gazebo_simulator"] is not None]),
            "sim_bus_cpu_pct": mean([x["sim_bus"] for x in s if x["sim_bus"] is not None]),
            "ship_cpu_pct": mean([x["ship"] for x in s if x["ship"] is not None]),
            "loop_p99_ms_max": max(flat("loop_p99"), default=None),
            "overruns": sum(x["overruns"] for x in s),
            "sensor_age_max_ms": max(flat("sensor_age"), default=None),
            "drone_rx_msgs_per_s": mean(flat("rx")),
            "drone_tx_bytes_per_s": mean(flat("tx_bytes")),
            "ship_rx_msgs_per_s": mean([x["ship_rx"] for x in s]),
        }


def compose_down():
    subprocess.run(["docker", "compose", "--env-file", ".swarm.env", "down"], cwd=REPO,
                   stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)


def run_one(args, profile, cond, netem, rep, outdir, name=None, extra_env=None):
    rundir = outdir / (name or f"{profile}_{cond}_r{rep}")
    rundir.mkdir(parents=True, exist_ok=True)
    rtf = args.rtf
    env = dict(os.environ, RADIO_QOS=profile, SIM_RTF=f"{rtf:g}", **(extra_env or {}))
    applied = scale_netem(netem, rtf) if netem else ""
    t0 = time.time()
    log(f"=== {profile} / {cond} #{rep} ({netem or 'no impairment'}"
        + (f"; applied as '{applied}' at {rtf:g}x" if netem and rtf != 1 else "") + ")")
    swarm_log = open(rundir / "swarm.log", "w")
    swarm = subprocess.Popen(
        ["bash", "scripts/run_swarm.sh", str(args.drones), "--threats", str(args.threats),
         "--threat-interval", str(args.interval), "--first-threat", str(args.first), "--seed", str(args.seed)],
        cwd=REPO, env=env, stdout=swarm_log, stderr=subprocess.STDOUT)
    bot = None
    summary = {}
    sampler = Sampler(getattr(args, "sample_s", 10.0)) if getattr(args, "sample", False) else None
    startup_s = None
    try:
        if not wait_for(lambda: get("/api/state")["roster"]["count"] >= args.drones,
                        getattr(args, "startup_timeout", 300)):
            raise RuntimeError("swarm did not come up")
        startup_s = round(time.time() - t0)
        if sampler:
            sampler.start()
        if applied:
            subprocess.run([str(HERE / "degrade_radio.sh"), "apply", applied], check=True)
        bot = subprocess.Popen([sys.executable, str(HERE / "operator_bot.py"), f"{args.reaction / rtf:g}",
                                f"{args.timeout / rtf:g}", f"{1.0 / rtf:g}"],
                               stdout=open(rundir / "operator.log", "w"), stderr=subprocess.STDOUT)

        def resolved():
            s = get("/api/summary")
            return s["threats_detected"] >= args.threats and s["still_active"] == 0
        if not wait_for(resolved, args.timeout / rtf, step=min(3.0, 3.0 / rtf + 0.5)):
            log("timeout: not every threat resolved")
        if sampler:
            sampler.stop()
            (rundir / "samples.json").write_text(json.dumps(sampler.samples))
        summary = get("/api/summary", timeout=5)
        (rundir / "summary.json").write_text(json.dumps(summary, indent=2))
        (rundir / "state.json").write_text(json.dumps(get("/api/state", timeout=5), indent=2))
    finally:
        if sampler and sampler.is_alive():
            sampler.stop()
        if bot:
            bot.terminate()
        if applied:
            subprocess.run([str(HERE / "degrade_radio.sh"), "clear"], stdout=subprocess.DEVNULL)
        compose_down()
        swarm.wait(timeout=60)
        swarm_log.close()
    row = {k: summary.get(k) for k in COLUMNS if k in summary}
    if sampler:
        row.update(sampler.summary(), startup_s=startup_s)
    row.update(profile=profile, condition=cond, netem=netem or "-", rep=rep, rtf=rtf, drones=args.drones,
               threats=args.threats,
               hb_rx_per_s_at_ship=summary.get("radio_rx_at_ship", {}).get("swarm/heartbeat", {}).get("msgs_per_s"),
               wall_s=round(time.time() - t0))
    log("    " + " ".join(f"{k}={row.get(k)}" for k in COLUMNS[7:] + (RESOURCE_COLUMNS if sampler else [])))
    return row


def aggregate(rows):
    """Markdown table of mean ± sd of the key metrics per (condition, profile)."""
    cells = {}
    for r in rows:
        cells.setdefault((r["condition"], r["profile"]), []).append(r)
    md = ["| condition | profile | runs | " + " | ".join(AGGREGATE) + " |", "|" + "---|" * (len(AGGREGATE) + 3)]
    for (cond, profile), rs in cells.items():
        out = []
        for k in AGGREGATE:
            vals = [float(r[k]) for r in rs if r.get(k) is not None]
            if not vals:
                out.append("")
            elif len(vals) == 1:
                out.append(f"{vals[0]:g}")
            else:
                out.append(f"{statistics.mean(vals):.3g} ± {statistics.stdev(vals):.2g}")
        ok = sum(1 for r in rs if r.get("approved") is not None)
        md.append(f"| {cond} | {profile} | {ok}/{len(rs)} | " + " | ".join(out) + " |")
    return md


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--profiles", nargs="+", default=["default", "tuned"])
    ap.add_argument("--conditions", nargs="+", metavar="NAME=NETEM",
                    help="e.g. loss10='loss 10%%' (empty netem = no impairment); default: built-in matrix")
    ap.add_argument("--repeats", type=int, default=1, help="runs per (condition, profile)")
    ap.add_argument("--rtf", type=float, default=1.0, help="simulation speed-up (real-time factor)")
    ap.add_argument("--drones", type=int, default=8)
    ap.add_argument("--threats", type=int, default=4)
    ap.add_argument("--interval", type=float, default=30)
    ap.add_argument("--first", type=float, default=25)
    ap.add_argument("--seed", type=int, default=42)
    ap.add_argument("--reaction", type=float, default=3.0, help="operator reaction time, sim seconds")
    ap.add_argument("--timeout", type=float, default=360, help="sim seconds per run after the swarm is up")
    ap.add_argument("--out", default=str(REPO / "results" / time.strftime("sweep_%Y%m%d_%H%M%S")))
    args = ap.parse_args()

    conditions = DEFAULT_CONDITIONS if not args.conditions else dict(
        (c.split("=", 1) + [""])[:2] for c in args.conditions)
    outdir = Path(args.out)
    outdir.mkdir(parents=True, exist_ok=True)
    compose_down()
    rows = []
    for rep in range(1, args.repeats + 1):     # repeats outermost: a partial sweep still covers every cell
        for cond, netem in conditions.items():
            for profile in args.profiles:
                try:
                    rows.append(run_one(args, profile, cond, netem, rep, outdir))
                except Exception as e:
                    log(f"run failed: {e}")
                    rows.append({"profile": profile, "condition": cond, "netem": netem or "-", "rep": rep,
                                 "rtf": args.rtf})
                with open(outdir / "results.csv", "w", newline="") as f:
                    w = csv.DictWriter(f, fieldnames=COLUMNS)
                    w.writeheader()
                    w.writerows(rows)
    md = ["| " + " | ".join(COLUMNS) + " |", "|" + "---|" * len(COLUMNS)]
    md += ["| " + " | ".join(str(r.get(k, "")) for k in COLUMNS) + " |" for r in rows]
    md += ["", "Mean ± sd per cell:", ""] + aggregate(rows)
    (outdir / "results.md").write_text("\n".join(md) + "\n")
    print("\n".join(md))
    log(f"results in {outdir}")


if __name__ == "__main__":
    main()
