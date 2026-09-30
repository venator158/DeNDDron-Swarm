"""Scaling sweep: the same kind of scenario at several swarm sizes, with resource sampling.

For N drones the ship's radar generates N/2 threats (default type mix) with detections every
240/N sim seconds, so the load per drone stays about the same while the swarm grows. Each size
runs at its own real-time factor (limited by host CPU); the achieved factor is measured and
reported (rtf_measured), since a simulator that cannot keep up would make the protocol clock run
ahead of simulated time.

    python3 tools/comms/scaling_sweep.py                          # 8:3 16:3 25:2 50:1
    python3 tools/comms/scaling_sweep.py --sizes 8:3 16:3 --repeats 2 --conditions baseline= loss10="loss 10%"

Results: <out>/results.csv, <out>/results.md, and per run <out>/<run>/{swarm.log,operator.log,
summary.json,state.json,samples.json}
"""

import argparse
import copy
import csv
import threading
import time
from pathlib import Path

from degradation_sweep import (COLUMNS, REPO, RESOURCE_COLUMNS, arp_check, compose_down, log, parse_env, run_jobs,
                               run_one)

SCALE_COLUMNS = ["drones", "threats", "rtf", "condition", "profile", "rep", "instance"] + [
    c for c in COLUMNS + RESOURCE_COLUMNS if c not in ("drones", "threats", "rtf", "condition", "profile", "rep")]


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--sizes", nargs="+", default=["8:3", "16:3", "25:2", "50:1"], metavar="N:RTF",
                    help="swarm sizes, each with the real-time factor to run it at")
    ap.add_argument("--conditions", nargs="+", default=["baseline="], metavar="NAME=NETEM")
    ap.add_argument("--profile", default="default")
    ap.add_argument("--repeats", type=int, default=1)
    ap.add_argument("--threats-per-drone", type=float, default=0.5)
    ap.add_argument("--load-s", type=float, default=240.0,
                    help="detection interval = load_s / N sim seconds (30 s at 8 drones)")
    ap.add_argument("--first", type=float, default=25)
    ap.add_argument("--maneuver-p", type=float, default=0.0,
                    help="probability that a threat turns once mid-flight (THREAT_MANEUVER_P)")
    ap.add_argument("--seed", type=int, default=42)
    ap.add_argument("--reaction", type=float, default=3.0)
    ap.add_argument("--timeout", type=float, default=420, help="sim seconds per run after the swarm is up")
    ap.add_argument("--startup-timeout", type=float, default=180,
                    help="wall seconds for the full roster (a roster that stops growing for 60 s aborts sooner)")
    ap.add_argument("--sample-s", type=float, default=10.0, help="wall seconds between resource samples")
    ap.add_argument("--algorithm", default="apf", choices=["orca", "apf"], help="drones' path planner")
    ap.add_argument("--instance", type=int, default=0, help="swarm instance to use (0 = default)")
    ap.add_argument("--parallel", type=int, default=1,
                    help="runs at once, each on its own swarm instance (instances 1..P); mind host CPU")
    ap.add_argument("--env", nargs="+", default=[], metavar="KEY=VALUE",
                    help="extra swarm environment for every run, e.g. CLOCK_DRIFT_SPREAD_PPM=20")
    ap.add_argument("--out", default=str(REPO / "results" / time.strftime("scaling_%Y%m%d_%H%M%S")))
    args = ap.parse_args()
    extra_env = parse_env(args.env)

    conditions = dict((c.split("=", 1) + [""])[:2] for c in args.conditions)
    outdir = Path(args.out)
    outdir.mkdir(parents=True, exist_ok=True)
    arp_check(max(int(size.split(":")[0]) for size in args.sizes), args.parallel)
    done, lock = [], threading.Lock()

    def job(size, cond, netem, rep):
        n, rtf = size.split(":")
        run = copy.copy(args)
        run.drones, run.rtf = int(n), float(rtf)
        run.threats = max(1, round(run.drones * args.threats_per_drone))
        run.interval = args.load_s / run.drones
        run.sample = True

        def go(inst):
            compose_down(inst)
            name = f"n{run.drones}_{cond}_r{rep}"
            try:
                row = run_one(run, args.profile, cond, netem, rep, outdir, name=name, extra_env=extra_env,
                              inst=inst)
            except Exception as e:
                log(f"run failed: {e}", inst)
                row = {"profile": args.profile, "condition": cond, "rep": rep, "rtf": run.rtf,
                       "drones": run.drones, "threats": run.threats, "instance": inst.k}
            with lock:
                done.append(row)
                with open(outdir / "results.csv", "w", newline="") as f:
                    w = csv.DictWriter(f, fieldnames=SCALE_COLUMNS, extrasaction="ignore")
                    w.writeheader()
                    w.writerows(done)
            return row
        return go

    jobs = [job(size, cond, netem, rep) for rep in range(1, args.repeats + 1)
            for size in args.sizes for cond, netem in conditions.items()]
    rows = run_jobs(jobs, args.parallel, args.instance)
    md = ["| " + " | ".join(SCALE_COLUMNS) + " |", "|" + "---|" * len(SCALE_COLUMNS)]
    md += ["| " + " | ".join(str(r.get(k, "")) for k in SCALE_COLUMNS) + " |" for r in rows]
    (outdir / "results.md").write_text("\n".join(md) + "\n")
    print("\n".join(md))
    log(f"results in {outdir}")


if __name__ == "__main__":
    main()
