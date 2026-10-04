"""Sensing and ship-link sweep: the same seeded scenario under UWB, radar and ship-link degradation.

Each condition is a set of swarm environment variables (the simulator's impairment knobs) and/or an
impairment of the ship's radio link. The impairments make the environment worse than the hardware
record says; the drones keep assuming the record (their filters still use its UWB sigma), which is
what a real degradation looks like.

UWB (simulator):    UWB_EXTRA_SIGMA_M  extra Gaussian range noise
                    UWB_NLOS_P, UWB_NLOS_BIAS_M  non-line-of-sight ranges: positive bias, exponential mean
                    UWB_DROPOUT_P      lost exchanges (replaces the record's)
                    UWB_MAX_RANGE_M    shorter range (replaces the record's)
                    UWB_JAM            what:t0:t1[:x:y:r] windows (anchors|peers|all)
Radar (simulator):  RADAR_RANGE_SIGMA_M, RADAR_AZ_SIGMA_DEG, RADAR_EL_SIGMA_DEG  noise (replace the record's)
                    RADAR_MISS_P       each real contact missed per scan
                    RADAR_CLUTTER      mean false contacts per scan, uniform in range
                    RADAR_LATENCY_S    scan latency
Ship link (tc netem, degrade_radio.sh): the ship's own UDP and the drones' UDP addressed to the ship, so
                    drone-to-drone links are untouched; for the whole run, or a window of sim time.

    python3 tools/comms/sensing_sweep.py --list
    python3 tools/comms/sensing_sweep.py                                   # every condition, 15 drones, 8 threats, 3x
    python3 tools/comms/sensing_sweep.py --conditions baseline uwb_nlos ship_outage --repeats 3
    python3 tools/comms/sensing_sweep.py --custom wet="UWB_NLOS_P=0.2 RADAR_CLUTTER=1" --conditions wet

Results: <out>/results.csv, <out>/results.md (key metrics per run, then mean ± sd per condition),
<out>/<run>/{swarm.log,operator.log,summary.json,state.json,impairments.log}
"""

import argparse
import csv
import json
import statistics
import subprocess
import threading
import time
from pathlib import Path

import degradation_sweep as ds

HERE = Path(__file__).resolve().parent

# name -> (description, env, ship-link netem, ship-link window in sim s or None for the whole run)
CONDITIONS = {
    "baseline": ("no impairment", {}, None, None),
    # UWB
    "uwb_noise": ("UWB range noise 0.32 m (record: 0.1 m)", {"UWB_EXTRA_SIGMA_M": "0.3"}, None, None),
    "uwb_nlos": ("10 % of UWB ranges non-line-of-sight, +1 m mean bias", {"UWB_NLOS_P": "0.1", "UWB_NLOS_BIAS_M": "1.0"}, None, None),
    "uwb_dropout": ("half of all UWB exchanges lost", {"UWB_DROPOUT_P": "0.5"}, None, None),
    "uwb_short": ("UWB range 80 m (record: 250 m): far stations and intercepts need peers",
                  {"UWB_MAX_RANGE_M": "80"}, None, None),
    "uwb_jam_local": ("anchors jammed within 40 m of (80, 0) from t = 60 s", {"UWB_JAM": "anchors:60:100000:80:0:40"}, None, None),
    "uwb_jam_all": ("all UWB (anchors and peers) jammed for t = 60-120 s", {"UWB_JAM": "all:60:120"}, None, None),
    # ship link
    "ship_loss30": ("ship link 30 % loss, both ways, whole run", {}, "loss 30%", None),
    "ship_outage": ("ship link cut for t = 90-120 s", {}, "loss 100%", (90.0, 120.0)),
    # radar
    "radar_noise": ("radar noise 0.3 m, 6 deg (record: 0.05 m, 2 deg)",
                    {"RADAR_RANGE_SIGMA_M": "0.3", "RADAR_AZ_SIGMA_DEG": "6", "RADAR_EL_SIGMA_DEG": "6"}, None, None),
    "radar_miss": ("radar misses 30 % of contacts per scan", {"RADAR_MISS_P": "0.3"}, None, None),
    "radar_clutter": ("radar: 2 false contacts per scan", {"RADAR_CLUTTER": "2"}, None, None),
    "radar_latency": ("radar scans 0.1 s late", {"RADAR_LATENCY_S": "0.1"}, None, None),
    # everything moderate at once
    # GNSS (README next steps, item 5).  The spoofer covers the eastern stations, not the ship: a spoofer
    # covering the ship too would shift both fixes alike, which cancels in the drone-minus-ship fix.
    "gnss_fallback_uwb_short": ("UWB range 80 m with GNSS on (compare uwb_short with GNSS=0)",
                                {"UWB_MAX_RANGE_M": "80"}, None, None),
    "gnss_jam": ("GNSS jammed everywhere from t = 60 s", {"GNSS_JAM": "60:100000"}, None, None),
    "gnss_spoof_ramp": ("GNSS spoofed within 60 m of (85, 0) from t = 60 s: 0.1 m/s ramp",
                        {"GNSS_SPOOF": "60:45:0.1:0:85:0:60"}, None, None),
    "gnss_spoof_step": ("GNSS spoofed within 60 m of (85, 0) from t = 60 s: 20 m step",
                        {"GNSS_SPOOF": "60:45:0:20:85:0:60"}, None, None),
    "gnss_spoof_uwb_jam": ("all UWB jammed t = 60-120 s and GNSS spoofed (0.1 m/s ramp) near (85, 0) from 60 s",
                           {"UWB_JAM": "all:60:120", "GNSS_SPOOF": "60:45:0.1:0:85:0:60"}, None, None),
    "combined": ("UWB noise + NLOS, radar noise + clutter, ship link 30 % loss",
                 {"UWB_EXTRA_SIGMA_M": "0.2", "UWB_NLOS_P": "0.05", "UWB_NLOS_BIAS_M": "1.0",
                  "RADAR_RANGE_SIGMA_M": "0.2", "RADAR_AZ_SIGMA_DEG": "4", "RADAR_EL_SIGMA_DEG": "4",
                  "RADAR_CLUTTER": "1"}, "loss 30%", None),
}

KEY = ["condition", "rep", "approved", "destroyed", "kill_ratio", "det_miss_m_mean", "det_miss_m_max", "det_reasons",
       "fuze_no_detection", "fuze_false_triggers", "missed_slots", "never_fully_assigned", "reannounces",
       "friendly_fire", "collisions", "min_separation_m", "min_ship_range_m", "loc_err_mean_m", "loc_err_p95_m",
       "loc_err_max_m", "loc_nees_mean", "loc_within95", "loc_relocks", "gnss_spoof_reports", "award_latency_ms_mean", "wall_s"]
AGGREGATE = ["destroyed", "kill_ratio", "det_miss_m_mean", "fuze_no_detection", "fuze_false_triggers", "missed_slots",
             "loc_err_p95_m", "loc_err_max_m", "loc_nees_mean", "min_separation_m"]


def _containers(inst):
    names = subprocess.run(["docker", "ps", "--format", "{{.Names}}"], capture_output=True, text=True).stdout.split()
    ship = f"{inst.prefix}ship"
    drones = sorted(n for n in names if n.startswith(f"{inst.project}-agent-"))
    return ship, drones


def _radio_ip(container, inst):
    """The container's address on the radio network."""
    import ipaddress
    net = ipaddress.ip_network(inst.cfg["RADIO_SUBNET"])
    out = subprocess.run(["docker", "inspect", "-f", "{{range .NetworkSettings.Networks}}{{.IPAddress}} {{end}}",
                          container], capture_output=True, text=True).stdout.split()
    return next(ip for ip in out if ip and ipaddress.ip_address(ip) in net)


def ship_link(mode, netem, env, inst):
    """Impair (mode=apply) or restore (clear) the ship's radio link both ways: the ship's own UDP, and the
    drones' UDP addressed to the ship. Drone-to-drone traffic is untouched."""
    ship, drones = _containers(inst)
    script = str(HERE / "degrade_radio.sh")
    if mode == "apply":
        subprocess.run([script, "apply", netem, ship], env=env, check=True, stdout=subprocess.DEVNULL)
        subprocess.run([script, "apply", netem] + drones, env=dict(env, DST_IP=_radio_ip(ship, inst)),
                       check=True, stdout=subprocess.DEVNULL)
    else:
        subprocess.run([script, "clear", ship] + drones, env=env, stdout=subprocess.DEVNULL)


def make_hook(netem, window):
    """run_one hook: the ship-link impairment for the whole run, or for a window of sim time."""
    if not netem:
        return None

    def hook(env, inst, rundir):
        logf = open(rundir / "impairments.log", "a")

        def note(msg):
            logf.write(f"{time.strftime('%H:%M:%S')} {msg}\n")
            logf.flush()
            ds.log(msg, inst)
        if window is None:
            ship_link("apply", netem, env, inst)
            note(f"ship link: {netem} (whole run)")
            return lambda: (ship_link("clear", None, env, inst), logf.close())
        stop = threading.Event()
        state = {"on": False}

        def sim_time():
            try:
                return float(ds.get("/api/state", inst=inst).get("sim_time") or 0.0)
            except Exception:
                return None

        def run():
            t0, t1 = window
            while not stop.is_set():
                t = sim_time()
                if t is not None and not state["on"] and t0 <= t < t1:
                    ship_link("apply", netem, env, inst)
                    state["on"] = True
                    note(f"ship link: {netem} at sim t = {t:.1f}")
                elif t is not None and state["on"] and t >= t1:
                    ship_link("clear", None, env, inst)
                    state["on"] = False
                    note(f"ship link restored at sim t = {t:.1f}")
                    return
                stop.wait(0.5)
        th = threading.Thread(target=run, daemon=True)
        th.start()

        def cleanup():
            stop.set()
            th.join(timeout=5)
            if state["on"]:
                ship_link("clear", None, env, inst)
            logf.close()
        return cleanup
    return hook


def aggregate(rows):
    cells = {}
    for r in rows:
        cells.setdefault(r["condition"], []).append(r)
    md = ["| condition | runs | " + " | ".join(AGGREGATE) + " |", "|" + "---|" * (len(AGGREGATE) + 2)]
    for cond, rs in cells.items():
        out = []
        for k in AGGREGATE:
            vals = [float(r[k]) for r in rs if r.get(k) is not None]
            out.append("" if not vals else f"{vals[0]:g}" if len(vals) == 1
                       else f"{statistics.mean(vals):.3g} ± {statistics.stdev(vals):.2g}")
        ok = sum(1 for r in rs if r.get("approved") is not None)
        md.append(f"| {cond} | {ok}/{len(rs)} | " + " | ".join(out) + " |")
    return md


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--conditions", nargs="+", help="condition names (default: all; see --list)")
    ap.add_argument("--custom", nargs="+", default=[], metavar='NAME="K=V K=V"',
                    help="extra conditions from swarm environment variables")
    ap.add_argument("--list", action="store_true", help="list the conditions and exit")
    ap.add_argument("--repeats", type=int, default=1)
    ap.add_argument("--rtf", type=float, default=3.0)
    ap.add_argument("--drones", type=int, default=15)
    ap.add_argument("--threats", type=int, default=8)
    ap.add_argument("--maneuver-p", type=float, default=0.0)
    ap.add_argument("--interval", type=float, default=30)
    ap.add_argument("--first", type=float, default=25)
    ap.add_argument("--seed", type=int, default=42)
    ap.add_argument("--reaction", type=float, default=3.0, help="operator reaction, sim seconds")
    ap.add_argument("--timeout", type=float, default=420, help="sim seconds per run after the swarm is up")
    ap.add_argument("--auto-approve", action="store_true", help="the ship approves threats itself")
    ap.add_argument("--algorithm", default="apf", choices=["orca", "apf"])
    ap.add_argument("--instance", type=int, default=0)
    ap.add_argument("--parallel", type=int, default=1)
    ap.add_argument("--env", nargs="+", default=[], metavar="KEY=VALUE", help="extra environment for every run")
    ap.add_argument("--out", default=str(ds.REPO / "results" / time.strftime("sensing_%Y%m%d_%H%M%S")))
    args = ap.parse_args()

    conditions = dict(CONDITIONS)
    for c in args.custom:
        name, _, kv = c.partition("=")
        conditions[name] = (kv, ds.parse_env(kv.split()), None, None)
    if args.list:
        for name, (desc, env, netem, window) in conditions.items():
            link = f"; ship link {netem}" + (f" for t = {window[0]:g}-{window[1]:g} s" if window else "") if netem else ""
            print(f"{name:15s} {desc}{link}  {' '.join(f'{k}={v}' for k, v in env.items())}")
        return
    names = args.conditions or list(conditions)
    unknown = [n for n in names if n not in conditions]
    if unknown:
        ap.error(f"unknown condition(s): {', '.join(unknown)} (see --list)")
    base_env = ds.parse_env(args.env)
    outdir = Path(args.out)
    outdir.mkdir(parents=True, exist_ok=True)
    ds.arp_check(args.drones, args.parallel)
    done, lock = [], threading.Lock()

    def job(cond, rep):
        desc, env, netem, window = conditions[cond]

        def run(inst):
            ds.compose_down(inst)
            try:
                row = ds.run_one(args, "default", cond, "", rep, outdir, name=f"{cond}_r{rep}",
                                 extra_env={**base_env, **env}, inst=inst, hook=make_hook(netem, window))
            except Exception as e:
                ds.log(f"run failed: {e}", inst)
                row = {"condition": cond, "rep": rep}
            row["condition"] = cond
            with lock:
                done.append(row)
                with open(outdir / "results.csv", "w", newline="") as f:
                    w = csv.DictWriter(f, fieldnames=ds.COLUMNS, extrasaction="ignore")
                    w.writeheader()
                    w.writerows(done)
            return row
        return run

    jobs = [job(c, rep) for rep in range(1, args.repeats + 1) for c in names]
    rows = ds.run_jobs(jobs, args.parallel, args.instance)
    md = [f"Sensing sweep: {args.drones} drones, {args.threats} threats, --rtf {args.rtf:g}, seed {args.seed}", ""]
    md += ["| " + " | ".join(KEY) + " |", "|" + "---|" * len(KEY)]
    md += ["| " + " | ".join(str(r.get(k, "")) for k in KEY) + " |" for r in rows]
    md += ["", "Mean ± sd per condition:", ""] + aggregate(rows)
    md += ["", "Conditions:", ""] + [f"- `{n}`: {conditions[n][0]}" for n in names]
    (outdir / "results.md").write_text("\n".join(md) + "\n")
    (outdir / "conditions.json").write_text(json.dumps({n: conditions[n] for n in names}, indent=2))
    print("\n".join(md))
    ds.log(f"results in {outdir}")


if __name__ == "__main__":
    main()
