#!/usr/bin/env python3
"""Fan-out probe: bytes a radio actually sends vs bytes the drone software publishes.

For each swarm size: start the swarm with no threats (routine traffic only), wait for the full roster,
let it settle, then over a window read every container's radio-interface counters
(/sys/class/net/<radio>/statistics/tx_bytes, tx_packets) and the drones' own telemetry
`tx_bytes_per_s` (payload bytes handed to Zenoh).  The ratio wire / app shows how many copies of
each publication go out; if the radio sends one copy per connected peer, the ratio grows with N.

    python3 tools/comms/fanout_probe.py --sizes 8 50
    python3 tools/comms/fanout_probe.py --sizes 8 --window 30

Results: <out>/results.csv, <out>/results.md, <out>/n<N>/{swarm.log,state.json}
"""
import argparse
import csv
import statistics as st
import subprocess
import sys
import time
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
from degradation_sweep import REPO, DEFAULT_INST, compose_down, get, log, wait_for_roster  # noqa: E402

COUNTERS = r"""
import ipaddress, os, socket, fcntl, struct
net = ipaddress.ip_network('%s')
for n in sorted(os.listdir('/sys/class/net')):
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        ip = socket.inet_ntoa(fcntl.ioctl(s.fileno(), 0x8915, struct.pack('256s', n[:15].encode()))[20:24])
    except OSError:
        continue
    if ipaddress.ip_address(ip) in net:
        b = '/sys/class/net/%%s/statistics/' %% n
        print(open(b + 'tx_bytes').read().strip(), open(b + 'tx_packets').read().strip(),
              open(b + 'rx_bytes').read().strip())
        break
"""


def containers(inst):
    names = subprocess.run(["docker", "ps", "--format", "{{.Names}}"], capture_output=True, text=True).stdout.split()
    agents = sorted(n for n in names if n.startswith(f"{inst.project}-agent-"))
    ship = [n for n in names if n == f"{inst.prefix}ship"]
    return agents, ship


def counters(name, subnet):
    out = subprocess.run(["docker", "exec", name, "python", "-c", COUNTERS % subnet],
                         capture_output=True, text=True, timeout=30).stdout.split()
    return tuple(int(x) for x in out) if len(out) == 3 else None


def snapshot(names, subnet):
    return {n: counters(n, subnet) for n in names}


def app_tx(state):
    """Mean telemetry tx_bytes_per_s over drones that report it (payload bytes, sim time)."""
    drones = state.get("drones", {})
    items = drones.values() if isinstance(drones, dict) else drones
    vals = [d.get("telemetry", {}).get("tx_bytes_per_s") for d in items if isinstance(d, dict)]
    vals = [v for v in vals if v is not None]
    return (st.mean(vals), len(vals)) if vals else (None, 0)


def run_size(n, args, outdir, inst=DEFAULT_INST):
    rundir = outdir / f"n{n}"
    rundir.mkdir(parents=True, exist_ok=True)
    log(f"=== {n} drones, routine traffic only")
    swarm_log = open(rundir / "swarm.log", "w")
    # One threat, scheduled far beyond the run: the radar stays quiet, so only routine traffic flows.
    swarm = subprocess.Popen(["bash", "scripts/run_swarm.sh", str(n), "--threats", "1", "--first-threat", "100000",
                              "--seed", str(args.seed)], cwd=REPO, stdout=swarm_log, stderr=subprocess.STDOUT)
    try:
        wait_for_roster(n, args.startup_timeout, inst=inst)
        time.sleep(args.settle)
        agents, ship = containers(inst)
        subnet = inst.cfg["RADIO_SUBNET"]
        t0 = time.time()
        a = snapshot(agents + ship, subnet)
        time.sleep(args.window)
        b = snapshot(agents + ship, subnet)
        dt = time.time() - t0
        state = get("/api/state", timeout=5, inst=inst)
        (rundir / "state.json").write_text(__import__("json").dumps(state, indent=2))
    finally:
        compose_down(inst)
        swarm.wait(timeout=60)
        swarm_log.close()

    def rates(names):
        r = [((b[x][0] - a[x][0]) / dt, (b[x][1] - a[x][1]) / dt, (b[x][2] - a[x][2]) / dt)
             for x in names if a.get(x) and b.get(x)]
        return r
    dr = rates(agents)
    sr = rates(ship)
    app, reporting = app_tx(state)
    wire = st.mean(x[0] for x in dr)
    row = {
        "drones": n,
        "window_s": round(dt, 1),
        "drones_measured": len(dr),
        "drone_wire_tx_Bps_mean": round(wire),
        "drone_wire_tx_Bps_max": round(max(x[0] for x in dr)),
        "drone_wire_tx_kbit_mean": round(wire * 8 / 1000, 1),
        "drone_wire_tx_pkts_mean": round(st.mean(x[1] for x in dr), 1),
        "drone_wire_rx_kbit_mean": round(st.mean(x[2] for x in dr) * 8 / 1000, 1),
        "drone_app_tx_Bps_mean": round(app) if app else None,
        "drones_reporting_app": reporting,
        "wire_over_app": round(wire / app, 1) if app else None,
        "ship_wire_tx_kbit": round(sr[0][0] * 8 / 1000, 1) if sr else None,
        "ship_wire_rx_kbit": round(sr[0][2] * 8 / 1000, 1) if sr else None,
    }
    log(f"{n} drones: {row}")
    return row


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--sizes", type=int, nargs="+", default=[8, 50])
    ap.add_argument("--window", type=float, default=60.0, help="wall seconds to count bytes over")
    ap.add_argument("--settle", type=float, default=30.0, help="wall seconds after the full roster")
    ap.add_argument("--startup-timeout", type=float, default=300.0)
    ap.add_argument("--seed", type=int, default=42)
    ap.add_argument("--out", default="results/fanout")
    args = ap.parse_args()
    outdir = Path(args.out)
    outdir.mkdir(parents=True, exist_ok=True)
    rows = [run_size(n, args, outdir) for n in args.sizes]
    with open(outdir / "results.csv", "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=list(rows[0]))
        w.writeheader()
        w.writerows(rows)
    with open(outdir / "results.md", "w") as f:
        f.write("| " + " | ".join(rows[0]) + " |\n|" + "---|" * len(rows[0]) + "\n")
        for r in rows:
            f.write("| " + " | ".join(str(v) for v in r.values()) + " |\n")
    print((outdir / "results.md").read_text())


if __name__ == "__main__":
    main()
