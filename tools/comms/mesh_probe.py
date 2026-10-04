"""Radio mesh under loss (run by mesh_probe.sh): do peer sessions flap, and do subscriptions keep working?

Every peer publishes a heartbeat on mesh/hb/{me} at 2 Hz and subscribes to mesh/hb/* at start.  After
the impairment is applied (START_S), every LATE_EVERY_S each peer also declares a fresh subscription to
mesh/late/{n} (a topic all peers publish on at 2 Hz) and watches it for WATCH_S.  Reports, as one JSON
line: session closes that reopened (flaps), sessions closed at the end, per-sender heartbeat delivery,
and late subscriptions that heard a given sender nothing within WATCH_S.
"""
import json
import os
import threading
import time

from links import open_radio

ME = os.environ["NAME"]
PEERS = os.environ["PEER_NAMES"].split(",")
DURATION = float(os.environ.get("DURATION", "60"))
START_S = float(os.environ.get("START_S", "8"))
LATE_EVERY_S = float(os.environ.get("LATE_EVERY_S", "2"))
WATCH_S = float(os.environ.get("WATCH_S", "3"))


def main():
    if os.environ.get("RADIO_PROCESS") == "1":
        from radio_process import RadioProcess
        s = RadioProcess()
        session = None
    else:
        s = session = open_radio()
    lock = threading.Lock()
    hb_rx = {p: 0 for p in PEERS if p != ME}
    hb_last = {}                    # sender -> monotonic time of its last heartbeat
    late = {}                       # n -> (declared_at, {sender: first rx})
    t0 = time.monotonic()

    def on_hb(sample):
        p = str(sample.key_expr).rsplit("/", 1)[-1]
        if p != ME and time.monotonic() - t0 >= START_S:
            with lock:
                hb_rx[p] = hb_rx.get(p, 0) + 1
                hb_last[p] = time.monotonic()

    def on_late(n):
        def cb(sample):
            p = json.loads(bytes(sample.payload))["from"]
            with lock:
                late[n][1].setdefault(p, time.monotonic() - late[n][0])
        return cb

    s.declare_subscriber("mesh/hb/*", on_hb)
    pub_hb = s.declare_publisher(f"mesh/hb/{ME}")
    pub_late = {}
    flaps, known, last_known = [0], set(), [set()]

    def watch_sessions():
        while session is not None:
            now = {str(z) for z in session.info.peers_zid()}
            if time.monotonic() - t0 >= START_S:
                flaps[0] += len(last_known[0] - now)
            last_known[0] = now
            time.sleep(0.1)
    threading.Thread(target=watch_sessions, daemon=True).start()

    sent = 0
    n_next, next_late = 0, t0 + START_S + 1.0
    while time.monotonic() - t0 < DURATION:
        tick = time.monotonic()
        if tick - t0 >= START_S:
            sent += 1
        pub_hb.put(json.dumps({"from": ME}))
        # every peer publishes on every late topic opened so far (and the next one, so it is ready)
        for n in range(n_next + 1):
            p = pub_late.get(n)
            if p is None:
                p = pub_late[n] = s.declare_publisher(f"mesh/late/{n}")
            p.put(json.dumps({"from": ME}))
        if tick >= next_late and tick - t0 < DURATION - WATCH_S - 1:
            with lock:
                late[n_next] = (time.monotonic(), {})
            s.declare_subscriber(f"mesh/late/{n_next}", on_late(n_next))
            n_next += 1
            next_late += LATE_EVERY_S
        time.sleep(max(0.0, 0.5 - (time.monotonic() - tick)))

    others = [p for p in PEERS if p != ME]
    with lock:
        dead_list = [(n, p) for n, (_, got) in late.items() for p in others
                     if p not in got or got[p] > WATCH_S]
        dead = len(dead_list)
        trials = len(late) * len(others)
        end = time.monotonic()
        hb_dead = [p for p in others if end - hb_last.get(p, 0.0) > 5.0]
    print(json.dumps({"name": ME, "flaps": flaps[0], "sessions_end": len(last_known[0]),
                      "hb_delivery": round(sum(hb_rx.values()) / max(1, sent * len(others)), 3),
                      "late_trials": trials, "late_dead": dead,
                      "hb_dead_pairs": len(hb_dead),
                      "dead_list": dead_list if os.environ.get("DEAD_LIST") else None}), flush=True)
    os._exit(0)


if __name__ == "__main__":
    main()
