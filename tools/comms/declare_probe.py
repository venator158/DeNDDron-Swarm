"""Zenoh declaration probe (run by declare_probe.sh): do late subscriber declarations and first puts on
fresh publishers survive a lossy UDP radio?  Two peers, PUB and SUB; the shell script impairs one side.

late_sub:   PUB puts on keys late/0..N-1 every 0.1 s.  SUB declares a subscriber on late/i every 0.5 s and
            watches it for WATCH_S.  Impair SUB's egress only: data from PUB is then never lost, so a
            subscriber that hears nothing means its declaration never reached PUB.
fresh_pub:  SUB subscribes to fresh/* at start, before the impairment.  PUB declares a new publisher
            fresh/i every 0.5 s and puts on it 5 times, 0.1 s apart.  Impair PUB's egress: the first
            put should be delivered as often as the later ones.
"""
import json
import os
import threading
import time

from links import open_radio

N = int(os.environ.get("TRIALS", "40"))
WATCH_S = float(os.environ.get("WATCH_S", "3"))
START_S = float(os.environ.get("START_S", "10"))   # sessions up and impairment applied by then


def main():
    role, test = os.environ["ROLE"], os.environ["TEST"]
    s = open_radio(os.environ.get("RADIO_SUBNET"))
    t0 = time.monotonic()
    if role == "pub":
        time.sleep(START_S)
        if test == "late_sub":
            pubs = [s.declare_publisher(f"late/{i}") for i in range(N)]
            end = time.monotonic() + N * 0.5 + WATCH_S + 2
            while time.monotonic() < end:
                for i, p in enumerate(pubs):
                    p.put(json.dumps({"i": i}))
                time.sleep(0.1)
        else:
            for i in range(N):
                p = s.declare_publisher(f"fresh/{i}")
                for k in range(5):
                    p.put(json.dumps({"i": i, "k": k}))
                    time.sleep(0.1)
            time.sleep(3)
        print("pub done", flush=True)
        os._exit(0)

    lock = threading.Lock()
    if test == "late_sub":
        first, count, declared = {}, {}, {}

        def cb(i):
            def on(sample):
                with lock:
                    count[i] = count.get(i, 0) + 1
                    first.setdefault(i, time.monotonic() - declared[i])
            return on

        time.sleep(START_S + 1.0)          # PUB is publishing every key by now
        subs = []
        for i in range(N):
            declared[i] = time.monotonic()
            subs.append(s.declare_subscriber(f"late/{i}", cb(i)))
            time.sleep(0.5)
        time.sleep(WATCH_S)
        heard = [i for i in range(N) if i in first]
        lat = sorted(first.values())
        print(json.dumps({"test": test, "trials": N, "heard": len(heard), "never": N - len(heard),
                          "first_rx_s_median": lat[len(lat) // 2] if lat else None,
                          "first_rx_s_max": lat[-1] if lat else None,
                          "never_heard": [i for i in range(N) if i not in first],
                          "first_rx_s_all": [round(x, 2) for x in lat]}), flush=True)
    else:
        got = {}

        def on(sample):
            m = json.loads(bytes(sample.payload))
            with lock:
                got.setdefault(m["i"], set()).add(m["k"])

        s.declare_subscriber("fresh/*", on)
        time.sleep(START_S + N * 0.5 + 4)
        first = sum(1 for i in range(N) if 0 in got.get(i, ()))
        later = sum(len(got.get(i, set()) - {0}) for i in range(N))
        print(json.dumps({"test": test, "trials": N, "first_put_delivered": first / N,
                          "later_puts_delivered": later / (4 * N),
                          "keys_never_heard": sum(1 for i in range(N) if i not in got)}), flush=True)
    os._exit(0)


if __name__ == "__main__":
    main()
