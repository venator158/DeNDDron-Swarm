"""Threat dispatcher for the naval-defence scenario.

Generates waves of incoming threats and announces the ones the swarm can
afford on ``swarm/threats``.  A threat's level is its priority and the number
of drones it needs, and drones are expended on intercept, so the dispatcher
enforces

    drones committed (engaged or still owed to open threats) <= drones alive

by admitting threats in priority order only while they fit the free budget.
Threats that do not fit are reported as unengaged.

Watches ``drone/*/sensors`` (liveness), ``swarm/awards`` and
``swarm/intercepts``.  A threat left under-assigned (divergent bids, yields)
is re-announced for the missing drones.  Prints a summary and exits when all
waves are resolved, the swarm is exhausted, or the timeout hits.
"""

import argparse
import json
import logging
import os
import random
import threading
import time

import zenoh

from threats import DEFAULT_THREAT_MIX, Threat, admit, parse_mix, random_threats

logging.basicConfig(level=logging.INFO, format="%(asctime)s [ThreatDispatcher] %(message)s", datefmt="%H:%M:%S")
log = logging.getLogger("ThreatDispatcher")

ALIVE_TIMEOUT_S = 2.0


def _env(name, default, cast):
    return cast(os.environ.get(name, default))


class _Tracked:
    def __init__(self, threat: Threat):
        self.threat = threat
        self.holders = set()       # drones currently engaged
        self.intercepted = set()   # drones expended on this threat
        self.announces = 1
        self.last_announce = time.monotonic()
        self.abandoned = False

    @property
    def assigned(self) -> int:
        return len(self.holders | self.intercepted)

    @property
    def unfilled(self) -> int:
        return 0 if self.abandoned else max(0, self.threat.level - self.assigned)

    @property
    def neutralized(self) -> bool:
        return len(self.intercepted) >= self.threat.level

    @property
    def active(self) -> bool:
        return not self.neutralized and not self.abandoned


def main():
    mix_default = ",".join(f"{k}:{lvl}:{w}" for k, (lvl, w) in DEFAULT_THREAT_MIX.items())
    ap = argparse.ArgumentParser(description="DeNDDron naval-defence threat dispatcher")
    ap.add_argument("--zenoh-router", default=os.environ.get("ZENOH_ROUTER_IP"))
    ap.add_argument("--waves", type=int, default=_env("THREAT_WAVES", 3, int))
    ap.add_argument("--max-threats-per-wave", type=int, default=_env("THREATS_PER_WAVE", 4, int))
    ap.add_argument("--mix", default=os.environ.get("THREAT_MIX") or mix_default,
                    help="type:level:weight,...  (level = priority = drones required)")
    ap.add_argument("--seed", type=int, default=_env("THREAT_SEED", 42, int))
    ap.add_argument("--initial-delay", type=float, default=_env("THREAT_INITIAL_DELAY_S", 25.0, float),
                    help="wall seconds to let drones join and settle before the first wave")
    ap.add_argument("--interval", type=float, default=_env("THREAT_INTERVAL_S", 15.0, float),
                    help="wall seconds between waves")
    ap.add_argument("--retry-after", type=float, default=_env("THREAT_RETRY_AFTER_S", 5.0, float),
                    help="wall seconds before an under-assigned threat is re-announced")
    ap.add_argument("--max-announces", type=int, default=_env("THREAT_MAX_ANNOUNCES", 4, int))
    ap.add_argument("--timeout", type=float, default=_env("THREAT_TIMEOUT_S", 400.0, float))
    ap.add_argument("--min-radius", type=float, default=25.0)
    ap.add_argument("--max-radius", type=float, default=45.0)
    ap.add_argument("--min-z", type=float, default=14.0)
    ap.add_argument("--max-z", type=float, default=26.0)
    args = ap.parse_args()

    rng = random.Random(args.seed)
    mix = parse_mix(args.mix)

    conf = zenoh.Config()
    if args.zenoh_router:
        conf.insert_json5("connect/endpoints", f'["{args.zenoh_router}"]')
    session = zenoh.open(conf)

    lock = threading.Lock()
    last_seen = {}      # drone -> monotonic time of last sensor frame
    expended = set()
    tracked = {}        # threat_id -> _Tracked
    stats = {"double_assigned": set(), "invariant_violations": 0, "peak_committed": 0, "peak_alive_at": 0}

    def parse(sample):
        return json.loads(bytes(sample.payload).decode("utf-8"))

    def on_sensors(sample):
        # key: drone/<id>/sensors; the payload is never decoded (50 Hz per drone).
        parts = str(sample.key_expr).split("/")
        if len(parts) == 3:
            with lock:
                last_seen[parts[1]] = time.monotonic()

    def on_award(sample):
        try:
            a = parse(sample)
            tid, agent, status = a["threat_id"], a["agent_id"], a.get("status", "engaged")
        except Exception as e:
            log.warning("bad award: %s", e)
            return
        with lock:
            tr = tracked.get(tid)
            if tr is None:
                return
            if status == "engaged":
                tr.holders.add(agent)
                if tr.assigned > tr.threat.level:
                    stats["double_assigned"].add(tid)
            else:
                tr.holders.discard(agent)
        log.info("award: %s %s %s", tid, status, agent)

    def on_intercept(sample):
        try:
            m = parse(sample)
            tid, agent = m["threat_id"], m["agent_id"]
        except Exception as e:
            log.warning("bad intercept: %s", e)
            return
        with lock:
            expended.add(agent)
            last_seen.pop(agent, None)
            tr = tracked.get(tid)
            if tr is not None:
                tr.holders.discard(agent)
                tr.intercepted.add(agent)
                done = tr.neutralized
            else:
                done = False
        log.info("intercept: %s by %s%s", tid, agent, " -> NEUTRALIZED" if done else "")

    subs = [
        session.declare_subscriber("drone/*/sensors", on_sensors),
        session.declare_subscriber("swarm/awards", on_award),
        session.declare_subscriber("swarm/intercepts", on_intercept),
    ]
    pub = session.declare_publisher("swarm/threats")

    def alive_drones(now):
        return {d for d, t in last_seen.items() if now - t <= ALIVE_TIMEOUT_S and d not in expended}

    def committed():
        engaged = set()
        owed = 0
        for tr in tracked.values():
            if tr.active:
                engaged |= tr.holders
                owed += tr.unfilled
        return len(engaged), owed

    def announce(wave_id, threats):
        pub.put(json.dumps({"wave_id": wave_id, "threats": [t.to_dict() for t in threats]}))

    wave_log = []   # (wave_id, admitted, unengaged, free_before)
    log.info("waiting %.0fs for drones before %d waves (seed %d, mix %s)",
             args.initial_delay, args.waves, args.seed, args.mix)
    time.sleep(args.initial_delay)

    started = time.monotonic()
    next_wave = started
    waves_sent = 0
    retry_seq = 0
    next_threat_num = 1
    reason = "timeout"
    try:
        while time.monotonic() - started < args.timeout:
            now = time.monotonic()
            with lock:
                alive = alive_drones(now)
                engaged, owed = committed()
                free = len(alive) - engaged - owed
                stats["peak_committed"] = max(stats["peak_committed"], engaged + owed)
                if engaged + owed > len(alive):
                    stats["invariant_violations"] += 1

                # --- new wave ---------------------------------------------------
                if waves_sent < args.waves and now >= next_wave:
                    waves_sent += 1
                    n = rng.randint(1, max(1, args.max_threats_per_wave))
                    candidates = random_threats(rng, mix, n, next_threat_num, args.min_radius,
                                                args.max_radius, args.min_z, args.max_z)
                    next_threat_num += n
                    admitted, unengaged = admit(candidates, free)
                    wave_id = f"W{waves_sent}"
                    for t in admitted:
                        tracked[t.threat_id] = _Tracked(t)
                    wave_log.append((wave_id, admitted, unengaged, free, len(alive)))
                    next_wave = now + args.interval
                    log.info("%s: %d threats, alive=%d free=%d -> admit %s | unengaged %s", wave_id, n, len(alive),
                             free, [f"{t.threat_id}:{t.type}/L{t.level}" for t in admitted],
                             [f"{t.threat_id}:{t.type}/L{t.level}" for t in unengaged])
                    if admitted:
                        announce(wave_id, admitted)

                # --- re-announce under-assigned threats ------------------------
                retry = []
                for tr in tracked.values():
                    if not tr.active or tr.unfilled == 0 or now - tr.last_announce < args.retry_after:
                        continue
                    if tr.announces >= args.max_announces:
                        tr.abandoned = True
                        log.warning("abandoning %s: still needs %d drones after %d announces",
                                    tr.threat.threat_id, tr.threat.level - tr.assigned, tr.announces)
                        continue
                    tr.announces += 1
                    tr.last_announce = now
                    t = tr.threat
                    retry.append(Threat(t.threat_id, t.type, t.level, t.level - tr.assigned, t.location))
                if retry:
                    retry_seq += 1
                    wave_id = f"R{retry_seq}"
                    log.info("%s: re-announce %s", wave_id, [f"{t.threat_id} need {t.required}" for t in retry])
                    announce(wave_id, retry)

                all_waves_done = waves_sent >= args.waves and not any(tr.active for tr in tracked.values())
                exhausted = waves_sent > 0 and not alive and not any(tr.active for tr in tracked.values())
            if all_waves_done:
                reason = "all waves resolved"
                break
            if exhausted:
                reason = "swarm exhausted"
                break
            time.sleep(0.25)
    except KeyboardInterrupt:
        reason = "interrupted"

    with lock:
        alive_end = len(alive_drones(time.monotonic()))
        all_tracked = dict(tracked)
        n_expended = len(expended)
    neutralized = [t for t in all_tracked.values() if t.neutralized]
    abandoned = [t for t in all_tracked.values() if t.abandoned]
    unengaged = [t for _, _, un, _, _ in wave_log for t in un]
    log.info("SUMMARY (%s): %d waves, %d threats generated", reason, len(wave_log),
             sum(len(a) + len(u) for _, a, u, _, _ in wave_log))
    log.info("  engaged=%d neutralized=%d abandoned=%d unengaged(no budget)=%d",
             len(all_tracked), len(neutralized), len(abandoned), len(unengaged))
    log.info("  drones expended=%d alive at end=%d peak committed=%d",
             n_expended, alive_end, stats["peak_committed"])
    log.info("  invariant (committed <= alive) violations=%d, threats with more drones than level=%s",
             stats["invariant_violations"], sorted(stats["double_assigned"]) or 0)
    for wave_id, adm, un, free, alive in wave_log:
        log.info("  %s alive=%d free=%d admitted=%s unengaged=%s", wave_id, alive, free,
                 [f"{t.threat_id}:{t.type}/L{t.level}" + ("" if all_tracked[t.threat_id].neutralized else "(open)")
                  for t in adm],
                 [f"{t.threat_id}:{t.type}/L{t.level}" for t in un])

    for s in subs:
        s.undeclare()
    pub.undeclare()
    session.close()
    ok = not abandoned and stats["invariant_violations"] == 0 and not stats["double_assigned"] \
        and all(not t.active for t in all_tracked.values())
    raise SystemExit(0 if ok else 1)


if __name__ == "__main__":
    main()
