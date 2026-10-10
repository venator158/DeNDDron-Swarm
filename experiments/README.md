# Experiments

One file per experiment. Each records its full configuration, the claim it supports,
and which metrics answer it — so an experiment can be re-run from the file rather than
reconstructed from someone's terminal history. Results land in `results/<experiment>/`
alongside an `experiment.txt` stamped with the commit, host and parameters that produced them.

```bash
docker compose build                      # images must match the code; the scripts check
bash experiments/fuze_ab.sh               # 3 repeats per arm, seed 42
REPEATS=5 bash experiments/fuze_ab.sh     # more repeats where a cell varies
```

| Script | Question it tests | Result so far | Runtime |
|---|---|---|---|
| `legacy_split.sh` | which legacy setting causes the old-vs-new difference? | 8 Oct: the no-fly zone (miss 1.19 → 0.38 m); localization and perception change nothing | ~20 min |
| `fuze_ab.sh` | does the proximity fuze cut miss distance? | 8 Oct: −59 % at 8 drones, −18 % at 50; kills unchanged; `radius` 7.44 m vs `cpa` 1.21 m | ~35 min |
| `rdl_vs_coop.sh` | does RDL beat `coop` beyond anchor range (`uwb_short`)? | 8 Oct, GNSS off: 8/8 vs 7.33 vs 6 (anchors); p95 0.78 vs 9.2 m; still overconfident (NEES 15) | ~40 min |
| `planner_ab.sh` | does ORCA dead-end in crowds where APF does not? | 8 Oct: no dead-end at 50; ORCA worst miss 5.85 vs 6.96 m, but 3.6× the close calls | ~35 min |
| `clock_sync.sh` | which sync mode under skewed clocks? | 10 Oct, 3 runs: `none` 3/4 every run (fuze never detects); ttg, master, consensus 4/4 every run; consensus bound covers 99%, master 82–92% | ~25 min |
| `radio_degradation.sh` | kill rate under loss and delay; where is the bandwidth cliff? | 10 Oct, 3 runs: loss up to 30% and 200 ms delay 4/4; 64 and 48 kbit/s 4/4; cliff below: 32 kbit/s 2/4, 24 kbit/s 1/4, identical in every run | ~40 min |
| `scaling.sh` | what limits swarm size? | not yet run as a script (4–6 Oct: density, not CPU; 100 drones exceeds 15 GB) | ~60 min |
| `scale_stress.sh` | do the radio and clock findings hold at 50 drones? | 10 Oct, 3 runs: no impairment 23–24/24; 256 and 128 kbit/s collapse to 2–5/25 (cliff far above the 8-drone one); skewed `none` 8–9/25, `consensus` 24/24 every run | ~75 min |

Times assume `REPEATS=3` and one cell at a time.

## Conventions

- **Seed 42 everywhere.** Fixes the spawn layout and the threat script, so two cells
  differ only in the variable under test. Override with `SEED=`.
- **3 repeats minimum.** A single run misleads: at 75 drones three runs lost a threat
  and the fourth did not. Three is enough to *detect* variance; raise `REPEATS` for
  any cell that shows it.
- **`--rtf 3` for outcome metrics, real time for anything else.** Kill counts and
  second-scale latencies survive the speed-up. CPU, loop timing and messages/s stay on
  the real clock and are **not** comparable across `--rtf` values; millisecond clock
  measurements are distorted, because real processing latency does not scale.
- **Results are disposable, scripts are not.** `results/` is git-ignored. If a number
  matters, it belongs in the paper or the README, not left in `results/`.

## Two things that have bitten us

**Stale images.** The sweeps run whatever images exist, so an out-of-date build silently
tests old code. `check_images` compares the agent image against the last commit touching
`src/`, `sim/` or `docker/` and refuses to run. Override with `SKIP_IMAGE_CHECK=1` only
when you mean it.

**Orphaned swarms.** Compose sets `restart: unless-stopped`, so containers outlive a
driver that dies — an interrupted 100-drone run once left 92 containers up for two days.
Every script traps `EXIT`/`INT`/`TERM` and tears every project down.

## Adding an experiment

Copy the shape of `fuze_ab.sh`: source `_common.sh`, set `EXPERIMENT` and `CLAIM`, call
`check_images` and `need_arp <max drones>`, then `run_sweep <outdir> <command...>`. The
header comment should say what claim the experiment supports and which metrics answer it —
that comment is what makes the file readable a month later.
