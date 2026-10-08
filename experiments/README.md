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

| Script | Claim it tests | Runtime |
|---|---|---|
| `fuze_ab.sh` | the proximity fuze halves miss distance, and makes clock sync mandatory | ~35 min |
| `rdl_vs_coop.sh` | RDL stays consistent where `coop` becomes overconfident (`uwb_short`) | ~40 min |
| `planner_ab.sh` | ORCA dead-ends in crowds; APF does not | ~35 min |
| `clock_sync.sh` | `none`/`ttg`/`master`/`consensus` under skewed clocks | ~25 min |
| `radio_degradation.sh` | kill rate under loss and delay; where the bandwidth cliff is | ~40 min |
| `scaling.sh` | density, not computation, limits swarm size | ~60 min |

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
