#!/usr/bin/env bash
# Shared setup for every experiment in this directory.  Sourced, not run.
#
#   source "$(dirname "${BASH_SOURCE[0]}")/_common.sh"
#   need_arp <max drones in this experiment>
#   run_sweep <tool> <args...>
#
# Why this exists: an experiment that cannot be re-run from a file is not a method,
# and an interrupted run used to leave its containers alive (compose sets
# restart: unless-stopped), which once left 92 of them running for two days.

set -euo pipefail
REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$REPO"

REPEATS="${REPEATS:-3}"          # runs per cell; override: REPEATS=5 bash experiments/<x>.sh
PARALLEL="${PARALLEL:-1}"        # cells at once (each is a full swarm; watch RAM)
SEED="${SEED:-42}"               # fixes spawn layout and the threat script

# --- teardown: never leave a swarm running -----------------------------------
teardown() {
  local rc=$?
  for p in denddron-swarm denddron-swarm-1 denddron-swarm-2 denddron-swarm-3 denddron-swarm-4; do
    docker compose -p "$p" down --remove-orphans >/dev/null 2>&1 || true
  done
  [[ $rc -ne 0 ]] && echo "!! experiment exited with code $rc; swarms torn down" >&2
  return $rc
}
trap teardown EXIT INT TERM

# --- the ARP ceiling ---------------------------------------------------------
# The radio is a full mesh and Linux keeps ONE ARP table for every container on
# the host, so N drones need about (N+1)*N entries.  Past gc_thresh3 the later
# drones reach nobody and the kernel log fills with "neighbor table overflow".
need_arp() {
  local drones=$1
  local need=$(( (drones + 1) * drones + 2 * (drones + 4) ))
  local have; have=$(cat /proc/sys/net/ipv4/neigh/default/gc_thresh3 2>/dev/null || echo 0)
  if (( have < need )); then
    cat >&2 <<MSG
!! $drones drones need ~$need ARP entries; this host allows $have.
   Raise it (needs root), then re-run:
     sudo sysctl -w net.ipv4.neigh.default.gc_thresh1=4096 \\
       net.ipv4.neigh.default.gc_thresh2=8192 net.ipv4.neigh.default.gc_thresh3=16384
   To survive a reboot, put the same three lines in /etc/sysctl.d/99-denddron-arp.conf
MSG
    exit 1
  fi
}

# --- images must match the code ----------------------------------------------
# The sweeps run whatever images exist.  Stale images silently test old code,
# which is how a whole round of results once got measured against a 4-day-old build.
check_images() {
  local img_epoch code_epoch
  local created
  created=$(docker image inspect denddron-swarm-agent:latest --format '{{.Created}}' 2>/dev/null || true)
  img_epoch=$(date -d "$created" +%s 2>/dev/null || echo 0)
  code_epoch=$(git log -1 --format=%ct -- src/ sim/ docker/ 2>/dev/null || echo 0)
  if (( img_epoch == 0 )); then
    echo "!! no denddron-swarm-agent image. Build first:  docker compose build" >&2; exit 1
  fi
  if (( code_epoch > img_epoch )); then
    echo "!! images are older than the last change to src/, sim/ or docker/." >&2
    echo "   Rebuild first:  docker compose build     (or set SKIP_IMAGE_CHECK=1)" >&2
    [[ "${SKIP_IMAGE_CHECK:-0}" == "1" ]] || exit 1
  fi
}

# --- provenance --------------------------------------------------------------
# Every experiment writes what produced it next to the results.
stamp() {
  local out=$1; shift
  mkdir -p "$out"
  {
    echo "experiment : ${EXPERIMENT:-unnamed}"
    echo "claim      : ${CLAIM:-unstated}"
    echo "run at     : $(date -Is)"
    echo "commit     : $(git rev-parse --short HEAD)$([[ -n $(git status --porcelain) ]] && echo ' (DIRTY TREE)')"
    echo "host       : $(nproc) cores, $(free -g | awk 'NR==2{print $2}') GB"
    echo "repeats    : $REPEATS   seed: $SEED   parallel: $PARALLEL"
    echo "command    : $*"
  } > "$out/experiment.txt"
  echo "== ${EXPERIMENT:-experiment} -> $out"
  cat "$out/experiment.txt"
}

run_sweep() {
  local out=$1; shift
  stamp "$out" "$@"
  "$@" 2>&1 | tee "$out/console.log"
}
