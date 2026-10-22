#!/bin/bash
# The whole round in one command (agent phys): join segments → physics of every frame → stats → checks.
# usage: ./cycle_run.sh [--prov] [--every N]     (heavy part runs under the shared lock; ~4–6 min of CPU on this machine)
#   --prov     allow provisional segments where segN.json is missing (cycle_build.mjs --prov)
#   --every N  solve every N-th frame only (quick look); default 1 = all frames
set -e
cd "$(dirname "$0")"
PROV=""; EVERY=1
while [ $# -gt 0 ]; do case "$1" in --prov) PROV="--prov";; --every) EVERY="$2"; shift;; esac; shift; done
node cycle_build.mjs $PROV
flock ./.heavy.lock node cycle_phys.mjs --every "$EVERY" --quiet
node cycle_stats.mjs
node cycle_physreport.mjs
