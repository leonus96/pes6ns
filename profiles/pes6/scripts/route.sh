#!/usr/bin/env bash
# Deterministic headless run with a scripted button route.
#   route.sh STOP_VBLANK "BUTTONS@START[+LENGTH]" ... (any number of presses;
#   each lasts LENGTH vblanks, 8 by default)
#   e.g. route.sh 3000 0x4000@200 0x8@1800 0x40@2000+30
# Buttons: 0x4000 cross, 0x2000 circle, 0x8000 square, 0x1000 triangle,
#          0x8 start, 0x1 select, 0x10 up, 0x40 down, 0x80 left, 0x20 right,
#          0x100 L, 0x200 R.
# Frames from the last press onwards (or DUMP_START, every DUMP_INTERVAL vblanks) go to profiles/pes6/analysis/frames.
set -uo pipefail
repo="$(cd "$(dirname "$0")/../../.." && pwd)"
stop="$1"; shift
last=0
for press in "$@"; do
    start="${press#*@}"; last="${start%+*}"
done
route="$*"
frames="$repo/profiles/pes6/analysis/frames"
mkdir -p "$frames"; rm -f "$frames"/*
# The intro movie is skipped (PES6_SKIP_MOVIES=0 plays it): routes count
# vblanks, and nothing waits for vblank while it plays.
PES6_SKIP_MOVIES="${PES6_SKIP_MOVIES:-1}" PSPRECOMP_CTRL_ROUTE="${route// /,}" PSPRECOMP_FRAME_DUMP_INTERVAL="${DUMP_INTERVAL:-1}" \
PSPRECOMP_FRAME_DUMP_DIR="$frames" PSPRECOMP_FRAME_DUMP_START="${DUMP_START:-$last}" \
PSPRECOMP_FRAME_DUMP_LIMIT="${DUMP_LIMIT:-400}" PSPRECOMP_STOP_VBLANK="$stop" \
PES6_RUN_SECONDS="${PES6_RUN_SECONDS:-600}" "$repo/profiles/pes6/scripts/run.sh"
