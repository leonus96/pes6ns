#!/usr/bin/env bash
# Deterministic headless run with a scripted button route.
#   route.sh STOP_VBLANK "BUTTONS@START" ... (up to 8 presses; each lasts 8 vblanks)
#   e.g. route.sh 3000 0x4000@200 0x8@1800
# Buttons: 0x4000 cross, 0x2000 circle, 0x8000 square, 0x1000 triangle,
#          0x8 start, 0x1 select, 0x10 up, 0x40 down, 0x80 left, 0x20 right.
# Frames from the last press onwards go to profiles/pes6/analysis/frames.
set -uo pipefail
repo="$(cd "$(dirname "$0")/../../.." && pwd)"
stop="$1"; shift
suffixes=("" 2 3 4 5 6 7 8)
i=0; last=0
for press in "$@"; do
    buttons="${press%@*}"; start="${press#*@}"
    s="${suffixes[$i]}"
    export "PSPRECOMP_CTRL_PULSE${s}_BUTTONS=$buttons"
    export "PSPRECOMP_CTRL_PULSE${s}_START_VBLANK=$start"
    export "PSPRECOMP_CTRL_PULSE${s}_END_VBLANK=$((start + 8))"
    last="$start"; i=$((i + 1))
done
frames="$repo/profiles/pes6/analysis/frames"
mkdir -p "$frames"; rm -f "$frames"/*
PSPRECOMP_FRAME_DUMP_DIR="$frames" PSPRECOMP_FRAME_DUMP_START="${DUMP_START:-$last}" \
PSPRECOMP_FRAME_DUMP_LIMIT="${DUMP_LIMIT:-400}" PSPRECOMP_STOP_VBLANK="$stop" \
PES6_RUN_SECONDS="${PES6_RUN_SECONDS:-600}" "$repo/profiles/pes6/scripts/run.sh"
