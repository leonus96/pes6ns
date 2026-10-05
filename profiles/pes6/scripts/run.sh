#!/usr/bin/env bash
# Runs PES6Native headless (PES6_HEADLESS=0 for a window) against profiles/pes6/game with a wall-clock limit.
# The frame limiter is off by default: it advances guest time by the host's
# wall-clock deficit, which makes runs depend on host load (non-deterministic).
# The guest's wall clock (RTC, libc time) is pinned too: with the host clock the
# same route diverged once a match started. PES6_FIXED_CLOCK="" restores it.
#   PES6_RUN_SECONDS=60 profiles/pes6/scripts/run.sh [extra args...]
set -uo pipefail

repo="$(cd "$(dirname "$0")/../../.." && pwd)"
game="$repo/profiles/pes6/game"
seconds="${PES6_RUN_SECONDS:-60}"

PSPRECOMP_FRAME_LIMIT="${PSPRECOMP_FRAME_LIMIT:-0}" PES6_FIXED_CLOCK="${PES6_FIXED_CLOCK-1790000000}" PES6_HEADLESS="${PES6_HEADLESS:-1}" PES6_LOG="${PES6_LOG:-stderr}" "$repo/out/pes6/bin/PES6Native" \
    "$game/EBOOT_DECRYPTED.BIN" "$game" "$@" &
pid=$!
( sleep "$seconds"; kill "$pid" 2>/dev/null && echo "[run.sh] killed after ${seconds}s" >&2 ) &
watchdog=$!
wait "$pid"
status=$?
kill "$watchdog" 2>/dev/null
exit "$status"
