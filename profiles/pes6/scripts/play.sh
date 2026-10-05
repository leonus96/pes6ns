#!/usr/bin/env bash
# Plays PES6 in a window: profiles/pes6/scripts/play.sh
# Keyboard: arrows = D-pad, X = cross, C = circle, Z = square, V = triangle,
# Q/E = L/R, Enter = START, Space = SELECT, WASD = analog stick, Esc = quit.
set -euo pipefail
repo="$(cd "$(dirname "$0")/../../.." && pwd)"
game="$repo/profiles/pes6/game"
exec "$repo/out/pes6/bin/PES6Native" "$game/EBOOT_DECRYPTED.BIN" "$game" "$@"
