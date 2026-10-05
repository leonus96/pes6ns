#!/usr/bin/env bash
# Plays a whole exhibition match headless and deterministically: boot from an
# empty SAVEDATA, Austria vs Belgium, 5-minute match without extra time or
# penalties, then the match itself up to the "Resultado" screen (~v47200).
#   match_route.sh [STOP_VBLANK]   (default 48000; DUMP_START/DUMP_INTERVAL as in route.sh)
# During the match it presses ✕, START and ✕ again every 400 vblanks: ✕ takes
# kick-offs and goal kicks, START leaves the replay viewer the game opens after
# every goal (it waits for "START Final"), and in open play START only pauses
# with "Volver al partido" selected, which the second ✕ confirms.
# Note: it deletes profiles/pes6/game/PSP/SAVEDATA (first-boot dialogs expected).
set -euo pipefail
repo="$(cd "$(dirname "$0")/../../.." && pwd)"
stop="${1:-48000}"
rm -rf "$repo/profiles/pes6/game/PSP/SAVEDATA"

# Language, option file and level dialogs, menus, team and kit selection.
route="0x4000@300 0x4000@1000 0x4000@1700 0x4000@2400 0x4000@3100 0x4000@3800 0x4000@4500 0x4000@5200"
route+=" 0x4000@5700 0x4000@5800 0x20@5950 0x4000@6050 0x4000@6400"
# Ajustes generales: TE -> No, PP -> No, duration 10 -> 5 min, back up to
# "A ajustes de formación"; then "Comienzo del partido".
route+=" 0x40@6500 0x40@6540 0x80@6580 0x40@6640 0x80@6700 0x40@6760 0x40@6800 0x40@6840 0x80@6880"
route+=" 0x10@6920 0x10@6960 0x10@7000 0x10@7040 0x10@7080 0x10@7120 0x4000@7250 0x4000@7500"
for v in $(seq 9200 400 "$stop"); do
    route+=" 0x4000@$v 0x8@$((v + 200)) 0x4000@$((v + 240))"
done
# shellcheck disable=SC2086
exec "$repo/profiles/pes6/scripts/route.sh" "$stop" $route
