#!/usr/bin/env bash
# Master League save/load check, headless and deterministic, in three boots
# that share a scratch SAVEDATA folder (never the player's):
#   1. first boot: language, option file and level dialogs (creates the
#      option file, ULES004760013000);
#   2. Liga Master > Nuevo juego, defaults everywhere (Arsenal), close the
#      "Tópicos" help with △, then Guardar in the hub (LISTSAVE -> 5008000);
#   3. Liga Master > Cargar datos (LISTLOAD) back to the hub.
# Prints PASS when both savedata calls succeed; the last frame of boot 3 (the
# hub, "2006 Semana 6 / Partido 1") is left in profiles/pes6/analysis/frames.
set -uo pipefail
repo="$(cd "$(dirname "$0")/../../.." && pwd)"
route="$repo/profiles/pes6/scripts/route.sh"
export PES6_SAVEDATA_DIR="$repo/out/pes6/ml-savedata"
rm -rf "$PES6_SAVEDATA_DIR"
log="$(mktemp)"
trap 'rm -f "$log"' EXIT

echo "[ml_route] boot 1: first boot"
"$route" 5600 0x4000@300 0x4000@1000 0x4000@1700 0x4000@2400 0x4000@3100 0x4000@3800 0x4000@4500 0x4000@5200 \
    2>&1 | grep -a "\[savedata\] mode="

# Main menu (v~2300 once the option file exists): ↓ to LIGA MASTER, ✕.
menu="0x4000@300 0x4000@1700 0x40@2350 0x4000@2450"

echo "[ml_route] boot 2: new Master League, save"
presses="$menu 0x10@2550 0x4000@2600 0x4000@2700"
for v in $(seq 2800 150 5200); do presses+=" 0x4000@$v"; done
presses+=" 0x1000@5400 0x1000@5500 0x4000@5600"
for v in $(seq 6450 100 6950); do presses+=" 0x20@$v"; done
presses+=" 0x4000@7050"
# shellcheck disable=SC2086
DUMP_START=99999 "$route" 7300 $presses 2>&1 | grep -a "\[savedata\] mode=" | tee -a "$log"

echo "[ml_route] boot 3: load the Master League"
# shellcheck disable=SC2086
DUMP_START=3100 "$route" 3100 $menu 0x4000@2550 2>&1 | grep -a "\[savedata\] mode=" | tee -a "$log"

if grep -q "mode=5 .*save=5008000 .*-> 0x00000000" "$log" && grep -q "mode=4 .*save=5008000 .*-> 0x00000000" "$log"; then
    echo "[ml_route] PASS"
else
    echo "[ml_route] FAIL"
    exit 1
fi
