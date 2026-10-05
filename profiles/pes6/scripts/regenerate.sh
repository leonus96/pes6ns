#!/usr/bin/env bash
# Regenerates profiles/pes6/generated/ from the user's decrypted EBOOT.
#   profiles/pes6/scripts/regenerate.sh [path/to/EBOOT_DECRYPTED.BIN]
set -euo pipefail

repo="$(cd "$(dirname "$0")/../../.." && pwd)"
profile="$repo/profiles/pes6"
eboot="${1:-$profile/game/EBOOT_DECRYPTED.BIN}"
expected="361b85a6c5576fa9ada7d57b0c9b5cccb62a4c999e33ccadaccfe30726385dd7"

actual="$(shasum -a 256 "$eboot" | cut -d' ' -f1)"
if [[ "$actual" != "$expected" ]]; then
    echo "warning: $eboot has SHA-256 $actual, expected $expected (ULES-00476)" >&2
fi

cmake -S "$repo" -B "$repo/out/framework" -G Ninja -DPSPRECOMP_PROFILE="" >/dev/null
cmake --build "$repo/out/framework" --target psp_recomp
# 16 KiB units, like the VCS profile: smaller translation units compile faster.
"$repo/out/framework/psp_recomp" "$eboot" --auto "$profile/generated" 0x08804000 16384
