#!/bin/bash
set -e
cd "$(dirname "$0")"
CC=${CC:-gcc}
for t in test_color test_effects test_overlay test_ble_status; do
    [ -f "$t.c" ] || continue
    echo "== $t =="
    $CC -std=c11 -Wall -Wextra -O1 -o "${TMPDIR:-/tmp}/$t" "$t.c" \
        ../color.c ../effects.c ../led_map.c ../overlay.c ../ble_status.c -lm
    "${TMPDIR:-/tmp}/$t"
done
