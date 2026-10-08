#!/bin/bash
set -e
cd "$(dirname "$0")"
CC=${CC:-gcc}
echo "== test_snap_tap =="
$CC -std=c11 -Wall -Wextra -O1 -o "${TMPDIR:-/tmp}/test_snap_tap" test_snap_tap.c ../snap_tap.c
"${TMPDIR:-/tmp}/test_snap_tap"
