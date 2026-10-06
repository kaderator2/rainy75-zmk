#!/bin/bash
set -e
cd "$(dirname "$0")"
CC=${CC:-gcc}
echo "== test_open_profile =="
$CC -std=c11 -Wall -Wextra -O1 -o "${TMPDIR:-/tmp}/test_open_profile" test_open_profile.c ../open_profile.c
"${TMPDIR:-/tmp}/test_open_profile"
