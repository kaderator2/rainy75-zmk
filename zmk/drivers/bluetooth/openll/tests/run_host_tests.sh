#!/bin/bash
set -e
cd "$(dirname "$0")"
CC=${CC:-gcc}
build_run() {
    local t="$1"; shift
    echo "== $t =="
    $CC -std=c11 -Wall -Wextra -Werror -O1 -I.. -o "/tmp/openll_$t" "$t.c" "$@"
    "/tmp/openll_$t"
}
build_run test_hci ../ll_hci.c
build_run test_pdu ../ll_pdu.c
build_run test_adv ../ll_adv.c ../ll_pdu.c
exit 0
