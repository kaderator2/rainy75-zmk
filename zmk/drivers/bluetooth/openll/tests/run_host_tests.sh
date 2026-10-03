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
# build_run_as <binary name> <test source name> <cc args...>
build_run_as() {
    local n="$1" t="$2"; shift 2
    echo "== $n =="
    $CC -std=c11 -Wall -Wextra -Werror -O1 -I.. -o "/tmp/openll_$n" "$t.c" "$@"
    "/tmp/openll_$n"
}
build_run test_hci ../ll_hci.c
build_run test_pdu ../ll_pdu.c
build_run test_adv ../ll_adv.c ../ll_pdu.c
build_run test_csa1 ../ll_csa1.c
build_run test_crypt ../ll_crypt.c aes_ref.c
build_run test_txq ../ll_txq.c
build_run_as test_txq_safe test_txq -DLL_TXQ_SAFE_MODE ../ll_txq.c
exit 0
