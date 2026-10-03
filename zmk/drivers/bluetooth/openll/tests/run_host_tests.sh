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
build_run test_pdu ../ll_pdu.c
build_run test_adv ../ll_adv.c ../ll_pdu.c
build_run test_csa1 ../ll_csa1.c
build_run test_crypt ../ll_crypt.c aes_ref.c
# Suites with per-link state run for LL_MAX_CONN 1, 3 and 5 (slice 6a):
# binaries <test>_n<N>.
for n in 1 3 5; do
    build_run_as test_hci_n$n test_hci -DLL_MAX_CONN=$n ../ll_hci.c
    build_run_as test_txq_n$n test_txq -DLL_MAX_CONN=$n ../ll_txq.c
    build_run_as test_txq_safe_n$n test_txq -DLL_MAX_CONN=$n -DLL_TXQ_SAFE_MODE ../ll_txq.c
    build_run_as test_conn_n$n test_conn -DLL_MAX_CONN=$n \
        ../ll_conn.c ../ll_txq.c ../ll_rxq.c ../ll_csa1.c ../ll_crypt.c aes_ref.c
    build_run_as test_rxq_n$n test_rxq -DLL_MAX_CONN=$n ../ll_rxq.c ../ll_crypt.c aes_ref.c
    build_run_as test_llcp_n$n test_llcp -DLL_MAX_CONN=$n \
        ../ll_llcp.c ../ll_crypt.c aes_ref.c
done
exit 0
