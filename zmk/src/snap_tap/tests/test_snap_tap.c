/*
 * Copyright (c) 2026 rainy75-zmk contributors
 * SPDX-License-Identifier: Apache-2.0
 *
 * Host tests for the Snap Tap decisions (snap_tap.c), one per rule of
 * snap_tap.h plus the toggle edge cases.
 */
#include "../snap_tap.h"
#include <stdio.h>
#include <string.h>

static int failed;
#define CHECK(cond)                                                                                \
    do {                                                                                           \
        if (!(cond)) {                                                                             \
            printf("FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond);                                 \
            failed++;                                                                              \
        }                                                                                          \
    } while (0)

#define A 0x04
#define D 0x07
#define S 0x16
#define W 0x1A
#define Q 0x14

static const uint16_t PAIRS[SNAP_TAP_PAIRS][2] = {{A, D}, {W, S}};

static struct snap_tap st;
static struct snap_tap_action act[SNAP_TAP_MAX_ACTIONS];

static void boot(bool enabled) { snap_tap_init(&st, PAIRS, enabled); }

static int key(uint16_t k, bool pressed) {
    memset(act, 0xFF, sizeof(act));
    return snap_tap_key(&st, k, pressed, act);
}

static bool is(int i, enum snap_tap_op op, uint16_t k) { return act[i].op == op && act[i].key == k; }

static void test_passthrough_non_member(void) {
    boot(true);
    CHECK(key(Q, true) == 1 && is(0, SNAP_TAP_REPORT_PRESS, Q));
    CHECK(key(Q, false) == 1 && is(0, SNAP_TAP_REPORT_RELEASE, Q));
    CHECK(!snap_tap_is_member(&st, Q) && snap_tap_is_member(&st, W));
}

static void test_single_key_plain(void) {
    boot(true);
    CHECK(key(A, true) == 1 && is(0, SNAP_TAP_REPORT_PRESS, A));
    CHECK(key(A, false) == 1 && is(0, SNAP_TAP_REPORT_RELEASE, A));
    CHECK(st.pair[0].held == 0 && st.pair[0].reported == 0);
}

/* Rule 1: D while A held -> A silently out, D pressed (one report). */
static void test_rule1_last_input_wins(void) {
    boot(true);
    key(A, true);
    CHECK(key(D, true) == 2);
    CHECK(is(0, SNAP_TAP_SILENT_RELEASE, A));
    CHECK(is(1, SNAP_TAP_REPORT_PRESS, D));
    CHECK(st.pair[0].held == 3 && st.pair[0].reported == 2);
}

/* Rule 2: release D while A still held -> A silently back, D released. */
static void test_rule2_resume(void) {
    boot(true);
    key(A, true);
    key(D, true);
    CHECK(key(D, false) == 2);
    CHECK(is(0, SNAP_TAP_SILENT_PRESS, A));
    CHECK(is(1, SNAP_TAP_REPORT_RELEASE, D));
    CHECK(st.pair[0].held == 1 && st.pair[0].reported == 1);
    /* Then release A normally. */
    CHECK(key(A, false) == 1 && is(0, SNAP_TAP_REPORT_RELEASE, A));
    CHECK(st.pair[0].held == 0 && st.pair[0].reported == 0);
}

/* Rule 3: release the suppressed key first -> only its (symmetric) release,
 * the report does not change, D stays. */
static void test_rule3_release_suppressed_first(void) {
    boot(true);
    key(A, true);
    key(D, true);
    CHECK(key(A, false) == 1 && is(0, SNAP_TAP_REPORT_RELEASE, A));
    CHECK(st.pair[0].held == 2 && st.pair[0].reported == 2);
    CHECK(key(D, false) == 1 && is(0, SNAP_TAP_REPORT_RELEASE, D));
}

/* Ping-pong: A, D, A, D while all stay held: each press wins. */
static void test_alternate_while_held(void) {
    boot(true);
    key(A, true);
    key(D, true);
    CHECK(key(A, false) == 1);            /* A was suppressed: plain release */
    CHECK(key(A, true) == 2 && is(0, SNAP_TAP_SILENT_RELEASE, D) && is(1, SNAP_TAP_REPORT_PRESS, A));
    CHECK(key(D, false) == 1 && is(0, SNAP_TAP_REPORT_RELEASE, D));
    CHECK(st.pair[0].reported == 1);
    CHECK(key(D, true) == 2 && is(0, SNAP_TAP_SILENT_RELEASE, A) && is(1, SNAP_TAP_REPORT_PRESS, D));
}

/* Pairs are independent: W/S never touches A/D. */
static void test_pairs_independent(void) {
    boot(true);
    key(A, true);
    CHECK(key(W, true) == 1 && is(0, SNAP_TAP_REPORT_PRESS, W));
    CHECK(key(S, true) == 2 && is(0, SNAP_TAP_SILENT_RELEASE, W) && is(1, SNAP_TAP_REPORT_PRESS, S));
    CHECK(st.pair[0].reported == 1 && st.pair[1].reported == 2);
    CHECK(key(D, true) == 2 && is(0, SNAP_TAP_SILENT_RELEASE, A));
}

/* Rule 4: disabled, everything passes through, both keys stay reported. */
static void test_disabled_passthrough(void) {
    boot(false);
    key(A, true);
    CHECK(key(D, true) == 1 && is(0, SNAP_TAP_REPORT_PRESS, D));
    CHECK(st.pair[0].reported == 3);
    CHECK(key(D, false) == 1 && is(0, SNAP_TAP_REPORT_RELEASE, D));
    CHECK(st.pair[0].reported == 1);
    CHECK(key(A, false) == 1 && is(0, SNAP_TAP_REPORT_RELEASE, A));
}

/* Disabling while A is suppressed puts A back and sends. */
static void test_disable_resumes_suppressed(void) {
    boot(true);
    key(A, true);
    key(D, true);
    CHECK(snap_tap_set_enabled(&st, false, act) == 2);
    CHECK(is(0, SNAP_TAP_SILENT_PRESS, A) && act[1].op == SNAP_TAP_SEND);
    CHECK(st.pair[0].reported == 3 && !st.enabled);
    CHECK(key(A, false) == 1 && is(0, SNAP_TAP_REPORT_RELEASE, A));
    CHECK(key(D, false) == 1 && is(0, SNAP_TAP_REPORT_RELEASE, D));
}

/* Disabling with nothing suppressed, or a no-op toggle, emits nothing. */
static void test_toggle_noop(void) {
    boot(true);
    CHECK(snap_tap_set_enabled(&st, true, act) == 0);
    key(A, true);
    CHECK(snap_tap_set_enabled(&st, false, act) == 0);
    CHECK(snap_tap_set_enabled(&st, false, act) == 0);
    key(A, false);
}

/* Enabling with both keys held and reported changes nothing until the next
 * event; the next press then applies rule 1. */
static void test_enable_mid_hold(void) {
    boot(false);
    key(A, true);
    key(D, true);
    CHECK(snap_tap_set_enabled(&st, true, act) == 0);
    CHECK(st.pair[0].reported == 3);
    CHECK(key(A, false) == 1 && is(0, SNAP_TAP_REPORT_RELEASE, A));
    CHECK(key(A, true) == 2 && is(0, SNAP_TAP_SILENT_RELEASE, D) && is(1, SNAP_TAP_REPORT_PRESS, A));
}

int main(void) {
    test_passthrough_non_member();
    test_single_key_plain();
    test_rule1_last_input_wins();
    test_rule2_resume();
    test_rule3_release_suppressed_first();
    test_alternate_while_held();
    test_pairs_independent();
    test_disabled_passthrough();
    test_disable_resumes_suppressed();
    test_toggle_noop();
    test_enable_mid_hold();
    if (failed) {
        printf("%d FAILED\n", failed);
        return 1;
    }
    printf("OK\n");
    return 0;
}
