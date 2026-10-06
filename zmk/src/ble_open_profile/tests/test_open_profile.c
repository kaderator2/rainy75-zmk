/*
 * Copyright (c) 2026 scholzri
 * SPDX-License-Identifier: Apache-2.0
 *
 * Host tests for the open profile timeout decisions (open_profile.c).
 * One test per rule of open_profile.h.
 */

#include "../open_profile.h"
#include <stdio.h>

static int failed;
#define CHECK(cond)                                                                                \
    do {                                                                                           \
        if (!(cond)) {                                                                             \
            printf("FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond);                                 \
            failed++;                                                                              \
        }                                                                                          \
    } while (0)

#define KEEP OPEN_PROFILE_TIMER_KEEP
#define START OPEN_PROFILE_TIMER_START
#define STOP OPEN_PROFILE_TIMER_STOP
#define B(i) (1U << (i))

/* 5 profiles: 0 bonded + connected, 1 bonded, 2..4 open. */
static const struct open_profile_view V = {.open = B(2) | B(3) | B(4), .connected = B(0)};

static struct open_profile s;

/* Booted on profile 0, its host connected. */
static void boot_connected(void) {
    open_profile_init(&s, 0);
    open_profile_connected(&s, 0);
}

static void test_arm_on_open_select(void) {
    boot_connected();
    CHECK(open_profile_active_changed(&s, 2, &V) == START);
    CHECK(s.armed && s.ret == 0);
    CHECK(open_profile_expired(&s, &V) == 0);
    CHECK(!s.armed);
    /* ZMK raises the change to 0 while selecting it: already disarmed, stays so */
    CHECK(open_profile_active_changed(&s, 0, &V) == STOP);
    CHECK(!s.armed && s.active == 0);
}

static void test_not_armed_without_return(void) {
    struct open_profile_view all_open = {.open = B(0) | B(1) | B(2) | B(3) | B(4)};

    open_profile_init(&s, 0); /* nothing bonded, nothing connected since boot */
    CHECK(open_profile_active_changed(&s, 1, &all_open) == STOP);
    CHECK(!s.armed);
}

static void test_reselect_and_connection_events_keep(void) {
    boot_connected();
    CHECK(open_profile_active_changed(&s, 2, &V) == START);
    /* re-select of the active open profile, or a connection change on it */
    CHECK(open_profile_active_changed(&s, 2, &V) == KEEP);
    CHECK(s.armed);
    /* a connection on a bonded active profile is no selection either */
    open_profile_init(&s, 0);
    CHECK(open_profile_active_changed(&s, 0, &V) == KEEP);
}

static void test_open_to_open_keeps_return(void) {
    boot_connected();
    open_profile_active_changed(&s, 2, &V);
    CHECK(open_profile_active_changed(&s, 3, &V) == START); /* full timeout again */
    CHECK(s.ret == 0);
    CHECK(open_profile_expired(&s, &V) == 0);
}

static void test_return_to_most_recently_connected(void) {
    /* previous profile 1 is bonded but not connected; 0 connected last */
    boot_connected();
    open_profile_active_changed(&s, 1, &V); /* bonded: disarmed, nothing armed */
    CHECK(open_profile_active_changed(&s, 2, &V) == START);
    CHECK(s.ret == 1);
    CHECK(open_profile_expired(&s, &V) == 0); /* 1 not connected -> last connected 0 */
}

static void test_return_to_previous_when_last_is_gone(void) {
    /* 1 connected, last connected 0 is not; the previous profile wins */
    struct open_profile_view v = {.open = B(2) | B(3) | B(4), .connected = B(1)};

    boot_connected();
    open_profile_active_changed(&s, 1, &v);
    open_profile_active_changed(&s, 2, &v);
    CHECK(open_profile_expired(&s, &v) == 1);
}

static void test_stay_without_target(void) {
    /* previous 1 not connected, last connected 1 is the open profile now:
     * its bond was cleared elsewhere (view says open) */
    struct open_profile_view v = {.open = B(1) | B(2) | B(3) | B(4)};

    open_profile_init(&s, 0);
    open_profile_connected(&s, 1);
    CHECK(open_profile_active_changed(&s, 2, &v) == START); /* 0 bonded: ret = 0 */
    CHECK(open_profile_expired(&s, &v) == -1);              /* 0 not connected, 1 open */
    CHECK(!s.armed);
}

static void test_pairing_pauses_and_failure_restarts(void) {
    boot_connected();
    open_profile_active_changed(&s, 2, &V);
    CHECK(open_profile_pairing_started(&s) == STOP);
    CHECK(s.armed && s.pairing);
    /* a stale run of the timer while paused: stay, keep the state */
    CHECK(open_profile_expired(&s, &V) == -1);
    CHECK(s.armed && s.pairing);
    CHECK(open_profile_pairing_failed(&s, &V) == START);
    CHECK(s.armed && !s.pairing);
    CHECK(open_profile_expired(&s, &V) == 0);
}

static void test_failure_without_pairing_does_not_restart(void) {
    /* a host with stale keys fails security over and over: no restart */
    boot_connected();
    open_profile_active_changed(&s, 2, &V);
    CHECK(open_profile_pairing_failed(&s, &V) == KEEP);
    CHECK(open_profile_pairing_failed(&s, &V) == KEEP);
}

static void test_pairing_while_not_armed(void) {
    open_profile_init(&s, 0);
    CHECK(open_profile_pairing_started(&s) == KEEP);
    CHECK(!s.pairing);
    CHECK(open_profile_pairing_failed(&s, &V) == KEEP);
}

static void test_paired_disarms(void) {
    boot_connected();
    open_profile_active_changed(&s, 2, &V);
    open_profile_pairing_started(&s);
    CHECK(open_profile_paired(&s, 2) == STOP);
    CHECK(!s.armed && !s.pairing && s.last_conn == 2);
    CHECK(open_profile_expired(&s, &V) == -1);
}

static void test_bonded_select_disarms(void) {
    boot_connected();
    open_profile_active_changed(&s, 2, &V);
    CHECK(open_profile_active_changed(&s, 1, &V) == STOP);
    CHECK(!s.armed && s.ret == OPEN_PROFILE_NONE);
    CHECK(open_profile_expired(&s, &V) == -1);
}

static void test_clear_active_disarms(void) {
    /* BT_CLR on the armed open profile: the user wants to pair there */
    boot_connected();
    open_profile_active_changed(&s, 2, &V);
    CHECK(open_profile_cleared(&s, 2) == STOP);
    CHECK(!s.armed);
    CHECK(s.last_conn == 0);
    /* BT_CLR on the connected bonded profile forgets it as a return */
    open_profile_init(&s, 0);
    open_profile_connected(&s, 0);
    CHECK(open_profile_cleared(&s, 0) == STOP);
    CHECK(s.last_conn == OPEN_PROFILE_NONE);
}

static void test_clear_all(void) {
    /* BT_CLR_ALL from profile 1 (armed open slot 2 before): ZMK clears all
     * bonds, selects profile 0 (that event comes first, synchronously) and
     * the CLEARED events follow from its queue. */
    struct open_profile_view all_open = {.open = B(0) | B(1) | B(2) | B(3) | B(4)};

    boot_connected();
    open_profile_active_changed(&s, 2, &V);
    CHECK(s.armed);
    open_profile_active_changed(&s, 0, &all_open); /* stale ret/last: may arm */
    for (uint8_t i = 0; i < 5; i++) {
        CHECK(open_profile_cleared(&s, i) == STOP);
    }
    CHECK(!s.armed && s.ret == OPEN_PROFILE_NONE && s.last_conn == OPEN_PROFILE_NONE);
    CHECK(open_profile_expired(&s, &all_open) == -1);
    /* afterwards selecting another empty slot does not arm */
    CHECK(open_profile_active_changed(&s, 1, &all_open) == STOP);
}

static void test_unbonded_connect_ignored(void) {
    open_profile_init(&s, 0);
    open_profile_connected(&s, -19); /* -ENODEV: not a bonded peer */
    CHECK(s.last_conn == OPEN_PROFILE_NONE);
}

int main(void) {
    test_arm_on_open_select();
    test_not_armed_without_return();
    test_reselect_and_connection_events_keep();
    test_open_to_open_keeps_return();
    test_return_to_most_recently_connected();
    test_return_to_previous_when_last_is_gone();
    test_stay_without_target();
    test_pairing_pauses_and_failure_restarts();
    test_failure_without_pairing_does_not_restart();
    test_pairing_while_not_armed();
    test_paired_disarms();
    test_bonded_select_disarms();
    test_clear_active_disarms();
    test_clear_all();
    test_unbonded_connect_ignored();
    if (failed) {
        printf("%d FAILED\n", failed);
        return 1;
    }
    printf("OK\n");
    return 0;
}
