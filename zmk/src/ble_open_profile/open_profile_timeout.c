/*
 * Copyright (c) 2026 scholzri
 * SPDX-License-Identifier: Apache-2.0
 *
 * Open profile timeout (CONFIG_RAINY75_BLE_OPEN_PROFILE_TIMEOUT): leaves an
 * explicitly selected open BLE profile that no host pairs with in time. The
 * rules are in open_profile.h, the decisions in open_profile.c; this file
 * feeds them from public ZMK and Zephyr interfaces only and runs the timer.
 *
 * Where the signals come from:
 *  - Profile selection: ZMK's zmk_ble_active_profile_changed. Every
 *    zmk_ble_prof_select() that changes the profile raises it synchronously
 *    (behaviors, BT_NXT/BT_PRV, BT_CLR_ALL's select of profile 0). ZMK also
 *    raises it with the unchanged index when the active profile's host
 *    connects or disconnects or its address is stored; open_profile.c only
 *    acts on an index change, so those are not selections. The timeout's
 *    own return is a change to a bonded profile, which disarms like any
 *    other: it needs no marker to tell it from a user selection.
 *  - Pairing start: ZMK_BLE_AUTH_PASSKEY_REQ (zmk-src patch 0006). Zephyr
 *    has no "pairing started" callback besides pairing_accept, which belongs
 *    to the single bt_conn_auth_cb that ZMK registers. ZMK selects
 *    BT_SMP_SC_PAIR_ONLY, Zephyr's BT_SMP_ENFORCE_MITM (default y) sets MITM
 *    in our pairing response, and with CONFIG_ZMK_BLE_PASSKEY_ENTRY the
 *    keyboard is KeyboardOnly: every host with a display or keyboard
 *    (DisplayOnly, DisplayYesNo, KeyboardOnly, KeyboardDisplay) gets Passkey
 *    Entry, so its pairing starts with the passkey request. The difference
 *    to the old patch (which paused from the accepted pairing request): the
 *    feature exchange and key generation before the passkey request (well
 *    under a second) are not paused, and a NoInputNoOutput host, or a build
 *    without CONFIG_ZMK_BLE_PASSKEY_ENTRY, pairs Just Works without a pause.
 *    A Just Works pairing takes about a second; if the timeout hits it, ZMK
 *    rejects the result on the bonded profile (FAILED) and the host simply
 *    pairs again.
 *  - Pairing end: ZMK_BLE_AUTH_PAIRED_OK, or ZMK_BLE_AUTH_FAILED, which ZMK
 *    raises for pairing_failed, security_changed with an error, cancel, a
 *    pairing completed on a taken profile, and (patch 0007) the disconnect
 *    of a pairing nothing else ended. These come from the same ordered queue
 *    as PASSKEY_REQ; Zephyr's auth info callbacks would run in the BT RX
 *    thread, and an end seen before its queued start would leave the timer
 *    paused for good.
 *  - Bonds cleared: ZMK_BLE_AUTH_CLEARED (BT_CLR, every profile of
 *    BT_CLR_ALL).
 *  - Most recently connected profile: our own BT_CONN_CB_DEFINE connected
 *    callback (a bonded peer, peripheral role), and PAIRED_OK.
 *
 * Threads: the ZMK events arrive on the system workqueue (keymap behaviors
 * and the auth event work item), the connected callback in the BT RX
 * thread, the timer on the system workqueue. A spinlock serialises the
 * state; the profile view is read before taking it.
 */

#include <zephyr/kernel.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/logging/log.h>
#include <zephyr/settings/settings.h>

#include <zmk/ble.h>
#include <zmk/event_manager.h>
#include <zmk/events/ble_active_profile_changed.h>
#include <zmk/events/ble_auth_state_changed.h>
#include <rainy75/events/ble_open_profile_timeout.h>

#include "open_profile.h"

LOG_MODULE_REGISTER(ble_open_profile, CONFIG_ZMK_LOG_LEVEL);

BUILD_ASSERT(ZMK_BLE_PROFILE_COUNT <= OPEN_PROFILE_MAX, "profile bit masks are 32 bits wide");

ZMK_EVENT_IMPL(rainy75_ble_open_profile_timeout);

// ZMK starts on profile 0 until the settings load; the commit handler below updates it.
static struct open_profile state = {
    .active = 0,
    .ret = OPEN_PROFILE_NONE,
    .last_conn = OPEN_PROFILE_NONE,
};
static struct k_spinlock lock;

static void timeout_expired(struct k_work *work);
static K_WORK_DELAYABLE_DEFINE(timeout_work, timeout_expired);

static struct open_profile_view profile_view(void) {
    struct open_profile_view v = {0};

    for (uint8_t i = 0; i < ZMK_BLE_PROFILE_COUNT; i++) {
        if (zmk_ble_profile_is_open(i)) {
            v.open |= BIT(i);
        } else if (zmk_ble_profile_is_connected(i)) {
            v.connected |= BIT(i);
        }
    }
    return v;
}

// Called with the lock held, so a START and a STOP from two threads cannot swap.
static void timer_apply(enum open_profile_timer t) {
    switch (t) {
    case OPEN_PROFILE_TIMER_START:
        k_work_reschedule(&timeout_work, K_SECONDS(CONFIG_RAINY75_BLE_OPEN_PROFILE_TIMEOUT));
        break;
    case OPEN_PROFILE_TIMER_STOP:
        k_work_cancel_delayable(&timeout_work);
        break;
    default:
        break;
    }
}

static void timeout_expired(struct k_work *work) {
    struct open_profile_view v = profile_view();
    k_spinlock_key_t key = k_spin_lock(&lock);
    uint8_t from = state.active;
    bool waiting = state.armed && !state.pairing;
    int target = open_profile_expired(&state, &v);

    k_spin_unlock(&lock, key);

    if (target < 0) {
        if (waiting) {
            LOG_INF("Open profile %d timeout: no profile to return to, staying", from);
        }
        return;
    }

    LOG_INF("Open profile %d timeout: no host, back to profile %d", from, target);
    // Before the profile change, so indicators see it first.
    raise_rainy75_ble_open_profile_timeout(
        (struct rainy75_ble_open_profile_timeout){.profile = from, .target = target});
    // The behaviors switch the output to BLE, zmk_ble_prof_select() does not: the return keeps
    // the output as it is.
    int err = zmk_ble_prof_select(target);
    if (err) {
        LOG_WRN("Open profile %d timeout: selecting profile %d failed (%d)", from, target, err);
    }
}

static int open_profile_listener(const zmk_event_t *eh) {
    const struct zmk_ble_active_profile_changed *pc = as_zmk_ble_active_profile_changed(eh);
    const struct zmk_ble_auth_state_changed *auth = as_zmk_ble_auth_state_changed(eh);
    struct open_profile_view v = profile_view();
    k_spinlock_key_t key = k_spin_lock(&lock);

    if (pc) {
        timer_apply(open_profile_active_changed(&state, pc->index, &v));
    } else if (auth) {
        switch (auth->state) {
        case ZMK_BLE_AUTH_PASSKEY_REQ:
            timer_apply(open_profile_pairing_started(&state));
            break;
        case ZMK_BLE_AUTH_FAILED:
            timer_apply(open_profile_pairing_failed(&state, &v));
            break;
        case ZMK_BLE_AUTH_PAIRED_OK:
            timer_apply(open_profile_paired(&state, auth->profile));
            break;
        case ZMK_BLE_AUTH_CLEARED:
            timer_apply(open_profile_cleared(&state, auth->profile));
            break;
        default:
            break;
        }
    }

    k_spin_unlock(&lock, key);
    return ZMK_EV_EVENT_BUBBLE;
}

ZMK_LISTENER(rainy75_open_profile, open_profile_listener);
ZMK_SUBSCRIPTION(rainy75_open_profile, zmk_ble_active_profile_changed);
ZMK_SUBSCRIPTION(rainy75_open_profile, zmk_ble_auth_state_changed);

static void open_profile_connected_cb(struct bt_conn *conn, uint8_t err) {
    struct bt_conn_info info;

    if (err || bt_conn_get_info(conn, &info) || info.role != BT_CONN_ROLE_PERIPHERAL) {
        return;
    }

    int index = zmk_ble_profile_index(bt_conn_get_dst(conn));
    k_spinlock_key_t key = k_spin_lock(&lock);

    open_profile_connected(&state, index);
    k_spin_unlock(&lock, key);
}

BT_CONN_CB_DEFINE(rainy75_open_profile_conn_cb) = {
    .connected = open_profile_connected_cb,
};

#if IS_ENABLED(CONFIG_SETTINGS)
// The active profile is loaded from settings: start from it, so the first selection after boot
// knows the profile it came from.
static int open_profile_settings_commit(void) {
    k_spinlock_key_t key = k_spin_lock(&lock);

    state.active = zmk_ble_active_profile_index();
    k_spin_unlock(&lock, key);
    return 0;
}

SETTINGS_STATIC_HANDLER_DEFINE(rainy75_open_profile, "rainy75_open_profile", NULL, NULL,
                               open_profile_settings_commit, NULL);
#endif
