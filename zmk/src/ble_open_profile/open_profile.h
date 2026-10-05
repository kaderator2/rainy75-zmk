/*
 * Copyright (c) 2026 scholzri
 * SPDX-License-Identifier: Apache-2.0
 *
 * Open profile timeout: the decisions, pure and ZMK-free (host tested in
 * tests/). open_profile_timeout.c feeds it from ZMK events and Bluetooth
 * callbacks and runs the timer it asks for.
 *
 * Selecting an open (unpaired) BLE profile puts the keyboard in pairing mode
 * for good: when the user picks the wrong slot or gives up, it keeps
 * advertising for a new host and types nowhere useful. With the timeout,
 * an explicitly selected open profile that no host pairs with in time is
 * left again:
 *
 *  - Armed when a profile change makes an open profile active and there is
 *    something to return to (the bonded profile active before, or a profile
 *    that was connected since boot). Selecting the already active open
 *    profile again is not a change and does not restart it.
 *  - On expiry: back to the previously active profile if it is connected,
 *    else to the most recently connected profile (RAM only), else stay.
 *  - A pairing in progress pauses it; a failed pairing restarts the full
 *    timeout. A plain connection does not pause it, so a host that stays
 *    connected after a failed pairing, or one with stale keys that keeps
 *    reconnecting, cannot hold the open profile forever.
 *  - A completed pairing, selecting a bonded profile and clearing bonds
 *    disarm it: clearing the active profile means the user wants to pair
 *    there, and after clearing all bonds there is nothing to return to.
 *  - Lives in RAM: not re-armed after a reboot or deep sleep wake.
 */

#ifndef RAINY75_OPEN_PROFILE_H
#define RAINY75_OPEN_PROFILE_H

#include <stdbool.h>
#include <stdint.h>

#define OPEN_PROFILE_NONE 0xFF
#define OPEN_PROFILE_MAX 32 /* profiles a view can describe (bit masks) */

struct open_profile {
    uint8_t active;    /* last active profile seen */
    uint8_t ret;       /* bonded profile active before the open one was selected */
    uint8_t last_conn; /* most recently connected bonded profile, RAM only */
    bool armed;        /* the timeout runs, or a pairing paused it */
    bool pairing;      /* a pairing runs (passkey requested until it ends) */
};

/* The profiles as they are now: bit i describes profile i. */
struct open_profile_view {
    uint32_t open;      /* no bond */
    uint32_t connected; /* bonded host connected */
};

/* What the caller does with its timer after a call. */
enum open_profile_timer {
    OPEN_PROFILE_TIMER_KEEP,  /* leave it as it is */
    OPEN_PROFILE_TIMER_START, /* (re)start the full timeout */
    OPEN_PROFILE_TIMER_STOP,  /* cancel it */
};

void open_profile_init(struct open_profile *s, uint8_t active);

/* The active profile is index now (ZMK's active profile changed event). An
 * event with the index already known is not a selection: ZMK raises it on
 * connection changes of the active profile too, and the timeout's own return
 * is a change to a bonded profile, which disarms like any other. */
enum open_profile_timer open_profile_active_changed(struct open_profile *s, uint8_t index,
                                                    const struct open_profile_view *v);

/* A bonded host connected (index < 0: not a bonded peer). */
void open_profile_connected(struct open_profile *s, int index);

/* A host asks for the passkey: a pairing runs, pause the timeout. */
enum open_profile_timer open_profile_pairing_started(struct open_profile *s);

/* A pairing failed, was cancelled or its host left: wait the full timeout
 * again. Only after a pairing that paused it. */
enum open_profile_timer open_profile_pairing_failed(struct open_profile *s,
                                                    const struct open_profile_view *v);

/* A pairing completed on profile. */
enum open_profile_timer open_profile_paired(struct open_profile *s, uint8_t profile);

/* The bond of profile was cleared (BT_CLR, or each profile of BT_CLR_ALL). */
enum open_profile_timer open_profile_cleared(struct open_profile *s, uint8_t profile);

/* The timer expired: returns the profile to select, or -1 to stay. Disarms
 * when it returns a profile or when there is none to return to. */
int open_profile_expired(struct open_profile *s, const struct open_profile_view *v);

#endif /* RAINY75_OPEN_PROFILE_H */
