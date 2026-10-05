/*
 * Copyright (c) 2026 scholzri
 * SPDX-License-Identifier: Apache-2.0
 *
 * Open profile timeout decisions, see open_profile.h. Pure: no ZMK, no
 * Zephyr; the caller serialises the calls.
 */

#include "open_profile.h"

static bool bit(uint32_t mask, uint8_t index) {
    return index < OPEN_PROFILE_MAX && (mask >> index) & 1U;
}

static bool is_open(const struct open_profile_view *v, uint8_t index) {
    return bit(v->open, index);
}

static enum open_profile_timer disarm(struct open_profile *s) {
    s->armed = false;
    s->pairing = false;
    s->ret = OPEN_PROFILE_NONE;
    return OPEN_PROFILE_TIMER_STOP;
}

void open_profile_init(struct open_profile *s, uint8_t active) {
    s->active = active;
    s->ret = OPEN_PROFILE_NONE;
    s->last_conn = OPEN_PROFILE_NONE;
    s->armed = false;
    s->pairing = false;
}

enum open_profile_timer open_profile_active_changed(struct open_profile *s, uint8_t index,
                                                    const struct open_profile_view *v) {
    uint8_t prev = s->active;

    if (index == prev) {
        return OPEN_PROFILE_TIMER_KEEP; /* a connection change, or nothing new */
    }
    s->active = index;

    if (!is_open(v, index)) {
        return disarm(s); /* a bonded profile, also the timeout's own return */
    }
    if (prev != OPEN_PROFILE_NONE && !is_open(v, prev)) {
        s->ret = prev;
    } /* else open -> open: keep the bonded profile from before */
    if (s->ret == OPEN_PROFILE_NONE && s->last_conn == OPEN_PROFILE_NONE) {
        return disarm(s); /* e.g. all bonds were just cleared: nothing to return to */
    }
    s->armed = true;
    return OPEN_PROFILE_TIMER_START;
}

void open_profile_connected(struct open_profile *s, int index) {
    if (index >= 0 && index < OPEN_PROFILE_MAX) {
        s->last_conn = (uint8_t)index;
    }
}

enum open_profile_timer open_profile_pairing_started(struct open_profile *s) {
    if (!s->armed) {
        return OPEN_PROFILE_TIMER_KEEP;
    }
    s->pairing = true; /* its end restarts the timer, its success disarms it */
    return OPEN_PROFILE_TIMER_STOP;
}

enum open_profile_timer open_profile_pairing_failed(struct open_profile *s,
                                                    const struct open_profile_view *v) {
    if (!s->pairing) {
        return OPEN_PROFILE_TIMER_KEEP;
    }
    s->pairing = false;
    return s->armed && is_open(v, s->active) ? OPEN_PROFILE_TIMER_START : OPEN_PROFILE_TIMER_KEEP;
}

enum open_profile_timer open_profile_paired(struct open_profile *s, uint8_t profile) {
    s->last_conn = profile;
    return disarm(s);
}

enum open_profile_timer open_profile_cleared(struct open_profile *s, uint8_t profile) {
    if (s->last_conn == profile) {
        s->last_conn = OPEN_PROFILE_NONE;
    }
    return disarm(s); /* also forgets ret */
}

int open_profile_expired(struct open_profile *s, const struct open_profile_view *v) {
    int target = -1;

    if (!s->armed || !is_open(v, s->active) || s->pairing) {
        return -1; /* stale run, or a pairing paused it meanwhile */
    }
    if (s->ret != OPEN_PROFILE_NONE && s->ret != s->active && bit(v->connected, s->ret)) {
        target = s->ret;
    } else if (s->last_conn != OPEN_PROFILE_NONE && s->last_conn != s->active &&
               !is_open(v, s->last_conn)) {
        target = s->last_conn;
    }
    disarm(s);
    return target;
}
