/*
 * Copyright (c) 2026 rainy75-zmk contributors
 * SPDX-License-Identifier: Apache-2.0
 *
 * Snap Tap decisions, see snap_tap.h.
 */
#include "snap_tap.h"

void snap_tap_init(struct snap_tap *s, const uint16_t pairs[SNAP_TAP_PAIRS][2], bool enabled) {
    for (int p = 0; p < SNAP_TAP_PAIRS; p++) {
        s->pair[p].key[0] = pairs[p][0];
        s->pair[p].key[1] = pairs[p][1];
        s->pair[p].held = 0;
        s->pair[p].reported = 0;
    }
    s->enabled = enabled;
}

static bool find(const struct snap_tap *s, uint16_t key, int *pair, int *idx) {
    for (int p = 0; p < SNAP_TAP_PAIRS; p++) {
        for (int k = 0; k < 2; k++) {
            if (s->pair[p].key[k] == key) {
                *pair = p;
                *idx = k;
                return true;
            }
        }
    }
    return false;
}

bool snap_tap_is_member(const struct snap_tap *s, uint16_t key) {
    int p, k;
    return find(s, key, &p, &k);
}

static void put(struct snap_tap_action *out, int *n, enum snap_tap_op op, uint16_t key) {
    out[*n].op = (uint8_t)op;
    out[*n].key = key;
    (*n)++;
}

int snap_tap_key(struct snap_tap *s, uint16_t key, bool pressed,
                 struct snap_tap_action out[SNAP_TAP_MAX_ACTIONS]) {
    int n = 0;
    int p, k;

    if (!find(s, key, &p, &k)) {
        put(out, &n, pressed ? SNAP_TAP_REPORT_PRESS : SNAP_TAP_REPORT_RELEASE, key);
        return n;
    }

    struct snap_tap_pair *pr = &s->pair[p];
    int o = k ^ 1;
    uint8_t kb = (uint8_t)(1u << k), ob = (uint8_t)(1u << o);

    if (pressed) {
        pr->held |= kb;
        /* Rule 1: the opposite key is held and reported, it yields. */
        if (s->enabled && (pr->held & ob) && (pr->reported & ob)) {
            put(out, &n, SNAP_TAP_SILENT_RELEASE, pr->key[o]);
            pr->reported &= (uint8_t)~ob;
        }
        put(out, &n, SNAP_TAP_REPORT_PRESS, key);
        pr->reported |= kb;
    } else {
        pr->held &= (uint8_t)~kb;
        /* Rule 2: the opposite key is still held but was suppressed, it resumes. */
        if (s->enabled && (pr->held & ob) && !(pr->reported & ob)) {
            put(out, &n, SNAP_TAP_SILENT_PRESS, pr->key[o]);
            pr->reported |= ob;
        }
        /* Rule 3 (and the release of a suppressed key: the report does not
         * change, the event keeps press / release symmetric). */
        put(out, &n, SNAP_TAP_REPORT_RELEASE, key);
        pr->reported &= (uint8_t)~kb;
    }
    return n;
}

int snap_tap_set_enabled(struct snap_tap *s, bool enabled,
                         struct snap_tap_action out[SNAP_TAP_MAX_ACTIONS]) {
    int n = 0;

    if (s->enabled == enabled) {
        return 0;
    }
    s->enabled = enabled;
    if (enabled) {
        return 0;   /* both keys down stay as they are until the next event */
    }
    /* Rule 4: a suppressed key is physically down, pass-through means it is
     * in the report. */
    for (int p = 0; p < SNAP_TAP_PAIRS; p++) {
        struct snap_tap_pair *pr = &s->pair[p];
        for (int k = 0; k < 2; k++) {
            uint8_t kb = (uint8_t)(1u << k);
            if ((pr->held & kb) && !(pr->reported & kb)) {
                put(out, &n, SNAP_TAP_SILENT_PRESS, pr->key[k]);
                pr->reported |= kb;
            }
        }
    }
    if (n > 0) {
        put(out, &n, SNAP_TAP_SEND, 0);
    }
    return n;
}
