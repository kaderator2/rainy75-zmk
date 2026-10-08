/*
 * Copyright (c) 2026 rainy75-zmk contributors
 * SPDX-License-Identifier: Apache-2.0
 *
 * Snap Tap: Razer-style last-input priority for two opposing-key pairs
 * (A/D and W/S by default). Pure logic, no Zephyr: the ZMK adapter
 * (src/behaviors/behavior_snap_tap.c) feeds physical press / release of a
 * key and executes the returned actions in order. Host tests in tests/.
 *
 * Rules per pair, with "held" = keys physically down and "reported" = keys
 * in the HID report:
 *  1. Press X while the opposite Y is held: Y leaves the report, X enters it,
 *     in ONE report (last input wins).
 *  2. Release X while Y is still held and not reported: Y re-enters the
 *     report together with X leaving it, in ONE report (resume).
 *  3. Release X while Y is not held: X leaves the report.
 *  4. Disabled: every key passes through untouched. Disabling while a key is
 *     suppressed puts it back into the report (one extra report), so no key
 *     stays silently held.
 *
 * Every physical press raises exactly one press event and every physical
 * release exactly one release event (REPORT_*); the suppress / resume of
 * the opposite key changes only the HID report (SILENT_*), so the next
 * REPORT_* action carries both changes in a single report.
 */
#ifndef RAINY75_SNAP_TAP_H
#define RAINY75_SNAP_TAP_H

#include <stdint.h>
#include <stdbool.h>

#define SNAP_TAP_PAIRS       2
#define SNAP_TAP_MAX_ACTIONS 3

enum snap_tap_op {
    SNAP_TAP_REPORT_PRESS,    /* raise the key's press through the normal path */
    SNAP_TAP_REPORT_RELEASE,  /* raise the key's release through the normal path */
    SNAP_TAP_SILENT_RELEASE,  /* drop the key from the HID report, no event, no send */
    SNAP_TAP_SILENT_PRESS,    /* put the key into the HID report, no event, no send */
    SNAP_TAP_SEND,            /* send the keyboard report now (after silent ops only) */
};

struct snap_tap_action {
    uint8_t op;      /* enum snap_tap_op */
    uint16_t key;    /* HID keyboard usage id */
};

struct snap_tap_pair {
    uint16_t key[2];   /* HID keyboard usage ids of the opposing keys */
    uint8_t held;      /* bit i: key[i] physically down */
    uint8_t reported;  /* bit i: key[i] in the HID report */
};

struct snap_tap {
    struct snap_tap_pair pair[SNAP_TAP_PAIRS];
    bool enabled;
};

void snap_tap_init(struct snap_tap *s, const uint16_t pairs[SNAP_TAP_PAIRS][2], bool enabled);

/* True if key is a member of one of the pairs. */
bool snap_tap_is_member(const struct snap_tap *s, uint16_t key);

/* Physical press (pressed = true) or release of key. Writes the actions to
 * execute, in order, and returns their count (1..2). A key that is not a
 * pair member passes through as one REPORT_* action. */
int snap_tap_key(struct snap_tap *s, uint16_t key, bool pressed,
                 struct snap_tap_action out[SNAP_TAP_MAX_ACTIONS]);

/* Enable or disable. Returns the number of actions written (0 when nothing
 * is held or the state does not change). */
int snap_tap_set_enabled(struct snap_tap *s, bool enabled,
                         struct snap_tap_action out[SNAP_TAP_MAX_ACTIONS]);

#endif
