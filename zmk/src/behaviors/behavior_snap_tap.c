/*
 * Copyright (c) 2026 rainy75-zmk contributors
 * SPDX-License-Identifier: Apache-2.0
 *
 * Snap Tap behaviors (Razer-style last-input priority, docs/usage.md):
 *   &snap KEY   a key press that takes part in Snap Tap when KEY is one of
 *               the pairs of the snap_tap node (A/D, W/S); any other KEY
 *               behaves exactly like &kp KEY
 *   &snap_tog   toggle Snap Tap on/off, persisted in settings, with a short
 *               LED confirmation on W/A/S/D (green = on, red = off)
 *
 * The decisions are pure (../snap_tap/snap_tap.c, host tests in
 * ../snap_tap/tests). This file executes them: REPORT_* actions raise the
 * usual keycode event (ZMK's hid_listener then updates and sends the HID
 * report), SILENT_* actions only edit the HID report, so the suppress /
 * resume of the opposite key leaves in the same report as the key that
 * caused it. A behavior is used instead of a keycode listener because
 * ZMK's event subscriptions run in link order and hid_listener (in the app
 * library) would already have sent the report with both keys down.
 */
#define DT_DRV_COMPAT rainy_behavior_snap_tap

#include <zephyr/device.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <drivers/behavior.h>
#include <zmk/behavior.h>
#include <zmk/event_manager.h>
#include <zmk/events/keycode_state_changed.h>
#include <zmk/hid.h>
#include <zmk/endpoints.h>
#include <dt-bindings/zmk/hid_usage_pages.h>
#include "../snap_tap/snap_tap.h"
#if IS_ENABLED(CONFIG_SETTINGS)
#include <zephyr/settings/settings.h>
#endif
#if IS_ENABLED(CONFIG_RAINY_RGB)
#include "../rainy_rgb/engine.h"
#endif

LOG_MODULE_REGISTER(snap_tap, CONFIG_LOG_DEFAULT_LEVEL);

#if DT_HAS_COMPAT_STATUS_OKAY(DT_DRV_COMPAT)

#define SNAP_TAP_SETTINGS_KEY     "snap_tap/on"
/* The toggle is a deliberate single keypress, so a short debounce is enough
 * (ZMK's 60 s global debounce would lose a toggle followed by a replug). */
#define SNAP_TAP_SAVE_DEBOUNCE_MS 1000

BUILD_ASSERT(DT_INST_PROP_LEN(0, pairs) == 2 * SNAP_TAP_PAIRS,
             "snap_tap: 'pairs' must list exactly two pairs of keycodes, e.g. <A D>, <W S>");

static const uint32_t pair_keycodes[] = DT_INST_PROP(0, pairs);

/* Single writer: behavior callbacks run on ZMK's work queue thread and the
 * settings load at boot finishes before the first keypress. */
static struct snap_tap st;

static void run(const struct snap_tap_action *a, int n, uint32_t encoded, int64_t ts) {
    for (int i = 0; i < n; i++) {
        switch (a[i].op) {
        case SNAP_TAP_REPORT_PRESS:
            raise_zmk_keycode_state_changed_from_encoded(encoded, true, ts);
            break;
        case SNAP_TAP_REPORT_RELEASE:
            raise_zmk_keycode_state_changed_from_encoded(encoded, false, ts);
            break;
        case SNAP_TAP_SILENT_RELEASE:
            zmk_hid_keyboard_release(a[i].key);
            break;
        case SNAP_TAP_SILENT_PRESS:
            zmk_hid_keyboard_press(a[i].key);
            break;
        case SNAP_TAP_SEND:
            zmk_endpoint_send_report(HID_USAGE_KEY);
            break;
        default:
            break;
        }
    }
}

static int snap_key(struct zmk_behavior_binding *binding, struct zmk_behavior_binding_event event,
                    bool pressed) {
    uint32_t enc = binding->param1;
    uint16_t page = ZMK_HID_USAGE_PAGE(enc);

    /* Only keyboard-page usages can be pair members; anything else is &kp. */
    if (page != 0 && page != HID_USAGE_KEY) {
        return raise_zmk_keycode_state_changed_from_encoded(enc, pressed, event.timestamp);
    }
    struct snap_tap_action act[SNAP_TAP_MAX_ACTIONS];
    int n = snap_tap_key(&st, ZMK_HID_USAGE_ID(enc), pressed, act);
    run(act, n, enc, event.timestamp);
    return ZMK_BEHAVIOR_OPAQUE;
}

static int on_snap_pressed(struct zmk_behavior_binding *binding,
                           struct zmk_behavior_binding_event event) {
    return snap_key(binding, event, true);
}

static int on_snap_released(struct zmk_behavior_binding *binding,
                            struct zmk_behavior_binding_event event) {
    return snap_key(binding, event, false);
}

/* --- on/off, persisted ---------------------------------------------------- */

#if IS_ENABLED(CONFIG_SETTINGS)
static void save_work_fn(struct k_work *w) {
    ARG_UNUSED(w);
    uint8_t on = st.enabled ? 1 : 0;
    int rc = settings_save_one(SNAP_TAP_SETTINGS_KEY, &on, sizeof(on));
    LOG_INF("persisted: on=%u (%d)", on, rc);
}
static K_WORK_DELAYABLE_DEFINE(save_work, save_work_fn);
#endif

static void set_enabled(bool on, bool persist, bool show) {
    struct snap_tap_action act[SNAP_TAP_MAX_ACTIONS];
    int n = snap_tap_set_enabled(&st, on, act);
    run(act, n, 0, k_uptime_get());
    LOG_INF("snap tap %s", on ? "on" : "off");
#if IS_ENABLED(CONFIG_RAINY_RGB)
    if (show) { rrgb_snap_tap_show(on); }
#else
    ARG_UNUSED(show);
#endif
#if IS_ENABLED(CONFIG_SETTINGS)
    if (persist) { k_work_reschedule(&save_work, K_MSEC(SNAP_TAP_SAVE_DEBOUNCE_MS)); }
#else
    ARG_UNUSED(persist);
#endif
}

bool snap_tap_enabled(void) { return st.enabled; }

#if IS_ENABLED(CONFIG_SETTINGS)
static int snap_tap_settings_set(const char *name, size_t len, settings_read_cb read_cb,
                                 void *cb_arg) {
    const char *next;
    if (settings_name_steq(name, "on", &next) && !next) {
        uint8_t on;
        if (len != sizeof(on)) { return -EINVAL; }
        int rc = read_cb(cb_arg, &on, sizeof(on));
        if (rc >= 0) {
            set_enabled(on != 0, false, false);
            return 0;
        }
        return rc;
    }
    return -ENOENT;
}
SETTINGS_STATIC_HANDLER_DEFINE(snap_tap, "snap_tap", NULL, snap_tap_settings_set, NULL, NULL);
#endif

static int behavior_snap_tap_init(const struct device *dev) {
    ARG_UNUSED(dev);
    uint16_t pairs[SNAP_TAP_PAIRS][2];
    for (int p = 0; p < SNAP_TAP_PAIRS; p++) {
        pairs[p][0] = ZMK_HID_USAGE_ID(pair_keycodes[2 * p]);
        pairs[p][1] = ZMK_HID_USAGE_ID(pair_keycodes[2 * p + 1]);
    }
    snap_tap_init(&st, pairs, IS_ENABLED(CONFIG_RAINY75_SNAP_TAP_DEFAULT_ON));
    return 0;
}

#if IS_ENABLED(CONFIG_ZMK_BEHAVIOR_METADATA)
/* Same parameter as &kp, so ZMK Studio offers the key picker. */
static const struct behavior_parameter_value_metadata snap_param_values[] = {
    { .display_name = "Key", .type = BEHAVIOR_PARAMETER_VALUE_TYPE_HID_USAGE },
};
static const struct behavior_parameter_metadata_set snap_param_metadata_set[] = {{
    .param1_values = snap_param_values,
    .param1_values_len = ARRAY_SIZE(snap_param_values),
}};
static const struct behavior_parameter_metadata snap_metadata = {
    .sets_len = ARRAY_SIZE(snap_param_metadata_set),
    .sets = snap_param_metadata_set,
};
#endif

static const struct behavior_driver_api behavior_snap_tap_api = {
    .binding_pressed = on_snap_pressed,
    .binding_released = on_snap_released,
#if IS_ENABLED(CONFIG_ZMK_BEHAVIOR_METADATA)
    .parameter_metadata = &snap_metadata,
#endif
};

BEHAVIOR_DT_INST_DEFINE(0, behavior_snap_tap_init, NULL, NULL, NULL, POST_KERNEL,
                        CONFIG_KERNEL_INIT_PRIORITY_DEFAULT, &behavior_snap_tap_api);

/* --- &snap_tog ------------------------------------------------------------ */

#if DT_HAS_COMPAT_STATUS_OKAY(rainy_behavior_snap_tap_toggle)

static int on_tog_pressed(struct zmk_behavior_binding *binding,
                          struct zmk_behavior_binding_event event) {
    ARG_UNUSED(binding); ARG_UNUSED(event);
    set_enabled(!st.enabled, true, true);
    return ZMK_BEHAVIOR_OPAQUE;
}

static int on_tog_released(struct zmk_behavior_binding *binding,
                           struct zmk_behavior_binding_event event) {
    ARG_UNUSED(binding); ARG_UNUSED(event);
    return ZMK_BEHAVIOR_OPAQUE;
}

static const struct behavior_driver_api behavior_snap_tap_toggle_api = {
    .binding_pressed = on_tog_pressed,
    .binding_released = on_tog_released,
#if IS_ENABLED(CONFIG_ZMK_BEHAVIOR_METADATA)
    .get_parameter_metadata = zmk_behavior_get_empty_param_metadata,
#endif
};

BEHAVIOR_DT_DEFINE(DT_INST(0, rainy_behavior_snap_tap_toggle), NULL, NULL, NULL, NULL,
                   POST_KERNEL, CONFIG_KERNEL_INIT_PRIORITY_DEFAULT,
                   &behavior_snap_tap_toggle_api);

#endif /* rainy_behavior_snap_tap_toggle */
#endif /* DT_HAS_COMPAT_STATUS_OKAY(DT_DRV_COMPAT) */
