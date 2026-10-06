/*
 * Copyright (c) 2026 scholzri
 * SPDX-License-Identifier: Apache-2.0
 *
 * &bt_sel_ble N: select BLE profile N, like &bt BT_SEL N, and switch the
 * output from USB to BLE: pressing a profile key means the user wants to
 * type over BLE. Connection events and the open profile timeout's return
 * call zmk_ble_prof_select() directly and keep the output as it is.
 */

#define DT_DRV_COMPAT rainy_behavior_bt_sel_ble

#include <errno.h>
#include <zephyr/device.h>
#include <zephyr/logging/log.h>
#include <drivers/behavior.h>
#include <zmk/behavior.h>
#include <zmk/ble.h>
#include <zmk/endpoints.h>

LOG_MODULE_REGISTER(behavior_bt_sel_ble, CONFIG_ZMK_LOG_LEVEL);

#if DT_HAS_COMPAT_STATUS_OKAY(DT_DRV_COMPAT)

#if IS_ENABLED(CONFIG_ZMK_BEHAVIOR_METADATA)

static const struct behavior_parameter_value_metadata profile_values[] = {
    {
        .display_name = "Profile",
        .type = BEHAVIOR_PARAMETER_VALUE_TYPE_RANGE,
        .range = {.min = 0, .max = ZMK_BLE_PROFILE_COUNT - 1},
    },
};

static const struct behavior_parameter_metadata_set profile_set = {
    .param1_values = profile_values,
    .param1_values_len = ARRAY_SIZE(profile_values),
};

static const struct behavior_parameter_metadata metadata = {
    .sets_len = 1,
    .sets = &profile_set,
};

#endif // IS_ENABLED(CONFIG_ZMK_BEHAVIOR_METADATA)

static int on_bt_sel_ble_pressed(struct zmk_behavior_binding *binding,
                                 struct zmk_behavior_binding_event event) {
    int err = zmk_ble_prof_select(binding->param1);

    if (err) {
        LOG_ERR("Selecting BLE profile %d failed (%d)", binding->param1, err);
        return err;
    }
    // After the profile change, so the output never flips to the old profile. Also when the
    // profile was already active: the key then only switches the output.
    if (zmk_endpoint_get_preferred_transport() == ZMK_TRANSPORT_USB) {
        zmk_endpoint_set_preferred_transport(ZMK_TRANSPORT_BLE);
    }
    return ZMK_BEHAVIOR_OPAQUE;
}

static int on_bt_sel_ble_released(struct zmk_behavior_binding *binding,
                                  struct zmk_behavior_binding_event event) {
    return ZMK_BEHAVIOR_OPAQUE;
}

static const struct behavior_driver_api behavior_bt_sel_ble_api = {
    .binding_pressed = on_bt_sel_ble_pressed,
    .binding_released = on_bt_sel_ble_released,
#if IS_ENABLED(CONFIG_ZMK_BEHAVIOR_METADATA)
    .parameter_metadata = &metadata,
#endif
};

BEHAVIOR_DT_INST_DEFINE(0, NULL, NULL, NULL, NULL, POST_KERNEL,
                        CONFIG_KERNEL_INIT_PRIORITY_DEFAULT, &behavior_bt_sel_ble_api);

#endif /* DT_HAS_COMPAT_STATUS_OKAY(DT_DRV_COMPAT) */
