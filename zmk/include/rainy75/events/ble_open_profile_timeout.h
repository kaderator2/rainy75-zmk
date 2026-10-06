/*
 * Copyright (c) 2026 scholzri
 * SPDX-License-Identifier: Apache-2.0
 */

#pragma once

#include <zephyr/kernel.h>
#include <zmk/event_manager.h>

/* No host paired with the explicitly selected open profile in time
 * (CONFIG_RAINY75_BLE_OPEN_PROFILE_TIMEOUT): the keyboard leaves it for
 * target. Raised on the system workqueue right before the profile change,
 * so a listener sees it before the active profile changed event. */
struct rainy75_ble_open_profile_timeout {
    uint8_t profile; /* the open profile that is left */
    uint8_t target;  /* the profile selected instead */
};

ZMK_EVENT_DECLARE(rainy75_ble_open_profile_timeout);
