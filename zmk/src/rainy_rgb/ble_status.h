#ifndef RAINY_RGB_BLE_STATUS_H
#define RAINY_RGB_BLE_STATUS_H
#include <stdint.h>
#include <stdbool.h>
#include "color.h"   /* struct rrgb */

/* BLE profile slot status on F1..F4, the number row and Enter.
 * Pure, ZMK-free. Each setter has a single writer thread (see ble_status.c),
 * render/active run on the render thread (single-core benign races,
 * as in overlay.c). Time is the engine's 50 FPS frame counter. */

enum rrgb_ble_slot { RRGB_BLE_EMPTY = 0, RRGB_BLE_PAIRED = 1, RRGB_BLE_CONNECTED = 2 };

enum rrgb_ble_ev {
	RRGB_BLE_EV_LOST,          /* slot was connected, now not */
	RRGB_BLE_EV_FAILED,        /* pairing failed / cancelled / passkey timeout */
	RRGB_BLE_EV_CLEARED,       /* bond cleared */
	RRGB_BLE_EV_PASSKEY_REQ,   /* host asks for the passkey on this slot */
	RRGB_BLE_EV_PASSKEY_DIGITS,/* arg = digits typed so far (0..6) */
	RRGB_BLE_EV_PAIRED_OK,
};

#define RRGB_BLE_SLOTS   3
#define RRGB_BLE_NONE    0xFF   /* key position not present */

/* Timing in render frames (50 FPS). Shared with the host tests. */
#define RRGB_BLE_BLINK_PERIOD    12    /* fast blink 4 Hz ...            */
#define RRGB_BLE_BLINK_ON         6    /* ... 50 % duty, on first        */
#define RRGB_BLE_BREATHE_PERIOD  50    /* breathe 1 Hz, triangle, bright at phase 0 */
#define RRGB_BLE_CONN_SOLID     100    /* connected: solid 2 s ...       */
#define RRGB_BLE_CONN_FADE       25    /* ... then fade out over 0.5 s   */
#define RRGB_BLE_FLASH_ON         8    /* red flash: COUNT x (ON + OFF)  */
#define RRGB_BLE_FLASH_OFF        8
#define RRGB_BLE_FLASH_COUNT      3
#define RRGB_BLE_FLASH_TOTAL     (RRGB_BLE_FLASH_COUNT * (RRGB_BLE_FLASH_ON + RRGB_BLE_FLASH_OFF))
#define RRGB_BLE_ENTER_PERIOD    50    /* Enter pulse 1 Hz               */
#define RRGB_BLE_PASSKEY_MAX   3000    /* safety: guidance ends 60 s after the last passkey event */
#define RRGB_BLE_PASSKEY_LEN      6    /* progress keys on the number row */
#define RRGB_BLE_STEADY_HOLD_FRAMES 1500 /* blink/breathe shown 30 s after the slot's last event */

/* Fixed brightness levels (0..255), independent of the RGB brightness. */
#define RRGB_BLE_BRIGHT 153   /* ~60 % */
#define RRGB_BLE_DIM     38   /* ~15 % */
#define RRGB_BLE_VDIM    13   /* ~5 %  */

/* LED indices (not keymap positions), provided by overlay.c which resolves
 * its keymap-coupled positions with rrgb_led_for_position(). An index of
 * RRGB_BLE_NONE or >= n is skipped. */
struct rrgb_ble_keys {
	uint8_t slot[RRGB_BLE_SLOTS];  /* F1..F3 */
	uint8_t output;                /* F4 */
	uint8_t numrow[10];            /* 1..0 */
	uint8_t enter;
};

/* Stores the key table and resets all state (all slots EMPTY, no active
 * slot, output USB, Fn released, no animation). */
void rrgb_ble_init(const struct rrgb_ble_keys *keys);

/* Polled slot states for slots 0..2 and the active slot index (>= 3: none
 * shown; state values > 2 are ignored). A slot that becomes CONNECTED starts
 * solid-then-fade; CONNECTED -> PAIRED raises the LOST red flash itself;
 * CONNECTED -> EMPTY (bond cleared) only cancels the solid, the CLEARED event
 * flashes. The first call after init (boot/wake), an active slot change and a
 * state change of the active slot restart the steady hold window. */
void rrgb_ble_set_slots(const uint8_t state[3], uint8_t active, uint32_t tick);
void rrgb_ble_set_output_ble(bool ble);
void rrgb_ble_set_fn(bool held);
void rrgb_ble_event(enum rrgb_ble_ev ev, uint8_t slot, uint8_t arg, uint32_t tick);

/* Paints only the keys it owns; returns true if it painted anything this frame. */
bool rrgb_ble_render(struct rrgb *px, uint16_t n, uint32_t tick);

/* Anything to draw (keeps the frame loop alive with RGB off). */
bool rrgb_ble_active(uint32_t tick);

#endif
