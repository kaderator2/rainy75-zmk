#ifndef RAINY_RGB_OVERLAY_H
#define RAINY_RGB_OVERLAY_H
#include <stdint.h>
#include <stdbool.h>
#include "color.h"   /* struct rrgb */

/* Resolves the keymap-coupled BLE status key table and resets ble_status.
 * ble = false (build without BLE): ble_status owns no keys and is ignored.
 * Call once at boot, before the first BLE event. */
void rrgb_overlay_init(bool ble);

/* Neutral state, set by the adapter from the ZMK event thread. */
void rrgb_overlay_set_caps(bool on);
void rrgb_overlay_set_fn(bool active);
void rrgb_overlay_set_battery(uint8_t pct);
void rrgb_overlay_battery_show(uint32_t tick);   /* start the ~3s gauge window */

/* Applied AFTER the effect, BEFORE strip_show. Order: Fn-highlight,
 * CapsLock, battery gauge, BLE status (last, so it owns F1..F4 and wins
 * the number row while the passkey is typed). */
void rrgb_overlay_render(struct rrgb *px, uint16_t n, uint32_t tick);

/* True if any functional overlay needs to show this frame (caps on, Fn held,
 * battery gauge window open, or a BLE status indication), so the engine
 * renders indicators even when the decorative RGB is toggled off. */
bool rrgb_overlay_active(uint32_t tick);

/* True while ble_status shows an automatic BLE animation (connecting,
 * switching, pairing, see rrgb_ble_suppress_effect): the engine renders the
 * effect layer black (fading), the overlays still render on top. Always
 * false in a build without BLE. Implies rrgb_overlay_active(tick), so the
 * LED rail stays on with RGB off. */
bool rrgb_overlay_suppress_effect(uint32_t tick);

/* Effect layer gain 0..255 for the next frame: down to 0 over
 * RRGB_EFFECT_FADE_OUT_FRAMES while suppressed, back to 255 over
 * RRGB_EFFECT_FADE_IN_FRAMES after. Pure, the engine keeps the value. */
#define RRGB_EFFECT_FADE_OUT_FRAMES  5    /* 0.1 s at 50 FPS */
#define RRGB_EFFECT_FADE_IN_FRAMES  25    /* 0.5 s */
uint8_t rrgb_effect_gain_next(uint8_t gain, bool suppress);

/* False while the Fn layer is held: Fn combinations are commands (BT slots,
 * output, media, RGB controls), not typing, so they leave no reactive
 * afterglow or ripple; the indicators own those keys. Also false while the
 * effect is suppressed (tick = current render frame): the passkey digits
 * typed during pairing would otherwise leave invisible reactive state
 * (heat, ripples, the walker's step) that pops when the effect returns. */
bool rrgb_overlay_key_reactive(uint32_t position, uint32_t tick);

#endif
