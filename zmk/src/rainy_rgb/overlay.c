#include <stdint.h>
#include <stdbool.h>
#include "overlay.h"
#include "color.h"
#include "led_map.h"     /* rrgb_led_for_position */
#include "ble_status.h"

#define CAPS_POS         44   /* keymap position of CapsLock (&kp CLCK) */
#define BAT_SHOW_FRAMES  90   /* ~3s at 30fps */
#define BAT_SEG_FIRST    16   /* number row keys 1..0 = positions 16..25 */
#define BAT_SEG_COUNT    10
#define LGUI_POS         74   /* keymap position of Left GUI (&kp LGUI) */
#define SNAP_SHOW_FRAMES 60   /* ~1.2 s at 50 fps */
/* Snap Tap pair keys (W, A, S, D), keymap positions. KEYMAP-COUPLED with
 * the pairs of dts/rainy_snap_tap.dtsi and the default layer. */
static const uint8_t snap_keys[] = { 32, 45, 46, 47 };
#define SNAP_KEYS_COUNT (sizeof(snap_keys) / sizeof(snap_keys[0]))

/* Fn-active keymap positions (non-&trans on the Fn layer). KEYMAP-COUPLED:
 * update this if the Fn layer changes. */
static const uint8_t fn_keys[] = {
    0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,   /* top row: studio/BT/output/media/BT_CLR */
    28,   /* BKSP  -> RGB_TOG */
    35,   /* T     -> Snap Tap toggle */
    43,   /* Enter -> RGB_EFF (ANSI: backslash -> RGB_HUI) */
    56,   /* NUHS  -> RGB_HUI (ANSI: Enter -> RGB_EFF) */
    65,   /* B     -> RGB_BAT */
    72,   /* UP    -> RGB_BRI */
    74,   /* LGUI  -> Win lock layer toggle */
    80, 81, 82,                            /* arrows -> speed/bright */
};
#define FN_KEYS_COUNT (sizeof(fn_keys) / sizeof(fn_keys[0]))

/* BLE status keys (ble_status.h), keymap positions. KEYMAP-COUPLED:
 * F1..F3 = &bt_sel_ble 0..2, F4 = &out OUT_TOG on the Fn layer; the number
 * row 1..0 takes the passkey digits; Enter submits it. */
#define BLE_POS_SLOT0    1    /* F1..F3 = positions 1..3 */
#define BLE_POS_OUTPUT   4    /* F4 */
#define BLE_POS_NUMROW  16    /* 1..0 = positions 16..25 */
#ifdef CONFIG_RAINY_RGB_ANSI_LEDMAP
#define BLE_POS_ENTER   56    /* ANSI wide Enter (the ISO #~ slot) */
#else
#define BLE_POS_ENTER   43    /* ISO Enter */
#endif

static volatile bool     s_caps;
static volatile bool     s_fn;
static volatile uint8_t  s_battery;
static volatile uint32_t s_bat_until;
static volatile bool     s_winlock;
static volatile bool     s_snap_on;
static volatile uint32_t s_snap_until;
static bool              s_ble;   /* ble_status owns its keys (BLE build) */

static uint8_t ble_led(uint8_t pos) {
    int led = rrgb_led_for_position(pos);
    return (led < 0) ? RRGB_BLE_NONE : (uint8_t)led;
}

void rrgb_overlay_init(bool ble) {
    struct rrgb_ble_keys k;
    for (uint8_t s = 0; s < RRGB_BLE_SLOTS; s++) {
        k.slot[s] = ble ? ble_led((uint8_t)(BLE_POS_SLOT0 + s)) : RRGB_BLE_NONE;
    }
    k.output = ble ? ble_led(BLE_POS_OUTPUT) : RRGB_BLE_NONE;
    for (uint8_t d = 0; d < 10; d++) {
        k.numrow[d] = ble ? ble_led((uint8_t)(BLE_POS_NUMROW + d)) : RRGB_BLE_NONE;
    }
    k.enter = ble ? ble_led(BLE_POS_ENTER) : RRGB_BLE_NONE;
    rrgb_ble_init(&k);
    rrgb_ble_set_fn(s_fn);
    s_ble = ble;
}

void rrgb_overlay_set_caps(bool on)        { s_caps = on; }
void rrgb_overlay_set_fn(bool active)      { s_fn = active; rrgb_ble_set_fn(active); }
void rrgb_overlay_set_battery(uint8_t pct) { s_battery = pct; }
void rrgb_overlay_battery_show(uint32_t tick) { s_bat_until = tick + BAT_SHOW_FRAMES; }
void rrgb_overlay_set_winlock(bool on)     { s_winlock = on; }
void rrgb_overlay_snap_tap_show(bool on, uint32_t tick) {
    s_snap_on = on;
    s_snap_until = tick + SNAP_SHOW_FRAMES;
}

bool rrgb_overlay_suppress_effect(uint32_t tick) {
    return s_ble && rrgb_ble_suppress_effect(tick);
}

#define GAIN_OUT_STEP ((255 + RRGB_EFFECT_FADE_OUT_FRAMES - 1) / RRGB_EFFECT_FADE_OUT_FRAMES)
#define GAIN_IN_STEP  ((255 + RRGB_EFFECT_FADE_IN_FRAMES - 1) / RRGB_EFFECT_FADE_IN_FRAMES)

uint8_t rrgb_effect_gain_next(uint8_t gain, bool suppress) {
    if (suppress) {
        return gain > GAIN_OUT_STEP ? (uint8_t)(gain - GAIN_OUT_STEP) : 0;
    }
    return gain < 255 - GAIN_IN_STEP ? (uint8_t)(gain + GAIN_IN_STEP) : 255;
}

uint8_t rrgb_effect_gain_frame(uint8_t gain, bool suppress, bool drawn) {
    if (!drawn) {
        return suppress ? 0 : 255;
    }
    return rrgb_effect_gain_next(gain, suppress);
}

/* Called from the key event thread with the render thread's tick: a
 * deliberately unlocked cross-thread read (a 32-bit load, at worst one frame
 * stale, which moves the suppression edge by one frame). */
bool rrgb_overlay_key_reactive(uint32_t position, uint32_t tick) {
    (void)position;   /* every Fn-layer press is a command, not only F1..F4 */
    return !s_fn && !rrgb_overlay_suppress_effect(tick);
}

bool rrgb_overlay_active(uint32_t tick) {
    return s_caps || s_fn || s_winlock || (tick < s_bat_until) || (tick < s_snap_until) ||
           (s_ble && rrgb_ble_active(tick));
}

static void set_pos(struct rrgb *px, uint16_t n, uint8_t pos, struct rrgb c) {
    int led = rrgb_led_for_position(pos);
    if (led >= 0 && led < (int)n) { px[led] = c; }
}

void rrgb_overlay_render(struct rrgb *px, uint16_t n, uint32_t tick) {
    /* 1. Fn-highlight: black out, light only Fn-active keys white (F1..F4
     *    are repainted by the BLE status in step 4). */
    if (s_fn) {
        for (uint16_t i = 0; i < n; i++) { px[i] = (struct rrgb){0, 0, 0}; }
        for (unsigned k = 0; k < FN_KEYS_COUNT; k++) {
            set_pos(px, n, fn_keys[k], (struct rrgb){255, 255, 255});
        }
    }
    /* 1b. Snap Tap confirmation: W/A/S/D green (on) or red (off), ~1 s. It
     *     starts while Fn is held (Fn+T) and outlives the Fn highlight. */
    if (tick < s_snap_until) {
        struct rrgb c = s_snap_on ? (struct rrgb){0, 255, 0} : (struct rrgb){255, 0, 0};
        for (unsigned k = 0; k < SNAP_KEYS_COUNT; k++) {
            set_pos(px, n, snap_keys[k], c);
        }
    }
    /* 2. CapsLock: white on the CapsLock key; Win lock: dim red on LGUI. */
    if (s_caps) {
        set_pos(px, n, CAPS_POS, (struct rrgb){255, 255, 255});
    }
    if (s_winlock) {
        set_pos(px, n, LGUI_POS, (struct rrgb){96, 0, 0});
    }
    /* 3. Battery gauge: 10-segment bar on the number row, ~3s window. */
    if (tick < s_bat_until) {
        uint8_t lit = (uint8_t)((s_battery * BAT_SEG_COUNT + 50) / 100);  /* 0..10 */
        uint8_t hue = (uint8_t)(85 * (uint16_t)s_battery / 100);          /* 0%=red,100%=green */
        for (uint8_t s = 0; s < BAT_SEG_COUNT; s++) {
            struct rrgb c = (s < lit) ? hsv2rgb(hue, 255, 255)
                                      : (struct rrgb){8, 8, 8};
            set_pos(px, n, (uint8_t)(BAT_SEG_FIRST + s), c);
        }
    }
    /* 4. BLE status, LAST: F1..F4 replace the Fn white with the slot/output
     * colours, and the passkey guidance wins over the battery gauge on the
     * number row. It paints only the keys it owns. */
    if (s_ble) {
        (void)rrgb_ble_render(px, n, tick);
    }
}
