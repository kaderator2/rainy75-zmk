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

/* Fn-active keymap positions (non-&trans on layer 1). KEYMAP-COUPLED:
 * update this if the Fn layer changes. */
static const uint8_t fn_keys[] = {
    0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,   /* top row: studio/BT/output/media/BT_CLR */
    28,   /* BKSP  -> RGB_TOG */
    43,   /* Enter -> RGB_EFF */
    56,   /* NUHS  -> RGB_HUI */
    65,   /* B     -> RGB_BAT */
    72,   /* UP    -> RGB_BRI */
    80, 81, 82,                            /* arrows -> speed/bright */
};
#define FN_KEYS_COUNT (sizeof(fn_keys) / sizeof(fn_keys[0]))

/* BLE status keys (ble_status.h), keymap positions. KEYMAP-COUPLED:
 * F1..F3 = &bt BT_SEL 0..2, F4 = &out OUT_TOG on the Fn layer; the number
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

bool rrgb_overlay_key_reactive(uint32_t position, uint32_t tick) {
    (void)position;   /* every Fn-layer press is a command, not only F1..F4 */
    return !s_fn && !rrgb_overlay_suppress_effect(tick);
}

bool rrgb_overlay_active(uint32_t tick) {
    return s_caps || s_fn || (tick < s_bat_until) || (s_ble && rrgb_ble_active(tick));
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
    /* 2. CapsLock: white on the CapsLock key. */
    if (s_caps) {
        set_pos(px, n, CAPS_POS, (struct rrgb){255, 255, 255});
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
