#include "../overlay.h"
#include "../led_map.h"
#include "../ble_status.h"
#include "test.h"
#include <string.h>

static int eq(struct rrgb a, struct rrgb b) { return a.r == b.r && a.g == b.g && a.b == b.b; }
static struct rrgb blue(uint8_t v)  { return (struct rrgb){0, 0, v}; }
static struct rrgb white(uint8_t v) { return (struct rrgb){v, v, v}; }
static struct rrgb at(const struct rrgb *px, int pos) { return px[rrgb_led_for_position((uint32_t)pos)]; }

#define POS_F(i)  (1 + (i))   /* F1..F3 = positions 1..3 */
#define POS_F4    4
#define POS_F5    5
#define POS_NUM(i) (16 + (i)) /* number row 1..0 */
#define POS_ENTER 43          /* ISO Enter */

/* ble_status integration: render order, ownership, overlay_active. */
static void test_ble(void) {
    struct rrgb px[83];
    const uint8_t all_empty[3] = {RRGB_BLE_EMPTY, RRGB_BLE_EMPTY, RRGB_BLE_EMPTY};

    rrgb_overlay_init(true);
    rrgb_overlay_set_caps(false);
    rrgb_overlay_set_fn(false);
    rrgb_overlay_set_battery(0);
    rrgb_overlay_battery_show(0);
    uint32_t t = 1000;                          /* past the battery window */

    /* idle: nothing to draw, nothing touched */
    CHECK(!rrgb_overlay_active(t));
    for (int i = 0; i < 83; i++) { px[i] = (struct rrgb){7, 7, 7}; }
    rrgb_overlay_render(px, 83, t);
    for (int i = 0; i < 83; i++) { CHECK(eq(px[i], (struct rrgb){7, 7, 7})); }

    /* Fn held, all slots EMPTY, output USB: ble owns F1..F4, the rest of the
     * Fn-highlight stays white (ble does not paint keys it does not own). */
    rrgb_overlay_set_fn(true);
    CHECK(rrgb_overlay_active(t));
    for (int i = 0; i < 83; i++) { px[i] = (struct rrgb){50, 50, 50}; }
    rrgb_overlay_render(px, 83, t);
    for (int s = 0; s < 3; s++) { CHECK(eq(at(px, POS_F(s)), white(RRGB_BLE_VDIM))); }
    CHECK(eq(at(px, POS_F4), white(RRGB_BLE_BRIGHT)));
    CHECK(eq(at(px, 0), white(255)));            /* ESC: Fn white */
    CHECK(eq(at(px, POS_F5), white(255)));       /* F5: Fn white */
    CHECK(eq(at(px, 13), white(255)));           /* BT_CLR key: Fn white */
    CHECK(eq(at(px, 31), white(0)));             /* Q: black */

    /* slot states through the overlay: active connected / other paired, BLE output */
    const uint8_t mixed[3] = {RRGB_BLE_CONNECTED, RRGB_BLE_PAIRED, RRGB_BLE_EMPTY};
    rrgb_ble_set_output_ble(true);
    rrgb_ble_set_slots(mixed, 0, t);
    t += RRGB_BLE_CONN_SOLID + RRGB_BLE_CONN_FADE;   /* connected solid+fade over */
    rrgb_overlay_render(px, 83, t);
    CHECK(eq(at(px, POS_F(0)), blue(RRGB_BLE_BRIGHT)));
    CHECK(eq(at(px, POS_F(1)), blue(RRGB_BLE_VDIM)));
    CHECK(eq(at(px, POS_F(2)), white(RRGB_BLE_VDIM)));
    CHECK(eq(at(px, POS_F4), blue(RRGB_BLE_BRIGHT)));
    CHECK(eq(at(px, POS_F5), white(255)));

    /* Fn released, everything connected and settled: nothing to draw */
    rrgb_overlay_set_fn(false);
    CHECK(!rrgb_overlay_active(t));
    for (int i = 0; i < 83; i++) { px[i] = (struct rrgb){7, 7, 7}; }
    rrgb_overlay_render(px, 83, t);
    CHECK(eq(at(px, POS_F(0)), (struct rrgb){7, 7, 7}));

    /* CapsLock and a ble animation at the same time: both show */
    rrgb_overlay_set_caps(true);
    rrgb_ble_event(RRGB_BLE_EV_FAILED, 1, 0, t);
    CHECK(rrgb_overlay_active(t));
    rrgb_overlay_render(px, 83, t);
    CHECK(eq(at(px, 44), white(255)));                          /* CapsLock */
    CHECK(eq(at(px, POS_F(1)), (struct rrgb){RRGB_BLE_BRIGHT, 0, 0}));   /* red flash on */
    rrgb_overlay_set_caps(false);
    t += RRGB_BLE_FLASH_TOTAL;

    /* passkey guidance wins over the battery gauge on the number row */
    rrgb_overlay_set_battery(100);
    rrgb_overlay_battery_show(t);
    rrgb_ble_event(RRGB_BLE_EV_PASSKEY_REQ, 2, 0, t);
    rrgb_ble_event(RRGB_BLE_EV_PASSKEY_DIGITS, 2, 2, t);
    for (int i = 0; i < 83; i++) { px[i] = (struct rrgb){7, 7, 7}; }
    rrgb_overlay_render(px, 83, t);
    CHECK(eq(at(px, POS_NUM(0)), blue(RRGB_BLE_BRIGHT)));
    CHECK(eq(at(px, POS_NUM(1)), blue(RRGB_BLE_BRIGHT)));
    for (int k = 2; k < 10; k++) { CHECK(eq(at(px, POS_NUM(k)), white(RRGB_BLE_DIM))); }
    CHECK(eq(at(px, POS_ENTER), white(RRGB_BLE_BRIGHT)));       /* Enter pulse, phase 0 */
    CHECK(eq(at(px, 15), (struct rrgb){7, 7, 7}));              /* ` untouched */
    CHECK(eq(at(px, 26), (struct rrgb){7, 7, 7}));              /* - untouched */

    /* guidance over: the gauge shows again in its window */
    rrgb_ble_event(RRGB_BLE_EV_PAIRED_OK, 2, 0, t + 1);
    rrgb_overlay_render(px, 83, t + 1);
    CHECK(at(px, POS_NUM(9)).g > 30);                           /* 100 % green */

    /* RGB off with only a ble animation: overlay_active keeps the loop alive */
    t += 1000;                                                  /* all settled */
    CHECK(!rrgb_overlay_active(t));
    rrgb_ble_set_slots(all_empty, 1, t);                        /* active slot 1 cleared */
    CHECK(rrgb_overlay_active(t));                              /* blinking */
    for (int i = 0; i < 83; i++) { px[i] = (struct rrgb){0, 0, 0}; }
    rrgb_overlay_render(px, 83, t);
    CHECK(eq(at(px, POS_F(1)), blue(RRGB_BLE_BRIGHT)));
    rrgb_ble_set_output_ble(false);
    CHECK(!rrgb_overlay_active(t));                             /* USB: steady gated */

    /* build without BLE: ble owns nothing, Fn-highlight unchanged */
    rrgb_overlay_init(false);
    rrgb_overlay_set_fn(true);
    rrgb_ble_event(RRGB_BLE_EV_PASSKEY_REQ, 0, 0, t);
    for (int i = 0; i < 83; i++) { px[i] = (struct rrgb){50, 50, 50}; }
    rrgb_overlay_render(px, 83, t);
    for (int s = 0; s < 3; s++) { CHECK(eq(at(px, POS_F(s)), white(255))); }
    CHECK(eq(at(px, POS_F4), white(255)));
    CHECK(eq(at(px, POS_NUM(0)), white(0)));
    rrgb_overlay_set_fn(false);
    CHECK(!rrgb_overlay_active(t));         /* ble state ignored without BLE */
}

int main(void) {
    struct rrgb px[83];

    /* reset state */
    rrgb_overlay_set_caps(false);
    rrgb_overlay_set_fn(false);
    rrgb_overlay_set_battery(0);

    /* nothing active -> render leaves pixels untouched */
    memset(px, 7, sizeof(px));
    rrgb_overlay_render(px, 83, 100);
    CHECK(px[0].r == 7 && px[10].g == 7);           /* untouched */

    /* CapsLock on -> CapsLock LED (pos 44) is white, others untouched */
    memset(px, 0, sizeof(px));
    rrgb_overlay_set_caps(true);
    rrgb_overlay_render(px, 83, 100);
    int caps_led = rrgb_led_for_position(44);
    CHECK(caps_led >= 0);
    CHECK(px[caps_led].r == 255 && px[caps_led].g == 255 && px[caps_led].b == 255);
    rrgb_overlay_set_caps(false);

    /* Fn active -> only Fn keys lit white, rest black */
    for (int i = 0; i < 83; i++) { px[i] = (struct rrgb){50,50,50}; }
    rrgb_overlay_set_fn(true);
    rrgb_overlay_render(px, 83, 100);
    int led_esc = rrgb_led_for_position(0);    /* ESC: Fn-active */
    int led_q   = rrgb_led_for_position(31);   /* Q: NOT Fn-active */
    CHECK(px[led_esc].r == 255);               /* Fn key lit */
    CHECK((px[led_q].r|px[led_q].g|px[led_q].b) == 0);  /* non-Fn key black */
    rrgb_overlay_set_fn(false);

    /* battery gauge: 50% -> 5 of 10 segments lit on number row (pos 16..25) */
    memset(px, 0, sizeof(px));
    rrgb_overlay_set_battery(50);
    rrgb_overlay_battery_show(0);              /* window until tick 90 */
    rrgb_overlay_render(px, 83, 10);           /* tick 10 < 90 -> active */
    int seg0 = rrgb_led_for_position(16);      /* first segment (lit) */
    int seg9 = rrgb_led_for_position(25);      /* last segment (unlit, dim) */
    CHECK((px[seg0].r|px[seg0].g|px[seg0].b) > 30);   /* lit */
    CHECK(px[seg9].r < 20 && px[seg9].g < 20 && px[seg9].b < 20); /* dim/unlit */

    /* gauge window expires */
    memset(px, 0, sizeof(px));
    rrgb_overlay_render(px, 83, 100);          /* tick 100 >= 90 -> no gauge */
    CHECK((px[seg0].r|px[seg0].g|px[seg0].b) == 0);

    /* rrgb_overlay_active: true while caps/fn/battery-window, else false
       (so indicators render even when decorative RGB is off) */
    rrgb_overlay_set_caps(false);
    rrgb_overlay_set_fn(false);
    rrgb_overlay_battery_show(0);              /* window until tick 90 */
    CHECK(rrgb_overlay_active(10));            /* battery window open */
    CHECK(!rrgb_overlay_active(100));          /* window closed, nothing else */
    rrgb_overlay_set_caps(true);
    CHECK(rrgb_overlay_active(100));           /* caps */
    rrgb_overlay_set_caps(false);
    rrgb_overlay_set_fn(true);
    CHECK(rrgb_overlay_active(100));           /* fn */
    rrgb_overlay_set_fn(false);
    CHECK(!rrgb_overlay_active(100));          /* nothing active */

    test_ble();
    DONE();
}
