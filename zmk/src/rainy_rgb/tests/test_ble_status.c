#include "../ble_status.h"
#include "test.h"
#include <string.h>

#define N 83
static struct rrgb px[N];

/* Non-default LED indices: nothing in ble_status may assume real positions. */
static const struct rrgb_ble_keys KEYS = {
	.slot = {40, 41, 42},
	.output = 43,
	.numrow = {60, 61, 62, 63, 64, 65, 66, 67, 68, 69},
	.enter = 70,
};
#define F(i)   (KEYS.slot[i])
#define F4     (KEYS.output)
#define NUM(i) (KEYS.numrow[i])
#define ENTER  (KEYS.enter)

static const struct rrgb SENT = {7, 7, 7};

static int eq(struct rrgb a, struct rrgb b) { return a.r == b.r && a.g == b.g && a.b == b.b; }
static struct rrgb blue(uint8_t v)  { return (struct rrgb){0, 0, v}; }
static struct rrgb white(uint8_t v) { return (struct rrgb){v, v, v}; }
static struct rrgb red(uint8_t v)   { return (struct rrgb){v, 0, 0}; }
static const struct rrgb BLACK = {0, 0, 0};

static bool frame(uint32_t tick) {
	for (int i = 0; i < N; i++) { px[i] = SENT; }
	return rrgb_ble_render(px, N, tick);
}

/* All pixels except the listed LEDs are untouched. */
static int only_touched(const uint8_t *leds, int count) {
	for (int i = 0; i < N; i++) {
		int owned = 0;
		for (int k = 0; k < count; k++) { if (leds[k] == i) { owned = 1; } }
		if (!owned && !eq(px[i], SENT)) { return 0; }
	}
	return 1;
}

static void slots(uint8_t a, uint8_t b, uint8_t c, uint8_t active, uint32_t tick) {
	const uint8_t s[3] = {a, b, c};
	rrgb_ble_set_slots(s, active, tick);
}

/* Breathe/pulse triangle: bright at phase 0, dark at half period. */
static uint8_t tri(uint32_t dt, uint32_t period) {
	uint32_t p = dt % period, h = period / 2;
	uint32_t d = p < h ? h - p : p - h;
	return (uint8_t)(RRGB_BLE_BRIGHT * d / h);
}

static void reset(void) {
	rrgb_ble_init(&KEYS);
	/* reach a quiet baseline: slot 0 connected and its solid/fade over */
	slots(RRGB_BLE_CONNECTED, RRGB_BLE_EMPTY, RRGB_BLE_EMPTY, 0, 0);
}
#define QUIET 1000u   /* tick after the baseline's connected animation */

static void test_timing_constants(void) {
	CHECK(RRGB_BLE_BLINK_PERIOD == 12 && RRGB_BLE_BLINK_ON == 6);
	CHECK(RRGB_BLE_BREATHE_PERIOD == 50);
	CHECK(RRGB_BLE_CONN_SOLID == 100 && RRGB_BLE_CONN_FADE == 25);
	CHECK(RRGB_BLE_FLASH_COUNT == 3 && RRGB_BLE_FLASH_ON == 8 && RRGB_BLE_FLASH_OFF == 8);
	CHECK(RRGB_BLE_FLASH_TOTAL == 48);
	CHECK(RRGB_BLE_ENTER_PERIOD == 50);
}

static void test_idle(void) {
	rrgb_ble_init(&KEYS);
	/* after init: no active slot shown, nothing to draw */
	CHECK(!rrgb_ble_active(0));
	CHECK(!frame(0));
	CHECK(only_touched(NULL, 0));

	reset();
	CHECK(!rrgb_ble_active(QUIET));
	CHECK(!frame(QUIET));
	CHECK(only_touched(NULL, 0));
}

static void test_fn_overview(void) {
	reset();
	/* slot0 active connected, slot1 connected (background), slot2 paired */
	slots(RRGB_BLE_CONNECTED, RRGB_BLE_CONNECTED, RRGB_BLE_PAIRED, 0, 0);
	uint32_t t = QUIET;
	rrgb_ble_set_fn(true);
	CHECK(rrgb_ble_active(t));
	CHECK(frame(t));
	CHECK(eq(px[F(0)], blue(RRGB_BLE_BRIGHT)));
	CHECK(eq(px[F(1)], blue(RRGB_BLE_DIM)));
	CHECK(eq(px[F(2)], blue(RRGB_BLE_VDIM)));
	CHECK(eq(px[F4], white(RRGB_BLE_BRIGHT)));   /* output USB */
	const uint8_t owned[] = {F(0), F(1), F(2), F4};
	CHECK(only_touched(owned, 4));

	rrgb_ble_set_output_ble(true);
	frame(t);
	CHECK(eq(px[F4], blue(RRGB_BLE_BRIGHT)));    /* output BLE */

	/* other slot EMPTY: very dim white; active slot 1 connected */
	slots(RRGB_BLE_EMPTY, RRGB_BLE_CONNECTED, RRGB_BLE_EMPTY, 1, t);
	frame(t + 500);
	CHECK(eq(px[F(0)], white(RRGB_BLE_VDIM)));
	CHECK(eq(px[F(2)], white(RRGB_BLE_VDIM)));

	/* active EMPTY under Fn: fast blink bright blue */
	slots(RRGB_BLE_EMPTY, RRGB_BLE_CONNECTED, RRGB_BLE_EMPTY, 2, 2000);
	frame(2000);
	CHECK(eq(px[F(2)], blue(RRGB_BLE_BRIGHT)));
	frame(2000 + RRGB_BLE_BLINK_ON);
	CHECK(eq(px[F(2)], BLACK));
	CHECK(eq(px[F(1)], blue(RRGB_BLE_DIM)));   /* other connected */

	/* active PAIRED under Fn: breathing */
	slots(RRGB_BLE_PAIRED, RRGB_BLE_CONNECTED, RRGB_BLE_EMPTY, 0, 3000);
	frame(3000 + 10);
	CHECK(eq(px[F(0)], blue(tri(10, RRGB_BLE_BREATHE_PERIOD))));

	rrgb_ble_set_fn(false);
	rrgb_ble_set_output_ble(false);
}

static void test_active_empty_blinks(void) {
	reset();
	uint32_t t0 = 5000;
	slots(RRGB_BLE_CONNECTED, RRGB_BLE_EMPTY, RRGB_BLE_EMPTY, 1, t0);
	const uint8_t owned[] = {F(1)};
	for (uint32_t dt = 0; dt < 3 * RRGB_BLE_BLINK_PERIOD; dt++) {
		CHECK(rrgb_ble_active(t0 + dt));
		CHECK(frame(t0 + dt));
		struct rrgb want = (dt % RRGB_BLE_BLINK_PERIOD) < RRGB_BLE_BLINK_ON
			? blue(RRGB_BLE_BRIGHT) : BLACK;
		CHECK(eq(px[F(1)], want));
		CHECK(only_touched(owned, 1));   /* not F4, not the other slots */
	}
	/* still blinking much later (until the state changes) */
	CHECK(rrgb_ble_active(t0 + 100000));
}

static void test_active_paired_breathes(void) {
	reset();
	uint32_t t0 = 7000;
	slots(RRGB_BLE_PAIRED, RRGB_BLE_EMPTY, RRGB_BLE_EMPTY, 0, t0);
	CHECK(eq((frame(t0), px[F(0)]), blue(RRGB_BLE_BRIGHT)));   /* bright at phase 0 */
	frame(t0 + RRGB_BLE_BREATHE_PERIOD / 2);
	CHECK(eq(px[F(0)], BLACK));                                /* dark at half period */
	for (uint32_t dt = 0; dt < 2 * RRGB_BLE_BREATHE_PERIOD; dt++) {
		frame(t0 + dt);
		CHECK(eq(px[F(0)], blue(tri(dt, RRGB_BLE_BREATHE_PERIOD))));
	}
	frame(t0 + RRGB_BLE_BREATHE_PERIOD);
	CHECK(eq(px[F(0)], blue(RRGB_BLE_BRIGHT)));
	/* non-active PAIRED slot is not animated without Fn */
	CHECK(eq(px[F(1)], SENT) && eq(px[F(2)], SENT));
}

static void test_connected_solid_fade(void) {
	reset();
	uint32_t t0 = 9000;
	slots(RRGB_BLE_CONNECTED, RRGB_BLE_PAIRED, RRGB_BLE_EMPTY, 1, t0 - 10); /* connecting */
	slots(RRGB_BLE_CONNECTED, RRGB_BLE_CONNECTED, RRGB_BLE_EMPTY, 1, t0);   /* connected */
	for (uint32_t dt = 0; dt < RRGB_BLE_CONN_SOLID; dt++) {
		CHECK(frame(t0 + dt));
		CHECK(eq(px[F(1)], blue(RRGB_BLE_BRIGHT)));
	}
	uint8_t prev = 255;
	for (uint32_t dt = RRGB_BLE_CONN_SOLID; dt < RRGB_BLE_CONN_SOLID + RRGB_BLE_CONN_FADE; dt++) {
		CHECK(rrgb_ble_active(t0 + dt));
		CHECK(frame(t0 + dt));
		uint8_t want = (uint8_t)(RRGB_BLE_BRIGHT *
			(RRGB_BLE_CONN_SOLID + RRGB_BLE_CONN_FADE - dt) / RRGB_BLE_CONN_FADE);
		CHECK(eq(px[F(1)], blue(want)));
		CHECK(want < prev);
		prev = want;
	}
	CHECK(eq((frame(t0 + RRGB_BLE_CONN_SOLID), px[F(1)]), blue(RRGB_BLE_BRIGHT)));
	uint32_t end = t0 + RRGB_BLE_CONN_SOLID + RRGB_BLE_CONN_FADE;
	CHECK(!rrgb_ble_active(end));
	CHECK(!frame(end));
	CHECK(only_touched(NULL, 0));

	/* a slot that stays connected does not restart the animation */
	slots(RRGB_BLE_CONNECTED, RRGB_BLE_CONNECTED, RRGB_BLE_EMPTY, 1, end + 5);
	CHECK(!rrgb_ble_active(end + 5));

	/* a background slot connecting also gets solid/fade */
	slots(RRGB_BLE_CONNECTED, RRGB_BLE_CONNECTED, RRGB_BLE_CONNECTED, 1, end + 10);
	frame(end + 10);
	CHECK(eq(px[F(2)], blue(RRGB_BLE_BRIGHT)));

	/* connected then dropped (polled, no LOST event): solid cancelled */
	slots(RRGB_BLE_CONNECTED, RRGB_BLE_CONNECTED, RRGB_BLE_PAIRED, 1, end + 20);
	frame(end + 20);
	CHECK(eq(px[F(2)], SENT));
}

static void check_red_flash(uint8_t slot, uint32_t t0) {
	for (uint32_t dt = 0; dt < RRGB_BLE_FLASH_TOTAL; dt++) {
		CHECK(rrgb_ble_active(t0 + dt));
		CHECK(frame(t0 + dt));
		uint32_t p = dt % (RRGB_BLE_FLASH_ON + RRGB_BLE_FLASH_OFF);
		CHECK(eq(px[F(slot)], p < RRGB_BLE_FLASH_ON ? red(RRGB_BLE_BRIGHT) : BLACK));
	}
	/* exactly COUNT rising edges */
	int edges = 0;
	bool was_on = false;
	for (uint32_t dt = 0; dt < RRGB_BLE_FLASH_TOTAL + 100; dt++) {
		frame(t0 + dt);
		bool on = eq(px[F(slot)], red(RRGB_BLE_BRIGHT));
		if (on && !was_on) { edges++; }
		was_on = on;
	}
	CHECK(edges == RRGB_BLE_FLASH_COUNT);
}

static void test_red_flash(void) {
	/* LOST on the active slot: red flash, then breathing (polled PAIRED) */
	reset();
	uint32_t t0 = 20000;
	slots(RRGB_BLE_PAIRED, RRGB_BLE_EMPTY, RRGB_BLE_EMPTY, 0, t0);
	rrgb_ble_event(RRGB_BLE_EV_LOST, 0, 0, t0);
	check_red_flash(0, t0);
	uint32_t after = t0 + RRGB_BLE_FLASH_TOTAL;
	frame(after);
	CHECK(eq(px[F(0)], blue(RRGB_BLE_BRIGHT)));            /* breathing, phase 0 */
	frame(after + RRGB_BLE_BREATHE_PERIOD / 2);
	CHECK(eq(px[F(0)], BLACK));

	/* LOST on a non-active slot: red flash, then nothing */
	reset();
	slots(RRGB_BLE_CONNECTED, RRGB_BLE_PAIRED, RRGB_BLE_EMPTY, 0, QUIET);
	rrgb_ble_event(RRGB_BLE_EV_LOST, 1, 0, QUIET);
	check_red_flash(1, QUIET);
	CHECK(!rrgb_ble_active(QUIET + RRGB_BLE_FLASH_TOTAL));
	CHECK(!frame(QUIET + RRGB_BLE_FLASH_TOTAL));

	/* FAILED on the active empty slot: red flash, then fast blink */
	reset();
	slots(RRGB_BLE_CONNECTED, RRGB_BLE_EMPTY, RRGB_BLE_EMPTY, 1, QUIET);
	rrgb_ble_event(RRGB_BLE_EV_FAILED, 1, 0, QUIET + 3);
	check_red_flash(1, QUIET + 3);
	after = QUIET + 3 + RRGB_BLE_FLASH_TOTAL;
	frame(after);
	CHECK(eq(px[F(1)], blue(RRGB_BLE_BRIGHT)));
	frame(after + RRGB_BLE_BLINK_ON);
	CHECK(eq(px[F(1)], BLACK));

	/* CLEARED on the active paired slot: red flash, then fast blink (slot now EMPTY
	 * even before the next set_slots) */
	reset();
	slots(RRGB_BLE_PAIRED, RRGB_BLE_EMPTY, RRGB_BLE_EMPTY, 0, QUIET);
	rrgb_ble_event(RRGB_BLE_EV_CLEARED, 0, 0, QUIET);
	check_red_flash(0, QUIET);
	after = QUIET + RRGB_BLE_FLASH_TOTAL;
	frame(after);
	CHECK(eq(px[F(0)], blue(RRGB_BLE_BRIGHT)));
	frame(after + RRGB_BLE_BLINK_ON);
	CHECK(eq(px[F(0)], BLACK));
	frame(after + RRGB_BLE_BLINK_PERIOD);
	CHECK(eq(px[F(0)], blue(RRGB_BLE_BRIGHT)));

	/* red flash wins over the Fn overview colour too (newest event per slot) */
	reset();
	rrgb_ble_set_fn(true);
	rrgb_ble_event(RRGB_BLE_EV_LOST, 2, 0, QUIET);
	frame(QUIET);
	CHECK(eq(px[F(2)], red(RRGB_BLE_BRIGHT)));
	frame(QUIET + RRGB_BLE_FLASH_TOTAL);
	CHECK(eq(px[F(2)], white(RRGB_BLE_VDIM)));   /* back to the overview colour */
	rrgb_ble_set_fn(false);

	/* newest event wins: a connect during a red flash replaces it */
	reset();
	slots(RRGB_BLE_CONNECTED, RRGB_BLE_PAIRED, RRGB_BLE_EMPTY, 0, QUIET);
	rrgb_ble_event(RRGB_BLE_EV_LOST, 1, 0, QUIET);
	slots(RRGB_BLE_CONNECTED, RRGB_BLE_CONNECTED, RRGB_BLE_EMPTY, 0, QUIET + RRGB_BLE_FLASH_ON);
	frame(QUIET + RRGB_BLE_FLASH_ON);
	CHECK(eq(px[F(1)], blue(RRGB_BLE_BRIGHT)));

	/* slot index out of range is ignored */
	reset();
	rrgb_ble_event(RRGB_BLE_EV_LOST, 3, 0, QUIET);
	rrgb_ble_event(RRGB_BLE_EV_PASSKEY_REQ, 4, 0, QUIET);
	CHECK(!rrgb_ble_active(QUIET));
}

static void test_passkey(void) {
	reset();
	uint32_t t0 = 30000;
	slots(RRGB_BLE_CONNECTED, RRGB_BLE_EMPTY, RRGB_BLE_EMPTY, 1, t0 - 2000);
	slots(RRGB_BLE_CONNECTED, RRGB_BLE_CONNECTED, RRGB_BLE_EMPTY, 1, t0 - 1000); /* link up */
	rrgb_ble_event(RRGB_BLE_EV_PASSKEY_REQ, 1, 0, t0);
	CHECK(rrgb_ble_active(t0));
	CHECK(frame(t0));
	for (int k = 0; k < 10; k++) { CHECK(eq(px[NUM(k)], white(RRGB_BLE_DIM))); }
	CHECK(eq(px[ENTER], white(RRGB_BLE_BRIGHT)));       /* pulse phase 0 */
	const uint8_t owned[] = {NUM(0), NUM(1), NUM(2), NUM(3), NUM(4), NUM(5), NUM(6),
				 NUM(7), NUM(8), NUM(9), ENTER};
	CHECK(only_touched(owned, 11));

	/* Enter pulses at 1 Hz */
	for (uint32_t dt = 0; dt < 2 * RRGB_BLE_ENTER_PERIOD; dt++) {
		frame(t0 + dt);
		CHECK(eq(px[ENTER], white(tri(dt, RRGB_BLE_ENTER_PERIOD))));
	}

	/* digits 0..6: n progress keys bright blue, the rest dim white */
	for (uint8_t d = 0; d <= RRGB_BLE_PASSKEY_LEN; d++) {
		rrgb_ble_event(RRGB_BLE_EV_PASSKEY_DIGITS, 1, d, t0 + 100 + d);
		frame(t0 + 100 + d);
		for (int k = 0; k < 10; k++) {
			struct rrgb want = k < d ? blue(RRGB_BLE_BRIGHT) : white(RRGB_BLE_DIM);
			CHECK(eq(px[NUM(k)], want));
		}
	}
	/* digits beyond 6 clamp to 6 */
	rrgb_ble_event(RRGB_BLE_EV_PASSKEY_DIGITS, 1, 9, t0 + 200);
	frame(t0 + 200);
	CHECK(eq(px[NUM(6)], white(RRGB_BLE_DIM)));
	/* backspace: fewer digits */
	rrgb_ble_event(RRGB_BLE_EV_PASSKEY_DIGITS, 1, 2, t0 + 201);
	frame(t0 + 201);
	CHECK(eq(px[NUM(1)], blue(RRGB_BLE_BRIGHT)));
	CHECK(eq(px[NUM(2)], white(RRGB_BLE_DIM)));

	/* PAIRED_OK: guidance ends, slot solid then fade */
	uint32_t ok = t0 + 300;
	rrgb_ble_event(RRGB_BLE_EV_PAIRED_OK, 1, 0, ok);
	frame(ok);
	CHECK(eq(px[F(1)], blue(RRGB_BLE_BRIGHT)));
	CHECK(eq(px[NUM(0)], SENT) && eq(px[ENTER], SENT));
	frame(ok + RRGB_BLE_CONN_SOLID + 1);
	CHECK(px[F(1)].b < RRGB_BLE_BRIGHT && px[F(1)].b > 0);
	CHECK(!rrgb_ble_active(ok + RRGB_BLE_CONN_SOLID + RRGB_BLE_CONN_FADE));
	/* the polled CONNECTED that follows does not restart the solid */
	slots(RRGB_BLE_CONNECTED, RRGB_BLE_CONNECTED, RRGB_BLE_EMPTY, 1,
	      ok + RRGB_BLE_CONN_SOLID + RRGB_BLE_CONN_FADE);
	CHECK(!rrgb_ble_active(ok + RRGB_BLE_CONN_SOLID + RRGB_BLE_CONN_FADE));

	/* FAILED during guidance: guidance ends, red flash on the slot */
	reset();
	slots(RRGB_BLE_CONNECTED, RRGB_BLE_EMPTY, RRGB_BLE_EMPTY, 0, QUIET);
	rrgb_ble_event(RRGB_BLE_EV_PASSKEY_REQ, 2, 0, QUIET);
	rrgb_ble_event(RRGB_BLE_EV_PASSKEY_DIGITS, 2, 3, QUIET + 10);
	rrgb_ble_event(RRGB_BLE_EV_FAILED, 2, 0, QUIET + 20);
	frame(QUIET + 20);
	CHECK(eq(px[F(2)], red(RRGB_BLE_BRIGHT)));
	CHECK(eq(px[NUM(0)], SENT) && eq(px[ENTER], SENT));
	CHECK(!rrgb_ble_active(QUIET + 20 + RRGB_BLE_FLASH_TOTAL));

	/* passkey guidance has a safety end */
	reset();
	rrgb_ble_event(RRGB_BLE_EV_PASSKEY_REQ, 0, 0, QUIET);
	CHECK(rrgb_ble_active(QUIET + RRGB_BLE_PASSKEY_MAX - 1));
	CHECK(!rrgb_ble_active(QUIET + RRGB_BLE_PASSKEY_MAX));
	CHECK(!frame(QUIET + RRGB_BLE_PASSKEY_MAX));
}

static void test_multi_slot(void) {
	reset();
	uint32_t t0 = 40000;
	/* slot 0 active pairing (blink), slot 1 connects (solid), slot 2 lost (red) */
	slots(RRGB_BLE_EMPTY, RRGB_BLE_PAIRED, RRGB_BLE_CONNECTED, 0, t0 - 1);
	slots(RRGB_BLE_EMPTY, RRGB_BLE_CONNECTED, RRGB_BLE_PAIRED, 0, t0);
	rrgb_ble_event(RRGB_BLE_EV_LOST, 2, 0, t0);
	frame(t0);
	CHECK(eq(px[F(0)], blue(RRGB_BLE_BRIGHT)));
	CHECK(eq(px[F(1)], blue(RRGB_BLE_BRIGHT)));
	CHECK(eq(px[F(2)], red(RRGB_BLE_BRIGHT)));
	frame(t0 + 8);
	CHECK(eq(px[F(0)], BLACK));                 /* blink off */
	CHECK(eq(px[F(1)], blue(RRGB_BLE_BRIGHT)));  /* solid */
	CHECK(eq(px[F(2)], BLACK));                 /* flash off */
	frame(t0 + 16);
	CHECK(eq(px[F(2)], red(RRGB_BLE_BRIGHT)));
}

static void test_active_exact(void) {
	/* rrgb_ble_active(t) == rrgb_ble_render(t) painted something, over a scenario */
	reset();
	uint32_t t0 = 50000;
	slots(RRGB_BLE_CONNECTED, RRGB_BLE_PAIRED, RRGB_BLE_EMPTY, 0, t0);
	rrgb_ble_event(RRGB_BLE_EV_LOST, 1, 0, t0 + 5);
	rrgb_ble_event(RRGB_BLE_EV_PASSKEY_REQ, 2, 0, t0 + 20);
	rrgb_ble_event(RRGB_BLE_EV_FAILED, 2, 0, t0 + 70);
	for (uint32_t t = t0; t < t0 + 300; t++) {
		bool a = rrgb_ble_active(t);
		bool p = frame(t);
		CHECK(a == p);
		int touched = 0;
		for (int i = 0; i < N; i++) { if (!eq(px[i], SENT)) { touched = 1; } }
		CHECK(touched == (int)p);
	}
	CHECK(!rrgb_ble_active(t0 + 300));
}

static void test_bounds(void) {
	/* positions >= n or NONE are skipped, no out-of-range writes */
	struct rrgb_ble_keys k = KEYS;
	k.slot[0] = RRGB_BLE_NONE;
	k.output = 200;
	rrgb_ble_init(&k);
	rrgb_ble_set_fn(true);
	struct rrgb small[50];
	for (int i = 0; i < 50; i++) { small[i] = SENT; }
	CHECK(!rrgb_ble_render(small, 30, 0));   /* every owned LED is >= 30 */
	for (int i = 0; i < 50; i++) { CHECK(eq(small[i], SENT)); }
	rrgb_ble_set_fn(false);
}

int main(void) {
	test_timing_constants();
	test_idle();
	test_fn_overview();
	test_active_empty_blinks();
	test_active_paired_breathes();
	test_connected_solid_fade();
	test_red_flash();
	test_passkey();
	test_multi_slot();
	test_active_exact();
	test_bounds();
	DONE();
}
