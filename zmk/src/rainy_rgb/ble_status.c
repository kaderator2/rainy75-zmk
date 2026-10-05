#include <stdint.h>
#include <stdbool.h>
#include "ble_status.h"

/* BLE slot status indicators. Pure, ZMK-free; see ble_status.h.
 *
 * Per slot key (F1..F3), highest priority first:
 *   1. a timed animation (connected solid+fade, or red flash), newest wins;
 *   2. the active slot, not connected, blinks (EMPTY, pairing) or breathes
 *      (PAIRED, connecting): with Fn held always, without Fn only while the
 *      output is BLE and for RRGB_BLE_STEADY_HOLD_FRAMES after the slot's
 *      last event (boot/wake, profile select, state change, flash end);
 *   3. with Fn held: the overview colour.
 * F4 shows the output (white USB / blue BLE) only with Fn held. The passkey
 * guidance owns the number row and Enter while the host waits for the code.
 *
 * Timing uses wrap-safe tick differences (int32). Expired animation kinds
 * and an expired s_pk_on are never cleared: harmless until the tick has
 * moved ~2^31 frames (~497 days at 50 FPS) past them, when the difference
 * turns negative (still off); only after ~2^32 frames could one reappear. Setters run on the ZMK event
 * thread, render/active on the render thread: single writer, single core,
 * a torn read is a one-frame glitch. */

enum anim_kind { ANIM_NONE = 0, ANIM_CONN, ANIM_FLASH };

struct slot_anim {
	uint8_t kind;    /* enum anim_kind, kept after expiry (flash end = blink origin) */
	uint32_t t0;
};

static struct rrgb_ble_keys s_keys;
static volatile uint8_t s_state[RRGB_BLE_SLOTS];
static volatile uint8_t s_active = RRGB_BLE_NONE;
static volatile uint32_t s_steady_t0;    /* active slot's last steady event */
static volatile bool s_first_poll;       /* next set_slots is boot/wake: an event */
static volatile bool s_output_ble;
static volatile bool s_fn;
static struct slot_anim s_anim[RRGB_BLE_SLOTS];

static volatile bool s_pk_on;
static volatile uint8_t s_pk_slot;
static volatile uint8_t s_pk_digits;
static volatile uint32_t s_pk_t0;        /* Enter pulse origin */
static volatile uint32_t s_pk_last;      /* last passkey event, for the safety end */

static int32_t since(uint32_t tick, uint32_t t0) { return (int32_t)(tick - t0); }

static struct rrgb blue(uint8_t v)  { return (struct rrgb){0, 0, v}; }
static struct rrgb white(uint8_t v) { return (struct rrgb){v, v, v}; }
static struct rrgb red(uint8_t v)   { return (struct rrgb){v, 0, 0}; }

/* Triangle: RRGB_BLE_BRIGHT at phase 0, 0 at half period. */
static uint8_t tri(int32_t dt, uint32_t period) {
	uint32_t p = (uint32_t)dt % period, h = period / 2;
	uint32_t d = p < h ? h - p : p - h;
	return (uint8_t)(RRGB_BLE_BRIGHT * d / h);
}

static bool anim_running(uint8_t s, uint32_t tick) {
	int32_t dt = since(tick, s_anim[s].t0);
	switch (s_anim[s].kind) {
	case ANIM_CONN:  return dt >= 0 && dt < RRGB_BLE_CONN_SOLID + RRGB_BLE_CONN_FADE;
	case ANIM_FLASH: return dt >= 0 && dt < RRGB_BLE_FLASH_TOTAL;
	default:         return false;
	}
}

static bool pk_running(uint32_t tick) {
	int32_t dt = since(tick, s_pk_last);
	return s_pk_on && dt >= 0 && dt < RRGB_BLE_PASSKEY_MAX;
}

/* Steady phase origin: the last steady event, or the end of a later red
 * flash on the slot (the flash comes first, then the steady animation). */
static uint32_t steady_origin(uint8_t s) {
	uint32_t origin = s_steady_t0;
	if (s_anim[s].kind == ANIM_FLASH) {
		uint32_t end = s_anim[s].t0 + RRGB_BLE_FLASH_TOTAL;
		if (since(end, origin) > 0) { origin = end; }
	}
	return origin;
}

/* Slot s shows its steady (blink/breathe) animation this frame. */
static bool steady_shown(uint8_t s, uint32_t tick) {
	if (s != s_active || s_state[s] == RRGB_BLE_CONNECTED) { return false; }
	if (s_fn) { return true; }
	int32_t dt = since(tick, steady_origin(s));
	return s_output_ble && dt >= 0 && dt < RRGB_BLE_STEADY_HOLD_FRAMES;
}

static void start_anim(uint8_t s, uint8_t kind, uint32_t tick) {
	s_anim[s].t0 = tick;
	s_anim[s].kind = kind;
}

void rrgb_ble_init(const struct rrgb_ble_keys *keys) {
	s_keys = *keys;
	for (uint8_t s = 0; s < RRGB_BLE_SLOTS; s++) {
		s_state[s] = RRGB_BLE_EMPTY;
		s_anim[s] = (struct slot_anim){ANIM_NONE, 0};
	}
	s_active = RRGB_BLE_NONE;
	s_steady_t0 = 0;
	s_first_poll = true;
	s_output_ble = false;
	s_fn = false;
	s_pk_on = false;
	s_pk_digits = 0;
}

void rrgb_ble_set_slots(const uint8_t state[3], uint8_t active, uint32_t tick) {
	if (active >= RRGB_BLE_SLOTS) { active = RRGB_BLE_NONE; }
	bool steady_changed = s_first_poll || active != s_active;
	s_first_poll = false;
	for (uint8_t s = 0; s < RRGB_BLE_SLOTS; s++) {
		uint8_t old = s_state[s], now = state[s];
		if (now == old || now > RRGB_BLE_CONNECTED) { continue; }   /* bad value: ignored */
		if (now == RRGB_BLE_CONNECTED) {
			start_anim(s, ANIM_CONN, tick);
		} else if (old == RRGB_BLE_CONNECTED && now == RRGB_BLE_PAIRED) {
			start_anim(s, ANIM_FLASH, tick);   /* LOST (still bonded) */
		} else if (s_anim[s].kind == ANIM_CONN) {
			/* CONNECTED -> EMPTY is a bond clear, not a LOST: CLEARED flashes */
			s_anim[s].kind = ANIM_NONE;
		}
		s_state[s] = now;
		if (s == active) { steady_changed = true; }
	}
	s_active = active;
	if (steady_changed) { s_steady_t0 = tick; }
}

void rrgb_ble_set_output_ble(bool ble) { s_output_ble = ble; }
void rrgb_ble_set_fn(bool held)        { s_fn = held; }

static void pk_end(uint8_t slot) {
	if (s_pk_on && s_pk_slot == slot) { s_pk_on = false; }
}

void rrgb_ble_event(enum rrgb_ble_ev ev, uint8_t slot, uint8_t arg, uint32_t tick) {
	if (slot >= RRGB_BLE_SLOTS) { return; }
	switch (ev) {
	case RRGB_BLE_EV_LOST:
	case RRGB_BLE_EV_FAILED:
		pk_end(slot);
		start_anim(slot, ANIM_FLASH, tick);
		break;
	case RRGB_BLE_EV_CLEARED:
		pk_end(slot);
		s_state[slot] = RRGB_BLE_EMPTY;
		if (slot == s_active) { s_steady_t0 = tick; }
		start_anim(slot, ANIM_FLASH, tick);
		break;
	case RRGB_BLE_EV_PASSKEY_REQ:
		s_pk_slot = slot;
		s_pk_digits = 0;
		s_pk_t0 = tick;
		s_pk_last = tick;
		s_pk_on = true;
		break;
	case RRGB_BLE_EV_PASSKEY_DIGITS:
		if (!s_pk_on || s_pk_slot != slot) {   /* missed REQ: start the guidance now */
			s_pk_slot = slot;
			s_pk_t0 = tick;
			s_pk_on = true;
		}
		s_pk_digits = arg > RRGB_BLE_PASSKEY_LEN ? RRGB_BLE_PASSKEY_LEN : arg;
		s_pk_last = tick;
		break;
	case RRGB_BLE_EV_PAIRED_OK:
		pk_end(slot);
		s_state[slot] = RRGB_BLE_CONNECTED;   /* the following poll does not restart it */
		start_anim(slot, ANIM_CONN, tick);
		break;
	}
}

bool rrgb_ble_active(uint32_t tick) {
	if (s_fn || pk_running(tick)) { return true; }
	for (uint8_t s = 0; s < RRGB_BLE_SLOTS; s++) {
		if (anim_running(s, tick)) { return true; }
	}
	return s_active < RRGB_BLE_SLOTS && steady_shown(s_active, tick);
}

static bool put(struct rrgb *px, uint16_t n, uint8_t led, struct rrgb c) {
	if (led == RRGB_BLE_NONE || led >= n) { return false; }
	px[led] = c;
	return true;
}

static struct rrgb timed_colour(uint8_t s, uint32_t tick) {
	int32_t dt = since(tick, s_anim[s].t0);
	if (s_anim[s].kind == ANIM_FLASH) {
		bool on = (uint32_t)dt % (RRGB_BLE_FLASH_ON + RRGB_BLE_FLASH_OFF) < RRGB_BLE_FLASH_ON;
		return red(on ? RRGB_BLE_BRIGHT : 0);
	}
	if (dt < RRGB_BLE_CONN_SOLID) { return blue(RRGB_BLE_BRIGHT); }
	return blue((uint8_t)(RRGB_BLE_BRIGHT *
		(uint32_t)(RRGB_BLE_CONN_SOLID + RRGB_BLE_CONN_FADE - dt) / RRGB_BLE_CONN_FADE));
}

/* Active slot not connected: blink (EMPTY, pairing) or breathe (PAIRED, connecting),
 * phase 0 at steady_origin(). */
static struct rrgb steady_colour(uint8_t s, uint32_t tick) {
	int32_t dt = since(tick, steady_origin(s));
	if (dt < 0) { dt = 0; }
	if (s_state[s] == RRGB_BLE_EMPTY) {
		bool on = (uint32_t)dt % RRGB_BLE_BLINK_PERIOD < RRGB_BLE_BLINK_ON;
		return blue(on ? RRGB_BLE_BRIGHT : 0);
	}
	return blue(tri(dt, RRGB_BLE_BREATHE_PERIOD));
}

static struct rrgb overview_colour(uint8_t s) {
	switch (s_state[s]) {
	case RRGB_BLE_CONNECTED: return blue(s == s_active ? RRGB_BLE_BRIGHT : RRGB_BLE_DIM);
	case RRGB_BLE_PAIRED:    return blue(RRGB_BLE_VDIM);
	default:                 return white(RRGB_BLE_VDIM);
	}
}

bool rrgb_ble_render(struct rrgb *px, uint16_t n, uint32_t tick) {
	bool painted = false;

	for (uint8_t s = 0; s < RRGB_BLE_SLOTS; s++) {
		struct rrgb c;
		if (anim_running(s, tick)) {
			c = timed_colour(s, tick);
		} else if (steady_shown(s, tick)) {
			c = steady_colour(s, tick);
		} else if (s_fn) {
			c = overview_colour(s);
		} else {
			continue;
		}
		painted |= put(px, n, s_keys.slot[s], c);
	}

	if (s_fn) {
		painted |= put(px, n, s_keys.output,
			       s_output_ble ? blue(RRGB_BLE_BRIGHT) : white(RRGB_BLE_BRIGHT));
	}

	if (pk_running(tick)) {
		for (uint8_t k = 0; k < 10; k++) {
			struct rrgb c = k < s_pk_digits ? blue(RRGB_BLE_BRIGHT) : white(RRGB_BLE_DIM);
			painted |= put(px, n, s_keys.numrow[k], c);
		}
		painted |= put(px, n, s_keys.enter,
			       white(tri(since(tick, s_pk_t0), RRGB_BLE_ENTER_PERIOD)));
	}
	return painted;
}
