/*
 * Copyright (c) 2026 scholzri
 * SPDX-License-Identifier: Apache-2.0
 *
 * ll_fifo.h host tests: record placement in a FIFO byte area and the
 * word copy.
 */
#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include "test.h"
#include "../ll_fifo.h"

/* A reference FIFO of records driven through ll_fifo_place: every placed
 * record must lie inside the area and overlap no live record. */
#define AREA 256
#define MAXR 64

static struct {
	uint16_t off[MAXR], len[MAXR];
	unsigned head, tail;
} f;

static bool overlaps(uint16_t a, uint16_t an, uint16_t b, uint16_t bn)
{
	return a < b + bn && b < a + an;
}

static int put(uint16_t len)
{
	unsigned n = f.head - f.tail;
	unsigned o = f.tail % MAXR, w = (f.head - 1) % MAXR;
	int32_t at = ll_fifo_place(AREA, (uint8_t)n, n ? f.off[o] : 0, n ? f.off[w] : 0,
				   n ? (uint16_t)(f.off[w] + ll_fifo_size(f.len[w])) : 0, len);

	if (at < 0) {
		return -1;
	}
	CHECK((uint32_t)at + ll_fifo_size(len) <= AREA);
	CHECK((at & 3) == 0);
	for (unsigned i = f.tail; i != f.head; i++) {
		CHECK(!overlaps((uint16_t)at, ll_fifo_size(len), f.off[i % MAXR],
				ll_fifo_size(f.len[i % MAXR])));
	}
	f.off[f.head % MAXR] = (uint16_t)at;
	f.len[f.head % MAXR] = len;
	f.head++;
	return at;
}

static void pop(void)
{
	f.tail++;
}

static void test_size(void)
{
	CHECK(ll_fifo_size(0) == 4);
	CHECK(ll_fifo_size(1) == 4);
	CHECK(ll_fifo_size(4) == 4);
	CHECK(ll_fifo_size(5) == 8);
	CHECK(ll_fifo_size(31) == 32);
	CHECK(ll_fifo_size(255) == 256);
}

static void test_place(void)
{
	/* empty: at 0, if it fits at all */
	CHECK(ll_fifo_place(AREA, 0, 0, 0, 0, 255) == 0);
	CHECK(ll_fifo_place(AREA, 0, 0, 0, 0, 256) == 0);
	CHECK(ll_fifo_place(AREA, 0, 0, 0, 0, 257) == -1);
	CHECK(ll_fifo_place(1024, 0, 100, 200, 300, 31) == 0);   /* stale offsets ignored */
	/* behind the newest */
	CHECK(ll_fifo_place(AREA, 1, 0, 0, 32, 31) == 32);
	CHECK(ll_fifo_place(AREA, 2, 32, 64, 96, 160) == 96);
	CHECK(ll_fifo_place(AREA, 2, 32, 64, 96, 161) == -1);    /* end too short, start too short */
	/* end too short, start free: wraps to 0 */
	CHECK(ll_fifo_place(AREA, 1, 64, 64, 224, 64) == 0);
	CHECK(ll_fifo_place(AREA, 1, 64, 64, 224, 65) == -1);
	CHECK(ll_fifo_place(AREA, 1, 64, 64, 224, 32) == 224);   /* fits at the end first */
	/* wrapped: the gap between the newest end and the oldest */
	CHECK(ll_fifo_place(AREA, 2, 128, 0, 64, 64) == 64);
	CHECK(ll_fifo_place(AREA, 2, 128, 0, 64, 65) == -1);
	CHECK(ll_fifo_place(AREA, 2, 128, 0, 128, 1) == -1);     /* full: no zero-size gap */
	/* a full unwrapped area */
	CHECK(ll_fifo_place(AREA, 2, 0, 128, 256, 1) == -1);
}

static void test_random(void)
{
	uint32_t x = 12345;
	unsigned placed = 0, refused = 0;

	memset(&f, 0, sizeof(f));
	for (int i = 0; i < 200000; i++) {
		x = x * 1103515245u + 12345u;
		if (((x >> 16) & 3) != 0 && f.head - f.tail < MAXR) {
			uint16_t len = (uint16_t)((x >> 8) % 128);

			if (put(len) >= 0) {
				placed++;
			} else {
				refused++;
				CHECK(f.head != f.tail);   /* an empty area takes any record <= AREA */
			}
		} else if (f.head != f.tail) {
			pop();
		}
	}
	CHECK(placed > 30000 && refused > 1000);
}

static void test_copy(void)
{
	uint8_t src[300], dst[300];

	for (int i = 0; i < 300; i++) {
		src[i] = (uint8_t)(i * 7 + 1);
	}
	for (unsigned so = 0; so < 4; so++) {
		for (unsigned dof = 0; dof < 4; dof++) {
			for (uint16_t n = 0; n < 260; n += 13) {
				memset(dst, 0xEE, sizeof(dst));
				ll_fifo_copy(&dst[dof], &src[so], n);
				CHECK(memcmp(&dst[dof], &src[so], n) == 0);
				CHECK(dst[dof + n] == 0xEE);
				if (dof) {
					CHECK(dst[dof - 1] == 0xEE);
				}
			}
		}
	}
}

int main(void)
{
	test_size();
	test_place();
	test_random();
	test_copy();
	DONE();
}
