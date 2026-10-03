/*
 * Copyright (c) 2026 scholzri
 * SPDX-License-Identifier: Apache-2.0
 *
 * Per-link host credits and up state (rules in ll_credit.h).
 */
#include <string.h>

#include "ll_credit.h"
#include "ll_plat.h"

static struct {
	bool up;
	uint32_t gen;
	uint32_t count;
} cr[LL_MAX_CONN];

void ll_credit_init(void)
{
	unsigned int key = ll_plat_lock();

	memset(cr, 0, sizeof(cr));
	ll_plat_unlock(key);
}

void ll_credit_open(uint8_t link)
{
	unsigned int key;

	if (link >= LL_MAX_CONN) {
		return;
	}
	key = ll_plat_lock();
	cr[link].count = 0;
	cr[link].gen++;
	cr[link].up = true;
	ll_plat_unlock(key);
}

bool ll_credit_close(uint8_t link)
{
	unsigned int key;
	bool was;

	if (link >= LL_MAX_CONN) {
		return false;
	}
	key = ll_plat_lock();
	was = cr[link].up;
	cr[link].count = 0;
	cr[link].up = false;
	ll_plat_unlock(key);
	return was;
}

bool ll_credit_up(uint8_t link)
{
	return link < LL_MAX_CONN && cr[link].up;
}

bool ll_credit_up_gen(uint8_t link, uint32_t *gen)
{
	unsigned int key;
	bool up;

	if (link >= LL_MAX_CONN) {
		*gen = 0;
		return false;
	}
	key = ll_plat_lock();
	up = cr[link].up;
	*gen = cr[link].gen;
	ll_plat_unlock(key);
	return up;
}

uint32_t ll_credit_gen(uint8_t link)
{
	return link < LL_MAX_CONN ? cr[link].gen : 0;
}

void ll_credit_acked(uint8_t link)
{
	unsigned int key;

	if (link >= LL_MAX_CONN) {
		return;
	}
	key = ll_plat_lock();
	cr[link].count++;
	ll_plat_unlock(key);
}

bool ll_credit_back(uint8_t link)
{
	unsigned int key;
	bool counted = false;

	if (link >= LL_MAX_CONN) {
		return false;
	}
	key = ll_plat_lock();
	if (cr[link].up) {
		cr[link].count++;
		counted = true;
	}
	ll_plat_unlock(key);
	return counted;
}

uint16_t ll_credit_take(uint8_t link)
{
	unsigned int key;
	uint32_t n;

	if (link >= LL_MAX_CONN) {
		return 0;
	}
	key = ll_plat_lock();
	n = cr[link].up ? cr[link].count : 0;
	cr[link].count = 0;
	ll_plat_unlock(key);
	return n > UINT16_MAX ? UINT16_MAX : (uint16_t)n;
}
