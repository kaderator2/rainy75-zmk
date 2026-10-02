/*
 * Copyright (c) 2026 scholzri
 * SPDX-License-Identifier: Apache-2.0
 *
 * Platform hooks for the link layer (implemented in ll_glue.c).
 */
#ifndef LL_PLAT_H_
#define LL_PLAT_H_

#include <stdint.h>

uint32_t ll_plat_rand32(void);
unsigned int ll_plat_lock(void);
void ll_plat_unlock(unsigned int key);

#endif /* LL_PLAT_H_ */
