/*
 * Copyright (c) 2026 scholzri
 * SPDX-License-Identifier: Apache-2.0
 *
 * Byte FIFO helpers shared by ll_txq and ll_rxq (slice 6b Task 4, long
 * PDUs). Both queues keep their PDUs as variable-length records in one
 * per-link byte area instead of fixed 255-byte slots: a record is freed
 * only after every older one (FIFO), so the free space is one contiguous
 * gap (two before a wrap). Pure, header only.
 */
#ifndef LL_FIFO_H_
#define LL_FIFO_H_

#include <stdint.h>
#include <string.h>

/* Records start on a 4-byte boundary of the area (sizes are rounded up), so
 * the queues can place the area 2 bytes past a word boundary and copy with
 * whole words against the radio's DMA entries, whose payload starts at
 * offset 6 (ll_fifo_copy). */
#define LL_FIFO_ALIGN 4u

static inline uint16_t ll_fifo_size(uint16_t len)
{
	/* a 0-byte record still takes room, so a full area is never mistaken
	 * for an empty gap */
	return (uint16_t)((len + LL_FIFO_ALIGN - (len ? 1u : 0u)) & ~(LL_FIFO_ALIGN - 1u));
}

/* Where a record of len bytes goes in an area of size bytes (a multiple of
 * LL_FIFO_ALIGN), given the live records: n of them, the oldest starting at
 * oldest_off, the newest starting at newest_off and ending at newest_end
 * (offset after its rounded size). Returns the offset, or -1 when it does
 * not fit now. A record never wraps: it goes behind the newest one, or at 0
 * when the end of the area is too short and the start is free. */
static inline int32_t ll_fifo_place(uint16_t size, uint8_t n, uint16_t oldest_off,
				    uint16_t newest_off, uint16_t newest_end, uint16_t len)
{
	uint16_t need = ll_fifo_size(len);

	if (n == 0) {
		return need <= size ? 0 : -1;
	}
	if (newest_off >= oldest_off) {
		/* live region [oldest_off, newest_end) */
		if ((uint32_t)size - newest_end >= need) {
			return newest_end;
		}
		return oldest_off >= need ? 0 : -1;
	}
	/* wrapped: live [oldest_off, end) and [0, newest_end) */
	return (uint32_t)(oldest_off - newest_end) >= need ? (int32_t)newest_end : -1;
}

/* memcpy, with 32-bit words when dst and src have the same alignment
 * (the picolibc memcpy of the device build is a byte loop, -Os). */
typedef uint32_t __attribute__((may_alias)) ll_fifo_word_t;

static inline void ll_fifo_copy(void *dst, const void *src, uint16_t n)
{
	uint8_t *d = (uint8_t *)dst;
	const uint8_t *s = (const uint8_t *)src;

	if ((((uintptr_t)d ^ (uintptr_t)s) & 3u) != 0) {
		memcpy(d, s, n);
		return;
	}
	while (n && ((uintptr_t)d & 3u)) {
		*d++ = *s++;
		n--;
	}
	while (n >= 4) {
		*(ll_fifo_word_t *)(void *)d = *(const ll_fifo_word_t *)(const void *)s;
		d += 4;
		s += 4;
		n -= 4;
	}
	while (n--) {
		*d++ = *s++;
	}
}

#endif /* LL_FIFO_H_ */
