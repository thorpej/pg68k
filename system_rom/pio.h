/*
 * Copyright (c) 2025 Jason R. Thorpe.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR ``AS IS'' AND ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
 * IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 */

#ifndef pio_h_included
#define	pio_h_included

/*
 * ISA-like device access functions.
 */

#include "config.h"
#include "systypes.h"
#include "endian.h"

#define	PA_TO_ADDR(t, pa)	((t)(pa))

/***** Byte access *****/

static inline uint8_t
inb(uintptr_t pa)
{
	return *PA_TO_ADDR(vptr8_t, pa);
}

static inline void
insb(uintptr_t pa, void *v, size_t n)
{
	uint8_t *dst = v;

	while (n--) {
		*dst++ = *PA_TO_ADDR(vptr8_t, pa);
	}
}

static inline void
outb(uintptr_t pa, uint8_t v)
{
	*PA_TO_ADDR(vptr8_t, pa) = v;
}

static inline void
outsb(uintptr_t pa, const void *v, size_t n)
{
	const uint8_t *src = v;

	while (n--) {
		*PA_TO_ADDR(vptr8_t, pa) = *src++;
	}
}

/***** Word access *****/

static inline uint16_t
inw(uintptr_t pa)
{
	return le16toh(*PA_TO_ADDR(vptr16_t, pa));
}

static inline void
insw(uintptr_t pa, void *v, size_t n)
{
	uint16_t *dst = v;
	n >>= 1;

	while (n--) {
		/* "stream" are not swapped */
		*dst++ = *PA_TO_ADDR(vptr16_t, pa);
	}
}

static inline void
outw(uintptr_t pa, uint16_t v)
{
	*PA_TO_ADDR(vptr16_t, pa) = htole16(v);
}

static inline void
outsw(uintptr_t pa, const void *v, size_t n)
{
	const uint16_t *src = v;
	n >>= 1;

	while (n--) {
		/* "stream" are not swapped */
		*PA_TO_ADDR(vptr16_t, pa) = *src++;
	}
}

#endif /* pio_h_included */
