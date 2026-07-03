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

#define	PIO_ADDR(t, a)	((t)(a))

/***** Byte access *****/

static inline uint8_t
inb(uintptr_t addr)
{
	return *PIO_ADDR(vptr8_t, addr);
}

static inline void
insb(uintptr_t addr, void *v, size_t n)
{
	uint8_t *dst = v;

	while (n--) {
		*dst++ = *PIO_ADDR(vptr8_t, addr);
	}
}

static inline void
outb(uintptr_t addr, uint8_t v)
{
	*PIO_ADDR(vptr8_t, addr) = v;
}

static inline void
outsb(uintptr_t addr, const void *v, size_t n)
{
	const uint8_t *src = v;

	while (n--) {
		*PIO_ADDR(vptr8_t, addr) = *src++;
	}
}

/***** Word access *****/

static inline uint16_t
inw(uintptr_t addr)
{
	return le16toh(*PIO_ADDR(vptr16_t, addr));
}

static inline void
insw(uintptr_t addr, void *v, size_t n)
{
	uint16_t *dst = v;
	n >>= 1;

	while (n--) {
		/* "stream" are not swapped */
		*dst++ = *PIO_ADDR(vptr16_t, addr);
	}
}

static inline void
outw(uintptr_t addr, uint16_t v)
{
	*PIO_ADDR(vptr16_t, addr) = htole16(v);
}

static inline void
outsw(uintptr_t addr, const void *v, size_t n)
{
	const uint16_t *src = v;
	n >>= 1;

	while (n--) {
		/* "stream" are not swapped */
		*PIO_ADDR(vptr16_t, addr) = *src++;
	}
}

#endif /* pio_h_included */
