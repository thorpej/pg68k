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

#include "config.h"
#include "syslib.h"

#include "trap.h"
#include "simglue.h"

jmp_buf nofault_env;
bool nofault;

#undef RAM0_SIZE
#define	RAM0_SIZE	(8*1024*1024)
#define	RAM0_MASK	(RAM0_SIZE - 1)

#define	RAM0_OFF(x)	(((x) - RAM0_START) & RAM0_MASK)

static uint8_t ram0[RAM0_SIZE];

bool
badaddr_read32(volatile uint32_t *p, uint32_t *valp)
{
	uintptr_t x = (uintptr_t)p;

	if (x >= RAM0_START && x < RAM0_START+RAM0_MAXSIZE) {
		memcpy(valp, &ram0[RAM0_OFF(x)], sizeof(*valp));
		return false;
	}

	return true;
}

bool
badaddr_write32(volatile uint32_t *p, uint32_t val)
{
	uintptr_t x = (uintptr_t)p;

	if (x >= RAM0_START && x < RAM0_START+RAM0_MAXSIZE) {
		memcpy(&ram0[RAM0_OFF(x)], &val, sizeof(val));
		return false;
	}

	return true;
}

ssize_t sim_loader_read(int, uintptr_t, size_t);
ssize_t
sim_loader_read(int fd, uintptr_t dst, size_t sz)
{
	return sz;
}

void sim_loader_bcopy(const void *, uintptr_t, size_t);
void
sim_loader_bcopy(const void *src, uintptr_t dst, size_t sz)
{
}

void sim_loader_bzero(uintptr_t, size_t);
void
sim_loader_bzero(uintptr_t dst, size_t sz)
{
}
