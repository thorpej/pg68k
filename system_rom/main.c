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

#include "console.h"
#include "trap.h"
#include "uart.h"

struct memory_bank {
	uintptr_t	start;
	size_t		size;
	size_t		maxsize;
	const char *	desc;
};

static struct memory_bank memory_banks[] = {
#ifdef RAM0_START
	{
		.start   = RAM0_START,
		.size    = RAM0_SIZE,
		.maxsize = RAM0_MAXSIZE,
		.desc    = RAM0_DESC,
	},
#endif
#ifdef RAM1_START
	{
		.start   = RAM1_START,
		.size    = RAM1_SIZE,
		.maxsize = RAM1_MAXSIZE,
		.desc    = RAM1_DESC,
	},
#endif
#ifdef RAM2_START
	{
		.start   = RAM2_START,
		.size    = RAM2_SIZE,
		.maxsize = RAM2_MAXSIZE,
		.desc    = RAM2_DESC,
	},
#endif
#ifdef RAM3_START
	{
		.start   = RAM3_START,
		.size    = RAM3_SIZE,
		.maxsize = RAM3_MAXSIZE,
		.desc    = RAM3_DESC,
	},
#endif
#ifdef RAM4_START
	{
		.start   = RAM4_START,
		.size    = RAM4_SIZE,
		.maxsize = RAM4_MAXSIZE,
		.desc    = RAM4_DESC,
	},
#endif
};

static void
size_memory_bank(struct memory_bank *mb)
{
	/* XXX assume 1MB chunks for now. */
	uint32_t *p, val;
	size_t chunksize = 1024 * 1024;
	size_t maxchunks = mb->maxsize / chunksize;
	size_t chunk, i;

	for (chunk = 0; chunk < maxchunks; chunk++) {
		p = (uint32_t *)(mb->start + (chunk * chunksize));
		if (badaddr_write32(p, chunk)) {
			/* Bus error writing this chunk. */
			break;
		}
		/* Validate the write. */
		for (i = 0; i < chunk; i++) {
			p = (uint32_t *)(mb->start + (i * chunksize));
			if (badaddr_read32(p, &val)) {
				/* Bus error reading chunk. */
				break;
			}
			if (val != i) {
				/* Write wrapped. */
				break;
			}
		}
		if (i != chunk) {
			break;
		}
	}

	mb->size = chunk * chunksize;
}

static void
size_memory(void)
{
	struct memory_bank *mb;
	u_long psize;
	int i;
	char mod;

	for (i = 0; i < arraycount(memory_banks); i++) {
		mb = &memory_banks[i];
		if (mb->size != 0) {
			/* fixed size region; skip */
			continue;
		}
		if (mb->maxsize == 0) {
			/* not supported on this machine; skip */
			continue;
		}
		size_memory_bank(mb);
		if (mb->size == 0) {
			/* no memory in this bank; skip */
			continue;
		}

		psize = mb->size / 1024;
		mod = 'K';

		if (psize > 1024) {
			mod = 'M';
			psize = psize / 1024;
		}

		printf("%lu%cB %s @ 0x%08lx\n",
		    psize, mod, mb->desc, (u_long)mb->start);
	}
}

static void
configure(void)
{
	printf("Memory configuration:\n");
	size_memory();
	printf("\n");

	printf("Device configuration:\n");
#ifdef UART0_ADDR
	uart_configure();
#endif
}

#ifndef CONFIG_MACH_HOST_SIM
int	errno;

static const char *errno_strings[] = {
[ENOENT]	=	"No such file or directory",
[EIO]		=	"Input/output error",
[ENXIO]		=	"Device not configured",
[ENOEXEC]	=	"Exec format error",
[EBADF]		=	"Bad file descriptor",
[ENOMEM]	=	"Cannot allocate memory",
[EBUSY]		=	"Device busy",
[EEXIST]	=	"File exists",
[ENODEV]	=	"Operation not supported by device",
[ENOTDIR]	=	"Not a directory",
[EINVAL]	=	"Invalid argument",
[EMFILE]	=	"Too many open files",
[ENOSPC]	=	"No space left on device",
[EROFS]		=	"Read-only file system",
[EOPNOTSUPP]	=	"Operation not supported",
[ELOOP]		=	"Too many levels of symbolic links",
[ENAMETOOLONG]	=	"File name too long",
};

const char *
strerror(int err)
{
	static char unkerr[sizeof("unknown error XXXXXXXXXXXX")];

	if (err < 0 || err > arraycount(errno_strings) ||
	    errno_strings[err] == NULL) {
		snprintf(unkerr, sizeof(unkerr), "unknown error %d", err);
		return unkerr;
	}
	return errno_strings[err];
}
#endif

int
main(int argc, char *argv[])
{
	/* First step - initialize console so we can see messages. */
	cons_init();

	/* Hello, world! */
	printf("%s\n", CONFIG_MACHINE_STRING);
	printf("Firmware version %d.%d\n\n", CONFIG_ROM_VERSION_MAJOR,
	    CONFIG_ROM_VERSION_MINOR);

	/* Configure / probe the hardware. */
	configure();

	/* A stub, obviously. */
	return 0;
}

void
panic(const char *fmt, ...)
{
	/* XXX */
}
