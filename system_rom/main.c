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
#include "sysfile.h"

#include "console.h"
#include "trap.h"
#include "ata.h"
#include "uart.h"
#include "loadfile.h"	/* for load flags passed to exec() */
#include "ls.h"
#include "cli.h"

#include "memory.h"

struct memory_bank memory_banks[] = {
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
const int memory_bank_count = arraycount(memory_banks);

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
#ifdef ATA_ADDR
	ata_configure();
#endif
}

#ifndef CONFIG_MACH_HOST_SIM
int	errno;

static const char *errno_strings[] = {
[ENOENT]	=	"No such file or directory",
[ESRCH]		=	"No such process",
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
[EFBIG]		=	"File too large",
[ENOSPC]	=	"No space left on device",
[EROFS]		=	"Read-only file system",
[EMLINK]	=	"Too many links",
[EOPNOTSUPP]	=	"Operation not supported",
[ELOOP]		=	"Too many levels of symbolic links",
[ENAMETOOLONG]	=	"File name too long",
[EFTYPE]	=	"Inappropriate file type or format",
[ERANGE]	=	"Result too large or too small",
};

char *
strerror(int err)
{
	static char unkerr[sizeof("unknown error XXXXXXXXXXXX")];

	if (err < 0 || err > arraycount(errno_strings) ||
	    errno_strings[err] == NULL) {
		snprintf(unkerr, sizeof(unkerr), "unknown error %d", err);
		return unkerr;
	}
	return UNCONST(errno_strings[err]);
}
#endif

static char cli_cmdline[256];
static size_t cli_cmdline_idx;
#define	CLI_CMDLINE_LIMIT	(sizeof(cli_cmdline) - 1)

#define	MAX_CL_ARGS	64

static char *cli_argv[MAX_CL_ARGS];
static int cli_argc;

jmp_buf cli_env;
static bool cli_env_valid;

static void
cli_get_cmdline(void)
{
	int ch;

	printf(">>> ");

	for (cli_cmdline_idx = 0;;) {
		ch = cons_getc();
		switch (ch) {
		case 3: /* ETX - ^C */
			putstrn("^C\n", 3);
			cli_longjmp();
			break;

#ifdef CONFIG_MACH_HOST_SIM
		case 4: /* EOT - ^D */
			putstrn("^D\n", 3);
			exit(0);
			break;
#endif

		case 7: /* BEL - ^G */
			cons_putc(ch);	/* beep! */
			break;

		case '\r':
			cli_cmdline[cli_cmdline_idx++] = '\0';
			cons_putc('\n');
			return;

		case '\b':
		case 127: /* DEL */
			if (cli_cmdline_idx != 0) {
				putstrn("\b \b", 3);
				cli_cmdline_idx--;
			}
			break;

		case 21: /* NAK - ^U */
			while (cli_cmdline_idx != 0) {
				putstrn("\b \b", 3);
				cli_cmdline_idx--;
			}
			break;

		case '\t':
			goto accept;

		default:
			/* Ignore other control characters. */
			if (ch < 32) {
				continue;
			}
		accept:
			if (cli_cmdline_idx < CLI_CMDLINE_LIMIT) {
				cons_putc(ch);
				cli_cmdline[cli_cmdline_idx++] = ch;
			}
		}
	}
}

static inline bool
cli_whitespace_p(int ch)
{
	return (ch == ' ' || ch == '\t');
}

static char *
cli_skip_whitespace(char *cp)
{
	while (cli_whitespace_p(*cp)) {
		cp++;
	}
	return cp;
}

static void
cli_get_argv(void)
{
	char *cp = cli_cmdline;

	for (cli_argc = 0;;) {
		cp = cli_skip_whitespace(cp);
		if (*cp == '\0') {
			return;
		}
		if (cli_argc == MAX_CL_ARGS) {
			printf("Argument list too long.\n");
			cli_longjmp();
		}
		cli_argv[cli_argc++] = cp;
		while (! cli_whitespace_p(*cp) && *cp != '\0') {
			cp++;
		}
		if (cli_whitespace_p(*cp)) {
			*cp++ = '\0';
		}
	}
}

static void	cli_h_help(int, char *[]);
static void	cli_u_help(const char *);

static void
cli_u_ls(const char *str)
{
	printf("usage: %s path\n", str);
	printf("example: %s ata()/\n", str);
}

static void
cli_h_ls(int argc, char *argv[])
{
	if (argc != 2) {
		cli_u_ls(argv[0]);
		return;
	}
	ls(argv[1]);
}

static void
cli_u_boot(const char *str)
{
	printf("usage: %s file [args]\n", str);
}

static void
cli_h_boot(int argc, char *argv[])
{
	if (argc < 2) {
		cli_u_boot(argv[0]);
		return;
	}

	(void) exec(LOAD_ALL, argc, argv);
}

static void
cli_u_part(const char *str)
{
	printf("usage: %s disk\n", str);
	printf("example: %s ata(0,0)\n", str);
}

static void
cli_h_part(int argc, char *argv[])
{
	char dstr[DEV_STRING_SIZE];

	if (argc != 2) {
		cli_u_part(argv[0]);
		return;
	}

	int fd = open(argv[1], O_RDONLY | O_RAW | O_WHOLE);
	if (fd < 0) {
		printf("%s: %s\n", argv[1], strerror(errno));
		return;
	}
	struct open_file *f = getfile(fd);

	if (DEV_IS_BLKDEV(f->f_dev)) {
		printf("Partitions for %s:\n",
		    dev_string(f, dstr, sizeof(dstr)));
		partition_list_show(&f->f_blkdev.f_partitions);
	} else {
		printf("%s is not a block device.\n",
		    dev_string(f, dstr, sizeof(dstr)));
	}
	close(fd);
}

static const struct cli_handler {
	const char	*h_str;
	const char	*h_desc;
	void		(*h_func)(int, char *[]);
	void		(*h_usage)(const char *);
} cli_handlers[] = {
	{ "help",
	  "get help about a command",
	  cli_h_help,
	  cli_u_help,
	},
	{ "boot",
	  "boot an executable file",
	  cli_h_boot,
	  cli_u_boot,
	},
	{ "ls",
	  "show listing of a directory",
	  cli_h_ls,
	  cli_u_ls,
	},
	{ "part",
	  "show disk partitions",
	  cli_h_part,
	  cli_u_part,
	},
};

static const struct cli_handler *
cli_handler_lookup(const char *str)
{
	for (int i = 0; i < arraycount(cli_handlers); i++) {
		if (strcmp(cli_handlers[i].h_str, str) == 0) {
			return &cli_handlers[i];
		}
	}
	return NULL;
}

static void
cli_u_help(const char *str)
{
	printf("usage: %s [command]\n", str);
}

static void
cli_h_help(int argc, char *argv[])
{
	const struct cli_handler *h;

	switch (argc) {
	case 2:
		h = cli_handler_lookup(argv[1]);
		if (h != NULL) {
			(*h->h_usage)(argv[0]);
			break;
		}
		/* FALLTHROUGH */
	case 1:
		for (int i = 0; i < arraycount(cli_handlers); i++) {
			printf("%-10s %s\n", cli_handlers[i].h_str,
			    cli_handlers[i].h_desc);
		}
		break;

	default:
		cli_u_help(argv[0]);
	}
}

static void
cli_dispatch(void)
{
	if (cli_argc == 0) {
		return;
	}

	const struct cli_handler *h = cli_handler_lookup(cli_argv[0]);

	if (h != NULL) {
		(*h->h_func)(cli_argc, cli_argv);
		return;
	}
	printf("Unknown command: %s\n", cli_argv[0]);
}

static void
cli_loop(void)
{
	setjmp(cli_env);
	cli_env_valid = true;

	for (;;) {
		cli_get_cmdline();
		cli_get_argv();
		cli_dispatch();
	}
}

void
cli_longjmp(void)
{
	if (cli_env_valid) {
		longjmp(cli_env, 1);
	}
	printf("XXX cli_longjmp XXX\n");
	for (;;) {
		/* forever */
	}
}

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

	printf("\n");

	cli_loop();
}

void
panic(const char *fmt, ...)
{
	/* XXX */
}
