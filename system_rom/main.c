/*
 * Copyright (c) 2025, 2026 Jason R. Thorpe.
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
#include "clock.h"
#include "intr.h"
#include "ata.h"
#include "uart.h"
#include "loadfile.h"	/* for load flags passed to exec() */
#include "ls.h"
#include "cli.h"

#include "memory.h"

#ifdef CONFIG_MC68010
#include "control.h"
#include "cpu010_mmu.h"

static uint8_t
mmu_getcontext(void)
{
	return control_inb(MMUREG_CONTEXT);
}

static uint8_t
mmu_setcontext(uint8_t val)
{
	control_outb(MMUREG_CONTEXT, val);
}

static uint16_t
mmu_sme_get(unsigned long segva)
{
	return control_inw(MMUREG_SEGMAP_ENTRY(PGMMU_SEGNUM(segva)));
}

static void
mmu_sme_set(unsigned long segva, uint16_t val)
{
	control_outw(MMUREG_SEGMAP_ENTRY(PGMMU_SEGNUM(segva)), val);
}

static uint32_t
mmu_pme_get(unsigned long pme_index)
{
	return control_inl(MMUREG_PAGEMAP_ENTRY(pme_index));
}

static uint32_t
mmu_pme_set(unsigned long pme_index, uint32_t val)
{
	control_outl(MMUREG_PAGEMAP_ENTRY(pme_index), val);
}
#define	CONFIG_MMU_COMMAND
#endif /* CONFIG_MC68010 */

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
			/*
			 * This is a fixed-size region.  In this case,
			 * if maxsize == 0, we silently skip reporting.
			 */
			if (mb->maxsize == 0) {
				continue;
			}
		} else if (mb->maxsize == 0) {
			/* not supported on this machine; skip */
			continue;
		} else {
			size_memory_bank(mb);
			if (mb->size == 0) {
				/* no memory in this bank; skip */
				continue;
			}
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
	intr_init();
	clock_configure();
#ifdef UART0_ADDR
	uart_configure();
#endif
#ifdef ATA_ADDR
	ata_configure();
#endif
}

void
quiesce(void)
{
	clock_quiesce();
	intr_fini();
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
[ETIMEDOUT]	=	"Operation timed out",
[EHOSTUNREACH]	=	"No route to host",
[EBADRPC]	=	"RPC struct is bad",
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
		partition_list_show(&f->f_blk.f_partitions);
	} else {
		printf("%s is not a block device.\n",
		    dev_string(f, dstr, sizeof(dstr)));
	}
	close(fd);
}

static void
cli_u_uptime(const char *str)
{
	printf("usage: %s\n", str);
}

static void
cli_h_uptime(int argc, char *argv[])
{
	time_t secs = clock_getsecs();

	printf("uptime: %lld second%s\n", (long long)secs, plural((int)secs));
}

static void
cli_u_reboot(const char *str)
{
	printf("usage: %s\n", str);
}

static void
cli_h_reboot(int argc, char *argv[])
{
	reboot();
}

static void version(void);

static void
cli_u_version(const char *str)
{
	printf("usage: %s\n", str);
}

static void
cli_h_version(int argc, char *argv[])
{
	version();
}

static void
cli_u_mem(const char *str)
{
	printf("usage: mem <addr>\n");
}

static void
cli_h_mem(int argc, char *argv[])
{
	/* XXX test only */
	printf("oink! 0x%02x\n", *(char *)(10 * 1024 * 1024));
}

static void
cli_u_probe(const char *str)
{
	printf("usage: probe <addr>\n");
}

static void
cli_h_probe(int argc, char *argv[])
{
	/* XXX test only */
	uint32_t val;
	uint32_t *addr = (void *)(OBIO_VIRT + 0x0f00);

	if (badaddr_read32(addr, &val)) {
		printf("Nothing at 0x%08lx\n", (u_long)addr);
		return;
	}
	printf("0x%08lx: 0x%08x\n", (u_long)addr, val);
}

#ifdef CONFIG_MMU_COMMAND
static void
cli_u_mmu(const char *str)
{
	printf("usage: %s show context\n", str);
	printf("       %s set context <0...%d>\n", str, PGMMU_NUM_CONTEXTS - 1);
	printf("       %s show sme <va>\n", str);
	printf("       %s show pmeg <0...%d>\n", str, PGMMU_NUM_PMEGS - 1);
}

static bool
parse_va(const char *str, unsigned long *va_out)
{
	int base = 0;
	unsigned long va;
	char *endptr;

	if (str[0] == '$') {
		str++;
		base = 16;
	}

	va = strtoul(str, &endptr, base);
	if (*endptr != '\0') {
		return false;
	}
#ifdef CONFIG_MC68010
	if (va > 0x00ffffff) {
		return false;
	}
#endif /* CONFIG_MC68010 */
	*va_out = va;
	return true;
}

static bool
parse_pmeg(const char *str, unsigned int *pmeg_out)
{
	int base = 0;
	unsigned long pmeg;
	char *endptr;

	pmeg = strtoul(str, &endptr, base);
	if (*endptr != '\0') {
		return false;
	}
	if (pmeg >= PGMMU_NUM_PMEGS) {
		return false;
	}
	*pmeg_out = (unsigned int)pmeg;
	return true;
}

static bool
parse_context(const char *str, uint8_t *context_out)
{
	int base = 0;
	unsigned long context;
	char *endptr;

	context = strtoul(str, &endptr, base);
	if (*endptr != '\0') {
		return false;
	}
	if (context >= PGMMU_NUM_CONTEXTS) {
		return false;
	}
	*context_out = (uint8_t)context;
	return true;
}

static const char *
indent_string(unsigned int level)
{
	static const char zee_tabs[] = "\t\t\t\t\t\t\t";
	return &zee_tabs[sizeof(zee_tabs) - (level + 1)];
}

static void
print_sme(unsigned int indent_level, unsigned long va, uint16_t sme)
{
	printf("%sSME<0x%08lx>: ", indent_string(indent_level), va);
	if (sme & SME_V) {
		printf("PMEG %d (0x%04x)\n",
		    sme & SME_PMEG, sme & SME_PMEG);
	} else {
		printf("invalid (0x%04x)\n", sme);
	}
}

static void
print_pme(unsigned int indent_level, unsigned int pme_index, uint32_t pme)
{
	printf("%sPME<%d>: ", indent_string(indent_level), pme_index);
	if (pme & PME_V) {
		printf("%s %s %s %s %s %s %s %s 0x%04x (phys=0x%08x)\n",
		    (pme & PME_W)   ? "w"   : " ",
		    (pme & PME_K)   ? "k"   : " ",
		    (pme & PME_REF) ? "r"   : " ",
		    (pme & PME_MOD) ? "m"   : " ",
		    (pme & PME_SW3) ? "sw3" : "   ",
		    (pme & PME_SW2) ? "sw2" : "   ",
		    (pme & PME_SW1) ? "sw1" : "   ",
		    (pme & PME_SW0) ? "sw0" : "   ",
		    (pme & PME_PFN),
		    (pme & PME_PFN) << PAGE_SHIFT);
	} else {
		printf("invalid (0x%08x)\n", pme);
	}
}

static void
print_pmeg(unsigned int indent_level, unsigned int pmeg)
{
	unsigned int first_pme = pmeg * PGMMU_PMES_PER_PMEG;
	unsigned int limit_pme = first_pme + PGMMU_PMES_PER_PMEG;
	unsigned int pme_index;

	printf("%sPMEG %d (0x%04x):\n", indent_string(indent_level),
	    pmeg, pmeg);
	for (pme_index = first_pme; pme_index < limit_pme; pme_index++) {
		print_pme(indent_level + 1, pme_index % PGMMU_PMES_PER_PMEG,
		    mmu_pme_get(pme_index));
	}
}

static void
cli_h_mmu(int argc, char *argv[])
{
	unsigned long va;
	unsigned int pmeg;
	uint8_t context;

	if (argc < 2) {
		goto usage;
	}

	if (strcmp(argv[1], "show") == 0) {
		if (argc < 3) {
			goto usage;
		}
		
		if (strcmp(argv[2], "context") == 0) {
			printf("Context: %d\n", mmu_getcontext());
			return;
		}

		if (strcmp(argv[2], "sme") == 0) {
			if (argc < 4) {
				goto usage;
			}
			if (! parse_va(argv[3], &va)) {
				printf("Bad virtual address: %s\n",
				    argv[3]);
				return;
			}
			va &= ~PGMMU_SEG_OFFSET;
			print_sme(0, va, mmu_sme_get(va));
			return;
		}

		if (strcmp(argv[2], "pmeg") == 0) {
			if (argc < 4) {
				goto usage;
			}
			if (! parse_pmeg(argv[3], &pmeg)) {
				printf("Bad PMEG: %s\n",
				    argv[3]);
				return;
			}
			print_pmeg(0, pmeg);
			return;
		}

		goto usage;
	}

	if (strcmp(argv[1], "set") == 0) {
		if (argc < 4) {
			goto usage;
		}

		if (strcmp(argv[2], "context") == 0) {
			if (! parse_context(argv[3], &context)) {
				printf("Bad Context: %s\n",
				    argv[3]);
				return;
			}
			mmu_setcontext(context);
			printf("Context: %d\n", mmu_getcontext());
			return;
		}

		goto usage;
	}

	/* FALLTHROUGH */

 usage:
	cli_u_mmu(argv[0]);
}
#endif /* CONFIG_MMU_COMMAND */

static const struct cli_handler {
	const char	*h_str;
	const char	*h_desc;
	void		(*h_func)(int, char *[]);
	void		(*h_usage)(const char *);
} cli_handlers[] = {
	{ "?",
	  NULL,
	  cli_h_help,
	  cli_u_help,
	},
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
	{ "uptime",
	  "get number of seconds since boot",
	  cli_h_uptime,
	  cli_u_uptime,
	},
	{ "reboot",
	  "reboot the system",
	  cli_h_reboot,
	  cli_u_reboot,
	},
	{ "version",
	  "print the system and firmware versions",
	  cli_h_version,
	  cli_u_version,
	},
	{ "mem",
	  "examine / modify memory",
	  cli_h_mem,
	  cli_u_mem,
	},
	{ "probe",
	  "probe a memory location",
	  cli_h_probe,
	  cli_u_probe,
	},
#ifdef CONFIG_MMU_COMMAND
	{ "mmu",
	  "interact with the MMU",
	  cli_h_mmu,
	  cli_u_mmu,
	},
#endif /* CONFIG_MMU_COMMAND */
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
			(*h->h_usage)(argv[1]);
			break;
		}
		/* FALLTHROUGH */
	case 1:
		for (int i = 0; i < arraycount(cli_handlers); i++) {
			if (cli_handlers[i].h_desc == NULL) {
				continue;
			}
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
	reboot();
}

static void
version(void)
{
	/* Hello, world! */
	printf("%s", CONFIG_MACHINE_STRING);
#ifdef CONFIG_CPU_DESC_STRING
	printf(" - %s", CONFIG_CPU_DESC_STRING);
#endif
	printf("\n");
	printf("Firmware version %d.%d\n\n", CONFIG_ROM_VERSION_MAJOR,
	    CONFIG_ROM_VERSION_MINOR);
}

int
main(int argc, char *argv[])
{
	/* First step - initialize console so we can see messages. */
	cons_init();

	/* Hello, world! */
	version();

	/* Configure / probe the hardware. */
	configure();

	printf("\n");

	cli_loop();
}

void
reboot(void)
{
#if defined(CONFIG_REBOOT_VECTAB)
	/*
	 * This implementation reboots the system by jumping to the
	 * reset vector.  The start-up code deals with all the rest.
	 */
	extern unsigned long vectab[];
	void (*reset_vec)(void) = (void *)vectab[1];
	quiesce();
	(*reset_vec)();
#else
	printf("HOW CAN I HAZ REBOOT?!?\n");
#endif
}

void
panic(const char *fmt, ...)
{
	/* XXX */
}
