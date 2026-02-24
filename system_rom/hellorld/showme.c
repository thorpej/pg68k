/*
 * Copyright (c) 2026 Jason R. Thorpe.
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

#include "systypes.h"
#include "syslib.h"
#include "bootinfo.h"
#include "console.h"
#include "romcalls.h"

extern const struct romcalls_v1 *romcall_base;

#define	bootinfo_get_u32(bi)	(*(uint32_t *)bootinfo_dataptr(bi))

void
cons_putc(int ch)
{
	(*romcall_base->rv1_cnputc)(ch);
}

struct bi_record *
bootinfo_next(struct bi_record *bi)
{
	return (struct bi_record *)(((uintptr_t)bi) + bi->bi_size);
}

static char unkbuf[32];

static const char *
unk(uint32_t unk)
{
	snprintf(unkbuf, sizeof(unkbuf), "UNK-%u", unk);
	return unkbuf;
}

static const char *
unkx(uint32_t unk)
{
	snprintf(unkbuf, sizeof(unkbuf), "UNK-0x%08x", unk);
	return unkbuf;
}

typedef void (*prfunc_t)(struct bi_record *);

static void
print_machtype(struct bi_record *bi)
{
	static const char * const machtypes[] = {
	[0]			=	"UNK-0",
	[BI_MACH_AMIGA]		=	"Amiga",
	[BI_MACH_ATARI]		=	"Atari",
	[BI_MACH_MAC]		=	"Macintosh",
	[BI_MACH_APOLLO]	=	"Apollo",
	[BI_MACH_SUN3]		=	"Sun3",
	[BI_MACH_MVME147]	=	"MVME-147",
	[BI_MACH_MVME16x]	=	"MVME-16x",
	[BI_MACH_BVME6000]	=	"BVME6000",
	[BI_MACH_HP300]		=	"HP 9000/300",
	[BI_MACH_Q40]		=	"Q40",
	[BI_MACH_SUN3X]		=	"Sun3x",
	[BI_MACH_M54XX]		=	"MPC54xx",
	[BI_MACH_M5441X]	=	"MPC5441x",
	[BI_MACH_VIRT]		=	"Virt",
	};
	static const int nmachtypes = __arraycount(machtypes);
	uint32_t val = bootinfo_get_u32(bi);
	const char *cp = NULL;

	if (val == BI_MACH_FDT) {
		cp = "FDT Platform";
	} else if (val < nmachtypes) {
		cp = machtypes[val];
	}
	if (cp == NULL) {
		cp = unk(val);
	}

	printf("MACHTYPE = %s\n", cp);
}

static void
print_cputype(struct bi_record *bi)
{
	uint32_t val = bootinfo_get_u32(bi);
	const char *cp;

	switch (val) {
	case BI_CPU_68020:	cp = "68020";	break;
	case BI_CPU_68030:	cp = "68030";	break;
	case BI_CPU_68040:	cp = "68040";	break;
	case BI_CPU_68060:	cp = "68060";	break;
	case BI_CPU_COLDFIRE:	cp = "ColdFire"; break;
	case BI_CPU_68010:	cp = "68010";	break;
	default:		cp = unkx(val);	break;
	}

	printf("CPUTYPE = %s\n", cp);
}

static void
print_fputype(struct bi_record *bi)
{
	uint32_t val = bootinfo_get_u32(bi);
	const char *cp;

	switch (val) {
	case BI_FPU_68881:	cp = "68881";	break;
	case BI_FPU_68882:	cp = "68882";	break;
	case BI_FPU_68040:	cp = "68040";	break;
	case BI_FPU_68060:	cp = "68060";	break;
	case BI_FPU_SUNFPA:	cp = "SunFPA";	break;
	case BI_FPU_COLDFIRE:	cp = "ColdFire"; break;
	default:		cp = unkx(val);	break;
	}

	printf("FPUTYPE = %s\n", cp);
}

static void
print_mmutype(struct bi_record *bi)
{
	uint32_t val = bootinfo_get_u32(bi);
	const char *cp;

	switch (val) {
	case BI_MMU_68851:	cp = "68851";	break;
	case BI_MMU_68030:	cp = "68030";	break;
	case BI_MMU_68040:	cp = "68040";	break;
	case BI_MMU_68060:	cp = "68060";	break;
	case BI_MMU_APOLLO:	cp = "Apollo";	break;
	case BI_MMU_SUN3:	cp = "Sun3"; break;
	case BI_MMU_COLDFIRE:	cp = "ColdFire"; break;
	case BI_MMU_PG010:	cp = "PG010"; break;
	default:		cp = unkx(val);	break;
	}

	printf("MMUTYPE = %s\n", cp);
}

static void
print_memchunk(struct bi_record *bi)
{
	struct bi_mem_info *m = bootinfo_dataptr(bi);

	printf("MEMCHUNK: addr=0x%08x size=0x%08x\n", m->mem_addr, m->mem_size);
}

static void
print_ramdisk(struct bi_record *bi)
{
	struct bi_mem_info *m = bootinfo_dataptr(bi);

	printf("RAMDISK: addr=0x%08x size=0x%08x\n", m->mem_addr, m->mem_size);
}

static void
print_commandline(struct bi_record *bi)
{
	printf("COMMANDLINE: \"%s\"\n", bootinfo_dataptr(bi));
}

static void
print_rndseed(struct bi_record *bi)
{
	struct bi_data *d = bootinfo_dataptr(bi);

	printf("RND_SEED: %u bytes @ %p\n", d->data_length, &d->data_bytes[0]);
}

static void
print_fdt_platform(struct bi_record *bi)
{
	printf("PLATFORM: \"%s\"\n", bootinfo_dataptr(bi));
}

static void
print_fdt_blob(struct bi_record *bi)
{
	struct bi_data *d = bootinfo_dataptr(bi);

	printf("FDT BLOB: %u bytes @ %p\n", d->data_length, &d->data_bytes[0]);
}

static void
print_fdt_elf_syms(struct bi_record *bi)
{
	struct bi_mem_info *m = bootinfo_dataptr(bi);

	printf("ELF SYMS: addr=0x%08x size=0x%08x\n", m->mem_addr, m->mem_size);
}

static const prfunc_t prfuncs[] = {
[BI_MACHTYPE]		=	print_machtype,
[BI_CPUTYPE]		=	print_cputype,
[BI_FPUTYPE]		=	print_fputype,
[BI_MMUTYPE]		=	print_mmutype,
[BI_MEMCHUNK]		=	print_memchunk,
[BI_RAMDISK]		=	print_ramdisk,
[BI_COMMAND_LINE]	=	print_commandline,
[BI_RNG_SEED]		=	print_rndseed,
};
static const u_int nprfuncs = __arraycount(prfuncs);

#define	I(x)	((x) - BI_MACHDEP(0))

static const prfunc_t mdprfuncs[] = {
[I(BI_FDT_PLATFORM)]	=	print_fdt_platform,
[I(BI_FDT_BLOB)]	=	print_fdt_blob,
[I(BI_FDT_ELF_SYMS)]	=	print_fdt_elf_syms,
};
static const u_int nmdprfuncs = __arraycount(mdprfuncs);

#undef I

static void
dump_bi(struct bi_record *bi)
{
	prfunc_t prf;

	for (; bi->bi_tag != BI_LAST; bi = bootinfo_next(bi)) {
		prf = NULL;
		if (bi->bi_tag < nprfuncs) {
			prf = prfuncs[bi->bi_tag];
		} else if (bi->bi_tag >= BI_MACHDEP(0) &&
			   bi->bi_tag <  (BI_MACHDEP(0) + nmdprfuncs)) {
			prf = mdprfuncs[bi->bi_tag - BI_MACHDEP(0)];
		}
		if (prf != NULL) {
			(*prf)(bi);
		} else {
			printf("UNK-<%u> size %u\n", bi->bi_tag, bi->bi_size);
		}
	}
}

void
showme(void *vbi)
{
	dump_bi(vbi);
}
