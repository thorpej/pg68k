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

#include "config.h"
#include "syslib.h"
#include "memory.h"
#include "bootinfo.h"

#ifdef CONFIG_DEVICETREE
#include "libfdt.h"
#endif

static struct bi_record *
_bootinfo_set_size(struct bi_record *bi, uint16_t tag, size_t sz, bool wr)
{
	uintptr_t addr = (uintptr_t)bi;
	uintptr_t next_addr = roundup(addr + sizeof(*bi) + sz, BOOTINFO_ROUND);

	if (wr) {
		bi->bi_tag = tag;
		bi->bi_size = next_addr - addr;
	}
	return (struct bi_record *)next_addr;
}

static struct bi_record *
_bootinfo_set_u32(struct bi_record *bi, uint16_t tag, uint32_t val, bool wr)
{
	if (wr) {
		*(uint32_t *)bootinfo_dataptr(bi) = val;
	}
	return _bootinfo_set_size(bi, tag, sizeof(val), wr);
}

struct bi_record *
bootinfo_set_u32(struct bi_record *bi, uint16_t tag, uint32_t val)
{
	return _bootinfo_set_u32(bi, tag, val, true);
}

static struct bi_record *
_bootinfo_set_mem_info(struct bi_record *bi, uint16_t tag,
    uint32_t addr, uint32_t size, bool wr)
{
	struct bi_mem_info *m = bootinfo_dataptr(bi);

	if (wr) {
		m->mem_addr = addr;
		m->mem_size = size;
	}

	return _bootinfo_set_size(bi, tag, sizeof(*m), wr);
}

struct bi_record *
bootinfo_set_mem_info(struct bi_record *bi, uint16_t tag,
    uint32_t addr, uint32_t size)
{
	return _bootinfo_set_mem_info(bi, tag, addr, size, true);
}

static struct bi_record *
_bootinfo_set_data(struct bi_record *bi, uint16_t tag,
    const void *data, size_t len, bool wr)
{
	struct bi_data *d = bootinfo_dataptr(bi);

	if (wr) {
		d->data_length = (uint16_t)len;
		memcpy(&d->data_bytes[0], data, d->data_length);
	}

	return _bootinfo_set_size(bi, tag, sizeof(*d) + (uint16_t)len, wr);
}

struct bi_record *
bootinfo_set_data(struct bi_record *bi, uint16_t tag,
    const void *data, size_t len)
{
	return _bootinfo_set_data(bi, tag, data, len, true);
}

static struct bi_record *
_bootinfo_set_string(struct bi_record *bi, uint16_t tag,
    const char *str, bool wr)
{
	if (wr) {
		strcpy(bootinfo_dataptr(bi), str);
	}
	return _bootinfo_set_size(bi, tag, strlen(str) + 1, wr);
}

struct bi_record *
bootinfo_set_string(struct bi_record *bi, uint16_t tag,
    const char *str)
{
	return _bootinfo_set_string(bi, tag, str, true);
}

#if defined(CONFIG_MC68010)
/* No BI_CPU_value */
#elif defined(CONFIG_MC68020)
#define	BI_CPU_value	BI_CPU_68020
#elif defined(CONFIG_MC68030)
#define	BI_CPU_value	BI_CPU_68030
#elif defined(CONFIG_MC68040)
#define	BI_CPU_value	BI_CPU_68040
#elif defined(CONFIG_MC68060)
#define	BI_CPU_value	BI_CPU_68060
#endif

#if defined(CONFIG_MC68881)
#define	BI_FPU_value	BI_FPU_68881
#elif defined(CONFIG_MC68882)
#define	BI_FPU_value	BI_FPU_68882
#elif defined(CONFIG_MC68040) && !defined(CONFIG_MC68LC040)
#define	BI_FPU_value	BI_FPU_68040
#elif defined(CONFIG_MC68060) && !defined(CONFIG_MC68LC060)
#define	BI_FPU_value	BI_FPU_68060
#endif

#if defined(CONFIG_MC68010)
/* No BI_MMU_value */
#elif defined(CONFIG_MC68020)
#error need a 68020 case
#elif defined(CONFIG_MC68030)
#define	BI_MMU_value	BI_MMU_68030
#elif defined(CONFIG_MC68040)
#define	BI_MMU_value	BI_MMU_68040
#elif defined(CONFIG_MC68060)
#define	BI_MMU_value	BI_MMU_68060
#endif

#define	MAX_MEM_RECORDS	8
static struct bi_mem_info mem_records[MAX_MEM_RECORDS];
static unsigned int num_mem_records;

static void
_bootinfo_set_mem_record(uintptr_t start, size_t size)
{
	int idx = num_mem_records;

	/* Check to see if we're setting the first record. */
	if (num_mem_records == 0) {
		goto add_record;
	}

	/* Check to see if we can merge with the previous record. */
	int prev_idx = idx - 1;
	if (start ==
	    mem_records[prev_idx].mem_addr + mem_records[prev_idx].mem_size) {
		mem_records[prev_idx].mem_size += size;
		return;
	}

	/* Must add a record; is there room? */
	if (idx == MAX_MEM_RECORDS) {
		return;
	}

 add_record:
	/* "Hey, we've got another record over here!" */
	num_mem_records++;
	mem_records[idx].mem_addr = start;
	mem_records[idx].mem_size = size;
}

void
bootinfo_set_mem_records(void)
{
	uintptr_t start;
	size_t size;
	int i;

	for (i = 0; i < memory_bank_count; i++) {
		if (memory_banks[i].size == 0) {
			continue;
		}
		start = memory_banks[i].start;
		size = memory_banks[i].size;

		if (RESV_RAM_START >= start &&
		    RESV_RAM_SIZE < (start + size)) {
			uintptr_t a0, a1, a2, a3;

			a0 = start;
			a1 = RESV_RAM_START;
			a2 = (RESV_RAM_START + RESV_RAM_SIZE);
			a3 = (start + size);

			if (a1 - a0) {
				_bootinfo_set_mem_record(a0, a1 - a0);
			}
			if (a3 - a2) {
				_bootinfo_set_mem_record(a2, a3 - a2);
			}
		} else {
			_bootinfo_set_mem_record(start, size);
		}
	}
}

struct bi_mem_info *
bootinfo_get_mem_record(struct bi_mem_info *bi)
{
	if (bi == NULL) {
		return mem_records;
	}

	uintptr_t idx = bi - mem_records;
	if (idx + 1 < num_mem_records) {
		return bi + 1;
	}

	return NULL;
}

static struct bi_record *
_bootinfo_populate(struct bi_record *bi, const char *bargs, bool wr)
{
	int i;

	bi = _bootinfo_set_u32(bi, BI_MACHTYPE, BI_MACH_FDT, wr);
#ifdef B_CPU_value
	bi = _bootinfo_set_u32(bi, BI_CPUTYPE, BI_CPU_value, wr);
#endif
#ifdef BI_FPU_value
	/* XXX should also probe for FPU type. */
	bi = _bootinfo_set_u32(bi, BI_FPUTYPE, BI_FPU_value, wr);
#endif
#ifdef BI_MMU_value
	bi = _bootinfo_set_u32(bi, BI_MMUTYPE, BI_MMU_value, wr);
#endif

	for (i = 0; i < num_mem_records; i++) {
		bi = _bootinfo_set_mem_info(bi, BI_MEMCHUNK,
		    mem_records[i].mem_addr, mem_records[i].mem_size, wr);
	}

	if (bargs != NULL && strlen(bargs) != 0) {
		bi = _bootinfo_set_string(bi, BI_COMMAND_LINE, bargs, wr);
	}

#ifdef CONFIG_DEVICETREE
	void *fdt = get_fdt();
	int off = fdt_path_offset(fdt, "/");
	const char *str = fdt_getprop(fdt, off, "compatible", NULL);
	if (str != NULL) {
		bi = _bootinfo_set_string(bi, BI_FDT_PLATFORM, str, wr);
	}
	bi = _bootinfo_set_data(bi, BI_FDT_BLOB, fdt, fdt_totalsize(fdt), wr);
#else
	bi = _bootinfo_set_string(bi, BI_FDT_PLATFORM,
	    CONFIG_MACHINE_STRING, wr);
#endif /* CONFIG_DEVICETREE */

	/*
	 * Put ELF symbols record at the end.  This is patched up later.
	 * If we have symbols, the addr / size fields are fixed up to
	 * reflect their location.  If we don't, then the tag is set to
	 * BI_LAST.
	 */
	bi = _bootinfo_set_mem_info(bi, BI_FDT_ELF_SYMS, 0, 0, wr);

	return _bootinfo_set_size(bi, BI_LAST, 0, wr);
}

size_t
bootinfo_size(const char *bargs)
{
	return (size_t)_bootinfo_populate(NULL, bargs, false);
}

void
bootinfo_populate(void *vbi, const char *bargs)
{
	(void)_bootinfo_populate(vbi, bargs, true);
}

struct bi_record *
bootinfo_next(struct bi_record *bi)
{
	if (bi->bi_tag == BI_LAST) {
		return NULL;
	}
	return (struct bi_record *)(((uintptr_t)bi) + bi->bi_size);
}

void
bootinfo_enumerate(void *vbi, bool (*cb)(struct bi_record *, void *),
    void *ctx)
{
	struct bi_record *bi = vbi;

	for (; bi != NULL; bi = bootinfo_next(bi)) {
		if ((*cb)(bi, ctx) == false) {
			break;
		}
	}
}

struct bootinfo_find_ctx {
	uint32_t tag;
	struct bi_record *result;
};

static bool
bootinfo_find_cb(struct bi_record *bi, void *v)
{
	struct bootinfo_find_ctx *ctx = v;

	if (bi->bi_tag == ctx->tag) {
		ctx->result = bi;
		return false;
	}

	return true;
}

struct bi_record *
bootinfo_find(void *vbi, uint32_t tag)
{
	struct bootinfo_find_ctx ctx = {
		.tag = tag,
	};

	bootinfo_enumerate(vbi, bootinfo_find_cb, &ctx);
	return ctx.result;
}
