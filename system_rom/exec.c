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

#include "bootinfo.h"
#include "loadfile.h"

#include "memory.h"
#include "clock.h"

#ifdef CONFIG_MACH_HOST_SIM
#include "simglue.h"
#endif

#ifdef CONFIG_DEVICETREE
#include "libfdt.h"

/*
 * Allocate a static 4KB buffer for the device tree that's passed to
 * the booted kernel.
 */
static uint32_t fdt_store[4096 / sizeof(uint32_t)];

void *
get_fdt(void)
{
	return fdt_store;
}

static const void *
rom_fdt(void)
{
#ifdef CONFIG_MACH_HOST_SIM
	return sim_rom_fdt();
#else
	extern const char dt_blob_start[];
	return dt_blob_start;
#endif
}

/*
 * We set the following properties to indicate where we booted from:
 *
 * ==> pg68k,booted-controller-type
 *     The controller type (our driver name), e.g. "ata", "eth", etc.
 *
 * ==> pg68k,booted-controller-phys
 *     The system physical address of the booted controller.
 *
 * ==> pg68k,booted-unit
 *     For controllers with multiple drive units (e.g. "ata"), the
 *     unit number of the booted device.
 *
 * ==> pg68k,booted-partition
 * ==> pg68k,booted-partition-startblk		(2 cells)
 * ==> pg68k,booted-partition-nblks		(2 cells)
 *     For device types that can have partitions, the partition number,
 *     starting block, and size in blocks of the booted parition.
 */
static const char prop_booted_ctlr_type[] = "pg68k,booted-controller-type";
static const char prop_booted_ctlr_phys[] = "pg68k,booted-controller-phys";
static const char prop_booted_unit[] = "pg68k,booted-unit";
static const char prop_booted_part[] = "pg68k,booted-partition";
static const char prop_booted_pstart[] = "pg68k,booted-partition-startblk";
static const char prop_booted_psize[] = "pg68k,booted-partition-nblks";
#ifdef CONFIG_DISKLABEL_GPT
static const char prop_booted_gpt_guid[] = "netbsd,gpt-guid";
#endif

static void
set_booted_device(int fd)
{
	struct open_file *f = getfile(fd);
	int chosen, fdterr;
	const struct partition *p;

	if (f->f_dev == NULL) {
		return;
	}

	chosen = fdt_path_offset(fdt_store, "/chosen");
	if (chosen < 0) {
		return;
	}

	/*
	 * Even though we're adding properties to /chosen, the node
	 * offset of /chosen should not change.
	 */

	verbose_printf("FDT: /chosen/%s = \"%s\"\n", prop_booted_ctlr_type,
	    f->f_dev->dv_name);
	fdterr = fdt_setprop_string(fdt_store, chosen, prop_booted_ctlr_type,
	    f->f_dev->dv_name);
	if (fdterr) {
		printf("%s: fdt_setprop(/chosen/%s) - %s\n", __func__,
		    prop_booted_ctlr_type, fdt_strerror(fdterr));
	}

	verbose_printf("FDT: /chosen/%s = 0x%lx\n", prop_booted_ctlr_phys,
	    (u_long)DEVICE_PHYSADDR(f->f_devaddr));
	fdterr = fdt_setprop_u32(fdt_store, chosen, prop_booted_ctlr_phys,
	    DEVICE_PHYSADDR(f->f_devaddr));
	if (fdterr) {
		printf("%s: fdt_setprop(/chosen/%s) - %s\n", __func__,
		    prop_booted_ctlr_phys, fdt_strerror(fdterr));
	}

	if (f->f_dev->dv_nargs > 1) {
		verbose_printf("FDT: /chosen/%s = %u\n",
		    prop_booted_unit, f->f_devunit);
		fdterr = fdt_setprop_u32(fdt_store, chosen,
		    prop_booted_unit, f->f_devunit);
		if (fdterr) {
			printf("%s: fdt_setprop(/chosen/%s) - %s\n", __func__,
			    prop_booted_unit, fdt_strerror(fdterr));
		}
	}

	if (DEV_IS_BLKDEV(f->f_dev) &&
	    (p = f->f_blk.f_partitions.pl_chosen) != NULL &&
	    p->p_partnum >= 0) {
		verbose_printf("FDT: /chosen/%s = %u\n",
		    prop_booted_part, p->p_partnum);
		fdterr = fdt_setprop_u32(fdt_store, chosen,
		    prop_booted_part, p->p_partnum);
		if (fdterr) {
			printf("%s: fdt_setprop(/chosen/%s) - %s\n", __func__,
			    prop_booted_part, fdt_strerror(fdterr));
		}
		verbose_printf("FDT: /chosen/%s = %llu\n",
		    prop_booted_pstart, (unsigned long long)p->p_startblk);
		fdterr = fdt_setprop_u64(fdt_store, chosen,
		    prop_booted_pstart, p->p_startblk);
		if (fdterr) {
			printf("%s: fdt_setprop(/chosen/%s) - %s\n", __func__,
			    prop_booted_pstart, fdt_strerror(fdterr));
		}
		verbose_printf("FDT: /chosen/%s = %llu\n",
		    prop_booted_psize, (unsigned long long)p->p_nblks);
		fdterr = fdt_setprop_u64(fdt_store, chosen,
		    prop_booted_psize, p->p_nblks);
		if (fdterr) {
			printf("%s: fdt_setprop(/chosen/%s) - %s\n", __func__,
			    prop_booted_psize, fdt_strerror(fdterr));
		}
#ifdef CONFIG_DISKLABEL_GPT
		if (f->f_blk.f_partitions.pl_scheme == PARTITION_SCHEME_GPT) {
			/* Encode into the native GPT byte order. */
			char strbuf[UUID_STR_LEN];
			char buf[sizeof(struct uuid)];
			uuid_snprintf(strbuf, sizeof(strbuf),
			    &p->p_gpt_info.gpt_ent);
			uuid_enc_le(buf, &p->p_gpt_info.gpt_ent);
			verbose_printf("FDT: /chosen/%s = << %s >>\n",
			    prop_booted_gpt_guid, strbuf);
			fdterr = fdt_setprop(fdt_store, chosen,
			    prop_booted_gpt_guid, buf, sizeof(buf));
			if (fdterr) {
				printf("%s: fdt_setprop(/chosen/%s) - %s\n",
				    __func__, prop_booted_gpt_guid,
				    fdt_strerror(fdterr));
			}
		}
#endif /* CONFIG_DISKLABEL_GPT */
	}
}

static int
find_fdt_memory_entry(uint32_t addr, const fdt32_t **regp)
{
	int offset;
	int plen;
	const fdt32_t *reg;

	offset = -1;
	while ((offset = fdt_node_offset_by_prop_value(fdt_store, offset,
			"device_type", "memory", sizeof("memory"))) >= 0) {
		reg = fdt_getprop(fdt_store, offset, "reg", &plen);
		if (reg == NULL || plen != sizeof(*reg) * 2) {
			continue;
		}

		if (fdt32_ld(reg) == addr) {
			/* Found it. */
			*regp = reg;
			return offset;
		}
	}
	return offset;
}

static int
find_memory_bank_entry(uint32_t addr)
{
	int bank;

	for (bank = 0; bank < memory_bank_count; bank++) {
		if (memory_banks[bank].start == addr) {
			return bank;
		}
	}
	return -1;
}

static void
set_memory_nodes(void)
{
	char memnode_name[sizeof("memory@XXXXXXXX")];
	int bank, offset, plen;
	int last_entry_offset;
	const fdt32_t *reg;
	int fdterr;
	fdt32_t newreg[2];

	/*
	 * First, go through our memory bank entries and add fixup
	 * existing FDT entries or add them, as needed.
	 */
	for (bank = 0, last_entry_offset = -1;
	     bank < memory_bank_count;
	     bank++, last_entry_offset = offset) {
		if (memory_banks[bank].size == 0) {
			continue;
		}

		/* Attempt to merge adjacent banks into a single entry. */
		if (bank > 0 &&
		    (memory_banks[bank - 1].start +
		     memory_banks[bank - 1].size)
		    == memory_banks[bank].start &&
		    last_entry_offset != -1) {
			reg = fdt_getprop(fdt_store, last_entry_offset,
			    "reg", NULL);
			memcpy(newreg, reg, sizeof(newreg));
			fdt32_st(&newreg[1],
			    fdt32_ld(&newreg[1]) + memory_banks[bank].size);
			verbose_printf("FDT: MERGED "
			    "/memory@%x/reg = <0x%x 0x%x>\n",
			    fdt32_ld(&newreg[0]),
			    fdt32_ld(&newreg[0]),
			    fdt32_ld(&newreg[1]));
			fdterr = fdt_setprop_inplace(fdt_store,
			    offset, "reg", newreg, sizeof(newreg));
			if (fdterr) {
				printf("%s: "
				  "fdt_setprop_inplace(/memory@%x/reg)"
				  " - %s\n", __func__,
				  fdt32_ld(&newreg[0]),
				  fdt_strerror(fdterr));
			}
			continue;
		}

		offset = find_fdt_memory_entry(memory_banks[bank].start, &reg);
		if (offset >= 0) {
			/* Patch up the entry. */
			memcpy(newreg, reg, sizeof(newreg));
			fdt32_st(&newreg[1], memory_banks[bank].size);
			verbose_printf("FDT: /memory@%x/reg = <%x %x>\n",
			    fdt32_ld(&newreg[0]),
			    fdt32_ld(&newreg[0]),
			    fdt32_ld(&newreg[1]));
			fdterr = fdt_setprop_inplace(fdt_store,
			    offset, "reg", newreg, sizeof(newreg));
			if (fdterr) {
				printf("%s: "
				  "fdt_setprop_inplace(/memory@%lx/reg)"
				  " - %s\n", __func__,
				  (u_long)memory_banks[bank].start,
				  fdt_strerror(fdterr));
			}
		} else {
			/* Create the node. */
			snprintf(memnode_name, sizeof(memnode_name),
			    "memory@%lx",
			    (u_long)memory_banks[bank].start);
			fdt32_st(&newreg[0], memory_banks[bank].start);
			fdt32_st(&newreg[1], memory_banks[bank].size);
			verbose_printf("FDT: /%s/reg = <%x %x>\n",
			    memnode_name,
			    fdt32_ld(&newreg[0]),
			    fdt32_ld(&newreg[1]));
			offset = fdt_add_subnode(fdt_store,
			    fdt_path_offset(fdt_store, "/"), memnode_name);
			if (offset < 0) {
				printf("%s: fdt_add_subnode(/%s) - %s\n",
				    __func__, memnode_name,
				    fdt_strerror(offset));
				continue;
			}
			fdterr = fdt_setprop(fdt_store, offset,
			    "device_type", "memory", sizeof("memory"));
			if (fdterr) {
				printf("%s: fdt_setprop(/%s/device_type - %s\n",
				    __func__, memnode_name,
				    fdt_strerror(offset));
				offset = -1;
				continue;
			}
			fdterr = fdt_setprop(fdt_store, offset,
			    "reg", newreg, sizeof(newreg));
			if (fdterr) {
				printf("%s: fdt_setprop(/%s/reg - %s\n",
				    __func__, memnode_name,
				    fdt_strerror(fdterr));
				offset = -1;
				continue;
			}
		}
	}

	/*
	 * Now go through all of the memory entries and delete the
	 * "reg" property from any we don't know about or whose size
	 * is 0.
	 */
 again:
	offset = -1;
	while ((offset = fdt_node_offset_by_prop_value(fdt_store, offset,
			"device_type", "memory", sizeof("memory"))) >= 0) {
		reg = fdt_getprop(fdt_store, offset, "reg", &plen);
		if (reg == NULL || plen != sizeof(*reg) * 2) {
			continue;
		}
		if (find_memory_bank_entry(fdt32_ld(&reg[0])) < 0 ||
		    fdt32_ld(&reg[1]) == 0) {
			fdterr = fdt_delprop(fdt_store, offset, "reg");
			if (fdterr < 0) {
				printf("%s: fdt_delprop(/memory@%x/reg - %s\n",
				    __func__, fdt32_ld(&reg[0]),
				    fdt_strerror(fdterr));
				continue;
			}
			/* Offsets changed; start over. */
			goto again;
		}
	}
}

static int
exec_prep_fdt(int fd, int load_flags, const char *bargs, u_long *marks)
{
	int fdterr, offset;
	const void *rfdt = rom_fdt();

	if (rfdt == NULL) {
		printf("%s: ROM device tree damaged!\n", __func__);
		return EIO;
	}

	fdterr = fdt_open_into(rfdt, fdt_store, sizeof(fdt_store));
	if (fdterr) {
		printf("%s: fdt_open_into() - %s\n", __func__,
		    fdt_strerror(fdterr));
		return EIO;
	}

	/* Set the bootargs. */
	if (bargs != NULL && strlen(bargs) != 0 &&
	    (offset = fdt_path_offset(fdt_store, "/chosen")) >= 0) {
		verbose_printf("FDT: /chosen/bootargs = \"%s\"\n",
		    bargs);
		fdterr = fdt_setprop_string(fdt_store, offset,
		    "bootargs", bargs);
		if (fdterr) {
			printf("%s: fdt_setprop(/chosen/bootargs) "
			    "- %s\n", __func__,
			    fdt_strerror(fdterr));
		}
	}

	/* Set the boot device entries. */
	set_booted_device(fd);

	/* Set the memory entries. */
	set_memory_nodes();

	fdterr = fdt_pack(fdt_store);
	if (fdterr) {
		printf("%s: fdt_pack() - %s\n", __func__,
		    fdt_strerror(fdterr));
		return EIO;
	}

	return 0;
}
#endif /* CONFIG_DEVICETREE */

int
exec(int load_flags, int argc, char *argv[])
{
	char dstr[DEV_STRING_SIZE];
	u_long marks[MARK_MAX] = { 0 };
	char *bargs = NULL;
	size_t size;
	int i, error;

	/* Args already checked. */

	int fd = open(argv[1], O_RDONLY);
	if (fd < 0) {
		error = errno;
		printf("%s: %s\n", argv[1], strerror(error));
		goto bad;
	}

	/* Construct bootargs. */
	for (size = 0, i = 2; i < argc; i++) {
		size += strlen(argv[i]) + 1;
	}
	if (size != 0) {
		bargs = malloc(size);
		char *cp, *str = bargs;
		for (i = 2; i < argc; i++) {
			cp = argv[i];
			while ((*str = *cp++) != '\0') {
				str++;
			}
			if (i < argc - 1) {
				*str++ = ' ';
			}
		}
	}

#ifdef CONFIG_DEVICETREE
	error = exec_prep_fdt(fd, load_flags, bargs, marks);
	if (error) {
		goto bad;
	}
#endif

	if (load_flags & LOAD_BOOTINFO) {
		marks[MARK_BOOTINFOSZ] = bootinfo_size(bargs);
	}

	printf("Loading %s%s ...\n",
	    dev_string(getfile(fd), dstr, sizeof(dstr)),
	    file_name(fd));
	error = fdloadfile(fd, marks, load_flags);
	if (error) {
		error = errno;
		printf("Failed to load %s: %s\n", argv[1], strerror(error));
		goto bad;
	}

	bool have_bootinfo = (load_flags & LOAD_BOOTINFO) != 0
			     && marks[MARK_BOOTINFO] != 0;

	bool have_sym      = (load_flags & LOAD_SYM) != 0
			     && marks[MARK_SYM] != 0;

	if ((load_flags & LOAD_BOOTINFO) != 0 && !have_bootinfo) {
		printf("Failed to reserve space for Bootinfo.\n");
		error = ENOMEM;
		goto bad;
	}

	struct bi_record *bi;
	void *vbi = (void *)marks[MARK_BOOTINFO];

	if (have_bootinfo) {
		bootinfo_populate(vbi, bargs);
		if (have_sym) {
			bi = bootinfo_find(vbi, BI_PG68K_ELF_SYMS);
			if (bi != NULL) {
				bootinfo_set_mem_info(bi,
				    BI_PG68K_ELF_SYMS,
				    marks[MARK_SYM],
				    marks[MARK_END] - marks[MARK_SYM]);
			}
		}
		printf("Bootinfo @ 0x%lx\n", marks[MARK_BOOTINFO]);
#ifdef CONFIG_DEVICETREE
		bi = bootinfo_find(vbi, BI_PG68K_FDT);
		if (bi != NULL) {
			struct bi_data *d = bootinfo_dataptr(bi);
			printf("FDT @ %p\n", &d->data_bytes[0]);
		}
#endif /* CONFIG_DEVICETREE */
	}

	if (have_sym) {
		printf("Symbols @ 0x%lx\n", marks[MARK_SYM]);
	}

	if (bargs != NULL) {
		free(bargs);
	}
	closeall();
	quiesce();
	printf("Start @ 0x%lx...\n", marks[MARK_ENTRY]);

#ifdef CONFIG_MACH_HOST_SIM
	sim_boot_fdt(fdt_store, fdt_totalsize(fdt_store));
#else
	if (load_flags & LOAD_BOOTINFO) {
		/*
		 * The bootinfo is loaded at the next longword boundary
		 * after _end[], but we will pass in a pointer as the
		 * argument anyway.
		 */
		transfer(marks[MARK_ENTRY], vbi);
	} else {
		transfer(marks[MARK_ENTRY], argc, argv);
	}
#endif /* CONFIG_MACH_HOST_SIM */

	/* NOTREACHED */
	return 0;
 bad:
	if (fd >= 0) {
		close(fd);
	}
	if (bargs != NULL) {
		free(bargs);
	}
	return error;
}
