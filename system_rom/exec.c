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

#include "loadfile.h"

#include "libfdt.h"

#include "memory.h"

#ifdef CONFIG_MACH_HOST_SIM
#include "simglue.h"
#endif

#ifdef CONFIG_DEVICETREE
/*
 * Allocate a static 4KB buffer for the device tree that's passed to
 * the booted kernel.
 */
static uint32_t fdt_store[4096 / sizeof(uint32_t)];

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

static const char prop_booted_ctlr[] = "pg68k,booted-controller";
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
	char devstr[16];
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

	if (f->f_dev->dv_nargs > 0) {
		snprintf(devstr, sizeof(devstr), "%s%d",
		    f->f_dev->dv_name, f->f_devctlr);
	} else {
		snprintf(devstr, sizeof(devstr), "%s",
		    f->f_dev->dv_name);
	}
	fdterr = fdt_setprop_string(fdt_store, chosen, prop_booted_ctlr,
	    devstr);
	if (fdterr) {
		printf("%s: fdt_setprop(/chosen/%s) - %s\n", __func__,
		    prop_booted_ctlr, fdt_strerror(fdterr));
	}

	if (f->f_dev->dv_nargs > 1) {
		fdterr = fdt_setprop_u32(fdt_store, chosen,
		    prop_booted_unit, f->f_devunit);
		if (fdterr) {
			printf("%s: fdt_setprop(/chosen/%s) - %s\n", __func__,
			    prop_booted_unit, fdt_strerror(fdterr));
		}
	}

	if (f->f_dev->dv_nargs > 2) {
		fdterr = fdt_setprop_u32(fdt_store, chosen,
		    prop_booted_part, f->f_devpart);
		if (fdterr) {
			printf("%s: fdt_setprop(/chosen/%s) - %s\n", __func__,
			    prop_booted_part, fdt_strerror(fdterr));
		}
	}

	if ((p = f->f_partitions.pl_chosen) != NULL) {
		fdterr = fdt_setprop_u64(fdt_store, chosen,
		    prop_booted_pstart, p->p_startblk);
		if (fdterr) {
			printf("%s: fdt_setprop(/chosen/%s) - %s\n", __func__,
			    prop_booted_pstart, fdt_strerror(fdterr));
		}
		fdterr = fdt_setprop_u64(fdt_store, chosen,
		    prop_booted_psize, p->p_nblks);
		if (fdterr) {
			printf("%s: fdt_setprop(/chosen/%s) - %s\n", __func__,
			    prop_booted_psize, fdt_strerror(fdterr));
		}
#ifdef CONFIG_DISKLABEL_GPT
		if (f->f_partitions.pl_scheme == PARTITION_SCHEME_GPT) {
			/* Encode into the native GPT byte order. */
			char buf[sizeof(struct uuid)];
			uuid_enc_le(buf, &p->p_gpt_info.gpt_ent);
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
	const fdt32_t *reg;
	int fdterr;
	fdt32_t newreg[2];

	/*
	 * First, go through our memory bank entries and add fixup
	 * existing FDT entries or add them, as needed.
	 */
	for (bank = 0; bank < memory_bank_count; bank++) {
		offset = find_fdt_memory_entry(memory_banks[bank].start, &reg);
		if (offset >= 0) {
			/* Patch up the entry, if needed. */
			if (memory_banks[bank].size != fdt32_ld(reg + 1)) {
				memcpy(newreg, reg, sizeof(newreg));
				fdt32_st(&newreg[1], memory_banks[bank].size);
				fdterr = fdt_setprop_inplace(fdt_store,
				    offset, "reg", newreg, sizeof(newreg));
				if (fdterr) {
					printf("%s: "
					  "fdt_setprop_inplace(/memory@%lx/reg)"
					  " - %s\n", __func__,
					  (u_long)memory_banks[bank].start,
					  fdt_strerror(fdterr));
				}
			}
		} else {
			/* Create the node. */
			snprintf(memnode_name, sizeof(memnode_name),
			    "memory@%lx",
			    (u_long)memory_banks[bank].start);
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
				continue;
			}
			fdt32_st(&newreg[0], memory_banks[bank].start);
			fdt32_st(&newreg[1], memory_banks[bank].size);
			fdterr = fdt_setprop(fdt_store, offset,
			    "reg", newreg, sizeof(newreg));
			if (fdterr) {
				printf("%s: fdt_setprop(/%s/reg - %s\n",
				    __func__, memnode_name,
				    fdt_strerror(fdterr));
				continue;
			}
		}
	}

	/*
	 * Now go through all of the memory entries and delete the
	 * "reg" property from any we don't know about.
	 */
 again:
	offset = -1;
	while ((offset = fdt_node_offset_by_prop_value(fdt_store, offset,
			"device_type", "memory", sizeof("memory"))) >= 0) {
		reg = fdt_getprop(fdt_store, offset, "reg", &plen);
		if (reg == NULL || plen != sizeof(*reg) * 2) {
			continue;
		}
		if (find_memory_bank_entry(fdt32_ld(reg)) < 0) {
			fdterr = fdt_delprop(fdt_store, offset, "reg");
			if (fdterr < 0) {
				printf("%s: fdt_delprop(/memory@%x/reg - %s\n",
				    __func__, be32toh(reg[0]),
				    fdt_strerror(fdterr));
				continue;
			}
			/* Offsets changed; start over. */
			goto again;
		}
	}
}

static int
exec_prep_fdt(int fd, int load_flags, int argc, char *argv[], u_long *marks)
{
	int i, fdterr, offset;
	size_t size;
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

	/* Construct bootargs. */
	for (size = 0, i = 2; i < argc; i++) {
		size += strlen(argv[i]) + 1;
	}
	if (size != 0) {
		char *bootargs = malloc(size);
		char *cp, *str = bootargs;
		for (i = 2; i < argc; i++) {
			cp = argv[i];
			while ((*str = *cp++) != '\0') {
				str++;
			}
			if (i < argc - 1) {
				*str++ = ' ';
			}
		}
		offset = fdt_path_offset(fdt_store, "/chosen");
		if (offset >= 0) {
			fdterr = fdt_setprop_string(fdt_store, offset,
			    "bootargs", bootargs);
			if (fdterr) {
				printf("%s: fdt_setprop(/chosen/bootargs) "
				    "- %s\n", __func__,
				    fdt_strerror(fdterr));
			}
		}
		free(bootargs);
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
	int error;

	/* Args already checked. */

	int fd = open(argv[1], O_RDONLY);
	if (fd < 0) {
		error = errno;
		printf("%s: %s\n", argv[1], strerror(error));
		return error;
	}

	printf("Loading %s%s ...\n",
	    dev_string(getfile(fd), dstr, sizeof(dstr)),
	    file_name(fd));
	error = fdloadfile(fd, marks, load_flags);
	if (error) {
		error = errno;
		printf("Failed to load %s: %s\n", argv[1], strerror(error));
		close(fd);
		return error;
	}
	if ((load_flags & LOAD_SYM) != 0 && marks[MARK_SYM] != 0) {
		printf("Symbols @ 0x%lx\n", marks[MARK_SYM]);
	}

#ifdef CONFIG_DEVICETREE
	error = exec_prep_fdt(fd, load_flags, argc, argv, marks);
	if (error) {
		close(fd);
		return error;
	}
#endif

	closeall();
	printf("Start @ 0x%lx...\n", marks[MARK_ENTRY]);

#ifdef CONFIG_MACH_HOST_SIM
	sim_booted_fdt(fdt_store, fdt_totalsize(fdt_store));
#endif

	return 0;
}
