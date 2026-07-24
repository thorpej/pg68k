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
#include "nvram.h"

#define	NVRAM_VERSION		1
#define	NVRAM_MAGIC		"pg68k,nvram"	/* 12 bytes incl. NUL */

struct nvram_header {
	uint8_t		magic[12];
	uint32_t	version;
	uint32_t	cksum;
};

#define	BOOTDEV_SZ	16
#define	BOOTFILE_SZ	32
#define	BOOTARGS_SZ	32

struct nvram_payload_v1 {
	uint8_t		bootdev[BOOTDEV_SZ];	/* C string */
	uint8_t		bootfile[BOOTFILE_SZ];	/* C string */
	uint8_t		bootargs[BOOTARGS_SZ];	/* C string */
	/* remainder is reserved and MBZ */
};

#define	NVRAM_VAR_STRING	0

struct nvram_var {
	const char *name;
	size_t offset;
	size_t size;
	int type;
};

static const struct nvram_var nvram_vars_v1[] = {
	{ .name = "bootdev",
	  .offset = offsetof(struct nvram_payload_v1, bootdev),
	  .size = BOOTDEV_SZ,
	  .type = NVRAM_VAR_STRING },

	{ .name = "bootfile",
	  .offset = offsetof(struct nvram_payload_v1, bootfile),
	  .size = BOOTFILE_SZ,
	  .type = NVRAM_VAR_STRING },

	{ .name = "bootargs",
	  .offset = offsetof(struct nvram_payload_v1, bootargs),
	  .size = BOOTARGS_SZ,
	  .type = NVRAM_VAR_STRING },
};
static const int nvram_num_vars_v1 = __arraycount(nvram_vars_v1);

#if defined(CONFIG_NVRAM_DS3232)
#include "nvram_dsrtc.c"
#endif

static uint32_t
nvram_checksum(const struct nvram_data *d)
{
	uint32_t sum = 0;
	uint32_t *v = (uint32_t *)d;
	int i;

	for (i = 0; i < sizeof(*d) / sizeof(uint32_t); i++) {
		sum ^= v[i];
	}

	return sum;
}

static void
nvram_init_defaults(void)
{
	struct nvram_data *d = &nvram_data;

	memset(d, 0, sizeof(*d));
	memcpy(d->hdr.magic, NVRAM_MAGIC, sizeof(d->hdr.magic));
	d->hdr.version = NVRAM_VERSION;

	struct nvram_payload_v1 *v1 = (void *)d->payload;
	strcpy((char *)v1->bootdev, CONFIG_NVRAM_DEFAULT_BOOTDEV);
	strcpy((char *)v1->bootfile, CONFIG_NVRAM_DEFAULT_BOOTFILE);
	strcpy((char *)v1->bootargs, CONFIG_NVRAM_DEFAULT_BOOTARGS);

	d->hdr.cksum = nvram_checksum(d);
}

static bool nvram_write_back;

static int
nvram_save(void)
{
	struct nvram_data *d = &nvram_data;

	d->hdr.cksum = 0;
	d->hdr.cksum = nvram_checksum(d);

	return nvram_write_back ? nvram_write() : 0;
}

void
nvram_init(void)
{
	struct nvram_data *d = &nvram_data;

	if (! nvram_present()) {
 use_defaults:
		nvram_init_defaults();
		(void)nvram_save();
		return;
	}

	/* Read the NVRAM blob. */
	if (nvram_read() != 0) {
		goto use_defaults;
	}

	/*
	 * From this point on, every bad NVRAM check causes us to write
	 * back the defaults.
	 */
	nvram_write_back = true;

	if (memcmp(d->hdr.magic, NVRAM_MAGIC, sizeof(d->hdr.magic)) != 0) {
		configure_printf("NVRAM: bad magic, "
				 "using defaults.\n");
		goto use_defaults;
	}

	if (d->hdr.version != NVRAM_VERSION) {
		configure_printf("NVRAM: invalid version %u, "
				 "using defaults.\n", d->hdr.version);
		goto use_defaults;
	}

	if (nvram_checksum(d) != 0) {
		configure_printf("NVRAM: invalid checksum, "
				 "using defaults.\n");
		goto use_defaults;
	}

	/* Ensure the strings are NUL-terminated. */
	uint8_t *blob = (void *)d->payload;
	const struct nvram_var *v;
	for (int i = 0; i < nvram_num_vars_v1; i++) {
		v = &nvram_vars_v1[i];
		if (v->type != NVRAM_VAR_STRING) {
			continue;
		}
		blob[v->offset + v->size - 1] = '\0';
	}
}

static const struct nvram_var *
nvram_lookup(const char *name)
{
	const struct nvram_var *var;

	for (int i = 0; i < nvram_num_vars_v1; i++) {
		var = &nvram_vars_v1[i];
		if (strcmp(var->name, name) == 0) {
			return var;
		}
	}

	return NULL;
}

const char *
nvram_get_string(const char *name)
{
	const struct nvram_var *var = nvram_lookup(name);

	if (var == NULL ||
	    var->type != NVRAM_VAR_STRING) {
	    	return NULL;
	}

	return (const char *)&nvram_data.payload[var->offset];
}

int
nvram_set(const char *name, const char *val)
{
	const struct nvram_var *var = nvram_lookup(name);
	uint8_t *payload = nvram_data.payload;

	if (var == NULL) {
		return EINVAL;
	}

	switch (var->type) {
	case NVRAM_VAR_STRING:
		if (strlen(val) >= var->size) {
			return EFBIG;
		}
		memset(&payload[var->offset], 0, var->size);
		strcpy((char *)&payload[var->offset], val);
		break;

	default:
		return EINVAL;
	}

	return nvram_save();
}

void
nvram_print(const char *name)
{
	const struct nvram_var *var = nvram_lookup(name);
	uint8_t *payload = nvram_data.payload;
	char nstr[16];

	if (var == NULL) {
		printf("\n");
		return;
	}

	snprintf(nstr, sizeof(nstr), "%s=", var->name);

	switch (var->type) {
	case NVRAM_VAR_STRING:
		printf("%-16s%s\n", nstr, (char *)&payload[var->offset]);
		break;

	default:
		printf("\n");
	}
}

void
nvram_print_all(void)
{
	const struct nvram_var *var;
	int i;

	for (i = 0; i < nvram_num_vars_v1; i++) {
		var = &nvram_vars_v1[i];
		nvram_print(var->name);
	}
}
