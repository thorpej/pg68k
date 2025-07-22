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

/*-
 * GPT parsing is derived from code that carries this license:
 *
 * Copyright (c) 2004 The NetBSD Foundation, Inc.
 * All rights reserved.
 *
 * This code is derived from software contributed to The NetBSD Foundation
 * by Jason R. Thorpe.
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
 * THIS SOFTWARE IS PROVIDED BY THE NETBSD FOUNDATION, INC. AND CONTRIBUTORS
 * ``AS IS'' AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED
 * TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL THE FOUNDATION OR CONTRIBUTORS
 * BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include "config.h"
#include "syslib.h"
#include "endian.h"
#include "uuid.h"
#include "disklabel.h"
#include "disklabel_gpt.h"

static struct partition *
partition_list_add_partition(struct partition_list *pl, uint64_t startblk,
    uint64_t nblks, u_int partnum)
{
	struct partition *p = calloc(1, sizeof(*p));
	if (p != NULL) {
		p->p_startblk = startblk;
		p->p_nblks = nblks;
		p->p_partnum = partnum;
		TAILQ_INSERT_TAIL(&pl->pl_list, p, p_link);
	}
	return p;
}

#ifdef CONFIG_DISKLABEL_GPT
static const struct uuid ent_type_unused = GPT_ENT_TYPE_UNUSED;
static const struct uuid ent_type_netbsd_ffs = GPT_ENT_TYPE_NETBSD_FFS;
static const struct uuid ent_type_ms_basic_dat = GPT_ENT_TYPE_MS_BASIC_DATA;
static const struct uuid ent_type_linux_dat = GPT_ENT_TYPE_LINUX_DATA;

static int
gpt_verify_header_crc(struct gpt_hdr *hdr)
{
	uint32_t crc;
	int rv;

	crc = hdr->hdr_crc_self;
	hdr->hdr_crc_self = 0;
	rv = le32toh(crc) == crc32(0, (void *)hdr, le32toh(hdr->hdr_size));
	hdr->hdr_crc_self = crc;

	return rv;
}

static int
partition_list_scan_gpt(struct open_file *f, struct partition_list *pl)
{
	static const char gpt_hdr_sig[] = GPT_HDR_SIG;
	size_t sz, secsize = getsecsize(f);
	void *buf;
	struct gpt_hdr *hdr;
	struct gpt_ent *ent;
	uint32_t entries, entsz;
	daddr_t lba_start, lba_end, lba_table;
	uint32_t gpe_crc;
	int error = 0;

	sz = secsize;
	buf = malloc(sz);

	if (buf == NULL) {
		return ENOMEM;
	}

	pl->pl_scheme = PARTITION_SCHEME_GPT;

	/* Read in the GPT Header. */
	error = dev_read(f, GPT_HDR_BLKNO, buf, sz);
	if (error) {
		goto out;
	}
	hdr = buf;

	/* Validate it. */
	if (memcmp(gpt_hdr_sig, hdr->hdr_sig, sizeof(hdr->hdr_sig)) != 0) {
		/* XXX Should check at end-of-disk. */
		error = ESRCH;
		goto out;
	}
	if (hdr->hdr_revision != htole32(GPT_HDR_REVISION)) {
		/* XXX Should check at end-of-disk. */
		error = ESRCH;
		goto out;
	}
	if (le32toh(hdr->hdr_size) > secsize) {
		/* XXX Should check at end-of-disk. */
		error = ESRCH;
		goto out;
	}
	if (gpt_verify_header_crc(hdr) == 0) {
		/* XXX Should check at end-of-disk. */
		error = ESRCH;
		goto out;
	}

	/* XXX Now that we found it, should we validate the backup? */

	entries = le32toh(hdr->hdr_entries);
	entsz = roundup(le32toh(hdr->hdr_entsz), 8);
	if (entsz != sizeof(struct gpt_ent)) {
		error = ESRCH;
		goto out;
	}
	gpe_crc = le32toh(hdr->hdr_crc_table);

	/* XXX Clamp entries at 16 for now. */
	if (entries > 16) {
		printf("WARNING: clamping number of GPT entries to 16 "
		       "(was %u)\n", entries);
		entries = 16;
	}

	lba_start = le64toh(hdr->hdr_lba_start);
	lba_end = le64toh(hdr->hdr_lba_end);
	lba_table = le64toh(hdr->hdr_lba_table);
	if (lba_start < 0 || lba_end < 0 || lba_table < 0) {
		printf("WARNING: GPT block numbers out of range\n");
		error = ESRCH;
		goto out;
	}

	free(buf);
	sz = roundup(entries * entsz, getsecsize(f));
	buf = malloc(sz);
	if (buf == NULL) {
		error = ENOMEM;
		goto out;
	}
	error = dev_read(f, lba_table, buf, sz);
	if (error) {
		/* XXX Should check alternate location. */
		printf("WARNING: unable to read GPT partition array: %s\n",
		    strerror(error));
		goto out;
	}

	if (crc32(0, buf, entries * entsz) != gpe_crc) {
		/* XXX Should check alternate location. */
		printf("Bad GPT partition array CRC\n");
		error = ESRCH;
		goto out;
	}

	for (u_int i = 0; i < entries; i++) {
		struct uuid ptype_guid;
		uint64_t startblk, nblks;
		struct partition *p;
		char *cp;
		size_t r, n;

		ent = (struct gpt_ent *)((uintptr_t)buf + (i * entsz));

		uuid_dec_le(ent->ent_type, &ptype_guid);
		if (memcmp(&ptype_guid, &ent_type_unused,
			   sizeof(ptype_guid)) == 0) {
			continue;
		}

		startblk = le64toh(ent->ent_lba_start);
		nblks = le64toh(ent->ent_lba_end) - startblk + 1;

		/* XXX Make sure it falls within the disk's data area. */

		p = partition_list_add_partition(pl, startblk, nblks, i);
		if (p == NULL) {
			error = ENOMEM;
			goto out;
		}
		memcpy(&p->p_gpt_info.gpt_ptype, &ptype_guid,
		    sizeof(p->p_gpt_info.gpt_ptype));
		uuid_dec_le(ent->ent_guid, &p->p_gpt_info.gpt_ent);

		cp = p->p_gpt_info.gpt_name;
		r = sizeof(p->p_gpt_info.gpt_name) - 1;
		for (int j = 0;
		     j < arraycount(ent->ent_name) && ent->ent_name[j] != 0;
		     j++) {
			n = ucs2_to_utf8(cp, r, le16toh(ent->ent_name[j]));
			if (n == 0) {
				break;
			}
			cp += n; r -= n;
		}
		*cp = '\0';
	}

 out:
	if (buf != NULL) {
		free(buf);
	}
	return error;
}

static struct partition *
gpt_search_type(struct partition_list *pl, const struct uuid *type)
{
	struct partition *p;

	TAILQ_FOREACH(p, &pl->pl_list, p_link) {
		if (memcmp(&p->p_gpt_info.gpt_ptype, type,
			   sizeof(*type)) == 0) {
			return p;
		}
	}
	return NULL;
}

static void
partition_list_choose_gpt(struct partition_list *pl)
{
	struct partition *p;

	/*
	 * We're being asked to choose a default partition because one
	 * was not specified.  Look for the first partition of any of
	 * these types and, if found, select as the default.
	 */
	if ((p = gpt_search_type(pl, &ent_type_netbsd_ffs)) != NULL ||
	    (p = gpt_search_type(pl, &ent_type_linux_dat)) != NULL ||
	    (p = gpt_search_type(pl, &ent_type_ms_basic_dat)) != NULL) {
		pl->pl_chosen = p;
	}
}
#endif /* CONFIG_DISKLABEL_GPT */

int
partition_list_scan(struct open_file *f, struct partition_list *pl)
{
	int error = ENXIO;

	pl->pl_chosen = NULL;
	pl->pl_scheme = PARTITION_SCHEME_UNKNOWN;
	TAILQ_INIT(&pl->pl_list);

#ifdef CONFIG_DISKLABEL_GPT
	error = partition_list_scan_gpt(f, pl);
	if (error == 0) {
		return 0;
	}
#endif

	if (error) {
		partition_list_discard(pl);
	}
	return error;
}

void
partition_list_discard(struct partition_list *pl)
{
	struct partition *p;

	pl->pl_chosen = NULL;
	pl->pl_scheme = PARTITION_SCHEME_UNKNOWN;
	while ((p = TAILQ_FIRST(&pl->pl_list)) != NULL) {
		TAILQ_REMOVE(&pl->pl_list, p, p_link);
		free(p);
	}
}

int
partition_list_choose(struct partition_list *pl, int partnum)
{
	const struct partition *p;

	pl->pl_chosen = NULL;

	if (partnum == -1) {
		switch (pl->pl_scheme) {
#ifdef CONFIG_DISKLABEL_GPT
		case PARTITION_SCHEME_GPT:
			partition_list_choose_gpt(pl);
			break;
#endif
		default:
			break;
		}
		if ((p = pl->pl_chosen) != NULL) {
			printf("Choosing partition %u\n", p->p_partnum);
			return 0;
		}
		return ESRCH;
	}

	TAILQ_FOREACH(p, &pl->pl_list, p_link) {
		if (p->p_partnum == partnum) {
			pl->pl_chosen = p;
			return 0;
		}
	}
	return ESRCH;
}

const char *
partition_scheme_name(u_int scheme)
{
	static const char * const names[] = {
	[PARTITION_SCHEME_UNKNOWN]	=	"(unknown)",
	[PARTITION_SCHEME_GPT]		=	"GUID Partition Table",
	[PARTITION_SCHEME_MBR]		=	"Master Boot Record",
	[PARTITION_SCHEME_BSD44]	=	"4.4BSD disk label",
	};

	if (scheme > arraycount(names) || names[scheme] == NULL) {
		scheme = PARTITION_SCHEME_UNKNOWN;
	}
	return names[scheme];
}
