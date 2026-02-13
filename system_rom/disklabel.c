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
#include "sysfile.h"
#include "endian.h"
#include "uuid.h"
#include "disklabel.h"

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
#include "disklabel_gpt.h"

static const struct uuid ent_type_unused = GPT_ENT_TYPE_UNUSED;
static const struct uuid ent_type_netbsd_swap = GPT_ENT_TYPE_NETBSD_SWAP;
static const struct uuid ent_type_netbsd_ffs = GPT_ENT_TYPE_NETBSD_FFS;
static const struct uuid ent_type_ms_basic_dat = GPT_ENT_TYPE_MS_BASIC_DATA;
static const struct uuid ent_type_linux_dat = GPT_ENT_TYPE_LINUX_DATA;
static const struct uuid ent_type_linux_swap = GPT_ENT_TYPE_LINUX_SWAP;

static const struct gpt_type_name {
	const struct uuid *type;
	const char *name;
} gpt_type_names[] = {
	{ .type = &ent_type_netbsd_swap,
	  .name = "NetBSD swap" },
	{ .type = &ent_type_netbsd_ffs,
	  .name = "NetBSD FFS" },
	{ .type = &ent_type_ms_basic_dat,
	  .name = "MS Basic Data" },
	{ .type = &ent_type_linux_dat,
	  .name = "Linux data" },
	{ .type = &ent_type_linux_swap,
	  .name = "Linux swap" },
};

static const char *
gpt_type_name(const struct uuid *type, char *buf, size_t buflen)
{
	for (int i = 0; i < arraycount(gpt_type_names); i++) {
		if (memcmp(gpt_type_names[i].type, type, sizeof(*type)) == 0) {
			return gpt_type_names[i].name;
		}
	}
	uuid_snprintf(buf, buflen, type);
	return buf;
}

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

	lba_start = le64toh(hdr->hdr_lba_start);
	lba_end = le64toh(hdr->hdr_lba_end);
	lba_table = le64toh(hdr->hdr_lba_table);
	if (lba_start < 0 || lba_end < 0 || lba_table < 0) {
		verbose_printf("WARNING: GPT block numbers out of range\n");
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
		verbose_printf("WARNING: unable to read GPT partition array: "
			       "%s\n", strerror(error));
		goto out;
	}

	if (crc32(0, buf, entries * entsz) != gpe_crc) {
		/* XXX Should check alternate location. */
		verbose_printf("Bad GPT partition array CRC\n");
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

static void
partition_list_show_gpt(struct partition_list *pl)
{
	struct partition *p;
	char uuidstr[UUID_STR_LEN];

	TAILQ_FOREACH(p, &pl->pl_list, p_link) {
		printf("%3d: start=%-10llu size=%-10llu type=%s\n",
		    p->p_partnum, (unsigned long long)p->p_startblk,
		    (unsigned long long)p->p_nblks,
		    gpt_type_name(&p->p_gpt_info.gpt_ptype,
				  uuidstr, sizeof(uuidstr)));
	}
}
#endif /* CONFIG_DISKLABEL_GPT */

#ifdef CONFIG_DISKLABEL_BSD44
#include "disklabel_bsd44.h"

static const struct bsd44_type_name {
	uint8_t type;
	const char *name;
} bsd44_type_names[] = {
	{ .type = FS_SWAP,
	  .name = "swap" },
	{ .type = FS_BSDFFS,
	  .name = "4.2BSD" },
	{ .type = FS_EX2FS,
	  .name = "Linux Ext2" },
};

static const char *
bsd44_type_name(uint8_t type, char *buf, size_t buflen)
{
	for (int i = 0; i < arraycount(bsd44_type_names); i++) {
		if (bsd44_type_names[i].type == type) {
			return bsd44_type_names[i].name;
		}
	}
	snprintf(buf, buflen, "<%u>", type);
	return buf;
}

/*
 * The BSD partition scheme is a bit annoying in that not every
 * platform that used it kept the label in the same place on disk.
 * Some placed it in sector 0, some in sector 1, some in sector 2.
 * Furthermore, it was not always located at the beginning of the
 * sector -- sometimes offset 64 bytes, sometimes offset 128 bytes.
 *
 * To complicate matters, some platforms did something even sillier:
 * the BSD disk label was stashed inside of the "BSD portion" of a
 * disk labeled with an MBR partition table (with either the 386BSD
 * or NETBSD partition type).  Not going to bother supporting that.
 */
static const daddr_t bsd44_label_sectors[] = { 0, 1, 2 };

#define	DISKLABEL_BSD44_SIZE(x)						\
	(offsetof(struct disklabel_bsd44, d_partitions) +		\
	 (sizeof(struct partition_bsd44) * (x)))

/* Smallest 4.4BSD disklabel that was ever in use. */
#define	DISKLABEL_BSD44_MINSIZE	DISKLABEL_BSD44_SIZE(8)

static uint16_t
dkcsum_bsd44(const struct disklabel_bsd44 *lp, size_t nparts)
{
	const uint16_t *start = (const uint16_t *)lp;
	const uint16_t *end   = (const uint16_t *)&lp->d_partitions[nparts];
	uint16_t sum = 0;

	while (start < end) {
		sum ^= *start++;
	}
	return sum;
}

static struct disklabel_bsd44 *
find_bsd44_label_in_sector(void *buf, size_t secsize, bool *swappedp)
{
	struct disklabel_bsd44 *lp = buf;
	uintptr_t lp_lim = ((uintptr_t)buf + secsize - DISKLABEL_BSD44_MINSIZE);
	uint16_t npartitions;

	for (;; lp = (void *)((uintptr_t)lp + sizeof(uint32_t))) {
		if ((uintptr_t)lp > lp_lim) {
			/* Not found in this sector. */
			return NULL;
		}
		if (lp->d_magic != DISKMAGIC || lp->d_magic2 != DISKMAGIC) {
			if (lp->d_magic != bswap32(DISKMAGIC) ||
			    lp->d_magic2 != bswap32(DISKMAGIC)) {
				continue;
			}
			/* Need to swap label. */
			*swappedp = true;
		} else {
			*swappedp = false;
		}

		npartitions = (*swappedp) ? bswap16(lp->d_npartitions)
					  : lp->d_npartitions;

		if ((uintptr_t)lp + DISKLABEL_BSD44_SIZE(npartitions) >
		    (uintptr_t)buf + secsize) {
			verbose_printf("4.4BSD partition count out of range\n");
			continue;
		}

		if (dkcsum_bsd44(lp, npartitions) != 0) {
			verbose_printf("Bad 4.4BSD disk label checksum\n");
			continue;
		}

		/* Label looks OK. */
		return lp;
	}
}

static int
partition_list_scan_bsd44(struct open_file *f, struct partition_list *pl)
{
	size_t secsize = getsecsize(f);
	void *buf;
	const struct disklabel_bsd44 *lp;
	const struct partition_bsd44 *pp;
	struct partition *p;
	int error = 0;
	bool swapped;
	int i, nparts;
	uint64_t startblk, nblks;

	buf = malloc(secsize);
	if (buf == NULL) {
		return ENOMEM;
	}

	pl->pl_scheme = PARTITION_SCHEME_BSD44;

	for (i = 0; i < arraycount(bsd44_label_sectors); i++) {
		/* Different (or first) sector; read it. */
		error = dev_read(f, bsd44_label_sectors[i], buf, secsize);
		if (error) {
			goto out;
		}
		lp = find_bsd44_label_in_sector(buf, secsize, &swapped);
		if (lp != NULL) {
			/* Found it! */
			break;
		}
	}

	if (lp == NULL) {
		/* Didn't find one. :-( */
		error = ESRCH;
		goto out;
	}

	nparts = swapped ? bswap16(lp->d_npartitions) : lp->d_npartitions;
	for (i = 0; i < nparts; i++) {
		pp = &lp->d_partitions[i];
		if (pp->p_fstype == FS_UNUSED) {
			continue;
		}

		/* XXX Explicitly skip "whole disk" partition? */

		startblk = swapped ? bswap32(pp->p_offset) : pp->p_offset;
		nblks    = swapped ? bswap32(pp->p_size)   : pp->p_size;

		/* XXX Make sure it falls within the disk's data area. */

		p = partition_list_add_partition(pl, startblk, nblks, i);
		if (p == NULL) {
			error = ENOMEM;
			goto out;
		}
		p->p_bsd44_type = pp->p_fstype;
	}

 out:
	if (buf != NULL) {
		free(buf);
	}
	return error;
}

static struct partition *
bsd44_search_type(struct partition_list *pl, uint8_t fstype)
{
	struct partition *p;

	TAILQ_FOREACH(p, &pl->pl_list, p_link) {
		if (p->p_bsd44_type == fstype) {
			return p;
		}
	}
	return NULL;
}

static void
partition_list_choose_bsd44(struct partition_list *pl)
{
	struct partition *p;

	/*
	 * We're being asked to choose a default partition because one
	 * was not specified.  Look for the first partition of any of
	 * these types and, if found, select as the default.
	 */
	if ((p = bsd44_search_type(pl, FS_BSDFFS)) != NULL ||
	    (p = bsd44_search_type(pl, FS_EX2FS)) != NULL) {
		pl->pl_chosen = p;
	}
}

static void
partition_list_show_bsd44(struct partition_list *pl)
{
	struct partition *p;
	char buf[sizeof("<XXX>")];

	TAILQ_FOREACH(p, &pl->pl_list, p_link) {
		printf("%3d: start=%-10llu size=%-10llu type=%s\n",
		    p->p_partnum, (unsigned long long)p->p_startblk,
		    (unsigned long long)p->p_nblks,
		    bsd44_type_name(p->p_bsd44_type, buf, sizeof(buf)));
	}
}
#endif /* CONFIG_DISKLABEL_BSD44 */

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
#ifdef CONFIG_DISKLABEL_BSD44
	error = partition_list_scan_bsd44(f, pl);
	if (error == 0) {
		return 0;
	}
#endif

	if (error) {
		partition_list_discard(pl);
	}
	return 0;
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

static const struct partition full_disk = {
	.p_startblk	= 0,
	.p_nblks	= UINT64_MAX,
	.p_partnum	= -2,
};

int
partition_list_choose(struct partition_list *pl, int *partnump)
{
	const struct partition *p;
	int partnum = *partnump;

	pl->pl_chosen = NULL;

	if (partnum == -2) {
		pl->pl_chosen = &full_disk;
		return 0;
	}

	if (partnum == -1) {
		switch (pl->pl_scheme) {
#ifdef CONFIG_DISKLABEL_GPT
		case PARTITION_SCHEME_GPT:
			partition_list_choose_gpt(pl);
			break;
#endif
#ifdef CONFIG_DISKLABEL_BSD44
		case PARTITION_SCHEME_BSD44:
			partition_list_choose_bsd44(pl);
			break;
#endif
		default:
			break;
		}
		if ((p = pl->pl_chosen) != NULL) {
			*partnump = p->p_partnum;
			verbose_printf("Choosing partition %u\n", p->p_partnum);
			return 0;
		} else if (TAILQ_EMPTY(&pl->pl_list)) {
			*partnump = -2;
			pl->pl_chosen = &full_disk;
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

void
partition_list_show(struct partition_list *pl)
{
	switch (pl->pl_scheme) {
#ifdef CONFIG_DISKLABEL_GPT
	case PARTITION_SCHEME_GPT:
		partition_list_show_gpt(pl);
		break;
#endif
#ifdef CONFIG_DISKLABEL_BSD44
	case PARTITION_SCHEME_BSD44:
		partition_list_show_bsd44(pl);
		break;
#endif
	default:
		break;
	}
}

const char *
partition_scheme_name(u_int scheme)
{
	static const char * const names[] = {
	[PARTITION_SCHEME_UNKNOWN]	=	"(unknown)",
	[PARTITION_SCHEME_GPT]		=	"GUID Partition Table",
	[PARTITION_SCHEME_MBR]		=	"Master Boot Record",
	[PARTITION_SCHEME_BSD44]	=	"4.4BSD",
	};

	if (scheme > arraycount(names) || names[scheme] == NULL) {
		scheme = PARTITION_SCHEME_UNKNOWN;
	}
	return names[scheme];
}
