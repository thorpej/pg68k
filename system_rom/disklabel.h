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

#ifndef disklabel_h_included
#define	disklabel_h_included

#include "config.h"
#include "systypes.h"
#include "uuid.h"
#include "queue.h"

/*
 * List of partitions that gets hooked up to a block device.  They
 * are arranged in the order discovered, but also contain an index
 * field in case a particular partitioning scheme defines a numbering
 * system for them.
 */
struct partition_list {
	TAILQ_HEAD(, partition)	pl_list;
	const struct partition	*pl_chosen;
	u_int			pl_scheme;
};

#define	PARTITION_SCHEME_UNKNOWN	0
#define	PARTITION_SCHEME_GPT		1
#define	PARTITION_SCHEME_MBR		2
#define	PARTITION_SCHEME_BSD44		3

struct partition {
	TAILQ_ENTRY(partition)	p_link;
	uint64_t		p_startblk;
	uint64_t		p_nblks;
	int			p_partnum;
	union {
		struct {
			struct uuid gpt_ptype;
			struct uuid gpt_ent;
			char gpt_name[36*3 + 1]; /* UCS-2 -> UTF8 */
		}		p_gpt_info;
		uint8_t		p_mbr_type;
		uint8_t		p_bsd44_type;
	};
};

struct open_file;

int	partition_list_scan(struct open_file *, struct partition_list *);
void	partition_list_discard(struct partition_list *);
int	partition_list_choose(struct partition_list *, int);
void	partition_list_show(struct partition_list *);

const char *partition_scheme_name(u_int);

#endif /* disklabel_h_included */
