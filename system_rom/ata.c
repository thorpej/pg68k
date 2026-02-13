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

#include "ata.h"

#ifdef ATA_ADDR
const uintptr_t ata_addrs[] = {
	ATA_ADDR, ATA_AUX_ADDR,
};
int	ata_count = arraycount(ata_addrs) / 2;
#endif /* ATA_ADDR */

#define	MAXATA		1	/* max # of controllers */

void
ata_configure(bool do_init)
{
	int i;

	for (i = 0; i < ata_count; i++) {
		configure_printf("ata%d at 0x%08lx\n", i,
		    (u_long)ata_addrs[(i << 1)]);
		ata_init(i, do_init);
	}
}

#if defined(CONFIG_ATA_GENERIC)
#include "ata_generic.c"
#elif defined(CONFIG_ATA_HOST_SIM)
#include "ata_hostsim.c"
#endif

const struct devsw ata_devsw = {
	.dv_name	=	"ata",
	.dv_nargs	=	3,	/* ctlr,unit,part */
	.dv_blk = {
		.dv_strategy =	ata_strategy,
	},
	.dv_open	=	ata_open,
	.dv_close	=	ata_close,
	.dv_ioctl	=	ata_ioctl,
};
