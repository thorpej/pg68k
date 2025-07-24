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

#include "systypes.h"
#include "syslib.h"
#include "sysfile.h"
#include "simglue.h"

#include "ata.h"

void
ata_init(int ctlr)
{
	if (ctlr == 0) {
		sim_ata_init();
	}
}

static int
ata_strategy(struct open_file *f, int flags, daddr_t blk, size_t len,
    void *buf, size_t *residp)
{
	return sim_ata_strategy(f->f_devdata, flags, blk, len, buf, residp);
}

static int
ata_open(struct open_file *f)
{
	if (f->f_devctlr != 0) {
		return ENXIO;
	}

	if (f->f_devunit < 0 || f->f_devunit > 1) {
		return ENXIO;
	}

	return sim_ata_open(f->f_devunit, &f->f_devdata);
}

static int
ata_close(struct open_file *f)
{
	int error = sim_ata_close(f->f_devdata);
	f->f_devdata = NULL;
	return error;
}

static int
ata_ioctl(struct open_file *f, u_long cmd, void *data)
{
	return sim_ata_ioctl(f->f_devdata, cmd, data);
}

const struct devsw ata_devsw = {
	.dv_name	=	"ata",
	.dv_nargs	=	3,	/* ctlr,unit,part */
	.dv_strategy	=	ata_strategy,
	.dv_open	=	ata_open,
	.dv_close	=	ata_close,
	.dv_ioctl	=	ata_ioctl,
};
