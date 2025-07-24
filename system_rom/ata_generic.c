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
#include "pio.h"

#include "ata.h"
#include "atareg.h"

#define	REG_ADDR(u, r)		(ata_addrs[((u) << 1)    ] + _REG_OFF(r))
#define	CTLREG_ADDR(u, r)	(ata_addrs[((u) << 1) + 1] + _REG_OFF(r))

#define	REG_READ(u, r)		inb(REG_ADDR((u), (r)))
#define	REG_WRITE(u, r, v)	outb(REG_ADDR((u), (r)), (v))

#define	CTLREG_READ(u, r)	inb(CTLREG_ADDR((u), (r)))
#define	CTLREG_WRITE(u, r, v)	outb(CTLREG_ADDR((u), (r)), (v))

#define	DATA_IN8(u, b, c)	insb(REG_ADDR((u), wd_data), (b), (c))
#define	DATA_IN16(u, b, c)	insw(REG_ADDR((u), wd_data), (b), (c))

#define	DATA_OUT8(u, b, c)	outsb(REG_ADDR((u), wd_data), (b), (c))
#define	DATA_OUT16(u, b, c)	outsw(REG_ADDR((u), wd_data), (b), (c))

void
ata_init(int ctlr)
{
}

static int
ata_strategy(struct open_file *f, int flags, daddr_t blk, size_t len,
    void *buf, size_t *residp)
{
	*residp = len;
	return EIO;
}

static int
ata_open(struct open_file *f)
{
	return ENXIO;
}

static int
ata_close(struct open_file *f)
{
	return 0;
}

static int
ata_ioctl(struct open_file *f, u_long cmd, void *data)
{
	return EINVAL;
}

const struct devsw ata_devsw = {
	.dv_name	=	"ata",
	.dv_nargs	=	3,	/* ctlr,unit,part */
	.dv_strategy	=	ata_strategy,
	.dv_open	=	ata_open,
	.dv_close	=	ata_close,
	.dv_ioctl	=	ata_ioctl,
};
