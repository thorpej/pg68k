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

/*-
 * Copyright (c) 2003 The NetBSD Foundation, Inc.
 * All rights reserved.
 *
 * This code is derived from software contributed to The NetBSD Foundation
 * by Manuel Bouyer.
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

#include "systypes.h"
#include "syslib.h"
#include "sysfile.h"
#include "pio.h"
#include "clock.h"

#include "ata.h"
#include "atareg.h"
#include "wdc1003reg.h"

#define	REG_ADDR(u, r)		\
	(ata_addrs[((u) << 1)    ] + _REG_OFF(r))
#define	CTLREG_ADDR(u, r)	\
	(ata_addrs[((u) << 1) + 1] + _REG_OFF((r) + wd_ctrl_block_bias))

#define	REG_READ(u, r)		inb(REG_ADDR((u), (r)))
#define	REG_WRITE(u, r, v)	outb(REG_ADDR((u), (r)), (v))

#define	CTLREG_READ(u, r)	inb(CTLREG_ADDR((u), (r)))
#define	CTLREG_WRITE(u, r, v)	outb(CTLREG_ADDR((u), (r)), (v))

#define	DATA_IN8(u, b, c)	insb(REG_ADDR((u), wd_data), (b), (c))
#define	DATA_IN16(u, b, c)	insw(REG_ADDR((u), wd_data), (b), (c))

#define	DATA_OUT8(u, b, c)	outsb(REG_ADDR((u), wd_data), (b), (c))
#define	DATA_OUT16(u, b, c)	outsw(REG_ADDR((u), wd_data), (b), (c))

/*
 * Reset timeout.  We delay ~100us per iteration in the loop,
 * and it might take up to 31s (!!) for the drives to reset.
 */
#define	WDC_RESET_DELAY	100
#define	WDC_RESET_TIMO	(31000000 / WDC_RESET_DELAY)

/*
 * Command timeout.  We deleay ~100us per iteration in the loop,
 * and wait up to 15 seconds.
 */
#define	WDC_CMD_DELAY	100
#define	WDC_CMD_TIMO	(15000000 / WDC_CMD_DELAY)

struct ata_drive {
	uint64_t	drv_nblks;
	int		drv_secsize;
};

struct ata_controller {
	struct ata_drive ata_drives[2];
};

static struct ata_controller ata_controllers[MAXATA];

static struct ataparams ata_identify_buf;

/*
 * Wait for a reset to complete.  The drives we're waiting for are
 * specified by drv_mask, and we clear any bit for a drive that
 * fails to complete the reset cycle.
 */
static int
wdc_wait_reset(int ctlr, int drv_mask)
{
	int timeout;
	uint8_t st0, st1;
	int ready_mask;

	/* sanitize inputs. */
	drv_mask &= __BITS(0,1);
	if (drv_mask == 0) {
		return 0;
	}

	ready_mask = 0;

	/* Wait for BSY to de-assert. */
	for (timeout = 0; timeout < WDC_RESET_TIMO; timeout++) {
		REG_WRITE(ctlr, wd_sdh, WDSD_IBM | WDSD_DRV0);
		clock_delay(10);
		st0 = REG_READ(ctlr, wd_status);

		REG_WRITE(ctlr, wd_sdh, WDSD_IBM | WDSD_DRV1);
		clock_delay(10);
		st1 = REG_READ(ctlr, wd_status);

		if ((drv_mask & __BIT(0)) != 0 &&
		    (st0 & (WDCS_BSY | WDCS_DRDY)) == WDCS_DRDY) {
			ready_mask |= __BIT(0);
		}

		if ((drv_mask & __BIT(1)) != 0 &&
		    (st1 & (WDCS_BSY | WDCS_DRDY)) == WDCS_DRDY) {
			ready_mask |= __BIT(1);
		}

		if (ready_mask == drv_mask) {
			/*
			 * All of the drives we're watching have
			 * completed their reset cycle.
			 */
			break;
		}

		clock_delay(WDC_RESET_DELAY);
	}

	return ready_mask;
}

/*
 * Test to see that at least one drive is attached to the interface.
 * Returns a bit mask of each of the two possible drives found.
 *
 * Logic:
 * ==> If a register is 0xff, assume there is no drive there (the ATA bus
 *     has pull-up resistors).  N.B. the status register cannot be used
 *     for this initial check as the ATA-1 specification states that if
 *     drive 1 is not present, drive 0 will respond with 0x00 for drive 1
 *     status register reads in order to report DRDY=0.
 * ==> Reset the controller, wait for it to complete (may take up to 31s!).
 *     If timeout -> return.
 */
static int
wdc_probe(int ctlr)
{
	int rv = __BITS(0,1);
	uint8_t reg0, reg1;
	uint8_t drive;

	/*
	 * Step 1: Make sure there is something there at all.
	 * We will read the sector register for this initial
	 * probe (see note about status register above).
	 */
	REG_WRITE(ctlr, wd_sdh, WDSD_IBM | WDSD_DRV0);
	clock_delay(10);
	reg0 = REG_READ(ctlr, wd_sector);

	REG_WRITE(ctlr, wd_sdh, WDSD_IBM | WDSD_DRV1);
	clock_delay(10);
	reg1 = REG_READ(ctlr, wd_sector);

	if (reg0 == 0xff) {
		rv &= ~__BIT(0);
	}
	if (reg1 == 0xff) {
		rv &= ~__BIT(1);
	}

	if (rv == 0) {
		return rv;
	}

	/*
	 * Step 2: Issue a software reset.  This will reset both
	 * drives at the same time.  Then wait for the reset to
	 * complete.
	 */
	REG_WRITE(ctlr, wd_sdh, WDSD_IBM | WDSD_DRV0);
	clock_delay(10);
	CTLREG_WRITE(ctlr, wd_aux_control, WDCTL_RST | WDCTL_IDS);
	clock_delay(1000);
	CTLREG_WRITE(ctlr, wd_aux_control, WDCTL_IDS);
	clock_delay(1000);
	CTLREG_WRITE(ctlr, wd_aux_control, WDCTL_4BIT | WDCTL_IDS);
	clock_delay(10);

	return wdc_wait_reset(ctlr, rv);
}

struct wdc_command {
	uint8_t		r_drive;	/* drive ID */

	uint8_t		r_command;	/* command to run */
	uint16_t	r_cyl;
	uint8_t		r_head;
	uint8_t		r_sector;
	uint8_t		r_count;
	uint8_t		r_features;
};

/*
 * Wait until the device is ready.
 */
int
wdc_wait_for_ready(int ctlr)
{
	u_int timo;
	uint8_t st;

	for (timo = WDC_CMD_TIMO; timo > 0; timo--) {
		st = REG_READ(ctlr, wd_status);

		if ((st & (WDCS_BSY | WDCS_DRDY)) == WDCS_DRDY) {
			return 0;
		}
		clock_delay(WDC_CMD_DELAY);
	}
	printf("%s: status=0x%02x\n", __func__, st);
	return EIO;
}

/*
 * Send a command to the device.
 */
static int
wdc_exec_command(int ctlr, const struct wdc_command *cmd)
{

	REG_WRITE(ctlr, wd_features, cmd->r_features);
	REG_WRITE(ctlr, wd_seccnt,   cmd->r_count);
	REG_WRITE(ctlr, wd_sector,   cmd->r_sector);
	REG_WRITE(ctlr, wd_cyl_lo,   cmd->r_cyl);
	REG_WRITE(ctlr, wd_cyl_hi,   cmd->r_cyl >> 8);
	REG_WRITE(ctlr, wd_sdh,
	    WDSD_IBM | (cmd->r_drive << 4) | cmd->r_head);
	REG_WRITE(ctlr, wd_command,  cmd->r_command);

	if (wdc_wait_for_ready(ctlr) != 0) {
		printf("ata(%d,%d): command timed out\n",
		    ctlr, cmd->r_drive);
		return ENXIO;
	}

	if (REG_READ(ctlr, wd_status) & WDCS_ERR) {
		printf("ata(%d,%d): error 0x%02x\n", ctlr, cmd->r_drive,
		    REG_READ(ctlr, wd_error));
		return EIO;
	}

	return 0;
}

static void
wdc_read_data(int ctlr, void *buf, u_int count)
{
	DATA_IN8(ctlr, buf, count);
}

static void
wdc_read_sector(int ctlr, void *buf)
{
	wdc_read_data(ctlr, buf, 512);
}

static int
wdc_cmd_set_feature(int ctlr, int drive, uint8_t feat)
{
	struct wdc_command cmd = {
		.r_drive = drive,
		.r_features = feat,
		.r_command = SET_FEATURES,
	};

	return wdc_exec_command(ctlr, &cmd);
}

static int
wdc_cmd_identify_drive(int ctlr, int drive, void *buf)
{
	struct wdc_command cmd = {
		.r_drive = drive,
		.r_command = WDCC_IDENTIFY,
	};
	int error;

	error = wdc_exec_command(ctlr, &cmd);
	if (error) {
		return error;
	}
	wdc_read_sector(ctlr, buf);
	return 0;
}

static int
wdc_cmd_read(int ctlr, int drive, uint32_t lba, void *buf)
{
	struct wdc_command cmd = {
		.r_drive = drive,
		.r_count = 1,
		.r_command = WDCC_READ,
	};
	int error;

	cmd.r_sector = (uint8_t)  lba;
	cmd.r_cyl    = (uint16_t)(lba >> 8);
	cmd.r_head   = (uint8_t)((lba >> 24) & 0x0f) | WDSD_LBA;

	error = wdc_exec_command(ctlr, &cmd);
	if (error) {
		return error;
	}
	wdc_read_sector(ctlr, buf);
	return 0;
}

static int
wdc_cmd_write(int ctlr, int drive, uint32_t lba, const void *buf)
{
	return EIO;
}

static const char *
wdc_decode_identify_string(const char *in, size_t insize, char *out)
{
	int i, blank;
	char c, *q;

	for (blank = 0, q = out, i = 0; i < insize; i++) {
		c = *in++;
		if (c == '\0') {
			break;
		}
		if (c != ' ') {
			if (blank) {
				*q++ = ' ';
				blank = 0;
			}
			*q++ = c;
		} else {
			blank = 1;
		}
	}
	*q++ = '\0';
	return out;
}

void
ata_init(int ctlr)
{
	struct ataparams *atap = &ata_identify_buf;
	int rv, drive, error;
	char model[sizeof(atap->atap_model) + 1];
	uint16_t u16;
	uint32_t capacity;
	struct ata_drive *drv;

	rv = wdc_probe(ctlr);
#if 0
	printf("ata_init(%d): result -> 0x%02x\n", ctlr, rv);
#endif

	for (drive = 0; drive < 2; drive++) {
		if ((rv & __BIT(drive)) == 0) {
			continue;
		}
#if 1 /* XXX */
		error = wdc_cmd_set_feature(ctlr, drive, WDSF_8BIT_PIO_EN);
		if (error) {
			continue;
		}
#endif
		error = wdc_cmd_identify_drive(ctlr, drive, atap);
		if (error) {
			continue;
		}
		u16 = le16toh(atap->atap_capabilities1);
		if ((u16 & WDC_CAP_LBA) == 0) {
			configure_printf("  drive %d: no LBA capability, "
					 "ignoring\n", drive);
			continue;
		}

		capacity = le16toh(atap->atap_capacity[1]);
		capacity = (capacity << 16) | le16toh(atap->atap_capacity[0]);

		drv = &ata_controllers[ctlr].ata_drives[drive];
		drv->drv_secsize = 512;
		drv->drv_nblks = capacity;

		configure_printf("  drive %d: <%s> %llu %d-byte blocks\n",
		    drive,
		    wdc_decode_identify_string(atap->atap_model,
					       sizeof(atap->atap_model),
					       model),
		    drv->drv_nblks, drv->drv_secsize);
	}
}

static int
ata_strategy(struct open_file *f, int flags, daddr_t blk, size_t len,
    void *buf, size_t *actualp)
{
	struct ata_drive *drv = f->f_devdata;
	size_t blkcnt;
	ssize_t actual = 0;
	int error;

	*actualp = 0;

	if (len % drv->drv_secsize) {
		return EINVAL;
	}
	blkcnt = len / drv->drv_secsize;

	if (blk >= drv->drv_nblks) {
		return EIO;
	}

	if ((blk + blkcnt) > drv->drv_nblks) {
		blkcnt -= (blk + blkcnt) - drv->drv_nblks;
	}

	if (flags == 1/*XXX FREAD*/) {
		for (char *cp = buf; blkcnt != 0;
		     cp += drv->drv_secsize, actual += drv->drv_secsize,
		     blk++, blkcnt--) {
			error = wdc_cmd_read(f->f_devctlr, f->f_devunit,
			    blk, cp);
			if (error) {
				goto out;
			}
		}
	} else {
		for (const char *cp = buf; blkcnt != 0;
		     cp += drv->drv_secsize, actual += drv->drv_secsize,
		     blk++, blkcnt--) {
			error = wdc_cmd_write(f->f_devctlr, f->f_devunit,
			    blk, cp);
			if (error) {
				goto out;
			}
		}
	}
 out:
	*actualp = actual;
	return error;
}

static int
ata_open(struct open_file *f)
{
	struct ata_drive *drv;

	if (f->f_devctlr >= MAXATA) {
		return ENXIO;
	}

	if (f->f_devunit < 0 || f->f_devunit > 1) {
		return ENXIO;
	}

	drv = &ata_controllers[f->f_devctlr].ata_drives[f->f_devunit];
	if (drv->drv_secsize == 0) {
		return ENXIO;
	}

	f->f_devdata = drv;

	return 0;
}

static int
ata_close(struct open_file *f)
{
	f->f_devdata = NULL;
	return 0;
}

static int
ata_ioctl(struct open_file *f, u_long cmd, void *data)
{
	return EINVAL;
}
