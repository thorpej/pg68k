/*
 * Copyright (c) 2003 Wasabi Systems, Inc.
 * All rights reserved.
 *
 * Written by Steve C. Woodford and Jason R. Thorpe for Wasabi Systems, Inc.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. All advertising materials mentioning features or use of this software
 *    must display the following acknowledgement:
 *      This product includes software developed for the NetBSD Project by
 *      Wasabi Systems, Inc.
 * 4. The name of Wasabi Systems, Inc. may not be used to endorse
 *    or promote products derived from this software without specific prior
 *    written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY WASABI SYSTEMS, INC. ``AS IS'' AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED
 * TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL WASABI SYSTEMS, INC
 * BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include "syslib.h"
#include "i2c.h"

#define	DSRTC_I2C_ADDR		0x68	/* fixed i2c slave address */

#define	DSRTC_SECONDS		0
#define	DSRTC_MINUTES		1
#define	DSRTC_HOURS		2
#define	DSRTC_DAY		3
#define	DSRTC_DATE		4
#define	DSRTC_MONTH		5
#define	DSRTC_YEAR		6

#define	DSRTC_RTC_START		0
#define	DSRTC_RTC_SIZE		7

#define	DSRTC_SECONDS_MASK	0x7f
#define	DSRTC_MINUTES_MASK	0x7f
#define	DSRTC_HOURS_12HRS_MODE	__BIT(6)	/* Set for 12 hour mode */
#define	DSRTC_HOURS_12HRS_PM	__BIT(5)	/* If 12 hr mode, set = PM */
#define	DSRTC_HOURS_12MASK	0x1f
#define	DSRTC_HOURS_24MASK	0x3f
#define	DSRTC_DAY_MASK		0x07
#define	DSRTC_DATE_MASK	0x3f
#define	DSRTC_MONTH_MASK	0x1f
#define	DSRTC_MONTH_CENTURY	0x80

int
clock_type(void)
{
	uint8_t cmdbuf[1], buf[1];
	int error, type = 3231;

	/*
	 * The DS3231 has no NVRAM, so it's useful to know if that's
	 * what we actually have vs a DS3232, which does have NVRAM.
	 *
	 * We can distinguish beteen them as follows:
	 *
	 * On a DS3231, bit 6 of the STATUS register is hard-wired to 0.
	 *
	 * On a DS3232, bit 6 controls battery-backed 32KHz output and
	 * can be toggled to 1.
	 */
	cmdbuf[0] = 0x0f;
	error = i2c_exec(I2C_OP_READ, DSRTC_I2C_ADDR,
	    cmdbuf, sizeof(cmdbuf), buf, sizeof(buf));
	if (error) {
		goto out;
	}
	if (buf[0] & __BIT(6)) {
		/* If it's set, it's for sure a 3232. */
		type = 3232;
		goto out;
	}

	/* Not set, try setting it. */
	buf[0] |= __BIT(6);
	error = i2c_exec(I2C_OP_WRITE, DSRTC_I2C_ADDR,
	    cmdbuf, sizeof(cmdbuf), buf, sizeof(buf));
	if (error) {
		goto out;
	}

	/* Read it back. */
	error = i2c_exec(I2C_OP_READ, DSRTC_I2C_ADDR,
	    cmdbuf, sizeof(cmdbuf), buf, sizeof(buf));
	if (error) {
		goto out;
	}
	if (buf[0] & __BIT(6)) {
		/* It stuck!  It's a 3232. */
		type = 3232;
	}

	/* Set it back to what it was. */
	buf[0] &= ~__BIT(6);
	(void) i2c_exec(I2C_OP_WRITE, DSRTC_I2C_ADDR,
	    cmdbuf, sizeof(cmdbuf), buf, sizeof(buf));

 out:
	return type;
}

int
clock_gettime(struct clock_ymdhms *dt)
{
	uint8_t bcd[DSRTC_RTC_SIZE], cmdbuf[1];
	int error;

	cmdbuf[0] = DSRTC_RTC_START;
	error = i2c_exec(I2C_OP_READ, DSRTC_I2C_ADDR,
	    cmdbuf, sizeof(cmdbuf), bcd, sizeof(bcd));
	if (error) {
		return error;
	}

	dt->dt_sec = bcdtobin(bcd[DSRTC_SECONDS] & DSRTC_SECONDS_MASK);
	dt->dt_min = bcdtobin(bcd[DSRTC_MINUTES] & DSRTC_MINUTES_MASK);

	if ((bcd[DSRTC_HOURS] & DSRTC_HOURS_12HRS_MODE) != 0) {
		dt->dt_hour = bcdtobin(bcd[DSRTC_HOURS] &
		    DSRTC_HOURS_12MASK) % 12; /* 12AM -> 0, 12PM -> 12 */
		if (bcd[DSRTC_HOURS] & DSRTC_HOURS_12HRS_PM) {
			dt->dt_hour += 12;
		}
	} else {
		dt->dt_hour = bcdtobin(bcd[DSRTC_HOURS] &
		    DSRTC_HOURS_24MASK);
	}

	dt->dt_day = bcdtobin(bcd[DSRTC_DATE] & DSRTC_DATE_MASK);
	dt->dt_mon = bcdtobin(bcd[DSRTC_MONTH] & DSRTC_MONTH_MASK);

	dt->dt_year = bcdtobin(bcd[DSRTC_YEAR]) + CONFIG_RTC_EPOCH_YEAR;
	if (bcd[DSRTC_MONTH] & DSRTC_MONTH_CENTURY) {
		dt->dt_year += 100;
	}

	return 0;
}
