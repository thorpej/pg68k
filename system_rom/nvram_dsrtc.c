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

#include "clock.h"
#include "i2c.h"

#define	NVRAM_DS3232_TOTSIZE	236

#define	NVRAM_DATASIZE		\
	(NVRAM_DS3232_TOTSIZE - sizeof(struct nvram_header))

struct nvram_data {
	struct nvram_header hdr;
	uint8_t payload[NVRAM_DATASIZE];
};

static struct nvram_data nvram_data;

#define	DSRTC_I2C_ADDR		0x68	/* fixed i2c slave address */

#define	DS3232_NVRAM_START	0x14

static bool
nvram_present(void)
{
	return clock_type() == 3232;
}

static int
nvram_read(void)
{
	uint8_t cmd[1] = { DS3232_NVRAM_START };
	int error = i2c_exec(I2C_OP_READ, DSRTC_I2C_ADDR,
	    cmd, sizeof(cmd), &nvram_data, sizeof(nvram_data));
	if (error) {
		configure_printf("NVRAM: failed to read RTC, "
				 "using defaults.\n");
	}
	return error;
}

static int
nvram_write(void)
{
	uint8_t cmd[1] = { DS3232_NVRAM_START };
	return i2c_exec(I2C_OP_WRITE, DSRTC_I2C_ADDR,
	    cmd, sizeof(cmd), &nvram_data, sizeof(nvram_data));
}
