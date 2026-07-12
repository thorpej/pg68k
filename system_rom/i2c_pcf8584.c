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

#include "systypes.h"
#include "syslib.h"
#include "clock.h"
#include "pio.h"

#include "i2c.h"
#include "pcf8584reg.h"

#define	REG_READ(r)	inb(I2C_ADDR + _REG_OFF(r))
#define	REG_WRITE(r, v)	outb(I2C_ADDR + _REG_OFF(r), (v))

#if CONFIG_PCF8584_FREQ == 12000000
#define	CLK_VAL		PCF8584_CLK_12
#elif CONFIG_PCF8584_FREQ == 8000000
#define	CLK_VAL		PCF8584_CLK_8
#elif CONFIG_PCF8584_FREQ == 6000000
#define	CLK_VAL		PCF8584_CLK_6
#elif CONFIG_PCF8584_FREQ == 4430000
#define	CLK_VAL		PCF8584_CLK_4_43
#elif CONFIG_PCF8584_FREQ == 3000000
#define	CLK_VAL		PCF8584_CLK_3
#else
#error Invalid PCF8584 clock frequency.
#endif

#define	I2C_TIMEOUT_SEC		5	/* possibly excessive */

void
i2c_init(bool do_init)
{

	if (! do_init) {
		return;
	}

	/*
	 * Follows "Fig 5 - PCF8584 initialization sequence" from
	 * the PCF8584 data sheet.
	 */
	REG_WRITE(1, PCF8584_CTRL_PIN | PCF8584_REG_S0_);
	REG_WRITE(0, 0);	/* own-address == 0x00 */

	REG_WRITE(1, PCF8584_CTRL_PIN | PCF8584_REG_S2);
	REG_WRITE(0, CLK_VAL | PCF8584_SCL_90);

	REG_WRITE(1, PCF8584_CTRL_PIN | PCF8584_CTRL_ESO | PCF8584_CTRL_ACK);
}

static int
i2c_wait_pin(uint8_t *valp)
{
	time_t deadline = clock_getsecs() + I2C_TIMEOUT_SEC;
	uint8_t val;

	while (clock_getsecs() < deadline) {
		if (((val = REG_READ(1)) & PCF8584_STATUS_PIN) == 0) {
			if (valp != NULL) {
				*valp = val;
			}
			return 0;
		}
	}
	return ETIMEDOUT;
}

static int
i2c_wait_bbn(void)
{
	time_t deadline = clock_getsecs() + I2C_TIMEOUT_SEC;

	while (clock_getsecs() < deadline) {
		if (REG_READ(1) & PCF8584_STATUS_BBN) {
			return 0;
		}
	}
	printf("i2c: Bus Busy timeout\n");
	return ETIMEDOUT;
}

static int
i2c_sendbytes(const void *buf, size_t len)
{
	const uint8_t *cp = buf;
	int error;
	uint8_t st;

	for (;;) {
		error = i2c_wait_pin(&st);
		if (error) {
			return error;
		}
		if (st & PCF8584_STATUS_LRB) {
			/* NACK */
			return EIO;
		}
		if (len == 0) {
			/* transmission complete */
			return 0;
		}
		REG_WRITE(0, *cp);
		cp++;
		len--;
	}
}

static int
i2c_recvbytes(void *buf, size_t len)
{
	uint8_t *cp = buf;
	size_t i;
	int error;
	uint8_t st, val;

	for (i = 0; i <= len; i++) {
		error = i2c_wait_pin(&st);
		if (error) {
			REG_WRITE(1, PCF8584_CMD_STOP);
			return error;
		}
		if (i < len && (st & PCF8584_STATUS_LRB) != 0) {
			/* NACK */
			REG_WRITE(1, PCF8584_CMD_STOP);
			REG_READ(0);
			return EIO;
		}
		if (i == len - 1) {
			REG_WRITE(1, PCF8584_CMD_NAK);
		} else if (i == len) {
			REG_WRITE(1, PCF8584_CMD_STOP);
		}
		/* first read is dummy read */
		val = REG_READ(0);
		if (i > 0) {
			cp[i - 1] = val;
		}
	}
	return 0;
}

int
i2c_exec(i2c_op_t op, i2c_addr_t addr,
    const void *cmdbuf, size_t cmdlen, void *buf, size_t len)
{
	int error;

	error = i2c_wait_bbn();
	if (error) {
		return error;
	}

	/*
	 * If we are doing a write op or sending a command buffer
	 * for a read op, start with a send operation.
	 */
	if (op == I2C_OP_WRITE || cmdlen != 0) {
		REG_WRITE(0, addr << 1);
		REG_WRITE(1, PCF8584_CMD_START);
		if (cmdlen != 0) {
			error = i2c_sendbytes(cmdbuf, cmdlen);
		}
		if (op == I2C_OP_WRITE && len != 0 && error == 0) {
			error = i2c_sendbytes(buf, len);
		}
		/* If an error or a write or no data length, we're done. */
		if (error != 0 || op == I2C_OP_WRITE || len == 0) {
			REG_WRITE(1, PCF8584_CMD_STOP);
			return error;
		}
	}

	/*
	 * If we sent a command, then send a REPEATED START,
	 * otherwise just send a START.
	 */
	if (cmdlen != 0) {
		REG_WRITE(1, PCF8584_CMD_REPSTART);
		REG_WRITE(0, (addr << 1) | 1);
	} else {
		error = i2c_wait_bbn();
		if (error) {
			return error;
		}
		REG_WRITE(0, (addr << 1) | 1);
		REG_WRITE(1, PCF8584_CMD_START);
	}

	/* i2c_recvbytes() always sets STOP condition. */
	return i2c_recvbytes(buf, len);
}
