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
#include "pio.h"

#include "ns16550.h"

#define	REG_READ(u, r)		inb(uart_addrs[(u)] + _REG_OFF(r))
#define	REG_WRITE(u, r, v)	outb(uart_addrs[(u)] + _REG_OFF(r), (v))

#define	divrnd(n, q)		(((n)*2/(q)+1)/2)

static int
ns16550_speed(int speed)
{
	int x, err;

	if (speed <= 0) {
		return -1;
	}
	x = divrnd((CONFIG_UART_FREQ / 16), speed);
	if (x <= 0) {
		return -1;
	}
	err =
	  divrnd((((uint64_t)CONFIG_UART_FREQ) / 16) * 1000, speed * x) - 1000;
	if (err < 0) {
		err = -err;
	}
	if (err > COM_TOLERANCE) {
		return -1;
	}
	return x;
}

void
uart_init(int unit, int speed)
{
	int rate = ns16550_speed(speed);

	REG_WRITE(unit, com_lcr, LCR_DLAB);
	REG_WRITE(unit, com_dlbl, rate);
	REG_WRITE(unit, com_dlbh, rate >> 8);
	REG_WRITE(unit, com_lcr, LCR_8BITS);
	REG_WRITE(unit, com_mcr, MCR_DTR | MCR_RTS);
	REG_WRITE(unit, com_fifo,
	    FIFO_ENABLE | FIFO_RCV_RST | FIFO_XMT_RST | FIFO_TRIGGER_1);
	REG_WRITE(unit, com_ier, 0);
}

bool
uart_pollc(int unit, int *chp)
{
	if (REG_READ(unit, com_lsr) & LSR_RXRDY) {
		*chp = REG_READ(unit, com_data);
		return true;
	}
	return false;
}

int
uart_getc(int unit)
{
	while ((REG_READ(unit, com_lsr) & LSR_RXRDY) == 0) {
		/* spin */;
	}
	return REG_READ(unit, com_data);
}

static void
ns16550_txwait(int unit)
{
	while ((REG_READ(unit, com_lsr) & LSR_TXRDY) == 0) {
		/* spin */;
	}
}

void
uart_putc(int unit, int c)
{
	ns16550_txwait(unit);
	REG_WRITE(unit, com_data, c);
}
