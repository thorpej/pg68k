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

#include "clock.h"

#include "pio.h"
#include "psl.h"

static volatile uint64_t clock_ticks;

#define	REG_READ(r)		inb(TIMER_ADDR + _REG_OFF(r))
#define	REG_WRITE(r, v)		outb(TIMER_ADDR + _REG_OFF(r), (v))

#define	TIMER_CSR	0
#define	  CSR_ENAB	0x01
#define	  CSR_IPEND	0x02
#define	TIMER_LSB	1
#define	TIMER_MSB	2

static uint32_t
pgtimer_us_to_ticks(unsigned int interval_us)
{
	const uint32_t tick_ns = 1000000000 / CONFIG_PGTIMER_FREQ;
	const uint64_t interval_ns = interval_us * 1000;
	uint64_t ticks = interval_ns / tick_ns;

	if (ticks == 0 || ticks > 0xffff) {
		printf("%s: impossible interval %u us\n", __func__,
		    interval_us);
		/* limp along... */
		if (ticks == 0) {
			ticks = 1;
		} else {
			ticks = 0xffff;
		}
	}

	return (uint32_t)ticks;
}

static void
pgtimer_initclock(unsigned int interval_us)
{
	const uint32_t ticks = pgtimer_us_to_ticks(interval_us);

	/*
	 * Changing the counter reload value implicitly disables the
	 * timer.  The reload value is unpredictable until both bytes
	 * have been written.  The value registers can be written in
	 * any order.
	 */
	REG_WRITE(TIMER_LSB, (uint8_t)ticks);
	REG_WRITE(TIMER_MSB, (uint8_t)(ticks >> 8));
	REG_WRITE(TIMER_CSR, CSR_ENAB);
}

static void
pgtimer_delaycal(void)
{
	const uint32_t ticks = pgtimer_us_to_ticks(10000);
	int s;

	/*
	 * Algorithm: We start with the initial divisor.  We program
	 * the timer to fire after 10000us (10ms), and then call delay
	 * for that long.  If, after the delay, we check to see if an
	 * interrupt is pending.  If so, we're done!  If not, we decrement
	 * the divisor and try again.
	 *
	 * All this with interrupts blocked, obviously.  This will be
	 * called before the system timer is initialized.
	 */

	s = splhigh();
	for (; delay_divisor > 0; delay_divisor--) {
		REG_WRITE(TIMER_LSB, (uint8_t)ticks);
		REG_WRITE(TIMER_MSB, (uint8_t)(ticks >> 8));
		REG_WRITE(TIMER_CSR, CSR_ENAB);
		clock_delay(10000);
		if (REG_READ(TIMER_CSR) & CSR_IPEND) {
			/* Got it! */
			break;
		}
	}
	REG_WRITE(TIMER_CSR, 0);
	splx(s);
}

static void
pgtimer_intr(void *arg)
{
	clock_ticks++;
}

#ifndef CONFIG_HZ
#define	CONFIG_HZ	100
#endif
static int hz = CONFIG_HZ;

void
clock_configure(void)
{
	int s;

	printf("timer0 at 0x%08lx\n", (u_long)TIMER_ADDR);

	/* Calibrate the delay divisor. */
	pgtimer_delaycal();
	printf("  delay divisor: %u\n", delay_divisor);

	/* Register the timer interrupt. */
	// intr_establish(6, 0, pgtimer_intr);	XXX

	/* Start the timer. */
	s = splhigh();
	if (1000000 % hz) {
		printf("Cannot get %d Hz clock; using 100 Hz\n", hz);
		hz = 100;
	}
	pgtimer_initclock(1000000 / hz);

	/*
	 * Drop to IPL5 to allow timer interrupts, but block
	 * everything else.
	 */
	_spl(PSL_S|PSL_IPL5);
}

void
clock_quiesce(void)
{
	splhigh();		/* block all interrupts again */
	REG_WRITE(TIMER_CSR, 0);/* stop the timer */
}

/* clock_delay() - see clock.h */

time_t
clock_getsecs(void)
{
	uint64_t tickread;
	int s;

	/*
	 * The counter is incremented in the timer interrupt handler.
	 * Even though a 64-bit increment isn't atomic on m68k, all
	 * we need is to block the timer interrupt while we read it.
	 */
	s = splhigh();
	tickread = clock_ticks;
	splx(s);

	return (time_t)(clock_ticks / hz);
}
