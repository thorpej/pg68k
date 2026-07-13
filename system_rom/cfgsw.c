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

#include "config.h"

#include "syslib.h"
#include "control.h"
#include "cfgsw.h"

#define	CFGSW_IDX(x)	((x) - 1)		/* switches are 1-N */
#define	CFGSW_BIT(x)	__BIT(CFGSW_IDX(x))

#ifdef CONFIG_MACH_PG68010_MK_I
#define	NUM_CFGSW	16
#define	CFGSW_AUTOBOOT	16
#define	CFGSW_VERBOSE	15
#endif

static const char * const cfgsw_descriptions[NUM_CFGSW] = {
#ifdef CFGSW_AUTOBOOT
[CFGSW_IDX(CFGSW_AUTOBOOT)]	 	= "auto-boot",
#endif
#ifdef CFGSW_VERBOSE
[CFGSW_IDX(CFGSW_VERBOSE)]		= "verbose",
#endif
};

static const char *
cfgsw_description(unsigned int sw)
{
	if (sw >= 1 && sw <= NUM_CFGSW &&
	    cfgsw_descriptions[CFGSW_IDX(sw)] != NULL) {
		return cfgsw_descriptions[CFGSW_IDX(sw)];
	}
	return "<undefined>";
}

bool
cfwsw_set_p(unsigned int sw)	/* 1...N */
{
#ifndef NUM_CFGSW
	return false;
#elif NUM_CFGSW <= 8
	uint8_t val = control_inb(CTLREG_CFGSW);
#elif NUM_CFGSW <= 16
	uint16_t val = control_inw(CTLREG_CFGSW);
#else
#error Invalid config switch configuration.
#endif

	if (sw >= 1 && sw <= NUM_CFGSW) {
		return (val & CFGSW_BIT(sw)) != 0;
	}
	return false;
}

bool
cfgsw_autoboot_p(void)
{
#ifdef CFGSW_AUTOBOOT
	return cfwsw_set_p(CFGSW_AUTOBOOT);
#else
	return false;
#endif
}

bool
cfgsw_verbose_p(void)
{
#ifdef CFGSW_VERBOSE
	return cfwsw_set_p(CFGSW_VERBOSE);
#else
	return false;
#endif
}

#if NUM_CFGSW <= 8
#define	SWNO_FORMAT	"%u"
#else
#define	SWNO_FORMAT	"%2u"
#endif

void
cfgsw_print(unsigned int sw)	/* 0 == all */
{
#ifdef NUM_CFGSW
	unsigned int i;

	for (i = NUM_CFGSW; i > 0; i--) {
		if (sw != 0 && sw != i) {
			continue;
		}
		printf(SWNO_FORMAT ": %-30s %s\n",
		    i, cfgsw_description(i), cfwsw_set_p(i) ? "ON" : "OFF");
	}
#endif
}
