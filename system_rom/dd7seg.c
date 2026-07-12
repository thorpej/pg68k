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

#include "dd7seg.h"
#include "dd7seg_chrs.h"
#include "control.h"

static const uint8_t hex_digits[] = {
[0]	= DD7SEG_CHR_0,
[1]	= DD7SEG_CHR_1,
[2]	= DD7SEG_CHR_2,
[3]	= DD7SEG_CHR_3,
[4]	= DD7SEG_CHR_4,
[5]	= DD7SEG_CHR_5,
[6]	= DD7SEG_CHR_6,
[7]	= DD7SEG_CHR_7,
[8]	= DD7SEG_CHR_8,
[9]	= DD7SEG_CHR_9,
[10]	= DD7SEG_CHR_A,
[11]	= DD7SEG_CHR_B,
[12]	= DD7SEG_CHR_C,
[13]	= DD7SEG_CHR_D,
[14]	= DD7SEG_CHR_E,
[15]	= DD7SEG_CHR_F,
};

void
dd7seg_hex(uint8_t hex)
{
	unsigned short val;

	val = ((int)hex_digits[hex >> 4] << 8) | hex_digits[hex & 0xf];
	control_outw(CTLREG_DD7SEG, val);
}

void
dd7seg_blank(void)
{
	control_outw(CTLREG_DD7SEG, (DD7SEG_CHR_SPACE << 8) | DD7SEG_CHR_SPACE);
}

static int
dd7seg_nybble(uint8_t val)
{
	if (val >= 0 && val <= 15) {
		return hex_digits[val];
	}

	if (val >= '0' && val <= '9') {
		return hex_digits[val - '0'];
	}

	if (val >= 'A' && val <= 'F') {
		return hex_digits[val - 'A'];
	}

	if (val >= 'a' && val <= 'f') {
		return hex_digits[val - 'a'];
	}

	/* default to a blank */
	return DD7SEG_CHR_SPACE;
}

void
dd7seg_digits(uint8_t upper, uint8_t lower)
{
	unsigned short val;

	val = (dd7seg_nybble(upper) << 8) | dd7seg_nybble(lower);
	control_outw(CTLREG_DD7SEG, val);
}
