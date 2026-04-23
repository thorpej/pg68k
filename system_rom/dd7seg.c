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
#include "control.h"

static const unsigned char digits[] = {
/*
 *           A
 *      +---------+
 *      |         |
 *      |         |
 *    F |         | B
 *      |         |
 *      |    G    |
 *      +         +
 *      |         |
 *      |         |
 *    E |         | C
 *      |         |
 *      |         |
 *      +---------+
 *           D
 */
[0]	= DD7SEG_G,
/*
 *           A
 *      +         +
 *                |
 *                |
 *    F           | B
 *                |
 *           G    |
 *      +         +
 *                |
 *                |
 *    E           | C
 *                |
 *                |
 *      +         +
 *           D
 */
[1]	= ~(DD7SEG_B|DD7SEG_C),
/*
 *           A
 *      +---------+
 *                |
 *                |
 *    F           | B
 *                |
 *           G    |
 *      +---------+
 *      |          
 *      |          
 *    E |           C
 *      |          
 *      |          
 *      +---------+
 *           D
 */
[2]	= DD7SEG_F|DD7SEG_C,
/*
 *           A
 *      +---------+
 *                |
 *                |
 *    F           | B
 *                |
 *           G    |
 *      +---------+
 *                |
 *                |
 *    E           | C
 *                |
 *                |
 *      +---------+
 *           D
 */
[3]	= DD7SEG_F|DD7SEG_E,
/*
 *           A
 *      +         +
 *      |         |
 *      |         |
 *    F |         | B
 *      |         |
 *      |    G    |
 *      +---------+
 *                |
 *                |
 *    E           | C
 *                |
 *                |
 *      +         +
 *           D
 */
[4]	= DD7SEG_A|DD7SEG_D|DD7SEG_E,
/*
 *           A
 *      +---------+
 *      |          
 *      |          
 *    F |           B
 *      |          
 *      |    G     
 *      +---------+
 *                |
 *                |
 *    E           | C
 *                |
 *                |
 *      +---------+
 *           D
 */
[5]	= DD7SEG_B|DD7SEG_E,
/*
 *           A
 *      +         +
 *      |          
 *      |          
 *    F |           B
 *      |          
 *      |    G     
 *      +---------+
 *      |         |
 *      |         |
 *    E |         | C
 *      |         |
 *      |         |
 *      +---------+
 *           D
 */
[6]	= DD7SEG_A|DD7SEG_B,
/*
 *           A
 *      +---------+
 *                |
 *                |
 *    F           | B
 *                |
 *           G    |
 *      +         +
 *                |
 *                |
 *    E           | C
 *                |
 *                |
 *      +         +
 *           D
 */
[7]	= ~(DD7SEG_A|DD7SEG_B|DD7SEG_C),
/*
 *           A
 *      +---------+
 *      |         |
 *      |         |
 *    F |         | B
 *      |         |
 *      |    G    |
 *      +---------+
 *      |         |
 *      |         |
 *    E |         | C
 *      |         |
 *      |         |
 *      +---------+
 *           D
 */
[8]	= 0,
/*
 *           A
 *      +---------+
 *      |         |
 *      |         |
 *    F |         | B
 *      |         |
 *      |    G    |
 *      +---------+
 *                |
 *                |
 *    E           | C
 *                |
 *                |
 *      +         +
 *           D
 */
[9]	= DD7SEG_D|DD7SEG_E,
/*
 * The remainder are blank spaces.
 */
[10]	= 0xff,
[11]	= 0xff,
[12]	= 0xff,
[13]	= 0xff,
[14]	= 0xff,
[15]	= 0xff,
};

void
dd7seg(unsigned char bcd)
{
	unsigned short val;

	val = ((int)digits[bcd >> 4] << 8) | digits[bcd & 0xf];
	control_outw(CTLREG_DD7SEG, val);
}
