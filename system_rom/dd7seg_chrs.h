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

#ifndef dd7seg_chrs_h_included
#define	dd7seg_chrs_h_included

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
#define	DD7SEG_CHR_0	DD7SEG_G

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
#define	DD7SEG_CHR_1	(~(DD7SEG_B+DD7SEG_C))

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
#define	DD7SEG_CHR_2	(DD7SEG_F+DD7SEG_C)

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
#define	DD7SEG_CHR_3	(DD7SEG_F+DD7SEG_E)

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
#define	DD7SEG_CHR_4	(DD7SEG_A+DD7SEG_D+DD7SEG_E)

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
#define	DD7SEG_CHR_5	(DD7SEG_B+DD7SEG_E)

/*
 *           A
 *      +---------+
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
#define	DD7SEG_CHR_6	DD7SEG_B

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
#define	DD7SEG_CHR_7	(~(DD7SEG_A+DD7SEG_B+DD7SEG_C))

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
#define	DD7SEG_CHR_8	0

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
#define	DD7SEG_CHR_9	(DD7SEG_D+DD7SEG_E)

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
 *      +         +
 *           D
 */
#define	DD7SEG_CHR_A	DD7SEG_D

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
#define	DD7SEG_CHR_B	(DD7SEG_A+DD7SEG_B)

/*
 *           A
 *      +---------+
 *      |          
 *      |          
 *    F |           B
 *      |          
 *      |    G     
 *      +         +
 *      |          
 *      |          
 *    E |           C
 *      |          
 *      |          
 *      +---------+
 *           D
 */
#define	DD7SEG_CHR_C	(DD7SEG_B+DD7SEG_C+DD7SEG_G)

/*
 *           A
 *      +         +
 *                |
 *                |
 *    F           | B
 *                |
 *           G    |
 *      +---------+
 *      |         |
 *      |         |
 *    E |         | C
 *      |         |
 *      |         |
 *      +---------+
 *           D
 */
#define	DD7SEG_CHR_D	(DD7SEG_A+DD7SEG_F)

/*
 *           A
 *      +---------+
 *      |          
 *      |          
 *    F |           B
 *      |          
 *      |    G     
 *      +---------+
 *      |          
 *      |          
 *    E |           C
 *      |          
 *      |          
 *      +---------+
 *           D
 */
#define	DD7SEG_CHR_E	(DD7SEG_B+DD7SEG_C)

/*
 *           A
 *      +---------+
 *      |          
 *      |          
 *    F |           B
 *      |          
 *      |    G     
 *      +---------+
 *      |          
 *      |          
 *    E |           C
 *      |          
 *      |          
 *      +         +
 *           D
 */
#define	DD7SEG_CHR_F	(DD7SEG_B+DD7SEG_C+DD7SEG_D)

#define	DD7SEG_CHR_SPACE 0xf

#endif /* dd7seg_chrs_h_included */
