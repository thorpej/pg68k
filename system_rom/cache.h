/*	From NetBSD: cpu.h,v 1.25 2024/02/28 13:05:39 thorpej Exp	*/

/*
 * Copyright (c) 1988 University of Utah.
 * Copyright (c) 1982, 1990, 1993
 *	The Regents of the University of California.  All rights reserved.
 *
 * This code is derived from software contributed to Berkeley by
 * the Systems Programming Group of the University of Utah Computer
 * Science Department.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. Neither the name of the University nor the names of its contributors
 *    may be used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE REGENTS AND CONTRIBUTORS ``AS IS'' AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE REGENTS OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 *
 * from: Utah $Hdr: cpu.h 1.16 91/03/25$
 *
 *	@(#)cpu.h	8.4 (Berkeley) 1/5/94
 */

#ifndef cache_h_included
#define	cache_h_included

/* fields in the 68020 cache control register */
#define	IC_ENABLE	0x0001	/* enable instruction cache */
#define	IC_FREEZE	0x0002	/* freeze instruction cache */
#define	IC_CE		0x0004	/* clear instruction cache entry */
#define	IC_CLR		0x0008	/* clear entire instruction cache */

/* additional fields in the 68030 cache control register */
#define	IC_BE		0x0010	/* instruction burst enable */
#define	DC_ENABLE	0x0100	/* data cache enable */
#define	DC_FREEZE	0x0200	/* data cache freeze */
#define	DC_CE		0x0400	/* clear data cache entry */
#define	DC_CLR		0x0800	/* clear entire data cache */
#define	DC_BE		0x1000	/* data burst enable */
#define	DC_WA		0x2000	/* write allocate */

/* fields in the 68040 cache control register */
#define	IC40_ENABLE	0x00008000	/* instruction cache enable bit */
#define	DC40_ENABLE	0x80000000	/* data cache enable bit */

/* additional fields in the 68060 cache control register */
#define	DC60_NAD	0x40000000	/* no allocate mode, data cache */
#define	DC60_ESB	0x20000000	/* enable store buffer */
#define	DC60_DPI	0x10000000	/* disable CPUSH invalidation */
#define	DC60_FOC	0x08000000	/* four kB data cache mode (else 8) */

#define	IC60_EBC	0x00800000	/* enable branch cache */
#define IC60_CABC	0x00400000	/* clear all branch cache entries */
#define	IC60_CUBC	0x00200000	/* clear user branch cache entries */

#define	IC60_NAI	0x00004000	/* no allocate mode, instr. cache */
#define	IC60_FIC	0x00002000	/* four kB instr. cache (else 8) */

#define	CACHE_ON	(DC_WA+DC_BE+DC_CLR+DC_ENABLE+IC_BE+IC_CLR+IC_ENABLE)
#define	CACHE_OFF	(DC_CLR+IC_CLR)
#define	CACHE_CLR	(CACHE_ON)
#define	IC_CLEAR	(DC_WA+DC_BE+DC_ENABLE+IC_BE+IC_CLR+IC_ENABLE)
#define	DC_CLEAR	(DC_WA+DC_BE+DC_CLR+DC_ENABLE+IC_BE+IC_ENABLE)

#define	CACHE40_ON	(IC40_ENABLE+DC40_ENABLE)
#define	CACHE40_OFF	(0x00000000)

#define	CACHE60_ON	(CACHE40_ON+IC60_CABC+IC60_EBC+DC60_ESB)
#define	CACHE60_OFF	(CACHE40_OFF+IC60_CABC)

#define CACHELINE_SIZE	16
#define CACHELINE_MASK	(CACHELINE_SIZE - 1)

#endif /* cache_h_included */
