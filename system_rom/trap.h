/*	From NetBSD: cpuframe.h,v 1.10 2023/09/26 14:33:55 tsutsui Exp	*/
/*	From NetBSD: trap.h,v 1.12 2024/10/16 06:54:55 isaki Exp	*/

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
 * from: Utah $Hdr: frame.h 1.8 92/12/20$
 *
 *	@(#)frame.h	8.1 (Berkeley) 6/10/93
 */

#ifndef trap_h_included
#define	trap_h_included

#include "config.h"
#include "systypes.h"

#if !defined(CONFIG_MACH_HOST_SIM) && !defined(__ASSEMBLER__)

struct trap_frame {
	u_int	tf_regs[16];
	/* hardware frame */
	u_short	tf_sr;
	u_int	tf_pc;
	u_short	/* BITFIELDTYPE */ tf_format:4,
		/* BITFIELDTYPE */ tf_vector:12;
} __attribute__((__packed__));

/*
 * What follows the trap_frame depends on tf_format.
 *
 * 0	-->	Nothing! (4 total words)
 * 8	-->	Bus error / Address error frame (29 total words)
 */

struct trap_frame_ext8 {
	u_short	tf_ssw;		/* special status word */
	u_int	tf_faultaddr;	/* fault address */
	u_short	tf__rsvd0;
	u_short	tf_dob;		/* data output buffer */
	u_short	tf__rsvd1;
	u_short	tf_dib;		/* data input buffer */
	u_short	tf__rsvd2;
	u_short	tf_iib;		/* instruction input buffer */
	u_short	tf_ii[16];	/* internal information */
} __attribute__((__packed__));

#define	SSW_FC		__BITS(0,2)	/* function code for faulting access */
#define	SSW_RW		__BIT(8)	/* 1=read, 0=write */
#define	SSW_BY		__BIT(9)	/* 1=byte, 0=word */
#define	SSW_HB		__BIT(10)	/* 1=high byte, 0=low byte (BY=1) */
#define	SSW_RM		__BIT(11)	/* read/mod/write cycle */
#define	SSW_DF		__BIT(12)	/* data fetch to DIB */
#define	SSW_IF		__BIT(13)	/* insn fetch to IIB */
#define	SSW_RR		__BIT(15)	/* 0=proc rerun, 1=sw rerun */

#define	trap_frame_ext(t)						\
	((void *)(((uintptr_t)(t)) + sizeof(struct trap_frame)))

#endif /* ! CONFIG_MACH_HOST_SIM && ! __ASSEMBLER__ */

#ifndef __ASSEMBLER__
extern bool	nofault;
extern jmp_buf	nofault_env;

bool	badaddr_read8(volatile uint8_t *, uint8_t *);
bool	badaddr_read16(volatile uint16_t *, uint16_t *);
bool	badaddr_read32(volatile uint32_t *, uint32_t *);

bool	badaddr_write32(volatile uint32_t *, uint32_t);

#ifndef CONFIG_MACH_HOST_SIM
void	trap_handler_buserr(struct trap_frame);
#endif /* ! CONFIG_MACH_HOST_SIM */
#endif /* ! __ASSEMBLER__ */

#endif	/* trap_h_included */
