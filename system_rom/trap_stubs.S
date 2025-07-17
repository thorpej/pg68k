|	From: NetBSD: trap_subr.s,v 1.17 2024/10/31 07:30:28 isaki Exp
|
|
| Copyright (c) 1988 University of Utah.
| Copyright (c) 1980, 1990, 1993
|	The Regents of the University of California.  All rights reserved.
|
| This code is derived from software contributed to Berkeley by
| the Systems Programming Group of the University of Utah Computer
| Science Department.
|
| Redistribution and use in source and binary forms, with or without
| modification, are permitted provided that the following conditions
| are met:
| 1. Redistributions of source code must retain the above copyright
|    notice, this list of conditions and the following disclaimer.
| 2. Redistributions in binary form must reproduce the above copyright
|    notice, this list of conditions and the following disclaimer in the
|    documentation and/or other materials provided with the distribution.
| 3. Neither the name of the University nor the names of its contributors
|    may be used to endorse or promote products derived from this software
|    without specific prior written permission.
|
| THIS SOFTWARE IS PROVIDED BY THE REGENTS AND CONTRIBUTORS ``AS IS'' AND
| ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
| IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
| ARE DISCLAIMED.  IN NO EVENT SHALL THE REGENTS OR CONTRIBUTORS BE LIABLE
| FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
| DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
| OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
| HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
| LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
| OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
| SUCH DAMAGE.
|
| from: Utah $Hdr: locore.s 1.66 92/12/22$
|
|	@(#)locore.s	8.6 (Berkeley) 5/27/94
|

|
| Common fault handling code.  Called by exception vector handlers.
| Registers have been saved, and type for trap() placed in d0.  Upon
| return, it will fix up the stack, if necessary.
|
fault:
	move.l	%usp,%a0		| get and save
	move.l	%a0,FR_SP(%sp)		|   the user stack pointer
	clr.l	-(%sp)			| no VA arg
	clr.l	-(%sp)			| or code arg
	move.l	%d0,-(%sp)		| push trap type
	pea	12(%sp)			| address of trap frame
	jbsr	_C_LABEL(trap)		| handle trap
	lea	16(%sp),%sp		| pop value args
	move.l	FR_SP(%sp),%a0		| restore
	move.l	%a0,%usp		|   user SP
	movem.l	(%sp)+,#0x7FFF		| restore most user regs
	addq.l	#8,%sp			| pop SP and stack adjust
	rte				| all done

|
| Similar to above, but will tidy up the stack, if necessary.
|
faultstakadj:
	pea	12(%sp)			| address of trap frame
	jbsr	_C_LABEL(trap)		| handle the error
	lea	16(%sp),%sp		| pop value args
| for new 68060 Branch Prediction Error handler
faultstkadjnotrap2:
	move.l	FR_SP(%sp),%a0		| restore user SP
	move.l	%a0,%usp		|   from save area
	move.w	FR_ADJ(%sp),%d0		| need to adjust stack?
	jne	1f			| yes, go to it
	movem.l	(%sp)+,#0x7FFF		| no, restore most user regs
	addq.l	#8,%sp			| toss SSP and stkadj
	rte				| all done
1:
	lea	FR_HW(%sp),%a1		| pointer to HW frame
	addq.l	#8,%a1			| source pointer
	move.l	%a1,%a0			| source
	addw	%d0,%a0			|  + hole size = dest pointer
	move.l	-(%a1),-(%a0)		| copy
	move.l	-(%a1),-(%a0)		|  8 bytes
	move.l	%a0,FR_SP(%sp)		| new SSP
	movem.l	(%sp)+,#0x7FFF		| restore user registers
	move.l	(%sp),%sp		| and our SP
	rte				| all done

|
| The following exceptions only cause four and six word stack frames
| and require no post-trap stack adjustment.
|
	.globl	trap_illinst
trap_illinst:
	clr.l	-(%sp)
	movem.l	#0xFFFF,-(%sp)
	moveq	#T_ILLINST,%d0
	jra	fault

	.globl	trap_zerodiv
trap_zerodiv:
	clr.l	-(%sp)
	movem.l	#0xFFFF,-(%sp)
	moveq	#T_ZERODIV,%d0
	jra	fault

	.globl	trap_chk
trap_chk:
	clr.l	-(%sp)
	movem.l	#0xFFFF,-(%sp)
	moveq	#T_CHKINST,%d0
	jra	fault

	.globl	trap_trapv
trap_trapv:
	clr.l	-(%sp)
	movem.l	#0xFFFF,-(%sp)
	moveq	#T_TRAPVINST,%d0
	jra	fault

	.globl	trap_priv
trap_priv:
	clr.l	-(%sp)
	movem.l	#0xFFFF,-(%sp)
	moveq	#T_PRIVINST,%d0
	jra	fault

	.globl	trap_trace
trap_trace:
	clr.l	-(%sp)
	movem.l	#0xFFFF,-(%sp)
	moveq	#T_TRACE,%d0
	jra	fault

|
| FP exceptions
| XXX Will require work for 68040/68060.
|
	.globl	trap_fpfline
trap_fpfline:
	clr.l	-(%sp)
	movem.l	#0xFFFF,-(%sp)
	moveq	#T_FPEMULI,%d0
	jra	fault

	.globl	trap_fpunsupp
trap_fpunsupp:
	clr.l	-(%sp)
	movem.l	#0xFFFF,-(%sp)
	moveq	#T_FPEMULD,%d0
	jra	fault

|
| Coprocessor and format errors can generate mid-instruction stack
| hence we need to check for potential stack adjustment.
|

	.globl	trap_cpv
trap_cpv:
	clr.l	-(%sp)			| stack adjust count
	movem.l	#0xFFFF,-(%sp)
	move.l	%usp,%a0		| get and save
	move.l	%a0,FR_SP(%sp)		|   the user stack pointer
	clr.l	-(%sp)			| no VA arg
	clr.l	-(%sp)			| or code arg
	move.l	#T_COPERR,-(%sp)	| push trap type
	jra	faultstkadj		| call trap and deal with stack
					|   adjustments

	.globl	trap_fmterr
trap_fmterr:
	clr.l	-(%sp)			| stack adjust count
	movem.l	#0xFFFF,-(%sp)
	move.l	%usp,%a0		| get and save
	move.l	%a0,FR_SP(%sp)		|   the user stack pointer
	clr.l	-(%sp)			| no VA arg
	clr.l	-(%sp)			| or code arg
	move.l	#T_FMTERR,-(%sp)	| push trap type
	jra	faultstkadj		| call trap and deal with stack
					|   adjustments

	.globl	trap_fpfault
trap_fpfault:
	clr.l	-(%sp)			| stack adjust count
	move.l	%usp,%a0		| and save
	move.l	%a0,FR_SP(%sp)		|   the user stack pointer
	clr.l	-(%sp)			| no VA arg
	lea	fpctx,%a0		| address of FP save area
	fsave	(%a0)			| save state
	tst.b	(%a0)			| null state frame?
	jeq	1f			| yes, safe
	clr.w	%d0			| no, need to tweak BIU
	move.b	1(%a0),%d0		| get frame size
	bset	#3,%a0@(0,%d0:w)	| set exc_pend bit of BIU
1:	fmovem	%fpsr,-(%sp)		| push fpsr as code argument
	frestore (%a0)			| restore state
	move.l	#T_FPERR,-(%sp)		| push type arg
	jra	faultstkadj		| call trap and deal with stack cleanup

|
| Other exceptions only cause four and six word stack frames and require
| no post-trap stack adjustment.
|
	.globl	trap_badtrap
trap_badtrap:
	movem.l	#0xC0C0,-(%sp)		| save scratch regs
	move.w	%sp@(22),-(%sp)		| push exception vector info
	clr.w	-(%sp)
	move.l	22(%sp),-(%sp)		| and PC
	jbsr	straytrap		| report
	addq.l	#8,%sp			| pop args
	movem.l	(%sp)+,#0x0303		| restore regs
	rte				| all done
