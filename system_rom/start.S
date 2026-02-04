|
| Copyright (c) 2025 Jason R. Thorpe.
| All rights reserved.
|
| Redistribution and use in source and binary forms, with or without
| modification, are permitted provided that the following conditions
| are met:
| 1. Redistributions of source code must retain the above copyright
|    notice, this list of conditions and the following disclaimer.
| 2. Redistributions in binary form must reproduce the above copyright
|    notice, this list of conditions and the following disclaimer in the
|    documentation and/or other materials provided with the distribution.
|
| THIS SOFTWARE IS PROVIDED BY THE AUTHOR ``AS IS'' AND ANY EXPRESS OR
| IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
| OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
| IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY DIRECT, INDIRECT,
| INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
| BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
| LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
| AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
| OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
| OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
| SUCH DAMAGE.
|

|
| Start of the pg68k system ROM.
|
| The nominal address of the system ROM is $FFF0.0000, but for the
| first four bus cycles after a reset, the ROM always responds in order
| to load the initial PC and SP.
|
| Why four?  The system ROM is a 16-bit port (EVEN and ODD only).
|

#include "config.h"
#include "cache.h"
#include "psl.h"

#ifdef CONFIG_MC68010
#include "control.h"
#include "mmu_config.h"
#endif /* CONFIG_MC68010 */

	.text

|
| VECTOR TABLE
|
| The vector table must be at the beginning of ROM for reset.
|

#define	BADTRAP16						\
	.long	trap_badtrap	;				\
	.long	trap_badtrap	;				\
	.long	trap_badtrap	;				\
	.long	trap_badtrap	;				\
	.long	trap_badtrap	;				\
	.long	trap_badtrap	;				\
	.long	trap_badtrap	;				\
	.long	trap_badtrap	;				\
	.long	trap_badtrap	;				\
	.long	trap_badtrap	;				\
	.long	trap_badtrap	;				\
	.long	trap_badtrap	;				\
	.long	trap_badtrap	;				\
	.long	trap_badtrap	;				\
	.long	trap_badtrap	;				\
	.long	trap_badtrap

	.globl	vectab
vectab:
	.long	CONFIG_RIISP	|  0: Reset Initial Interrupt SP
	.long	start		|  1: Reset Initial PC
	.long	trap_buserr	|  2: Bus Error
	.long	trap_addrerr	|  3: Address Error
	.long	trap_illinst	|  4: Illegal Instruction
	.long	trap_zerodiv	|  5: Zero Divide
	.long	trap_chk	|  6: CHK, CHK2 instruction
	.long	trap_trapv	|  7: cpTRAPcc, TRAPcc, TRAPV
	.long	trap_priv	|  8: Privilege Violation
	.long	trap_trace	|  9: Trace
	.long	trap_illinst	| 10: Line 1010 Emulator
	.long	trap_fpfline	| 11: Line 1111 Emulator
	.long	trap_badtrap	| 12: unassigned, reserved
#if defined(CONFIG_MC68000) || defined(CONFIG_MC68010)
	.long	trap_badtrap	| 13: unassigned, reserved
#else
	.long	trap_badtrap	| 13: Coprocessor Protocol Violation
#endif
#if defined(CONFIG_MC68000)
	.long	trap_badtrap	| 14: unassigned, reserved
#else
	.long	trap_fmterr	| 14: Format Error
#endif
	.long	trap_badtrap	| 15: Uninitialized Interrupt
	.long	trap_badtrap	| 16: unassigned, reserved
	.long	trap_badtrap	| 17: unassigned, reserved
	.long	trap_badtrap	| 18: unassigned, reserved
	.long	trap_badtrap	| 19: unassigned, reserved
	.long	trap_badtrap	| 20: unassigned, reserved
	.long	trap_badtrap	| 21: unassigned, reserved
	.long	trap_badtrap	| 22: unassigned, reserved
	.long	trap_badtrap	| 23: unassigned, reserved
	.long	trap_intrav	| 24: Spurious Interrrupt
	.long	trap_intrav	| 25: Level 1 Interrupt Autovector
	.long	trap_intrav	| 26: Level 2 Interrupt Autovector
	.long	trap_intrav	| 27: Level 3 Interrupt Autovector
	.long	trap_intrav	| 28: Level 4 Interrupt Autovector
	.long	trap_intrav	| 29: Level 5 Interrupt Autovector
	.long	trap_intrav	| 30: Level 6 Interrupt Autovector
	.long	trap_nmi	| 31: Level 7 Interrupt Autovector / NMI
	.long	trap_illinst	| 32: Trap #0 instruction
	.long	trap_illinst	| 33: Trap #1 instruction
	.long	trap_illinst	| 34: Trap #2 instruction
	.long	trap_illinst	| 35: Trap #3 instruction
	.long	trap_illinst	| 36: Trap #4 instruction
	.long	trap_illinst	| 37: Trap #5 instruction
	.long	trap_illinst	| 38: Trap #6 instruction
	.long	trap_illinst	| 39: Trap #7 instruction
	.long	trap_illinst	| 40: Trap #8 instruction
	.long	trap_illinst	| 41: Trap #9 instruction
	.long	trap_illinst	| 42: Trap #10 instruction
	.long	trap_illinst	| 43: Trap #11 instruction
	.long	trap_illinst	| 44: Trap #12 instruction
	.long	trap_illinst	| 45: Trap #13 instruction
	.long	trap_illinst	| 46: Trap #14 instruction
	.long	trap_illinst	| 47: Trap #15 instruction
#if defined(CONFIG_MC68000) || defined(CONFIG_MC68010)
	BADTRAP16		| 48-63: unassigned, reserved
#else /* ! (CONFIG_MC68000 || CONFIG_MC68010) */
	.long	trap_fpfault	| 48: FPCP Branch or Set on Unordered
	.long	trap_fpfault	| 49: FPCP Inexact Result
	.long	trap_fpfault	| 50: FPCP Divide by Zero
	.long	trap_fpfault	| 51: FPCP Underflow
	.long	trap_fpfault	| 52: FPCP Operand Error
	.long	trap_fpfault	| 53: FPCP Overflow
	.long	trap_fpfault	| 54: FPCP Signalling NaN
#ifdef CONFIG_MC68060
	.long	trap_unimp60	| 55: FP Unimplemented Data Type
#else
	.long	trap_fpunsupp	| 55: FP Unimplemented Data Type (XXX ??)
#endif
#ifdef CONFIG_MC68030
	.long	trap_badtrap	| 56: PMMU Configuration
#else
	.long	trap_badtrap	| 56: unassigned, reserved
#endif
	.long	trap_badtrap	| 57: PMMU Illegal Operation
	.long	trap_badtrap	| 58: PMMU Access Level Violation
	.long	trap_badtrap	| 59: unassigned, reserved
#ifdef CONFIG_MC68060
	.long	trap_illinst	| 60: Unimplemented Effective Address
	.long	trap_illinst	| 61: Unimplemented Integer Instruction
#else
	.long	trap_badtrap	| 60: unassigned, reserved
	.long	trap_badtrap	| 61: unassigned, reserved
#endif
	.long	trap_badtrap	| 62: unassigned, reserved
	.long	trap_badtrap	| 63: unassigned, reserved
#endif /* CONFIG_MC68000 || CONFIG_MC68010 */
	BADTRAP16		| 64-79: user vectors
	BADTRAP16		| 80-95: user vectors
	BADTRAP16		| 96-111: user vectors
	BADTRAP16		| 112-128: user vectors
	BADTRAP16		| 128-143: user vectors
	BADTRAP16		| 144-160: user vectors
	BADTRAP16		| 160-175: user vectors
	BADTRAP16		| 176-191: user vectors
	BADTRAP16		| 192-207: user vectors
	BADTRAP16		| 208-223: user vectors
	BADTRAP16		| 224-239: user vectors
	BADTRAP16		| 240-255: user vectors

|
| ROM call vectors.  Clients of this expect this to immediately
| follow the vector table.  The first entry is the version number,
| which defines the layout and size of this vector table.
|
	.globl	romcalls
romcalls:
	.long	0		| version 0 -- nothing

rom_argv0:
	.asciz	"pg68krom"

	.align	4

	.globl	start
start:
	| Before we do anything else, disable all interrupts.
	| Except the NMI, obviously.
	move.w	#PSL_HIGHIPL,%sr

	| In case we ended up here via nefarious means, go ahead and
	| re-load the stack pointer from the vector table.
	lea	vectab,%a0
	movea.l	(%a0),%sp

#ifdef CONFIG_MC68010
	| Set source and destination FCs for MOVES.
	moveq	#FC_CONTROL,%d0
	movec	%d0,%sfc
	movec	%d0,%dfc

	|
	| We're not sure how we got here; it could have been a system
	| reset, or it could have been the operating system requesting
	| a reboot.  In the latter case, the MMU would be turned on,
	| so we need to turn basically everything off.  But we need to
	| preserve the "please reboot" bit.
	|
	| N.B. this also disables all interrupts, including the NMI,
	| which is good, actually, because we can't use the stack yet!
	|
	movea.l	#CTLREG_SYSEN,%a5	| %a5 = System Enable register
	moves.b	(%a5),%d5		| %d5 <- System Enable register
	and.l	#SYSEN_REBOOT,%d5	| clear everything except REBOOT
	moves.b	%d5,(%a5)		| %d5 -> System Enable register

	|
	| RAM is inaccessible until the MMU is enabled, so we need to
	| bootstrap the MMU.
	|
	| We've allocated sufficient PMEGs to map everything linearly.
	|

	| Zero out all of the Segment Map space for context 0.
	moveq	#0,%d0
	move.l	#MMU_CTLSEL_SEGMAP0,%d1
	movea.l	%d1,%a0
	add.l	#(PGMMU_SEG_SIZE * PGMMU_NUM_SEGS),%d1
1:	moves.l	%d0,(%a0)
	adda.l	#PGMMU_SEG_SIZE,%a0	| advance SME address
	cmpa.l	%d1,%a0
	bne	1b

	| Zero out all of the Page Map space.
	move.l	#MMU_CTLSEL_PAGEMAP,%d1
	movea.l	%d1,%a0
	add.l	#(PGMMU_NUM_PMES << 4),%d1
1:	moves.l	%d0,(%a0)
	adda.l	#(1 << 4),%a0		| advance PME address
	cmpa.l	%d1,%a0
	bne	1b

#define	FILL_PMES(v, p, s, w)						 \
	movea.l	%d2,%a0							;\
	adda.l	#(((v) >> PAGE_SHIFT) << 4),%a0				;\
	move.l	%d2,%d1							;\
	add.l	#((((v)+(s)) >> PAGE_SHIFT) << 4),%d1			;\
									 \
	move.l	#(PME_V+(w)+PME_K+((p) >> PAGE_SHIFT)),%d0		;\
									 \
1:	moves.l	%d0,(%a0)						;\
	adda.l	#(1 << 4),%a0						;\
	addq.l	#1,%d0							;\
	cmpa.l	%d1,%a0							;\
	bne	1b

	| %d2 = base of the ROM's Page Map region
	move.l	#((ROM_PME_BASE << 4)+MMU_CTLSEL_PAGEMAP),%d2

	| Map all of the on-board system RAM VA==PA.
	FILL_PMES(RAM0_START, RAM0_START, RAM0_SIZE, PME_W)

	| Map the ROM into the correct location.
	FILL_PMES(ROM_VIRT, ROM_PHYS, ROM_SIZE, 0)

	| Map the on-board I/O.
	FILL_PMES(OBIO_VIRT, OBIO_PHYS, OBIO_SIZE, PME_W);

#undef FILL_PMES

	|
	| Now that all of the Page Map entries are valid, we can
	| set the Segment Map entries to point to them.
	|
	move.l	#MMU_CTLSEL_SEGMAP0,%d2

#define	FILL_SMES(v, s)							 \
	movea.l	%d2,%a0							;\
	adda.l	#((v) & ~PGMMU_SEG_OFFSET),%a0				;\
	move.l	%d2,%d1							;\
	add.l	#(((v)+(s)) & ~PGMMU_SEG_OFFSET),%d1			;\
									 \
	move.l	#(SME_V+(ROM_PMEG_BASE+((v) / PGMMU_SEG_SIZE))),%d0	;\
									 \
1:	moves.l	%d0,(%a0)						;\
	adda.l	#PGMMU_SEG_SIZE,%a0					;\
	addq.l	#1,%d0							;\
	cmpa.l	%d1,%a0							;\
	bne	1b

	| RAM Segment Map entries.
	FILL_SMES(RAM0_START, RAM0_SIZE)

	| ROM Segment Map entries.
	FILL_SMES(ROM_VIRT, ROM_SIZE)

	| On-board I/O Segment Map entries.
	FILL_SMES(OBIO_VIRT, OBIO_SIZE)

#undef FILL_SMES

	|
	| The tables in the MMU are all configured for the firmware
	| environment now.  %d5 still has the System Enable register
	| value we preserved above, and %a5 still has the address,
	| so add in the MMU bit and enable the MMU!
	|
	or.b	#SYSEN_MMU,%d5
	moves.b	%d5,(%a5)

#endif /* CONFIG_MC68010 */

#ifndef CONFIG_MC68000
	| Relocate the vector table to the ROM address.
	lea	vectab,%a0
	movec	%a0,%vbr
#endif /* ! CONFIG_MC68000 */

#ifdef CONFIG_MC68030
	|
	| Make sure the MMU is turned off and the caches are cleared.
	|
	move.l	#CACHE_OFF,%d0
	movc	%d0,%cacr
	clr.l	-(%sp)
	pmove	(%sp),%tc
	addq.l	#4,%sp
#endif /* CONFIG_MC68030 */

	|
	| Copy the .data segment from ROM to the run-time location
	| in RAM.
	|
	move.l	#_edata,%d0
	sub.l	#__data_start,%d0
	move.l	%d0,-(%sp)
	pea	__data_store
	pea	__data_start
	jbsr	memcpy		| memcpy(__data_start, __data_store, d0)
	add.l	#12,%sp

	|
	| Zero the BSS segment in RAM.
	|
	move.l	#_end,%d0
	sub.l	#_edata,%d0
	move.l	%d0,-(%sp)
	clr.l	-(%sp)
	pea	_edata
	jbsr	memset		| memset(_edata, 0, d0)
	add.l	#12,%sp

#ifdef CONFIG_MC68030
	|
	| Clear and enable the I-cache.  Not much compelling reason
	| to enable the D-cache at this point, and it just makes us
	| run more like a 68020 anyway.
	|
	move.l	#IC_BE+IC_CLR+IC_ENABLE,%d0
	movc	%d0,%cacr
#endif /* CONFIG_MC68030 */

	|
	| Terminate the call stack by loading $0000.0000 into
	| %fp, push's main's arguments, and call it.
	|
	movea.l	#0,%fp
	pea	rom_argv0
	move.l	#1,-(%sp)
	jbsr	main		| main(1, &rom_argv0)
	addq.l	#8,%sp

	| main() should never return.  If it does, just loop forever.
1:	bra	1b

|
| delay_(unsigned int N)
|
| Delay for at least (N/1024) microseconds.
| This routine depends on the variable delay_divisor,
| which should be set based on the CPU clock rate or
| calibrated against a timer source.
|
| The delay value is pre-shifted in C code because the
| delay value is often a constant.
|
	.globl	delay_
delay_:
	move.l	4(%sp),%d0		| d0 = arg = (us << 10)
	move.l	delay_divisor,%d1	| d1 = delay_divisor
	jra	1f
	|
	| Align the branch target of the loop to a half-line (8 byte)
	| boundary to minimize cache effects.  This guarantees both
	| that there will be no prefetch stalls due to cache line burst
	| operations and that the loop will run from a single cache
	| half-line.
	|
	.align	8
1:	sub.l	%d1,%d0
	jgt	1b
	rts

|
| Trap handler stubs.
|
trap_buserr:

trap_addrerr:

trap_fpfline:

trap_intrav:
	movem.l	#0xC0C0,-(%sp)		| d0, d1, a0, a1
	jbsr	intr_autovec
	movem.l	(%sp)+,#0x0303
	rte

trap_nmi:

trap_fpfault:

