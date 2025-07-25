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
	.long	trap_cpv	| 13: Coprocessor Protocol Violation
	.long	trap_fmterr	| 14: Format Error
	.long	trap_badtrap	| 15: Uninitialized Interrupt
	.long	trap_badtrap	| 16: unassigned, reserved
	.long	trap_badtrap	| 17: unassigned, reserved
	.long	trap_badtrap	| 18: unassigned, reserved
	.long	trap_badtrap	| 19: unassigned, reserved
	.long	trap_badtrap	| 20: unassigned, reserved
	.long	trap_badtrap	| 21: unassigned, reserved
	.long	trap_badtrap	| 22: unassigned, reserved
	.long	trap_badtrap	| 23: unassigned, reserved
	.long	trap_intrav0	| 24: Spurious Interrrupt
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
#ifdef CONFIG_MC68000
	.long	trap_badtrap	| 57: unassigned, reserved
	.long	trap_badtrap	| 58: unassigned, reserved
#else
	.long	trap_badtrap	| 57: PMMU Illegal Operation
	.long	trap_badtrap	| 58: PMMU Access Level Violation
#endif
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
	move.l	(%a0),%sp

#ifndef CONFIG_MC68000
	| Relocate the vector table to the ROM address.
	movc	%a0,%vbr

	|
	| Before we go any further, make sure the MMU is turned off
	| and the caches are cleared.
	|
#ifdef CONFIG_MC68030
	move.l	#CACHE_OFF,%d0
	movc	%d0,%cacr
	clr.l	-(%sp)
	pmove	(%sp),%tc
	addq.l	#4,%sp
#else
#error Need non-68030 cache+MMU code.
#endif
#endif /* ! CONFIG_MC68000 */

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

	|
	| Terminate the call stack by loading $0000.0000 into
	| %fp, push's main's arguments, and call it.
	|
	move.l	#0,%fp
	pea	rom_argv0
	move.l	#1,-(%sp)
	jbsr	main		| main(1, &rom_argv0)
	addq.l	#8,%sp

	| main() should never return.  If it does, just loop forever.
1:	bra	1b

|
| Trap handler stubs.
|
trap_buserr:

trap_addrerr:

trap_fpfline:

trap_intrav0:

trap_intrav:

trap_nmi:

trap_fpfault:

