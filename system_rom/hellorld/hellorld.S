|
| HELLORLD
|
| Because, of course.
|
| Written by Jason R. Thorpe, Feb 2026.
| Public domain.
|

#include "config.h"

|
| The romcall vector is located in ROM just after the vector table.
|
#ifdef ROM_VIRT
#define	ROMCALL_BASE	(ROM_VIRT + 0x400)
#else
#define	ROMCALL_BASE	(ROM_ADDR + 0x400)
#endif

|
| The romcall version number is the 0th longword. The (*halt)() vector
| is at slot 2 and the (*putc)() vector is at slot 8.
|
#define	ROMCALL_HALT	(2 * 4)
#define	ROMCALL_PUTC	(8 * 4)

	.text
	.globl	_start
_start:
	|
	| We arrive here with the system basically quiesced, running
	| with the firmware environment's stack and address space,
	| whatever that may be.
	|

	| Get the firmware (*putc)() function.
	movea.l	ROMCALL_BASE+ROMCALL_PUTC,%a2

	lea	hellorld_string,%a3	| %a3 = string to print
	moveq	#0,%d2			| clear all of %d2
	move.l	%d2,-(%sp)		| make stack slot for char argument
1:	move.b	(%a3)+,%d2		| %d2 = *cp++
	beq	1f			| break out of loop if '\0'
	move.l	%d2,(%sp)		| char into arg slot on stack
	jsr	(%a2)			| (*putc)(ch)
	bra	1b			| go around again
1:
	addql	#4,%sp			| pop the stack slot
					|   (not that it matters)

	| Get the formware (*halt)() function and halt back out to
	| the firmware.
	movea.l	ROMCALL_BASE+ROMCALL_HALT,%a2
	jmp	(%a2)

hellorld_string:
	.string	"Hellorld!\n"
