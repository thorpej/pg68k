|
| HELLORLD
|
| Because, of course.
|
| Written by Jason R. Thorpe, Feb 2026.
| Public domain.
|

|
| The romcall version number is the 0th longword.  The (*putc)() vector
| is at slot 8.
|
#define	ROMCALL_PUTC	(8 * 4)

	.text
	.globl	_start
_start:
	|
	| We arrive here with the system basically quiesced, running
	| with the firmware environment's stack and address space,
	| whatever that may be.
	|

	|
	| The firmware call vector immediately follows the vector
	| table.  Adust romcall_base by the value of the VBR.
	| Obviously, we can't do this on a plain-Jane 68000.
	| (It's very lame that the compiler doesn't give us a better
	| signal here.)
	|
#if defined(__mc68010__) || defined(__mc68020__) || defined(__mc68030__) || \
    defined(__mc68040__)
	movec	%vbr,%a2
	add.l	romcall_base,%a2	| %a2 = base of call vector
	move.l	%a2,romcall_base	| stash it away for later
#endif

	lea	hellorld_string,%a3	| %a3 = string to print
	moveq	#0,%d2			| clear all of %d2
	move.l	%d2,-(%sp)		| make stack slot for char argument
1:	move.b	(%a3)+,%d2		| %d2 = *cp++
	beq	1f			| break out of loop if '\0'
	move.l	%d2,(%sp)		| char into arg slot on stack
	jsr	ROMCALL_PUTC(%a2)	| (*putc)(ch)
	bra	1b			| go around again
1:
	addql	#4,%sp			| pop the stack slot

	| Call into C code to dump the bootinfo.
	pea	_end
	jsr	showme			| showme(&_end)
	addql	#4,%sp

	|
	| For very simple programs (like this one), we can simply return
	| and the firmware will gracefully perform the halt for us.
	|
	rts

hellorld_string:
	.asciz	"Hellorld!\n"

	.data
	.globl	romcall_base
romcall_base:
	.long	0x400			| immediately follows vector table
