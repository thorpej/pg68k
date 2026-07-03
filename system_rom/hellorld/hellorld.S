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
| is at slot 5.
|
#define	ROMCALL_PUTC	(5 * 4)

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

	/*
	 * When the firmware transfers control to the kernel, the
	 * stack looks like this:
	 *
	 *	[int]    bootinfo_size
	 *	[void *] bootinfo_ptr
	 *	[int]    MAGIC ('BnFo')
	 *	[int]    MAGIC ('pg68')
	 * SP->	[void *] return-address-back-to-firmware
	 *
	 * Check to see if the bootinfo was written at _end[].  If it 
	 * wasn't, then the kernel was likely loaded from an S-Record 
	 * file rather than an ELF image, and the S-Record file doesn't
	 * have the BSS segment in it.  The loader doesn't know any better
	 * (and doesn't know how big the BSS segment is, in any case), and
	 * so it plops the bootinfo at the end of where it finished loading.
	 *
	 * If we detect this, move the bootinfo into the expected location
	 * before clearing BSS.
	 */
	cmpl	#0x70673638,%sp@(4)	| 'pg68'
	bne	9f			| skip if magic is wrong
	cmpl	#0x4263466f,%sp@(8)	| 'BnFo'
	bne	9f

	movl	%sp@(12),%a1		| %a1 = bootinfo pointer
	lea	_end,%a0		| %a0 = _end[]
	cmpal	%a0,%a1			| bootinfo pointer >= end[]?
	bcc	9f			| yes, skip

	movl	%sp@(16),%d2		| bootinfo size

	addl	%d2,%a1
	addl	%d2,%a0
	subql	#1,%d2
1:
	move.b	%a1@-,%a0@-		| relocate the bootinfo
	dbra	%d2,1b

	| Clear the BSS.
	lea	edata,%a0
	movl	#_end - 4,%d0
	subl	#edata,%d0
	lsrl	#2,%d0
1:	clrl	%a0@+
	dbra	%d0,1b

9:
	pea	hellorld_string
	jsr	print_string
	addql	#4,%sp

	| Call into C code to dump the bootinfo.
	movl	%sp@(16),%d0		| bootinfo size
	movl	%d0,%sp@-
	pea	_end
	jsr	showme			| showme(&_end, sz)
	addql	#8,%sp

	|
	| For very simple programs (like this one), we can simply return
	| and the firmware will gracefully perform the halt for us.
	|
	rts

print_string:
	moveml	%d2/%a2-%a3,%sp@-	| save regs
	move.l	romcall_base,%a2	| %a2 = romcall base
	movea.l	ROMCALL_PUTC(%a2),%a2	| %a2 = (*putc)() function pointer
	move.l	16(%sp),%a3		| pointer to string
	moveq	#0,%d2			| clear all of %d2
	move.l	%d2,-(%sp)		| make stack slot for char argument
1:	move.b	(%a3)+,%d2		| %d2 = *cp++
	beq	1f			| break out of loop if '\0'
	move.l	%d2,(%sp)		| char into arg slot on stack
	jsr	(%a2)			| (*putc)(ch)
	bra	1b			| go around again
1:
	addql	#4,%sp			| pop char arg
	moveml	%sp@+,%d2/%a2-%a3	| restore regs
	rts

hellorld_string:
	.asciz	"Hellorld!\n"

	.data
	.globl	romcall_base
romcall_base:
	.long	0x400			| immediately follows vector table
