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

#ifndef romcalls_h_included
#define	romcalls_h_included

struct romcalls_v1 {
	int	rv1_version;	/* version of the ROM call vector */

	/*
	 * System boot lifecycle.  These functions can be called
	 * to long as the firmware appears in the address space where
	 * it's expected, but does not need to run with the firmware
	 * mappings.
	 *
	 * void (*reboot)(void);
	 *	Reboot the system, auto-booting the system, if possible.
	 *
	 * void (*halt)(void);
	 *	Halt the system, returning to the firmware command
	 *	prompt.
	 *
	 * void (*poweroff)(void);
	 *	Power-off the system, if possible.  If power-off is
	 *	not supported or unsuccessful, this is the equivalent
	 *	of (*halt)().
	 */
	void	(*rv1_reboot)(void);
	void	(*rv1_halt)(void);
	void	(*rv1_poweroff)(void);

	/*
	 * Memory allocation.  These functions may only be called when
	 * running on the firmware's mappings, and are provided as a
	 * convenience for client programs who wish to allocate and
	 * free memory dynamically.  These are not usable if the client
	 * program implements its own memory management.
	 */
	void *	(*rv1_malloc)(unsigned int);
	void *	(*rv1_realloc)(void *, unsigned int);
	void	(*rv1_free)(void *);

	/*
	 * console I/O.  These functions may ONLY be called when running
	 * on the firmware's mappings.  Once the operating system is running
	 * on its own mappings, these functions may not be used.
	 *
	 * int (*cnpollc)(void);
	 *	Poll for a character from the console.  If no character
	 *	is available, -1 is returned.
	 *
	 * void (*cnputc)(int ch);
	 *	Output a character to the console.  Newlines are "cooked",
	 *	i.e. a '\n' sent to (*cnputc)() results in "\r\n" being
	 *	sent to the terminal.
	 */
	int	(*rv1_cnpollc)(void);
	void	(*rv1_cnputc)(int);
};

extern struct romcalls_v1 romcalls;

void	romcall_reboot(void);
void	romcall_reboot_noauto(void);
void	romcall_halt(void);
void	romcall_poweroff(void);

#endif /* romcalls_h_included */
