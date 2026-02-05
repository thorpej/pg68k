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

#ifndef intr_h_included
#define	intr_h_included

#include "systypes.h"
#include "queue.h"

struct intr_frame {
	/* regs saved on the stack by the interrupt stub */
	u_int		if_regs[4];	/* d0,d1,a0,a1 */
	/* hardware frame */
	u_short		if_sr;		/* SR at time of interrupt */
	u_int		if_pc;		/* PC at time of interrupt */
	u_short		if_vo;		/* vector offset (4-word HW frame) */
} __attribute__((__packed__));

struct intr_handle {
	LIST_ENTRY(intr_handle) ih_link;
	void	(*ih_func)(void *);
	void	*ih_arg;
	int	ih_ipl;
};

void	intr_init(void);
void	intr_fini(void);
void	intr_establish(struct intr_handle *);
void	intr_autovec(struct intr_frame);

#endif /* intr_h_included */
