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

#include "intr.h"
#include "syslib.h"

#define	NIPL	8

LIST_HEAD(intr_handle_list, intr_handle) intr_handle_lists[NIPL];

static void
spurious_intr(void *arg)
{
	printf("WARNING: spurious interrupt!\n");
}

static struct intr_handle spurious_intr_handle = {
	.ih_func = spurious_intr,
	.ih_ipl  = 0,
};

#if defined(CONFIG_MC68010)
#include "control.h"

static void
intr_enable(void)
{
	uint8_t sysen = control_inb(CTLREG_SYSEN);
	control_outb(CTLREG_SYSEN, sysen | SYSEN_INT);
}

static void
intr_disable(void)
{
	uint8_t sysen = control_inb(CTLREG_SYSEN);
	control_outb(CTLREG_SYSEN, sysen & ~SYSEN_INT);
}
#else /* not any of the above */
static void
intr_enable(void)
{
	return;
}

static void
intr_disable(void)
{
	return;
}
#endif

void
intr_init(void)
{
	int ipl;

	for (ipl = 0; ipl < NIPL; ipl++) {
		LIST_INIT(&intr_handle_lists[ipl]);
	}
	LIST_INSERT_HEAD(&intr_handle_lists[0], &spurious_intr_handle, ih_link);
	intr_enable();
}

void
intr_fini(void)
{
	intr_disable();
}

void
intr_establish(struct intr_handle *ih)
{
	if (ih->ih_func == NULL ||
	    ih->ih_ipl < 1 || ih->ih_ipl >= NIPL) {
		printf("%s: ignoring invalid intr_handle=%p\n",
		    __func__, ih);
		return;
	}
	LIST_INSERT_HEAD(&intr_handle_lists[ih->ih_ipl], ih, ih_link);
}

#define	VECI_INTRAV0		24
#define	VECO_TO_VECI(x)		((uint8_t)((unsigned int)(x) >> 2))

void
intr_autovec(struct intr_frame frame)
{

	const int ipl = VECO_TO_VECI(frame.if_vo) - VECI_INTRAV0;
	const struct intr_handle *ih;

	LIST_FOREACH(ih, &intr_handle_lists[ipl], ih_link) {
		void *arg = ih->ih_arg ? ih->ih_arg : &frame;
		(*ih->ih_func)(arg);
	}
}
