/*
 * Copyright (c) 2025, 2026 Jason R. Thorpe.
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

#include "config.h"
#include "syslib.h"
#include "cli.h"

#include "trap.h"

#ifdef CONFIG_MC68010
#include "control.h"
#include "cpu010_mmu.h"
#endif

jmp_buf nofault_env;
bool nofault;

bool
badaddr_read8(volatile uint8_t *p, uint8_t *valp)
{
	nofault = true;
	if (setjmp(nofault_env)) {
		/* nofault already cleared */
		return true;
	}
	*valp = *p;
	nofault = false;
	return false;
}

bool
badaddr_read16(volatile uint16_t *p, uint16_t *valp)
{
	nofault = true;
	if (setjmp(nofault_env)) {
		/* nofault already cleared */
		return true;
	}
	*valp = *p;
	nofault = false;
	return false;
}

bool
badaddr_read32(volatile uint32_t *p, uint32_t *valp)
{
	nofault = true;
	if (setjmp(nofault_env)) {
		/* nofault already cleared */
		return true;
	}
	*valp = *p;
	nofault = false;
	return false;
}

bool
badaddr_write32(volatile uint32_t *p, uint32_t val)
{
	nofault = true;
	if (setjmp(nofault_env)) {
		/* nofault already cleared */
		return true;
	}
	*p = val;
	nofault = false;
	return false;
}

#ifdef CONFIG_MC68010
void
cpu010_mmu_page_fault(struct trap_frame *tf, uint8_t berr)
{
	struct trap_frame_ext8 *ext = trap_frame_ext(tf);
	int fc;
	const char *errstr;

	if (berr & BERR_PRIV) {
		errstr = "PRIVILEGE";
	} else if (berr & BERR_PROT) {
		errstr = "PROTECTION";
	} else {
		errstr = "INVALID";
	}

	if (tf->tf_format != 8) {
		printf("!!! PAGE FAULT UNEXPECTED FRAME FORMAT %d\n",
		    tf->tf_format);
		cli_longjmp();
	}

	fc = __SHIFTOUT(ext->tf_ssw, SSW_FC);

	printf("!!! PAGE FAULT PC=0x%08x ADDR=0x%08x FC=%d - %s ERROR\n",
	    tf->tf_pc, ext->tf_faultaddr, fc, errstr);
	cli_longjmp();
}
#endif /* CONFIG_MC68010 */

void
trap_handler_buserr(struct trap_frame tf)
{
#ifdef CONFIG_MC68010
	uint8_t berr = control_inb(MMUREG_BUSERROR);

	/* Check for a page fault. */
	if (berr & (BERR_INVALID | BERR_PROT | BERR_PRIV)) {
		cpu010_mmu_page_fault(&tf, berr);
		return;
	}
#endif /* CONFIG_MC68010 */

	if (nofault) {
		longjmp(nofault_env, 1);
	}

	printf("Yup, got a Bus Error trap!\n");
	cli_longjmp();
}
