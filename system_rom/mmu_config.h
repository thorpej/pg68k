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

#ifndef mmu_config_h_included
#define	mmu_config_h_included

#include "config.h"

#ifdef CONFIG_MC68010

#include "cpu010_mmu.h"

/*
 * We just slurp up the last 512 PMEGs (4096 PMEs) to use for the
 * firmware mappings.  We won't use them all, but the kernel can
 * use any unused PMEGs in that range if it likes (and can leverage
 * some of our PMEGs as well).
 */
#define	ROM_PMEG_BASE	(PGMMU_NUM_PMEGS - PGMMU_NUM_SEGS)
#define	ROM_PME_BASE	(ROM_PMEG_BASE * PGMMU_PMES_PER_PMEG)
#define	ROM_PME_SIZE	(0x01000000 / PAGE_SIZE)

#endif /* CONFIG_MC68010 */

#endif /* mmu_config_h_included */
