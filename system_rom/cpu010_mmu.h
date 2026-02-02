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

#ifndef _CPU010_MMU_H_
#define	_CPU010_MMU_H_

/*
 * Definitions for the Phaethon 1 MMU.
 *
 * The Phaethon 1 MMU is inspired by the Sun3 MMU (which is itself similar
 * to the Sun2 MMU), but has some differences.  Here are the features of
 * the Phaethon 1's MMU:
 *
 * ==> 24 bits of virtual address (16MB, as limited by the 68010) is mapped
 *     onto 28 bits (256MB) of physical address.
 *
 * ==> 64 contexts; kernel accessess all use context 0, leaving 1-63
 *     available for user processes.  There is no restriction against
 *     using context 0 for user processes, and there is per-page kernel
 *     privilege protection in case you do.
 *
 * ==> 2-level address translation: each context is comprised of 512
 *     32KB segments, and each segment is comprised of 8 4KB pages.
 *
 * ==> A total of 262,144 32-bit Page Map entries.  Because of the 8-entry
 *     grouping, there are a total of 32,768 Page Map Entry Groups (PMEGs)
 *     shared by all contexts.  Thus, each Segment Map entry is 16 bits
 *     (16-bit PMEG index plus a VALID bit).
 *
 *     This is actually another key difference from the Sun MMU.  With
 *     the Sun MMU, PMEGs were a fairly scarce resource.  Here, there
 *     are a lot of them... in fact, enough to fully map then entirety
 *     of all 64 contexts (32,768 / 64 == 512).  This implies that we
 *     shouldn't burn too much memory on a per-PMEG basis to manage them.
 *
 * ==> Independent access to the Page Map entries.  This is a key difference
 *     from the Sun MMU, which always uses the context register + Segment
 *     Map to address Page Map entries.  The Phaethon 1's MMU has an address
 *     mux in front of the Page Map, which selects where the Page Map entry
 *     index comes from.
 *
 * ==> Protection on a per-page basis: K (kernel privilege required) and
 *     W (writable).
 *
 * ==> Page Referenced and Modified tracking.
 *
 * ==> Fast access to the kernel segment map via a dedicated address range
 *     for this purpose.
 *
 * ==> Separate reporting for privilege (user access to kernel-only page)
 *     and protection (write access to read-only page) violations, with
 *     privilege violation reporting taking priority.
 *
 * ==> Centralized bus error handling for the whole system, include a
 *     a bus cycle watchdog timer.
 *
 * ==> MMU register access via FC#4.
 *
 * ==> Separate MMU-enable signal.
 *
 * Address translation:
 *
 * +---------------+--------------------+
 * | Context [5:0] | Segment [VA 23:15] |
 * +---------------+--------------------+--> 15-bit SegMap index --+
 *                                                                 |
 * +---------------------------------------------------------------+
 * |
 * +--> SegMap contains 15-bit PMEG index --+
 *                                          |
 * +----------------------------------------+
 * |
 * +----------------------------+
 * |         PageMap index      |
 * | PMEG -> [17:3] | VA[14:12] |
 * +----------------------------+--> 18-bit PageMap index --+
 *                                                          |
 * +--------------------------------------------------------+
 * |
 * +--> PageMap entry (32-bits) (lower 16 bits are PFN) -----+
 *                                                           |
 * +---------------------------------------------------------+
 * |
 * +--> (PFN << 12) | VA[11:0] -> PROFIT!
 *
 * Format of a Page Map entry:
 *
 *  31          28   26         23         20 19 16 15                0
 * | V | W | K |  (r)  | R | M | s3 s2 s1 s0 | (r) | Page Frame Number |
 * | a   r   e     e     e   o   (software      e
 * | l   i   r     s     f   d      defined)    s
 * | i   t   n     e     e   i                  e
 * | d   e   e     r     r   f                  r
 *           l     v     e   i                  v
 *                 e     n   e                  e
 *                 d     c   d                  d
 *                       e
 *                       d
 */

#define	PGMMU_NUM_CONTEXTS 64		/* 64 total contexts */
#define	PGMMU_CONTEXT_SUPER 0		/* supervisor is always context 0 */

#define	PGMMU_NUM_SEGS	512		/* 512 segments per context */
#define	PGMMU_SEG_SIZE	(0x01000000 / PGMMU_NUM_SEGS)
#define	PGMMU_SEG_OFFSET (PGMMU_SEG_SIZE - 1)
#define	PGMMU_SEGNUM(va) ((va) / PGMMU_SEG_SIZE)

#define	PGMMU_NUM_PMEGS	32768		/* 32K total Page Map Entry Groups */
#define	PGMMU_PMES_PER_PMEG 8		/* 8 PMEs per PMEG */
#define	PGMMU_NUM_PMES	(PGMMU_NUM_PMEGS * PGMMU_PMES_PER_PMEG)

#define	PAGE_SHIFT	12		/* 4K pages */
#define	PAGE_SIZE	(1 << PAGE_SHIFT)
#define	PAGE_OFFSET	(PAGE_SIZE - 1)

#define	SME_PMEG	0x7fff		/* Page Map Entry Group for segment */
#define	SME_V		0x8000		/* Segment Map entry valid */

#define	PME_PFN		0x0000ffff	/* Page Frame Number (PA >> 12) */
#define	PME_SW0		0x00100000	/* software defined bits */
#define	PME_SW1		0x00200000
#define	PME_SW2		0x00400000
#define	PME_SW3		0x00800000
#define	PME_MOD		0x01000000	/* page was modified */
#define	PME_REF		0x02000000	/* page was referenced */
#define	PME_K		0x20000000	/* kernel-only page */
#define	PME_W		0x40000000	/* writable page */
#define	PME_V		0x80000000	/* Page Map entry valid */

/*
 * FC#4 is used by the MMU as "control space".  CPU address bits A3..A1
 * are used as a register selector, and the space is decoded like so:
 *
 *                          selector bits
 *                               vvv
 *      xxxx.xxxx xxxx.xxxx xxxx.000x   non-MMU control space (ignored)
 *      SSSS.SSSS Sxxx.xxxx xxxx.0010   SegMap entry for Context 0
 *      SSSS.SSSS Sxxx.xxxx xxxx.0100   SegMap entry (relative to Context Reg)
 *      xxxx.xxxx xxxx.xxxx xxxx.0110   Context Register (byte)
 *      xxGG.GGGG GGGG.GGGG GEEE.1000   PageMap entry (upper word)
 *      xxGG.GGGG GGGG.GGGG GEEE.1010   PageMap entry (lower word)
 *        ^^^^^^^^^^^^^^^^^^^|||
 *                 PMEG      |||
 *                           ^^^
 *                    Entry within PMEG
 *      xxxx.xxxx xxxx.xxxx xxxx.1100   Bus Error Register (byte)
 *      xxxx.xxxx xxxx.xxxx xxxx.1110   (unused; reserved)
 */
#define	MMU_CTLSEL_SEGMAP0	0x02
#define	MMU_CTLSEL_SEGMAP	0x04
#define	MMU_CTLSEL_CONTEXT	0x06
#define	MMU_CTLSEL_PAGEMAP_U	0x08
#define	MMU_CTLSEL_PAGEMAP_L	0x0a
#define	MMU_CTLSEL_PAGEMAP	MMU_CTLSEL_PAGEMAP_U
#define	MMU_CTLSEL_BUSERROR	0x0c

#define	MMUREG_SEGMAP0_ENTRY(seg) (((seg) << 15) | MMU_CTLSEL_SEGMAP0)
#define	MMUREG_SEGMAP_ENTRY(seg)  (((seg) << 15) | MMU_CTLSEL_SEGMAP)
#define	MMUREG_CONTEXT		  (MMU_CTLSEL_CONTEXT)
#define	MMUREG_PAGEMAP_ENTRY(pmeg, e) \
		(((pmeg) << 7) | ((e) << 4) | MMU_CTLSEL_PAGEMAP)
#define	MMUREG_BUSERROR		(MMU_CTLSEL_BUSERROR)

/*
 * Bus Error Register
 *
 * The Bus Error Register contains the cause of the most recent
 * bus error.  It is reset when read; writes are ignored.
 */
#define	BERR_INVALID	0x01	/* invalid translation */
#define	BERR_PROT	0x02	/* protection error (ro page) */
#define	BERR_PRIV	0x04	/* privilege error (ko page) */
#define	BERR_TIMEOUT	0x10	/* bus cycle watchdog timed out */
#define	BERR_VME	0x20	/* VMEbus /BERR asserted */

	/* short-hand for "any MMU page fault type" */
#define	BERR_PAGEFAULT	(BERR_INVALID|BERR_PROT|BERR_PRIV)

#endif /* _CPU010_MMU_H_ */
