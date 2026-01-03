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

/*
 * XXX THIS IS AN INCOMPLETE WORK-IN-PROGRESS XXX
 *
 * Memory Management Unit for the Motorola 68010 CPU.
 *
 * This is very much modeled on the Sun MMU as realized in the Sun3,
 * but with more Contexts and PMEGs, and adapted for the constraints
 * of the 68010 address space.
 *
 * Features of this MMU:
 *
 * ==> 64 contexts; 0 for the kernel, 1-63 for user programs.  When the
 *     CPU performs a Supervisor {Data,Program} access, the context used
 *     for the translation is always 0.  User {Data,Program} translations
 *     use the context set in the Context Register.
 *
 * ==> 2-level address translation: each context is comprised of 512
 *     32KB segments, each segment is comprised of 8 4KB pages.
 *
 * ==> A total of 262,144 32-bit PageMap entries.  Because of the 8-entry
 *     grouping, there are a total of 32,768 PageMap Entry Groups (PMEGs)
 *     shared by all contexts.  Thus, each SegMap entry is 16 bits (15-bit
 *     PMEG index plus a VALID bit).
 *
 * ==> VALID bit for SegMap entries.  This means you don't have to burn
 *     a PMEG filled with invalid entries just to have unmapped segments.
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
 * Some random commentary:
 *
 * The PageMap entry format is similar to that used by the Sun3, but is
 * for 4K pages and the bits are shuffled around a little:
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
 *
 * VALID, WRITE, KERNEL, REF, and MOD are in the same locations as Sun3.
 * The Type field has been eliminated in this implementation (it basically
 * served as an extension of the physical address, and I don't really see
 * much point in having it).  The Page Frame Number has been shortened to
 * 16 bits, and with 4K pages that gives us a total of 256MB of physical
 * address space (28 bits worth), which seems perfectly adequate for a
 * 68010.
 *
 * Why is it arranged this way?  Mainly because it makes updating the
 * MOD and REF bits easier.  The PageMap is implemented as a pair of
 * 256K x 16 SRAMs.  The lower SRAM contains all of the information
 * needed to translate the virtual address to the physical address,
 * and the upper SRAM contains only metadata (both hardware and software
 * defined).  When an address translation takes place, the lower SRAM
 * can remain output-enabled for the entire duration of the bus cycle,
 * driving the system address bus.  But the upper SRAM is only needed at
 * the very beginning of the bus cycle, to check validity and protection.
 * At the end of the bus cycle, the MOD and REF bits that need to be
 * updated in the PageMap entry reside within the upper bye of that
 * upper word; that upper SRAM can be output-disabled at any time during
 * the cycle for updating those bits.
 *
 * ==> Because this MMU always uses context 0 for kernel accesses, the
 *     inclusion of a K bit seems a little redundant.  However, including
 *     it is practically free, and it would make it easier to adapt this
 *     MMU to some other application where you might not need to reserve
 *     a context just for the kernel (the only reason to do this on the
 *     68010 because each context gets just 16MB of virtual address space).
 *
 * ==> It would be pretty easy to add an X (execute) bit to the PageMap
 *     entry, and then enforce it then {Supervisor,User} Program accesses
 *     are being performed, but honestly I don't see much value in that
 *     for a CPU like the 68010.
 *
 * ==> I've put a little, but not a lot, of thought into making the
 *     translation fast enough so that MMU-driven wait states don't
 *     have to be incurred by a 10MHz 68010.  I do assume that the MMU
 *     is running at at least 2x of the 68010 clock.
 *
 * ==> This design uses a lot of external muxes.  Like, a lot.  Maybe
 *     too many.  Assuming 74ACHT157 quad 2-to-1 muxes:
 *
 *	* 4x for the MMU output address (20 bits: XA27..XA12).  Direct
 *	  passthrough from the CPU's A23..A12 if the address is untranslated,
 *	  otherwise PA27..PA12 from the PageMap.
 *
 *	* 5x for the PageMap address inputs (18 bits).  These addresses
 *	  must be driven either by the CPU (A21..A4) or by the SegMap.
 *
 *     Seriously, like it's enough that I might consider another CPLD
 *     just to deal with the address muxing.
 *
 * In addition to this CPLD, there are some external components required
 * to implement the MMU.  Specific logic families selected in order to
 * minimize propagation delays.
 *
 * ==> 1x IS61C3216AL (32K x 16 12ns SRAM) for the SegMap.
 *
 * ==> 2x AS7C4098A (256K x 16 12ns SRAM) for the PageMap.
 *
 * ==> 10x 74AHCT157 quad 2-to-1 muxes (see random thought above; maybe
 *     this gets replaced by another CPLD to mux the CPU addresses?)
 *
 * ==> 3x 74ACHT373 octal transparent latch; these are used to ensure
 *     the PageMap index remains stable during a translated bus cycle
 *     so the MOD and REF bits can be updated safely.
 *
 * ==> 6x 74AHCT245 8-bit bus transceivers for CPU data bus interface
 *     to the SegMap and PageMap.
 *
 * Notable property of the chosen SRAMs: /WE overrides /OE, so /OE can
 * always be asserted.
 */

module mmu010(
	input wire nRST,	/* system /RESET signal */
	input wire CLK,		/* system CLK20 signal (2x CPU) */

	input wire nAS,		/* /AS from CPU */
	input wire RnW,		/* R/W from CPU */
	input wire nUDS,	/* /UDS from CPU */
	input wire nLDS,	/* /LDS from CPU */
	input wire nDTACK,	/* /DTACK as viewed by CPU */

	input wire [2:0] FC,	/* FC2..FC0 from CPU */

	input wire [2:0] ADDR,	/* A3..A1 from CPU */
	inout wire [7:0] DATA,	/* D15..D8 to/from CPU */

	input wire MMU_EN,	/* MMU enabled */

	/* Important bits from the SegMap and PageMap entries. */
	input wire SME_V,	/* SegMap entry valid */
	inout wire [7:0] PME,	/* most significant PageMap entry byte */

	output wire [5:0] CTX,	/* Context to use for SegMap access */

	output wire PMACC,	/* CPU is accessing PageMap (mux control) */
	output wire MMU_ADDR,	/* MMU drives A31..A12 (mux control) */

	output wire nSMSEL,	/* SegMap select by CPU (xcvr control) */
	output wire nPMUSEL,	/* PageMapU select by CPU (xcvr control) */
	output wire nPMLSEL,	/* PageMapL select by CPU (xcvr control) */

	/* Bus controls for the SegMap SRAM (IS61C3216AL). */
	output wire nSM_WE,
	output wire nSM_UB,
	output wire nSM_LB,

	/* Bus controls for the PageMap SRAMs (AS7C4098A). */
	output wire nPMU_WE,
	output wire nPMU_UB,
	output wire nPMU_LB,
	output wire nPML_WE,
	output wire nPML_UB,
	output wire nPML_LB,

	output wire nPM_LE,	/* PageMap index latch enable */

	output wire nAS_out,	/* gated outputs for these signals. */
	output wire nUDS_out,	/* we only assert them if we're not going */
	output wire nLDS_out,	/* to abort the cycle for a xlation error */

	output wire MMU_FAULT,
	output wire MMU_DTACK	/* drives open-drain inverter */
);

/*
 * We have to synchronize /AS because we're dealing with two clock
 * domains.  RnW does not need this treatment because it's not acted
 * upon until our latched /AS signal is stable, and we don't do
 * anything with /UDS or /LDS except combinatorial logic[*].
 *
 * [*] This is actually not true, we do have to pay attention to /UDS
 * on Context Register writes and the RnW signal *changing* during a
 * bus cycle because that's how 68010 Read-Modify-Write cycles work.
 * So, we synchronize these here as well.  Generally, it's bad practice
 * to separately synchronize multiple signals like this, however we can
 * get away with it because:
 *
 * ==> For /UDS, we watch for that to happen only for Context Register
 *     writes and we know that it will transition well after /AS is
 *     stable (/AS is asserted on the rising edge of S2 and /UDS is
 *     asserted on the rising edge of S4, so there is ample time for
 *     signal synchronization to occur).  The practical upshot is that
 *     there will probably end up being a wait state incurred for
 *     Context Register writes.
 *
 * ==> For RnW, what we need to do there is, while waiting for termination
 *     of a translated read cycle, we watch for RnW to transition from
 *     high to low before the cycle terminates.  In a normal read cycle
 *     the 68010 de-asserts /AS on the falling edge of S7 before changing
 *     RnW (in S0) for any write cycle immediately following, so we would
 *     observe AS_s being de-assrted before RnW_s going low.  However, for
 *     a R-M-W cycle, only /UDS and /LDS are de-asserted; /AS remains asserted,
 *     and the bus signals aren't changed by the 68010 for S8-S11.  S12
 *     begins the write portion of the cycle, and RnW transitions to low
 *     on the rising edge of S14, and the data strobe is re-asserted on
 *     the rising edge of S16.  That is a VERY tight margin for observing
 *     the high-to-low transition of RnW_s, and if we are late, then the
 *     consequence is that a byte of data might be written to a read-only
 *     page; NOT GOOD.  So, for this reason, we also have to synchronize
 *     a copy of /DTACK.  We will watch for DTACK_s to de-assert while
 *     waiting for the termination of a read cycle, and when we observe
 *     that, we will re-gate the data strobes while waiting to see what
 *     happens next: either RnW_s transitions or AS_s de-asserts.
 *
 * NOTE: ALL OF THESE SYNCHRONIZED INPUTS ARE ACTIVE-HIGH, EVEN IF THE
 * ORIGINAL SIGNAL IS ACTIVE-LOW.
 *
 * ALSO NOTE: The MC6800 peripheral interface is -- not supported --,
 * because I don't want to be bothered with dealing with both /DTACK
 * and /VPA.
 */
reg AS1;
reg AS_s;
reg UDS1;
reg UDS_s;
reg DTACK1;
reg DTACK_s;
reg RnW1;
reg RnW_s;
always @(posedge CLK, negedge nRST) begin
	if (~nRST) begin
		AS1  <= 1'b0;
		AS_s <= 1'b0;

		UDS1  <= 1'b0;
		UDS_s <= 1'b0;

		DTACK1  <= 1'b0;
		DTACK_s <= 1'b0;

		RnW1  <= 1'b0;
		RnW_s <= 1'b0;
	end
	else begin
		AS1  <= ~nAS;
		AS_s <= AS1;

		UDS1  <= ~nUDS;
		UDS_s <= UDS1;

		DTACK1  <= ~nDTACK;
		DTACK_s <= DTACK1;

		RnW1  <= RnW;
		RnW_s <= RnW1;
	end
end

/*
 * We need to latch the upper byte of the PME when performing a
 * translation so that we can update the MOD and REF bits upon
 * a successful bus cycle.  We also define shorthand for the
 * important bits.
 */
reg [7:0] PME_copy;
wire PME_V = PME[7];		/* valid bit */
wire PME_W = PME[6];		/* write bit */
wire PME_K = PME[5];		/* kernel bit */

/*
 * The updated PageMap entry always gets the REF bit set, and we set
 * the MOD bit if RnW is low if it hasn't been set already.
 */
wire [7:0] PME_new = {PME_copy[7], PME_copy[6], PME_copy[5], PME_copy[4],
    PME_copy[3], PME_copy[2], 1'b1 /* REF */, (PME_copy[0] | ~RnW) /* MOD */};

/*
 * PME_update is asserted in the bus cycle state machine whenever the
 * PageMap entry needs to have the MOD and REF bits written back.
 */
reg PME_update;
assign PME = PME_update ? PME_new : 8'bzzzzzzzz;

/*
 * Booleans to indicate a User vs Kernel access.
 *
 * FC2   FC1   FC0
 *  0     0     0       (Undefined, reserved)
 *  0     0     1       User Data Space
 *  0     1     0       User Program Space
 *  0     1     1       (Undefined, reserved)
 *  1     0     0       (Undefined, reserved) - Control Space
 *  1     0     1       Supervisor Data Space
 *  1     1     0       Supervisor Program Space
 *  1     1     1       CPU Space
 */
localparam FC_USER_DATA		= 3'd1;
localparam FC_USER_PROGRAM	= 3'd2;
localparam FC_CONTROL		= 3'd4;
localparam FC_SUPER_DATA	= 3'd5;
localparam FC_SUPER_PROGRAM	= 3'd6;

wire UserAcc   = (FC == FC_USER_DATA  || FC == FC_USER_PROGRAM);
wire KernelAcc = (FC == FC_SUPER_DATA || FC == FC_SUPER_PROGRAM);
wire Translate = (UserAcc || KernelAcc) && MMU_EN;

/*
 * ADDR[2:0] (CPU A3..A1) indicates which part of the Control space is
 * being accessed.
 *
 *	                    selector bits
 *	                         vvv
 *	xxxx.xxxx xxxx.xxxx xxxx.000x	non-MMU control space (ignored)
 *	SSSS.SSSS Sxxx.xxxx xxxx.0010	SegMap entry for Context 0
 *	SSSS.SSSS Sxxx.xxxx xxxx.0100	SegMap entry (relative to Context Reg)
 *	xxxx.xxxx xxxx.xxxx xxxx.0110	Context Register (byte)
 *	xxPP.PPPP PPPP.PPPP PPPP.1000	PageMap entry (upper word)
 *      xxPP.PPPP PPPP.PPPP PPPP.1010	PageMap entry (lower word)
 *        ^^^^^^^^^^^^^^^^^^^|||
 *                 PMEG      |||
 *                           ^^^
 *                    Entry within PMEG
 *	xxxx.xxxx xxxx.xxxx xxxx.1100	Error Register (byte)
 *	xxxx.xxxx xxxx.xxxx xxxx.1110	(unused; reserved)
 *
 * The index for SegMap access is placed where it is because those are the
 * address bits that would be normally connected to select the SegMap index
 * for normal address translation; this saves having to use a mux for those
 * address bits.
 *
 * Access rules:
 *
 * ==> Context and Error registers are 8-bit on even addresses
 *     (/UDS is the governing data strobe).
 *
 * ==> SegMap entries are 16-bit and must be accessed as words
 *     (both /UDS and /LDS asserted).
 *
 * ==> PageMap entries are 32-bit and must be accessed as 2 16-bit
 *     words (both /UDS and /LDS asserted - software an do this on
 *     its own, or let the processor deal with it).
 *
 * We don't bother enforcing these rules in hardware, because it would
 * require us to synchronize the /UDS and /LDS signals.  Instead, we
 * merely gate them on having a valid translation using combinatorial
 * logic, and only synchronize the incoming /AS signal.  If software
 * accesses these registers wrong, that's their problem!
 */
localparam MMUADDR_SegMap0	= 3'd1;
localparam MMUADDR_SegMap	= 3'd2;
localparam MMUADDR_ContextReg	= 3'd3;
localparam MMUADDR_PageMapU	= 3'd4;
localparam MMUADDR_PageMapL	= 3'd5;
localparam MMUADDR_ErrorReg	= 3'd6;
localparam MMUADDR_Reserved7	= 3'd7;

wire SegMap0Sel    = (FC == FC_CONTROL) && (ADDR == MMUADDR_SegMap0);
wire SegMapSel     = (FC == FC_CONTROL) && (ADDR == MMUADDR_SegMap);
wire PageMapUSel   = (FC == FC_CONTROL) && (ADDR == MMUADDR_PageMapU);
wire PageMapLSel   = (FC == FC_CONTROL) && (ADDR == MMUADDR_PageMapL);
wire ContextRegSel = (FC == FC_CONTROL) && (ADDR == MMUADDR_ContextReg);
wire ErrorRegSel   = (FC == FC_CONTROL) && (ADDR == MMUADDR_ErrorReg);

/*
 * From the MMU's perspective, CPU access to any of the MMU's SRAMs
 * is the same.  MapSel is to simplify the bus cycle state machine
 * logic, and the rest is all handled with combinatorial logic below.
 */
wire MapSel = (SegMap0Sel || SegMapSel || PageMapUSel || PageMapLSel);

/*
 * Address inputs to the SegMap always come from ContextReg + upper CPU
 * Address bits, so there is no mux required.  We just need combinatorial
 * logic to control the SegMap /WE, /UB, and /LB inputs.
 */
assign nSM_WE = (SegMap0Sel || SegMapSel) ? (RnW  | nAS) : 1'b1;
assign nSM_UB = (SegMap0Sel || SegMapSel) ? (nUDS | nAS) : 1'b0;
assign nSM_LB = (SegMap0Sel || SegMapSel) ? (nLDS | nAS) : 1'b0;

/* Connect CPU to SegMap data pins if SegMap is selected. */
assign nSMSEL = ~(SegMap0Sel || SegMapSel) | nAS;

/*
 * If the CPU wants to access the PageMap, then it needs to drive the
 * address inputs to those SRAMs.  This output signal controls those
 * muxes.
 */
assign PMACC = (PageMapUSel || PageMapLSel);

/*
 * Combinatorial logic for the PageMap /WE, /UB, and /LB inputs.
 *
 * The lower word drives the address bus and thus must have its byte
 * enables always driven when not being accessed by the CPU.
 *
 * The upper word's upper byte contains inputs to the MMU's combinatorial
 * logic, and thus has to remain driven when not being updated.  The
 * lower byte, however, contains only reserved and software-defined bits,
 * so the MMU doesn't need it at all; it only exists for the CPU to read
 * and write.  So, we keep it's data strobe de-asserted unless the CPU
 * is accessing it, so there is absolutely no risk of it getting clobbered
 * when the /WE is asserted when MOD and REF bits are updated in the upper
 * byte.
 */
assign nPMU_WE = PageMapUSel ? (RnW  | nAS) : ~PME_update;
assign nPMU_UB = PageMapUSel ? (nUDS | nAS) : 1'b0;
assign nPMU_LB = PageMapUSel ? (nLDS | nAS) : 1'b1;

assign nPML_WE = PageMapLSel ? (RnW  | nAS) : 1'b1;
assign nPML_UB = PageMapLSel ? (nUDS | nAS) : 1'b0;
assign nPML_LB = PageMapLSel ? (nLDS | nAS) : 1'b0;

/* Connect CPU to PageMap data pins if PageMap is selected. */
assign nPMUSEL = ~PageMapUSel | nAS;
assign nPMLSEL = ~PageMapLSel | nAS;

/*
 * If the MMU is translating an address, then it gets to drive the
 * system address bus (well, A31..A12, anyway).  This output signal
 * controls those muxes.
 */
assign MMU_ADDR = Translate;

/*
 * Context output to SegMap is hard-wired to 0 for translated kernel
 * accesses or when accessing the SegMap via the dedicated kernel SegMap
 * address range.
 */
reg [5:0] ContextReg;
assign CTX = (SegMap0Sel || KernelAcc) ? 6'd0 : ContextReg;

/*
 * Error register contains the result of the most recent MMU error.
 * These are enumerated values:
 *
 *	(highest priority)
 *	NONE		no error
 *	INVALID		invalid translation
 *	PRIV		privilege error (user access to kernel page)
 *	PROT		protection error (write access to read-only page)
 *	(lowest priority)
 *
 * Reading it resets it to zero.
 */
localparam ERR_NONE		= 2'd0; /* no error */
localparam ERR_INVALID		= 2'd1; /* invalid mapping */
localparam ERR_PRIV		= 2'd2; /* privilege (kernel page) */
localparam ERR_PROT		= 2'd3; /* protection (r/o page) */
reg [1:0] ErrorReg;
reg ErrorReg_consumed;

/*
 * Compute the translation error continuously.  It will be latched into
 * ErrorReg when needed.
 *
 * Translation is valid when both SME_V and PME_V are set.
 *
 * Privilege check is OK if PME_K is not set -OR- if kernel access is
 * being performed.
 *
 * Protection check is OK is it's a read operation -OR- if PME_W is set.
 */
wire TransOK = (SME_V && PME_V);
wire PrivOK  = (~PME_K || KernelAcc);
wire ProtOK  = (RnW || PME_W);

reg [2:0] TranslationError;
always @(*) begin
	casex ({TransOK,PrivOK,ProtOK})
	/* If invalid translation, INVALID error. */
	3'b0xx:		TranslationError = ERR_INVALID;

	/* If privilege check not OK, PRIV error. */
	3'b10x:		TranslationError = ERR_PRIV;

	/* If protection check not OK, PROT error. */
	3'b110:		TranslationError = ERR_PROT;

	/* If everything is OK, then no error \o/ */
	3'b111:		TranslationError = ERR_NONE;
	endcase
end

/*
 * Qual MMU_FAULT and MMU_DTACK on the non-synchronized /AS input to ensure
 * they de-assert as quickly as possbile.
 *
 * "AC ELECTRICAL SPECIFICATIONS - READ AND WRITE CYCLES" from the "M68000
 * Family Reference Manual" lists a minimum 0ns "/AS negated to /DTACK negated"
 * time (characteristic #28), as well as for "/AS negated to /BERR negated"
 * time (characteristic #30).
 */
reg dtack;
assign MMU_DTACK = dtack & ~nAS;
reg mmu_fault;
assign MMU_FAULT = mmu_fault & ~nAS;

/*
 * Gate the /AS signal that goes to the system on address translation success.
 *
 * We do this by OR'in (Translate && ~TranslationValid).  Here's now that
 * works:
 *
 *	- Translate is 1 when we detect {User,Supervisor} {Data,Program}
 *	  accesses -and- the MMU is enabled.
 *		==> If not a translated space, Translate is 0.
 *		==> If MMU is disabled, Translate is 0.
 *
 *	- TranslationValid starts with 0, inverting it gives us 1, and
 *	  it stays that way until we have validated the translation,
 *	  at which time TranslationValid is set to 1 and inverting it
 *	  gives us 0.
 *
 * So, the result of AND'ing those two things together results in a 1
 * when the output of the signal should be blocked, and since it's
 * active-low, we can just OR block it.
 *
 * We do something similar with /UDS and /LDS, except it's with an AccessValid
 * flag.  We need a separate gating flag for the data strobes because of the
 * way Read-Modify-Write cycles (used by the TAS instruction) work on the
 * 68010: /AS remains asserted for the entirety of the R-M-W cycle, and the
 * R/W signal changes between the read and write portions.  This means we
 * may have to perform a second permission check while waiting for the
 * termination of a read cycle, and when we detect RnW going low during
 * that time, we want to immediately block the data strobes while we perform
 * that check.
 */
reg TranslationValid;
reg AccessValid;
assign nAS_out  = nAS  | (Translate && ~TranslationValid);
assign nUDS_out = nUDS | (Translate && ~AccessValid);
assign nLDS_out = nLDS | (Translate && ~AccessValid);

/*
 * We latch the PageMap index while we consider the translation valid
 * so that the index remains stable while we update the MOD and REF bits.
 *
 * The /LE input on the 74'373 is high when transparent, and low when
 * latched.
 */
assign nPM_LE = ~TranslationValid;

/*
 * Logic for reading internal MMU registers.
 */
reg enable_data_out;
reg [7:0] data_out;
always @(*) begin
	case ({ContextRegSel, ErrorRegSel})
	2'b10:	 data_out = {2'b0, ContextReg};
	2'b01:	 data_out = {6'b0, ErrorReg};
	default: data_out = 8'hFF;
	endcase
end
assign DATA = (enable_data_out && ~nUDS) ? data_out : 8'bzzzzzzzz;

/*
 * BUS CYCLE STATE MACHINE
 *
 * Here we take care of reading (Context, Error) and writing (Context)
 * internal MMU registers as well as performing address translations.
 *
 * It's worth noting that address translations largely happen on their
 * own by virtue of the driving of address lines into the SegMap and
 * PageMap SRAMs 
 */

wire [5:0] Cycle = {AS_s, Translate, RnW, ContextRegSel, ErrorRegSel, MapSel};

localparam CYCLE_NONE		= 6'b0xxxxx;
localparam CYCLE_RD_CONTEXT	= 6'b101100;
localparam CYCLE_WR_CONTEXT	= 6'b100100;
localparam CYCLE_RD_ERROR	= 6'b101010;
localparam CYCLE_RD_MAP		= 6'b101001;
localparam CYCLE_WR_MAP		= 6'b100001;
localparam CYCLE_XLATE_READ	= 6'b111000;
localparam CYCLE_XLATE_WRITE	= 6'b110000;

localparam Idle			= 3'd0;
localparam WriteContext		= 3'd1;
localparam TermWaitRMW0		= 3'd2;
localparam TermWaitRMW1		= 3'd3;
localparam TermWaitRMW2		= 3'd4;
localparam TermWait		= 3'd5;

reg [2:0] state;
always @(posedge CLK, negedge nRST) begin
	if (~nRST) begin
		ErrorReg <= ERR_NONE;
		ErrorReg_consumed <= 1'b0;

		ContextReg <= 6'd0;

		TranslationValid <= 1'b0;
		AccessValid <= 1'b0;

		PME_update <= 1'b0;
		PME_copy <= 8'b0;

		enable_data_out <= 1'b0;
		mmu_fault <= 1'b0;
		dtack <= 1'b0;
		state <= Idle;
	end
	else begin
		/*
		 * NOTE: We assume the CPU clock is 10MHz and that
		 * the memory subsystem is at 2x CPU clock, so each
		 * state represents 50ns.
		 */

		/*
		 * Always grab a copy of the relevant PageMap entry.
		 * That way we can use the values stored there as
		 * soon as we see a translated bus cycle begin.
		 */
		PME_copy <= PME;

		case (state)
		Idle: begin
			/* Check for the beginning of a bus cycle. */
			casex (Cycle)
			CYCLE_NONE: begin
				state <= Idle;
			end

			CYCLE_RD_CONTEXT: begin
				enable_data_out <= 1'b1;
				dtack <= 1'b1;
				state <= TermWait;
			end

			CYCLE_WR_CONTEXT: begin
				/*
				 * We have to wait for the synchronized
				 * /UDS signal to be asserted.
				 */
				state <= WriteContext;
			end

			CYCLE_RD_ERROR: begin
				ErrorReg_consumed <= 1'b1;
				enable_data_out <= 1'b1;
				dtack <= 1'b1;
				state <= TermWait;
			end

			CYCLE_RD_MAP: begin
				dtack <= 1'b1;
				state <= TermWait;
			end

			CYCLE_WR_MAP: begin
				/*
				 * The data strobes might not have been
				 * assrted yet, so we might need to insert
				 * an extra state (or 2) here to wait for
				 * that to happen, but I don't suspect that
				 * our asserting /DTACK early is going to
				 * be a problem.
				 */
				dtack <= 1'b1;
				state <= TermWait;
			end

			CYCLE_XLATE_READ: begin
				if (TranslationError) begin
					ErrorReg <= TranslationError;
					mmu_fault <= 1'b1;
					state <= TermWait;
				end
				else begin
					PME_update <= 1'b1;
					TranslationValid <= 1'b1;
					AccessValid <= 1'b1;
					state <= TermWaitRMW0;
				end
			end

			CYCLE_XLATE_WRITE: begin
				/*
				 * A translated write cycle is basically
				 * the same as a translated read cycle,
				 * except that we don't have to check
				 * for R-M-W.
				 */
				if (TranslationError) begin
					ErrorReg <= TranslationError;
					mmu_fault <= 1'b1;
				end
				else begin
					PME_update <= 1'b1;
					TranslationValid <= 1'b1;
					AccessValid <= 1'b1;
				end
				state <= TermWait;
			end

			default: begin
				/*
				 * Do nothing for the default case of
				 * "unrecognized bus cycle".  The system's
				 * bus cycle timeout logic will kick in
				 * and signal the appropriate bus error.
				 */
				state <= Idle;
			end
			endcase
		end

		WriteContext: begin
			if (UDS_s) begin
				ContextReg <= DATA[5:0];
				dtack <= 1'b1;
				state <= TermWait;
			end
		end

		TermWaitRMW0: begin
			/*
			 * We watch for two different scenarios here.
			 * If we see /AS de-asserted first, then it's
			 * just a normal read.  Cool.  Otherwise, we
			 * watch for the target device to assert /DTACK
			 * and when wait for /DTACK to be de-asserted,
			 * at which time we will wait for RnW to go low
			 * and then perform another permission check for
			 * the R-M-W cycle.
			 */
			PME_update <= 1'b0;
			if (~AS_s) begin
				/*
				 * We know in this case that it's a
				 * translated read that passed the
				 * initial translation tests, so only
				 * need to go back to Idle state here.
				 */
				TranslationValid <= 1'b0;
				AccessValid <= 1'b0;
				state <= Idle;
			end
			/*
			 * Wait for /DTACK to be asserted by the target.
			 * Either it will be, and we move onto the next
			 * step, or the cycle ends for some other reason
			 * and we'll notice de-assertion of /AS on our
			 * next pass throug this state.
			 */
			else if (DTACK_s) begin
				state <= TermWaitRMW1;
			end
		end

		TermWaitRMW1: begin
			/*
			 * We've observed /DTACK being asserted by the
			 * target, now we need to wait for /DTACK to
			 * be de-asserted by the target.  When that
			 * happens, we need to re-gate the data strobe
			 * outputs (they should already be de-asserted
			 * by the CPU) in case we have to perform another
			 * permission check.
			 */
			if (~AS_s) begin
				TranslationValid <= 1'b0;
				AccessValid <= 1'b0;
				state <= Idle;
			end
			else if (~DTACK_s) begin
				AccessValid <= 1'b0;
				state <= TermWaitRMW2;
			end
		end

		TermWaitRMW2: begin
			/*
			 * Now we wait to see if this is really an
			 * R-M-W cycle.  If it is, /AS will remain
			 * asserted while RnW will go low.  If we
			 * obvserve that happening, then we need to
			 * perform another permission check.
			 */
			if (~AS_s) begin
				TranslationValid <= 1'b0;
				AccessValid <= 1'b0;
				state <= Idle;
			end
			else if (~RnW_s) begin
				if (TranslationError) begin
					ErrorReg <= TranslationError;
					mmu_fault <= 1'b1;
				end
				else begin
					PME_update <= 1'b1;
					AccessValid <= 1'b1;
				end
				state <= TermWait;
			end
		end

		TermWait: begin
			/*
			 * N.B. it's totally safe to update both PME_update
			 * and TranslationValid (which latches the PageMap
			 * indexes) in this same state because we know we'll
			 * visit this state at least twice per bus cycle
			 * before observing AS_s de-assert.
			 */
			PME_update <= 1'b0;
			if (~AS_s) begin
				/*
				 * If the Error register was consumed,
				 * clear it now, because we might update
				 * it again at the end of the next Idle
				 * state.
				 */
				if (ErrorReg_consumed) begin
					ErrorReg <= ERR_NONE;
					ErrorReg_consumed <= 1'b0;
				end

				TranslationValid <= 1'b0;
				AccessValid <= 1'b0;
				enable_data_out <= 1'b0;
				mmu_fault <= 1'b0;
				dtack <= 1'b0;
				state <= Idle;
			end
		end
		endcase
	end
end

endmodule

// Pin assignment for Yosys workflow.
//
//PIN: CHIP "mmu010" ASSIGNED TO AN TQFP100
//
//	=== CPU side of the chip ===
//
//PIN: MMU_DTACK	: 1
//PIN: MMU_FAULT	: 2
//PIN: nAS		: 5
//PIN: RnW		: 6
//PIN: nUDS		: 7
//PIN: nLDS		: 8
//PIN: nDTACK		: 9
//PIN: FC_0		: 10
//PIN: FC_1		: 12
//PIN: FC_2		: 13
//PIN: ADDR_0		: 14
//PIN: ADDR_1		: 16
//PIN: ADDR_2		: 17
//PIN: DATA_0		: 19
//PIN: DATA_1		: 20
//PIN: DATA_2		: 21
//PIN: DATA_3		: 22
//PIN: DATA_4		: 23
//PIN: DATA_5		: 24
//PIN: DATA_6		: 25
//PIN: DATA_7		: 27
//
//
//	=== SRAM / System side of the chip ===
//
//	== SegMap stuff ==
//PIN: nSMSEL		: 39
//PIN: nSM_WE		: 40
//PIN: nSM_UB		: 41
//PIN: nSM_LB		: 42
//PIN: CTX_0		: 44
//PIN: CTX_1		: 45
//PIN: CTX_2		: 46
//PIN: CTX_3		: 47
//PIN: CTX_4		: 48
//PIN: CTX_5		: 49
//PIN: SME_V		: 50
//
//	== PageMap stuff ==
//PIN: PMACC		: 52
//PIN: nPMUSEL		: 53
//PIN: nPMU_WE		: 54
//PIN: nPMU_UB		: 55
//PIN: nPMU_LB		: 56
//PIN: nPMLSEL		: 57
//PIN: nPML_WE		: 58
//PIN: nPML_UB		: 60
//PIN: nPML_LB		: 61
//PIN: nPM_LE		: 63
//PIN: PME_0		: 64
//PIN: PME_1		: 65
//PIN: PME_2		: 67
//PIN: PME_3		: 68
//PIN: PME_4		: 69
//PIN: PME_5		: 70
//PIN: PME_6		: 71
//PIN: PME_7		: 72
//
//	== General system outputs ==
//PIN: MMU_ADDR		: 75
//PIN: nAS_out		: 76
//PIN: nUDS_out		: 77
//PIN: nLDS_out		: 78
//
//
//	=== Top of the chip ===
//
//	== General control inputs ==
//PIN: nRST		: 89
//PIN: CLK		: 87
//PIN: MMU_EN		: 88
