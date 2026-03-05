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
 * XXX THIS IS A WORK-IN-PROGRESS, but essentially "complete" XXX
 *
 * Memory Management Unit for the Motorola 68010 CPU.  This implementation
 * is targeted at an Atmel/Microchip ATF1508AS-7AX100 7.5ns CPLD.
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
 * Address translation:
 *
 * +---------------+----------------------+
 * | Context [5:0] | Segment 9[VA 23:15]) |
 * +---------------+----------------------+--> 15-bit SegMap index --+
 *                                                                   |
 * +--------------------------==-------------------------------------+
 * |
 * +--> SegMap entry contains 15-bit PMEG number --+
 *                                                 |
 * +-----------------------------------------------+
 * |
 * +-------------------------+
 * |      PageMap index      |
 * | (PMEG << 3) | VA[14:12] |
 * +-------------------------+--> 18-bit PageMap index --+
 *                                                       |
 * +-----------------------------------------------------+
 * |
 * +--> PageMap entry (32-bits) (lower 16 bits are PFN) -----+
 *                                                           |
 * +---------------------------------------------------------+
 * |
 * +--> (PFN << 12) | VA[11:1] -> Physical address
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
 * Some random commentary:
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
 * The lower 4 bits of the upper SRAM are reserved for expansion of the
 * physical address space.  A transparent latch would be required to use
 * them so that they would remain stable while the upper SRAM is updated
 * with Mod/Ref information.  That latch would not need to be controlled
 * by any additional MMU logic; it could be controlled entirely by the
 * system /AS output already provided by the MMU.
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
 *	* 4x for the MMU output address (20 bits: A27..A12).  Direct
 *	  passthrough from the CPU's A23..A12 if the address is
 *	  untranslated, otherwise PA27..PA12 from the PageMap.
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
	input wire CLK40,	/* system CLK40 signal (4x CPU) */

	input wire nAS,		/* /AS from CPU */
	input wire RnW,		/* R/W from CPU */
	input wire nUDS,	/* /UDS from CPU */
	input wire nLDS,	/* /LDS from CPU */
	input wire nDTACK,	/* /DTACK as viewed by CPU */

	input wire [2:0] FC,	/* FC2..FC0 from CPU */

	input wire [2:0] ADDR,	/* A3..A1 from CPU */
	inout wire [7:0] DATA,	/* D15..D8 to/from CPU */

	input wire MMU_EN,	/* MMU enabled */
	input wire n_vme_berr,	/* /BERR output from VME */

	/* Important bits from the SegMap and PageMap entries. */
	input wire SME_V,	/* SegMap entry valid */
	inout wire [7:0] PME,	/* most significant PageMap entry byte */

	output wire [5:0] CTX,	/* Context to use for SegMap access */

	output wire PMACC,	/* CPU is accessing PageMap (mux control) */
	output wire MMU_ADDR,	/* MMU drives A27..A12 (mux control) */

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

	output wire CPU_CLK,	/* 10MHz CPU clock output */

	output wire n_berr_out,	/* /BERR output to CPU */

`ifdef BUILD_FOR_TEST
	output wire [7:0] bus_error_reg_test_out,
`endif

	output wire MMU_DTACK	/* drives open-drain inverter */
);

/*
 * Clock generation.  We receive the 40MHz clock from the oscillator as
 * input.  We clock ourselves at 40MHz, and provide the following clock
 * outputs:
 *
 * CPU_CLK (10MHz)	CLK40 / 4
 *
 * Because CPU_CLK is an even power-of-two and we want it to remain
 * running even when the CPU is driving a reset, its counter gets to
 * come up uninitialized.  The power-on-reset is elongated to ensure
 * many cycles of free-running before other devices begin consuming it.
 *
 * CPU_CLK (and the rest of the MMU) is clocked on the falling edge of
 * the 40MHz oscillator.
 */
reg[1:0] ClockDiv;
initial begin
	ClockDiv = 0;
end
always @(negedge CLK40) begin
	ClockDiv <= ClockDiv + 2'd1;
end
assign CPU_CLK = ClockDiv[1];

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
 * NOTE: ALL OF THESE SYNCHRONIZED INPUTS ARE ACTIVE-HIGH, EVEN IF THE
 * ORIGINAL SIGNAL IS ACTIVE-LOW.
 *
 * ALSO NOTE: The MC6800 peripheral interface is -- not supported --,
 * because I don't want to be bothered with dealing with both /DTACK
 * and /VPA.
 */
reg nAS1;
reg nAS_s;
reg nUDS1;
reg nUDS_s;
reg RnW1;
reg RnW_s;
always @(posedge CLK40, negedge nRST) begin
	if (~nRST) begin
		nAS1  <= 1'b1;
		nAS_s <= 1'b1;

		nUDS1  <= 1'b1;
		nUDS_s <= 1'b1;

		RnW1  <= 1'b1;
		RnW_s <= 1'b1;
	end
	else begin
		nAS1  <= nAS;
		nAS_s <= nAS1;

		nUDS1  <= nUDS;
		nUDS_s <= nUDS1;

		RnW1  <= RnW;
		RnW_s <= RnW1;
	end
end

/**************************** BUS ERROR REGISTER *****************************/

/*
 * Bus Error Register
 *
 * 00.0001	MMU Invalid translation
 * 00.0010	MMU Protection error
 * 00.0100	MMU Privelege error
 * 00.1000	(MMU reserved)
 * 01.0000	Bus cycle timed out
 * 10.0000	Bus error reported by VMEbus
 */

localparam BERR_NONE		= 6'b000000;
localparam BERR_MMUERR_UPPER	= 3'b000;
localparam BERR_TIMEOUT		= 6'b010000;
localparam BERR_VME		= 6'b100000;

reg [5:0] bus_error_reg;
initial begin
	bus_error_reg = BERR_NONE;
end

`ifdef BUILD_FOR_TEST
/* Provide a hook for testbench code to peek at the Bus Error Register. */
assign bus_error_reg_test_out = {2'b00, bus_error_reg};
`endif

/************************* PAGE MAP ENTRY LOGIC ******************************/

/*
 * We need to latch the upper byte of the PME when performing a
 * translation so that we can update the MOD and REF bits upon
 * a successful bus cycle.  This happens continuously in the bus
 * cycle state machine while we're waiting to observe the assertion
 * of the synchronized copy of /AS (see S_IDLE:CYCLE_NONE).  We
 * also define shorthand for the important bits.
 */
reg [7:0] PME_copy;
wire PME_V = PME_copy[7];	/* valid bit */
wire PME_W = PME_copy[6];	/* write bit */
wire PME_K = PME_copy[5];	/* kernel bit */

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
 * We latch the PageMap index while we consider the translation valid
 * so that the index remains stable while we update the MOD and REF bits.
 *
 * The /LE input on the 74'373 is high when transparent, and low when
 * latched.
 *
 * We can just use our gated /AS output for this purpose.  In a future
 * version of this MMU, we could simply use the gated /AS output directly,
 * or maybe we just skip the PME index latch completely?
 */
assign nPM_LE = nAS_out;

/************************** MMU ADDRESS DECODING *****************************/

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
 *
 * N.B. "regular space" is one in which FC[1] ^ FC[0] -> 1
 */
wire RegularSpace = (FC[1] ^ FC[0]);
wire ControlSpace = (FC == 3'd4);
wire KernelAcc    = FC[2];

/*
 * ADDR[2:0] (CPU A3..A1) indicates which part of the Control space is
 * being accessed.
 *
 *                          selector bits
 *	                         vvv
 *      xxxx.xxxx xxxx.xxxx xxxx.000x   non-MMU control space (ignored)
 *      SSSS.SSSS Sxxx.xxxx xxxx.0010   SegMap entry for Context 0
 *      SSSS.SSSS Sxxx.xxxx xxxx.0100   SegMap entry (relative to Context Reg)
 *      xxxx.xxxx xxxx.xxxx xxxx.0110   Context Register (byte)
 *      xxPP.PPPP PPPP.PPPP Pppp.1000   PageMap entry (upper word)
 *      xxPP.PPPP PPPP.PPPP Pppp.1010   PageMap entry (lower word)
 *        ^^^^^^^^^^^^^^^^^^^|||
 *                 PMEG      |||
 *                           ^^^
 *                    Entry within PMEG
 *      xxxx.xxxx xxxx.xxxx xxxx.1100   Bus Error Register (byte)
 *      xxxx.xxxx xxxx.xxxx xxxx.1110   (unused; reserved)
 *
 * The index for SegMap access is placed where it is because those are the
 * address bits that would be normally connected to select the SegMap index
 * for normal address translation; this saves having to use a mux for those
 * address bits.
 *
 * Access rules:
 *
 * ==> Context and Bus Error registers are 8-bit on even addresses
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
localparam MMUADDR_BusErrorReg	= 3'd6;

wire SegMap0Sel     = ControlSpace && (ADDR == MMUADDR_SegMap0);
wire SegMapSel      = ControlSpace && (ADDR == MMUADDR_SegMap);
wire PageMapUSel    = ControlSpace && (ADDR == MMUADDR_PageMapU);
wire PageMapLSel    = ControlSpace && (ADDR == MMUADDR_PageMapL);
wire ContextRegSel  = ControlSpace && (ADDR == MMUADDR_ContextReg);
wire BusErrorRegSel = ControlSpace && (ADDR == MMUADDR_BusErrorReg);

/*
 * From the MMU's perspective, CPU access to any of the MMU's SRAMs
 * is the same.  MapSel is to simplify the bus cycle state machine
 * logic, and the rest is all handled with combinatorial logic below.
 */
wire MapSel = (SegMap0Sel || SegMapSel || PageMapUSel || PageMapLSel);

/********************** SEGMENT MAP CONTROL LOGIC ****************************/

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

/*********************** PAGE MAP CONTROL LOGIC ******************************/

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

/********************** ADDRESS TRANSLATION LOGIC ****************************/

/*
 * If the MMU is translating an address, then it gets to drive the
 * system address bus (well, A31..A12, anyway).  This output signal
 * controls those muxes.
 */
wire Translate  = (RegularSpace & MMU_EN);
assign MMU_ADDR = Translate;

/*
 * Context output to SegMap is hard-wired to 0 for translated kernel
 * accesses or when accessing the SegMap via the dedicated kernel SegMap
 * address range.
 */
reg [5:0] ContextReg;
assign CTX = (SegMap0Sel || (RegularSpace & KernelAcc)) ? 6'd0 : ContextReg;

/*
 * Compute the translation error continuously.  It will be latched into
 * bus_error_reg when needed.
 *
 * Translation is valid when both SME_V and PME_V are set.
 *
 * Privilege check is OK if PME_K is not set -OR- if kernel access is
 * being performed.
 *
 * Protection check is OK is it's a read operation -OR- if PME_W is set.
 *
 * When a translation error is detected in the bus cycle state machine
 * (right at the beginning of the cycle, before the data strobe outputs
 * are un-gated), it's latched into the Bus Error Register and the state
 * machine signals a bus error to the CPU.  The bits are:
 *
 *	001	Invalid translation
 *	010	Protection error
 *	100	Privelege error
 *
 * An additional bit is reserved in the Bus Error Register for future
 * use (maybe adding an EXEC protection failure?)
 *
 * N.B. the MMU guarantees that PROT and PRIV will not be set if INV is
 * set, and PROT will not be set if PRIV is set.
 */
wire TransOK = (SME_V & PME_V);
wire PrivOK  = (KernelAcc | ~PME_K);
wire ProtOK  = (RnW | PME_W);

wire TranslationValid = (TransOK & PrivOK & ProtOK);
wire [2:0] TranslationError =
    {(~PrivOK & TransOK), (~ProtOK & TransOK & PrivOK), ~TransOK};

/*
 * Gate the /AS, /UDS, and /LDS signals that go to the system on address
 * translation success.
 *
 * We do this by OR'ing in:
 *
 *	(nAS_s | (Translate & ~TranslationValid))
 *
 * Here's how that works:
 *
 *	- Always gate on the synchronized copy of /AS.  Even when the
 *	  MMU is not involved in the bus cycle (either it's a non-tranlated
 *	  space or the MMU is disabled), there are muxes on the address
 *	  outputs, and delaying the assertion of /AS as observed by the
 *	  rest of the system allows for some switching and propagation delay
 *	  through those muxes.
 *
 *	- Translate is 0 if we have a not-translated-space or the MMU
 *	  is disabled.  In this scenario, TranslationValid will be ignored.
 *
 *	- While we are waiting for the synchronized /AS input signal,
 *	  combinatorial logic is continuously computing TranslationValid.
 *	  Once we observe the synchronized /AS assertion, we are guaranteed
 *	  that TranslationValid will be stable.  Inverting TranslationValid
 *	  will result in a 1 if the translation is *not* valid, which
 *	  will keep the strobes gated.
 *
 *	- In the case of an R-M-W cycle, RnW will transition from high
 *	  to low, which, since it's continously computed using combinatorial
 *	  logic, will update TranslationValid, re-gating the strobes if the
 *	  PME indicates a read-only page.  The bus cycle state machine is
 *	  watching for the transition of a synchronized copy of RnW and
 *	  and if that transition is observed, will consult TranslationValid
 *	  again and signal an MMU bus error if warranted.  However, even with
 *	  the synchronizer delay, the signal will be gated correctly due to
 *	  TranslationValid's continuous computation.
 */
wire strobe_gate = (nAS_s | (Translate & ~TranslationValid));
assign nAS_out   = nAS  | strobe_gate;
assign nUDS_out  = nUDS | strobe_gate;
assign nLDS_out  = nLDS | strobe_gate;

/*************************** MMU REGISTER ACCESS *****************************/

/*
 * Logic for reading internal MMU registers.
 */
reg enable_data_out;
reg [7:0] data_out;
always @(*) begin
	case ({ContextRegSel,BusErrorRegSel})
	2'b10:	 data_out = {2'b00, ContextReg};
	2'b01:	 data_out = {2'b00, bus_error_reg};
	default: data_out = 8'hFF;
	endcase
end
assign DATA = (enable_data_out && ~nUDS) ? data_out : 8'bzzzzzzzz;

/*
 * Qual MMU_DTACK on the non-synchronized /AS input to ensure it de-asserts
 * as quickly as possbile.
 *
 * "AC ELECTRICAL SPECIFICATIONS - READ AND WRITE CYCLES" from the "M68000
 * Family Reference Manual" lists a minimum 0ns "/AS negated to /DTACK negated"
 * time (characteristic #28), as well as for "/AS negated to /BERR negated"
 * time (characteristic #30).
 *
 * Also qual with the combined /xDS signal so that we can assert our
 * internal dtack early.
 */
reg dtack;
assign MMU_DTACK = dtack & (~nUDS | ~nLDS) & ~nAS;

/************************ BUS CYCLE TIMEOUT LOGIC ****************************/

/*
 * Bus timer module for 68000 busses.
 *
 * This is pretty simple: It's a counter that counts CPU clock cycles
 * so long as it's instructed to do so (typically when /AS is asserted,
 * although the consumer of this module is allowed to pick any criteria
 * it desires).
 *
 * XXX Unfortunately, the Yosys workflow I'm using here requires me to
 * XXX directly include all functionality into a single module, so the
 * XXX code is replicated here (see rtl/bus_timer.v).
 */

wire bus_timer_enable = ~nAS;

localparam BUS_TIMER_INITIAL	=	6'd0;
localparam BUS_TIMER_TICK	=	6'd1;
localparam BUS_TIMER_TIMEOUT	=	6'd63;

reg [5:0] bus_timer;
initial begin
	bus_timer = BUS_TIMER_INITIAL;
end

always @(posedge CPU_CLK, negedge nRST) begin
	if (~nRST) begin
		bus_timer <= BUS_TIMER_INITIAL;
	end
	else if (~bus_timer_enable) begin
		bus_timer <= BUS_TIMER_INITIAL;
	end
	else if (bus_timer != BUS_TIMER_TIMEOUT) begin
		bus_timer <= bus_timer + BUS_TIMER_TICK;
	end
end

wire bus_timeout = (bus_timer == BUS_TIMER_TIMEOUT);

/************************* BUS CYCLE STATE MACHINE ***************************/

/*
 * Here we take care of reading (Context, Bus Error) and writing (Context)
 * internal MMU registers as well as performing address translations.
 *
 * It's worth noting that address translations largely happen on their
 * own by virtue of the driving of address lines into the SegMap and
 * PageMap SRAMs 
 */

wire bus_error_detected = (bus_timeout | ~n_vme_berr);

wire [5:0] Cycle =
    {nAS_s, Translate, RnW, ContextRegSel, BusErrorRegSel, MapSel};

localparam CYCLE_NONE		= 6'b1xxxxx;
localparam CYCLE_RD_CONTEXT	= 6'b001100;
localparam CYCLE_WR_CONTEXT	= 6'b000100;
localparam CYCLE_RD_ERROR	= 6'b001010;
localparam CYCLE_WR_ERROR	= 6'b000010;
localparam CYCLE_RD_MAP		= 6'b001001;
localparam CYCLE_WR_MAP		= 6'b000001;
localparam CYCLE_RD_XLATE	= 6'b011000;
localparam CYCLE_WR_XLATE	= 6'b010000;

/*
 * State transitions:
 * S_IDLE
 *	S_WR_CONTEXT
 *	S_MMU_REG_TERM_WAIT
 *	S_ERROR_REG_TERM_WAIT
 *	S_RD_XLATE_TERM_WAIT
 *	S_WR_XLATE_TERM_WAIT
 *	S_MMU_ERROR
 *
 * S_WR_CONTEXT
 *	S_MMU_REG_TERM_WAIT
 *
 * S_MMU_REG_TERM_WAIT
 *	S_IDLE
 *
 * S_ERROR_REG_TERM_WAIT
 *	S_IDLE
 *
 * S_RD_XLATE_TERM_WAIT
 *	S_RD_XLATE_TERM_WAIT1
 *
 * S_RD_XLATE_TERM_WAIT1
 *	S_IDLE
 *	S_BUS_ERROR_DETECTED
 *	S_MMU_ERROR
 *	S_WR_XLATE_TERM_WAIT
 *
 * S_WR_XLATE_TERM_WAIT
 *	S_WR_XLATE_TERM_WAIT1
 *
 * S_WR_XLATE_TERM_WAIT1
 *	S_IDLE
 *	S_BUS_ERROR_DETECTED
 *
 * S_MMU_ERROR
 *	S_BUS_ERROR
 *
 * S_BUS_ERROR_DETECTED
 *	S_BUS_ERROR
 *
 * S_BUS_ERROR
 *	S_IDLE
 */
localparam S_IDLE			= 4'd0;
localparam S_WR_CONTEXT			= 4'd1;
localparam S_MMU_REG_TERM_WAIT		= 4'd2;
localparam S_ERROR_REG_TERM_WAIT	= 4'd3;
localparam S_RD_XLATE_TERM_WAIT		= 4'd4;
localparam S_RD_XLATE_TERM_WAIT1	= 4'd5;
localparam S_WR_XLATE_TERM_WAIT		= 4'd6;
localparam S_WR_XLATE_TERM_WAIT1	= 4'd7;
localparam S_MMU_ERROR			= 4'd8;
localparam S_BUS_ERROR_DETECTED		= 4'd9;
localparam S_BUS_ERROR			= 4'd10;

reg [3:0] state;
always @(negedge CLK40) begin
	if (~nRST) begin
		bus_error_reg <= BERR_NONE;

		ContextReg <= 6'd0;

		PME_update <= 1'b0;
		PME_copy <= 8'b0;

		enable_data_out <= 1'b0;
		dtack <= 1'b0;
		state <= S_IDLE;
	end
	else begin
		case (state)
		S_IDLE: begin
			/* Check for the beginning of a bus cycle. */
			casex (Cycle)
			CYCLE_NONE: begin
				/*
				 * Continuously grab a copy of the PageMap
				 * entry while we're waiting for nAS_s to
				 * be asserted.  By the time we've observed
				 * AS_s's assertion the address will have
				 * been stable on the bus for a sufficiently
				 * long time for the SegMap and PageMap
				 * SRAMs to result in a valid PME being
				 * present.
				 */
				PME_copy <= PME;
				state <= S_IDLE;
			end

			CYCLE_RD_CONTEXT: begin
				enable_data_out <= 1'b1;
				dtack <= 1'b1;
				state <= S_MMU_REG_TERM_WAIT;
			end

			CYCLE_WR_CONTEXT: begin
				/*
				 * We have to wait for the synchronized
				 * /UDS signal to be asserted.
				 */
				dtack <= 1'b1;
				state <= S_WR_CONTEXT;
			end

			CYCLE_RD_ERROR: begin
				enable_data_out <= 1'b1;
				dtack <= 1'b1;
				state <= S_ERROR_REG_TERM_WAIT;
			end

			CYCLE_WR_ERROR: begin
				/* We just ignore these writes. */
				dtack <= 1'b1;
				state <= S_MMU_REG_TERM_WAIT;
			end

			CYCLE_RD_MAP: begin
				dtack <= 1'b1;
				state <= S_MMU_REG_TERM_WAIT;
			end

			CYCLE_WR_MAP: begin
				dtack <= 1'b1;
				state <= S_MMU_REG_TERM_WAIT;
			end

			CYCLE_RD_XLATE: begin
				if (TranslationValid) begin
					PME_update <= 1'b1;
					state <= S_RD_XLATE_TERM_WAIT;
				end
				else begin
					state <= S_MMU_ERROR;
				end
			end

			CYCLE_WR_XLATE: begin
				/*
				 * A translated write cycle is basically
				 * the same as a translated read cycle,
				 * except that we don't have to check
				 * for R-M-W.
				 */
				if (TranslationValid) begin
					PME_update <= 1'b1;
					state <= S_WR_XLATE_TERM_WAIT;
				end
				else begin
					state <= S_MMU_ERROR;
				end
			end

			default: begin
				/*
				 * Mostly do nothing for the default case of
				 * "unrecognized bus cycle".  The system's
				 * bus cycle timeout logic will kick in
				 * and signal the appropriate bus error.
				 *
				 * This will generally be one of 3 things:
				 * - Interrupt Acknowledge cycle
				 * - Some other Control Space access that's
				 *   outside of the MMU.
				 * - Normal bus cycle with MMU disabled.
				 */
				if (bus_error_detected) begin
					state <= S_BUS_ERROR_DETECTED;
				end
			end
			endcase
		end

		S_WR_CONTEXT: begin
			if (~nUDS_s) begin
				ContextReg <= DATA[5:0];
				state <= S_MMU_REG_TERM_WAIT;
			end
		end

		S_MMU_REG_TERM_WAIT: begin
			if (nAS_s) begin
				dtack <= 1'b0;
				state <= S_IDLE;
			end
		end

		S_ERROR_REG_TERM_WAIT: begin
			/* Bus Error Register is reset after reading. */
			if (nAS_s) begin
				bus_error_reg <= BERR_NONE;
				dtack <= 1'b0;
				state <= S_IDLE;
			end
		end

		S_RD_XLATE_TERM_WAIT: begin
			PME_update <= 1'b0;
			state <= S_RD_XLATE_TERM_WAIT1;
		end

		S_RD_XLATE_TERM_WAIT1: begin
			/*
			 * We watch for two different scenarios here:
			 *
			 * 1. If we see /AS negated, then it's just a
			 *    normal read, cool.
			 *
			 * 2. If we see RnW go low, then we need to
			 *    perform another permission check because
			 *    it's an R-M-W cycle.
			 */
			if (bus_error_detected) begin
				state <= S_BUS_ERROR_DETECTED;
			end
			else if (nAS_s) begin
				state <= S_IDLE;
			end
			else if (~RnW_s) begin
				/*
				 * This is an R-M-W cycle; perform another
				 * permission check.
				 */
				if (TranslationValid) begin
					/* Mod bit needs updating. */
					PME_update <= 1'b1;
					state <= S_WR_XLATE_TERM_WAIT;
				end
				else begin
					state <= S_MMU_ERROR;
				end
			end
		end

		S_WR_XLATE_TERM_WAIT: begin
			PME_update <= 1'b0;
			state <= S_WR_XLATE_TERM_WAIT1;
		end

		S_WR_XLATE_TERM_WAIT1: begin
			if (bus_error_detected) begin
				state <= S_BUS_ERROR_DETECTED;
			end
			else if (nAS_s) begin
				state <= S_IDLE;
			end
		end

		S_MMU_ERROR: begin
			bus_error_reg <= {BERR_MMUERR_UPPER,
			    TranslationError};
			state <= S_BUS_ERROR;
		end

		S_BUS_ERROR_DETECTED: begin
			if (~n_vme_berr) begin
				bus_error_reg <= BERR_VME;
			end
			else begin
				bus_error_reg <= BERR_TIMEOUT;
			end
			state <= S_BUS_ERROR;
		end

		S_BUS_ERROR: begin
			if (nAS_s) begin
				state <= S_IDLE;
			end
		end

		endcase
	end
end

/* Signal /BERR to the CPU if the state machine says so. */
assign n_berr_out = ~(state == S_BUS_ERROR) | nAS;

endmodule

// Pin assignment for Yosys workflow.
//
//PIN: CHIP "mmu010" ASSIGNED TO AN TQFP100
//
//	=== CPU side of the chip ===
//
//PIN: MMU_DTACK	: 1
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
//PIN: n_berr_out	: 28
//
//
//	=== SRAM / System side of the chip ===
//
//	== SegMap stuff ==
//PIN: nSMSEL		: 37
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
//PIN: n_vme_berr	: 85
//PIN: nRST		: 89
//PIN: CLK40		: 87
//PIN: MMU_EN		: 88
//
//	== Clock outputs ==
//PIN: CPU_CLK		: 100
