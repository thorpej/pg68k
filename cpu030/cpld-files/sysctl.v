/*
 * Copyright (c) 2025 Jason R. Thorpe.
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
 * Playground 68030 System Controller.
 *
 * This contains most of the random glue logic, top-level address
 * decoding, fast-RAM memory interface, and bus error generation.
 */

module sysctl(
	input wire nRST,
	input wire DRAM_CLK,

	input wire nAS,
	input wire nDS,
	input wire RnW,
	input wire [1:0] SIZ,

	input wire [2:0] FC,
	input wire [31:0] ADDR,

	output wire STERM,		/* external open-drain inv */
	output wire CI,			/* external open-drain inv */
	output wire BERR,		/* external open-drain inv */
	output wire [1:0] DSACK,	/* external open-drain inv */

	output wire nROMSEL,
	output wire nDEVSEL,
	output wire nMMIOSEL,
	output wire [3:0] nDRAMSEL,
	output wire nFPUSEL,
	output wire nIACKSEL,

	output wire nINTCSEL,
	output wire nDUARTSEL,
	output wire nTMRSEL,
	output wire nATASEL,
	output wire nI2CSEL,

	output wire CPU_CLK,
	output wire CLK_6_25,

	output wire nFRAM_RD,
	output wire [3:0] nFRAM_WR,

	output wire nIORD,
	output wire nIOWR,

	output wire RESET
);

/*
 * Clock generation.  We receive the DRAM clock as input (50MHz).  We
 * produce the following clock outputs:
 *
 * - CPU_CLK (25MHz)	DRAM_CLK / 2
 * - CLK_6_25 (6.25MHz)	DRAM_CLK / 8
 */
reg [2:0] ClockDivider;
always @(posedge DRAM_CLK, negedge nRST) begin
	if (~nRST)
		ClockDivider <= 3'd0;
	else
		ClockDivider <= ClockDivider + 3'd1;
end
assign CPU_CLK = ClockDivider[0];
assign CLK_6_25 = ClockDivider[2];

/*
 * We count the first 4 bus cycles after a /RESET occurs, and use that
 * to ensure the ROM is selected for the reset vector fetch.
 *
 * N.B. We want the state to advance **after** the bus cycle is over,
 * thus we clock on the rising edge of nAS.
 */
reg [1:0] BootState;
always @(posedge nAS, negedge nRST) begin
	if (~nRST)
		BootState <= 2'd0;
	else begin
		if (BootState != 2'd3)
			BootState <= BootState + 2'd1;
	end
end
wire ResetVecFetch;
assign ResetVecFetch = (BootState != 2'd3);

/*
 * Bus error generation.
 *
 * This will drive the BERR output to the CPU if a bus cycle fails
 * to terminate after 64 CPU clock cycles.
 *
 * If /RESET is asserted or there is not active bus cycle (/AS not asserted),
 * then we reset the counter to 0.
 *
 * No other termination indication is required other than de-assertion of
 * /AS; when bus cycles terminate by whatever means, the CPU de-asserts /AS
 * before the rising edge of the CPU clock cycle that begins the next bus
 * cycle's S0.
 */
reg [5:0] BerrState;
always @(posedge CPU_CLK, negedge nRST) begin
	if (~nRST)
		BerrState <= 6'd0;
	else begin
		if (nAS)
			BerrState <= 6'd0;
		else if (BerrState != 6'd63)
			BerrState <= BerrState + 6'd1;
	end
end
assign BERR = (BerrState == 6'd63);

/*
 * Function bits for address space encodings:
 *
 * FC2   FC1   FC0
 *  0     0     0       (Undefined, reserved)
 *  0     0     1       User Data Space
 *  0     1     0       User Program Space
 *  0     1     1       (Undefined, reserved)
 *  1     0     0       (Undefined, reserved)
 *  1     0     1       Supervisor Data Space
 *  1     1     0       Supervisor Program Space
 *  1     1     1       CPU Space
 *
 * Note that {User,Supervisor}{Data,Program} -> FC1 xor FC0 -> 1
 */
wire SpaceNormal = (FC[1] ^ FC[0]);
wire SpaceCPU = (FC == 3'd7);

localparam QUAL_nAS	= 2'bx0;
localparam QUAL_nCLK	= 2'b0x;
wire [1:0] AddrQual = {CPU_CLK, nAS};

localparam SPC_CPU	= 2'b10;
localparam SPC_NORM	= 2'b01;
wire [1:0] AddrSpace = {SpaceCPU, SpaceNormal};

/*
 * Top-level address decoding:
 *
 * $FFF0.0000 - $FFFF.FFFF      System ROM (1MB)
 * $FFE0.0000 - $FFEF.FFFF      Peripheral device space (1MB)
 * $FF80.0000 - $FFBF.FFFF      Fast RAM (4MB)
 * $8000.0000 - $BFFF.FFFF      Memory mapped I/O space (1GB)
 * $0000.0000 - $3FFF.FFFF      DRAM (1GB, as 4x256MB)
 *
 * The DRAM region is arranged as up-to-four discrete 256MB banks, each
 * controlled by a DRAMCTL.
 *
 * MMIO space is further decoded externally, qualified by /MMIOSEL.
 *
 * The peripheral device space is mainly on-board peripherals (UARTs, timer,
 * etc.).  Some of this address is further decoded below, with the remaining
 * being externally decoded and qualified by /DEVSEL.
 */

/*			          Address bits
 *			      31               13
 *			      |                 |			*/
localparam REGION_ROM	= 19'b111111111111xxxxxxx;
localparam REGION_DEV	= 19'b111111111110xxxxxxx;
/*         REGION_FRAM	= 19'b1111111110xxxxxxxxx; Handled below.	*/
localparam REGION_MMIO	= 19'b10xxxxxxxxxxxxxxxxx;
localparam REGION_DRAM0	= 19'b0000xxxxxxxxxxxxxxx;
localparam REGION_DRAM1	= 19'b0001xxxxxxxxxxxxxxx;
localparam REGION_DRAM2	= 19'b0010xxxxxxxxxxxxxxx;
localparam REGION_DRAM3	= 19'b0011xxxxxxxxxxxxxxx;

	/* (CPU space) ACCTYPE = 0x02 (coproc), CPID = 0x01 */
localparam REGION_FPU	= 19'bxxxxxxxxxxxx0010001;

	/* (CPU space) ACCTYPE = 0x0f (IACK) */
localparam REGION_IACK	= 19'bxxxxxxxxxxxx1111xxx;

localparam RV_Y		= 1'b1;
localparam RV_N		= 1'b0;
localparam RV_X		= 1'bx;

localparam SEL_NONE	= 9'b111111111;
localparam SEL_ROM	= 9'b111111110;
localparam SEL_DEV	= 9'b111111101;
localparam SEL_MMIO	= 9'b111111011;
localparam SEL_DRAM0	= 9'b111110111;
localparam SEL_DRAM1	= 9'b111101111;
localparam SEL_DRAM2	= 9'b111011111;
localparam SEL_DRAM3	= 9'b110111111;
localparam SEL_FPU	= 9'b101111111;
localparam SEL_IACK	= 9'b011111111;

wire nDEVSELx;
wire nROMSELx;
reg [8:0] SelectOutputs;
always @(*) begin
	casex ({AddrQual, AddrSpace, ResetVecFetch, ADDR[31:13]})
	{QUAL_nAS,  SPC_NORM, RV_X, REGION_ROM}:   SelectOutputs = SEL_ROM;

	{QUAL_nAS,  SPC_NORM, RV_X, REGION_DEV}:   SelectOutputs = SEL_DEV;

	{QUAL_nAS,  SPC_NORM, RV_X, REGION_MMIO}:  SelectOutputs = SEL_MMIO;

	{QUAL_nAS,  SPC_NORM, RV_Y, REGION_DRAM0}: SelectOutputs = SEL_ROM;
	{QUAL_nAS,  SPC_NORM, RV_N, REGION_DRAM0}: SelectOutputs = SEL_DRAM0;
	{QUAL_nAS,  SPC_NORM, RV_X, REGION_DRAM1}: SelectOutputs = SEL_DRAM1;
	{QUAL_nAS,  SPC_NORM, RV_X, REGION_DRAM2}: SelectOutputs = SEL_DRAM2;
	{QUAL_nAS,  SPC_NORM, RV_X, REGION_DRAM3}: SelectOutputs = SEL_DRAM3;

	{QUAL_nCLK, SPC_CPU,  RV_X, REGION_FPU}:   SelectOutputs = SEL_FPU;
	{QUAL_nAS,  SPC_CPU,  RV_X, REGION_FPU}:   SelectOutputs = SEL_FPU;

	{QUAL_nAS,  SPC_CPU,  RV_X, REGION_IACK}:  SelectOutputs = SEL_IACK;

	default:                                   SelectOutputs = SEL_NONE;
	endcase
end
assign {nIACKSEL, nFPUSEL, nDRAMSEL, nMMIOSEL, nDEVSELx, nROMSELx}
    = SelectOutputs;

/*
 * ROM only selected for read cycles.  /ROMSEL can be connected to /CS
 * and /OE on both of the SST39SF040-70s.
 */
assign nROMSEL = nROMSELx | ~RnW;

/*
 * Further qualify the DEV space:
 *
 * 0.000x	TL16C2552 DUART
 * 0.001x	CP82C54-10Z timer
 * 0.002x	ATA registers
 * 0.010x	PCF8584 I2C controller
 * F.FFFx	Interrupt controller
 */
localparam DEV_DUART	= 20'h0000x;
localparam DEV_TMR	= 20'h0001x;
localparam DEV_ATA	= 20'h0002x;
localparam DEV_I2C	= 20'h0010x;
localparam DEV_INTC	= 20'hFFFFx;

localparam DSEL_NONE	= 5'b11111;
localparam DSEL_DUART	= 5'b01111;
localparam DSEL_TMR	= 5'b10111;
localparam DSEL_ATA	= 5'b11011;
localparam DSEL_I2C	= 5'b11101;
localparam DSEL_INTC	= 5'b11110;

reg [2:0] DevSelectOutputs;
always @(*) begin
	casex ({nDEVSELx, nDS, ADDR[19:0]})
	{1'b0, 1'bx, DEV_DUART}: DevSelectOutputs = DSEL_DUART;
	{1'b0, 1'bx, DEV_TMR}:   DevSelectOutputs = DSEL_TMR;
	{1'b0, 1'bx, DEV_INTC}:  DevSelectOutputs = DSEL_INTC;
	{1'b0, 1'bx, DEV_ATA}:   DevSelectOutputs = DSEL_ATA;
	/*
	 * Also need to qual /I2CSEL on /DS because we want to
	 * ensure that the PCF8584 comes up in 68000 interface
	 * mode.
	 */
	{1'b0, 1'b0, DEV_I2C}:   DevSelectOutputs = DSEL_I2C;
	default:                 DevSelectOutputs = DSEL_NONE;
	endcase
end
assign {nDUARTSEL, nTMRSEL, nATASEL, nI2CSEL, nINTCSEL}
    = DevSelectOutputs;

/*
 * Assert the external /DEVSEL signal if we don't match any devices
 * decoded internally.
 */
assign nDEVSEL = ~(~nDEVSELx && (DevSelectOutputs == DSEL_NONE));

localparam REGION_FRAM = 10'b1111111110;

/* 
 *           nFRAM_WR[3:0] -----++
 *                nFRAM_RD ---+ ||
 *                   STERM --+| ||
 *                           ||-++-				*/
localparam FRS_NONE	= 6'b011111;

localparam FRS_RD	= 6'b101111;

localparam FRS_WR1_0	= 6'b110111;
localparam FRS_WR1_1	= 6'b111011;
localparam FRS_WR1_2	= 6'b111101;
localparam FRS_WR1_3	= 6'b111110;

localparam FRS_WR2_0	= 6'b110011;
localparam FRS_WR2_1	= 6'b111001;
localparam FRS_WR2_2	= 6'b111100;
localparam FRS_WR2_3	= 6'b111110;

localparam FRS_WR3_0	= 6'b110001;
localparam FRS_WR3_1	= 6'b111000;
localparam FRS_WR3_2	= 6'b111100;
localparam FRS_WR3_3	= 6'b111110;

localparam FRS_WR4_0	= 6'b110000;
localparam FRS_WR4_1	= 6'b111000;
localparam FRS_WR4_2	= 6'b111100;
localparam FRS_WR4_3	= 6'b111110;

reg [5:0] FRSOutputs;
always @(*) begin
	casex ({nAS, AddrSpace, ADDR[31:21], RnW, SIZ[1:0], ADDR[1:0]})
	/*
	 * Byte enables, from Table 7-4 in the 68030 User's Manual.
	 * N.B. for reads, we enable all bytes.
	 */

	/* reads */
	{1'b0, SPC_NORM, REGION_FRAM, 5'b1xxxx}:  FRSOutputs = FRS_RD;

	/* byte writes */
	{1'b0, SPC_NORM, REGION_FRAM, 5'b00100}:  FRSOutputs = FRS_WR1_0;
	{1'b0, SPC_NORM, REGION_FRAM, 5'b00101}:  FRSOutputs = FRS_WR1_1;
	{1'b0, SPC_NORM, REGION_FRAM, 5'b00110}:  FRSOutputs = FRS_WR1_2;
	{1'b0, SPC_NORM, REGION_FRAM, 5'b00111}:  FRSOutputs = FRS_WR1_3;

	/* word writes */
	{1'b0, SPC_NORM, REGION_FRAM, 5'b01000}:  FRSOutputs = FRS_WR2_0;
	{1'b0, SPC_NORM, REGION_FRAM, 5'b01001}:  FRSOutputs = FRS_WR2_1;
	{1'b0, SPC_NORM, REGION_FRAM, 5'b01010}:  FRSOutputs = FRS_WR2_2;
	{1'b0, SPC_NORM, REGION_FRAM, 5'b01011}:  FRSOutputs = FRS_WR2_3;

	/* 3 byte writes */
	{1'b0, SPC_NORM, REGION_FRAM, 5'b01100}:  FRSOutputs = FRS_WR3_0;
	{1'b0, SPC_NORM, REGION_FRAM, 5'b01101}:  FRSOutputs = FRS_WR3_1;
	{1'b0, SPC_NORM, REGION_FRAM, 5'b01110}:  FRSOutputs = FRS_WR3_2;
	{1'b0, SPC_NORM, REGION_FRAM, 5'b01111}:  FRSOutputs = FRS_WR3_3;

	/* long word writes */
	{1'b0, SPC_NORM, REGION_FRAM, 5'b00000}:  FRSOutputs = FRS_WR4_0;
	{1'b0, SPC_NORM, REGION_FRAM, 5'b00001}:  FRSOutputs = FRS_WR4_1;
	{1'b0, SPC_NORM, REGION_FRAM, 5'b00010}:  FRSOutputs = FRS_WR4_2;
	{1'b0, SPC_NORM, REGION_FRAM, 5'b00011}:  FRSOutputs = FRS_WR4_3;

	default:                                  FRSOutputs = FRS_NONE;
	endcase
end
assign {STERM, nFRAM_RD, nFRAM_WR} = FRSOutputs;

/* Inhibit cache if fetching the reset vector or accessing DEV space. */
assign CI = (ResetVecFetch || ~nDEVSELx);

/*
 * Bus cycle state machine.
 *
 * Generate /IORD, /IOWR, and DSACKx based on the selected device and
 * its timing constraints.
 *
 * Note we only do this for devices that don't do the 68000 bus protocol
 * natively, which means: the ROM, the DUART, and the timer.
 */
reg [1:0] dsack;
localparam PORT_1	= 2'b01;
localparam PORT_2	= 2'b10;

reg [3:0] BCState;
localparam BC_Idle	= 4'd0;
localparam BC_B1W1	= 4'd1;	/* bytes: 1, waits: 1 */
localparam BC_B2W1	= 4'd2;	/* bytes: 2, waits: 1 */
localparam BC_TMR_RD1	= 4'd3;
localparam BC_TMR_RW2	= 4'd4;
localparam BC_TMR_RW3	= 4'd5;
localparam BC_TMR_RW4	= 4'd6;
localparam BC_TMR_F1	= 4'd7;
localparam BC_TMR_F2	= 4'd8;
localparam BC_TMR_F3	= 4'd9;
localparam BC_Finish	= 4'd10;

/*
 * Some devides need to delay the I/O strobes to meet setup times.
 * See the state machine below for commentary on when they're needed.
 */
reg io_strobe_d;
wire io_strobe_d_needed = (~nTMRSEL & RnW);

always @(posedge CPU_CLK, negedge nRST) begin
	if (~nRST) begin
		BCState <= 2'd0;
		io_strobe_d <= 1'b0;
		dsack <= 2'b00;
	end
	else begin
		case (BCState)
		BC_Idle: begin
			if (~nROMSEL) begin
				/*
				 * /ROMSEL is doing all the work required,
				 * but we need 1 wait state to meet the
				 * 70ns access time.
				 */
				BCState <= BC_B2W1;
			end
			else if (~nDUARTSEL & RnW) begin
				/* DUART read: no wait states */
				dsack <= PORT_1;
				BCState <= BC_Finish;
			end
			else if (~nDUARTSEL & ~RnW) begin
				/* DUART write: 1 wait state */
				/*
				 * XXX Might be unnecessary; revisit
				 * UART timing diagram
				 */
				BCState <= BC_B1W1;
			end
			else if (~nTMRSEL & RnW) begin
				/*
				 * The 82C54(-10) is a mess.  The
				 * address bus needs to be stable
				 * for 25ns before /IORD, so we have
				 * to delay the strobe by one cycle.
				 *
				 * Then we need a 95ns /IORD pulse width,
				 * which means 3(2?) wait states before we
				 * can assert DSACK.
				 *
				 * **THEN** we need to wait for 165ns (4!
				 * more cycles) for the "recovery time"
				 * before we can come around again to service
				 * something else (because there might be a
				 * back-to-back timer access).  But, we can
				 * shave one of those off because we delay
				 * the strobe artificially on read and it's
				 * delayed naturally (by the CPU) on write.
				 */
				BCState <= BC_TMR_RD1;
			end
			else if (~nTMRSEL & ~RnW & ~nDS) begin
				/*
				 * Ironically, writes are not that bad.
				 * There are no weird setup times before
				 * the strobe, but we need to hold it for
				 * the same 3 clock cycles and observe the
				 * same recovery time as in the read case.
				 */
				BCState <= BC_TMR_RW2;
			end
		end

		BC_B1W1: begin
			dsack <= PORT_1;
			BCState <= BC_Finish;
		end

		BC_B2W1: begin
			dsack <= PORT_2;
			BCState <= BC_Finish;
		end

		BC_TMR_RD1: begin
			io_strobe_d <= 1'b1;
			BCState <= BC_TMR_RW2;
		end

		BC_TMR_RW2: begin
			BCState <= BC_TMR_RW3;
		end

		BC_TMR_RW3: begin
			BCState <= BC_TMR_RW4;
		end

		BC_TMR_RW4: begin
			dsack <= PORT_1;
			BCState = BC_TMR_F1;
		end

		BC_TMR_F1: begin
			if (nDS) begin
				io_strobe_d <= 1'b0;
				dsack <= 2'b00;
				BCState <= BC_TMR_F2;
			end
		end

		BC_TMR_F2: begin
			BCState <= BC_TMR_F3;
		end

		BC_TMR_F3: begin
			BCState <= BC_Idle;
		end

		BC_Finish: begin
			if (nDS) begin
				io_strobe_d <= 1'b0;
				dsack <= 2'b00;
				BCState <= BC_Idle;
			end
		end
		endcase
	end
end

/*
 * Qual with /DS from the CPU so they de-assert immediately.
 */
assign DSACK = dsack & {~nDS, ~nDS};

/*
 * If we've selected an external device that we're not decoding ourselves,
 * always let /IORD and /IOWR be asserted.  Otherwise, they are gated based
 * on the requirements of the device.
 */
wire io_strobe = (~nDEVSELx & ~io_strobe_d_needed) | io_strobe_d;
assign nIORD = ~(io_strobe & ~nDS & RnW);
assign nIOWR = ~(io_strobe & ~nDS & ~RnW);

/* Generate a positive-level reset signal for anything that needs it. */
assign RESET = ~nRST;

endmodule

// Pin assignment for Yosys workflow.
//
//PIN: CHIP "sysctl" ASSIGNED TO AN TQFP100
//
//     === Inputs ===
//PIN: RnW		: 5
//PIN: SIZ_0		: 6
//PIN: SIZ_1		: 7
//PIN: ADDR_0		: 8
//PIN: ADDR_1		: 9
//PIN: ADDR_2		: 10
//PIN: ADDR_3		: 12
//PIN: ADDR_4		: 13
//PIN: ADDR_5		: 14
//PIN: ADDR_6		: 16
//PIN: ADDR_7		: 17
//PIN: ADDR_8		: 19
//PIN: ADDR_9		: 20
//PIN: ADDR_10		: 21
//PIN: ADDR_11		: 22
//PIN: ADDR_12		: 23
//PIN: ADDR_13		: 24
//PIN: ADDR_14		: 25
//PIN: ADDR_15		: 27
//PIN: ADDR_16		: 28
//PIN: ADDR_17		: 29
//PIN: ADDR_18		: 30
//PIN: ADDR_19		: 31
//PIN: ADDR_20		: 32
//PIN: ADDR_21		: 33
//PIN: ADDR_22		: 35
//PIN: ADDR_23		: 36
//PIN: ADDR_24		: 37
//PIN: ADDR_25		: 40
//PIN: ADDR_26		: 41
//PIN: ADDR_27		: 42
//PIN: ADDR_28		: 44
//PIN: ADDR_29		: 45
//PIN: ADDR_30		: 46
//PIN: ADDR_31		: 47
//PIN: FC_0		: 48
//PIN: FC_1		: 49
//PIN: FC_2		: 50
//PIN: nAS		: 87
//PIN: nDS		: 88
//PIN: nRST		: 89
//PIN: DRAM_CLK		: 90
//
//     === Outputs ===
//
//PIN: DSACK_0		: 1
//PIN: DSACK_1		: 2
//PIN: RESET		: 52
//PIN: nIORD		: 53
//PIN: nIOWR		: 54
//PIN: nI2CSEL		: 65
//PIN: nATASEL		: 67
//PIN: nTMRSEL		: 68
//PIN: nDUARTSEL	: 69
//PIN: nINTCSEL		: 70
//PIN: nDRAMSEL_0	: 71
//PIN: nDRAMSEL_1	: 72
//PIN: nDRAMSEL_2	: 75
//PIN: nDRAMSEL_3	: 76
//PIN: nFRAM_WR_0	: 77
//PIN: nFRAM_WR_1	: 78
//PIN: nFRAM_WR_2	: 79
//PIN: nFRAM_WR_3	: 80
//PIN: nFRAM_RD		: 81
//PIN: nMMIOSEL		: 83
//PIN: nDEVSEL		: 84
//PIN: nROMSEL		: 85
//PIN: nFPUSEL		: 92
//PIN: nIACKSEL		: 93
//PIN: CLK_6_25		: 96
//PIN: CPU_CLK		: 97
//PIN: CI		: 98
//PIN: STERM		: 99
//PIN: BERR		: 100
