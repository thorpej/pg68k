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
	input wire CPU_CLK,

	input wire nAS,
	input wire nDS,
	input wire RnW,
	input wire [2:0] FC,
	input wire [1:0] SIZ,

	input wire [31:0] ADDR,

	output wire STERM,		/* external open-drain inv */
	output wire CI,			/* external open-drain inv */
	output wire nBERR,

	output wire nROMSEL,
	output wire nDEV8SEL,
	output wire nMMIOSEL,
	output wire nRAMSEL,
	output wire nFPUSEL,
	output wire nIACKSEL,

	output wire nFRAM_RD,
	output wire [3:0] nFRAM_WR,
);

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
 * This will drive the /BERR output to the CPU if a bus cycle fails
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
assign nBERR = (BerrState != 6'd63);

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
 * $FFE0.0000 - $FFEF.FFFF      8-bit peripheral space (1MB)
 * $FF80.0000 - $FFBF.FFFF      Fast RAM (4MB)
 * $8000.0000 - $BFFF.FFFF      Memory mapped I/O space (1GB)
 * $0000.0000 - $7FFF.FFFF      RAM (2GB)
 *
 * The bottom 3GB are futher decoded by external decoding logic
 * (qualified by /RAMSEL and /MMIOSEL).
 */

/*			          Address bits
 *			      31               13
 *			      |                 |			*/
localparam REGION_ROM	= 19'b111111111111xxxxxxx;
localparam REGION_DEV8	= 19'b111111111110xxxxxxx;
/*         REGION_FRAM	= 19'b1111111110xxxxxxxxx; Handled below.	*/
localparam REGION_MMIO	= 19'b10xxxxxxxxxxxxxxxxx;
localparam REGION_RAM	= 19'b0xxxxxxxxxxxxxxxxxx;

	/* (CPU space) ACCTYPE = 0x02 (coproc), CPID = 0x01 */
localparam REGION_FPU	= 19'bxxxxxxxxxxxx0010001;

	/* (CPU space) ACCTYPE = 0x0f (IACK) */
localparam REGION_IACK	= 19'bxxxxxxxxxxxx1111xxx;

localparam RV_Y		= 1'b1;
localparam RV_N		= 1'b0;
localparam RV_X		= 1'bx;

localparam SEL_NONE	= 6'b111111;
localparam SEL_ROM	= 6'b111110;
localparam SEL_DEV8	= 6'b111101;
localparam SEL_MMIO	= 6'b111011;
localparam SEL_RAM	= 6'b110111;
localparam SEL_FPU	= 6'b101111;
localparam SEL_IACK	= 6'b011111;

reg [5:0] SelectOutputs;
always @(*) begin
	casex ({AddrQual, AddrSpace, ResetVecFetch, ADDR[31:13]})
	{QUAL_nAS,  SPC_NORM, RV_X, REGION_ROM}:  SelectOutputs = SEL_ROM;

	{QUAL_nAS,  SPC_NORM, RV_X, REGION_DEV8}: SelectOutputs = SEL_DEV8;

	{QUAL_nAS,  SPC_NORM, RV_X, REGION_MMIO}: SelectOutputs = SEL_MMIO;

	{QUAL_nAS,  SPC_NORM, RV_Y, REGION_RAM}:  SelectOutputs = SEL_ROM;
	{QUAL_nAS,  SPC_NORM, RV_N, REGION_RAM}:  SelectOutputs = SEL_RAM;

	{QUAL_nCLK, SPC_CPU,  RV_X, REGION_FPU}:  SelectOutputs = SEL_FPU;
	{QUAL_nAS,  SPC_CPU,  RV_X, REGION_FPU}:  SelectOutputs = SEL_FPU;

	{QUAL_nAS,  SPC_CPU,  RV_X, REGION_IACK}: SelectOutputs = SEL_IACK;

	default:                                  SelectOutputs = SEL_NONE;
	endcase
end
assign {nIACKSEL, nFPUSEL, nRAMSEL, nMMIOSEL, nDEV8SEL, nROMSEL}
    = SelectOutputs;

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

/* Inhibit cache if fetching the reset vector or accessing DEV8 space. */
assign CI = (ResetVecFetch || ~nDEV8SEL);

endmodule
