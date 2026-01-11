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

/*
 * Memory control logic for the cpu010.  This is targeted at an
 * ATF1504AS-10 (10ns) in a PLCC44 package.
 *
 * This handles address decoding and strobing for the 8MB of on-board
 * RAM, the on-board ROM, and selecting any off-board RAM expansion.
 *
 * This is entirely combinatorial logic, and put into a CPLD in order
 * to avoid having to use mutliple GAL22V10s.
 */

module memctl010(
	input wire nAS,		/* /AS from MMU */
	input wire nUDS,	/* /UDS from MMU */
	input wire nLDS,	/* /LDS from MMU */
	input wire RnW,		/* R/W signal from CPU */

	input wire MMUEN,	/* MMUEN signal from Enable register */

	input wire [2:0] FC,	/* FC2..FC0 from CPU */

	input wire [27:21] ADDR,/* A27..A20 from MMU */

	/* Output strobes. */
	output wire nU_RD,
	output wire nU_WR,
	output wire nL_RD,
	output wire nL_WR,

	/* Chip selects */
	output wire nROMSEL,
	output wire [3:0] nRAMSEL,
	output wire nEXPRAMSEL,

	output wire DTACK	/* drives open-drain inverter */
);

/*
 * FC2 FC1 FC0
 *  0   0   0    (Undefined, reserved)
 *  0   0   1    User Data Space
 *  0   1   0    User Program Space
 *  0   1   1    (Undefined, reserved)
 *  1   0   0    (Undefined, reserved) - Control Space
 *  1   0   1    Supervisor Data Space
 *  1   1   0    Supervisor Program Space
 *  1   1   1    CPU Space
 *
 * Normal space that we decode: FC[1] ^ FC[0] -> 1
 */
wire Decode = (FC[1] ^ FC[0]);

localparam SEL_NONE	= 6'b000000;
localparam SEL_ROM	= 6'b100000;
localparam SEL_RAM0	= 6'b010000;
localparam SEL_RAM1	= 6'b001000;
localparam SEL_RAM2	= 6'b000100;
localparam SEL_RAM3	= 6'b000010;
localparam SEL_EXPRAM	= 6'b000001;

localparam SP_dc	= 2'bxx;
localparam SP_RAM	= 2'b00;
localparam SP_ROM	= 2'b01;

reg [5:0] Selects;
always @(*) begin
	casex ({Decode, MMUEN, ADDR[27:26], ADDR[25:21]})
	/* MMU disabled -> everything goes to ROM. */
	{1'b1, 1'b0, SP_dc,  5'bxxxxx}: Selects = SEL_ROM;

	/* ROM space -> ROM, everything else don't care */
	{1'b1, 1'b1, SP_ROM, 5'bxxxxx}: Selects = SEL_ROM;

	/* 4 banks of 2MB RAM. */
	{1'b1, 1'b1, SP_RAM, 5'b00000}: Selects = SEL_RAM0;
	{1'b1, 1'b1, SP_RAM, 5'b00001}: Selects = SEL_RAM1;
	{1'b1, 1'b1, SP_RAM, 5'b00010}: Selects = SEL_RAM2;
	{1'b1, 1'b1, SP_RAM, 5'b00011}: Selects = SEL_RAM3;

	/* Provision for expansion RAM. */
	{1'b1, 1'b1, SP_RAM, 5'b001xx}: Selects = SEL_EXPRAM;
	{1'b1, 1'b1, SP_RAM, 5'b01xxx}: Selects = SEL_EXPRAM;
	{1'b1, 1'b1, SP_RAM, 5'b1xxxx}: Selects = SEL_EXPRAM;

	default:                        Selects = SEL_NONE;
	endcase
end

assign {nROMSEL, nRAMSEL, nEXPRAMSEL} = ~Selects;

/* Read vs write strobe. */
assign nU_RD = nUDS | ~RnW;
assign nU_WR = nUDS |  RnW;

assign nL_RD = nLDS | ~RnW;
assign nL_WR = nLDS |  RnW;

/* We don't ACK expansion RAM; we don't know how fast it is. */
assign DTACK = (Selects[5:1] != 5'd0);

endmodule

// Pin assignment for Yosys workflow.
//
//PIN: CHIP "memctl010" ASSIGNED TO AN PLCC44
//
//PIN: nAS		: 1
//PIN: nUDS		: 2
//PIN: nLDS		: 4
//PIN: RnW		: 5
//PIN: MMUEN		: 6
//PIN: FC_0		: 8
//PIN: FC_1		: 9
//PIN: FC_2		: 11
//PIN: ADDR_21		: 12
//PIN: ADDR_22		: 14
//PIN: ADDR_23		: 16
//PIN: ADDR_24		: 17
//PIN: ADDR_25		: 18
//PIN: ADDR_26		: 19
//PIN: ADDR_27		: 20
//PIN: nU_RD		: 21
//PIN: nU_WR		: 24
//PIN: nL_RD		: 25
//PIN: nL_WR		: 26
//PIN: nROMSEL		: 27
//PIN: nRAMSEL_0	: 28
//PIN: nRAMSEL_1	: 29
//PIN: nRAMSEL_2	: 31
//PIN: nRAMSEL_3	: 33
//PIN: nEXPRAMSEL	: 34
//PIN: DTACK		: 40
