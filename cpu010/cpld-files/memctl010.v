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

	input wire [6:0] ADDR,/* A27..A21 from MMU */

	/* Output strobes. */
	output wire nU_RD,
	output wire nU_WR,
	output wire nL_RD,
	output wire nL_WR,

	/* Chip selects */
	output wire nROMSEL,
	output wire [3:0] nRAMSEL,
	output wire nEXPRAMSEL,

`ifdef BUILD_FOR_TEST
	output wire decode_out,
`endif

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
wire decode = (FC[1] ^ FC[0]) & ~nAS;
`ifdef BUILD_FOR_TEST
assign decode_out = decode;
`endif

/*
 * Our ADDR[6:0] input is A27..A21 from the MMU.
 *
 * The spaces we decode are:
 *
 * 65.43210
 * A      A
 * 2      2
 * 7      1
 * 00.00000	on-board RAM bank 0 (2MB) -> DTACK
 * 00.00001	on-board RAM bank 1 (2MB) -> DTACK
 * 00.00010	on-board RAM bank 2 (2MB) -> DTACK
 * 00.00011	on-board RAM bank 3 (2MB) -> DTACK
 * 00.001xx	expansion RAM
 * 00.01xxx	expansion RAM
 * 00.1xxxx	expansion RAM
 * 01.xxxxx	on-board ROM              -> DTACK
 *
 * The on-board ROM is also selected if the MMU is disabled.
 */

wire ram_sel = decode & (MMUEN & ~ADDR[6] & ~ADDR[5]);
wire rom_sel = decode & (~MMUEN | (MMUEN & ~ADDR[6] & ADDR[5]));

wire onboard_ram = ~ADDR[4] & ~ADDR[3] & ~ADDR[2];

assign nRAMSEL[0] = ~(ram_sel &  onboard_ram & ~ADDR[1] & ~ADDR[0]);
assign nRAMSEL[1] = ~(ram_sel &  onboard_ram & ~ADDR[1] &  ADDR[0]);
assign nRAMSEL[2] = ~(ram_sel &  onboard_ram &  ADDR[1] & ~ADDR[0]);
assign nRAMSEL[3] = ~(ram_sel &  onboard_ram &  ADDR[1] &  ADDR[0]);
assign nEXPRAMSEL = ~(ram_sel & ~onboard_ram);
assign nROMSEL    = ~(rom_sel);

/* We don't ACK expansion RAM; we don't know how fast it is. */
assign DTACK = ((ram_sel & onboard_ram) | rom_sel) & (~nUDS | ~nLDS);

/* Read vs write strobe. */
assign nU_RD = nUDS | ~RnW;
assign nU_WR = nUDS |  RnW;

assign nL_RD = nLDS | ~RnW;
assign nL_WR = nLDS |  RnW;

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
//PIN: ADDR_0		: 12
//PIN: ADDR_1		: 14
//PIN: ADDR_2		: 16
//PIN: ADDR_3		: 17
//PIN: ADDR_4		: 18
//PIN: ADDR_5		: 19
//PIN: ADDR_6		: 20
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
