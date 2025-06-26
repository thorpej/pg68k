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
 *    Initial
 *       |
 *       |
 *       v
 *        ___     ___     ___
 * 50MHz |   |   |   |   |   |   |     DRAM
 *       |   |___|   |___|   |___|
 *       ^       ^
 *       |  20ns |
 *
 *        _______         _______
 * 25MHz |       |       |             68030
 *       |       |_______|
 *       ^               ^
 *       |      40ns     |
 *       A   B   C   D   E   F   G
 *
 * Signal transition in the 68030 can happen around B, D, and F.
 */
`timescale 10ns/1ns

module tb();

localparam CLK30	= 4;	/* full 030 clock */
localparam HCLK30	= 2;	/* half 030 clock */
localparam QCLK30	= 1;	/* quarter 030 clock */

reg n_rst;
reg clk;
reg n_as;
reg n_ramsel;
reg rnw;

reg [1:0] siz;
reg [27:0] addr;

reg simmsz;
reg [3:0] simmpd;

wire dram_n_wr;
wire [11:0] dram_addr;
wire [3:0] dram_n_rasa;
wire [3:0] dram_n_casa;
wire [3:0] dram_n_rasb;
wire [3:0] dram_n_casb;

wire [1:0] dsack;
wire berr;

wire cbreq_n;
wire cback;

wire cycle_terminated = berr | dsack[0] | dsack[1];

	/* Instantiate the device under test. */
	dramctl dut (
		.nRST(n_rst),
		.CLK(clk),
		.nAS(n_as),
		.nRAMSEL(n_ramsel),
		.RnW(rnw),
		.SIZ(siz),
		.ADDR(addr),
		.SIMMSZ(simmsz),
		.SIMMPDA(simmpd),
		.SIMMPDB(simmpd),
		.CBREQ(cbreq_n),
		.DRAM_nWR(dram_n_wr),
		.DRAM_ADDR(dram_addr),
		.DRAM_nRASA(dram_n_rasa),
		.DRAM_nCASA(dram_n_casa),
		.DRAM_nRASB(dram_n_rasb),
		.DRAM_nCASB(dram_n_casb),
		.DSACK(dsack),
		.BERR(berr),
		.CBACK(cback)
	);

	initial begin
		$dumpfile("dramctl_tb.vcd");
		$dumpvars;
		clk = 1;	/* 1 cycle is rising-edge-to-rising-edge */
		n_rst = 1;
		n_as = 1;
		n_ramsel = 1;
		rnw = 1;

		/* 60ns 16MB SIMM */
		simmsz = 1;
		simmpd = 4'b1110;

		cbreq_n = 1;

		siz = 2'd0;
		addr = 28'h0000000;
	end

	always #1 clk = ~clk;

	initial begin
		n_rst = 0;
		#(CLK30);	/* reset for 1 68030 clock */
		n_rst = 1;

		/*
		 * Do a long word read at 0x1000.
		 */

		/*
		 * ** S0 **
		 * - Address, RnW, and SIZ become valid.
		 */
		addr = 28'h1000;
		rnw = 1;
		siz = 2'b00;

		/*
		 * ** S1 **
		 * /AS is asserted.
		 */
		#(HCLK30)
		n_as = 0;
		n_ramsel = 0;

		/*
		 * ** S2 **
		 * Wait cycle termination.
		 */
		while (~cycle_terminated) begin
			#CLK30;
		end

		/*
		 * ** S4 **
		 * Wait for 1/2 clock.
		 */
		#HCLK30;

		/*
		 * ** S5 **
		 * De-assert /AS.
		 */
		n_as = 1;
		n_ramsel = 1;

		#(CLK30 * 4);	/* wait for DRAM controller to finish */

		/*
		 * Do a byte write at 0x1001.
		 */

		/*
		 * ** S0 **
		 * - Address, RnW, and SIZ become valid.
		 */
		addr = 28'h1001;
		rnw = 0;
		siz = 2'b01;

		/*
		 * ** S1 **
		 * /AS is asserted.
		 */
		#(HCLK30)
		n_as = 0;
		n_ramsel = 0;

		/*
		 * ** S2 **
		 * Wait cycle termination.
		 */
		while (~cycle_terminated) begin
			#CLK30;
		end

		/*
		 * ** S4 **
		 * Wait for 1/2 clock.
		 */
		#HCLK30;

		/*
		 * ** S5 **
		 * De-assert /AS.
		 */
		n_as = 1;
		n_ramsel = 1;

		#(CLK30 * 4);	/* wait for DRAM controller to finish */

		/*
		 * Do a long word read at 0x2000000 (just beyond 2x 16MB).
		 * This should generate a bus error.
		 */

		/*
		 * ** S0 **
		 * - Address, RnW, and SIZ become valid.
		 */
		addr = 28'h2000000;
		rnw = 1;
		siz = 2'b00;

		/*
		 * ** S1 **
		 * /AS is asserted.
		 */
		#(HCLK30)
		n_as = 0;
		n_ramsel = 0;

		/*
		 * ** S2 **
		 * Wait cycle termination.
		 */
		while (~cycle_terminated) begin
			#CLK30;
		end

		/*
		 * ** S4 **
		 * Wait for 1/2 clock.
		 */
		#HCLK30;

		/*
		 * ** S5 **
		 * De-assert /AS.
		 */
		n_as = 1;
		n_ramsel = 1;

		#(CLK30 * 4);	/* wait for DRAM controller to finish */


		/* Wait for 1000 DRAM clocks to see some refreshes. */
		#(1000*2);

		$finish;
	end

endmodule
