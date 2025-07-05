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
reg n_ds;
reg rnw;

reg [1:0] siz;
reg [9:0] addr;
reg [7:0] data;

wire [7:0] data_out;
assign data_out = data;

reg n_isasel;

reg isa_rdy;
reg n_isa_io16;

wire berr;
wire [1:0] dsack;

wire isa_aen;
wire n_isa_iord;
wire n_iso_iowr;

wire n_duartsel;
wire n_atasel;
wire n_ataauxsel;
wire n_ataben;

wire tmrint;

wire cycle_terminated = berr | dsack[0] | dsack[1];

	/* Instantiate the device under test. */
	isactl dut (
		.nRST(n_rst),
		.CPU_CLK(clk),
		.nAS(n_as),
		.nDS(n_ds),
		.nISASEL(n_isasel),
		.RnW(rnw),
		.SIZ(siz),
		.ADDR(addr),
		.DATA(data_out),
		.ISA_RDY(isa_rdy),
		.nISA_IO16(n_isa_io16),
		.DSACK(dsack),
		.BERR(berr),
		.ISA_AEN(isa_aen),
		.nISA_IORD(n_isa_iord),
		.nISA_IOWR(n_isa_iowr),
		.nDUARTSEL(n_duartsel),
		.nATASEL(n_atasel),
		.nATAAUXSEL(n_ataauxsel),
		.nATABEN(n_ataben),
		.TMRINT(tmrint)
	);

	initial begin
		$dumpfile("isactl_tb.vcd");
		$dumpvars;
		clk = 1;	/* 1 cycle is rising-edge-to-rising-edge */
		n_rst = 1;
		n_as = 1;
		n_ds = 1;
		n_isasel = 1;
		rnw = 1;

		isa_rdy = 1;	/* no device-inserted wait states for now */

		siz = 2'd0;
		addr = 10'h000;
	end

	always #2 clk = ~clk;	/* clk == cpu_clk */

	initial begin
		n_rst = 0;
		#(CLK30);	/* reset for 1 68030 clock */
		n_rst = 1;

		/*
		 * Do a DUART read.
		 */

		/*
		 * ** S0 **
		 * - Address, RnW, and SIZ become valid.
		 */
		addr = 10'h3f8;
		rnw = 1;
		siz = 2'b01;
		#(HCLK30)

		/*
		 * ** S1 **
		 * /AS and /DS asserted.
		 */
		n_as = 0;
		n_ds = 0;
		n_isasel = 0;
		#(HCLK30);

		/*
		 * ** S2 **
		 * Unrelated control signals happen.
		 */
		#(HCLK30);

		/*
		 * ** S3 **
		 * Wait for cycle termination.
		 */
		while (~cycle_terminated) begin
			#CLK30;
		end
		#HCLK30;

		/*
		 * ** S4 **
		 * Wait for 1/2 clock.
		 */
		#HCLK30;

		/*
		 * ** S5 **
		 * De-assert /AS and /DS.
		 */
		n_as = 1;
		n_ds = 1;
		n_isasel = 1;
		#HCLK30;

		/*
		 * Do a DUART write.  To the other UART.
		 */

		/*
		 * ** S0 **
		 * - Address, RnW, and SIZ become valid.
		 */
		addr = 10'h2f8;
		rnw = 0;
		siz = 2'b01;
		#(HCLK30)

		/*
		 * ** S1 **
		 * /AS asserted.
		 */
		n_as = 0;
		n_isasel = 0;
		#(HCLK30)

		/*
		 * ** S2 **
		 * Data placed onto data bus.
		 */
		#(HCLK30)

		/*
		 * ** S3 **
		 * /DS asserted.  Wait for cycle termination.
		 */
		n_ds = 0;
		while (~cycle_terminated) begin
			#CLK30;
		end
		#HCLK30;

		/*
		 * ** S4 **
		 * Wait for 1/2 clock.
		 */
		#HCLK30;

		/*
		 * ** S5 **
		 * De-assert /AS and /DS.
		 */
		n_as = 1;
		n_ds = 1;
		n_isasel = 1;
		#HCLK30;

		/*
		 * Do a 16-bit Ethernet data write.
		 */

		/*
		 * ** S0 **
		 * - Address, RnW, and SIZ become valid.
		 */
		addr = 10'h310;
		rnw = 0;
		siz = 2'b10;
		#(HCLK30)

		/*
		 * ** S1 **
		 * /AS asserted.
		 */
		n_as = 0;
		n_isasel = 0;
		#(HCLK30)

		/*
		 * ** S2 **
		 * Data placed onto data bus.
		 */
		#(HCLK30)

		/*
		 * ** S3 **
		 * /DS asserted.  Wait for cycle termination.
		 */
		n_ds = 0;
		while (~cycle_terminated) begin
			#CLK30;
		end
		#HCLK30;

		/*
		 * ** S4 **
		 * Wait for 1/2 clock.
		 */
		#HCLK30;

		/*
		 * ** S5 **
		 * De-assert /AS and /DS.
		 */
		n_as = 1;
		n_ds = 1;
		n_isasel = 1;
		#HCLK30;

		/*
		 * Do a TIMER read.
		 */

		/*
		 * ** S0 **
		 * - Address, RnW, and SIZ become valid.
		 */
		addr = 10'h040;
		rnw = 1;
		siz = 2'b01;
		#(HCLK30)

		/*
		 * ** S1 **
		 * /AS and /DS asserted.
		 */
		n_as = 0;
		n_ds = 0;
		n_isasel = 0;
		#(HCLK30);

		/*
		 * ** S2 **
		 * Unrelated control signals happen.
		 */
		#(HCLK30);

		/*
		 * ** S3 **
		 * Wait for cycle termination.
		 */
		while (~cycle_terminated) begin
			#CLK30;
		end
		#HCLK30;

		/*
		 * ** S4 **
		 * Wait for 1/2 clock.
		 */
		#HCLK30;

		/*
		 * ** S5 **
		 * De-assert /AS and /DS.
		 */
		n_as = 1;
		n_ds = 1;
		n_isasel = 1;
		#HCLK30;

		/*
		 * Do a TIMER write.
		 */

		/*
		 * ** S0 **
		 * - Address, RnW, and SIZ become valid.
		 */
		addr = 10'h040;
		rnw = 0;
		siz = 2'b01;
		#(HCLK30)

		/*
		 * ** S1 **
		 * /AS asserted.
		 */
		n_as = 0;
		n_isasel = 0;
		#(HCLK30)

		/*
		 * ** S2 **
		 * Data placed onto data bus.
		 */
		#(HCLK30)

		/*
		 * ** S3 **
		 * /DS asserted.  Wait for cycle termination.
		 */
		n_ds = 0;
		while (~cycle_terminated) begin
			#CLK30;
		end
		#HCLK30;

		/*
		 * ** S4 **
		 * Wait for 1/2 clock.
		 */
		#HCLK30;

		/*
		 * ** S5 **
		 * De-assert /AS and /DS.
		 */
		n_as = 1;
		n_ds = 1;
		n_isasel = 1;
		#HCLK30;

		/*
		 * Do an 8-bit ATA AUX write.
		 */

		/*
		 * ** S0 **
		 * - Address, RnW, and SIZ become valid.
		 */
		addr = 10'h3f6;
		rnw = 0;
		siz = 2'b01;
		#(HCLK30)

		/*
		 * ** S1 **
		 * /AS asserted.
		 */
		n_as = 0;
		n_isasel = 0;
		#(HCLK30)

		/*
		 * ** S2 **
		 * Data placed onto data bus.
		 */
		#(HCLK30)

		/*
		 * ** S3 **
		 * /DS asserted.  Wait for cycle termination.
		 */
		n_ds = 0;
		while (~cycle_terminated) begin
			#CLK30;
		end
		#HCLK30;

		/*
		 * ** S4 **
		 * Wait for 1/2 clock.
		 */
		#HCLK30;

		/*
		 * ** S5 **
		 * De-assert /AS and /DS.
		 */
		n_as = 1;
		n_ds = 1;
		n_isasel = 1;
		#HCLK30;

		/*
		 * 16-bit ATA data read.
		 */

		/*
		 * ** S0 **
		 * - Address, RnW, and SIZ become valid.
		 */
		addr = 10'h1f0;
		rnw = 1;
		siz = 2'b10;
		#(HCLK30)

		/*
		 * ** S1 **
		 * /AS and /DS asserted.
		 */
		n_as = 0;
		n_ds = 0;
		n_isasel = 0;
		#(HCLK30);

		/*
		 * ** S2 **
		 * Unrelated control signals happen.
		 */
		#(HCLK30);

		/*
		 * ** S3 **
		 * Wait for cycle termination.
		 */
		while (~cycle_terminated) begin
			#CLK30;
		end
		#HCLK30;

		/*
		 * ** S4 **
		 * Wait for 1/2 clock.
		 */
		#HCLK30;

		/*
		 * ** S5 **
		 * De-assert /AS and /DS.
		 */
		n_as = 1;
		n_ds = 1;
		n_isasel = 1;
		#HCLK30;

		/*
		 * Do a 16-bit unaligned write to trigger a bus error.
		 */

		/*
		 * ** S0 **
		 * - Address, RnW, and SIZ become valid.
		 */
		addr = 10'h311;
		rnw = 0;
		siz = 2'b10;
		#(HCLK30)

		/*
		 * ** S1 **
		 * /AS asserted.
		 */
		n_as = 0;
		n_isasel = 0;
		#(HCLK30)

		/*
		 * ** S2 **
		 * Data placed onto data bus.
		 */
		#(HCLK30)

		/*
		 * ** S3 **
		 * /DS asserted.  Wait for cycle termination.
		 */
		n_ds = 0;
		while (~cycle_terminated) begin
			#CLK30;
		end
		#HCLK30;

		/*
		 * ** S4 **
		 * Wait for 1/2 clock.
		 */
		#HCLK30;

		/*
		 * ** S5 **
		 * De-assert /AS and /DS.
		 */
		n_as = 1;
		n_ds = 1;
		n_isasel = 1;
		#HCLK30;

		/*
		 * Write 2 to the PIO_mode register.
		 */

		/*
		 * ** S0 **
		 * - Address, RnW, and SIZ become valid.
		 */
		addr = 10'h3f7;
		rnw = 0;
		siz = 2'b01;
		#(HCLK30)

		/*
		 * ** S1 **
		 * /AS asserted.
		 */
		n_as = 0;
		n_isasel = 0;
		#(HCLK30)

		/*
		 * ** S2 **
		 * Data placed onto data bus.
		 */
		data = 2;
		#(HCLK30)

		/*
		 * ** S3 **
		 * /DS asserted.  Wait for cycle termination.
		 */
		n_ds = 0;
		while (~cycle_terminated) begin
			#CLK30;
		end
		#HCLK30;

		/*
		 * ** S4 **
		 * Wait for 1/2 clock.
		 */
		#HCLK30;

		/*
		 * ** S5 **
		 * De-assert /AS and /DS.
		 */
		n_as = 1;
		n_ds = 1;
		n_isasel = 1;
		#HCLK30;




		/* 4 clocks to observe idle. */
		#(CLK30 * 4);

		$finish;
	end

endmodule
