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
 * Playground 68030 ISA bus controller.
 *
 * This contains the glue logic to convert the 68k bus signalling to
 * ISA bus signalling, generate the timing, and select the on-board
 * ISA (and ISA-like) peripherals.
 *
 * TODO:
 * - Add timings for ATA PIO modes other than mode 0.
 * - Optimize the cycle timings to squeeze out clock cycles where possible.
 * - Add a generic ISA bus interface that defaults to the slow timings?
 *   Right now, an unknown device will generate a bus error.
 */

module isactl(
	input wire nRST,
	input wire CPU_CLK,

	input wire [9:0] ADDR,
	inout wire [7:0] DATA,

	input wire nISASEL,
	input wire nAS,
	input wire nDS,
	input wire RnW,
	input wire [1:0] SIZ,

	input wire ISA_RDY,
	input wire nISA_IO16,
	input wire nISA_MEM16,

	output wire BERR,		/* external open-drain inv */
	output wire [1:0] DSACK,	/* external open-drain inv */

	output wire ISA_AEN,
	output wire nISA_IORD,
	output wire nISA_IOWR,

	output wire nDUARTSEL,
	output wire nTMRSEL,
	output wire nATASEL,
	output wire nATAAUXSEL
);

/* /AS will have been asserted by definition. */
assign ISA_AEN = ~nISASEL;

/*
 * Put devices in ISA-like locations:
 *
 *	DUART     -> 0x3f8 (0), 0x2f8 (1) (DUART checks A8) - 8 bytes
 *	Timer     -> 0x040 - 8 bytes
 *	ATA disk  -> 0x1f0, 0x3f6 - 8 bytes, 2 bytes
 *	RTL8019AS -> 0x300 - 32 bytes
 *
 * N.B. the RTL8019AS fully decodes its own address; we do not need to
 * provide a chip select output for it, but we do need to know when it
 * is selected.
 */
wire nETHSEL;
wire nPIOMODESEL;

localparam DEV_DUART		= 10'b1x11111xxx;
localparam DEV_TIMER		= 10'b0001000xxx;
localparam DEV_ATA		= 10'b0111110xxx;
localparam DEV_ATA_AUX		= 10'b1111110110;
localparam DEV_ATA_PMODE	= 10'b1111110111;
localparam DEV_ETH		= 10'b11000xxxxx;

localparam SEL_dc	 = 6'bxxxxxx;
localparam SEL_NONE	 = 6'b111111;
localparam SEL_DUART	 = 6'b011111;
localparam SEL_TIMER	 = 6'b101111;
localparam SEL_ATA	 = 6'b110111;
localparam SEL_ATA_AUX	 = 6'b111011;
localparam SEL_ATA_PMODE = 6'b111101;
localparam SEL_ETH	 = 6'b111110;

reg [5:0] DevSelectOutputs;
always @(*) begin
	casex ({nISASEL, ADDR})
	{1'b0, DEV_DUART}:     DevSelectOutputs = SEL_DUART;
	{1'b0, DEV_TIMER}:     DevSelectOutputs = SEL_TIMER;
	{1'b0, DEV_ATA}:       DevSelectOutputs = SEL_ATA;
	{1'b0, DEV_ATA_AUX}:   DevSelectOutputs = SEL_ATA_AUX;
	{1'b0, DEV_ATA_PMODE}: DevSelectOutputs = SEL_ATA_PMODE;
	{1'b0, DEV_ETH}:       DevSelectOutputs = SEL_ETH;
	default:               DevSelectOutputs = SEL_NONE;
	endcase
end
assign {nDUARTSEL, nTMRSEL, nATASEL, nATAAUXSEL, nPIOMODESEL, nETHSEL} =
    DevSelectOutputs;

reg bus_error;
assign BERR = bus_error;

/* This register holds the current PIO mode for the ATA interface. */
reg [1:0] PIO_mode;

reg enable_data_out;
reg [7:0] data_out;
always @(*) begin
	case (DevSelectOutputs)
	SEL_ATA_PMODE:	data_out = {6'b0, PIO_mode};
	default:	data_out = 8'hFF;
	endcase
end

assign DATA = (enable_data_out && ~nDS) ? data_out : 8'bzzzzzzzz;

localparam IO_STROBE_NONE  = 2'b00;
localparam IO_STROBE_READ  = 2'b10;
localparam IO_STROBE_WRITE = 2'b01;
wire [1:0] io_strobe_type = {RnW, ~RnW};

/*
 * Qual with /DS from the CPU so they de-assert immediately.
 */
reg [1:0] io_strobe;
reg [1:0] dsack;
assign {nISA_IORD, nISA_IOWR} = ~(io_strobe & {~nDS, ~nDS});
assign DSACK = dsack & {~nDS, ~nDS};

localparam Idle		= 4'd0;
localparam ATA_t1_0_8	= 4'd1;
localparam ATA_t1_0_16	= 4'd2;
localparam ATA_t1_1_8	= 4'd3;
localparam ATA_t1_1_16	= 4'd4;
localparam ISA_rw_3w	= 4'd5;
localparam ISA_rw_2w	= 4'd6;
localparam ISA_w8	= 4'd7;
localparam ISA_w7	= 4'd8;
localparam ISA_w6	= 4'd9;
localparam ISA_w5	= 4'd10;
localparam ISA_w4	= 4'd11;
localparam ISA_w3	= 4'd12;
localparam ISA_w2	= 4'd13;
localparam ISA_w1	= 4'd14;
localparam TermWait	= 4'd15;

wire [5:0] Cycle = {nAS, nDS, RnW, SIZ, ADDR[0]};
localparam NONE		= 6'b1xxxxx;
localparam STARTWRITE	= 6'b010xxx;
localparam READ8	= 6'b00101x;
localparam WRITE8	= 6'b00001x;
localparam READ16	= 6'b001100;
localparam WRITE16	= 6'b000100;
localparam ANY8		= 6'b00x01x;
localparam ANY16	= 6'b00x100;

reg [3:0] state;
always @(posedge CPU_CLK, negedge nRST) begin
	if (~nRST) begin
		enable_data_out <= 1'b0;
		io_strobe <= IO_STROBE_NONE;
		bus_error <= 1'b0;
		dsack <= 2'b00;
		PIO_mode <= 2'd0;
		state <= Idle;
	end
	else begin
		/* NOTE: Each state represents 40ns. */
		case (state)
		Idle: begin
			/*
			 * If we get any decode hit here, we know that
			 * the chip select for the device has already
			 * been asserted by combinatorial logic.  For
			 * read cycles, /AS and /DS are asserted at the
			 * same time, whereas for write cycles, /DS is
			 * assrted one CPU clock cycle later than /AS.
			 *
			 * This means that the number of slots needed
			 * for address setup times might be different
			 * for reads vs. writes.
			 */
			casex ({Cycle, DevSelectOutputs})
			{ANY8, SEL_ATA}: begin
				state <= ATA_t1_0_8;
			end

			{ANY8, SEL_ATA_AUX}: begin
				state <= ATA_t1_0_8;
			end

			{ANY16, SEL_ATA}: begin
				state <= ATA_t1_0_16;
			end

			{READ8, SEL_ATA_PMODE}: begin
				enable_data_out = 1'b1;
				dsack <= SIZ;
				state <= TermWait;
			end

			{WRITE8, SEL_ATA_PMODE}: begin
				PIO_mode <= DATA[1:0];
				dsack <= SIZ;
				state <= TermWait;
			end

			{ANY8, SEL_TIMER}: begin
				state <= ISA_rw_3w;
			end

			{ANY8, SEL_DUART}: begin
				io_strobe <= io_strobe_type;
				state <= ISA_w1;  /* maybe 1 clock faster? */
			end

			{ANY8, SEL_ETH}: begin
				state <= ISA_rw_2w;
			end

			{ANY16, SEL_ETH}: begin
				state <= ISA_rw_2w;
			end

			{STARTWRITE, SEL_dc}: begin
				state <= Idle;
			end

			{NONE, SEL_dc}: begin
				state <= Idle;
			end

			default: begin
				bus_error <= 1'b1;
				state <= TermWait;
			end
			endcase
		end

		/*
		 * ATA state machine matches timings for PIO mode 0
		 */
		ATA_t1_0_8: begin
			state <= ATA_t1_1_8;
		end

		ATA_t1_0_16: begin
			state <= ATA_t1_1_16;
		end

		ATA_t1_1_8: begin
			io_strobe <= io_strobe_type;
			state <= ISA_w8;
		end

		ATA_t1_1_16: begin
			io_strobe <= io_strobe_type;
			state <= ISA_w5;
		end

		ISA_rw_3w: begin
			io_strobe <= io_strobe_type;
			state <= ISA_w3;
		end

		ISA_rw_2w: begin
			io_strobe <= io_strobe_type;
			state <= ISA_w2;
		end

		ISA_w8: begin
			state <= ISA_w7;
		end

		ISA_w7: begin
			state <= ISA_w6;
		end

		ISA_w6: begin
			state <= ISA_w5;
		end

		ISA_w5: begin
			state <= ISA_w4;
		end

		ISA_w4: begin
			state <= ISA_w3;
		end

		ISA_w3: begin
			state <= ISA_w2;
		end

		ISA_w2: begin
			state <= ISA_w1;
		end

		ISA_w1: begin
			if (ISA_RDY) begin
				dsack <= SIZ;
				state <= TermWait;
			end
		end

		TermWait: begin
			if (nAS) begin
				enable_data_out <= 1'b0;
				io_strobe <= IO_STROBE_NONE;
				bus_error <= 1'b0;
				dsack <= 2'b00;
				state <= Idle;
			end
		end
		endcase
	end
end

endmodule
