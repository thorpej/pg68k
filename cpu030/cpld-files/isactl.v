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
	input wire nISA_IO16,		/* not used at this time */

	output wire BERR,		/* external open-drain inv */
	output wire [1:0] DSACK,	/* external open-drain inv */

	output wire ISA_AEN,
	output wire nISA_IORD,
	output wire nISA_IOWR,

	output wire nDUARTSEL,
	output wire nATASEL,
	output wire nATAAUXSEL,

	output wire TMRINT
);

/* /AS will have been asserted by definition. */
assign ISA_AEN = ~nISASEL;

/*
 * Put devices in ISA-like locations:
 *
 *	DUART     -> 0x3f8 (0), 0x2f8 (1) (DUART checks A8) - 8 bytes
 *	Timer     -> 0x040 - 4 bytes (well, 3)
 *	ATA disk  -> 0x1f0, 0x3f6 - 8 bytes, 2 bytes
 *	RTL8019AS -> 0x300 - 32 bytes
 *
 * N.B. the RTL8019AS fully decodes its own address; we do not need to
 * provide a chip select output for it, but we do need to know when it
 * is selected.
 */
wire nETHSEL;
wire nPIOMODESEL;
wire nTMRCSRSEL;
wire nTMRLSBSEL;
wire nTMRMSBSEL;

localparam DEV_DUART		= 10'b1x11111xxx;
localparam DEV_TMR_CSR		= 10'b0001000000;
localparam DEV_TMR_LSB		= 10'b0001000001;
localparam DEV_TMR_MSB		= 10'b0001000010;
localparam DEV_ATA		= 10'b0111110xxx;
localparam DEV_ATA_AUX		= 10'b1111110110;
localparam DEV_ATA_PMODE	= 10'b1111110111;
localparam DEV_ETH		= 10'b11000xxxxx;

localparam SEL_dc	 = 8'bxxxxxxxx;
localparam SEL_NONE	 = 8'b11111111;
localparam SEL_DUART	 = 8'b01111111;
localparam SEL_ATA	 = 8'b10111111;
localparam SEL_ATA_AUX	 = 8'b11011111;
localparam SEL_ATA_PMODE = 8'b11101111;
localparam SEL_ETH	 = 8'b11110111;
localparam SEL_TMR_CSR	 = 8'b11111011;
localparam SEL_TMR_LSB	 = 8'b11111101;
localparam SEL_TMR_MSB	 = 8'b11111110;

reg [7:0] DevSelectOutputs;
always @(*) begin
	casex ({nISASEL, ADDR})
	{1'b0, DEV_DUART}:     DevSelectOutputs = SEL_DUART;
	{1'b0, DEV_TMR_CSR}:   DevSelectOutputs = SEL_TMR_CSR;
	{1'b0, DEV_TMR_LSB}:   DevSelectOutputs = SEL_TMR_LSB;
	{1'b0, DEV_TMR_MSB}:   DevSelectOutputs = SEL_TMR_MSB;
	{1'b0, DEV_ATA}:       DevSelectOutputs = SEL_ATA;
	{1'b0, DEV_ATA_AUX}:   DevSelectOutputs = SEL_ATA_AUX;
	{1'b0, DEV_ATA_PMODE}: DevSelectOutputs = SEL_ATA_PMODE;
	{1'b0, DEV_ETH}:       DevSelectOutputs = SEL_ETH;
	default:               DevSelectOutputs = SEL_NONE;
	endcase
end
assign {nDUARTSEL, nATASEL, nATAAUXSEL, nPIOMODESEL, nETHSEL,
    nTMRCSRSEL, nTMRLSBSEL, nTMRMSBSEL} = DevSelectOutputs;

reg bus_error;
assign BERR = bus_error;

/* This register holds the current PIO mode for the ATA interface. */
reg [1:0] PIO_mode;
wire pio_mode0 = (PIO_mode == 2'd0);
wire pio_mode1 = (PIO_mode == 2'd1);
wire pio_mode2 = (PIO_mode[1] == 1'b1);

/* Interface between system timer and bus cycle state machine. */
reg [15:0] Timer_value;
reg Timer_valmod;
reg Timer_intack;
reg Timer_enab;
reg Timer_int;

assign TMRINT = Timer_int;

reg enable_data_out;
reg [7:0] data_out;
always @(*) begin
	case (DevSelectOutputs)
	SEL_ATA_PMODE:	data_out = {6'b0, PIO_mode};
	SEL_TMR_CSR:	data_out = {6'b0, Timer_int, Timer_enab};
	SEL_TMR_LSB:	data_out = Timer_value[7:0];
	SEL_TMR_MSB:	data_out = Timer_value[15:8];
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

localparam Idle			= 4'd0;
localparam ATA_t1_r_8		= 4'd1;
localparam ATA_t1_r_16		= 4'd2;
localparam ATA_t1_rw_8		= 4'd3;
localparam ATA_t1_rw_16		= 4'd4;
localparam ATA_t1_r_16_m1	= 4'd5;
localparam ATA_t1_r_16_m2	= 4'd6;
localparam ISA_rw_3w		= 4'd7;
localparam ISA_rw_2w		= 4'd8;
localparam ISA_w6		= 4'd9;
localparam ISA_w5		= 4'd10;
localparam ISA_w4		= 4'd11;
localparam ISA_w3		= 4'd12;
localparam ISA_w2		= 4'd13;
localparam ISA_w1		= 4'd14;
localparam TermWait		= 4'd15;

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

		Timer_value <= 16'd0;
		Timer_valmod <= 1'b0;
		Timer_intack <= 1'b0;
		Timer_enab <= 1'b0;
	end
	else begin
		/* NOTE: Each state represents 40ns. */
		case (state)
		Idle: begin
			/*
			 * The timer has processed these signals, so
			 * clear them now.
			 */
			Timer_valmod <= 1'b0;
			Timer_intack <= 1'b0;

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
			{READ8, SEL_ATA}: begin
				state <= ATA_t1_r_8;
			end

			{WRITE8, SEL_ATA}: begin
				state <= ATA_t1_rw_8;
			end

			{READ8, SEL_ATA_AUX}: begin
				state <= ATA_t1_r_8;
			end

			{WRITE8, SEL_ATA_AUX}: begin
				state <= ATA_t1_rw_8;
			end

			{READ16, SEL_ATA}: begin
				if (pio_mode2) begin
					state <= ATA_t1_r_16_m2;
				end
				else if (pio_mode1) begin
					state <= ATA_t1_r_16_m1;
				end
				else begin
					state <= ATA_t1_r_16;
				end
			end

			{WRITE16, SEL_ATA}: begin
				if (pio_mode2) begin
					io_strobe <= io_strobe_type;
					state <= ISA_w1;
				end
				else if (pio_mode1) begin
					io_strobe <= io_strobe_type;
					state <= ISA_w2;
				end
				else begin
					state <= ATA_t1_rw_16;
				end
			end

			{READ8, SEL_ATA_PMODE}: begin
				enable_data_out <= 1'b1;
				dsack <= SIZ;
				state <= TermWait;
			end

			{WRITE8, SEL_ATA_PMODE}: begin
				PIO_mode <= DATA[1:0];
				dsack <= SIZ;
				state <= TermWait;
			end

			{READ8, SEL_TMR_CSR}: begin
				Timer_intack <= Timer_int;
				enable_data_out <= 1'b1;
				dsack <= SIZ;
				state <= TermWait;
			end

			{WRITE8, SEL_TMR_CSR}: begin
				Timer_enab <= DATA[0];
				dsack <= SIZ;
				state <= TermWait;
			end

			{READ8, SEL_TMR_LSB}: begin
				enable_data_out <= 1'b1;
				dsack <= SIZ;
				state <= TermWait;
			end

			{WRITE8, SEL_TMR_LSB}: begin
				Timer_valmod <= 1'b1;
				Timer_enab <= 1'b0;
				Timer_value[7:0] <= DATA;
				dsack <= SIZ;
				state <= TermWait;
			end

			{READ8, SEL_TMR_MSB}: begin
				enable_data_out <= 1'b1;
				dsack <= SIZ;
				state <= TermWait;
			end

			{WRITE8, SEL_TMR_MSB}: begin
				Timer_valmod <= 1'b1;
				Timer_enab <= 1'b0;
				Timer_value[15:8] <= DATA;
				dsack <= SIZ;
				state <= TermWait;
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

		ATA_t1_r_8: begin
			state <= ATA_t1_rw_8;
		end

		ATA_t1_r_16: begin
			state <= ATA_t1_rw_16;
		end

		ATA_t1_rw_8: begin
			io_strobe <= io_strobe_type;
			state <= ISA_w6;
		end

		ATA_t1_rw_16: begin
			io_strobe <= io_strobe_type;
			state <= ISA_w3;
		end

		ATA_t1_r_16_m1: begin
			io_strobe <= io_strobe_type;
			state <= ISA_w2;
		end

		ATA_t1_r_16_m2: begin
			io_strobe <= io_strobe_type;
			state <= ISA_w1;
		end

		ISA_rw_3w: begin
			io_strobe <= io_strobe_type;
			state <= ISA_w3;
		end

		ISA_rw_2w: begin
			io_strobe <= io_strobe_type;
			state <= ISA_w2;
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

/* System timer registers */
reg [19:0] Timer_current;
reg Timer_was_enab;

always @(posedge CPU_CLK, negedge nRST) begin
	if (~nRST) begin
		Timer_current <= 20'd0;
		Timer_int <= 1'b0;
		Timer_was_enab <= 1'b0;
	end
	else begin
		Timer_int <= Timer_int && Timer_enab;
		if (Timer_valmod) begin
			/*
			 * If Timer_valmod is true, we know that
			 * Timer_enab will be false;
			 */
			Timer_current <= {Timer_value, 4'b0000};
		end
		else if (Timer_enab) begin
			if (~Timer_was_enab || Timer_current == 20'd0) begin
				Timer_current <= {Timer_value, 4'b0000};
				Timer_int <= Timer_was_enab;
			end
			else begin
				if (Timer_intack)
					Timer_int <= 1'b0;
				Timer_current <= Timer_current - 1;
			end
		end
		Timer_was_enab <= Timer_enab;
	end
end

endmodule

// Pin assignment for Yosys workflow.
//
//PIN: CHIP "isactl" ASSIGNED TO AN TQFP100
//
//	=== Inputs ===
//
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
//
//	== Bi-directional / High-Z ==
//
//PIN: DATA_0		: 21
//PIN: DATA_1		: 22
//PIN: DATA_2		: 23
//PIN: DATA_3		: 24
//PIN: DATA_4		: 25
//PIN: DATA_5		: 27
//PIN: DATA_6		: 28
//PIN: DATA_7		: 29
//
//	=== Inputs ===
//
//PIN: ISA_IO16		: 83
//PIN: ISA_RDY		: 84
//PIN: nISASEL		: 85
//PIN: nAS		: 87
//PIN: nDS		: 88
//PIN: nRST		: 89
//PIN: CPU_CLK		: 90
//     
//	=== Outputs ===  
//
//PIN: DSACK_0		: 1
//PIN: DSACK_1		: 2
//PIN: nDUARTSEL	: 72
//PIN: nATASEL		: 75
//PIN: nATAAUXSEL	: 76
//PIN: nISA_IORD	: 77
//PIN: nISA_IOWR	: 78
//PIN: ISA_AEN		: 79
//PIN: TMRINT		: 98
//PIN: BERR		: 100
