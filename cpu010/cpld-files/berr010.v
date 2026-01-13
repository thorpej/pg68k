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
 * Bus Error controller for the cpu010.  This is targeted at an
 * ATF1504AS-10 (10ns) in a PLCC44 package.
 *
 * This handles bus errors from multiple sources:
 *
 * ==> The MMU.
 *
 * ==> The VME expansion bus.
 *
 * ==> Whatever else we might want to explicitly add?
 *
 * ==> ...and we provide bus cycle timeout logic in the event that no
 *     one is around to complete a cycle.
 *
 * N.B. MC6800 style bus cycles are -- not supported --.
 */

module berr010(
	input wire n_reset,	/* system /RESET signal */
	input wire cpu_clk,	/* system CLK10 signal */

	input wire n_as,	/* /AS from CPU */
	input wire rnw,		/* R/W from CPU */

	input wire n_berrsel,	/* Bus Error register selected */

	inout wire [7:0] DATA,	/* D15..D8 to/from CPU */

	input wire [2:0] MMUERR, /* error from MMU */
	input wire n_vmeberr,	/* /BERR output from VME */

	output wire n_berr_out,	/* directly connected to CPU */
	output wire dtack_out	/* drives open-drain inverter */
);

/*****************************************************************************/

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

wire bus_timeout;
wire bus_timer_enable = ~n_as;

localparam BUS_TIMER_INITIAL	=	6'd0;
localparam BUS_TIMER_TICK	=	6'd1;
localparam BUS_TIMER_TIMEOUT	=	6'd63;

reg [5:0] bus_timer;
initial begin
	bus_timer = BUS_TIMER_INITIAL;
end

always @(posedge cpu_clk, negedge n_reset) begin
	if (~n_reset) begin
		bus_timer <= BUS_TIMER_INITIAL;
	end
	else if (~bus_timer_enable) begin
		bus_timer <= BUS_TIMER_INITIAL;
	end
	else if (bus_timer != BUS_TIMER_TIMEOUT) begin
		bus_timer <= bus_timer + BUS_TIMER_TICK;
	end
end

assign bus_timeout = (bus_timer == BUS_TIMER_TIMEOUT);

/*****************************************************************************/

/*
 * The lower 4 bits of the Bus Error register are reserved for
 * errors from the MMU (of which 3 are currently defined).
 */
localparam MMUERR_NONE		= 3'b000;
localparam BERR_NONE		= 6'b000000;
localparam BERR_MMUERR_UPPER	= 3'b000;
localparam BERR_TIMEOUT		= 6'b010000;
localparam BERR_VME		= 6'b100000;

reg [5:0] bus_error_reg;
initial begin
	bus_error_reg = BERR_NONE;
end

/************************* BUS CYCLE STATE MACHINE ***************************/

wire [2:0] Cycle = {n_as, rnw, n_berrsel};

localparam CYCLE_NONE		= 3'b1xx;
localparam CYCLE_NORMAL		= 3'b0x1;
localparam CYCLE_RD_BERR	= 3'b010;
localparam CYCLE_WR_BERR	= 3'b000;

localparam S_IDLE		= 3'd0;
localparam S_NORMAL_TERMWAIT	= 3'd1;
localparam S_RDBERR_TERMWAIT	= 3'd2; /* N.B. only two state values ... */
localparam S_WRBERR_TERMWAIT	= 3'd3; /* ...with bit #1 set. */
localparam S_BUSERROR		= 3'd4;

reg [2:0] state;
always @(posedge cpu_clk, negedge n_reset) begin
	if (~n_reset) begin
		bus_error_reg <= BERR_NONE;

		state <= S_IDLE;
	end
	else begin
		case (state)
		S_IDLE: begin
			/* Check for the beginning of a bus cycle. */
			casex (Cycle)
			CYCLE_NONE: begin
				state <= S_IDLE;
			end

			CYCLE_NORMAL: begin
				state <= S_NORMAL_TERMWAIT;
			end

			CYCLE_RD_BERR: begin
				state <= S_RDBERR_TERMWAIT;
			end

			CYCLE_WR_BERR: begin
				/* Ignored, but no /BERR here! */
				state <= S_WRBERR_TERMWAIT;
			end

			default: begin
				state <= S_IDLE;
			end
			endcase
		end

		S_RDBERR_TERMWAIT: begin
			if (n_as) begin
				bus_error_reg = BERR_NONE;
				state <= S_IDLE;
			end
		end

		S_WRBERR_TERMWAIT: begin
			if (n_as) begin
				state <= S_IDLE;
			end
		end

		S_NORMAL_TERMWAIT: begin
			if (n_as) begin
				state <= S_IDLE;
			end
			else if (MMUERR != MMUERR_NONE) begin
				bus_error_reg <= {BERR_MMUERR_UPPER, MMUERR};
				state <= S_BUSERROR;
			end
			else if (~n_vmeberr) begin
				bus_error_reg <= BERR_VME;
				state <= S_BUSERROR;
			end
			else if (bus_timeout) begin
				bus_error_reg <= BERR_TIMEOUT;
				state <= S_BUSERROR;
			end
		end

		S_BUSERROR: begin
			if (n_as) begin
				state <= S_IDLE;
			end
		end
		endcase
	end
end

/*****************************************************************************/

/* Signal /BERR to the CPU if the state machine says so. */
assign n_berr_out = ~(state == S_BUSERROR) | n_as;

/* Signal DTACK if the state machine says so. */
assign dtack_out = state[1] & ~n_as;

/* Output the Bus Error register if the state machine says so. */
assign DATA = ((state == S_RDBERR_TERMWAIT) && ~n_as)
    ? {2'd0, bus_error_reg} : 8'bzzzzzzzz;

endmodule

// Pin assignment for Yosys workflow.
//
//PIN: CHIP "berr010" ASSIGNED TO AN PLCC44
//
//PIN: n_reset		: 1
//PIN: n_as		: 2
//PIN: n_berr_out	: 4
//PIN: dtack_out	: 5
//PIN: rnw		: 6
//PIN: DATA_0		: 8
//PIN: DATA_1		: 9
//PIN: DATA_2		: 11
//PIN: DATA_3		: 12
//PIN: DATA_4		: 14
//PIN: DATA_5		: 16
//PIN: DATA_6		: 17
//PIN: DATA_7		: 18
//PIN: MMUERR_0		: 19
//PIN: MMUERR_1		: 20
//PIN: MMUERR_2		: 21
//PIN: n_vmeberr	: 24
//PIN: cpu_clk		: 43
//PIN: n_berrsel	: 44
