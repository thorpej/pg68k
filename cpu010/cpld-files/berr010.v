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
 * ==> Whatever else we might want to explcitly add?
 *
 * ==> ...and we provide bus cycle timeout logic in the event that no
 *     one is around to complete a cycle.
 *
 * N.B. MC6800 style bus cycles are -- not supported --.
 */

module berr010(
	input wire nRST,	/* system /RESET signal */
	input wire CLK,		/* system CLK10 signal */

	input wire nAS,		/* /AS from CPU */
	input wire RnW,		/* R/W from CPU */

	input wire nBERRSEL,	/* Bus Error register selected */

	inout wire [7:0] DATA,	/* D15..D8 to/from CPU */

	input wire [2:0] MMUERR, /* error from MMU */
	input wire nVMEBERR,	/* /BERR output from VME */

	output wire nBERR_out,	/* directly connected to CPU */
	output wire DTACK_out	/* drives open-drain inverter */
);

localparam MMUERR_NONE	= 3'd0;

reg [2:0] BusError_MMU;
reg BusError_Timeout;
reg BusError_VME;

localparam BERRSTATE_IDLE	= 6'd0;
localparam BERRSTATE_COUNT	= 6'd1;
localparam BERRSTATE_TIMEOUT	= 6'd63;

reg [5:0] BerrState;

/*
 * BUS CYCLE STATE MACHINE
 */
wire [2:0] Cycle = {nAS, RnW, nBERRSEL};

localparam CYCLE_NONE		= 3'b1xx;
localparam CYCLE_NORMAL		= 3'b0x1;
localparam CYCLE_RD_BERR	= 3'b010;
localparam CYCLE_WR_BERR	= 3'b000;

localparam Idle			= 3'd0;
localparam Normal		= 3'd1;
localparam RdBerTermWait	= 3'd2; /* N.B. only two state values ... */
localparam WrBerTermWait	= 3'd3; /* ...with bit #1 set. */
localparam NormalTermWait	= 3'd4;
localparam BerrTermWait		= 3'd5;

reg [2:0] state;
always @(posedge CLK, negedge nRST) begin
	if (~nRST) begin
		BusError_MMU <= MMUERR_NONE;
		BusError_Timeout <= 1'b0;
		BusError_VME <= 1'b0;
		BerrState <= 6'd0;

		state <= Idle;
	end
	else begin
		case (state)
		Idle: begin
			/* Bus cycle clock counter resets here. */
			BerrState <= BERRSTATE_IDLE;

			/* Check for the beginning of a bus cycle. */
			casex (Cycle)
			CYCLE_NONE: begin
				state <= Idle;
			end

			CYCLE_NORMAL: begin
				state <= Normal;
			end

			CYCLE_RD_BERR: begin
				state <= RdBerTermWait;
			end

			CYCLE_WR_BERR: begin
				/* Ignored, but no /BERR here! */
				state <= WrBerTermWait;
			end

			default: begin
				state <= Idle;
			end
			endcase
		end

		Normal: begin
			BerrState <= BERRSTATE_COUNT;
			state <= NormalTermWait;
		end

		RdBerTermWait: begin
			if (nAS) begin
				BusError_MMU <= MMUERR_NONE;
				BusError_Timeout <= 1'b0;
				BusError_VME <= 1'b0;

				state <= Idle;
			end
		end

		WrBerTermWait: begin
			if (nAS) begin
				state <= Idle;
			end
		end

		NormalTermWait: begin
			if (nAS) begin
				state <= Idle;
			end
			else if (BerrState == BERRSTATE_TIMEOUT) begin
				BusError_Timeout <= 1'b1;
				state <= BerrTermWait;
			end
			else if (MMUERR != MMUERR_NONE) begin
				BusError_MMU <= MMUERR;
				state <= BerrTermWait;
			end
			else if (~nVMEBERR) begin
				BusError_VME <= 1'b1;
				state <= BerrTermWait;
			end
			else if (BerrState != BERRSTATE_IDLE) begin
				BerrState <= BerrState + BERRSTATE_COUNT;
			end
		end

		BerrTermWait: begin
			if (nAS) begin
				state <= Idle;
			end
		end
		endcase
	end
end

/* Signal /BERR to the CPU if the state machine says so. */
assign nBERR_out = ~(state == BerrTermWait) | nAS;

/* Signal DTACK if the state machine says so. */
assign DTACK_out = state[1] & ~nAS;

/* Output the Bus Error register if the state machine says so. */
assign DATA = ((state == RdBerTermWait) && ~nAS)
    ? {2'd0, BusError_VME, BusError_Timeout, 1'b0, BusError_MMU}
    : 8'bzzzzzzzz;

endmodule

// Pin assignment for Yosys workflow.
//
//PIN: CHIP "berr010" ASSIGNED TO AN PLCC44
//
//PIN: nRST		: 1
//PIN: nAS		: 2
//PIN: BERR_out		: 4
//PIN: DTACK_out	: 5
//PIN: RnW		: 6
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
//PIN: nVMEBERR		: 24
//PIN: CLK		: 43
//PIN: nBERRSEL		: 44
