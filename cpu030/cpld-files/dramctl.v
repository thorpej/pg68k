/*
 * DRAM controller for the Playground 68030.  This is almost entirely
 * borrowed from crmaykish's mackerel-30, with adjustments to support
 * configurable SIMM sizes and multiple SIMMs.  Jumpers select the SIMM
 * size (16MB, 32MB, 64MB, or 128MB -- all SIMMs must be the same size)
 *
 * The Playground 68030's DRAM subsystem is arranged with a single DRAM
 * controller that manages two "banks" of 128MB (the largest 5V 72-pin
 * SIMM).  The resulting physical memory map is potentially non-contigous
 * (if < 128MB SIMMs are used).
 *
 * 72-pin SIMMs are 32-bits wide, and thus they are addressed in terms of
 * 32-bit (or 36-bit with parity) words.  Thus, DA0 is connected to A2.
 *
 *   4MB	DA0-DA9		1 rank (RAS0/RAS2)			10 bits
 *   8MB	DA0-DA9		2 rank (RAS0/RAS2, A22=1 RAS1/RAS3)	10 bits
 *  16MB	DA0-DA10	1 rank (RAS0/RAS2)			11 bits
 *  32MB	DA0-DA10	2 rank (RAS0/RAS2, A24=1 RAS1/RAS3)	11 bits
 *  64MB	DA0-DA11	1 rank (RAS0/RAS2)			12 bits
 * 128MB	DA0-DA11	2 rank (RAS0/RAS2, A26=1 RAS1/RAS3)	12 bits
 *
 * We're not going to bother to support anything other than 16MB, 32MB,
 * 64MB, and 128MB SIMMs.
 *
 * N.B. at 50MHz, this particular DRAM access state machine requires 60ns
 * SIMMs.
 */
module dramctl(
	input wire nRST,
	input wire CLK,

	/*
	 * We double-buffer /AS and /CS because we're dealing with
	 * two clock domains.  RnW does not need this treatment because
	 * it's not acted upon until our latched /AS signal is stable.
	 */
	input wire cpu_nAS,
	input wire cpu_nRAMSEL,
	input wire RnW,

	input wire SIZ0, SIZ1,

	input wire [27:0] ADDR,	/* good for 256MB (2x128MB) */

	input wire SIMMSZ,	/* size jumper: 1=11 bit, 0=12 bit */
	input wire [3:0] SIMMPD,/* Presence Detect from 1st SIMM */

	output reg DRAM_nWR,
	output reg [11:0] DRAM_ADDR,
	output reg [3:0] DRAM_nRAS,
	output reg [3:0] DRAM_nCAS,

	/* These drive open-drain inverters. */
	output reg DSACK0, DSACK1
);

reg AS1;
reg AS;
reg RAMSEL1;
reg RAMSEL;

always @(posedge CLK, negedge nRST) begin
	if (~nRST) begin
		AS1 <= 1'b0;
		AS  <= 1'b0;
		RAMSEL1 <= 1'b0;
		RAMSEL  <= 1'b0;
	end
	else begin
		AS1 <= ~cpu_nAS;
		RAMSEL1 <= ~cpu_nRAMSEL;
		AS  <= AS1;
		RAMSEL  <= RAMSEL1;
	end
end

/*
 * The CPU runs at 1/2 the DRAM clock frequency, which is 50MHz.
 * Standard DRAM rows wants to be refreshed within 32 ms.  Since we
 * have 12 address bits, there is a maximum of 4096 rows.  This
 * means we need to refresh a row once every 7.8125 us.  At 50MHz,
 * the clock period is 20 ns, which means max 390 clock cycles between
 * refreshes.  Since memory cycles take a few clock cycles each, we'll
 * give ourselves a 16 clock cycle margin.
 */
localparam REFRESH_CYCLE_CNT = 374;

/*
 * DRAM refresh generator.  We count clocks and when we reach the
 * refresh cycle count, we ask the DRAM state machine to perform
 * a refresh cycle.
 */
reg refresh_req;
reg refresh_ack;
reg [11:0] refresh_cnt;

always @(posedge CLK, negedge nRST) begin
	if (~nRST) begin
		refresh_req <= 1'b0;
		refresh_cnt <= 12'b0;
	end
	else begin
		if (refresh_cnt == REFRESH_CYCLE_CNT) begin
			refresh_req <= 1'b1;
			refresh_cnt <= 12'b0;
		end
		else begin
			refresh_cnt = refresh_cnt + 12'b1;
			if (refresh_ack) refresh_req <= 1'b0;
		end
	end
end

/*
 * PD bits are arranged according to the table in JEDEC 21-C, pg 4.4.2-3,
 * just to make it easier to read.  SZ is the SIMMSZ input.
 *
 *	SZ	PD1	PD2
 *	x	0	0	4MB	10-bit	1-rank	not supported
 *	x	1	1	8MB	10-bit	2-rank	not supported
 *	1	0	1	16MB	11-bit	1-rank
 *	1	1	0	32MB	11-bit	2-rank
 *	0	0	1	64MB	12-bit	1-rank
 *	0	1	0	128MB	12-bit	2-rank
 */

localparam SZ16  = 3'b101;
localparam SZ32  = 3'b110;
localparam SZ64  = 3'b001;
localparam SZ128 = 3'b010;

/*
 * Row address computation.
 */
wire [11:0] RowAddress;
assign RowAddress = SIMMSZ ? {1'b0, ADDR[12:2]}
			   :        ADDR[13:2];

/*
 * Column address computation.
 */
wire [11:0] ColumnAddress;
assign ColumnAddress = SIMMSZ ? {1'b0, ADDR[23:13]}
			      :        ADDR[25:14];

/*
 * Row select computation.
 */
wire [3:0] nRowSelects;
assign nRowSelects = SIMMSZ ? {~ADDR[24], ADDR[24], ~ADDR[24], ADDR[24]}
			    : {~ADDR[26], ADDR[26], ~ADDR[26], ADDR[26]};

/*
 * Byte enables, from Table 7-4 in the 68030 User's Manual.
 * N.B. for reads, we enable all bytes,  We mix the RnW signal
 * in to the type and catch it in the default case.
 */
function [3:0] ComputeByteEnables(
	input r,
	input sz1,
	input sz0,
	input ad1,
	input ad0
);
reg [3:0] enabs;
begin
	case ({r, sz1, sz0, ad1, ad0})
	/* byte writes */
	5'b00100:	enabs = 4'b1000;
	5'b00101:	enabs = 4'b0100;
	5'b00110:	enabs = 4'b0010;
	5'b00111:	enabs = 4'b0001;

	/* word writes */
	5'b01000:	enabs = 4'b1100;
	5'b01001:	enabs = 4'b0110;
	5'b01010:	enabs = 4'b0011;
	5'b01011:	enabs = 4'b0001;

	/* 3 byte writes */
	5'b01100:	enabs = 4'b1110;
	5'b01101:	enabs = 4'b0111;
	5'b01110:	enabs = 4'b0011;
	5'b01111:	enabs = 4'b0001;

	/* long word writes */
	5'b00000:	enabs = 4'b1111;
	5'b00001:	enabs = 4'b0111;
	5'b00010:	enabs = 4'b0011;
	5'b00011:	enabs = 4'b0001;

	/* What remains is: all the reads */
	default:	enabs = 4'b1111;
	endcase

	ComputeByteEnables = enabs;
end
endfunction

wire [3:0] ByteEnables = ComputeByteEnables(RnW, SIZ1, SIZ0, ADDR[1], ADDR[0]);

/*
 * Main DRAM state machine.
 */
localparam IDLE		= 4'd0;
localparam RW1		= 4'd1;
localparam RW2		= 4'd2;
localparam RW3		= 4'd3;
localparam RW4		= 4'd4;
localparam RW5		= 4'd5;
localparam REFRESH1	= 4'd6;
localparam REFRESH2	= 4'd7;
localparam REFRESH3	= 4'd8;
localparam REFRESH4	= 4'd9;
localparam PRECHARGE	= 4'd10;

reg [3:0] state;

always @(posedge CLK, negedge nRST) begin
	if (~nRST) begin
		state <= IDLE;
		DRAM_nRAS <= 4'b1111;
		DRAM_nCAS <= 4'b1111;
		DRAM_nWR <= 1'b1;
		DSACK0 <= 1'b0;
		DSACK1 <= 1'b0;
		refresh_ack <= 1'b0;
	end
	else begin
		case (state)
		IDLE: begin
			if (refresh_req) begin
				/* Start CAS-before-RAS refresh cycle. */
				state <= REFRESH1;
			end
			else if (RAMSEL && AS) begin
				/* DRAM selected, start normal R/W cycle */
				state <= RW1;
			end
		end

		RW1: begin
			/* Mux in the row address. */
			DRAM_ADDR <= RowAddress;
			state <= RW2;
		end

		RW2: begin
			/* Row address is valid, assert RAS. */
			DRAM_nRAS <= nRowSelects;

			state <= RW3;
		end

		RW3: begin
			/* Mux in the column address. */
			DRAM_ADDR <= ColumnAddress;

			/* Set the WE line. */
			DRAM_nWR <= RnW;

			state <= RW4;
		end

		RW4: begin
			/*
			 * Column address is valid, assert CAS based on
			 * the byte enables.
			 */
			DRAM_nCAS <= ~ByteEnables;

			state <= RW5;
		end

		RW5: begin
			/* Data is valid; terminate the cycle. */
			DSACK0 <= 1'b1;
			DSACK1 <= 1'b1;

			/* Stay in RW5 until the CPU ends the cycle. */
			if (~AS) state <= PRECHARGE;
		end

		REFRESH1: begin
			/* Acknowledge refresh request. */
			refresh_ack <= 1'b1;

			/* De-assert WE. */
			DRAM_nWR <= 1'b1;

			/* Assert CAS. */
			DRAM_nCAS <= 4'b0000;

			state <= REFRESH2;
		end

		REFRESH2: begin
			/* Assert RAS. */
			DRAM_nRAS <= 4'b0000;

			state <= REFRESH3;
		end

		REFRESH3: begin
			/* De-assert CAS. */
			DRAM_nCAS <= 4'b1111;

			state <= REFRESH4;
		end

		REFRESH4: begin
			/* De-assert RAS. */
			DRAM_nRAS <= 4'b1111;

			state <= PRECHARGE;
		end

		PRECHARGE: begin
			/*
			 * DRAM cycle finished.  De-assert RAS and CAS,
			 * de-assert DSACK.
			 */
			DRAM_nRAS <= 4'b1111;
			DRAM_nCAS <= 4'b1111;
			DRAM_ADDR <= 12'b0;
			DSACK0 <= 1'b0;
			DSACK1 <= 1'b0;
			refresh_ack <= 1'b0;

			state <= IDLE;
		end
		endcase
	end
end

endmodule
