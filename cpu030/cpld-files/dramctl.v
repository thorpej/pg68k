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
 * DRAM controller for the Playground 68030.  This is based on crmaykish's
 * mackerel-30 DRAM controller, but has been modified to support multiple
 * SIMM sizes (16MB, 32MB, 64MB, or 128MB -- all SIMMs must be the same size)
 * and 2 SIMMs, as well as to generate bus errors when the request does not
 * fit within the SIMM configuration.
 *
 * This consumes almost all of the available I/O on a TQFP100 ATF1508AS.
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
 * N.B. at 50MHz, this particular DRAM access state machine requires 60ns
 * SIMMs.  We can detect the SIMM speed via the Presence Detect pins, and
 * inserting additional states to support 70ns DRAM shouldn't be too bad,
 * so maybe I'll do it someday.  But for now, we require 60ns SIMMs, and
 * validate this using Presence Detect.
 */
module dramctl(
	input wire nRST,
	input wire CLK,

	input wire nAS,
	input wire nRAMSEL,
	input wire RnW,

	input wire [1:0] SIZ,

	input wire [27:0] ADDR,	/* good for 256MB (2x128MB) */

	input wire SIMMSZ,	 /* size jumper: 1=11 bit, 0=12 bit */
	input wire [3:0] SIMMPDA,/* Presence Detect from 1st SIMM */
	input wire [3:0] SIMMPDB,/* Presence Detect from 2nd SIMM */

	input wire nCBREQ,

	output reg DRAM_nWR,
	output reg [11:0] DRAM_ADDR,

	output reg [3:0] DRAM_nRASA,
	output reg [3:0] DRAM_nCASA,

	output reg [3:0] DRAM_nRASB,
	output reg [3:0] DRAM_nCASB,

	/* Drives external open-drain inverters. */
	output reg STERM,
	output reg CBACK,
	output wire BERR,
	output wire [1:0] DSACK
);

/*
 * We have to synchronize /AS and /RAMSEL because we're dealing with
 * two clock domains.  RnW does not need this treatment because
 * it's not acted upon until our latched /AS signal is stable.
 */
reg AS1;
reg AS_s;
reg RAMSEL1;
reg RAMSEL_s;

always @(posedge CLK, negedge nRST) begin
	if (~nRST) begin
		AS1 <= 1'b0;
		AS_s  <= 1'b0;
		RAMSEL1 <= 1'b0;
		RAMSEL_s  <= 1'b0;
	end
	else begin
		AS1 <= ~nAS;
		RAMSEL1 <= ~nRAMSEL;
		AS_s  <= AS1;
		RAMSEL_s  <= RAMSEL1;
	end
end

/*
 * Qual DSACK and BERR with the synchronized /AS input to ensure they
 * de-assert as quickly as possible.  See parameter 28 of the 68030
 * electrical specifications.
 */
reg [1:0] dsack;
assign DSACK = dsack & {AS_s, AS_s};
reg bus_error;
assign BERR = bus_error & AS_s;

/*
 * The CPU runs at 1/2 the DRAM clock frequency, which is 50MHz.
 * Standard DRAM rows wants to be refreshed within 32 ms.  Since we
 * have 12 address bits, there is a maximum of 4096 rows.  This
 * means we need to refresh a row once every 7.8125 us.  At 50MHz,
 * the clock period is 20 ns, which means max 390 clock cycles between
 * refreshes.  Since memory cycles take a few clock cycles each, we'll
 * give ourselves a 16 clock cycle margin.
 */
localparam REFRESH_CYCLE_CNT = 12'd374;

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
			refresh_cnt <= refresh_cnt + 12'b1;
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
 *
 *	60ns SIMMs are indicated by (PD3 & PD4) == 1;
 */
localparam SZ16  = 3'b101;
localparam SZ32  = 3'b110;
localparam SZ64  = 3'b001;
localparam SZ128 = 3'b010;

/*
 * Compute if the first SIMM is a valid configuration.
 */
wire ValidFirstSIMM = (SIMMPDA[0] ^ SIMMPDA[1]) & (SIMMPDA[2] & SIMMPDA[3]);

/*
 * Compute if the second SIMM is a valid configuration.  We require that
 * both SIMMs be the same.  If they're not, the second SIMM is ignored.
 */
wire ValidSecondSIMM = (SIMMPDB == SIMMPDA);

/*
 * Row address computation.
 */
wire [11:0] RowAddress;
assign RowAddress = SIMMSZ ? {1'b0, ADDR[25:14]}
			   :        ADDR[23:13];

/*
 * Column address computation.
 */
wire [11:0] ColumnAddress;
assign ColumnAddress = SIMMSZ ? {1'b0, ADDR[12:2]}
			      :        ADDR[13:2];

/*
 * Which SIMM computation.  Does the request exceed the limit of the
 * first SIMM?
 */
reg ExceedsFirstSIMM;
always @(*) begin
	case ({SIMMSZ, SIMMPDA[0], SIMMPDA[1]})
	SZ32:    ExceedsFirstSIMM = ADDR[25] || ADDR[26] || ADDR[27];
	SZ64:    ExceedsFirstSIMM = ADDR[26] || ADDR[27];
	SZ128:   ExceedsFirstSIMM = ADDR[27];
	default: ExceedsFirstSIMM =
		     ADDR[24] || ADDR[25] || ADDR[26] || ADDR[27];
	endcase
end

wire UsingSecondSIMM = (ExceedsFirstSIMM && ValidSecondSIMM);

/*
 * Row select computation.
 */
reg [3:0] nRowSelects;
always @(*) begin
	case ({UsingSecondSIMM, SIMMSZ})
	2'b00:	nRowSelects = {~ADDR[26], ADDR[26], ~ADDR[26], ADDR[26]};
	2'b01:	nRowSelects = {~ADDR[24], ADDR[24], ~ADDR[24], ADDR[24]};
	2'b10:	nRowSelects = {~ADDR[27], ADDR[27], ~ADDR[27], ADDR[27]};
	2'b11:	nRowSelects = {~ADDR[25], ADDR[25], ~ADDR[25], ADDR[25]};
	endcase
end

/*
 * Does the request fit in the second SIMM?  Since both SIMMs must be the
 * same size, we're essentially looking for "is the request <= 2x the SIMM
 * size?"
 */
reg FitsSecondSIMM;
always @(*) begin
	case ({SIMMSZ, SIMMPDB[0], SIMMPDB[1]})
	SZ16:    FitsSecondSIMM =
	             ValidSecondSIMM && ~(ADDR[25] || ADDR[26] || ADDR[27]);
	SZ32:    FitsSecondSIMM = ValidSecondSIMM && ~(ADDR[26] || ADDR[27]);
	SZ64:    FitsSecondSIMM = ValidSecondSIMM && ~ADDR[27];
	SZ128:   FitsSecondSIMM = ValidSecondSIMM;
	default: FitsSecondSIMM = 1'b0;	/* invalid second SIMM */
	endcase
end

/*
 * We have a valid address if:
 * 1- We have a valid first SIMM, and
 * 2- Either the request does not require the second SIMM or fits
 *    within it.
 */
wire ValidAddress = ValidFirstSIMM && (~ExceedsFirstSIMM || FitsSecondSIMM);

/*
 * Byte enables, from Table 7-4 in the 68030 User's Manual.
 * N.B. for reads, we enable all bytes.  We mix the RnW signal
 * in to the type and catch it in the default case.
 */
reg [3:0] ByteEnables;
always @(*) begin
	case ({RnW, SIZ[1:0], ADDR[1:0]})
	/* byte writes */
	5'b00100:	ByteEnables = 4'b1000;
	5'b00101:	ByteEnables = 4'b0100;
	5'b00110:	ByteEnables = 4'b0010;
	5'b00111:	ByteEnables = 4'b0001;

	/* word writes */
	5'b01000:	ByteEnables = 4'b1100;
	5'b01001:	ByteEnables = 4'b0110;
	5'b01010:	ByteEnables = 4'b0011;
	5'b01011:	ByteEnables = 4'b0001;

	/* 3 byte writes */
	5'b01100:	ByteEnables = 4'b1110;
	5'b01101:	ByteEnables = 4'b0111;
	5'b01110:	ByteEnables = 4'b0011;
	5'b01111:	ByteEnables = 4'b0001;

	/* long word writes */
	5'b00000:	ByteEnables = 4'b1111;
	5'b00001:	ByteEnables = 4'b0111;
	5'b00010:	ByteEnables = 4'b0011;
	5'b00011:	ByteEnables = 4'b0001;

	/* What remains is: all the reads */
	default:	ByteEnables = 4'b1111;
	endcase
end

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
localparam BERRWAIT	= 4'd11;

reg [3:0] state;

always @(posedge CLK, negedge nRST) begin
	if (~nRST) begin
		state <= IDLE;
		DRAM_ADDR <= 12'd0;
		DRAM_nRASA <= 4'b1111;
		DRAM_nRASB <= 4'b1111;
		DRAM_nCASA <= 4'b1111;
		DRAM_nCASB <= 4'b1111;
		DRAM_nWR <= 1'b1;
		dsack <= 2'b00;
		CBACK <= 1'b0;
		STERM <= 1'b0;
		bus_error <= 1'b0;
		refresh_ack <= 1'b0;
	end
	else begin
		case (state)
		IDLE: begin
			/*
			 * N.B. the time spent in IDLE counts for some
			 * of the precharge time.  (We actually get another
			 * time slice because /RAS isn't asserted until
			 * the end of RW1.)
			 */
			if (refresh_req) begin
				/* Start CAS-before-RAS refresh cycle. */
				state <= REFRESH1;
			end
			else if (RAMSEL_s && AS_s) begin
				/*
				 * DRAM selected.  If we have a valid SIMM
				 * configuration and the request fits within
				 * it, start a normal R/W cycle.  Otherwise,
				 * signal a bus error.
				 */
				if (ValidAddress) begin
					/* Mux in the row address. */
					DRAM_ADDR <= RowAddress;
					state <= RW1;
				end
				else begin
					bus_error <= 1'b1;
					state <= BERRWAIT;
				end
			end
		end

		RW1: begin
			/* Row address is valid, assert RAS. */
			if (ExceedsFirstSIMM)
				DRAM_nRASB <= nRowSelects;
			else
				DRAM_nRASA <= nRowSelects;

			/*
			 * Set the /WE line now to ensure we meet any
			 * write set-up time.
			 */
			DRAM_nWR <= RnW;

			state <= RW2;
		end

		RW2: begin
			/* Mux in the column address. */
			DRAM_ADDR <= ColumnAddress;

			state <= RW3;
		end

		RW3: begin
			/*
			 * Column address is valid, assert CAS based on
			 * the byte enables.
			 */
			if (ExceedsFirstSIMM)
				DRAM_nCASB <= ~ByteEnables;
			else
				DRAM_nCASA <= ~ByteEnables;

			/*
			 * If we wanted to support 70ns or slower
			 * RAM, we would need to insert additional
			 * states here before asserting DSACK.  If
			 * I have enough inputs pins for it, I will
			 * add it, but I'd rather use available
			 * pins for size detection.
			 */
			state <= RW4;
		end

		RW4: begin
			/* Terminate the bus cycle. */
			dsack <= 2'b11;
			state <= RW5;
		end

		RW5: begin
			/*
			 * De-assert our DRAM controls as soon as we
			 * detect de-assertion of /AS (since, due to
			 * synchronization, it happened up to one CPU
			 * clock cycle ago).
			 *
			 * Also note that for slower-than-60ns DRAM, we
			 * would possibly need to extend the precharge
			 * time.
			 */
			if (~AS_s) begin
				DRAM_nRASA <= 4'b1111;
				DRAM_nRASB <= 4'b1111;
				DRAM_nCASA <= 4'b1111;
				DRAM_nCASB <= 4'b1111;
				DRAM_nWR <= 1'b1;
				DRAM_ADDR <= 12'b0;
				dsack <= 2'b00;
				state <= PRECHARGE;
			end
		end

		REFRESH1: begin
			/* Acknowledge refresh request. */
			refresh_ack <= 1'b1;

			/* Assert CAS. */
			DRAM_nCASA <= 4'b0000;
			DRAM_nCASB <= 4'b0000;

			state <= REFRESH2;
		end

		REFRESH2: begin
			/* Assert RAS. */
			DRAM_nRASA <= 4'b0000;
			DRAM_nRASB <= 4'b0000;

			state <= REFRESH3;
		end

		REFRESH3: begin
			/* De-assert CAS. */
			DRAM_nCASA <= 4'b1111;
			DRAM_nCASB <= 4'b1111;

			state <= REFRESH4;
		end

		REFRESH4: begin
			/* De-assert RAS. */
			DRAM_nRASA <= 4'b1111;
			DRAM_nRASB <= 4'b1111;

			/* Refresh cycle is complete. */
			refresh_ack <= 1'b0;

			state <= PRECHARGE;
		end

		PRECHARGE: begin
			/*
			 * /RAS and /CAS are de-asserted before we get
			 * here, so this is effectly a 20ns delay for the
			 * precharge time.  For 60ns DRAM, we typically
			 * need 40ns after /RAS is de-asserted, but we
			 * get another 20ns in the IDLE state.
			 */
			state <= IDLE;
		end

		BERRWAIT: begin
			/* Stay here until the CPU ends the cycle. */
			if (~AS_s) begin
				bus_error <= 1'b0;
				state <= IDLE;
			end
		end
		endcase
	end
end

endmodule

// Pin assignment for Yosys workflow.
//
//PIN: CHIP "dramctl" ASSIGNED TO AN TQFP100
//
//     === Inputs ===
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
//PIN: ADDR_10		: 21
//PIN: ADDR_11		: 22
//PIN: ADDR_12		: 23
//PIN: ADDR_13		: 24
//PIN: ADDR_14		: 25
//PIN: ADDR_15		: 27
//PIN: ADDR_16		: 28
//PIN: ADDR_17		: 29
//PIN: ADDR_18		: 30
//PIN: ADDR_19		: 31
//PIN: ADDR_20		: 32
//PIN: ADDR_21		: 33
//PIN: ADDR_22		: 35
//PIN: ADDR_23		: 36
//PIN: ADDR_24		: 37
//PIN: ADDR_25		: 40
//PIN: ADDR_26		: 41
//PIN: ADDR_27		: 42
//PIN: SIMMPDA_0	: 80
//PIN: SIMMPDA_1	: 81
//PIN: SIMMPDA_2	: 83
//PIN: SIMMPDA_3	: 84
//PIN: SIMMSZ		: 85
//PIN: nAS		: 87
//PIN: nRAMSEL		: 88
//PIN: nRST		: 89
//PIN: CLK		: 90
//PIN: SIMMPDB_0	: 92
//PIN: SIMMPDB_1	: 93
//PIN: SIMMPDB_2	: 94
//PIN: SIMMPDB_3	: 96
//PIN: nCBREQ		: 97
//
//     === Outputs ===
//
//PIN: DSACK_0		: 1
//PIN: DSACK_1		: 2
//PIN: DRAM_nWR		: 45
//PIN: DRAM_ADDR_0	: 46
//PIN: DRAM_ADDR_1	: 47
//PIN: DRAM_ADDR_2	: 48
//PIN: DRAM_ADDR_3	: 49
//PIN: DRAM_ADDR_4	: 50
//PIN: DRAM_ADDR_5	: 52
//PIN: DRAM_ADDR_6	: 53
//PIN: DRAM_ADDR_7	: 54
//PIN: DRAM_ADDR_8	: 55
//PIN: DRAM_ADDR_9	: 56
//PIN: DRAM_ADDR_10	: 57
//PIN: DRAM_ADDR_11	: 58
//PIN: DRAM_nRASA_0	: 60
//PIN: DRAM_nRASA_1	: 61
//PIN: DRAM_nRASA_2	: 63
//PIN: DRAM_nRASA_3	: 64
//PIN: DRAM_nCASA_0	: 65
//PIN: DRAM_nCASA_1	: 67
//PIN: DRAM_nCASA_2	: 68
//PIN: DRAM_nCASA_3	: 69
//PIN: DRAM_nRASB_0	: 70
//PIN: DRAM_nRASB_1	: 71
//PIN: DRAM_nRASB_2	: 72
//PIN: DRAM_nRASB_3	: 75
//PIN: DRAM_nCASB_0	: 76
//PIN: DRAM_nCASB_1	: 77
//PIN: DRAM_nCASB_2	: 78
//PIN: DRAM_nCASB_3	: 79
//PIN: CBACK		: 98
//PIN: STERM		: 99
//PIN: BERR		: 100
