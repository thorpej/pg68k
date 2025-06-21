/*
 * DRAM controller for the Playground 68030.  This is based on crmaykish's
 * mackerel-30 DRAM controller, but has been modified to support multiple
 * SIMM sizes (16MB, 32MB, 64MB, or 128MB -- all SIMMs must be the same size)
 * and 2 SIMMs.  This consumes almost all of the available I/O on a TQFP100
 * ATF1508AS.
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
 * so maybe I'll do it someday.
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

	output reg DRAM_nWR,
	output reg [11:0] DRAM_ADDR,

	output reg [3:0] DRAM_nRASA,
	output reg [3:0] DRAM_nCASA,

	output reg [3:0] DRAM_nRASB,
	output reg [3:0] DRAM_nCASB,

	/* Drives external open-drain inverters. */
	output reg BERR,
	output reg [1:0] DSACK
);

/*
 * We have to synchronize /AS and /RAMSEL because we're dealing with
 * two clock domains.  RnW does not need this treatment because
 * it's not acted upon until our latched /AS signal is stable.
 */
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
		AS1 <= ~nAS;
		RAMSEL1 <= ~nRAMSEL;
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
 *	SIMMSZ	PD1	PD2
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
 * Which SIMM computation.
 */
reg SecondSIMM;
always @(*) begin
	case ({SIMMSZ, SIMMPDA[0], SIMMPDA[1]})
	SZ32:		SecondSIMM = ADDR[25];
	SZ64:		SecondSIMM = ADDR[26];
	SZ128:		SecondSIMM = ADDR[27];
	default:	SecondSIMM = ADDR[24];	/* default to 16MB boundary */
	endcase
end

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
localparam SIGNALBERR	= 4'd11;

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
		DSACK <= 2'b00;
		BERR <= 1'b0;
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
				/*
				 * DRAM selected.  If we have a valid SIMM
				 * configuration, start a normal R/W cycle.
				 * Otherwise, signal a bus error.
				 */
				if (~ValidFirstSIMM ||
				    (SecondSIMM & ~ValidSecondSIMM))
					state <= SIGNALBERR;
				else
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
			if (SecondSIMM)
				DRAM_nRASB <= nRowSelects;
			else
				DRAM_nRASA <= nRowSelects;

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
			if (SecondSIMM)
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
			state <= RW5;
		end

		RW5: begin
			/* Data is valid; terminate the cycle. */
			DSACK <= 2'b11;

			/* Stay in RW5 until the CPU ends the cycle. */
			if (~AS) state <= PRECHARGE;
		end

		REFRESH1: begin
			/* Acknowledge refresh request. */
			refresh_ack <= 1'b1;

			/* De-assert WE. */
			DRAM_nWR <= 1'b1;

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

			state <= PRECHARGE;
		end

		PRECHARGE: begin
			/*
			 * DRAM cycle finished.  De-assert RAS and CAS,
			 * de-assert DSACK.
			 */
			DRAM_nRASA <= 4'b1111;
			DRAM_nRASB <= 4'b1111;
			DRAM_nCASA <= 4'b1111;
			DRAM_nCASB <= 4'b1111;
			DRAM_ADDR <= 12'b0;
			DSACK <= 2'b00;
			refresh_ack <= 1'b0;

			state <= IDLE;
		end

		SIGNALBERR: begin
			/* Stay here until the CPU ends the cycle. */
			if (AS)
				BERR <= 1'b1;
			else begin
				BERR <= 1'b0;
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
//PIN: nAS		: 1
//PIN: nRAMSEL		: 2
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
//PIN: nRST		: 89
//PIN: CLK		: 90
//PIN: SIMMPDB_0	: 92
//PIN: SIMMPDB_1	: 93
//PIN: SIMMPDB_2	: 94
//PIN: SIMMPDB_3	: 96
//
//     === Outputs ===
//
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
//PIN: BERR		: 98
//PIN: DSACK_0		: 99
//PIN: DSACK_1		: 100
