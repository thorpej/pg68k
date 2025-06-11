/*
 * DRAM controller for the Playground 68030.  This is almost entirely
 * borrowed from crmaykish's mackerel-30, with adjustments to support
 * configurable SIMM sizes.
 *
 * XXX And, eventually, multiple SIMMs.
 */
module dramctl(
	input nRST,
	input CLK,

	input nCS,
	input RnW,
	input nAS,
	input nDS,

	input SIZ0, SIZ1,

	input [27:0] ADDR,	/* good for 256MB (2x128MB) */

	output reg DRAM_nWR,
	output reg [11:0] DRAM_ADDR,
	output reg [3:0] DRAM_nRAS,
	output reg [3:0] DRAM_nCAS,

	/* These drive open-drain inverters. */
	output reg DSACK0, DSACK1
);

/*
 * 25MHz system clock = 40ns clock period.  15.6 uS is a very common
 * refresh cycle interval, so we'll do one every 375 clocks.
 */
localparam REFRESH_CYCLE_CNT = 374;	/* fencepost */

/*
 * DRAM refresh generator.  We count clocks and when we reach the
 * refresh cycle count, we ask the DRAM state machine to perform
 * a refresh cycle.
 */
reg refresh_req = 1'b0;
reg refresh_ack = 1'b0;
reg [11:0] refresh_cnt = 12'b0;

always @(posedge CLK) begin
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
 * Row address computation.
 */
function [11:0] ComputeRowAddress(
	input [27:0] addr
);
begin
	ComputeRowAddress = addr[13:2];
end
endfunction

wire [11:0] RowAddress = ComputeRowAddress(ADDR);

/*
 * Column address computation.
 */
function [11:0] ComputeColumnAddress(
	input [27:0] addr
);
begin
	ComputeColumnAddress = addr[25:14];
end
endfunction

wire [11:0] ColumnAddress = ComputeColumnAddress(ADDR);

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
	enabs = 4'b1111;	/* default */
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

reg [3:0] state = IDLE;

always @(posedge CLK) begin
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
			else if (~nCS && ~nAS) begin
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

			/* XXX
			 * This is set up for 64/128 SIMMs.  If A26
			 * is 0, the first side of the SIMM is used.
			 * otherwise the second side.
			 * XXX Fix for other sizes.
			 */
			DRAM_nRAS[0] <= ADDR[26];
			DRAM_nRAS[1] <= ~ADDR[26];
			DRAM_nRAS[2] <= ADDR[26];
			DRAM_nRAS[3] <= ~ADDR[26];

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
			if (nAS) state <= PRECHARGE;
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
