`timescale 1ns / 1ps

module tb();

integer i;

`include "memctl_tb_common.v"

initial begin
	$display("MEMCTL TEST: Strobe logic.");

	#50;

	/*
	 * All of these just exercise the combinatorial logic
	 * and validate the expected outcomes.
	 *
	 * This one validates that the upper/lower read/write
	 * strobes are asserted at the correct times.
	 */

	$display("*** /xDS negated, RnW=1 ***");
	n_uds = 1;
	n_lds = 1;
	rnw = 1;

	#10;

	if (~n_u_rd || ~n_l_rd || ~n_u_wr || ~n_l_wr) begin
		$fatal(1,
		   "    --> FAILED n_u_rd=%0d n_l_rd=%0d n_u_wr=%0d n_l_wr=%0d",
		   n_u_rd, n_l_rd, n_u_wr, n_l_wr);
	end

	#10;

	$display("*** /xDS negated, RnW=0 ***");
	n_uds = 1;
	n_lds = 1;
	rnw = 1;

	#10;

	if (~n_u_rd || ~n_l_rd || ~n_u_wr || ~n_l_wr) begin
		$fatal(1,
		   "    --> FAILED n_u_rd=%0d n_l_rd=%0d n_u_wr=%0d n_l_wr=%0d",
		   n_u_rd, n_l_rd, n_u_wr, n_l_wr);
	end

	#10;

	$display("*** /UDS asserted, /LDS negated, RnW=1 ***");
	n_uds = 0;
	n_lds = 1;
	rnw = 1;

	#10;

	if (n_u_rd || ~n_l_rd || ~n_u_wr || ~n_l_wr) begin
		$fatal(1,
		   "    --> FAILED n_u_rd=%0d n_l_rd=%0d n_u_wr=%0d n_l_wr=%0d",
		   n_u_rd, n_l_rd, n_u_wr, n_l_wr);
	end

	#10;

	$display("*** /UDS asserted, /LDS negated, RnW=0 ***");
	n_uds = 0;
	n_lds = 1;
	rnw = 0;

	#10;

	if (~n_u_rd || ~n_l_rd || n_u_wr || ~n_l_wr) begin
		$fatal(1,
		   "    --> FAILED n_u_rd=%0d n_l_rd=%0d n_u_wr=%0d n_l_wr=%0d",
		   n_u_rd, n_l_rd, n_u_wr, n_l_wr);
	end

	#10;

	$display("*** /UDS negated, /LDS asserted, RnW=1 ***");
	n_uds = 1;
	n_lds = 0;
	rnw = 1;

	#10;

	if (~n_u_rd || n_l_rd || ~n_u_wr || ~n_l_wr) begin
		$fatal(1,
		   "    --> FAILED n_u_rd=%0d n_l_rd=%0d n_u_wr=%0d n_l_wr=%0d",
		   n_u_rd, n_l_rd, n_u_wr, n_l_wr);
	end

	#10;

	$display("*** /UDS negated, /LDS asserted, RnW=0 ***");
	n_uds = 1;
	n_lds = 0;
	rnw = 0;

	#10;

	if (~n_u_rd || ~n_l_rd || ~n_u_wr || n_l_wr) begin
		$fatal(1,
		   "    --> FAILED n_u_rd=%0d n_l_rd=%0d n_u_wr=%0d n_l_wr=%0d",
		   n_u_rd, n_l_rd, n_u_wr, n_l_wr);
	end

	#10;

	$display("*** /xDS asserted, RnW=1 ***");
	n_uds = 0;
	n_lds = 0;
	rnw = 1;

	#10;

	if (n_u_rd || n_l_rd || ~n_u_wr || ~n_l_wr) begin
		$fatal(1,
		   "    --> FAILED n_u_rd=%0d n_l_rd=%0d n_u_wr=%0d n_l_wr=%0d",
		   n_u_rd, n_l_rd, n_u_wr, n_l_wr);
	end

	#10;

	$display("*** /xDS asserted, RnW=0 ***");
	n_uds = 0;
	n_lds = 0;
	rnw = 0;

	#10;

	if (~n_u_rd || ~n_l_rd || n_u_wr || n_l_wr) begin
		$fatal(1,
		   "    --> FAILED n_u_rd=%0d n_l_rd=%0d n_u_wr=%0d n_l_wr=%0d",
		   n_u_rd, n_l_rd, n_u_wr, n_l_wr);
	end

	#10;
	$finish;
end

endmodule
