`timescale 1ns / 1ps

module tb();

integer i;

`include "memctl_tb_common.v"

initial begin
	$display("MEMCTL TEST: Decode rules.");

	#50;

	/*
	 * All of these just exercise the combinatorial logic
	 * and validate the expected outcomes.
	 *
	 * These tests determine of the internal "I should decode
	 * this address" logic is working correctly.  It should
	 * only decode for {User,Super}{Data,Program}, and only
	 * if /AS is asserted.
	 */

	$display("*** /AS negated ***");
	n_as = 1;
	for (i = 0; i < 8; i++) begin
		#10
		fc = i;
		if (decode_out) begin
			$fatal(1, "    --> FAILED fc=%0d", fc);
		end
	end

	#10

	$display("*** /AS asserted ***");
	n_as = 0;
	for (i = 0; i < 8; i++) begin
		#10
		fc = i;
		if (decode_out && ~want_decode) begin
			$fatal(1,
			    "    --> FAILED unexpected decode fc=%0d", fc);
		end
		if (~decode_out && want_decode) begin
			$fatal(1,
			    "    --> FAILED expected decode fc=%0d", fc);
		end
	end

	#10;
	$finish;
end

endmodule
