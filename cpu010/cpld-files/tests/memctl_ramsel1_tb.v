`timescale 1ns / 1ps

module tb();

integer i;

`include "memctl_tb_common.v"

initial begin
	$display("MEMCTL TEST: RAMSEL[1]");

	#50;

	/*
	 * All of these just exercise the combinatorial logic
	 * and validate the expected outcomes.
	 *
	 * Ensure /RAMSEL[0] is asserted (only) in the situations where
	 * it should be.
	 */

	mmu_en = 1;

	n_as = 0;
	fc = FC_SUPER_DATA;

	for (i = 0; i < ADDR_LIM; i = i + ONE_MB) begin
		addr = i;
		#10;
		if (addr >= RAM1_START && addr < RAM1_END &&
		    (n_ramsel != RAM1_ONLYSEL ||
		     ~n_romsel || ~n_expramsel)) begin
			$display("--> addr=%8x decode=%0d mmu_en=%0d space=%0d bank=%5b",
			    addr, decode_out, mmu_en, space_out, bank_out);
			$fatal(1,
			    "    --> FAILED unexpected n_ramsel=%4b n_expramsel=%0b n_romsel=%0b",
			    n_ramsel, n_expramsel, n_romsel);
		end
	end

	#10;
	$finish;
end

endmodule
