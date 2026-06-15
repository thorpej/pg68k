`timescale 1ns / 1ps

module tb();

integer i;

`include "memctl_tb_common.v"

initial begin
	$display("MEMCTL TEST: ROMSEL");

	#50;

	/*
	 * All of these just exercise the combinatorial logic
	 * and validate the expected outcomes.
	 *
	 * Ensure /ROMSEL is asserted (only) in the situations where
	 * it should be.
	 */

	mmu_en = 1;

	n_as = 0;
	fc = FC_SUPER_DATA;

	for (i = 0; i < ADDR_LIM; i = i + ONE_MB) begin
		addr = i;
		#10;
		if (addr >= ROM_START && addr < ROM_END &&
		    (n_ramsel != ALL_RAM_UNSEL ||
		     ~n_expramsel)) begin
			$display("--> addr=%8x decode=%0d mmu_en=%0d space=%0d bank=%5b",
			    addr, decode_out, mmu_en, space_out, bank_out);
			$fatal(1,
			    "    --> FAILED unexpected n_ramsel=%4b n_expramsel=%0b",
			    n_ramsel, n_expramsel);
		end
		if (addr >= ROM_START && addr < ROM_END &&
		    n_romsel) begin
			$display("--> addr=%8x decode=%0d mmu_en=%0d space=%0d bank=%5b",
			    addr, decode_out, mmu_en, space_out, bank_out);
			$fatal(1,
			    "    --> FAILED expected /ROMSEL");
		end
	end

	#10;
	$finish;
end

endmodule
