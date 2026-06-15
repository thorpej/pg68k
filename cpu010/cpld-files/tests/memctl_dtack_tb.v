`timescale 1ns / 1ps

module tb();

integer i;

`include "memctl_tb_common.v"

/*
 * We're only supposed to see the DTACK output for on-board RAM
 * and ROM.
 */
wire want_dtack = ((addr >= RAM0_START && addr < RAM3_END) ||
		   (addr >= ROM_START && addr < ROM_END))
	&& (~n_uds || ~n_lds);

initial begin
	$display("MEMCTL TEST: DTACK");

	#50;

	/*
	 * All of these just exercise the combinatorial logic
	 * and validate the expected outcomes.
	 *
	 * Ensure DTACK is asserted (only) in the situations where
	 * it should be.
	 */

	mmu_en = 1;

	n_as = 0;
	fc = FC_SUPER_DATA;

	$display("*** /xDS negated ***");
	n_uds = 1;
	n_lds = 1;

	for (i = 0; i < ADDR_LIM; i = i + ONE_MB) begin
		addr = i;
		#10;
		if (want_dtack != dtack) begin
			$fatal(1, "    --> FAILED addr=%8x dtack=%0d",
			    addr, dtack);
		end
	end

	#10;

	$display("*** /UDS asserted, /LDS negated ***");
	n_uds = 0;
	n_lds = 1;

	for (i = 0; i < ADDR_LIM; i = i + ONE_MB) begin
		addr = i;
		#10;
		if (want_dtack != dtack) begin
			$fatal(1, "    --> FAILED addr=%8x dtack=%0d",
			    addr, dtack);
		end
	end

	#10;

	$display("*** /UDS negated, /LDS asserted ***");
	n_uds = 1;
	n_lds = 0;

	for (i = 0; i < ADDR_LIM; i = i + ONE_MB) begin
		addr = i;
		#10;
		if (want_dtack != dtack) begin
			$fatal(1, "    --> FAILED addr=%8x dtack=%0d",
			    addr, dtack);
		end
	end

	#10;

	$display("*** /xDS asserted ***");
	n_uds = 0;
	n_lds = 0;

	for (i = 0; i < ADDR_LIM; i = i + ONE_MB) begin
		addr = i;
		#10;
		if (want_dtack != dtack) begin
			$fatal(1, "    --> FAILED addr=%8x dtack=%0d",
			    addr, dtack);
		end
	end

	#10;
	$finish;
end

endmodule
