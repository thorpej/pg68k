`timescale 1ns / 1ps

module tb();

`include "mmu_tb_common.v"

initial begin
	$display("MMU TEST: Segment Map access.");

	@(posedge cpu_clk);
	$display("Coming out of reset.");
	n_rst = 1;

	mmu_en = 1;
	sme_v = 1;

	@(posedge cpu_clk);
	$display("S0: CPU drives FC[2:0] and RnW=1");
	fc = FC_CONTROL;
	rnw = 1;

	@(negedge cpu_clk);
	$display("S1: CPU drives A[23:1]");
	addr = MMU_SegMap0;

	@(posedge cpu_clk);
	$display("S2: CPU asserts /AS, RnW=0");
	n_as = 0;
	rnw = 0;

	@(negedge cpu_clk);
	$display("S3: CPU places data on data bus");

	/* Verify MMU computations: */
	if (mmu_dtack) begin
		$fatal(1, "FATAL: MMU ACK'd SegMap cycle early.");
	end

	@(posedge cpu_clk);
	$display("S4: CPU asserts /UDS and /LDS, waits for cycle termination signal");
	n_uds = 0;
	n_lds = 0;

	@(negedge cpu_clk);
	$display("S5: no bus signals are altered");

	/* /UDS is asserted later in write cycles. */

	/* Verify MMU computations: */
	if (~mmu_dtack) begin
		$fatal(1, "FATAL: MMU failed to ACK SegMap cycle.");
	end
	n_dtack = 0;

	@(posedge cpu_clk);
	$display("S6: no bus signals are altered");

	@(negedge cpu_clk);
	$display("S7: CPU negates /AS, /UDS");
	n_as = 1;
	n_uds = 1;
	n_lds = 1;

	@(posedge cpu_clk);
	n_dtack = 1;

	@(posedge cpu_clk);
	$finish;
end

endmodule
