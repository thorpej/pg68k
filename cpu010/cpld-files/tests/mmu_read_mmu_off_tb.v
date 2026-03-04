`timescale 1ns / 1ps

module tb();

`include "mmu_tb_common.v"

initial begin
	$display("MMU TEST: Read cycle, MMU disabled.");

	@(posedge cpu_clk);
	$display("Coming out of reset.");
	n_rst = 1;

	@(posedge cpu_clk);
	$display("S0: CPU drives FC[2:0] and RnW=1");
	fc = FC_SUPER_DATA;
	rnw = 1;

	@(negedge cpu_clk);
	$display("S1: CPU drives A[23:1]");

	@(posedge cpu_clk);
	$display("S2: CPU asserts /AS, /UDS");
	n_as = 0;
	n_uds = 0;

	@(negedge cpu_clk);
	$display("S3: no bus signals are altered");

	@(posedge cpu_clk);
	$display("S4: CPU waits for cycle termination signal");
	n_dtack = 0;

	/* Verify MMU computations: */
	if (mmu_dtack) begin
		$fatal(1, "FATAL: MMU ACK'd regular cycle.");
	end
	if (mmu_addr) begin
		$fatal(1, "FATAL: MMU unexpectedly driving address bus.");
	end
	if (n_as_out) begin
		$fatal(1, "FATAL: MMU failed to un-gate /AS.");
	end
	if (n_uds_out) begin
		$fatal(1, "FATAL: MMU failed to un-gate /UDS.");
	end
	if (~n_berr_out) begin
		$fatal(1, "FATAL: MMU unexpedly signaled /BERR.");
	end

	@(negedge cpu_clk);
	$display("S5: no bus signals are altered");

	@(posedge cpu_clk);
	$display("S6: data from device is driven onto data bus");

	@(negedge cpu_clk);
	$display("S7: CPU latches data, negates /AS, /UDS");

	@(posedge cpu_clk);
	n_dtack = 1;

	@(posedge cpu_clk);
	$finish;
end

endmodule
