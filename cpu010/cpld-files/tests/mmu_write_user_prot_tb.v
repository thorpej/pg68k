`timescale 1ns / 1ps

module tb();

`include "mmu_tb_common.v"

initial begin
	$display("MMU TEST: User write cycle, prot error.");

	@(posedge cpu_clk);
	$display("Coming out of reset.");
	n_rst = 1;

	mmu_en = 1;
	sme_v = 1;

	@(posedge cpu_clk);
	$display("S0: CPU drives FC[2:0] and RnW=1");
	fc = FC_USER_DATA;
	rnw = 1;

	@(negedge cpu_clk);
	$display("S1: CPU drives A[23:1]");

	@(posedge cpu_clk);
	$display("S2: CPU asserts /AS, RnW=0");
	pme_from_sram = 8'b10000000;	/* V */
	n_as = 0;
	rnw = 0;

	@(negedge cpu_clk);
	$display("S3: CPU places data on data bus");

	/* Verify no PTE REF+MOD write-back. */
	@(posedge mmu_clk);
	if (~n_pmu_we) begin
		$fatal(1, "FATAL: MMU unexpectedly attempted to write-back to upper PageMap.");
	end

	@(posedge cpu_clk);
	$display("S4: CPU asserts /UDS, waits for cycle termination signal");
	n_uds = 0;

	/* Verify MMU computations: */
	if (mmu_dtack) begin
		$fatal(1, "FATAL: MMU ACK'd regular cycle.");
	end
	if (~mmu_addr) begin
		$fatal(1, "FATAL: MMU not driving address bus.");
	end

	if (~n_sm_we) begin
		$fatal(1, "FATAL: MMU unexpectedly writing to SegMap.");
	end
	if (n_sm_ub) begin
		$fatal(1, "FATAL: MMU not enabling SegMap upper byte.");
	end
	if (n_sm_lb) begin
		$fatal(1, "FATAL: MMU not enabling SegMap lower byte.");
	end
	if (~n_pmu_we) begin
		$fatal(1, "FATAL: MMU unexpectedly writing to upper PageMap.");
	end
	if (~n_pml_we) begin
		$fatal(1, "FATAL: MMU unexpectedly writing to lower PageMap.");
	end
	if (n_pmu_ub) begin
		$fatal(1, "FATAL: MMU not enabling upper PageMap upper byte.");
	end
	if (n_pml_ub) begin
		$fatal(1, "FATAL: MMU not enabling lower PageMap upper byte.");
	end
	if (n_pml_lb) begin
		$fatal(1, "FATAL: MMU not enabling lower PageMap lower byte.");
	end

	if (~n_as_out) begin
		$fatal(1, "FATAL: MMU unexpectedly un-gated /AS.");
	end
	if (n_berr_out) begin
		$fatal(1, "FATAL: MMU failed to signal /BERR.");
	end

	if (bus_error_reg != BUS_ERROR_PROT) begin
		$fatal(1, "FATAL: BUS ERROR REG=%b EXPECTED=%b",
		    bus_error_reg, BUS_ERROR_PROT);
	end

	@(negedge cpu_clk);
	$display("S5: no bus signals are altered");

	/* /UDS is asserted later in write cycles. */
	if (~n_uds_out) begin
		$fatal(1, "FATAL: MMU unexpectedly un-gated /UDS.");
	end

	@(posedge cpu_clk);
	$display("S6: no bus signals are altered");

	@(negedge cpu_clk);
	$display("S7: CPU negates /AS, /UDS");
	n_as = 1;
	n_uds = 1;

	@(posedge cpu_clk);
	n_dtack = 1;

	@(posedge cpu_clk);
	$finish;
end

endmodule
