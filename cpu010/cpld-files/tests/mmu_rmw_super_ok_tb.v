`timescale 1ns / 1ps

module tb();

`include "mmu_tb_common.v"

initial begin
	$display("MMU TEST: Supervisor R-M-W cycle, valid translation.");

	@(posedge cpu_clk);
	$display("Coming out of reset.");
	n_rst = 1;

	mmu_en = 1;
	sme_v = 1;

	@(posedge cpu_clk);
	$display("S0: CPU drives FC[2:0] and RnW=1");
	fc = FC_SUPER_DATA;
	rnw = 1;

	@(negedge cpu_clk);
	$display("S1: CPU drives A[23:1]");

	@(posedge cpu_clk);
	$display("S2: CPU asserts /AS, /UDS");
	pme_from_sram = 8'b11100000;	/* V+W+K */
	n_as = 0;
	n_uds = 0;

	@(negedge cpu_clk);
	$display("S3: no bus signals are altered");

	/* Verify PTE REF write-back. */
	@(posedge mmu_clk);
	if (n_pmu_we) begin
		$fatal(1, "FATAL: MMU failed to write-back to upper PageMap.");
	end
	if (n_pmu_ub) begin
		$fatal(1, "FATAL: MMU write-back not enabling upper PageMap upper byte.");
	end
	if (~n_pmu_lb) begin
		$fatal(1, "FATAL: MMU write-back unexpectedly enabling upper PageMap lower byte.");
	end
	if (~pme[1]) begin
		$fatal(1, "FATAL: MMU write-back did not set REF bit.");
	end
	if (pme[0]) begin
		$fatal(1, "FATAL: MMU write-back unexpectedly set MOD bit.");
	end

	@(posedge cpu_clk);
	$display("S4: CPU waits for cycle termination signal");
	n_dtack = 0;

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
	$display("S7: CPU latches data, negates /UDS");
	n_uds = 1;

	@(posedge cpu_clk);
	$display("S8: no bus signals are altered");
	n_dtack = 1;

	@(negedge cpu_clk);
	$display("S9: no bus signals are altered");

	@(posedge cpu_clk);
	$display("S10: no bus signals are altered");

	@(negedge cpu_clk);
	$display("S11: no bus signals are altered");

	@(posedge cpu_clk);
	$display("S12: write portion of cycle begins");

	@(negedge cpu_clk);
	$display("S13: no bus signals are altered");

	@(posedge cpu_clk);
	$display("S14: CPU drives RnW=0");
	rnw = 0;

	@(negedge cpu_clk);
	$display("S15: CPU places data on data bus");

	/* Verify PTE REF+MOD write-back. */
	@(posedge mmu_clk);
	if (n_pmu_we) begin
		$fatal(1, "FATAL: MMU failed to write-back to upper PageMap.");
	end
	if (n_pmu_ub) begin
		$fatal(1, "FATAL: MMU write-back not enabling upper PageMap upper byte.");
	end
	if (~n_pmu_lb) begin
		$fatal(1, "FATAL: MMU write-back unexpectedly enabling upper PageMap lower byte.");
	end
	if (~pme[1]) begin
		$fatal(1, "FATAL: MMU write-back did not set REF bit.");
	end
	if (~pme[0]) begin
		$fatal(1, "FATAL: MMU write-back did not set MOD bit.");
	end

	@(posedge cpu_clk);
	$display("S16: CPU asserts /UDS, waits for cycle termination signal");
	n_uds = 0;
	n_dtack = 0;

	if (n_as_out) begin
		$fatal(1, "FATAL: MMU failed to un-gate /AS.");
	end
	if (~n_berr_out) begin
		$fatal(1, "FATAL: MMU unexpedly signaled /BERR.");
	end

	@(negedge cpu_clk);
	$display("S17: no bus signals are altered");

	/* /UDS is asserted later in write cycles. */
	if (n_uds_out) begin
		$fatal(1, "FATAL: MMU failed to un-gate /UDS.");
	end

	@(posedge cpu_clk);
	$display("S18: no bus signals are altered");

	@(negedge cpu_clk);
	$display("S19: CPU negates /AS, /UDS");
	n_as = 1;
	n_uds = 1;

	@(posedge cpu_clk);
	n_dtack = 1;

	@(posedge cpu_clk);
	$finish;
end

endmodule
