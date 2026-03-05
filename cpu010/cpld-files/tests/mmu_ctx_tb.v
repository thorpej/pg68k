`timescale 1ns / 1ps

module tb();

`include "mmu_tb_common.v"

initial begin
	$display("MMU TEST: Context Register / selection.");

	@(posedge cpu_clk);
	$display("Coming out of reset.");
	n_rst = 1;

	mmu_en = 1;
	sme_v = 0;			/* !SME_V */

	$display("*** WRITING 1 TO CONTEXT REGISTER ***");

	@(posedge cpu_clk);
	$display("S0: CPU drives FC[2:0] and RnW=1");
	fc = FC_CONTROL;
	rnw = 1;

	@(negedge cpu_clk);
	$display("S1: CPU drives A[23:1]");
	addr = MMU_ContextReg;

	@(posedge cpu_clk);
	$display("S2: CPU asserts /AS, RnW=0");
	pme_from_sram = 8'b00000000;	/* !V */
	n_as = 0;
	rnw = 0;

	@(negedge cpu_clk);
	$display("S3: CPU places data on data bus");
	data_out = 8'd1;

	/* Verify no PTE REF+MOD write-back. */
	@(posedge mmu_clk);
	if (~n_pmu_we) begin
		$fatal(1, "FATAL: MMU unexpectedly attempted to write-back to upper PageMap.");
	end

	@(posedge cpu_clk);
	$display("S4: CPU asserts /UDS, waits for cycle termination signal");
	n_uds = 0;

	/* Verify MMU computations: */
	if (mmu_addr) begin
		$fatal(1, "FATAL: MMU unexpectedly driving address bus.");
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
	if (~n_berr_out) begin
		$fatal(1, "FATAL: MMU unexpedly signaled /BERR.");
	end

	@(negedge cpu_clk);
	$display("S5: no bus signals are altered");

	if (~mmu_dtack) begin
		$fatal(1, "FATAL: MMU failed to ACK control cycle.");
	end

	/* /UDS is asserted later in write cycles. */
	if (n_uds_out) begin
		$fatal(1, "FATAL: MMU failed to un-gate /UDS.");
	end

	@(posedge cpu_clk);
	$display("S6: no bus signals are altered");

	@(negedge cpu_clk);
	$display("S7: CPU negates /AS, /UDS");
	n_as = 1;
	n_uds = 1;

	@(posedge cpu_clk);
	n_dtack = 1;

	/*
	 * For the remainder of these tests, we don't need to do real
	 * bus cycles, but rather just toggle the address and /AS
	 * signals.
	 */

	$display("*** USER ACCESS, EXPECT CONTEXT 1. ***");
	fc = FC_USER_DATA;
	n_as = 1;
	@(posedge cpu_clk);
	if (ctx !== 6'd1) begin
		$fatal(1, "FATAL: MMU asserted CTX=%0d.", ctx);
	end

	$display("*** SUPER ACCESS, EXPECT CONTEXT 0. ***");
	fc = FC_SUPER_DATA;
	n_as = 1;
	@(posedge cpu_clk);
	if (ctx !== 6'd0) begin
		$fatal(1, "FATAL: MMU asserted CTX=%0d.", ctx);
	end

	$display("*** SegMap0 ACCESS, EXPECT CONTEXT 0. ***");
	fc = FC_CONTROL;
	addr = MMU_SegMap0;
	n_as = 1;
	@(posedge cpu_clk);
	if (ctx !== 6'd0) begin
		$fatal(1, "FATAL: MMU asserted CTX=%0d.", ctx);
	end

	$display("*** SegMap ACCESS, EXPECT CONTEXT 1. ***");
	fc = FC_CONTROL;
	addr = MMU_SegMap;
	n_as = 1;
	@(posedge cpu_clk);
	if (ctx !== 6'd1) begin
		$fatal(1, "FATAL: MMU asserted CTX=%0d.", ctx);
	end

	@(posedge cpu_clk);
	$display("*** READING CONTEXT REGISTER, EXPECT 1 ***");
	data_out = 8'hFF;	/* poison data_out */
	fc = FC_USER_DATA;

	@(posedge cpu_clk);
	$display("S0: CPU drives FC[2:0] and RnW=1");
	fc = FC_CONTROL;
	rnw = 1;

	@(negedge cpu_clk);
	$display("S1: CPU drives A[23:1]");
	addr = MMU_ContextReg;

	@(posedge cpu_clk);
	$display("S2: CPU asserts /AS, /UDS");
	n_as = 0;
	n_uds = 0;

	@(negedge cpu_clk);
	$display("S3: no bus signals are altered");

	/* Verify no PTE REF write-back. */
	@(posedge mmu_clk);
	if (~n_pmu_we) begin
		$fatal(1, "FATAL: MMU unexpectedly attempted to write-back to upper PageMap.");
	end

	@(posedge cpu_clk);
	$display("S4: CPU waits for cycle termination signal");

	/* Verify MMU computations: */
	if (mmu_addr) begin
		$fatal(1, "FATAL: MMU unexpectedly driving address bus.");
	end
	if (~mmu_dtack) begin
		$fatal(1, "FATAL: MMU failed to ACK control cycle.");
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
	if (data !== 8'd1) begin
		$fatal(1, "FATAL: MMU returned ContextReg=%0d.", data);
	end

	@(negedge cpu_clk);
	$display("S7: CPU latches data, negates /AS, /UDS");
	n_as = 1;
	n_uds = 1;

	@(posedge cpu_clk);
	n_dtack = 1;

	@(posedge cpu_clk);
	$finish;
end

endmodule
