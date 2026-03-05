`timescale 1ns / 1ps

module tb();

`include "mmu_tb_common.v"

initial begin
	$display("MMU TEST: Segment Map control.");

	@(posedge cpu_clk);
	$display("Coming out of reset.");
	n_rst = 1;

	mmu_en = 1;
	sme_v = 0;			/* !SME_V */

	rnw = 1;
	n_as = 0;

	/*
	 * All of these just exercise the combinatorial logic
	 * and validate the expected outcomes.
	 */

	$display("*** SEGMENT MAP SELECTION ***");

	@(posedge cpu_clk);
	fc = FC_SUPER_DATA;
	addr = MMU_SegMap0;
	$display("    Not selected, FC_SUPER_DATA + MMU_SegMap0");
	@(posedge cpu_clk);
	if (~n_smsel) begin
		$fatal(1, "    --> FAILED");
	end

	@(posedge cpu_clk);
	fc = FC_SUPER_DATA;
	addr = MMU_SegMap;
	$display("    Not selected, FC_SUPER_DATA + MMU_SegMap");
	@(posedge cpu_clk);
	if (~n_smsel) begin
		$fatal(1, "    --> FAILED");
	end

	@(posedge cpu_clk);
	fc = FC_CONTROL;
	addr = MMU_SegMap0;
	$display("    Selected, FC_CONTROL + MMU_SegMap0");
	@(posedge cpu_clk);
	if (n_smsel) begin
		$fatal(1, "    --> FAILED");
	end

	@(posedge cpu_clk);
	fc = FC_CONTROL;
	addr = MMU_SegMap;
	$display("    Selected, FC_CONTROL + MMU_SegMap");
	@(posedge cpu_clk);
	if (n_smsel) begin
		$fatal(1, "    --> FAILED");
	end

	$display("*** SEGMENT MAP /WE ***");

	@(posedge cpu_clk);
	fc = FC_SUPER_DATA;
	addr = MMU_SegMap0;
	$display("    Not selected");
	@(posedge cpu_clk);
	if (~n_sm_we) begin
		$fatal(1, "    --> FAILED");
	end

	@(posedge cpu_clk);
	fc = FC_CONTROL;
	rnw = 1;
	addr = MMU_SegMap0;
	$display("    Selected, MMU_SegMap0, R/W=1");
	@(posedge cpu_clk);
	if (~n_sm_we) begin
		$fatal(1, "    --> FAILED");
	end

	@(posedge cpu_clk);
	fc = FC_CONTROL;
	rnw = 1;
	addr = MMU_SegMap;
	$display("    Selected, MMU_SegMap, R/W=1");
	@(posedge cpu_clk);
	if (~n_sm_we) begin
		$fatal(1, "    --> FAILED");
	end

	@(posedge cpu_clk);
	fc = FC_CONTROL;
	rnw = 0;
	addr = MMU_SegMap0;
	$display("    Selected, MMU_SegMap0, R/W=0");
	@(posedge cpu_clk);
	if (n_sm_we) begin
		$fatal(1, "    --> FAILED");
	end

	@(posedge cpu_clk);
	fc = FC_CONTROL;
	rnw = 0;
	addr = MMU_SegMap;
	$display("    Selected, MMU_SegMap, R/W=0");
	@(posedge cpu_clk);
	if (n_sm_we) begin
		$fatal(1, "    --> FAILED");
	end

	$display("*** SEGMENT MAP /UB, /LB ***");

	@(posedge cpu_clk);
	fc = FC_SUPER_DATA;
	addr = MMU_SegMap0;
	$display("    Not selected");
	@(posedge cpu_clk);
	if (n_sm_ub) begin
		$fatal(1, "    --> FAILED /UB");
	end
	if (n_sm_lb) begin
		$fatal(1, "    --> FAILED /LB");
	end

	@(posedge cpu_clk);
	fc = FC_CONTROL;
	addr = MMU_SegMap0;
	$display("    /UB, /LB, selected, no data strobes");
	@(posedge cpu_clk);
	if (~n_sm_ub) begin
		$fatal(1, "    --> FAILED /UB");
	end
	if (~n_sm_lb) begin
		$fatal(1, "    --> FAILED /LB");
	end

	@(posedge cpu_clk);
	fc = FC_CONTROL;
	addr = MMU_SegMap0;
	n_uds = 0;
	$display("    /UB, /LB, selected, /UDS");
	@(posedge cpu_clk);
	if (n_sm_ub) begin
		$fatal(1, "    --> FAILED /UB");
	end
	if (~n_sm_lb) begin
		$fatal(1, "    --> FAILED /LB");
	end

	@(posedge cpu_clk);
	fc = FC_CONTROL;
	addr = MMU_SegMap0;
	n_uds = 1;
	n_lds = 0;
	$display("    /UB, /LB, selected, /LDS");
	@(posedge cpu_clk);
	if (~n_sm_ub) begin
		$fatal(1, "    --> FAILED /UB");
	end
	if (n_sm_lb) begin
		$fatal(1, "    --> FAILED /LB");
	end

	@(posedge cpu_clk);
	$finish;
end

endmodule
