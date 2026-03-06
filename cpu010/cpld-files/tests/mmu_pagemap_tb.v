`timescale 1ns / 1ps

module tb();

`include "mmu_tb_common.v"

initial begin
	$display("MMU TEST: Page Map control.");

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
	 *
	 * PageMap control signals during normal bus cycles are
	 * already validated by other test cases.  This is meant
	 * to perform simple exercises of direct PageMap selection.
	 */

	$display("*** PAGE MAP SELECTION ***");

	@(posedge cpu_clk);
	fc = FC_SUPER_DATA;
	addr = MMU_PageMapU;
	$display("    Not selected, FC_SUPER_DATA + MMU_PageMapU");
	@(posedge cpu_clk);
	if (~n_pmusel || pmacc) begin
		$fatal(1, "    --> FAILED n_pmusel=%0d pmacc=%0d",
		    n_pmusel, pmacc);
	end

	@(posedge cpu_clk);
	fc = FC_SUPER_DATA;
	addr = MMU_PageMapL;
	$display("    Not selected, FC_SUPER_DATA + MMU_PageMapL");
	@(posedge cpu_clk);
	if (~n_pmlsel || pmacc) begin
		$fatal(1, "    --> FAILED n_pmusel=%0d pmacc=%0d",
		    n_pmusel, pmacc);
	end

	@(posedge cpu_clk);
	fc = FC_CONTROL;
	addr = MMU_PageMapU;
	$display("    Selected, FC_CONTROL + MMU_PageMapU");
	@(posedge cpu_clk);
	if (n_pmusel || ~pmacc) begin
		$fatal(1, "    --> FAILED n_pmusel=%0d pmacc=%0d",
		    n_pmusel, pmacc);
	end

	@(posedge cpu_clk);
	fc = FC_CONTROL;
	addr = MMU_PageMapL;
	$display("    Selected, FC_CONTROL + MMU_PageMapL");
	@(posedge cpu_clk);
	if (n_pmlsel || ~pmacc) begin
		$fatal(1, "    --> FAILED n_pmlsel=%0d pmacc=%0d",
		    n_pmlsel, pmacc);
	end

	$display("*** PAGE MAP /WE ***");

	@(posedge cpu_clk);
	fc = FC_SUPER_DATA;
	addr = MMU_PageMapU;
	$display("    Not selected (Upper)");
	@(posedge cpu_clk);
	if (~n_pmu_we || ~n_pml_we) begin
		$fatal(1, "    --> FAILED n_pmu_we=%0d n_pml_we=%0d",
		    n_pmu_we, n_pml_we);
	end

	@(posedge cpu_clk);
	fc = FC_SUPER_DATA;
	addr = MMU_PageMapL;
	$display("    Not selected (Lower)");
	@(posedge cpu_clk);
	if (~n_pmu_we || ~n_pml_we) begin
		$fatal(1, "    --> FAILED n_pmu_we=%0d n_pml_we=%0d",
		    n_pmu_we, n_pml_we);
	end

	@(posedge cpu_clk);
	fc = FC_CONTROL;
	rnw = 1;
	addr = MMU_PageMapU;
	$display("    Selected (Upper), R/W=1");
	@(posedge cpu_clk);
	if (~n_pmu_we || ~n_pml_we) begin
		$fatal(1, "    --> FAILED n_pmu_we=%0d n_pml_we=%0d",
		    n_pmu_we, n_pml_we);
	end

	@(posedge cpu_clk);
	fc = FC_CONTROL;
	rnw = 1;
	addr = MMU_PageMapL;
	$display("    Selected (Lower), R/W=1");
	@(posedge cpu_clk);
	if (~n_pmu_we || ~n_pml_we) begin
		$fatal(1, "    --> FAILED n_pmu_we=%0d n_pml_we=%0d",
		    n_pmu_we, n_pml_we);
	end

	@(posedge cpu_clk);
	fc = FC_CONTROL;
	rnw = 0;
	addr = MMU_PageMapU;
	$display("    Selected (Upper), R/W=0");
	@(posedge cpu_clk);
	if (n_pmu_we || ~n_pml_we) begin
		$fatal(1, "    --> FAILED n_pmu_we=%0d n_pml_we=%0d",
		    n_pmu_we, n_pml_we);
	end

	@(posedge cpu_clk);
	fc = FC_CONTROL;
	rnw = 0;
	addr = MMU_PageMapL;
	$display("    Selected (Lower), R/W=0");
	@(posedge cpu_clk);
	if (~n_pmu_we || n_pml_we) begin
		$fatal(1, "    --> FAILED n_pmu_we=%0d n_pml_we=%0d",
		    n_pmu_we, n_pml_we);
	end

	$display("*** PAGE MAP /UB, /LB ***");

	@(posedge cpu_clk);
	fc = FC_SUPER_DATA;
	addr = MMU_PageMapU;
	$display("    Not selected (Upper)");
	@(posedge cpu_clk);
	if (n_pmu_ub || ~n_pmu_lb) begin
		$fatal(1, "    --> FAILED n_pmu_ub=%0d n_pmu_lb=%0d",
		    n_pmu_ub, n_pmu_lb);
	end

	@(posedge cpu_clk);
	fc = FC_CONTROL;
	addr = MMU_PageMapU;
	$display("    Selected (Upper), no data strobes");
	@(posedge cpu_clk);
	if (~n_pmu_ub || ~n_pmu_lb) begin
		$fatal(1, "    --> FAILED n_pmu_ub=%0d n_pmu_lb=%0d",
		    n_pmu_ub, n_pmu_lb);
	end

	@(posedge cpu_clk);
	fc = FC_CONTROL;
	addr = MMU_PageMapU;
	n_uds = 0;
	n_lds = 1;
	$display("    Selected (Upper), /UDS");
	@(posedge cpu_clk);
	if (n_pmu_ub || ~n_pmu_lb) begin
		$fatal(1, "    --> FAILED n_pmu_ub=%0d n_pmu_lb=%0d",
		    n_pmu_ub, n_pmu_lb);
	end

	@(posedge cpu_clk);
	fc = FC_CONTROL;
	addr = MMU_PageMapU;
	n_uds = 1;
	n_lds = 0;
	$display("    Selected (Upper), /LDS");
	@(posedge cpu_clk);
	if (~n_pmu_ub || n_pmu_lb) begin
		$fatal(1, "    --> FAILED n_pmu_ub=%0d n_pmu_lb=%0d",
		    n_pmu_ub, n_pmu_lb);
	end

	@(posedge cpu_clk);
	fc = FC_SUPER_DATA;
	addr = MMU_PageMapL;
	n_lds = 1;
	$display("    Not selected (Lower)");
	@(posedge cpu_clk);
	if (n_pml_ub || n_pml_lb) begin
		$fatal(1, "    --> FAILED n_pml_ub=%0d n_pml_lb=%0d",
		    n_pml_ub, n_pml_lb);
	end

	@(posedge cpu_clk);
	fc = FC_CONTROL;
	addr = MMU_PageMapL;
	$display("    Selected (Lower), no data strobes");
	@(posedge cpu_clk);
	if (~n_pml_ub || ~n_pml_lb) begin
		$fatal(1, "    --> FAILED n_pml_ub=%0d n_pml_lb=%0d",
		    n_pml_ub, n_pml_lb);
	end

	@(posedge cpu_clk);
	fc = FC_CONTROL;
	addr = MMU_PageMapL;
	n_uds = 0;
	n_lds = 1;
	$display("    Selected (Lower), /UDS");
	@(posedge cpu_clk);
	if (n_pml_ub || ~n_pml_lb) begin
		$fatal(1, "    --> FAILED n_pml_ub=%0d n_pml_lb=%0d",
		    n_pml_ub, n_pml_lb);
	end

	@(posedge cpu_clk);
	fc = FC_CONTROL;
	addr = MMU_PageMapL;
	n_uds = 1;
	n_lds = 0;
	$display("    Selected (Lower), /LDS");
	@(posedge cpu_clk);
	if (~n_pml_ub || n_pml_lb) begin
		$fatal(1, "    --> FAILED n_pml_ub=%0d n_pml_lb=%0d",
		    n_pml_ub, n_pml_lb);
	end

	@(posedge cpu_clk);
	$finish;
end

endmodule
