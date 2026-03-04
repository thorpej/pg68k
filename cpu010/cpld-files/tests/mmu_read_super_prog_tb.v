`timescale 1ns / 1ps

module tb();

reg n_rst;
reg mmu_clk;

reg n_as;
reg rnw;
reg n_uds;
reg n_lds;
reg n_dtack;

localparam FC_USER_DATA  = 3'd1;
localparam FC_USER_PROG  = 3'd2;
localparam FC_CONTROL    = 3'd4;
localparam FC_SUPER_DATA = 3'd5;
localparam FC_SUPER_PROG = 3'd6;

localparam BUS_ERROR_INV  = 8'b00000001;
localparam BUS_ERROR_PROT = 8'b00000010;
localparam BUS_ERROR_PRIV = 8'b00000100;
localparam BUS_ERROR_TIMO = 8'b00010000;
localparam BUS_ERROR_VME  = 8'b00100000;

reg [2:0] fc;
reg [2:0] addr;

wire [7:0] data;
reg [7:0] data_out;

assign data = ~rnw ? data_out : 8'bzzzzzzzz;

reg mmu_en;
reg n_vme_berr;

wire n_berr_out;
wire [7:0] bus_error_reg;

wire [5:0] ctx;

wire pmacc;
wire mmu_addr;

wire n_smsel;
wire n_pmusel;
wire n_pmlsel;

wire n_sm_we;
wire n_sm_ub;
wire n_sm_lb;

wire n_pmu_we;
wire n_pmu_ub;
wire n_pmu_lb;
wire n_pml_we;
wire n_pml_ub;
wire n_pml_lb;

reg sme_v;
wire [7:0] pme;
reg [7:0] pme_from_sram;

assign pme = n_pmu_we ? pme_from_sram : 8'bzzzzzzzz;

	/* Instantiate the device under test. */
	mmu010 dut (
		.nRST(n_rst),
		.CLK40(mmu_clk),

		.nAS(n_as),
		.RnW(rnw),
		.nUDS(n_uds),
		.nLDS(n_lds),
		.nDTACK(n_dtack),

		.FC(fc),

		.ADDR(addr),
		.DATA(data),

		.MMU_EN(mmu_en),
		.n_vme_berr(n_vme_berr),

		.SME_V(sme_v),
		.PME(pme),

		.CTX(ctx),

		.PMACC(pmacc),
		.MMU_ADDR(mmu_addr),

		.nSMSEL(n_smsel),
		.nPMUSEL(n_pmusel),
		.nPMLSEL(n_pmlsel),

		.nSM_WE(n_sm_we),
		.nSM_UB(n_sm_ub),
		.nSM_LB(n_sm_lb),

		.nPMU_WE(n_pmu_we),
		.nPMU_UB(n_pmu_ub),
		.nPMU_LB(n_pmu_lb),
		.nPML_WE(n_pml_we),
		.nPML_UB(n_pml_ub),
		.nPML_LB(n_pml_lb),

		.nPM_LE(n_pm_le),

		.nAS_out(n_as_out),
		.nUDS_out(n_uds_out),
		.nLDS_out(n_lds_out),

		.CPU_CLK(cpu_clk),

		.n_berr_out(n_berr_out),
		.bus_error_reg_test_out(bus_error_reg),

		.MMU_DTACK(mmu_dtack)
	);

initial begin
	$dumpfile("mmu_read_super_prog.vcd");
	$dumpvars;

	n_rst = 0;
	mmu_clk = 0;	/* 1 cycle is riding-edge-to-rising-edge */

	n_as = 1;
	rnw = 1;
	n_uds = 1;
	n_lds = 1;
	n_dtack = 1;

	fc = 0;
	addr = 0;
	data_out = 0;

	mmu_en = 0;
	n_vme_berr = 1;

	sme_v = 0;
	pme_from_sram = 0;
end

/* 40MHz MMU clock -> 25ns period */
always #12.500 mmu_clk = ~mmu_clk;

initial begin
	$display("MMU TEST: Supervisor prog read cycle, valid translation.");

	@(posedge cpu_clk);
	$display("Coming out of reset.");
	n_rst = 1;

	mmu_en = 1;
	sme_v = 1;

	@(posedge cpu_clk);
	$display("S0: CPU drives FC[2:0] and RnW=1");
	fc = FC_SUPER_PROG;
	rnw = 1;

	@(negedge cpu_clk);
	$display("S1: CPU drives A[23:1]");

	@(posedge cpu_clk);
	$display("S2: CPU asserts /AS, /UDS");
	pme_from_sram = 8'b10100000;	/* V+K */
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
	$display("S7: CPU latches data, negates /AS, /UDS");
	n_as = 1;
	n_uds = 1;

	@(posedge cpu_clk);
	n_dtack = 1;

	@(posedge cpu_clk);
	$finish;
end

endmodule
