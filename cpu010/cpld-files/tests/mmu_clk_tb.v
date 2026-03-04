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
	$dumpfile("mmu_clk.vcd");
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
	$display("MMU TEST: Basic MMU clock visualization.");

	@(posedge mmu_clk);
	$display("T=%0t: First positive edge detected", $time);

	@(posedge mmu_clk);
	$display("T=%0t: Second positive edge detected", $time);

	@(posedge mmu_clk);
	$display("T=%0t: Third positive edge detected", $time);

	@(posedge mmu_clk);
	$display("T=%0t: Fourth positive edge detected", $time);

	repeat (12) @(posedge mmu_clk);
	$display("T=%0t: after 12 more positive edge detected", $time);

	$finish;
end

endmodule
