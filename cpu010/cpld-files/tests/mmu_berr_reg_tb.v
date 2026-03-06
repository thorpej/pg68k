`timescale 1ns / 1ps

module tb();

`include "mmu_tb_common.v"

initial begin
	$display("MMU TEST: Bus Error Register behavior.");

	/*
	 * FIRST: Perform a faulting read cycle.
	 */

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
	pme_from_sram = 8'b00000000;	/* !V */
	n_as = 0;
	n_uds = 0;

	@(negedge cpu_clk);
	$display("S3: no bus signals are altered");

	@(posedge cpu_clk);
	$display("S4: CPU waits for cycle termination signal");

	if (bus_error_reg != BUS_ERROR_INV) begin
		$fatal(1, "FATAL: BUS ERROR REG=%b EXPECTED=%b",
		    bus_error_reg, BUS_ERROR_INV);
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

	/*
	 * SECOND: Read Bus Error register.
	 */
	$display("S0: drives FC[2:0] and RnW=1");
	fc = FC_CONTROL;
	rnw = 1;

	@(negedge cpu_clk);
	$display("S1: CPU drives A[23:1]");
	addr = MMU_BusErrorReg;

	@(posedge cpu_clk);
	$display("S2: CPU asserts /AS, /UDS");
	n_as = 0;
	n_uds = 0;

	@(negedge cpu_clk);
	$display("S3: no bus signals are altered");

	@(posedge cpu_clk);
	$display("S4: CPU waits for cycle termination signal");

	if (~mmu_dtack) begin
		$fatal(1, "FATAL: MMU failed to ACK control cycle.");
	end

	@(negedge cpu_clk);
	$display("S5: no bus signals are altered");

	@(posedge cpu_clk);
	$display("S6: data from device is driven onto data bus");
	if (data !== BUS_ERROR_INV) begin
		$fatal(1, "FATAL: MMU returned BusError=%0d.", data);
	end

	@(negedge cpu_clk);
	$display("S7: CPU latches data, negates /AS, /UDS");
	n_as = 1;
	n_uds = 1;

	/*
	 * THIRD: Read Bus Error register again.
	 */
	$display("S0: drives FC[2:0] and RnW=1");
	fc = FC_CONTROL;
	rnw = 1;

	@(negedge cpu_clk);
	$display("S1: CPU drives A[23:1]");
	addr = MMU_BusErrorReg;

	@(posedge cpu_clk);
	$display("S2: CPU asserts /AS, /UDS");
	n_as = 0;
	n_uds = 0;

	@(negedge cpu_clk);
	$display("S3: no bus signals are altered");

	@(posedge cpu_clk);
	$display("S4: CPU waits for cycle termination signal");

	if (~mmu_dtack) begin
		$fatal(1, "FATAL: MMU failed to ACK control cycle.");
	end

	@(negedge cpu_clk);
	$display("S5: no bus signals are altered");

	@(posedge cpu_clk);
	$display("S6: data from device is driven onto data bus");
	if (data !== 8'd0) begin
		$fatal(1, "FATAL: MMU returned BusError=%0d.", data);
	end

	@(negedge cpu_clk);
	$display("S7: CPU latches data, negates /AS, /UDS");
	n_as = 1;
	n_uds = 1;

	@(posedge cpu_clk);
	$finish;
end

endmodule
