`timescale 1ns / 1ps

module tb();

`include "ioctl_tb_common.v"

initial begin
	$display("IOCTL TEST: BPACK cycle");

	/*
	 * We don't need to be fully cycle-accurate here.
	 */

	@(posedge cpu_clk);
	$display("Coming out of reset.");
	n_rst = 1;

	@(posedge cpu_clk);
	$display("S0: CPU drives FC[2:0] and RnW=1");
	fc = FC_CPU;
	rnw = 1;

	@(negedge cpu_clk);
	$display("S1: CPU drives A[23:1]");
	addr = 0;

	@(posedge cpu_clk);
	$display("S2: CPU asserts /AS, /UDS, /LDS");
	n_as = 0;
	n_uds = 0;
	n_lds = 0;

	@(negedge cpu_clk);
	$display("S3: no bus signals are altered");

	@(posedge cpu_clk);
	$display("S4: CPU waits for cycle termination signal");

	if (~dtack) begin
		$fatal(1, "    --> FAILED");
	end

	@(negedge cpu_clk);
	$display("S5: no bus signals are altered");

	@(posedge cpu_clk);
	$display("S6: negates /AS, /UDS, /LDS");
	n_as = 1;
	n_uds = 1;
	n_lds = 1;

	@(negedge cpu_clk);
	$display("S7: no bus signals are altered");

	@(posedge cpu_clk);
	$finish;
end

endmodule
