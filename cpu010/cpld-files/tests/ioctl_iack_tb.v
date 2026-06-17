`timescale 1ns / 1ps

module tb();

`include "ioctl_tb_common.v"

initial begin
	$display("IOCTL TEST: IACK cycle");

	/*
	 * We don't need to be fully cycle-accurate here.
	 */

	@(posedge cpu_clk);
	$display("Coming out of reset.");
	n_rst = 1;

	/* some arbitrary previous state */
	rnw = 0;
	fc = FC_USER_DATA;
	addr = 0;

	@(posedge cpu_clk);
	$display("S0: CPU RnW=1");
	rnw = 1;

	@(negedge cpu_clk);
	$display("S1: no bus signals are altered");

	@(posedge cpu_clk);
	$display("S2: no bus signals are altered");

	@(negedge cpu_clk);
	$display("S3: CPU drives FC[2:0]");
	fc = FC_CPU;

	@(posedge cpu_clk);
	$display("S4: no bus signals are altered");

	@(negedge cpu_clk);
	$display("S5: CPU drives A[23:1]");
	addr = 32'hffffffff;

	@(posedge cpu_clk);
	$display("S6: CPU asserts /AS, /UDS, /LDS");
	n_as = 0;
	n_uds = 0;
	n_lds = 0;

	@(negedge cpu_clk);
	$display("S7: no bus signals are altered");

	@(posedge cpu_clk);
	$display("S0: no bus signals are altered");

	@(negedge cpu_clk);
	$display("S1: /DTACK or /VPA sampled");

	if (n_avec) begin
		$fatal(1, "    --> FAILED");
	end

	@(posedge cpu_clk);
	$display("S2: no bus signals are altered");

	@(negedge cpu_clk);
	$display("S3: CPU negates /AS, /UDS, /LDS");
	n_as = 1;
	n_uds = 1;
	n_lds = 1;

	@(posedge cpu_clk);
	$finish;
end

endmodule
