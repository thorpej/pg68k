`timescale 1ns / 1ps

module tb();

`include "ioctl_tb_common.v"

initial begin
	$display("IOCTL TEST: Revision Registers");

	@(posedge cpu_clk);
	$display("Coming out of reset.");
	n_rst = 1;

	$display("*** Reading BRDREV. ***");

	@(posedge cpu_clk);
	$display("S0: CPU drives FC[2:0] and RnW=1");
	fc = FC_CONTROL;
	rnw = 1;

	@(negedge cpu_clk);
	$display("S1: CPU drives A[23:1]");
	addr = CTRL_BRDREV;

	@(posedge cpu_clk);
	$display("S2: CPU asserts /AS, /UDS");
	n_as = 0;
	n_uds = 0;

	@(negedge cpu_clk);
	$display("S3: no bus signals are altered");

	@(posedge cpu_clk);
	$display("S4: CPU waits for cycle termination signal");

	if (~dtack) begin
		$fatal(1, "    --> FAILED to see DTACK");
	end

	@(negedge cpu_clk);
	$display("S5: no bus signals are altered");

	@(posedge cpu_clk);
	$display("S6: data from device is driven onto data bus");

	if (data != 8'h41) begin
		$fatal(1, "    --> FAILED data=%2x", data);
	end

	@(negedge cpu_clk);
	$display("S7: CPU latches data, negates /AS, /UDS");
	n_as = 1;
	n_uds = 1;

	$display("*** Reading PLDREV. ***");

	@(posedge cpu_clk);
	$display("S0: CPU drives FC[2:0] and RnW=1");
	fc = FC_CONTROL;
	rnw = 1;

	@(negedge cpu_clk);
	$display("S1: CPU drives A[23:1]");
	addr = CTRL_PLDREV;

	@(posedge cpu_clk);
	$display("S2: CPU asserts /AS, /UDS");
	n_as = 0;
	n_uds = 0;

	@(negedge cpu_clk);
	$display("S3: no bus signals are altered");

	@(posedge cpu_clk);
	$display("S4: CPU waits for cycle termination signal");

	if (~dtack) begin
		$fatal(1, "    --> FAILED to see DTACK");
	end

	@(negedge cpu_clk);
	$display("S5: no bus signals are altered");

	@(posedge cpu_clk);
	$display("S6: data from device is driven onto data bus");

	if (data != 8'd0) begin
		$fatal(1, "    --> FAILED data=%2x", data);
	end

	@(negedge cpu_clk);
	$display("S7: CPU latches data, negates /AS, /UDS");
	n_as = 1;
	n_uds = 1;

	@(posedge cpu_clk);
	$finish;
end

endmodule
