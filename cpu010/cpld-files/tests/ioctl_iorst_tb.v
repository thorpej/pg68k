`timescale 1ns / 1ps

module tb();

`include "ioctl_tb_common.v"

initial begin
	$display("IOCTL TEST: IORST output");

	@(posedge cpu_clk);
	$display("Staying in reset.");

	if (~iorst) begin
		$fatal(1, "    --> FAILED");
	end

	@(posedge cpu_clk);
	$display("Coming out of reset.");
	n_rst = 1;

	@(posedge cpu_clk);

	if (iorst) begin
		$fatal(1, "    --> FAILED");
	end

	@(posedge cpu_clk);
	$finish;
end

endmodule
