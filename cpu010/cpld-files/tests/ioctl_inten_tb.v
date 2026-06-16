`timescale 1ns / 1ps

module tb();

`include "ioctl_tb_common.v"

initial begin
	$display("IOCTL TEST: INT_EN logic.");

	@(posedge cpu_clk);
	$display("Coming out of reset.");
	n_rst = 1;

	$display("*** INT_EN negated ***");

	@(posedge cpu_clk);
	n_irq7 = 0;

	@(posedge cpu_clk);
	if (ipl != 0) begin
		$fatal(1,
		    "    --> FAILED ipl=%0d", ipl);
	end

	@(posedge cpu_clk);
	n_irq7 = 1;

	$display("*** INT_EN asserted ***");

	@(posedge cpu_clk);
	int_en = 1;

	@(posedge cpu_clk);
	n_irq7 = 0;

	@(posedge cpu_clk);
	if (ipl != 7) begin
		$fatal(1,
		    "    --> FAILED ipl=%0d", ipl);
	end

	@(posedge cpu_clk);
	$finish;
end

endmodule
