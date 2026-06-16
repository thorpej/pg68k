`timescale 1ns / 1ps

module tb();

`include "ioctl_tb_common.v"

initial begin
	$display("IOCTL TEST: External interrupt priority encoding logic.");

	@(posedge cpu_clk);
	$display("Coming out of reset.");
	n_rst = 1;

	@(posedge cpu_clk);
	int_en = 1;

	@(posedge cpu_clk);
	n_irq7 = 0;
	n_irq6 = 0;
	n_irq5 = 0;
	n_irq4 = 0;
	n_irq3 = 0;

	@(posedge cpu_clk);
	if (ipl != 7) begin
		$fatal(1,
		    "    --> FAILED ipl=%0d (expected 7)", ipl);
	end

	@(posedge cpu_clk);
	n_irq7 = 1;

	@(posedge cpu_clk);
	if (ipl != 6) begin
		$fatal(1,
		    "    --> FAILED ipl=%0d (expected 6)", ipl);
	end

	@(posedge cpu_clk);
	n_irq6 = 1;

	@(posedge cpu_clk);
	if (ipl != 5) begin
		$fatal(1,
		    "    --> FAILED ipl=%0d (expected 5)", ipl);
	end

	@(posedge cpu_clk);
	n_irq5 = 1;

	@(posedge cpu_clk);
	if (ipl != 4) begin
		$fatal(1,
		    "    --> FAILED ipl=%0d (expected 4)", ipl);
	end

	@(posedge cpu_clk);
	n_irq4 = 1;

	@(posedge cpu_clk);
	if (ipl != 3) begin
		$fatal(1,
		    "    --> FAILED ipl=%0d (expected 3)", ipl);
	end

	@(posedge cpu_clk);
	n_irq3 = 1;

	@(posedge cpu_clk);
	if (ipl != 0) begin
		$fatal(1,
		    "    --> FAILED ipl=%0d (expected 0)", ipl);
	end

	@(posedge cpu_clk);
	$finish;
end

endmodule
