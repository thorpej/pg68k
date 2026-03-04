`timescale 1ns / 1ps

module tb();

`include "mmu_tb_common.v"

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
