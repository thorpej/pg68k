`timescale 1ns / 1ps

module tb();

`include "mmu_tb_common.v"

localparam BUS_TIMER_LIMIT = 8'd63;

initial begin
	$display("MMU TEST: MMU bus cycle timer.");

	@(posedge cpu_clk);
	$display("Coming out of reset.");
	n_rst = 1;

	@(posedge cpu_clk);
	$display("T=%0t: Assert /AS", $time);
	n_as = 0;

	repeat (64) @(posedge cpu_clk);
	$display("T=%0t: after 64 positive edges", $time);

	if (n_berr_out) begin
		$fatal(1, "FATAL: MMU failed to signal Bus Error.");
	end
	if (bus_error_reg !== BUS_ERROR_TIMO) begin
		$fatal(1, "FATAL: Bus Error Reg=%b, expected %b",
		    bus_error_reg);
	end
	if (bus_timer_reg !== BUS_TIMER_LIMIT) begin
		$fatal(1, "FATAL: timer=%0d, expected %0d",
		    bus_timer_reg, BUS_TIMER_LIMIT);
	end

	repeat(16) @(posedge cpu_clk);

	if (bus_timer_reg !== BUS_TIMER_LIMIT) begin
		$fatal(1, "FATAL: After extra clocks timer=%0d, expected %0d",
		    bus_timer_reg, BUS_TIMER_LIMIT);
	end

	@(negedge cpu_clk);
	n_as = 1;
	@(posedge cpu_clk);

	/* XXX Why can't we observe this immediately upon latching it? */
	@(posedge mmu_clk);
	if (bus_timer_reg !== 8'd0) begin
		$fatal(1, "FATAL: After negating /AS timer=%0d",
		    bus_timer_reg);
	end

	$finish;
end

endmodule
