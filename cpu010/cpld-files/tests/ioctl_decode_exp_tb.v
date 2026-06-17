`timescale 1ns / 1ps

module tb();

`include "ioctl_tb_common.v"

integer i;

wire n_exp_expsel = ~(addr >= EXP_START && addr < EXP_END);

initial begin
	$display("IOCTL TEST: External I/O decode logic");

	@(posedge cpu_clk);
	$display("Coming out of reset.");
	n_rst = 1;

	@(posedge cpu_clk);
	fc = FC_SUPER_DATA;

	for (i = OBIO_START; i < EXP_END; i += 4096) begin
		@(posedge cpu_clk);
		addr = i;
		@(posedge cpu_clk);
		n_as = 0;
		@(posedge cpu_clk);

		if (n_expsel != n_exp_expsel) begin
			$fatal(1,
			    "    --> FAILED addr=%8x n_expsel=%0d expected=%0d",
			    addr, n_expsel, n_exp_expsel);
		end

		if (i == OBIO_START) begin
			i = EXP_START - 4096;
		end
	end

	@(posedge cpu_clk);
	$finish;
end

endmodule
