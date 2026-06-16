`timescale 1ns / 1ps

module tb();

`include "ioctl_tb_common.v"

integer i;

wire n_exp_duartsel = ~((addr >= OBIO_UARTA && addr < OBIO_UARTA_END) ||
			(addr >= OBIO_UARTB && addr < OBIO_UARTB_END));
wire n_exp_i2csel = ~(addr >= OBIO_I2C && addr < OBIO_I2C_END);
wire n_exp_atasel = ~(addr >= OBIO_ATA && addr < OBIO_ATA_END);
wire n_exp_ataauxsel = ~(addr >= OBIO_ATAAUX && addr < OBIO_ATAAUX_END);
wire n_exp_ataben = n_exp_atasel & n_exp_ataauxsel;

initial begin
	$display("IOCTL TEST: On-board I/O decode logic");

	@(posedge cpu_clk);
	$display("Coming out of reset.");
	n_rst = 1;

	@(posedge cpu_clk);
	fc = FC_SUPER_DATA;

	for (i = OBIO_START; i < OBIO_END; i += 2) begin
		@(posedge cpu_clk);
		addr = i;
		@(posedge cpu_clk);
		n_as = 0;
		@(posedge cpu_clk);

		if (n_duartsel != n_exp_duartsel) begin
			$fatal(1,
			    "    --> FAILED addr=%8x n_duartsel=%0d expected=%0d",
			    addr, n_duartsel, n_exp_duartsel);
		end

		if (n_i2csel != n_exp_i2csel) begin
			$fatal(1,
			    "    --> FAILED addr=%8x n_i2csel=%0d expected=%0d",
			    addr, n_i2csel, n_exp_i2csel);
		end

		if (n_atasel != n_exp_atasel) begin
			$fatal(1,
			    "    --> FAILED addr=%8x n_atasel=%0d expected=%0d",
			    addr, n_atasel, n_exp_atasel);
		end

		if (n_ataauxsel != n_exp_ataauxsel) begin
			$fatal(1,
			    "    --> FAILED addr=%8x n_ataauxsel=%0d expected=%0d",
			    addr, n_ataauxsel, n_exp_ataauxsel);
		end

		if (n_ataben != n_exp_ataben) begin
			$fatal(1,
			    "    --> FAILED addr=%8x n_ataben=%0d expected=%0d",
			    addr, n_ataben, n_exp_ataben);
		end
	end

	@(posedge cpu_clk);
	$finish;
end

endmodule
