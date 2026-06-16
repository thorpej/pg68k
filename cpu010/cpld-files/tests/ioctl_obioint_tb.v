`timescale 1ns / 1ps

module tb();

`include "ioctl_tb_common.v"

initial begin
	$display("IOCTL TEST: On-board I/O interrupts.");

	@(posedge cpu_clk);
	$display("Coming out of reset.");
	n_rst = 1;

	@(posedge cpu_clk);
	int_en = 1;

	$display("*** UARTA ***");

	@(posedge cpu_clk);
	uarta_int = 1;

	@(posedge cpu_clk);
	if (ipl != 5) begin
		$fatal(1,
		    "    --> FAILED ipl=%0d (expected 5)", ipl);
	end

	@(posedge cpu_clk);
	uarta_int = 0;

	$display("*** UARTB ***");

	@(posedge cpu_clk);
	uartb_int = 1;

	@(posedge cpu_clk);
	if (ipl != 5) begin
		$fatal(1,
		    "    --> FAILED ipl=%0d (expected 5)", ipl);
	end

	@(posedge cpu_clk);
	uartb_int = 0;

	$display("*** ATA ***");

	@(posedge cpu_clk);
	ata_int = 1;

	@(posedge cpu_clk);
	if (ipl != 3) begin
		$fatal(1,
		    "    --> FAILED ipl=%0d (expected 3)", ipl);
	end

	@(posedge cpu_clk);
	ata_int = 0;

	$display("*** I2C ***");

	@(posedge cpu_clk);
	n_i2c_int = 0;

	@(posedge cpu_clk);
	if (ipl != 3) begin
		$fatal(1,
		    "    --> FAILED ipl=%0d (expected 3)", ipl);
	end

	@(posedge cpu_clk);
	n_i2c_int = 1;

	@(posedge cpu_clk);
	$finish;
end

endmodule
