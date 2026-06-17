`timescale 1ns / 1ps

module tb();

`include "ioctl_tb_common.v"

initial begin
	$display("IOCTL TEST: ATA I/O");

	@(posedge cpu_clk);
	$display("Coming out of reset.");
	n_rst = 1;

	@(posedge cpu_clk);
	if (ipl != 0) begin
		$fatal(1, "    --> FAILED ipl=%0d", ipl);
	end

	$display("*** ATA 8-bit write cycle. ***");

	@(posedge cpu_clk);
	$display("S0: CPU drives FC[2:0] and RnW=1");
	fc = FC_SUPER_DATA;
	rnw = 1;

	@(negedge cpu_clk);
	$display("S1: CPU drives A[23:1]");
	addr = OBIO_ATA + 2;

	@(posedge cpu_clk);
	$display("S2: CPU asserts /AS, RnW=0");
	n_as = 0;
	rnw = 0;

	@(negedge cpu_clk);
	$display("S3: CPU places data on data bus");
	data_out = 8'h00;

	@(posedge cpu_clk);
	$display("S4: CPU asserts /xDS, waits for cycle termination signal");
	n_uds = 0;
	n_lds = 0;

	@(negedge cpu_clk);
	$display("S5: no bus signals are altered");

	if (~dtack) begin
		$fatal(1, "    --> FAILED to see DTACK");
	end

	@(posedge cpu_clk);
	$display("S6: no bus signals are altered");

	@(negedge cpu_clk);
	$display("S7: CPU negates /AS, /xDS");
	n_as = 1;
	n_uds = 1;
	n_lds = 1;

	$display("*** ATA 16-bit read cycle. ***");

	@(posedge cpu_clk);
	$display("S0: CPU drives FC[2:0] and RnW=1");
	fc = FC_SUPER_DATA;
	rnw = 1;

	@(negedge cpu_clk);
	$display("S1: CPU drives A[23:1]");
	addr = OBIO_ATA + 4;

	@(posedge cpu_clk);
	$display("S2: CPU asserts /AS, /xDS");
	n_as = 0;
	n_uds = 0;
	n_lds = 0;

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

	@(negedge cpu_clk);
	$display("S7: CPU latches data, negates /AS, /xDS");
	n_as = 1;
	n_uds = 1;
	n_lds = 1;

	$display("*** ATA 8-bit write cycle. ***");

	@(posedge cpu_clk);
	$display("S0: CPU drives FC[2:0] and RnW=1");
	fc = FC_SUPER_DATA;
	rnw = 1;

	@(negedge cpu_clk);
	$display("S1: CPU drives A[23:1]");
	addr = OBIO_ATA + 2;

	@(posedge cpu_clk);
	$display("S2: CPU asserts /AS, RnW=0");
	n_as = 0;
	rnw = 0;

	@(negedge cpu_clk);
	$display("S3: CPU places data on data bus");
	data_out = 8'h00;

	@(posedge cpu_clk);
	$display("S4: CPU asserts /UDS, waits for cycle termination signal");
	n_uds = 0;

	@(posedge cpu_clk);
	$display("S4-WAIT1: CPU waits for cycle termination signal");

	@(posedge cpu_clk);
	$display("S4-WAIT2: CPU waits for cycle termination signal");

	@(negedge cpu_clk);
	$display("S5: no bus signals are altered");

	if (~dtack) begin
		$fatal(1, "    --> FAILED to see DTACK");
	end

	@(posedge cpu_clk);
	$display("S6: no bus signals are altered");

	@(negedge cpu_clk);
	$display("S7: CPU negates /AS, /UDS");
	n_as = 1;
	n_uds = 1;

	$display("*** ATA 8-bit read cycle. ***");

	@(posedge cpu_clk);
	$display("S0: CPU drives FC[2:0] and RnW=1");
	fc = FC_SUPER_DATA;
	rnw = 1;

	@(negedge cpu_clk);
	$display("S1: CPU drives A[23:1]");
	addr = OBIO_ATA + 4;

	@(posedge cpu_clk);
	$display("S2: CPU asserts /AS, /UDS");
	n_as = 0;
	n_uds = 0;

	@(negedge cpu_clk);
	$display("S3: no bus signals are altered");

	@(posedge cpu_clk);
	$display("S4: CPU waits for cycle termination signal");

	@(posedge cpu_clk);
	$display("S4-WAIT1: CPU waits for cycle termination signal");

	@(posedge cpu_clk);
	$display("S4-WAIT2: CPU waits for cycle termination signal");

	if (~dtack) begin
		$fatal(1, "    --> FAILED to see DTACK");
	end

	@(negedge cpu_clk);
	$display("S5: no bus signals are altered");

	@(posedge cpu_clk);
	$display("S6: data from device is driven onto data bus");

	@(negedge cpu_clk);
	$display("S7: CPU latches data, negates /AS, /UDS");
	n_as = 1;
	n_uds = 1;

	@(posedge cpu_clk);
	$finish;
end

endmodule
