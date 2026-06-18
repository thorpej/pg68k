`timescale 1ns / 1ps

module tb();

`include "ioctl_tb_common.v"

/*
 * A 100Hz system timer is traditiona, which is 10000us.
 *
 * The IOCTL timer runs at CPU_CLK / 16, which means that
 * each IOCTL timer tick is 1600ns, or 1.6us.  Thus, we
 * need 6250 (0x186a) ticks to achieve this interval.
 */
localparam MSB = 8'h18;
localparam LSB = 8'h6a;

localparam ENAB = 8'h01;
localparam INTR = 8'h02;

integer saved_counter;
integer i;

initial begin
	$display("IOCTL TEST: Timer");

	@(posedge cpu_clk);
	$display("Coming out of reset.");
	n_rst = 1;

	$display("*** Verifying quiescent state. ***");

	if (timer_enab) begin
		$fatal(1, "    --> FAILED unexpected timer_enab=%0d",
		    timer_enab);
	end

	if (timer_current) begin
		$fatal(1, "    --> FAILED unexpected timer_current=%0d",
		    timer_current);
	end

	if (timer_int) begin
		$fatal(1, "    --> FAILED unexpected timer_int=%0d",
		    timer_int);
	end

	@(posedge cpu_clk);
	$display("Enabling interrupts.");
	int_en = 1;

	$display("*** Programming timer MSB. ***");

	@(posedge cpu_clk);
	$display("S0: CPU drives FC[2:0] and RnW=1");
	fc = FC_SUPER_DATA;
	rnw = 1;

	@(negedge cpu_clk);
	$display("S1: CPU drives A[23:1]");
	addr = OBIO_TMR_VAL;

	@(posedge cpu_clk);
	$display("S2: CPU asserts /AS, RnW=0");
	n_as = 0;
	rnw = 0;

	@(negedge cpu_clk);
	$display("S3: CPU places data on data bus");
	data_out = MSB;

	@(posedge cpu_clk);
	$display("S4: CPU asserts /UDS, waits for cycle termination signal");
	n_uds = 0;

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

	$display("*** Programming timer LSB. ***");

	@(posedge cpu_clk);
	$display("S0: CPU drives FC[2:0] and RnW=1");
	fc = FC_SUPER_DATA;
	rnw = 1;

	@(negedge cpu_clk);
	$display("S1: CPU drives A[23:1]");
	addr = OBIO_TMR_VAL;

	@(posedge cpu_clk);
	$display("S2: CPU asserts /AS, RnW=0");
	n_as = 0;
	rnw = 0;

	@(negedge cpu_clk);
	$display("S3: CPU places data on data bus");
	data_out = LSB;

	@(posedge cpu_clk);
	$display("S4: CPU asserts /UDS, waits for cycle termination signal");
	n_uds = 0;

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

	$display("*** Verifying timer value. ***");

	if (timer_value != {MSB,LSB}) begin
		$fatal(1, "    --> FAILED timer_value=%0d, expected=%0d",
		    timer_value, {MSB,LSB});
	end

	$display("*** Enabling timer. ***");

	@(posedge cpu_clk);
	$display("S0: CPU drives FC[2:0] and RnW=1");
	fc = FC_SUPER_DATA;
	rnw = 1;

	@(negedge cpu_clk);
	$display("S1: CPU drives A[23:1]");
	addr = OBIO_TMR_CSR;

	@(posedge cpu_clk);
	$display("S2: CPU asserts /AS, RnW=0");
	n_as = 0;
	rnw = 0;

	@(negedge cpu_clk);
	$display("S3: CPU places data on data bus");
	data_out = ENAB;

	@(posedge cpu_clk);
	$display("S4: CPU asserts /UDS, waits for cycle termination signal");
	n_uds = 0;

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

	$display("*** Verifying timer is running. ***");

	for (i = 0; i < 64; i++) begin
		@(posedge cpu_clk);
	end
	saved_counter = timer_current;

	for (i = 0; i < 64; i++) begin
		@(posedge cpu_clk);
	end
	if (saved_counter == timer_current) begin
		$fatal(1, "    --> FAILED timer is not running");
	end

	$display("*** Waiting for timer interrupt. ***");

	@(posedge timer_int);

	@(posedge cpu_clk);
	if (ipl != 6) begin
		$fatal(1, "    --> FAILED ipl=%0d", ipl);
	end

	$display("*** Reading status. ***");

	@(posedge cpu_clk);
	$display("S0: CPU drives FC[2:0] and RnW=1");
	fc = FC_SUPER_DATA;
	rnw = 1;

	@(negedge cpu_clk);
	$display("S1: CPU drives A[23:1]");
	addr = OBIO_TMR_CSR;

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

	if (~data[1]) begin
		$fatal(1, "    --> FAILED CSR=%2x", data);
	end

	@(negedge cpu_clk);
	$display("S7: CPU latches data, negates /AS, /UDS");
	n_as = 1;
	n_uds = 1;

	if (ipl != 0) begin
		$fatal(1, "    --> FAILED ipl=%0d", ipl);
	end

	$display("*** Waiting for timer interrupt. ***");

	@(posedge timer_int);

	$display("*** Stalling CPU. ***");

	for (i = 0; i < 40960; i++) begin
		@(posedge cpu_clk);
	end

	$display("*** Reading status. ***");

	@(posedge cpu_clk);
	$display("S0: CPU drives FC[2:0] and RnW=1");
	fc = FC_SUPER_DATA;
	rnw = 1;

	@(negedge cpu_clk);
	$display("S1: CPU drives A[23:1]");
	addr = OBIO_TMR_CSR;

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

	if (~data[1]) begin
		$fatal(1, "    --> FAILED CSR=%2x", data);
	end

	@(negedge cpu_clk);
	$display("S7: CPU latches data, negates /AS, /UDS");
	n_as = 1;
	n_uds = 1;

	if (ipl != 0) begin
		$fatal(1, "    --> FAILED ipl=%0d", ipl);
	end

	$display("*** Waiting for timer interrupt. ***");

	@(posedge timer_int);

	@(posedge cpu_clk);
	if (ipl != 6) begin
		$fatal(1, "    --> FAILED ipl=%0d", ipl);
	end

	$display("*** Programming timer MSB. ***");

	@(posedge cpu_clk);
	$display("S0: CPU drives FC[2:0] and RnW=1");
	fc = FC_SUPER_DATA;
	rnw = 1;

	@(negedge cpu_clk);
	$display("S1: CPU drives A[23:1]");
	addr = OBIO_TMR_VAL;

	@(posedge cpu_clk);
	$display("S2: CPU asserts /AS, RnW=0");
	n_as = 0;
	rnw = 0;

	@(negedge cpu_clk);
	$display("S3: CPU places data on data bus");
	data_out = MSB;

	@(posedge cpu_clk);
	$display("S4: CPU asserts /UDS, waits for cycle termination signal");
	n_uds = 0;

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

	@(posedge cpu_clk);
	@(posedge cpu_clk);
	if (ipl != 0) begin
		$fatal(1, "    --> FAILED ipl=%0d", ipl);
	end

	@(posedge cpu_clk);
	$finish;
end

endmodule
