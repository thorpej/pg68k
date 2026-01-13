/*     
 * Copyright (c) 2026 Jason R. Thorpe.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *      
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR ``AS IS'' AND ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
 * IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 */

/*
 * Bus timer module for 68000 busses.
 *
 * This is pretty simple: It's a counter that counts CPU clock cycles
 * so long as it's instructed to do so (typically when /AS is asserted,
 * although the consumer of this module is allowed to pick any criteria
 * it desires).
 *
 * Re-refactored from cpu010's Bus Error controller and cpu030's
 * System Controller.
 */

module bus_timer(
	input wire clk,
	input wire n_reset,
	input wire count,

	output wire bus_timeout
);

localparam BUS_TIMER_INITIAL	=	6'd0;
localparam BUS_TIMER_TICK	=	6'd1;
localparam BUS_TIMER_TIMEOUT	=	6'd63;

reg [5:0] bus_timer;
initial begin
	bus_timer = BUS_TIMER_INITIAL;
end

always @(posedge clk, negedge n_reset) begin
	if (~n_reset) begin
		bus_timer <= BUS_TIMER_INITIAL;
	end
	else if (~count) begin
		bus_timer <= BUS_TIMER_INITIAL;
	end
	else if (bus_timer != BUS_TIMER_TIMEOUT) begin
		bus_timer <= bus_timer + BUS_TIMER_TICK;
	end
end

assign bus_timeout = (bus_timer == BUS_TIMER_TIMEOUT);

endmodule
