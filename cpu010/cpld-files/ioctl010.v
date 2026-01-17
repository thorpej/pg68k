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
 * I/O controller for the cpu010.  This is targeted at an Atmel/Microchip
 * ATF1508AS-7AX100 7.5ns CPLD.
 *
 * This provides the following functionality:
 *
 * - Address decoding and bus cycle state machine for various on-board
 *   I/O devices:
 *
 *	-> TL16C2552 DUART.  The UARTs interrupt at IRQ5.
 *	-> System "hardclock" timer. The timer interrupts at IRQ6.
 *	-> PCF8584 I2C controller (decoding only; this device speaks
 *	   the 68000 protocol natively!)
 *	-> ATA disk interface (PIO mode 0 only; it's not like the 68010
 *	   can really go much faster!).  The ATA interface interrupts
 *	   at IRQ3.
 *	-> ?? anything else ??
 *
 * - Interrupt controller (located in Control space)
 *	-> Auto-vector only
 *	-> Interrupts are disabled at reset (in the System Enble Register),
 *	   single global interrupt enable (also masks NMI!)
 *	-> Open-drain wired-OR hardware interrupts for IPL3-IPL7.
 *	-> Software controlled interrupts at IPL1-IPL2 via two separate
 *	   SET and CLR registers.
 *
 * Random thoughts:
 *
 * It probably wouldn't be too difficult to support vectored interrupts
 * for the known on-board peripherals, assigning them each a static
 * vector.  Would have to think about how one would prioritize if two
 * different devices at the same IPL wanted to interrupt concurrently.
 * Example: the UARTs.  At first blush, I would prioritize UARTA ("com1",
 * as wired on the board) over UARTB ("com0", the console) because I
 * would assume it's being used for bulk data transfer (SLIP, perhaps?),
 * but I would want to put some more thought into it.
 *
 * Including a PCF8584 seems a little indulgent.  I could just provide
 * the facilities to bit-bang I2C here in the IOCTL.  Maybe I'll do that
 * in a future IOCTL revision.  Keeping I2C around is important for the
 * DS3231MZ real-time clock chip (and maybe for expansion card discovery
 * via SEEPROM?).
 */

module ioctl010(
	input wire nRST,	/* system /RESET signal */
	input wire CLK,		/* system CLK10 signal */

	input wire RnW,		/* R/W from CPU */
	input wire nAS,		/* /AS from MMU */
	input wire nUDS,	/* /UDS from MMU */
	input wire nLDS,	/* /LDS from MMU */

	input wire INT_EN,	/* Interrupt Enable input */

	input wire [2:0] FC,	/* FC2..FC0 from CPU */
	input wire [11:1] ADDR,	/* A11..A1 from MMU */

	input wire [1:0] ADDRSP, /* A27..A26 from MMU */
	input wire [3:0] CPUTYP, /* VA19-VA16 from CPU */

	/* active-low IRQ inputs */
	input wire nIRQ7,	/* NMI: debug button, etc. */
	input wire nIRQ6,	/* IPL6: timers and the like */
	input wire nIRQ5,	/* IPL5: UARTs and the like */
	input wire nIRQ4,	/* IPL4: network interfaces and the like */
	input wire nIRQ3,	/* IPL3: block devices and the like */

	input wire UARTA_INT,	/* UART A interrupt */
	input wire UARTB_INT,	/* UART B interrupt */
	input wire ATA_INT,	/* ATA disk interface interrupt */
	input wire nI2C_INT,	/* I2C controller interrupt */

	inout wire [7:0] DATA,	/* D15..D8 to/from CPU */

	output wire [2:0] IPL,	/* IPL output to CPU */

	/* On-board I/O device select outputs. */
	output wire nDUARTSEL,
	output wire nI2CSEL,
	output wire nATASEL,
	output wire nATAAUXSEL,
	output wire nATABEN,	/* enable ATA bus transceivers */

	output wire nIORD,	/* I/O /RD strobe */
	output wire nIOWR,	/* I/O /WR strobe */

	output wire IORST,	/* active-high /RESET for I/O devices */

	output wire nEXPSEL,	/* expansion space selected */

	output wire nAVEC,	/* /AVEC connected directly to CPU */
	output wire DTACK	/* drives open-drain inverter */
);

/* Inverted /RESET output for devices that have active-high reset inputs. */
assign IORST = ~nRST;

/* Internal combined /xDS value. */
wire nDS = nUDS & nLDS;

/* DTACK output register. */
reg dtack;
assign DTACK = dtack & ~nDS;

/* nAVEC output register. */
reg avec;
assign nAVEC = ~avec & ~nDS;

/* Interrupt enable and Software interrupt (IRQ1, IRQ2) registers */
reg [1:0] Intr_swint;

/* System "hardclock" timer state. */
reg [15:0] Timer_value;
reg Timer_valmod;
reg Timer_intack;
reg Timer_enab;
reg Timer_int;

/* ATA disk interface and I2C controller interrupt at IRQ3. */
wire IRQ3 = ~nIRQ3 | ATA_INT | ~nI2C_INT;

/* UARTs interrupt at IRQ5. */
wire IRQ5 = ~nIRQ5 | UARTA_INT | UARTB_INT;

/* Timer interrupts at IRQ6. */
wire IRQ6 = ~nIRQ6 | Timer_int;

/* Encode the IPL, IRQ7 highest priority, IRQ1 lowest. */
reg [2:0] encoded_ipl;
always @(*) begin
	casex ({INT_EN,~nIRQ7,IRQ6,IRQ5,~nIRQ4,IRQ3,Intr_swint})
	8'b11xxxxxx:	encoded_ipl = 3'd7;
	8'b101xxxxx:	encoded_ipl = 3'd6;
	8'b1001xxxx:	encoded_ipl = 3'd5;
	8'b10001xxx:	encoded_ipl = 3'd4;
	8'b100001xx:	encoded_ipl = 3'd3;
	8'b1000001x:	encoded_ipl = 3'd2;
	8'b10000001:	encoded_ipl = 3'd1;
	default:	encoded_ipl = 3'd0;
	endcase
end
assign IPL = ~encoded_ipl;

/* I/O strobe types. */
localparam IO_STROBE_NONE = 2'b00;
localparam IO_STROBE_RD   = 2'b10;
localparam IO_STROBE_WR   = 2'b01;
wire [1:0] io_strobe_type = {RnW, ~RnW};
reg [1:0] io_strobe;
assign {nIORD, nIOWR} = ~(io_strobe & {~nDS, ~nDS});

/*
 * Address decoding.
 *
 * I/O space addresses:
 *
 *   I/O space!
 *      ||                          always zero
 *      ||                               |
 *      ||                               |
 *      vv                               v
 *      10xx xxxx.xxxx xxxx.0000 0000.0000
 *                          ^^^^
 *                       device index
 *
 *             connect to DUART CHSEL input (0=UART B)
 *                             v
 *      10xx xxxx.xxxx xxxx.0000 0000.xxx0 - UART B ("com0") - 8 bytes
 *      10xx xxxx.xxxx xxxx.0001 0000.xxx0 - UART A ("com1") - 8 bytes
 *
 *           $8000000 - com0
 *           $8000100 - com1
 *
 *      10xx xxxx.xxxx xxxx.0010 0000.0000 - Timer CSR (1 byte)
 *      10xx xxxx.xxxx xxxx.0010 0000.0010 - Timer LSB (1 byte)
 *      10xx xxxx.xxxx xxxx.0010 0000.0100 - Timer MSB (1 byte)
 *
 *           $8000200 - Timer CSR
 *           $8000202 - Timer LSB
 *           $8000204 - Timer MSB
 *
 *      10xx xxxx.xxxx xxxx.0011 0000.00x0 - PCF8584 (2 bytes)
 *
 *           $8000300 - I2C controller
 *
 *      10xx xxxx.xxxx xxxx.0100 0000.xxx0 - ATA disk interface (8 bytes)
 *      10xx xxxx.xxxx xxxx.0100 0001.00x0 - ATA disk aux regs (2 bytes)
 *
 *           $8000400 - ATA interface
 *           $8000410 - ATA aux regs
 *
 * Control space addresses:
 *
 *           xxxx.xxxx xxxx.xxxx 0100.0000 - Interrupt Set (1 byte)
 *           xxxx.xxxx xxxx.xxxx 0101.0000 - Interrupt Clear (1 byte)
 *
 * CPU space addresses:
 *
 *           xxxx.0000 xxxx.xxxx xxxx.xxxx - BPACK cycle
 *           xxxx.1111 xxxx.xxxx xxxx.xxxx - IACK cycle
 *
 * FC2   FC1   FC0
 *  0     0     0	(Undefined, reserved)
 *  0     0     1	User Data Space
 *  0     1     0	User Program Space
 *  0     1     1	(Undefined, reserved)
 *  1     0     0	(Undefined, reserved) - Control Space
 *  1     0     1	Supervisor Data Space
 *  1     1     0	Supervisor Program Space
 *  1     1     1	CPU Space
 *
 * Normal access (User,Super Prog,Data) -> FC1 ^ FC0 == 1
 */
wire SpaceIO   = (FC[1] ^ FC[0]) && ADDRSP == 2'b10;
wire SpaceEXP  = (FC[1] ^ FC[0]) && ADDRSP == 2'b11;
wire SpaceCtrl = FC == 3'd4 && ADDR[3:1] == 3'b000;
wire SpaceCPU  = FC == 3'd7;

localparam DEVIDX_UART0		= 4'd0;
localparam DEVIDX_UART1		= 4'd1;
localparam DEVIDX_TMR		= 4'd2;
localparam DEVIDX_I2C		= 4'd3;
localparam DEVIDX_ATA		= 4'd4;

localparam CTLIDX_INTSET	= 4'd4;
localparam CTLIDX_INTCLR	= 4'd5;

localparam SEL_dc		= 9'bxxxxxxxxx;
localparam SEL_NONE		= 9'b111111111;
localparam SEL_DUART		= 9'b011111111;
localparam SEL_TMR_CSR		= 9'b101111111;
localparam SEL_TMR_LSB		= 9'b110111111;
localparam SEL_TMR_MSB		= 9'b111011111;
localparam SEL_I2C		= 9'b111101111;
localparam SEL_ATA		= 9'b111110111;	/* CH1Fx */
localparam SEL_ATA_AUX		= 9'b111111011;	/* CH3Fx */
localparam SEL_INTR_SET		= 9'b111111101;
localparam SEL_INTR_CLR		= 9'b111111110;

reg [8:0] DevSelects;
always @(*) begin
	casex ({SpaceIO, SpaceCtrl, ADDR[11:8], ADDR[4:7], ADDR[3:1]})
	{2'b10, DEVIDX_UART0, 4'd0, 3'bxxx}: DevSelects = SEL_DUART;
	{2'b10, DEVIDX_UART1, 4'd0, 3'bxxx}: DevSelects = SEL_DUART;
	{2'b10, DEVIDX_TMR,   4'd0, 3'd0}:   DevSelects = SEL_TMR_CSR;
	{2'b10, DEVIDX_TMR,   4'd0, 3'd1}:   DevSelects = SEL_TMR_LSB;
	{2'b10, DEVIDX_TMR,   4'd0, 3'd2}:   DevSelects = SEL_TMR_MSB;
	{2'b10, DEVIDX_I2C,   4'd0, 3'bxxx}: DevSelects = SEL_I2C;
	{2'b10, DEVIDX_ATA,   4'd0, 3'bxxx}: DevSelects = SEL_ATA;
	{2'b10, DEVIDX_ATA,   4'd1, 3'b00x}: DevSelects = SEL_ATA_AUX;

	{2'b01, 4'd0, CTLIDX_INTSET, 3'd0}:  DevSelects = SEL_INTR_SET;
	{2'b01, 4'd0, CTLIDX_INTCLR, 3'd0}:  DevSelects = SEL_INTR_CLR;

	default:                             DevSelects = SEL_NONE;
	endcase
end
assign nDUARTSEL  = DevSelects[8];
assign nI2CSEL    = DevSelects[4];
assign nATASEL    = DevSelects[3];
assign nATAAUXSEL = DevSelects[2];
assign nATABEN    = nATASEL && nATAAUXSEL;
assign nEXPSEL    = ~SpaceEXP;

wire BPACK = SpaceCPU && (CPUTYP == 4'b0000);
wire IACK  = SpaceCPU && (CPUTYP == 4'b1111);

/* Logic for reading the internal registers. */
reg enable_data_out;
reg [7:0] data_out;
always @(*) begin
	case (DevSelects)
	SEL_TMR_CSR:	data_out = {6'b0, Timer_int, Timer_enab};
	SEL_TMR_LSB:	data_out = Timer_value[7:0];
	SEL_TMR_MSB:	data_out = Timer_value[15:8];
	SEL_INTR_SET:	data_out = {6'b0, Intr_swint};
	SEL_INTR_CLR:	data_out = {6'b0, Intr_swint};
	default:	data_out = 8'hFF;
	endcase
end
assign DATA = (enable_data_out & ~nUDS) ? data_out : 8'bzzzzzzzz;

/*
 * BUS CYCLE STATE MACHINE
 */
wire [2:0] Cycle = {nDS, RnW, BPACK, IACK};

localparam CYCLE_BPACK		= 4'b0x10;
localparam CYCLE_IACK		= 4'b0x01;
localparam CYCLE_IOREAD		= 4'b0100;
localparam CYCLE_IOWRITE	= 4'b0000;
localparam CYCLE_IOEITHER	= 4'b0x00;

localparam Idle			= 1'd0;
localparam TermWait		= 1'd1;

reg state;
always @(posedge CLK, negedge nRST) begin
	if (~nRST) begin
		enable_data_out <= 1'b0;
		io_strobe <= IO_STROBE_NONE;
		dtack <= 1'b0;
		avec <= 1'b0;
		state <= Idle;

		Intr_swint <= 2'b0;

		Timer_value <= 16'd0;
		Timer_valmod <= 1'b0;
		Timer_intack <= 1'b0;
		Timer_enab <= 1'b0;
	end
	else begin
		/*
		 * We run on the CPU's 10MHz clock, so each state
		 * represents 100ns.
		 */
		case (state)
		Idle: begin
			/*
			 * The timer has processed these signals,
			 * so clear them now.
			 */
			Timer_valmod <= 1'b0;
			Timer_intack <= 1'b0;

			/*
			 * If we get any decode hit here, we know that
			 * the chip select for the device has already
			 * been asserted by combinatorial logic.  For
			 * read cycles, /AS and /DS are asserted at the
			 * same time, whereas for write cycles, /DS is
			 * assrted one CPU clock cycle later than /AS.
			 *
			 * This means that the number of slots needed
			 * for address setup times might be different
			 * for reads vs. writes.
			 */
			casex ({Cycle, DevSelects})
			{CYCLE_BPACK, SEL_dc}: begin
				dtack <= 1'b1;
				state <= TermWait;
			end

			{CYCLE_IACK, SEL_dc}: begin
				avec <= 1'b1;
				state <= TermWait;
			end

			/*
			 * DUART timings are for the TL16C2552 at 5V.
			 * That chip that respond faster than we can
			 * drive it, so no wait states are required.
			 */
			{CYCLE_IOEITHER, SEL_DUART}: begin
				io_strobe <= io_strobe_type;
				dtack <= 1'b1;
				state <= TermWait;
			end

			{CYCLE_IOREAD, SEL_TMR_CSR}: begin
				Timer_intack <= Timer_int;
				enable_data_out <= 1'b1;
				dtack <= 1'b1;
				state <= TermWait;
			end

			{CYCLE_IOWRITE, SEL_TMR_CSR}: begin
				Timer_enab <= DATA[0];
				dtack <= 1'b1;
				state <= TermWait;
			end

			{CYCLE_IOREAD, SEL_TMR_LSB}: begin
				enable_data_out <= 1'b1;
				dtack <= 1'b1;
				state <= TermWait;
			end

			{CYCLE_IOWRITE, SEL_TMR_LSB}: begin
				Timer_valmod <= 1'b1;
				Timer_enab <= 1'b0;
				Timer_value[7:0] <= DATA;
				dtack <= 1'b1;
				state <= TermWait;
			end

			{CYCLE_IOREAD, SEL_TMR_LSB}: begin
				enable_data_out <= 1'b1;
				dtack <= 1'b1;
				state <= TermWait;
			end

			{CYCLE_IOWRITE, SEL_TMR_LSB}: begin
				Timer_valmod <= 1'b1;
				Timer_enab <= 1'b0;
				Timer_value[15:8] = DATA;
				dtack <= 1'b1;
				state <= TermWait;
			end

			/*
			 * PCF8584 participates in the 68000 bus
			 * protocol natively.  The chip select is
			 * asserted via combinatorial logic.
			 */
			{CYCLE_IOEITHER, SEL_I2C}: begin
				state <= Idle;
			end

			/*
			 * PIO mode 0 timings at 10MHz.
			 *
			 * XXX We might need to insert a wait
			 * XXX state for 8-bit transfers, but
			 * XXX then again, any drive we plug
			 * XXX in is likely to be fast enough
			 * XXX to work either way.
			 */
			{CYCLE_IOEITHER, SEL_ATA}: begin
				io_strobe <= io_strobe_type;
				dtack <= 1'b1;
				state <= TermWait;
			end

			{CYCLE_IOEITHER, SEL_ATA_AUX}: begin
				io_strobe <= io_strobe_type;
				dtack <= 1'b1;
				state <= TermWait;
			end

			{CYCLE_IOREAD, SEL_INTR_SET}: begin
				enable_data_out <= 1'b1;
				dtack <= 1'b1;
				state <= TermWait;
			end

			{CYCLE_IOWRITE, SEL_INTR_SET}: begin
				Intr_swint <=
				    Intr_swint | DATA[1:0];
				dtack <= 1'b1;
				state <= TermWait;
			end

			{CYCLE_IOREAD, SEL_INTR_CLR}: begin
				enable_data_out <= 1'b1;
				dtack <= 1'b1;
				state <= TermWait;
			end

			{CYCLE_IOWRITE, SEL_INTR_CLR}: begin
				Intr_swint <=
				    Intr_swint & ~DATA[1:0];
				dtack <= 1'b1;
				state <= TermWait;
			end

			/*
			 * We don't explcitly signal any bus
			 * errors from this module.  May revisit
			 * that at some point later.
			 */
			default: begin
				state <= Idle;
			end
			endcase
		end

		TermWait: begin
			if (nDS) begin
				enable_data_out <= 1'b0;
				io_strobe <= IO_STROBE_NONE;
				dtack <= 1'b0;
				avec <= 1'b0;
				state <= Idle;
			end
		end
		endcase
	end
end

/*
 * SYSTEM TIMER IMPLEMENTATION
 */
reg [19:0] Timer_current;
reg Timer_was_enab;

always @(posedge CLK, negedge nRST) begin
	if (~nRST) begin
		Timer_current <= 20'd0;
		Timer_int <= 1'b0;
		Timer_was_enab <= 1'b0;
	end
	else begin
		if (Timer_valmod) begin
			/*
			 * If Timer_valmod is true, we know that
			 * Timer_enab will be false.
			 */
			Timer_current <= {Timer_value, 4'b0000};
			Timer_int <= 1'b0;
		end
		else if (Timer_enab) begin
			if (~Timer_was_enab || Timer_current == 20'd0) begin
				Timer_current <= {Timer_value, 4'b0000};
				Timer_int <= Timer_was_enab;
			end
			else begin
				if (Timer_intack) begin
					Timer_int <= 1'b0;
				end
				Timer_current <= Timer_current - 1;
			end
		end
		else begin
			Timer_int <= 1'b0;
		end
		Timer_was_enab <= Timer_enab;
	end
end

endmodule

// Pin assignment for Yosys workflow.
//
//PIN: CHIP "ioctl010" ASSIGNED TO AN TQFP100
//
//	=== CPU side of the chip ===
//
//PIN: DTACK		: 1
//PIN: IORST		: 2
//PIN: nAS		: 5
//PIN: RnW		: 6
//PIN: nUDS		: 7
//PIN: nLDS		: 8
//PIN: FC_0		: 10
//PIN: FC_1		: 12
//PIN: FC_2		: 13
//PIN: IPL_0		: 14
//PIN: IPL_1		: 16
//PIN: IPL_2		: 17
//PIN: DATA_0		: 19
//PIN: DATA_1		: 20
//PIN: DATA_2		: 21
//PIN: DATA_3		: 22
//PIN: DATA_4		: 23
//PIN: DATA_5		: 24
//PIN: DATA_6		: 25
//PIN: DATA_7		: 27
//PIN: CLK		: 87
//PIN: nRST		: 89
//PIN: nAVEC		: 100
//
//	=== MMU address output side of the chip ===
//
//PIN: ADDR_1		: 28
//PIN: ADDR_2		: 29
//PIN: ADDR_3		: 30
//PIN: ADDR_4		: 31
//PIN: ADDR_5		: 32
//PIN: ADDR_6		: 33
//PIN: ADDR_7		: 35
//PIN: ADDR_8		: 36
//PIN: ADDR_9		: 37
//PIN: ADDR_10		: 40
//PIN: ADDR_11		: 41
//PIN: ADDRSP_0		: 42
//PIN: ADDRSP_1		: 44
//PIN: CPUTYP_0		: 45
//PIN: CPUTYP_1		: 46
//PIN: CPUTYP_2		: 47
//PIN: CPUTYP_3		: 48
//PIN: INT_EN		: 49
//
//	=== Devices side of the chip ===
//
//PIN: nEXPSEL		: 75
//PIN: nI2CINT		: 76
//PIN: nI2CSEL		: 77
//PIN: nATABEN		: 78
//PIN: nATASEL		: 79
//PIN: nATAAUXSEL	: 80
//PIN: ATA_INT		: 81
//PIN: nDUARTSEL	: 83
//PIN: UARTA_INT	: 84
//PIN: UARTB_INT	: 85
//PIN: nIRQ3		: 92
//PIN: nIRQ4		: 93
//PIN: nIRQ5		: 94
//PIN: nIRQ6		: 96
//PIN: nIRQ7		: 97
//PIN: nIORD		: 98
//PIN: nIOWR		: 99
