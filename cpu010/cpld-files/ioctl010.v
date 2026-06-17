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
 * - Board and PLD revision registers (located in Control space)
 *	-> These can be used by system firmware to work around hardware
 *	   bugs, enable features found on later board revisions, and
 *	   detect whether or not it's running in an emulator.
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

	input wire ATA_IORDY,	/* ATA IORDY signal (unused for now) */

	inout wire [7:0] DATA,	/* D15..D8 to/from CPU */

	output wire [2:0] nIPL,	/* IPL output to CPU */

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
	casex ({~nIRQ7,IRQ6,IRQ5,~nIRQ4,IRQ3,Intr_swint})
	7'b1xxxxxx:	encoded_ipl = 3'd7;
	7'b01xxxxx:	encoded_ipl = 3'd6;
	7'b001xxxx:	encoded_ipl = 3'd5;
	7'b0001xxx:	encoded_ipl = 3'd4;
	7'b00001xx:	encoded_ipl = 3'd3;
	7'b000001x:	encoded_ipl = 3'd2;
	7'b0000001:	encoded_ipl = 3'd1;
	default:	encoded_ipl = 3'd0;
	endcase
end
assign nIPL = INT_EN ? ~encoded_ipl : 3'd7;

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
 *      10xx xxxx.xxxx xxxx.0100 0001.xxx0 - ATA disk aux regs (8 bytes)
 *
 *           $8000400 - ATA interface (CS1FX-)
 *           $8000410 - ATA aux regs  (CS3FX-)
 *
 * Control space addresses:
 *
 *           xxxx.xxxx xxxx.xxxx 0100.0000 - Interrupt Set (1 byte)
 *           xxxx.xxxx xxxx.xxxx 0101.0000 - Interrupt Clear (1 byte)
 *
 *           xxxx.xxxx xxxx.xxxx 1110.0000 - Board revision ('A' and up)
 *           xxxx.xxxx xxxx.xxxx 1111.0000 - CPLD set revision
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
wire SpaceIO   = ~nAS && (FC[1] ^ FC[0]) && ADDRSP == 2'b10;
wire SpaceEXP  = ~nAS && (FC[1] ^ FC[0]) && ADDRSP == 2'b11;
wire SpaceCtrl = ~nAS && FC == 3'd4 && ADDR[3:1] == 3'b000;
wire SpaceCPU  = ~nAS && FC == 3'd7;

localparam DEVIDX_UART0		= 4'd0;
localparam DEVIDX_UART1		= 4'd1;
localparam DEVIDX_TMR		= 4'd2;
localparam DEVIDX_I2C		= 4'd3;
localparam DEVIDX_ATA		= 4'd4;

localparam CTLIDX_INTSET	= 4'd4;
localparam CTLIDX_INTCLR	= 4'd5;
localparam CTLIDX_BRDREV	= 4'd14;
localparam CTLIDX_PLDREV	= 4'd15;

localparam SEL_dc		= 11'bxxxxxxxxxxx;
localparam SEL_NONE		= 11'b11111111111;
localparam SEL_DUART		= 11'b01111111111;
localparam SEL_TMR_CSR		= 11'b10111111111;
localparam SEL_TMR_LSB		= 11'b11011111111;
localparam SEL_TMR_MSB		= 11'b11101111111;
localparam SEL_I2C		= 11'b11110111111;
localparam SEL_ATA		= 11'b11111011111;	/* CH1Fx */
localparam SEL_ATA_AUX		= 11'b11111101111;	/* CH3Fx */
localparam SEL_INTR_SET		= 11'b11111110111;
localparam SEL_INTR_CLR		= 11'b11111111011;
localparam SEL_BRDREV		= 11'b11111111101;
localparam SEL_PLDREV		= 11'b11111111110;

localparam SEL_IDX_DUART        = 10;
localparam SEL_IDX_TMR_CSR      = 9;
localparam SEL_IDX_TMR_LSB      = 8;
localparam SEL_IDX_TMR_MSB      = 7;
localparam SEL_IDX_I2C          = 6;
localparam SEL_IDX_ATA          = 5;
localparam SEL_IDX_ATA_AUX      = 4;
localparam SEL_IDX_INTR_SET     = 3;
localparam SEL_IDX_INTR_CLR     = 2;
localparam SEL_IDX_BRDREV       = 1;
localparam SEL_IDX_PLDREV       = 0;

reg [10:0] DevSelects;
always @(*) begin
	casex ({SpaceIO, SpaceCtrl, ADDR[11:8], ADDR[7:4], ADDR[3:1]})
	{2'b10, DEVIDX_UART0, 4'd0, 3'bxxx}: DevSelects = SEL_DUART;
	{2'b10, DEVIDX_UART1, 4'd0, 3'bxxx}: DevSelects = SEL_DUART;
	{2'b10, DEVIDX_TMR,   4'd0, 3'd0}:   DevSelects = SEL_TMR_CSR;
	{2'b10, DEVIDX_TMR,   4'd0, 3'd1}:   DevSelects = SEL_TMR_LSB;
	{2'b10, DEVIDX_TMR,   4'd0, 3'd2}:   DevSelects = SEL_TMR_MSB;
	{2'b10, DEVIDX_I2C,   4'd0, 3'bxxx}: DevSelects = SEL_I2C;
	{2'b10, DEVIDX_ATA,   4'd0, 3'bxxx}: DevSelects = SEL_ATA;
	{2'b10, DEVIDX_ATA,   4'd1, 3'bxxx}: DevSelects = SEL_ATA_AUX;

	{2'b01, 4'd0, CTLIDX_INTSET, 3'd0}:  DevSelects = SEL_INTR_SET;
	{2'b01, 4'd0, CTLIDX_INTCLR, 3'd0}:  DevSelects = SEL_INTR_CLR;
	{2'b01, 4'd0, CTLIDX_BRDREV, 3'd0}:  DevSelects = SEL_BRDREV;
	{2'b01, 4'd0, CTLIDX_PLDREV, 3'd0}:  DevSelects = SEL_PLDREV;

	default:                             DevSelects = SEL_NONE;
	endcase
end
assign nDUARTSEL  = DevSelects[SEL_IDX_DUART];
assign nI2CSEL    = DevSelects[SEL_IDX_I2C];
assign nATASEL    = DevSelects[SEL_IDX_ATA];
assign nATAAUXSEL = DevSelects[SEL_IDX_ATA_AUX];
assign nATABEN    = nATASEL && nATAAUXSEL;
assign nEXPSEL    = ~SpaceEXP;

wire internal_reg_p = ~DevSelects[SEL_IDX_PLDREV] |
		      ~DevSelects[SEL_IDX_BRDREV] |
		      ~DevSelects[SEL_IDX_INTR_CLR] |
		      ~DevSelects[SEL_IDX_INTR_SET] |
		      ~DevSelects[SEL_IDX_TMR_MSB] |
		      ~DevSelects[SEL_IDX_TMR_LSB] |
		      ~DevSelects[SEL_IDX_TMR_CSR];

/* I/O strobe types. */
localparam IO_STROBE_NONE = 2'b00;
localparam IO_STROBE_RD   = 2'b10;
localparam IO_STROBE_WR   = 2'b01;
wire [1:0] io_strobe_type = {RnW, ~RnW};

/*
 * For devices where we can assert the I/O strobes immediately because
 * they're fast enough / we're slow enough to meet the timing requirements.
 */
wire fast_io_strobe_p = ~DevSelects[SEL_IDX_DUART];
wire [1:0] fast_io_strobe =
    io_strobe_type & {fast_io_strobe_p, fast_io_strobe_p};

reg [1:0] io_strobe;
assign {nIORD, nIOWR} = ~((io_strobe | fast_io_strobe) & {~nDS, ~nDS});

/*
 * This is a **fast** DTACK to avoid wait states introduced by timing in the
 * bus cycle state machine when we know it's possible to do so.
 *
 * This is at least for the internal registers, and may apply to
 * other on-board peripherals, as well.
 */
wire FAST_DTACK = internal_reg_p | fast_io_strobe_p;

wire BPACK = SpaceCPU && (CPUTYP == 4'b0000);
wire IACK  = SpaceCPU && (CPUTYP == 4'b1111);

localparam REV_BOARD = 8'h41;	/* 'A' */
localparam REV_PLDSET = 8'd0;	/* A.0 */

/* Logic for reading the internal registers. */
wire enable_data_out = RnW && internal_reg_p;
reg [7:0] data_out;
always @(*) begin
	case (DevSelects)
	SEL_TMR_CSR:	data_out = {6'b0, Timer_int, Timer_enab};
	SEL_TMR_LSB:	data_out = Timer_value[7:0];
	SEL_TMR_MSB:	data_out = Timer_value[15:8];
	SEL_INTR_SET:	data_out = {6'b0, Intr_swint};
	SEL_INTR_CLR:	data_out = {6'b0, Intr_swint};
	SEL_BRDREV:	data_out = REV_BOARD;
	SEL_PLDREV:	data_out = REV_PLDSET;
	default:	data_out = 8'hFF;
	endcase
end
assign DATA = (enable_data_out & ~nUDS) ? data_out : 8'bzzzzzzzz;

/*
 * BUS CYCLE STATE MACHINE
 */
wire [2:0] Cycle = {(SpaceIO | SpaceCtrl), nDS, RnW};

localparam CYCLE_IOREAD		= 3'b101;
localparam CYCLE_IOWRITE	= 3'b100;
localparam CYCLE_IOEITHER	= 3'b10x;

localparam S_IDLE		= 1'd0;
localparam S_DTACK		= 1'd1;

reg state;
always @(posedge CLK, negedge nRST) begin
	if (~nRST) begin
		io_strobe <= IO_STROBE_NONE;
		state <= S_IDLE;

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
		S_IDLE: begin
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
			 *
			 * Also note that only bus cycles that need to
			 * be processed here are, in fact, processed
			 * here.  Devices that natively participate
			 * in the 68000 bus protocol or FAST_DTACK
			 * devices need not be considered.
			 */
			casex ({Cycle, DevSelects})
			{CYCLE_IOREAD, SEL_TMR_CSR}: begin
				Timer_intack <= Timer_int;
				state <= S_DTACK;
			end

			{CYCLE_IOWRITE, SEL_TMR_CSR}: begin
				Timer_enab <= DATA[0];
				state <= S_DTACK;
			end

			{CYCLE_IOWRITE, SEL_TMR_LSB}: begin
				Timer_valmod <= 1'b1;
				Timer_enab <= 1'b0;
				Timer_value[7:0] <= DATA;
				state <= S_DTACK;
			end

			{CYCLE_IOWRITE, SEL_TMR_MSB}: begin
				Timer_valmod <= 1'b1;
				Timer_enab <= 1'b0;
				Timer_value[15:8] = DATA;
				state <= S_DTACK;
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
				state <= S_DTACK;
			end

			{CYCLE_IOEITHER, SEL_ATA_AUX}: begin
				io_strobe <= io_strobe_type;
				state <= S_DTACK;
			end

			{CYCLE_IOWRITE, SEL_INTR_SET}: begin
				Intr_swint <=
				    Intr_swint | DATA[1:0];
				state <= S_DTACK;
			end

			{CYCLE_IOWRITE, SEL_INTR_CLR}: begin
				Intr_swint <=
				    Intr_swint & ~DATA[1:0];
				state <= S_DTACK;
			end

			/*
			 * We don't explcitly signal any bus
			 * errors from this module.  No response
			 * will cause a bus timeout.
			 */
			default: begin
				state <= S_IDLE;
			end
			endcase
		end

		S_DTACK: begin
			if (nDS) begin
				io_strobe <= IO_STROBE_NONE;
				state <= S_IDLE;
			end
		end
		endcase
	end
end

/*
 * Assign DTACK according to the state machine.  BPACK also asserts DTACK
 * and IACK asserts /AVEC.
 */
assign DTACK = ((state == S_DTACK) | BPACK | FAST_DTACK) & ~nDS;
assign nAVEC = ~IACK | nDS;

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
//PIN: nIPL_0		: 14
//PIN: nIPL_1		: 16
//PIN: nIPL_2		: 17
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
//PIN: ATA_IORDY	: 72
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
