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
	input wire [10:0] ADDR,	/* A11..A1 from MMU */

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

`ifdef BUILD_FOR_TEST
	output wire timer_enab_out,
	output wire [15:0] timer_value_out,
	output wire [19:0] timer_current_out,
	output wire timer_int_out,
`endif

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
assign nIPL = ~encoded_ipl;

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

wire [3:0] DevIndex  = ADDR[10:7];
wire [3:0] DevPart   = ADDR[6:3];
wire [2:0] DevOffset = ADDR[2:0];

wire NilDevPart = DevPart == 4'b0;

wire SpaceNormal = FC[1] ^ FC[0];
wire SpaceIO   = ~nAS && SpaceNormal && ADDRSP == 2'b10;
wire SpaceEXP  = ~nAS && SpaceNormal && ADDRSP == 2'b11;
wire SpaceCtrl = ~nAS && FC == 3'd4 && DevOffset == 3'b000;
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

wire ControlSpaceDev = SpaceCtrl & (DevIndex == 4'd0);

wire sel_pldrev  = ControlSpaceDev && (DevPart == CTLIDX_PLDREV);
wire sel_brdrev  = ControlSpaceDev && (DevPart == CTLIDX_BRDREV);
wire sel_intrclr = ControlSpaceDev && (DevPart == CTLIDX_INTCLR);
wire sel_intrset = ControlSpaceDev && (DevPart == CTLIDX_INTSET);

wire SimpleIO    = SpaceIO && NilDevPart;

wire sel_duart   = SimpleIO && (DevIndex == DEVIDX_UART0 ||
			       DevIndex == DEVIDX_UART1);
wire sel_tmr     = SimpleIO && DevIndex == DEVIDX_TMR;
wire sel_i2c     = SimpleIO && DevIndex == DEVIDX_I2C;
wire sel_ata     = SpaceIO  && DevIndex == DEVIDX_ATA;

wire internal_reg_p =
    sel_pldrev | sel_brdrev | sel_intrclr | sel_intrset | sel_tmr;

assign nDUARTSEL  = ~sel_duart;
assign nI2CSEL    = ~sel_i2c;
assign nATASEL    = ~(sel_ata && DevPart == 4'd0);
assign nATAAUXSEL = ~(sel_ata && DevPart == 4'd1);
assign nATABEN    = nATASEL && nATAAUXSEL;
assign nEXPSEL    = ~SpaceEXP;

/* I/O strobe types. */
localparam IO_STROBE_NONE = 2'b00;
localparam IO_STROBE_RD   = 2'b10;
localparam IO_STROBE_WR   = 2'b01;
wire [1:0] io_strobe_type = {RnW, ~RnW};

/*
 * For devices where we can assert the I/O strobes immediately because
 * they're fast enough / we're slow enough to meet the timing requirements.
 *
 * N.B. for ATA PIO-0, we have to meet a 70ns address setup time (from
 * assertion of chip select) before we can assert either I/O strobe, but
 * at 10MHz, we can meet that without any extra delays for writes, but
 * /NOT/ for reads!
 */
wire fast_io_strobe_p = sel_duart | (sel_ata & ~RnW);
wire [1:0] fast_io_strobe =
    io_strobe_type & {fast_io_strobe_p, fast_io_strobe_p};

reg [1:0] io_strobe;
assign {nIORD, nIOWR} = ~((io_strobe | fast_io_strobe) & {~nDS, ~nDS});

/*
 * This is a **fast** DTACK to avoid wait states introduced by timing in the
 * bus cycle state machine when we know it's possible to do so.
 *
 * N.B. We are living on the edge with 16-bit PIO-0 ATA transfers,
 * which have a 165ns strobe pulse width (we hit 150ns, and we're
 * going to live with that for now because this should only really
 * be a problem on ancient drives).  For 8-bit transfers, however,
 * it's 290ns for PIO-0, PIO-1, and PIO-2, so we can only to a
 * FAST_DTACK for ATA if both byte lanes are selected.
 */
wire ata_fast_dtack_p = sel_ata & ~nUDS & ~nLDS;

wire FAST_DTACK = internal_reg_p | sel_duart | ata_fast_dtack_p;

wire BPACK = SpaceCPU && (CPUTYP == 4'b0000);
wire IACK  = SpaceCPU && (CPUTYP == 4'b1111);

localparam REV_BOARD = 8'h41;	/* 'A' */
localparam REV_PLDSET = 8'd0;	/* A.0 */

/* Logic for reading the internal registers. */
wire enable_data_out = RnW && internal_reg_p;
reg [7:0] data_out;
always @(*) begin
	if (sel_tmr) begin
		if (DevOffset == 3'd0) begin
			data_out = {6'b0, Timer_int, Timer_enab};
		end
		else if (DevOffset == 3'd1) begin
			data_out = Timer_value[7:0];
		end
		else if (DevOffset == 3'd2) begin
			data_out = Timer_value[15:8];
		end
		else begin
			data_out = 8'hFF;
		end
	end
	else if (sel_intrset || sel_intrclr) begin
		data_out = {6'b0, Intr_swint};
	end
	else if (sel_brdrev) begin
		data_out = REV_BOARD;
	end
	else if (sel_pldrev) begin
		data_out = REV_PLDSET;
	end
	else begin
		data_out = 8'hFF;
	end
end
assign DATA = (enable_data_out & ~nUDS) ? data_out : 8'bzzzzzzzz;

/*
 * BUS CYCLE STATE MACHINE
 */
wire [1:0] Cycle = {nDS, RnW};
localparam CYCLE_READ		= 3'b01;
localparam CYCLE_WRITE		= 3'b00;
localparam CYCLE_EITHER		= 3'b0x;

wire [3:0] Target = {sel_tmr, sel_intrset, sel_intrclr, sel_ata};
localparam TARG_TIMER		= 4'b1000;
localparam TARG_INTRSET		= 4'b0100;
localparam TARG_INTRCLR		= 4'b0010;
localparam TARG_ATA		= 4'b0001;

reg [1:0] state;
localparam S_IDLE		= 2'd0;
localparam S_ATA_WAIT_1		= 2'd1;
localparam S_DTACK		= 2'd2;

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
			casex ({Cycle, Target})
			{CYCLE_READ, TARG_TIMER}: begin
				if (DevOffset == 3'b0) begin
					Timer_intack <= Timer_int;
				end
			end

			{CYCLE_WRITE, TARG_TIMER}: begin
				if (DevOffset == 3'd0) begin
					Timer_enab <= DATA[0];
				end
				else if (DevOffset == 3'd1) begin
					Timer_value[7:0] <= DATA;
					Timer_valmod <= 1'b1;
					Timer_enab <= 1'b0;
				end
				else if (DevOffset == 3'd2) begin
					Timer_value[15:8] = DATA;
					Timer_valmod <= 1'b1;
					Timer_enab <= 1'b0;
				end
			end

			{CYCLE_WRITE, TARG_INTRSET}: begin
				Intr_swint <= Intr_swint | DATA[1:0];
			end

			{CYCLE_WRITE, TARG_INTRCLR}: begin
				Intr_swint <= Intr_swint & ~DATA[1:0];
			end

			/*
			 * PIO mode 0 timings at 10MHz.
			 *
			 * We can use the fast strobes for writes, but not
			 * for reads.  And for 8-bit transfers, we need to
			 * insert a wait state to meet the 290ns pulse width
			 * for PIO-0.  These 8-bit pulses end up being 350ns
			 * wide, and because this state machine transitions
			 * on the rising edge of CPU_CLK, we end up with
			 * 2 wait states.
			 *
			 * For 16-bit transfers, we end up using FAST_DTACK
			 * to ensure that it's asserted by the end of S4.
			 */
			{CYCLE_EITHER, TARG_ATA}: begin
				io_strobe <= io_strobe_type;
				if (nLDS) begin
					state <= S_ATA_WAIT_1;
				end
				else begin
					state <= S_DTACK;
				end
			end
			endcase
		end

		S_ATA_WAIT_1: begin
			state <= S_DTACK;
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

`ifdef BUILD_FOR_TEST
assign timer_enab_out = Timer_enab;
assign timer_value_out = Timer_value;
assign timer_current_out = Timer_current;
assign timer_int_out = Timer_int;
`endif

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
//PIN: ADDR_0		: 28
//PIN: ADDR_1		: 29
//PIN: ADDR_2		: 30
//PIN: ADDR_3		: 31
//PIN: ADDR_4		: 32
//PIN: ADDR_5		: 33
//PIN: ADDR_6		: 35
//PIN: ADDR_7		: 36
//PIN: ADDR_8		: 37
//PIN: ADDR_9		: 40
//PIN: ADDR_10		: 41
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
