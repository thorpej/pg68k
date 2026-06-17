reg n_rst;
reg cpu_clk;

reg n_as;
reg rnw;
reg n_uds;
reg n_lds;

reg int_en;

localparam FC_USER_DATA  = 3'd1;
localparam FC_USER_PROG  = 3'd2;
localparam FC_CONTROL    = 3'd4;
localparam FC_SUPER_DATA = 3'd5;
localparam FC_SUPER_PROG = 3'd6;
localparam FC_CPU        = 3'd7;

reg [2:0] fc;
reg [31:0] addr;

localparam OBIO_START    = 32'h08000000;
localparam OBIO_END      = 32'h08001000;

localparam OBIO_UARTB     = 32'h08000000;
localparam OBIO_UARTB_END = 32'h08000010;

localparam OBIO_UARTA     = 32'h08000100;
localparam OBIO_UARTA_END = 32'h08000110;

localparam OBIO_I2C       = 32'h08000300;
localparam OBIO_I2C_END   = 32'h08000310;

localparam OBIO_ATA        = 32'h08000400;
localparam OBIO_ATA_END    = 32'h08000410;

localparam OBIO_ATAAUX     = 32'h08000410;
localparam OBIO_ATAAUX_END = 32'h08000420;

localparam OBIO_TMR_CSR  = 32'h08000200;
localparam OBIO_TMR_LSB  = 32'h08000202;
localparam OBIO_TMR_MSB  = 32'h08000204;

localparam EXP_START     = 32'h0c000000;
localparam EXP_END       = 32'h0d000000;

localparam ADDR_LIM      = 32'h10000000;

localparam CTRL_INT_SET  = 32'h00000040;
localparam CTRL_INT_CLR  = 32'h00000050;
localparam CTRL_BRDREV   = 32'h000000E0;
localparam CTRL_PLDREV   = 32'h000000F0;

reg n_irq7;
reg n_irq6;
reg n_irq5;
reg n_irq4;
reg n_irq3;

reg uarta_int;
reg uartb_int;
reg ata_int;
reg n_i2c_int;

reg ata_iordy;

wire [7:0] data;
reg [7:0] data_out;
assign data = ~rnw ? data_out : 8'bzzzzzzzz;

wire [2:0] n_ipl;
wire [2:0] ipl = ~n_ipl;

wire n_duartsel;
wire n_i2csel;
wire n_atasel;
wire n_ataauxsel;
wire n_ataben;

wire n_iord;
wire n_iowr;

wire iorst;

wire n_expsel;

wire n_avec;
wire dtack;

wire timer_enab;
wire timer_int;
wire [19:0] timer_current;

	/* Instantiate the device under test. */
	ioctl010 dut (
		.nRST(n_rst),
		.CLK(cpu_clk),

		.RnW(rnw),
		.nAS(n_as),
		.nUDS(n_uds),
		.nLDS(n_lds),

		.INT_EN(inten),

		.FC(fc),
		.ADDR(addr[11:1]),
		.ADDRSP(addr[27:26]),
		.CPUTYP(addr[19:16]),

		.nIRQ7(n_irq7),
		.nIRQ6(n_irq6),
		.nIRQ5(n_irq5),
		.nIRQ4(n_irq4),
		.nIRQ3(n_irq3),

		.UARTA_INT(uarta_int),
		.UARTB_INT(uartb_int),
		.ATA_INT(ata_int),
		.nI2C_INT(n_i2cint),

		.ATA_IORDY(ata_iordy),

		.DATA(data),

		.nIPL(n_ipl),

		.nDUARTSEL(n_duartsel),
		.nI2CSEL(n_i2csel),
		.nATASEL(n_atasel),
		.nATAAUXSEL(n_ataauxsel),
		.nATABEN(n_ataben),

		.nIORD(n_iord),
		.nIOWR(n_iowr),

		.IORST(iorst),

		.nEXPSEL(n_expsel),

		.timer_enab_out(timer_enab),
		.timer_current_out(timer_current),
		.timer_int_out(timer_int),

		.nAVEC(n_avec),
		.DTACK(dtack)
	);

initial begin
	$dumpfile(`DUMPFILE);
	$dumpvars;

	n_rst = 0;
	cpu_clk = 0;	/* 1 cycle is rising-edge-to-rising-edge */

	rnw = 1;
	n_as = 1;
	n_uds = 1;
	n_lds = 1;

	int_en = 0;

	fc = 0;
	addr = 0;

	n_irq7 = 1;
	n_irq6 = 1;
	n_irq5 = 1;
	n_irq4 = 1;
	n_irq3 = 1;

	uarta_int = 0;
	uartb_int = 0;
	ata_int = 0;
	n_i2c_int = 1;

	ata_iordy = 1;

	data_out = 0;
end

/* 10MHz CPU clock -> 100ns period */
always #50.000 cpu_clk = ~cpu_clk;
