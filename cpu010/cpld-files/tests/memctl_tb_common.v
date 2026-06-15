reg n_as;
reg rnw;
reg n_uds;
reg n_lds;

reg mmu_en;

localparam FC_USER_DATA  = 3'd1;
localparam FC_USER_PROG  = 3'd2;
localparam FC_CONTROL    = 3'd4;
localparam FC_SUPER_DATA = 3'd5;
localparam FC_SUPER_PROG = 3'd6;

reg [2:0] fc;

localparam ONE_MB   = 32'h00100000;
localparam ADDR_LIM = 32'h10000000;

localparam RAM0_START = 32'h00000000;
localparam RAM0_END   = 32'h00200000;

localparam RAM1_START = 32'h00200000;
localparam RAM1_END   = 32'h00400000;

localparam RAM2_START = 32'h00400000;
localparam RAM2_END   = 32'h00600000;

localparam RAM3_START = 32'h00600000;
localparam RAM3_END   = 32'h00800000;

localparam EXPRAM_START = 32'h00800000;
localparam EXPRAM_END   = 32'h04000000;

localparam ROM_START = 32'h04000000;
localparam ROM_END   = 32'h08000000;

localparam OBIO_START = 32'h08000000;
localparam OBIO_END   = 32'h0c000000;

localparam VME_START = 32'h0c000000;
localparam VME_END   = 32'h10000000;

reg [31:0] addr;

wire n_u_rd;
wire n_u_wr;
wire n_l_rd;
wire n_l_wr;

localparam ALL_RAM_UNSEL = 4'b1111;
localparam RAM0_ONLYSEL  = 4'b1110;
localparam RAM1_ONLYSEL  = 4'b1101;
localparam RAM2_ONLYSEL  = 4'b1011;
localparam RAM3_ONLYSEL  = 4'b0111;

wire n_romsel;
wire [3:0] n_ramsel;
wire n_expramsel;

wire decode_out;
wire [1:0] space_out;
wire [4:0] bank_out;

wire dtack;

wire want_decode = (fc == FC_USER_DATA ||
		    fc == FC_USER_PROG ||
		    fc == FC_SUPER_DATA ||
		    fc == FC_SUPER_PROG);

	/* Instantiate the device under test. */
	memctl010 dut (
		.nAS(n_as),
		.nUDS(n_uds),
		.nLDS(n_lds),
		.RnW(rnw),

		.MMUEN(mmu_en),

		.FC(fc),
		.ADDR(addr[27:21]),

		.nU_RD(n_u_rd),
		.nU_WR(n_u_wr),
		.nL_RD(n_l_rd),
		.nL_WR(n_l_wr),

		.nROMSEL(n_romsel),
		.nRAMSEL(n_ramsel),
		.nEXPRAMSEL(n_expramsel),

		.decode_out(decode_out),
		.space_out(space_out),
		.bank_out(bank_out),

		.DTACK(dtack)
	);

initial begin
	$dumpfile(`DUMPFILE);
	$dumpvars;

	n_as = 1;
	rnw = 1;
	n_uds = 1;
	n_lds = 1;

	fc = 0;
	addr = 0;

	mmu_en = 0;

	#10;
end
