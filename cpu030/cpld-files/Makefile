RUN_YOSYS=	../atf15xx_yosys/run_yosys.sh

EDIFS=		dramctl.edif sysctl.edif

.SUFFIXES:
.SUFFIXES: .v .edif
.SUFFIXES: .vvp .vcd

.v.edif:
	$(RUN_YOSYS) `echo $< | cut -d '.' -f 1`

.vvp.vcd:
	vvp $<

all: $(EDIFS)

dramctl_tb.vcd: dramctl_tb.vvp
dramctl_tb.vvp: dramctl_tb.v dramctl.v
	iverilog -tvvp -o $@ dramctl_tb.v dramctl.v

clean:
	-rm -f *.edif *.io *.pin *.tt3 *.vvp *.vcd
