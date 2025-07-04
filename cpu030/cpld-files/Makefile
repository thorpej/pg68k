RUN_YOSYS=	../atf15xx_yosys/run_yosys.sh

EDIFS=		dramctl.edif isactl.edif sysctl.edif
VCDS=		dramctl_tb.vcd isactl_tb.vcd isactl_tmr_tb.vcd

.SUFFIXES:
.SUFFIXES: .v .edif
.SUFFIXES: .vvp .vcd

.v.edif:
	$(RUN_YOSYS) `echo $< | cut -d '.' -f 1`

.vvp.vcd:
	vvp $<

all: $(EDIFS) $(VCDS)

dramctl_tb.vcd: dramctl_tb.vvp
dramctl_tb.vvp: dramctl_tb.v dramctl.v
	iverilog -tvvp -o $@ dramctl_tb.v dramctl.v

isactl_tb.vcd: isactl_tb.vvp
isactl_tb.vvp: isactl_tb.v isactl.v
	iverilog -tvvp -o $@ isactl_tb.v isactl.v

isactl_tmr_tb.vcd: isactl_tmr_tb.vvp
isactl_tmr_tb.vvp: isactl_tmr_tb.v isactl.v
	iverilog -tvvp -o $@ isactl_tmr_tb.v isactl.v

clean:
	-rm -f *.edif *.io *.pin *.tt3 *.vvp *.vcd
