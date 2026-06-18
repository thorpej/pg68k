RUN_YOSYS=	../../atf15xx_yosys/run_yosys.sh

EDIFS=		ioctl010.edif memctl010.edif mmu010.edif
VCDS=

.SUFFIXES:
.SUFFIXES: .v .edif
.SUFFIXES: .vvp .vcd

.v.edif:
	$(RUN_YOSYS) `echo $< | cut -d '.' -f 1`

.vvp.vcd:
	vvp $<

all: $(EDIFS) $(VCDS)

program:
	openocd -f cpu010-jtag.cfg \
	    -c "svf cpu010-pld.svf" \
	    -c "exit"

clean:
	-rm -f *.edif *.io *.pin *.tt3 *.vvp *.vcd

spotless: clean
	-rm -f *.fit *.jed
