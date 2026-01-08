RUN_YOSYS=	../../atf15xx_yosys/run_yosys.sh

EDIFS=		berr010.edif ioctl010.edif mmu010.edif
VCDS=

.SUFFIXES:
.SUFFIXES: .v .edif
.SUFFIXES: .vvp .vcd

.v.edif:
	$(RUN_YOSYS) `echo $< | cut -d '.' -f 1`

.vvp.vcd:
	vvp $<

all: $(EDIFS) $(VCDS)

clean:
	-rm -f *.edif *.io *.pin *.tt3 *.vvp *.vcd
