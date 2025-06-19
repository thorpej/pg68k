RUN_YOSYS=	../atf15xx_yosys/run_yosys.sh

EDIFS=		dramctl.edif sysctl.edif

.SUFFIXES:
.SUFFIXES: .v .edif

.v.edif:
	$(RUN_YOSYS) `echo $< | cut -d '.' -f 1`

all: $(EDIFS)

clean:
	-rm -f *.edif *.io *.pin *.tt3
