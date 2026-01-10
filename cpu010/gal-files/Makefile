#GALASM=	galasm
GALASM=		galette

GALASM_FLAGS=	-f -p

JEDECS=		ctlspdec.jed obmem.jed topdec.jed vmeglue.jed

.SUFFIXES:
.SUFFIXES: .gal .jed

.gal.jed:
	$(GALASM) $(GALASM_FLAGS) $<

all: $(JEDECS)

clean:
	-rm -f *.fus *.pin
