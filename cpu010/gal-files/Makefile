#GALASM=	galasm
GALASM=		galette

GALASM_FLAGS=	-f -p

JEDECS=		ctlspdec.jed

.SUFFIXES:
.SUFFIXES: .gal .jed

.gal.jed:
	$(GALASM) $(GALASM_FLAGS) $<

all: $(JEDECS)

clean:
	-rm -f *.fus *.pin
