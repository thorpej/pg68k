#
# Common Makefile infrastructure for the pg68k system ROM.
#

CPPFLAGS=	-I. -I.. -DCONFIG_MACH_$(MACH_TYPE)
CFLAGS=		-Os
ASFLAGS=

.ifdef MACH_CPU
CFLAGS+=	-mcpu=$(MACH_CPU)
ASFLAGS+=	-mcpu=$(MACH_CPU)
.endif

GENASSYM_FLAGS=${CFLAGS:N-Wa,*:N-fstack-usage*} ${CPPFLAGS}

.PATH: ..

assym.h: genassym.sh genassym.cf
	cat ../genassym.cf | \
	    sh ../genassym.sh -- ${CC} ${GENASSYM_FLAGS} > assym.h.tmp && \
	mv -f assym.h.tmp assym.h

SYSLIBOBJS=	memcmp.o memcpy.o memset.o subr_prf.o

M68KOBJS=	start.o setjmp.o trap_stubs.o trap.o
OBJS=		$(SYSLIBOBJS) main.o uart.o console.o

CLEANFILES=	assym.h $(MACH_CLEANFILES)

${M68KOBJS}:	assym.h

all: $(MACH_IMGS)

clean:
	-rm -f $(M68KOBJS) $(OBJS) $(MACH_PROG) $(MACH_IMGS) $(CLEANFILES)
