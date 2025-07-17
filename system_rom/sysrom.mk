#
# Common Makefile infrastructure for the pg68k system ROM.
#

CPPFLAGS=	-I. -I.. -DCONFIG_MACH_$(MACH_TYPE) -DKERNEL_USE
CFLAGS=		-Os
ASFLAGS=

.ifdef MACH_CPU
CFLAGS+=	-mcpu=$(MACH_CPU)
ASFLAGS+=	-mcpu=$(MACH_CPU)
.endif

SYSLIBOBJS=	memcmp.o memcpy.o memset.o subr_prf.o

COMPRTOBJS=	int_util.o udivdi3.o umoddi3.o udivmoddi4.o
M68KOBJS=	start.o setjmp.o trap_stubs.o trap.o ${COMPRTOBJS}
OBJS=		$(SYSLIBOBJS) main.o uart.o console.o

CLEANFILES=	assym.h $(MACH_CLEANFILES)

GENASSYM_FLAGS=${CFLAGS:N-Wa,*:N-fstack-usage*} ${CPPFLAGS}

.PATH: ..
.PATH: ../compiler_rt

all: ${MACH_IMGS}

assym.h: genassym.sh genassym.cf
	cat ../genassym.cf | \
	    sh ../genassym.sh -- ${CC} ${GENASSYM_FLAGS} > assym.h.tmp && \
	mv -f assym.h.tmp assym.h

${M68KOBJS}: assym.h

clean:
	-rm -f $(M68KOBJS) $(OBJS) $(MACH_PROG) $(MACH_IMGS) $(CLEANFILES)
