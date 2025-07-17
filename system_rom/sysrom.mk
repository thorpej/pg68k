#
# Common Makefile infrastructure for the pg68k system ROM.
#

CPPFLAGS=	-I.. -DCONFIG_MACH_$(MACH_TYPE)
CFLAGS=		-Os
ASFLAGS=

.ifdef MACH_CPU
CFLAGS+=	-mcpu=$(MACH_CPU)
ASFLAGS+=	-mcpu=$(MACH_CPU)
.endif

.PATH: ..

SYSLIBOBJS=	memcmp.o memcpy.o memset.o subr_prf.o

M68KOBJS=	start.o setjmp.o trap_stubs.o trap.o
OBJS=		$(SYSLIBOBJS) main.o uart.o console.o

all: $(MACH_IMGS)

clean:
	-rm -f $(M68KOBJS) $(OBJS) $(MACH_PROG) $(MACH_IMGS) $(MACH_CLEANFILES)
