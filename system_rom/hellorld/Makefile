MACH_TYPE=	PG68010_MK_I
MACH_CPU=	68010
MACH_PROG=	hellorld.elf

XCOMP_CPU=	m68010

.include "../xcomp.mk"

.ifdef MACH_CPU
CPPFLAGS=	-I. -I.. -DCONFIG_MACH_$(MACH_TYPE)
CFLAGS=		-mcpu=$(MACH_CPU) -ffreestanding -Os
AFLAGS=		-mcpu=$(MACH_CPU)
.endif

.PATH:	.. ../compiler_rt

OBJS=	hellorld.o showme.o subr_prf.o
OBJS+=	clzsi2.o ctzsi2.o mulsi3.o \
	umodsi3.o umoddi3.o udivdi3.o udivmoddi4.o udivsi3.o

${MACH_PROG}: ${OBJS}
	${CC} -nostdlib -o ${MACH_PROG} ${OBJS}

clean:
	-rm -f ${MACH_PROG} ${OBJS}
