MACH_TYPE=	PG68010_MK_I
MACH_CPU=	68010
MACH_PROG=	hellorld.elf

XCOMP_CPU=	m68010

.include "../xcomp.mk"

.ifdef MACH_CPU
CPPFLAGS=	-I. -I.. -DCONFIG_MACH_$(MACH_TYPE)
CFLAGS=		-mcpu=$(MACH_CPU) -ffreestanding
ASFLAGS=	-mcpu=$(MACH_CPU)
.endif

${MACH_PROG}: hellorld.S
	${CC} ${CPPFLAGS} ${CFLAGS} -nostdlib -o ${MACH_PROG} hellorld.S

clean:
	-rm -f ${MACH_PROG}
