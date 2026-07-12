MACH_TYPE=	PG68010_MK_I
MACH_CPU=	68010
MACH_PROG=	pg68010mk1.elf
MACH_BIN=	pg68010mk1.bin
MACH_IMGS=	pg68010mk1_even.bin pg68010mk1_odd.bin
MACH_CLEANFILES=${MACH_PROG}.list
DEVICE_TREE=	../../cpu010/dts/pg68k-cpu010.dts

XCOMP_CPU=	m68010

.include "../xcomp.mk"
.include "../sysrom.mk"

OBJS+=		${ATAOBJS} ${DOSFSOBJS} ${UFSOBJS} ${FDTOBJS}
OBJS+=		${I2COBJS}
OBJS+=		${IMGOBJS}
OBJS+=		dd7seg.o
OBJS+=		device-tree.o
CPPFLAGS+=	${FDTCPPFLAGS}

${MACH_PROG}: $(M68KOBJS) $(OBJS)
	${LD} -M -T ldscript -o $(.TARGET) $(M68KOBJS) $(OBJS) \
	    > $(.TARGET).list

${MACH_BIN}: ${MACH_PROG} rompad
	${OBJCOPY} -O binary ${MACH_PROG} ${.TARGET}.tmp
	./rompad 1m ${.TARGET}.tmp && mv ${.TARGET}.tmp ${.TARGET}

${MACH_IMGS}: ${MACH_BIN} romsplit
	./romsplit ${MACH_BIN}
