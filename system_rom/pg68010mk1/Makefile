MACH_TYPE=	PG68010_MK_I
MACH_CPU=	68010
MACH_PROG=	pg68010mk1.elf
MACH_IMGS=	pg68010mk1.bin
MACH_CLEANFILES=${MACH_PROG}.list

XCOMP_CPU=	m68010

.include "../xcomp.mk"
.include "../sysrom.mk"

OBJS+=		${ATAOBJS} ${DOSFSOBJS} ${UFSOBJS}

${MACH_PROG}: $(M68KOBJS) $(OBJS)
	${LD} -M -T ldscript -o $(.TARGET) $(M68KOBJS) $(OBJS) \
	    > $(.TARGET).list

${MACH_IMGS}: ${MACH_PROG}
	${OBJCOPY} -O binary ${.ALLSRC} ${.TARGET}
