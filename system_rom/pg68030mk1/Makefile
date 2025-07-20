MACH_TYPE=	PG68030_MK_I
MACH_CPU=	68030
MACH_PROG=	pg68030mk1.elf
MACH_IMGS=	pg68030mk1.bin
MACH_CLEANFILES=${MACH_PROG}.list

.include "../xcomp.mk"
.include "../sysrom.mk"

OBJS+=		${DOSFSOBJS}

${MACH_PROG}: $(M68KOBJS) $(OBJS)
	${LD} -M -T ldscript -o $(.TARGET) $(M68KOBJS) $(OBJS) \
	    > $(.TARGET).list

${MACH_IMGS}: ${MACH_PROG}
	${OBJCOPY} -O binary ${.ALLSRC} ${.TARGET}
