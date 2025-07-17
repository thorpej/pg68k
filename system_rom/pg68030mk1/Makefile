MACH_TYPE=	PG68030_MK_I
MACH_CPU=	68030
MACH_PROG=	pg68030mk1.elf
MACH_IMGS=	pg68030mk1.bin
MACH_CLEANFILES=pg68030mk1.elf.list

.include "../xcomp.mk"
.include "../sysrom.mk"

pg68030mk1.elf: $(M68KOBJS) $(OBJS)
	${LD} -o $(.TARGET) $(M68KOBJS) $(OBJS) > $(.TARGET).list

pg68030mk1.bin: pg68030mk1.elf
