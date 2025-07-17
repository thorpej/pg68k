MACH_TYPE=	HOST_SIM
#MACH_PROG=	pgromsim
MACH_IMGS=	pgromsim

.include "../sysrom.mk"

OBJS+=		simtrap.o

pgromsim: $(OBJS)
	${CC} -o ${.TARGET} ${OBJS}
