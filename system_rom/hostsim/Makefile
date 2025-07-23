MACH_TYPE=	HOST_SIM
#MACH_PROG=	pgromsim
MACH_IMGS=	pgromsim

.include "../sysrom.mk"

CFLAGS+=	-g -Wall -Wmissing-prototypes

OBJS+=		${ATAOBJS} ${DOSFSOBJS} ${UFSOBJS}
OBJS+=		simglue.o

pgromsim: $(OBJS)
	${CC} -o ${.TARGET} ${OBJS}
