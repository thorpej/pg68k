MACH_TYPE=	HOST_SIM
#MACH_PROG=	pgromsim
MACH_IMGS=	pgromsim
DEVICE_TREE=	hosted-sim.dts

.include "../sysrom.mk"

CFLAGS+=	-g -Wall -Wmissing-prototypes

OBJS+=		${ATAOBJS} ${DOSFSOBJS} ${UFSOBJS} ${FDTOBJS} ${NETOBJS}
OBJS+=		simglue.o
CPPFLAGS+=	${FDTCPPFLAGS}

pgromsim: $(OBJS) device-tree.dtb
	${CC} -o ${.TARGET} ${OBJS}
