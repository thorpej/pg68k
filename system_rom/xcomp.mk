XCOMP_CPU?= m68k

HOST_CC:= ${CC}

CC=	${XCOMP_CPU}--netbsdelf-gcc
LD=	${XCOMP_CPU}--netbsdelf-ld
OBJCOPY=${XCOMP_CPU}--netbsdelf-objcopy
