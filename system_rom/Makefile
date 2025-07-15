CC=	m68k--netbsdelf-gcc
LD=	m68k--netbsdelf-ld

CPPFLAGS=-DCONFIG_MACH_PG68030_MK_I

SYSLIBOBJS= memcmp.o memcpy.o memset.o

OBJS=	$(SYSLIBOBJS) start.o main.o uart.o console.o
BINS=	pg68030mk1.bin

all: $(BINS)

pg68030mk1.bin: $(OBJS)

clean:
	-rm -f $(OBJS) $(BINS)
