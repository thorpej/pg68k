CC=	m68k--netbsdelf-gcc
LD=	m68k--netbsdelf-ld

.PATH: ..

CPPFLAGS=-I.. -DCONFIG_MACH_$(MACH_TYPE)
CFLAGS=-Os -mcpu=$(MACH_CPU)
ASFLAGS=-mcpu=$(MACH_CPU)

SYSLIBOBJS= memcmp.o memcpy.o memset.o subr_prf.o

OBJS=	$(SYSLIBOBJS) start.o main.o uart.o console.o
IMGS=	pg68030mk1.bin

all: $(MACH_IMGS)

clean:
	-rm -f $(OBJS) $(MACH_IMGS) $(MACH_CLEANFILES)
