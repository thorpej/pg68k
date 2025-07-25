#
# Common Makefile infrastructure for the pg68k system ROM.
#

CPPFLAGS=	-I. -I.. -DCONFIG_MACH_$(MACH_TYPE) -DKERNEL_USE
CFLAGS=		-Os -Werror
ASFLAGS=

.ifdef MACH_CPU
CFLAGS+=	-mcpu=$(MACH_CPU)
ASFLAGS+=	-mcpu=$(MACH_CPU)
.endif

.PATH: ..

SYSLIBOBJS=	crc32.o \
		memchr.o memcmp.o memcpy.o memmove.o memset.o \
		subr_prf.o \
		strchr.o strrchr.o strcmp.o strncmp.o strlen.o strnlen.o \
		    strcpy.o strdup.o \
		strtoul.o \
		unicode.o uuid.o

.PATH: ../libfdt
FDTCPPFLAGS=	-I../libfdt
FDTOBJS=	fdt_addresses.o fdt_empty_tree.o fdt_overlay.o fdt_ro.o \
		fdt_rw.o fdt_strerror.o fdt_sw.o fdt_wip.o fdt.o

ATAOBJS=	ata.o

.PATH: ../ufs
UFSOBJS=	ufs.o ffs_bswap.o

.PATH: ../dosfs
DOSFSOBJS=	dosfs.o

.PATH: ../compiler_rt
COMPRTOBJS=	int_util.o ashldi3.o ashrdi3.o divdi3.o udivdi3.o umoddi3.o \
		udivmoddi4.o

M68KOBJS=	start.o setjmp.o trap_stubs.o trap.o malloc.o ${COMPRTOBJS}

OBJS=		$(SYSLIBOBJS) main.o uart.o console.o dev.o disklabel.o \
		files.o fs.o ls.o fnmatch.o loadfile.o loadfile_elf32.o

CLEANFILES=	assym.h $(MACH_CLEANFILES)

GENASSYM_FLAGS=${CFLAGS:N-Wa,*:N-fstack-usage*} ${CPPFLAGS}

all: ${MACH_IMGS}

assym.h: genassym.sh genassym.cf
	cat ../genassym.cf | \
	    sh ../genassym.sh -- ${CC} ${GENASSYM_FLAGS} > assym.h.tmp && \
	mv -f assym.h.tmp assym.h

${M68KOBJS}: assym.h

clean:
	-rm -f $(M68KOBJS) $(OBJS) $(MACH_PROG) $(MACH_IMGS) $(CLEANFILES)
