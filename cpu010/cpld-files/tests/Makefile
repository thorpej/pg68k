#
# NOTE: This requires bmake (BSD make(1) -- preferably the one from NetBSD).
#

MMU_TESTCASES=	mmu_clk

MMU_TESTCASES+=	mmu_read_mmu_off

MMU_TESTCASES+=	mmu_read_super_ok \
		mmu_read_super_prog \
		mmu_read_super_inv \
		mmu_read_super_inv_sme \
		mmu_read_user_ok \
		mmu_read_user_prog \
		mmu_read_user_inv \
		mmu_read_user_priv

MMU_TESTCASES+=	mmu_write_super_ok \
		mmu_write_super_inv \
		mmu_write_super_prot \
		mmu_write_user_ok \
		mmu_write_user_inv \
		mmu_write_user_prot \
		mmu_write_user_priv

MMU_TESTCASES+=	mmu_rmw_super_ok \
		mmu_rmw_super_prot

MMU_TESTCASES+=	mmu_ctx \
		mmu_segmap \
		mmu_pagemap \
		mmu_bustimer \
		mmu_berr_reg

MEMCTL_TESTCASES= memctl_decode \
		  memctl_mmu_off \
		  memctl_romsel \
		  memctl_ramsel0 \
		  memctl_ramsel1 \
		  memctl_ramsel2 \
		  memctl_ramsel3 \
		  memctl_expramsel \
		  memctl_strobes \
		  memctl_dtack

IOCTL_TESTCASES= ioctl_inten \
		 ioctl_extint_encode \
		 ioctl_obioint \
		 ioctl_decode_obio \
		 ioctl_decode_exp \
		 ioctl_iorst \
		 ioctl_iack \
		 ioctl_bpack

TESTCASES=	${MMU_TESTCASES} \
		${MEMCTL_TESTCASES} \
		${IOCTL_TESTCASES}

.for tc in ${TESTCASES}
VCDS+=	${tc}.vcd
.endfor

VFLAGS=	-DBUILD_FOR_TEST -tvvp

.SUFFIXES: .vvp .vcd

.vvp.vcd:
	vvp $<

all: $(VCDS)

clean:
	-rm -f *.vvp *.vcd

.for tc in ${TESTCASES}
${tc}.vcd: ${tc}.vvp
.endfor

.for tc in ${MMU_TESTCASES}
${tc}.vvp: ${tc}_tb.v ../mmu010.v
	iverilog -D'DUMPFILE="${tc}.vcd"' $(VFLAGS) -o $@ ${.ALLSRC}
.endfor

.for tc in ${MEMCTL_TESTCASES}
${tc}.vvp: ${tc}_tb.v ../memctl010.v
	iverilog -D'DUMPFILE="${tc}.vcd"' $(VFLAGS) -o $@ ${.ALLSRC}
.endfor

.for tc in ${IOCTL_TESTCASES}
${tc}.vvp: ${tc}_tb.v ../ioctl010.v
	iverilog -D'DUMPFILE="${tc}.vcd"' $(VFLAGS) -o $@ ${.ALLSRC}
.endfor
