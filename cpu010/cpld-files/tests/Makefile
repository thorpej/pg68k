#
# NOTE: This requires bmake (BSD make(1) -- preferably the one from NetBSD).
#

MMU_TESTCASES=	mmu_clk

MMU_TESTCASES+=	mmu_read_mmu_off

MMU_TESTCASES+=	mmu_read_super_ok \
		mmu_read_super_inv \
		mmu_read_user_ok \
		mmu_read_user_inv

TESTCASES=	${MMU_TESTCASES}

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
	iverilog $(VFLAGS) -o $@ ${.ALLSRC}
.endfor
