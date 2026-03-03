VCDS=	mmu_clk.vcd \
	mmu_read_mmu_off.vcd \
	mmu_read_mmu_super_ok.vcd

.SUFFIXES: .vvp .vcd

.vvp.vcd:
	vvp $<

all: $(VCDS)

clean:
	-rm -f *.vvp *.vcd

mmu_clk.vcd: mmu_clk.vvp
mmu_clk.vvp: mmu_clk_tb.v ../mmu010.v
	iverilog -tvvp -o $@ mmu_clk_tb.v ../mmu010.v

mmu_read_mmu_off.vcd: mmu_read_mmu_off.vvp
mmu_read_mmu_off.vvp: mmu_read_mmu_off_tb.v ../mmu010.v
	iverilog -tvvp -o $@ mmu_read_mmu_off_tb.v ../mmu010.v

mmu_read_mmu_super_ok.vcd: mmu_read_mmu_super_ok.vvp
mmu_read_mmu_super_ok.vvp: mmu_read_mmu_super_ok_tb.v ../mmu010.v
	iverilog -tvvp -o $@ mmu_read_mmu_super_ok_tb.v ../mmu010.v
