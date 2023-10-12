.PHONY:	all sim
all:
	@make -C bench all
	@make -C rtl all
	@make -C rtl/fifo all
	@make -C synth all

sim:
	@make -C bench sim
	@make -C rtl sim
	@make -C rtl/fifo sim
	@make -C synth sim
	@make -C build sim
