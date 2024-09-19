.PHONY:	all build sim
all:	sim

#
#  Icarus Verilog settings
##
LIBDIR	?= ../lib/misc-verilog-cores
RTLDIR	:= ../rtl

IVC	?= iverilog
OPT	:= -g2005-sv -D__icarus -Wall -I. -I$(RTLDIR) -I$(LIBDIR)/axis

ARCH_V	:= $(wildcard $(LIBDIR)/arch/*.v) arch/gw2a_prim_sim.v
ARCH	:= $(filter-out %_tb.v, $(ARCH_V))

AXIS_V	:= $(wildcard $(LIBDIR)/axis/*.v)
AXIS	:= $(filter-out %_tb.v, $(AXIS_V))

FIFO_V	:= $(wildcard $(LIBDIR)/fifo/*.v)
FIFO	:= $(filter-out %_tb.v, $(FIFO_V))

MISC_V	:= $(wildcard $(LIBDIR)/misc/*.v)
MISC	:= $(filter-out %_tb.v, $(MISC_V))

DDR3_V	:= $(wildcard $(RTLDIR)/*.v)
DDR3	:= $(filter-out %_tb.v, $(DDR3_V))

LIB	:= $(ARCH) $(AXIS) $(DDR3) $(FIFO) $(MISC)
INC	:= $(wildcard *.vh $(RTLDIR)/*.vh $(LIBDIR)/axis/*.vh)

SRC	:= $(wildcard *.v)
RTL	:= $(filter-out %_tb.v, $(SRC))

TOP	?= $(wildcard *_tb.v)
OUT	?= $(TOP:%.v=../build/%.out)

sim:	build

build:	$(OUT)

../build/%.out: %.v $(RTL) $(LIB)
	$(IVC) $(OPT) -o $@ -s $(*F) $^
