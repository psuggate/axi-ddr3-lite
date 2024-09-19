.PHONY:	all sim build
all:	build
sim:	build

#
#  Icarus Verilog settings
##
LIBDIR	?= ../lib/misc-verilog-cores
BENCH	:= ../bench

IVC	?= iverilog
OPT	:= -g2005-sv -D__icarus -Wall -I. -I$(BENCH) -I$(LIBDIR)/axis

SRC	:= $(wildcard *.v)
INC	:= $(wildcard *.vh $(BENCH)/*.vh $(LIBDIR)/axis/*.vh)

ARCH	:= $(wildcard $(LIBDIR)/arch/*.v) $(wildcard $(BENCH)/arch/*.v)
AXIS	:= $(wildcard $(LIBDIR)/axis/*.v)
FIFO	:= $(wildcard $(LIBDIR)/fifo/*.v)
MISC	:= $(wildcard $(LIBDIR)/misc/*.v)

LIB	:= $(ARCH) $(AXIS) $(FIFO) $(MISC) $(BENCH)/ddr3.v
RTL	:= $(filter-out %_tb.v, $(SRC))
BENCH	:= $(filter %_tb.v, $(SRC))
OUT	:= $(BENCH:%.v=../build/%.out)

build:	$(OUT) $(RTL) $(INC)
	echo "$(LIB)"

../build/%.out: %.v $(RTL) $(LIB)
	$(IVC) $(OPT) -o $@ -s $(*F) $^
