#!/bin/bash
# Use TMP dir to clone
TMP=`mktemp --directory`
pushd ${TMP}
git clone --depth=1 https://github.com/psuggate/misc-verilog-cores.git
cd misc-verilog-cores
git checkout 6c9d67a6b10024b72b8e76aa474b91f9babd6280
popd
mkdir -p misc-verilog-cores
cp -a ${TMP}/misc-verilog-cores/rtl/arch misc-verilog-cores/
cp -a ${TMP}/misc-verilog-cores/rtl/axis misc-verilog-cores/
cp -a ${TMP}/misc-verilog-cores/rtl/fifo misc-verilog-cores/
cp -a ${TMP}/misc-verilog-cores/rtl/misc misc-verilog-cores/
cp -a ${TMP}/misc-verilog-cores/rtl/spi  misc-verilog-cores/
cp -a ${TMP}/misc-verilog-cores/rtl/uart misc-verilog-cores/
cp -a ${TMP}/misc-verilog-cores/rtl/Makefile misc-verilog-cores/
cp -a ${TMP}/misc-verilog-cores/*.md misc-verilog-cores/
cp -a ${TMP}/misc-verilog-cores/driver misc-verilog-cores/
echo "removing temp directory ${TMP}"
rm -rf ${TMP}
