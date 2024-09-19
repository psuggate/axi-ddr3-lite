#!/bin/bash
# Use TMP dir to clone
TMP=`mktemp --directory`
pushd ${TMP}
git clone --depth=1 https://github.com/psuggate/misc-verilog-cores.git
cd misc-verilog-cores
git checkout 943b45ab4608153f00dfac2753dc0816bbf54377
popd
mkdir -p misc-verilog-cores
cp -a ${TMP}/misc-verilog-cores/rtl/* misc-verilog-cores/
cp -a ${TMP}/misc-verilog-cores/*.md misc-verilog-cores/
cp -a ${TMP}/misc-verilog-cores/driver misc-verilog-cores/
echo "removing temp directory ${TMP}"
rm -rf ${TMP}
