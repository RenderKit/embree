#!/bin/bash

PROTEX_HOME=/NAS/tools/ip_protex/
PROTEX_ROOT=$PROTEX_HOME/ip_protex_7.1.3/protexIP
BDSTOOL=$PROTEX_ROOT/bin/bdstool
PROTEX_PROJECT_NAME=c_embreeraytracingkernels_14283
SRC_PATH=$CI_PROJECT_DIR/

export _JAVA_OPTIONS=-Duser.home=$PROTEX_HOME/protex_home_sc03

# enter source code directory before scanning
cd $SRC_PATH

$BDSTOOL new-project $PROTEX_PROJECT_NAME |& tee ip_protex.log
if grep -q "command failed" ip_protex.log; then
    exit 1
fi

$BDSTOOL analyze |& tee -a ip_protex.log
if grep -q "command failed" ip_protex.log; then
    exit 1
fi

if grep -E "^Files pending identification: [0-9]+$" ip_protex.log; then
    echo "Protex scan FAILED!"
    exit 1
fi

echo "Protex scan PASSED!"
exit 0

