#!/bin/bash

## Copyright 2009-2020 Intel Corporation
## SPDX-License-Identifier: Apache-2.0


PROTEX_ROOT=/NAS/tools/ip_protex/protex_v7.8
BDSTOOL=$PROTEX_ROOT/bin/bdstool
PROTEX_PROJECT_NAME=c_embreeraytracingkernels_14283
SERVER_URL=https://amrprotex003.devtools.intel.com/
SRC_PATH=$CI_PROJECT_DIR/

export _JAVA_OPTIONS=-Duser.home=$PROTEX_ROOT/home

# enter source code directory before scanning
cd $SRC_PATH

$BDSTOOL new-project --server $SERVER_URL $PROTEX_PROJECT_NAME |& tee ip_protex.log
if grep -q "fail\|error\|fatal\|not found" ip_protex.log; then
    exit 1
fi

$BDSTOOL analyze --server $SERVER_URL |& tee -a ip_protex.log
if grep -q "fail\|error\|fatal\|not found" ip_protex.log; then
    exit 1
fi

if grep -E "^Files pending identification: [0-9]+$" ip_protex.log; then
    echo "Protex scan FAILED!"
    exit 1
fi

echo "Protex scan PASSED!"
exit 0