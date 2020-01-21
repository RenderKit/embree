#!/bin/bash

## ======================================================================== ##
## Copyright 2009-2020 Intel Corporation                                    ##
##                                                                          ##
## Licensed under the Apache License, Version 2.0 (the "License");          ##
## you may not use this file except in compliance with the License.         ##
## You may obtain a copy of the License at                                  ##
##                                                                          ##
##     http://www.apache.org/licenses/LICENSE-2.0                           ##
##                                                                          ##
## Unless required by applicable law or agreed to in writing, software      ##
## distributed under the License is distributed on an "AS IS" BASIS,        ##
## WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. ##
## See the License for the specific language governing permissions and      ##
## limitations under the License.                                           ##
## ======================================================================== ##


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