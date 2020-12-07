#!/bin/bash

## Copyright 2009-2020 Intel Corporation
## SPDX-License-Identifier: Apache-2.0

# enable early exit on fail
set -e

BDSTOOL=$PROTEX_PATH/bin/bdstool
PROTEX_PROJECT_NAME=$PROTEX_PROJECT_NAME
SERVER_URL=$PROTEX_SERVER_URL
SRC_PATH=$CI_PROJECT_DIR
LOG_FILE=/tmp/ip_protex.log

export _JAVA_OPTIONS=-Duser.home=$PROTEX_PATH/home

# enter source code directory before scanning
cd $SRC_PATH

$BDSTOOL new-project --server $SERVER_URL $PROTEX_PROJECT_NAME |& tee $LOG_FILE
$BDSTOOL analyze --server $SERVER_URL --path $SRC_PATH |& tee -a $LOG_FILE

if ! grep -E "^Files scanned successfully with no discoveries: [0-9]+$" $LOG_FILE; then
    echo "Protex scan FAILED!"
    exit 1
fi

if grep -E "^Files pending identification: [0-9]+$" $LOG_FILE; then
    echo "Protex scan FAILED! Some pending identification found. Please check on $SERVER_URL"
    exit 1
fi

echo "Protex scan PASSED!"
exit 0
