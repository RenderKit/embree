#!/bin/bash

## Copyright 2009-2021 Intel Corporation
## SPDX-License-Identifier: Apache-2.0

set -e

KW_SERVER_PATH=$KW_PATH/server
KW_CLIENT_PATH=$KW_PATH/client
export KLOCWORK_LTOKEN=/tmp/ltoken

echo "$KW_SERVER_IP;$KW_SERVER_PORT;$KW_USER;$KW_LTOKEN" > $KLOCWORK_LTOKEN

mkdir -p $CI_PROJECT_DIR/klocwork
log_file=$CI_PROJECT_DIR/klocwork/build.log

make clean > /dev/null
$KW_CLIENT_PATH/bin/kwinject -w -o buildspec.txt make -j 8 | tee -a $log_file
$KW_SERVER_PATH/bin/kwbuildproject --classic --force --url http://$KW_SERVER_IP:$KW_SERVER_PORT/embree buildspec.txt --tables-directory $CI_PROJECT_DIR/kw_tables | tee -a $log_file
$KW_SERVER_PATH/bin/kwadmin --url http://$KW_SERVER_IP:$KW_SERVER_PORT/ load --force --name build-$CI_JOB_ID $KW_PROJECT_NAME $CI_PROJECT_DIR/kw_tables | tee -a $log_file

# Store kw build name for check status later
echo "build-$CI_JOB_ID" > $CI_PROJECT_DIR/klocwork/build_name
