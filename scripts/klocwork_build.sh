#!/bin/bash

## Copyright 2009-2020 Intel Corporation
## SPDX-License-Identifier: Apache-2.0

set -e

KW_PATH=/NAS/tools/kw
KW_SERVER_PATH=$KW_PATH/server
KW_CLIENT_PATH=$KW_PATH/client
export KLOCWORK_LTOKEN=/tmp/ltoken

echo "$KW_SERVER_IP;$KW_SERVER_PORT;$KW_USER;$KW_LTOKEN" > $KLOCWORK_LTOKEN
make clean > /dev/null
$KW_CLIENT_PATH/bin/kwinject -w -o buildspec.txt make -j 8
$KW_SERVER_PATH/bin/kwbuildproject --force --url http://$KW_SERVER_IP:$KW_SERVER_PORT/embree buildspec.txt --tables-directory $CI_PROJECT_DIR/kw_tables
$KW_SERVER_PATH/bin/kwadmin --url http://$KW_SERVER_IP:$KW_SERVER_PORT/ load --force --name build-$CI_JOB_ID embree $CI_PROJECT_DIR/kw_tables

# store kw build number for check status later
echo "build-$CI_JOB_ID" > ./kw_build_number
