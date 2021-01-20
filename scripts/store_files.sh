#!/bin/bash -xe
## Copyright 2019-2021 Intel Corporation
## SPDX-License-Identifier: Apache-2.0

project_name=$1
build_id=$2
group_name=$3
files=$4
STORAGE_DIR=$STORAGE_PATH/$project_name/$build_id/$group_name/
mkdir -p $STORAGE_DIR
cp $files $STORAGE_DIR/
