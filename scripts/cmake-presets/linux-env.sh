#!/bin/bash
## Copyright 2019-2021 Intel Corporation
## SPDX-License-Identifier: Apache-2.0

echo $STORAGE_PATH
export NAS_LINUX=$STORAGE_PATH/packages/apps
export CMAKE_EXE=${NAS_LINUX}/cmake/cmake-3.25.3-linux-x86_64/bin/cmake