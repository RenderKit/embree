#!/bin/bash

## Copyright 2009-2021 Intel Corporation
## SPDX-License-Identifier: Apache-2.0

PACKAGE_NAME=$1

mkdir embree_install
tar -xzf ${PACKAGE_NAME}.tar.gz -C embree_install
tar -xzf ${PACKAGE_NAME}-testing.tar.gz -C embree_install

cd embree_install/testing
cmake -B build -DEMBREE_TESTING_INTENSITY=4
cd build
ctest -VV
