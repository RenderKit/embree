#!/bin/bash

## Copyright 2009-2021 Intel Corporation
## SPDX-License-Identifier: Apache-2.0

EMBREE_VERSION=$1

mkdir embree_install
tar -xzf embree-${EMBREE_VERSION}.x86_64.linux.tar.gz -C embree_install
tar -xzf embree-${EMBREE_VERSION}.x86_64.linux-testing.tar.gz -C embree_install

cd embree_install/testing
cmake -B build -DEMBREE_TESTING_INTENSITY=4
cd build
ctest -VV
