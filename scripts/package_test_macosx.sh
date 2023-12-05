#!/bin/bash

## Copyright 2009-2021 Intel Corporation
## SPDX-License-Identifier: Apache-2.0

TESTING_INTENSITY=$1

# unpack embree
mkdir embree_install
cp embree-*.zip         embree_install
cd embree_install
mv embree-*-testing.zip testing.zip      # rename this first, so that embree-*.tar.gz only specifies one package
mv embree-*.zip         embree.zip
ls -l
tar -xf embree.zip
tar -xf testing.zip

# build and run tests
cd testing
echo "running embree tests with intensity $TESTING_INTENSITY"
cmake -B build -DEMBREE_TESTING_INTENSITY=$TESTING_INTENSITY
cd build 
ctest -VV






#PACKAGE=$1
#
#mkdir embree_install
#tar -xf ${PACKAGE}.x86_64.macosx.tar.gz -C embree_install
#tar -xf ${PACKAGE}.x86_64.macosx-testing.tar.gz -C embree_install
#
#cd embree_install/testing
#cmake -B build -DEMBREE_TESTING_INTENSITY=4
#cd build
#ctest -VV