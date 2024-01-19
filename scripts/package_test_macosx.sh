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
ctest -VV --output-log ctest.output
if grep "No tests were found" ctest.output; then exit 1; fi