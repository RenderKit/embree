rem ## Copyright 2009-2021 Intel Corporation
rem ## SPDX-License-Identifier: Apache-2.0

@echo off

set outfile=%1
set outarch=%2

mkdir embree_install
tar -xf %outfile%.%outarch%.windows.tar.gz -C embree_install
tar -xf %outfile%.%outarch%.windows-testing.tar.gz -C embree_install

cd embree_install/testing
cmake -B build -DEMBREE_TESTING_INTENSITY=4
cd build
ctest -VV