:: Copyright 2009-2021 Intel Corporation
:: SPDX-License-Identifier: Apache-2.0

@echo off

set TESTING_INTENSITY=%1
set TESTING_CONFIG=%2

:: unpack embree
mkdir embree_install
dir
copy embree-*.zip embree_install
cd embree_install
dir
:: rename this first, so that embree-*.tar.gz only specifies one package
ren embree-*-testing.zip testing.zip
ren embree-*.zip embree.zip
dir
tar -xf embree.zip
tar -xf testing.zip

:: build and run tests
cd testing
echo "running embree tests with intensity %TESTING_INTENSITY% and config %TESTING_CONFIG%"
cmake -B build -DEMBREE_TESTING_INTENSITY=%TESTING_INTENSITY%
cd build
ctest -VV -C %TESTING_CONFIG% --output-log ctest.output
findstr /C:"No tests were found" ctest.output && exit 1 || exit 0
