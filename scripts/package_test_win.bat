rem ## Copyright 2009-2021 Intel Corporation
rem ## SPDX-License-Identifier: Apache-2.0

@echo off


set TESTING_INTENSITY=%1

rem # unpack embree
mkdir embree_install
dir
copy embree-*.zip           embree_install
cd embree_install
echo "cd embree_install"
dir
echo "ren testing"
ren embree-*-testing.zip    testing.zip      rem # rename this first, so that embree-*.tar.gz only specifies one package
echo "ren embreet"
ren embree-*.zip            embree.zip
echo "renamed"
dir
tar -xf embree.zip
tar -xf testing.zip

rem # build and run tests
cd testing
echo "running embree tests with intensity %TESTING_INTENSITY%"
cmake -B build -DEMBREE_TESTING_INTENSITY=%TESTING_INTENSITY%
cd build 
ctest -VV






rem set package_name=%1
rem 
rem mkdir embree_install
rem tar -xf %package_name%.tar.gz -C embree_install
rem tar -xf %package_name%-testing.tar.gz -C embree_install
rem 
rem cd embree_install/testing
rem cmake -B build -DEMBREE_TESTING_INTENSITY=4
rem cd build
rem ctest -VV