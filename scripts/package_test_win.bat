rem ## Copyright 2009-2021 Intel Corporation
rem ## SPDX-License-Identifier: Apache-2.0

@echo off

set package_name=%1

mkdir embree_install
tar -xf %package_name%.tar.gz -C embree_install
tar -xf %package_name%-testing.tar.gz -C embree_install

cd embree_install/testing
cmake -B build -DEMBREE_TESTING_INTENSITY=4
cd build
ctest -VV