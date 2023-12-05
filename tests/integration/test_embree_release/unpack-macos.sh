#!/bin/bash
## Copyright 2019-2021 Intel Corporation
## SPDX-License-Identifier: Apache-2.0

# unpack embree
mkdir embree_install
cp embree-*.zip          embree_install
cd embree_install
mv embree-*-testing.zip  testing.zip      # rename this first, so that embree-*.tar.gz only specifies one package
mv embree-*.zip          embree.zip
ls -l
tar -xzf embree.zip
tar -xzf testing.zip
export EMBREE_CONFIG_DIR=`pwd`/lib/cmake/`cd lib/cmake && ls`
echo $EMBREE_CONFIG_DIR
ls -l
cd ..



#cp embree-*.zip tests/integration/test_embree_release/embree.zip
#cd tests/integration/test_embree_release
#mkdir embree_install
#tar -xf embree.zip -C embree_install
#export EMBREE_CONFIG_DIR=./embree_install/lib/cmake/`cd embree_install/lib/cmake && ls`

