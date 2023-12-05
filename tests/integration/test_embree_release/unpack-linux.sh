#!/bin/bash
## Copyright 2019-2021 Intel Corporation
## SPDX-License-Identifier: Apache-2.0

# unpack embree
mkdir embree_install
cp embree-*.tar.gz          embree_install
cd embree_install
mv embree-*-testing.tar.gz  testing.tar.gz      # rename this first, so that embree-*.tar.gz only specifies one package
mv embree-*.tar.gz          embree.tar.gz
ls -l
tar -xzf embree.tar.gz
tar -xzf testing.tar.gz
export EMBREE_CONFIG_DIR=`pwd`/lib/cmake/`cd lib/cmake && ls`
echo $EMBREE_CONFIG_DIR
ls -l
cd ..

#cp build/embree-*.tar.gz tests/integration/test_embree_release/embree.tar.gz
#cd tests/integration/test_embree_release
#mkdir embree_install
#tar -xzf embree.tar.gz -C embree_install
#export EMBREE_CONFIG_DIR=./embree_install/lib/cmake/`cd embree_install/lib/cmake && ls`

