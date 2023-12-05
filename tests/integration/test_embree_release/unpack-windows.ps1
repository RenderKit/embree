# Copyright 2019-2021 Intel Corporation
# SPDX-License-Identifier: Apache-2.0

# unpack embree
mkdir embree_install
cp embree-*.zip          embree_install
cd embree_install
mv embree-*-testing.zip  testing.tar.gz      # rename this first, so that embree-*.tar.gz only specifies one package
mv embree-*.zip          embree.tar.gz
tar -xf embree.tar.gz
tar -xf testing.tar.gz
ls
$env:EMBREE_CONFIG_DIR_PT1 = pwd
$env:EMBREE_CONFIG_DIR_PT2 = ls lib\cmake
$env:EMBREE_CONFIG_DIR = $env:EMBREE_CONFIG_DIR_PT1 + "\lib\cmake\" + $env:EMBREE_CONFIG_DIR_PT2
echo $env:EMBREE_CONFIG_DIR
cd ..