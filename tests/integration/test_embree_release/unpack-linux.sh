#!/bin/bash
## Copyright 2019-2021 Intel Corporation
## SPDX-License-Identifier: Apache-2.0

cp build/embree-*.tar.gz tests/integration/test_embree_release/embree.tar.gz
cd tests/integration/test_embree_release
mkdir embree_install
tar -xzf embree.tar.gz -C embree_install
export EMBREE_CONFIG_DIR=./embree_install/lib/cmake/`cd embree_install/lib/cmake && ls`

