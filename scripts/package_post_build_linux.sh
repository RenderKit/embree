#!/bin/bash

## Copyright 2009-2021 Intel Corporation
## SPDX-License-Identifier: Apache-2.0

EMBREE_VERSION=$1

echo $EMBREE_VERSION

mv embree-${EMBREE_VERSION}.x86_64.linux-embree.tar.gz embree-${EMBREE_VERSION}.x86_64.linux.tar.gz
mv embree-${EMBREE_VERSION}.x86_64.linux-embree-testing.tar.gz embree-${EMBREE_VERSION}.x86_64.linux-testing.tar.gz
rm embree-${EMBREE_VERSION}.x86_64.linux-Unspecified.tar.gz

ls -l *.tar.gz