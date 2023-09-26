#!/bin/bash

## Copyright 2009-2021 Intel Corporation
## SPDX-License-Identifier: Apache-2.0

PACKAGE_NAME=$1

echo $EMBREE_VERSION

mv ${PACKAGE_NAME}-embree.tar.gz ${PACKAGE_NAME}.tar.gz
mv ${PACKAGE_NAME}-embree-testing.tar.gz ${PACKAGE_NAME}-testing.tar.gz
rm -f ${PACKAGE_NAME}-Unspecified.tar.gz

ls -l *.tar.gz