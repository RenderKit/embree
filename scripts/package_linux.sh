#!/bin/bash

## Copyright 2009-2021 Intel Corporation
## SPDX-License-Identifier: Apache-2.0

PACKAGE_NAME=$1

## we should be in build folder
cmake --build . --target package -j8

echo renaming ${PACKAGE_NAME}-embree.tar.gz         -> ${PACKAGE_NAME}.tar.gz
mv ${PACKAGE_NAME}-embree.tar.gz ${PACKAGE_NAME}.tar.gz
echo renaming ${PACKAGE_NAME}-embree-testing.tar.gz -> ${PACKAGE_NAME}-testing.tar.gz
mv ${PACKAGE_NAME}-embree-testing.tar.gz ${PACKAGE_NAME}-testing.tar.gz
echo deleting ${PACKAGE_NAME}-Unspecified.tar.gz
rm -f ${PACKAGE_NAME}-Unspecified.tar.gz

ls -l *.tar.gz