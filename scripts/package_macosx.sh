#!/bin/bash

## Copyright 2009-2021 Intel Corporation
## SPDX-License-Identifier: Apache-2.0

# terminate if some error occurs
set -e

PACKAGE=$1
EMBREE_SIGN_FILE=$2 

## we should be in build folder
cmake --build . --target package -j8

echo renaming ${PACKAGE}.x86_64.macosx-embree.zip         -> ${PACKAGE}.x86_64.macosx.zip
mv ${PACKAGE}.x86_64.macosx-embree.zip ${PACKAGE}.x86_64.macosx.zip
echo renaming ${PACKAGE}.x86_64.macosx-embree-testing.zip -> ${PACKAGE}.x86_64.macosx-testing.zip
mv ${PACKAGE}.x86_64.macosx-embree-testing.zip ${PACKAGE}.x86_64.macosx-testing.zip
echo deleting ${PACKAGE}.x86_64.macosx-Unspecified.zip
rm ${PACKAGE}.x86_64.macosx-Unspecified.zip