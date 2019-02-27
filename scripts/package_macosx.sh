#!/bin/bash

# terminate if some error occurs
set -e

CONFIG=$1
PACKAGE=$2
EMBREE_SIGN_FILE=$3

# create package
cmake --build . --config $CONFIG --target package

# sign PKG package
if [ ${PACKAGE: -4} == ".pkg" ]; then
  if [ -n "${EMBREE_SIGN_FILE}" ]; then
    ${EMBREE_SIGN_FILE} $PACKAGE
  fi
fi
