#!/bin/bash

## Copyright 2009-2021 Intel Corporation
## SPDX-License-Identifier: Apache-2.0

# terminate if some error occurs
set -e

# check version of symbols
function check_symbols
{
  for sym in `nm $1 | grep $2_`
  do
    if [ ${#sym} -le 1 ]; then
        continue;
    fi;
    version=(`echo $sym | cut -d"@" -f2 | grep -E -o "[0-9]+"`)
    if [ ${#version[@]} -ne 0 ]; then
      if [ ${#version[@]} -eq 1 ]; then version[1]=0; fi
      if [ ${#version[@]} -eq 2 ]; then version[2]=0; fi
      #echo $sym
      #echo "version0 = " ${version[0]}
      #echo "version1 = " ${version[1]}
      #echo "version2 = " ${version[2]}
      if [ ${version[0]} -gt $3 ]; then
        echo "Error: problematic $2 symbol " $sym
        exit 1
      fi
      if [ ${version[0]} -lt $3 ]; then continue; fi

      if [ ${version[1]} -gt $4 ]; then
        echo "Error: problematic $2 symbol " $sym
        exit 1
      fi
      if [ ${version[1]} -lt $4 ]; then continue; fi

      if [ ${version[2]} -gt $5 ]; then
        echo "Error: problematic $2 symbol " $sym
        exit 1
      fi
    fi
  done
}

# read embree version
EMBREE_ZIP_MODE=$1
EMBREE_LIBRARY_NAME=$2
EMBREE_VERSION=$3
EMBREE_VERSION_MAJOR=$4
EMBREE_SIGN_FILE=$5

# create package
#make -j 16 preinstall
cmake --build . --target package

check_symbols lib${EMBREE_LIBRARY_NAME}.so GLIBC 2 28 0
check_symbols lib${EMBREE_LIBRARY_NAME}.so GLIBCXX 3 4 22
check_symbols lib${EMBREE_LIBRARY_NAME}.so CXXABI 1 3 11

#make -j 16 package
cmake --build . --target package

if [ "$EMBREE_ZIP_MODE" == "ON" ]; then
  # rename packages and make sure they contain a 'parent folder', so that not a truckload of files and folders are extracted unintentionally into the working directory
  mkdir embree-${EMBREE_VERSION}.x86_64.linux
  tar -xzf embree-${EMBREE_VERSION}.x86_64.linux-embree.tar.gz -C embree-${EMBREE_VERSION}.x86_64.linux
  rm embree-${EMBREE_VERSION}.x86_64.linux-embree.tar.gz
  tar -czvf embree-${EMBREE_VERSION}.x86_64.linux.tar.gz embree-${EMBREE_VERSION}.x86_64.linux

  mkdir embree-${EMBREE_VERSION}-testing.x86_64.linux
  tar -xzf embree-${EMBREE_VERSION}.x86_64.linux-embree-testing.tar.gz -C embree-${EMBREE_VERSION}-testing.x86_64.linux
  rm embree-${EMBREE_VERSION}.x86_64.linux-embree-testing.tar.gz
  tar -czvf embree-${EMBREE_VERSION}-testing.x86_64.linux.tar.gz embree-${EMBREE_VERSION}-testing.x86_64.linux

  rm embree-${EMBREE_VERSION}.x86_64.linux-Unspecified.tar.gz
fi

if [ "$EMBREE_ZIP_MODE" == "OFF" ]; then

  # sign all RPM files
  if [ $# -eq 5 ]; then
    ${EMBREE_SIGN_FILE} embree${EMBREE_VERSION_MAJOR}-*-${EMBREE_VERSION}-*.rpm
  fi

  # create TGZ of RPMs
  embree_tgz=embree-${EMBREE_VERSION}.x86_64.rpm.tar.gz
  tar czf ${embree_tgz} embree${EMBREE_VERSION_MAJOR}-*-${EMBREE_VERSION}-*.rpm

fi