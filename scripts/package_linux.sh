#!/bin/bash

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
    version=(`echo $sym | sed 's/.*@@\(.*\)$/\1/g' | grep -E -o "[0-9]+"`)
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
#EMBREE_VERSION_MAJOR=`sed -n 's/#define __EMBREE_VERSION_MAJOR__ \(.*\)/\1/p' version.h`
#EMBREE_VERSION_MINOR=`sed -n 's/#define __EMBREE_VERSION_MINOR__ \(.*\)/\1/p' version.h`
#EMBREE_VERSION_PATCH=`sed -n 's/#define __EMBREE_VERSION_PATCH__ \(.*\)/\1/p' version.h`
#EMBREE_VERSION=${EMBREE_VERSION_MAJOR}.${EMBREE_VERSION_MINOR}.${EMBREE_VERSION_PATCH}
EMBREE_VERSION=$2

# create package
make -j 16 preinstall
check_symbols libembree.so GLIBC 2 4 0
check_symbols libembree.so GLIBCXX 3 4 11
check_symbols libembree.so CXXABI 1 3 0
make -j 16 package

if [ "$1" == "OFF" ]; then

  # create TGZ of RPMs
  embree_tgz=embree-${EMBREE_VERSION}.x86_64.rpm.tar.gz
  tar czf ${embree_tgz} embree-*-${EMBREE_VERSION}-*.rpm

  # send RPMs to CDash
  echo "<DartMeasurement name=\"${embree_tgz}\" type=\"text/string\">${embree_tgz}</DartMeasurement>"
      
else

  # send ZIP to CDash
  embree_zip=embree-${EMBREE_VERSION}.x86_64.linux.tar.gz
  echo "<DartMeasurement name=\"${embree_zip}\" type=\"text/string\">${embree_zip}</DartMeasurement>"

fi
