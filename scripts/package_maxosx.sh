#!/bin/bash

# read embree version
#EMBREE_VERSION_MAJOR=`sed -n 's/#define __EMBREE_VERSION_MAJOR__ \(.*\)/\1/p' version.h`
#EMBREE_VERSION_MINOR=`sed -n 's/#define __EMBREE_VERSION_MINOR__ \(.*\)/\1/p' version.h`
#EMBREE_VERSION_PATCH=`sed -n 's/#define __EMBREE_VERSION_PATCH__ \(.*\)/\1/p' version.h`
#EMBREE_VERSION=${EMBREE_VERSION_MAJOR}.${EMBREE_VERSION_MINOR}.${EMBREE_VERSION_PATCH}
EMBREE_VERSION=$2

# create package
make -j 4 package

if [ "$1" == "ON" ]; then
  embree_tgz=embree-${EMBREE_VERSION}.x86_64.macosx.tar.gz
  echo "<DartMeasurementFile name=\"${embree_tgz}\" type=\"file\">${embree_tgz}</DartMeasurementFile>"
else
  embree_dmg=embree-${EMBREE_VERSION}.x86_64.dmg
  echo "<DartMeasurementFile name=\"${embree_dmg}\" type=\"file\">${embree_dmg}</DartMeasurementFile>"
fi

