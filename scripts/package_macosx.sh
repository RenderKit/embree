#!/bin/bash

# read embree version
#EMBREE_VERSION_MAJOR=`sed -n 's/#define __EMBREE_VERSION_MAJOR__ \(.*\)/\1/p' version.h`
#EMBREE_VERSION_MINOR=`sed -n 's/#define __EMBREE_VERSION_MINOR__ \(.*\)/\1/p' version.h`
#EMBREE_VERSION_PATCH=`sed -n 's/#define __EMBREE_VERSION_PATCH__ \(.*\)/\1/p' version.h`
#EMBREE_VERSION=${EMBREE_VERSION_MAJOR}.${EMBREE_VERSION_MINOR}.${EMBREE_VERSION_PATCH}
EMBREE_VERSION=$2

# create package
cmake --build . --config $1 --target package

echo "<DartMeasurement name=\"$2\" type=\"text/string\">$2</DartMeasurement>"
