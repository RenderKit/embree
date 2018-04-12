#!/bin/bash

# to make sure we do not include nor link against wrong TBB
export CPATH=
export LIBRARY_PATH=
export LD_LIBRARY_PATH=
TBB_PATH_LOCAL=$PWD/tbb

# check version of symbols
function check_symbols
{
  for sym in `nm $1 | grep $2_`
  do
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
EMBREE_VERSION_MAJOR=`sed -n 's/#define RTC_VERSION_MAJOR \(.*\)/\1/p' include/embree2/rtcore_version.h`
EMBREE_VERSION_MINOR=`sed -n 's/#define RTC_VERSION_MINOR \(.*\)/\1/p' include/embree2/rtcore_version.h`
EMBREE_VERSION_PATCH=`sed -n 's/#define RTC_VERSION_PATCH \(.*\)/\1/p' include/embree2/rtcore_version.h`
EMBREE_VERSION=${EMBREE_VERSION_MAJOR}.${EMBREE_VERSION_MINOR}.${EMBREE_VERSION_PATCH}

mkdir -p build
cd build
rm CMakeCache.txt # make sure to use default settings

# set compiler settings
cmake \
-D CMAKE_C_COMPILER:FILEPATH=icc \
-D CMAKE_CXX_COMPILER:FILEPATH=icpc \
..

# set release settings
cmake \
-D EMBREE_STACK_PROTECTOR=ON\
-D EMBREE_MAX_ISA=NONE \
-D EMBREE_ISA_SSE2=ON \
-D EMBREE_ISA_SSE42=ON \
-D EMBREE_ISA_AVX=ON \
-D EMBREE_ISA_AVX2=ON \
-D EMBREE_ISA_AVX512KNL=ON \
-D EMBREE_ISA_AVX512SKX=ON \
-D EMBREE_TUTORIALS_OPENIMAGEIO=OFF \
-D EMBREE_TUTORIALS_LIBJPEG=OFF \
-D EMBREE_TUTORIALS_LIBPNG=OFF \
..

# create RPM files
cmake \
-D EMBREE_INSTALL_DEPENDENCIES=OFF \
-D EMBREE_ZIP_MODE=OFF \
-D CPACK_PACKAGING_INSTALL_PREFIX=/usr \
-D EMBREE_TBB_ROOT=/usr ..
make -j 16 preinstall

check_symbols libembree.so GLIBC 2 4 0
check_symbols libembree.so GLIBCXX 3 4 11
check_symbols libembree.so CXXABI 1 3 0
make -j 16 package

tar czf embree-${EMBREE_VERSION}.x86_64.rpm.tar.gz embree-*-${EMBREE_VERSION}-*.rpm

# create tar.gz files
cmake \
-D EMBREE_INSTALL_DEPENDENCIES=ON \
-D EMBREE_ZIP_MODE=ON \
-D CPACK_PACKAGING_INSTALL_PREFIX=/ \
-D CMAKE_INSTALL_INCLUDEDIR=include \
-D CMAKE_INSTALL_LIBDIR=lib \
-D CMAKE_INSTALL_DOCDIR=doc \
-D CMAKE_INSTALL_BINDIR=bin \
-D EMBREE_TBB_ROOT=$TBB_PATH_LOCAL ..
make -j 16 preinstall

check_symbols libembree.so GLIBC 2 4 0
check_symbols libembree.so GLIBCXX 3 4 11
check_symbols libembree.so CXXABI 1 3 0
make -j 16 package

rm CMakeCache.txt # reset settings
cd ..
