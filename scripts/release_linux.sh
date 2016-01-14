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
    version=(`echo $sym | sed 's/.*@@\(.*\)$/\1/p' | grep -E -o "[0-9]+"`)
    if [ ${#version[@]} -ne 0 ]; then
      #echo "version0 = " ${version[0]}
      #echo "version1 = " ${version[1]}
      if [ ${version[0]} -gt $3 ]; then
        echo "Error: problematic $2 symbol " $sym
        exit 1
      fi
      if [ ${version[1]} -gt $4 ]; then
        echo "Error: problematic $2 symbol " $sym
        exit 1
      fi
    fi
  done
}

mkdir -p build
cd build
rm CMakeCache.txt # make sure to use default settings
rm version.h

# set release settings
cmake \
-D CMAKE_C_COMPILER:FILEPATH=icc \
-D CMAKE_CXX_COMPILER:FILEPATH=icpc \
-D XEON_ISA=AVX512KNL \
-D ENABLE_XEON_PHI_SUPPORT=ON \
-D USE_IMAGE_MAGICK=OFF \
-D USE_LIBJPEG=OFF \
-D USE_LIBPNG=OFF \
-D USE_OPENEXR=OFF \
..

# read embree version
EMBREE_VERSION_MAJOR=`sed -n 's/#define __EMBREE_VERSION_MAJOR__ \(.*\)/\1/p' version.h`
EMBREE_VERSION_MINOR=`sed -n 's/#define __EMBREE_VERSION_MINOR__ \(.*\)/\1/p' version.h`
EMBREE_VERSION_PATCH=`sed -n 's/#define __EMBREE_VERSION_PATCH__ \(.*\)/\1/p' version.h`
EMBREE_VERSION=${EMBREE_VERSION_MAJOR}.${EMBREE_VERSION_MINOR}.${EMBREE_VERSION_PATCH}

# create RPM files
cmake \
-D RTCORE_ZIP_MODE=OFF \
-D CMAKE_SKIP_INSTALL_RPATH=OFF \
-D CMAKE_INSTALL_PREFIX=/usr \
-D TBB_ROOT=/usr ..
make -j 16 preinstall

check_symbols libembree.so GLIBC 2 4
check_symbols libembree.so GLIBCXX 3 4
check_symbols libembree.so CXXABI 1 3
make -j 16 package

# rename RPMs to have component name before version
for i in embree-${EMBREE_VERSION}-1.*.rpm ; do 
  newname=`echo $i | sed -e "s/embree-\(.\+\)-\([a-z_]\+\)\.rpm/embree-\2-\1.rpm/"`
  mv $i $newname
done

tar czf embree-${EMBREE_VERSION}.x86_64.rpm.tar.gz embree-*-${EMBREE_VERSION}-1.x86_64.rpm

# create tar.gz files
cmake \
-D RTCORE_ZIP_MODE=ON \
-D CMAKE_SKIP_INSTALL_RPATH=ON \
-D CMAKE_INSTALL_INCLUDEDIR=include \
-D CMAKE_INSTALL_LIBDIR=lib \
-D CMAKE_INSTALL_DOCDIR=doc \
-D CMAKE_INSTALL_BINDIR=bin \
-D TBB_ROOT=$TBB_PATH_LOCAL ..
make -j 16 preinstall

check_symbols libembree.so GLIBC 2 4
check_symbols libembree.so GLIBCXX 3 4
check_symbols libembree.so CXXABI 1 3
make -j 16 package

rm CMakeCache.txt # reset settings
cd ..

