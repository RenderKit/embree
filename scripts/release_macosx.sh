#!/bin/bash

# to make sure we do not include nor link against wrong TBB
export CPATH=
export LIBRARY_PATH=
export DYLD_LIBRARY_PATH=
TBB_PATH_LOCAL=$PWD/tbb

mkdir -p build
cd build
rm CMakeCache.txt # make sure to use default settings
rm version.h

# set release settings
cmake \
-D CMAKE_C_COMPILER:FILEPATH=icc \
-D CMAKE_CXX_COMPILER:FILEPATH=icpc \
-D XEON_ISA=AVX2 \
-D USE_IMAGE_MAGICK=OFF \
-D USE_LIBJPEG=OFF \
-D USE_LIBPNG=OFF \
-D USE_OPENEXR=OFF \
..

# create installers
cmake \
-D RTCORE_ZIP_MODE=OFF \
-D CMAKE_SKIP_INSTALL_RPATH=OFF \
-D CMAKE_INSTALL_PREFIX=/opt/local \
-D CMAKE_INSTALL_INCLUDEDIR=include \
-D CMAKE_INSTALL_LIBDIR=lib \
-D CMAKE_INSTALL_DOCDIR=../../Applications/Embree2/doc \
-D CMAKE_INSTALL_BINDIR=../../Applications/Embree2/bin \
-D TBB_ROOT=/opt/local \
..
make -j 4 package

# create ZIP files
cmake \
-D RTCORE_ZIP_MODE=ON \
-D CMAKE_SKIP_INSTALL_RPATH=ON \
-D CMAKE_INSTALL_INCLUDEDIR=include \
-D CMAKE_INSTALL_LIBDIR=lib \
-D CMAKE_INSTALL_DOCDIR=doc \
-D CMAKE_INSTALL_BINDIR=bin \
-D TBB_ROOT=$TBB_PATH_LOCAL \
..
make -j 4 package

rm CMakeCache.txt # reset settings
cd ..
