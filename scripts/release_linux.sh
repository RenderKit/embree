#!/bin/bash

if [ "$#" -ne 1 ]; then
  echo "Usage: ./scripts/release_linux.sh path-to-bin-folder"
  exit 1
fi

destdir=`readlink -f "$1"`

mkdir -p build
cd build
rm CMakeCache.txt # make sure to use default settings
cmake \
-D COMPILER=ICC \
-D ENABLE_XEON_PHI_SUPPORT=ON \
-D RTCORE_SPINLOCKS=ON \
-D CMAKE_SKIP_INSTALL_RPATH=ON \
..
make -j 8 preinstall
cmake -D CMAKE_INSTALL_PREFIX="$destdir" -P cmake_install.cmake
cd ..

# install scripts
cp scripts/install_linux/install.sh scripts/install_linux/paths.sh "$destdir"

# assumes documentation repo cloned into embree-doc
make -C embree-doc docbin
cp embree-doc/docbin/* "$destdir"
