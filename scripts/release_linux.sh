#!/bin/bash

if [ "$#" -ne 1 ]; then
  echo "Usage: ./scripts/release_linux.sh path-to-bin-folder"
  exit 1
fi

destdir=`readlink -f "$1"`

# assumes documentation repo cloned into embree-doc
make -C embree-doc docbin

mkdir -p build
cd build
rm CMakeCache.txt # make sure to use default settings
cmake \
-D COMPILER=ICC \
-D ENABLE_XEON_PHI_SUPPORT=ON \
-D CMAKE_SKIP_INSTALL_RPATH=ON \
..
make -j 8 preinstall
umask_org=`umask` # workaround for bug in CMake/CPack: need to reset umask
umask 022
cmake -D CMAKE_INSTALL_PREFIX="$destdir" -P cmake_install.cmake
make -j 8 package
cp embree-*.rpm "$destdir"
umask $umask_org
cd ..

# install scripts
cp scripts/install_linux/install.sh scripts/install_linux/paths.sh "$destdir"
