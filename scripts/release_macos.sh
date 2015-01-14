#!/bin/bash

if [ "$#" -ne 1 ]; then
  echo "Usage: ./release_macos.sh path-to-bin-folder"
  exit 1
fi

realpath() {
  OURPWD=$PWD
  cd "$(dirname "$1")"
  LINK=$(readlink "$(basename "$1")")
  while [ "$LINK" ]; do
    cd "$(dirname "$LINK")"
    LINK=$(readlink "$(basename "$1")")
  done
  destdir="$PWD/$(basename "$1")"
  cd "$OURPWD"
}

realpath $1

mkdir -p build
cd build
rm CMakeCache.txt # make sure to use default settigs
cmake \
-D COMPILER=ICC \
-D CMAKE_SKIP_RPATH=ON \
..
make -j 8 preinstall
echo $destdir
cmake -D CMAKE_INSTALL_PREFIX="$destdir" -P cmake_install.cmake
cd ..

# assumes documentation repo cloned into embree-doc
make -C embree-doc docbin
cp embree-doc/docbin/* "$destdir"
