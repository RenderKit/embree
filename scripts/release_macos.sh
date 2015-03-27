#!/bin/bash

if [ "$#" -ne 1 ]; then
  echo "Usage: ./scripts/release_macos.sh path-to-bin-folder"
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

realpath "$1"

mkdir -p build
cd build
rm CMakeCache.txt # make sure to use default settings
cmake \
-D COMPILER=ICC \
-D CMAKE_SKIP_INSTALL_RPATH=ON \
..

# assumes documentation repo cloned into embree-doc
make -C embree-doc docbin

make -j 8 preinstall
cmake -D CMAKE_INSTALL_PREFIX="$destdir" -P cmake_install.cmake
cd ..

# install scripts
cp scripts/install_macos/install.sh scripts/install_macos/paths.sh "$destdir"

