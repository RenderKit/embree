#!/bin/bash

#if [ "$#" -ne 1 ]; then
#  echo "Usage: ./scripts/release_macos.sh path-to-bin-folder"
#  exit 1
#fi

#realpath() {
#  OURPWD=$PWD
#  cd "$(dirname "$1")"
#  LINK=$(readlink "$(basename "$1")")
#  while [ "$LINK" ]; do
#    cd "$(dirname "$LINK")"
#    LINK=$(readlink "$(basename "$1")")
#  done
#  destdir="$PWD/$(basename "$1")"
#  cd "$OURPWD"
#}

#realpath "$1"

TBB_PATH_LOCAL=$PWD/tbb

mkdir -p build
cd build
rm CMakeCache.txt # make sure to use default settings
rm version.h

# set release settings
cmake \
-D COMPILER=ICC \
-D CMAKE_INSTALL_PREFIX=/opt/local \
-D USE_IMAGE_MAGICK=OFF \
-D USE_LIBJPEG=OFF \
-D USE_LIBPNG=OFF \
-D USE_OPENEXR=OFF \
..

# create installers
cmake \
-D TBB_ROOT=/opt/local \
-D ENABLE_INSTALLER=ON \
..
make -j 8 package

# create ZIP files
cmake \
-D TBB_ROOT=$TBB_PATH_LOCAL \
-D ENABLE_INSTALLER=OFF \
..
make -j 8 package

cd ..
