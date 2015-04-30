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

TBB_PATH=$PWD/tbb

mkdir -p build
cd build

# create version.h
rm CMakeCache.txt # make sure to use default settings
cmake \
-D COMPILER=ICC \
-D CMAKE_INSTALL_PREFIX=/opt/local \
-D TBB_ROOT=$TBB_PATH \
-D USE_IMAGE_MAGICK=OFF \
-D USE_LIBJPEG=OFF \
-D USE_LIBPNG=OFF \
-D USE_OPENEXR=OFF \
..
make -j 8 preinstall

# make docu after cmake to have correct version.h
#make -C ../embree-doc doc
#cp ../embree-doc/doc/* ..

# create installers
cmake \
-D ENABLE_INSTALLER=ON \
..
make -j 8 package

# create ZIP files
cmake \
-D ENABLE_INSTALLER=OFF \
..
make -j 8 package

#make -j 8 preinstall
#umask_org=`umask` # workaround for bug in CMake/CPack: need to reset umask
#umask 022
#cmake -D CMAKE_INSTALL_PREFIX="$destdir" -P cmake_install.cmake
#umask $umask_org
#cd ..

# install scripts
#install scripts/install_macos/paths.sh "$destdir"
#sed -e "s/@EMBREE_VERSION@/`cat embree-doc/version`/" scripts/install_macos/install.sh > "$destdir"/install.sh
#chmod 755 "$destdir"/install.sh
