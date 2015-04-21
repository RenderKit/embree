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
-D CMAKE_SKIP_INSTALL_RPATH=ON \
-D USE_IMAGE_MAGICK=OFF \
-D USE_LIBJPEG=OFF \
-D USE_LIBPNG=OFF \
-D USE_OPENEXR=OFF \
..

# make docu after cmake to have correct version.h
# assumes documentation repo cloned into embree-doc
make -C ../embree-doc docbin

make -j 8 preinstall
umask_org=`umask` # workaround for bug in CMake/CPack: need to reset umask
umask 022
cmake -D CMAKE_INSTALL_PREFIX="$destdir" -P cmake_install.cmake
rm embree-*.rpm # remove stale RPM packages
make -j 8 package
for i in embree-*.rpm ; do # rename RPMs to have component name before version
  newname=`echo $i | sed -e "s/embree-\(.\+\)-\([a-z_]\+\)\.rpm/embree-\2-\1.rpm/"`
  cp $i "$destdir"/$newname
done
umask $umask_org
cd ..

# install scripts
install scripts/install_linux/paths.sh "$destdir"
sed -e "s/@EMBREE_VERSION@/`cat embree-doc/version`/" scripts/install_linux/install.sh > "$destdir"/install.sh
chmod 755 "$destdir"/install.sh
