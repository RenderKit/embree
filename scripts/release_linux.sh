#!/bin/bash

#if [ "$#" -ne 1 ]; then
#  echo "Usage: ./scripts/release_linux.sh path-to-bin-folder"
#  exit 1
#fi

#destdir=`readlink -f "$1"`

TBB_PATH=$PWD/tbb

mkdir -p build
cd build
rm CMakeCache.txt # make sure to use default settings
rm version.h

# set release settings
cmake \
-D COMPILER=ICC \
-D ENABLE_XEON_PHI_SUPPORT=ON \
-D USE_IMAGE_MAGICK=OFF \
-D USE_LIBJPEG=OFF \
-D USE_LIBPNG=OFF \
-D USE_OPENEXR=OFF \
..

# read embree version
VERSION_MAJOR=`sed -n 's/#define __EMBREE_VERSION_MAJOR__ \(.*\)/\1/p' version.h`
VERSION_MINOR=`sed -n 's/#define __EMBREE_VERSION_MINOR__ \(.*\)/\1/p' version.h`
VERSION_PATCH=`sed -n 's/#define __EMBREE_VERSION_PATCH__ \(.*\)/\1/p' version.h`

# make docu after cmake to have correct version.h
make -j 8 preinstall

# create RPM files
cmake -D ENABLE_INSTALLER=ON -D TBB_ROOT=/usr ..
make -j 8 package

# create tar.gz files
cmake -D ENABLE_INSTALLER=OFF -D TBB_ROOT=$TBB_PATH ..
make -j 8 package

# rename RPMs to have component name before version
for i in embree-${VERSION_MAJOR}.${VERSION_MINOR}.${VERSION_PATCH}-1.*.rpm ; do 
  newname=`echo $i | sed -e "s/embree-\(.\+\)-\([a-z_]\+\)\.rpm/embree-\2-\1.rpm/"`
  mv $i $newname
done

tar czf embree-${VERSION_MAJOR}.${VERSION_MINOR}.${VERSION_PATCH}.x86_64.rpm.tar.gz embree-*-${VERSION_MAJOR}.${VERSION_MINOR}.${VERSION_PATCH}-1.x86_64.rpm

cd ..

