#!/bin/bash

#if [ "$#" -ne 1 ]; then
#  echo "Usage: ./scripts/release_linux.sh path-to-bin-folder"
#  exit 1
#fi

#destdir=`readlink -f "$1"`

export CPATH=
export LD_LIBRARY_PATH=

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

check_symbols libembree.so GLIBC 2 3
check_symbols libembree.so GLIBCXX 3 4
check_symbols libembree.so CXXABI 1 3

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

