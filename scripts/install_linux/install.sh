#!/bin/bash

pushd . > /dev/null
SCRIPT_PATH="${BASH_SOURCE[0]}";
if ([ -h "${SCRIPT_PATH}" ]) then
  while([ -h "${SCRIPT_PATH}" ]) do cd `dirname "$SCRIPT_PATH"`; SCRIPT_PATH=`readlink "${SCRIPT_PATH}"`; done
fi
cd "`dirname "$SCRIPT_PATH"`" > /dev/null
SCRIPT_PATH=`pwd`;
popd > /dev/null

if [ "$#" -ne 1 ]; then
  INSTALL_PREFIX=/usr/local
else
  INSTALL_PREFIX="$1"
fi

echo "Installing Embree v@EMBREE_VERSION@ ... "

INCLUDE_INSTALL_DIR="$INSTALL_PREFIX"/include
mkdir -p -m 755 "$INCLUDE_INSTALL_DIR"

if [ -d "$INSTALL_PREFIX/lib64" ] ; then
  LIBRARY_INSTALL_DIR="$INSTALL_PREFIX"/lib64
else
  LIBRARY_INSTALL_DIR="$INSTALL_PREFIX"/lib
  mkdir -p -m 755 "$LIBRARY_INSTALL_DIR"
fi

echo "  installing include files in" $INCLUDE_INSTALL_DIR
cp --preserve=mode -r "$SCRIPT_PATH"/include/embree2 "$INCLUDE_INSTALL_DIR"

echo "  installing Xeon library files in" $LIBRARY_INSTALL_DIR
cp --preserve=mode -d "$SCRIPT_PATH"/lib/libembree.so* "$LIBRARY_INSTALL_DIR"

echo "  installing Xeon Phi library files in" $LIBRARY_INSTALL_DIR
cp --preserve=mode -d "$SCRIPT_PATH"/lib/libembree_xeonphi.so* "$LIBRARY_INSTALL_DIR"
