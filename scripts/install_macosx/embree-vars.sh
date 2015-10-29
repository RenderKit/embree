#!/bin/bash

pushd . > /dev/null
SCRIPT_PATH="${BASH_SOURCE[0]}";
if ([ -h "${SCRIPT_PATH}" ]) then
  while([ -h "${SCRIPT_PATH}" ]) do cd `dirname "$SCRIPT_PATH"`; SCRIPT_PATH=`readlink "${SCRIPT_PATH}"`; done
fi
cd "`dirname "$SCRIPT_PATH"`" > /dev/null
SCRIPT_PATH=`pwd`;
popd > /dev/null

CPATH="$SCRIPT_PATH/@CMAKE_INSTALL_INCLUDEDIR@:${CPATH}"
export CPATH

LIBRARY_PATH="$SCRIPT_PATH/@CMAKE_INSTALL_LIBDIR@:${LIBRARY_PATH}"
export LIBRARY_PATH

DYLD_LIBRARY_PATH="$SCRIPT_PATH/@CMAKE_INSTALL_LIBDIR@:${DYLD_LIBRARY_PATH}"
export DYLD_LIBRARY_PATH
