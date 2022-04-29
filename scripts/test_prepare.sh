#!/bin/bash
DIR=$(pwd)
sed -i -E "s|(For build in directory: ).*|\1${DIR}/build|" build/CMakeCache.txt
sed -i -E "s|(embree4_BINARY_DIR:STATIC=).*|\1${DIR}/build|" build/CMakeCache.txt
sed -i -E "s|(embree4_SOURCE_DIR:STATIC=).*|\1${DIR}|" build/CMakeCache.txt
sed -i -E "s|(CMAKE_CACHEFILE_DIR:INTERNAL=).*|\1${DIR}/build|" build/CMakeCache.txt
sed -i -E "s|(CMAKE_HOME_DIRECTORY:INTERNAL=).*|\1${DIR}|" build/CMakeCache.txt
