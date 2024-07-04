#!/usr/bin/env bash

if [ -z $3 ]; then
  echo "ERROR: usage $0 version target-directory platform <GITHUB_TOKEN>"
  echo "    platform: either WIN or LINUX"
  echo "    <GITHUB_TOKEN>: optional github access token. without the token api request rate limit can be exceeded quickly"
  exit 1
fi

if [ -z $4 ]; then
  DPCPP_VERSION=$1
  DST_DIR=$2
  PLATFORM=$3
  TOKEN=""
else
  DPCPP_VERSION=$1
  DST_DIR=$2
  PLATFORM=$3
  TOKEN=$4
fi

CUR_DIR=$(pwd)
mkdir -p ${DST_DIR}
cd ${DST_DIR}

if [ -d "${DPCPP_VERSION}" ]; then
  echo "DPCPP compiler ${DPCPP_VERSION} already downloaded"
  exit 0
fi

echo "Downloading DPCPP compiler with"
echo "  DPCPP_VERSION: ${DPCPP_VERSION}"
echo "  DST_DIR: ${DST_DIR}"
echo "  PLATFORM: ${PLATFORM}"
echo "  TOKEN: ${TOKEN}"

GH_REPO="https://api.github.com/repos/intel/llvm"
GH_TAGS="${GH_REPO}/releases/tags/${DPCPP_VERSION}"
AUTH="-u ${TOKEN}:x-oauth-basic"

# get asset tags.
response=$(curl ${AUTH} ${GH_TAGS})

# check for errors wrong URL or API request limit exceeded are the most common
error=$(echo $response | grep message)
if [ ! -z "${error}" ]; then
  echo "ERROR:"
  echo ${error}
  exit 1
fi

ARCHIVE_NAME="sycl_linux.tar.gz"
if [ "$PLATFORM" = "WIN" ]; then
  ARCHIVE_NAME="sycl_windows.tar.gz"
fi

# get id of compiler archive
eval $(echo "${response}" | grep -C3 "name.:.\+${ARCHIVE_NAME}" | grep -w id | tr : = | tr -cd '[[:alnum:]]=')
[ "$id" ] || { echo "Error: Failed to get asset id, response: ${response}" | awk 'length($0)<100' >&2; exit 1; }

# Download asset file.
echo "Downloading dpcpp compiler archive ${ARCHIVE_NAME} to ${DST_DIR}" >&2
curl -LJO# ${AUTH} -H 'Accept: application/octet-stream' "$GH_REPO/releases/assets/$id"

echo "Unpacking archive"
mkdir -p ${DPCPP_VERSION}
mv ${ARCHIVE_NAME} ${DPCPP_VERSION}
cd ${DPCPP_VERSION}
tar -xzf ${ARCHIVE_NAME}
rm -rf ${ARCHIVE_NAME}

# add startup script to Windows version of the compiler
if [ "$PLATFORM" = "WIN" ]; then
echo '@echo off
set "DPCPP_DIR=%~dp0"
set "PATH=%DPCPP_DIR%\bin;%PATH%"
set "PATH=%DPCPP_DIR%\lib;%PATH%"
set "CPATH=%DPCPP_DIR%\include;%CPATH%"
set "INCLUDE=%DPCPP_DIR%\include;%INCLUDE%"
set "LIB=%DPCPP_DIR%\lib;%LIB%"' > ${DST_DIR}/${DPCPP_VERSION}/startup.bat
fi


echo "dpcpp compiler available here ${DST_DIR}/${DPCPP_VERSION}"

cd ${CUR_DIR}
