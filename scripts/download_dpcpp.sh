#!/usr/bin/env bash

if [ -z $2 ]; then
  echo "ERROR: usage $0 <GITHUB_TOKEN> version target-directory (github token is optional but without the api request rate limit can be exceeded quickly)"
  exit 1
fi

if [ -z $3 ]; then
  TOKEN=""
  DPCPP_VERSION=$1
  DST_DIR=$2
else
  TOKEN=$1
  DPCPP_VERSION=$2
  DST_DIR=$3
fi

CUR_DIR=$(pwd)
cd ${DST_DIR}

[ -d "${DPCPP_VERSION}/dpcpp_compiler" ] && { echo "DPCPP compiler ${DPCPP_VERSION} already downloaded"; exit 0; }

GH_REPO="https://api.github.com/repos/intel/llvm"
GH_TAGS="${GH_REPO}/releases/tags/${DPCPP_VERSION}"
AUTH="-u ${TOKEN}:x-oauth-basic"

# get asset tags.
response=$(curl -s ${AUTH} ${GH_TAGS})

# check for errors wrong URL or API request limit exceeded are the most common
error=$(echo $response | grep message)
if [ ! -z "${error}" ]; then
  echo "ERROR:"
  echo ${error}
  exit 1
fi

# get id of compiler archive
eval $(echo "${response}" | grep -C3 "name.:.\+dpcpp-compiler.tar.gz" | grep -w id | tr : = | tr -cd '[[:alnum:]]=')
[ "$id" ] || { echo "Error: Failed to get asset id, response: ${response}" | awk 'length($0)<100' >&2; exit 1; }

# Download asset file.
echo "Downloading dpcpp compiler to ${DST_DIR}" >&2
curl -LJO# ${AUTH} -H 'Accept: application/octet-stream' "$GH_REPO/releases/assets/$id"

echo "Unpacking archive"
tar -xzf dpcpp-compiler.tar.gz
rm -rf dpcpp-compiler.tar.gz
mkdir -p ${DPCPP_VERSION}
mv dpcpp_compiler ${DPCPP_VERSION}

cd ${DPCPP_VERSION}/dpcpp_compiler
sed -i '/fpgavars/d' startup.sh \

echo "dpcpp compiler available here ${DST_DIR}/${DPCPP_VERSION}/dpcpp_compiler"

cd ${CUR_DIR}