#!/usr/bin/env bash
set -e

if [ -z $3 ]; then
  echo "ERROR: usage $0 ARTIFACTORY_TOKEN version target-directory (version: agama/VERSION_NUMBER, master/VERSION_NUMBER, master_release/VERSION_NUMBER, comp_igc/VERSION_NUMBER, neo/VERSION_NUMBER)"
  exit 1
fi

TOKEN=$1
VERSION=$2
DST_DIR=$3

GFX_TYPE=$(echo $VERSION | sed 's/\/.*//')
GFX_VERSION_NUMBER=$(echo $VERSION | sed 's/.*\///')

[ -d "${DST_DIR}/${VERSION}" ] && { echo "GFX version ${VERSION} already downloaded in ${DST_DIR}"; exit 0; }

URL=""
DEBS="level-zero|intel-level-zero-gpu|intel-opencl-icd|libigc1|libigc-tools|libigdfcl1|libigdgmm"

if [[ "${GFX_TYPE}" == "agama" ]]; then
  URL="https://gfx-assets-build.intel.com/artifactory/agama-builds/ci/prerelease/agama-ci-prerelease-${GFX_VERSION_NUMBER}/artifacts/linux/ubuntu/20.04/"
elif [[ "${GFX_TYPE}" == "master_release" ]]; then
  URL="https://gfx-assets-build.igk.intel.com/artifactory/gfx-driver-builds/ci/master/gfx-driver-ci-master-${GFX_VERSION_NUMBER}/artifacts/Linux/Ubuntu/20.04/Release/"
elif [[ "${GFX_TYPE}" == "master" ]]; then
  URL="https://gfx-assets-build.igk.intel.com/artifactory/gfx-driver-builds/ci/master/gfx-driver-ci-master-${GFX_VERSION_NUMBER}/artifacts/Linux/Ubuntu/20.04/ReleaseInternal/"
elif [[ "${GFX_TYPE}" == "comp_igc" ]]; then
  URL="https://gfx-assets-build.igk.intel.com/artifactory/gfx-driver-builds/ci/comp_igc/gfx-driver-ci-comp_igc-${GFX_VERSION_NUMBER}/artifacts/Linux/Ubuntu/20.04/ReleaseInternal/"
elif [[ "${GFX_TYPE}" == "neo" ]]; then
  URL="https://gfx-assets-build.intel.com/artifactory/neo-builds/ci/embargo/ci-neo-${GFX_VERSION_NUMBER}/artifacts/Linux/Ubuntu/20.04/Release/"
else
  echo "Error GFX version ${VERSION} not recognized because of type (only master, comp_igc, neo, and agama allowed)."
fi

CUR_DIR=$(pwd)

mkdir -p /tmp/${VERSION}
cd /tmp/${VERSION}

PACKAGES=$(curl --fail -H "X-JFrog-Art-Api:$TOKEN" -s $URL |
   sed 's/<a href/\n<a href/g' | sed 's/.deb"/.deb"\n/g' |
   grep -o '".*"' | grep deb | tr -d '"' | grep -E $DEBS |
   grep -v devel | grep -v dev | grep -v debuginfo)

for package in ${PACKAGES}
do
  echo "Download ${package}"
  #curl --silent --show-error --fail -H "X-JFrog-Art-Api:$TOKEN" -s -O ${URL}${package}
  curl -H "X-JFrog-Art-Api:${TOKEN}" -s -O ${URL}${package}
done

mkdir install
for p in *.deb
do
  echo "install $p"
  dpkg -x $p ./install
done

mkdir -p ${DST_DIR}/${VERSION}
cp -r * ${DST_DIR}/${VERSION}/

# fix path of libigdrcl in intel.icd file for local installation
echo "${DST_DIR}/${VERSION}/install/usr/lib/x86_64-linux-gnu/intel-opencl/libigdrcl.so" > ${DST_DIR}/${VERSION}/install/etc/OpenCL/vendors/intel.icd

echo "GFX $VERSION installed to ${DST_DIR}/${VERSION}/install"

cd ${CUR_DIR}

rm -rf /tmp/${VERSION}

