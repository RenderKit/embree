#!/usr/bin/env bash
set -e

if [ -z $3 ]; then
  echo "ERROR: usage $0 ARTIFACTORY_TOKEN version target-directory (version: master/VERSION_NUMBER, comp_igc/VERSION_NUMBER, or neo/VERSION_NUMBER)"
  exit 1
fi

TOKEN=$1
VERSION=$2
DST_DIR=$3

GFX_TYPE=$(echo $VERSION | sed 's/\/.*//')
GFX_VERSION_NUMBER=$(echo $VERSION | sed 's/.*\///')

[ -d "${DST_DIR}/${VERSION}" ] && { echo "GFX version ${VERSION} already downloaded in ${DST_DIR}"; exit 0; }

URL=""
DEBS="*"
if [[ "${GFX_TYPE}" == "master" ]]; then
  URL="https://gfx-assets-build.igk.intel.com/artifactory/gfx-driver-builds/ci/master/gfx-driver-ci-master-${GFX_VERSION_NUMBER}/artifacts/Linux/Ubuntu/20.04/ReleaseInternal/"
  DEBS="level-zero|intel-level-zero-gpu|intel-opencl-icd|libigc1|libigc-tools|libigdfcl1|libigdgmm"
elif [[ "${GFX_TYPE}" == "comp_igc" ]]; then
  URL="https://gfx-assets-build.igk.intel.com/artifactory/gfx-driver-builds/ci/comp_igc/gfx-driver-ci-comp_igc-${GFX_VERSION_NUMBER}/artifacts/Linux/Ubuntu/20.04/ReleaseInternal/"
  if [[ "${GFX_VERSION_NUMBER}" -gt "9423" ]]; then
    echo "gfx verison ${GFX_VERSION_NUMBER} is greater then 9423"
    DEBS="level-zero|intel-level-zero-gpu|intel-opencl-icd|libigc1|libigc-tools|libigdfcl1|libigdgmm"
  else
    echo "gfx verison ${GFX_VERSION_NUMBER} is not greater then 9423"
    DEBS="intel-gmmlib|intel-igc-core|intel-igc-opencl|intel-level-zero-gpu|intel-ocloc|intel-opencl"
  fi
elif [[ "${GFX_TYPE}" == "neo" ]]; then
  URL="https://gfx-assets-build.intel.com/artifactory/neo-builds/ci/embargo/ci-neo-${GFX_VERSION_NUMBER}/artifacts/Linux/Ubuntu/20.04/Release/"
  DEBS="intel-gmmlib|intel-igc-core|intel-igc-opencl|intel-level-zero-gpu|intel-ocloc|intel-opencl"
elif [[ "${GFX_TYPE}" == "neo" ]]; then
  URL="https://gfx-assets-build.intel.com/artifactory/neo-builds/ci/embargo/ci-neo-${GFX_VERSION_NUMBER}/artifacts/Linux/Ubuntu/20.04/Release/"
  DEBS="intel-gmmlib|intel-igc-core|intel-igc-opencl|intel-level-zero-gpu|intel-ocloc|intel-opencl"
else
  echo "Error GFX version ${VERSION} not recognized because of type (only master, igc_comp and neo allowed)."
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

mkdir -p ${DST_DIR}/${VERSION}
mv *.deb ${DST_DIR}/${VERSION}

cd ${DST_DIR}/${VERSION}

mkdir -p install

for p in *.deb
do
  echo "install $p"
  dpkg -x $p install
done

# fix path of libigdrcl in intel.icd file for local installation
echo "${DST_DIR}/${VERSION}/install/usr/lib/x86_64-linux-gnu/intel-opencl/libigdrcl.so" > ${DST_DIR}/${VERSION}/install/etc/OpenCL/vendors/intel.icd

echo "GFX $VERSION installed to ${DST_DIR}/${VERSION}/install"

cd ${CUR_DIR}

rm -rf /tmp/${VERSION}

