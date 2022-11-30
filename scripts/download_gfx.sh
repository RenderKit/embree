#!/usr/bin/env bash
set -e

if [ -z $3 ]; then
  echo "ERROR: usage $0 gfx-driver target-directory token"
  echo "    gfx-driver: URL to the driver packages relative to artifactory URL (e.g. gfx-driver-builds/ci/master/gfx-driver-ci-master-9999/artifacts/Linux/Ubuntu/22.04/Release"
  echo "    token: access token to (artifactory) repository"
  exit 1
fi

ARTIFACTORY_URL="https://gfx-assets-build.igk.intel.com/artifactory"
GFX_DRIVER=$1
DST_DIR=$2
TOKEN=$3

if [[ -d "${DST_DIR}/${GFX_DRIVER}" ]]; then
  echo "GFX driver ${GFX_DRIVER} already downloaded in ${DST_DIR}"
  exit 0
fi

URL=${ARTIFACTORY_URL}"/"${GFX_DRIVER}/

CUR_DIR=$(pwd)
mkdir -p /tmp/${GFX_DRIVER}
cd /tmp/${GFX_DRIVER}

if [[ $GFX_DRIVER == *"linux"* ]] || [[ $GFX_DRIVER == *"Linux"* ]] ; then
  echo "Downloading Linux driver packages using"
  echo "  ARTIFACTORY_URL=${ARTIFACTORY_URL}"
  echo "  GFX_DRIVER=${GFX_DRIVER}"
  echo "  DST_DIR=${DST_DIR}"
  echo "  TOKEN=${TOKEN}"
  
  if [[ $GFX_DRIVER == *"open-linux"* ]] ; then
    DEBS="*"
  elif [[ $GFX_DRIVER == *"neo-releases"* ]] ; then
      DEBS="*"
  elif [[ $GFX_DRIVER == *"neo-master"* ]] ; then
      DEBS="*"
  else
    DEBS="level-zero|intel-level-zero-gpu|intel-opencl-icd|libigc1|libigc-tools|libigdfcl1|libigdgmm"
  fi
      
  PACKAGES=$(curl -H "X-JFrog-Art-Api:$TOKEN" -s $URL |
     sed 's/<a href/\n<a href/g' | sed 's/.deb"/.deb"\n/g' |
     grep -o '".*"' | grep deb | tr -d '"' | grep -E $DEBS |
     grep -v devel | grep -v dev | grep -v debuginfo)

  mkdir -p install
  for package in ${PACKAGES}
  do
    echo "Download and install ${package}"
    curl -H "X-JFrog-Art-Api:${TOKEN}" -s -O ${URL}${package}
    dpkg -x $package ./install
  done

  mkdir -p ${DST_DIR}/${GFX_DRIVER}
  cp -r * ${DST_DIR}/${GFX_DRIVER}/

  # fix path of libigdrcl in intel.icd file for local installation
  echo "${DST_DIR}/${GFX_DRIVER}/install/usr/lib/x86_64-linux-gnu/intel-opencl/libigdrcl.so" > ${DST_DIR}/${GFX_DRIVER}/install/etc/OpenCL/vendors/intel.icd

  echo "Driver installed to ${DST_DIR}/${GFX_DRIVER}/install"

elif [[ $GFX_DRIVER == *"Windows"* ]]; then 
  echo "Downloading Windows driver packages using"
  echo "  ARTIFACTORY_URL=${ARTIFACTORY_URL}"
  echo "  GFX_DRIVER=${GFX_DRIVER}"
  echo "  DST_DIR=${DST_DIR}"
  echo "  TOKEN=${TOKEN}"

  echo "Download Installer-Release"
  curl -H "X-JFrog-Art-Api:${TOKEN}" -s -O ${URL}Installer-Release-64-bit.7z
  echo "Download Ocloc-Release"
  curl -H "X-JFrog-Art-Api:${TOKEN}" -s -O ${URL}Ocloc-Release-64-bit.7z

  echo "Unpacking Installer-Release"
  7z x Installer-Release-64-bit.7z
  echo "Unpacking Ocloc-Release"
  7z x Ocloc-Release-64-bit.7z

  rm Installer-Release-64-bit.7z Ocloc-Release-64-bit.7z
  mkdir -p ${DST_DIR}/${GFX_DRIVER}
  cp -r * ${DST_DIR}/${GFX_DRIVER}

  chmod -R +x ${DST_DIR}/${GFX_DRIVER}

  echo "Driver installed to ${DST_DIR}/${GFX_DRIVER}"
else
  echo "Platform could not be detected from gfx-driver URL"
fi

cd ${CUR_DIR}
rm -rf /tmp/${GFX_DRIVER}


