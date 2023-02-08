#!/usr/bin/env bash

## Copyright 2009-2021 Intel Corporation
## SPDX-License-Identifier: Apache-2.0

if [ -z $2 ]; then
  echo "ERROR: usage $0 dpcpp_dir igc_install_dir"
  exit 1
fi

DPCPP_DIR=$1
IGC_DIR=$2

# FIXME: for whatever reason this is still needed for the official compute runtime
# otherwise some sycl_test test fail.
export PATH=${DPCPP_DIR}/bin:${IGC_DIR}/usr/bin:${IGC_DIR}/usr/local/bin:${PATH}
export CPATH=${DPCPP_DIR}/include/sycl:${DPCPP_DIR}/include
export LIBRARY_PATH=${DPCPP_DIR}/lib:${LIBRARY_PATH}
export LD_LIBRARY_PATH=${DPCPP_DIR}/lib:${IGC_DIR}/usr/lib/x86_64-linux-gnu:${IGC_DIR}/usr/local/lib:${LD_LIBRARY_PATH}

# FIXME: this might be unnecessary because we directly tell libOpenCL which .ibigdrcl to load using OCL_ICD_VENDORS.
# However, some IGC versions on the NAS don't have a properly modified intel.icd yet.
export OCL_ICD_FILENAMES=${IGC_DIR}/usr/lib/x86_64-linux-gnu/intel-opencl/libigdrcl.so:${IGC_DIR}/usr/local/lib/intel-opencl/libigdrcl.so

export OCL_ICD_VENDORS=${IGC_DIR}/etc/OpenCL/vendors/intel.icd

export CXX=${DPCPP_DIR}/bin/clang++
export CC=${DPCPP_DIR}/bin/clang