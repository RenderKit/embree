// Copyright 2009-2021 Intel Corporation
// SPDX-License-Identifier: Apache-2.0

#if defined(EMBREE_SYCL_SUPPORT)
#include <sycl/sycl.hpp>
#endif
#include <embree4/rtcore.h>

#if defined(EMBREE_SYCL_SUPPORT)
#include "../common/sycl/util.h"
#endif

RTC_NAMESPACE_USE

#include <xmmintrin.h>
//#include <pmmintrin.h> // use this to get _MM_SET_DENORMALS_ZERO_MODE when compiling for SSE3 or higher

#if !defined(_MM_SET_DENORMALS_ZERO_MODE)
#define _MM_DENORMALS_ZERO_ON   (0x0040)
#define _MM_DENORMALS_ZERO_OFF  (0x0000)
#define _MM_DENORMALS_ZERO_MASK (0x0040)
#define _MM_SET_DENORMALS_ZERO_MODE(x) (_mm_setcsr((_mm_getcsr() & ~_MM_DENORMALS_ZERO_MASK) | (x)))
#endif

int main() {
  /* for best performance set FTZ and DAZ flags in MXCSR control and status register */
  _MM_SET_FLUSH_ZERO_MODE(_MM_FLUSH_ZERO_ON);
  _MM_SET_DENORMALS_ZERO_MODE(_MM_DENORMALS_ZERO_ON);

#if defined(EMBREE_SYCL_SUPPORT)
  try {
    sycl::device sycl_device(rtcSYCLDeviceSelector);
    sycl::context sycl_context(sycl_device);
    RTCDevice device = rtcNewSYCLDevice(sycl_context, "verbose=1");
    embree::printAllSYCLDevices();
    rtcReleaseDevice(device);
  } catch (std::exception& e) {
    std::cerr << "Failed to create a SYCL Embree GPU device. Reason: " << e.what() << std::endl;
    embree::printAllSYCLDevices();
    return 1;
  }
#else
  RTCDevice device = rtcNewDevice("verbose=1");
  rtcReleaseDevice(device);
#endif
  
  return 0;
}
