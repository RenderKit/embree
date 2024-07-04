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

int main() {

#if defined(EMBREE_SYCL_SUPPORT)
  try {
    embree::check_raytracing_support();
  } catch (std::exception& e) {
    std::cerr << e.what() << std::endl;
    return 1;
  }

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
