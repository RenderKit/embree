// Copyright 2009-2021 Intel Corporation
// SPDX-License-Identifier: Apache-2.0

#include "../common/scene.h"
 
namespace embree
{
  void* rthwifInit(sycl::device device, sycl::context context);
  
  void rthwifCleanup(void* dispatchGlobalsPtr, sycl::context context);

  bool rthwifIsSYCLDeviceSupported(const sycl::device& sycl_device);
  
  BBox3f rthwifBuild(Scene* scene, RTCBuildQuality quality_flags, Device::avector<char,64>& buffer_o);
}
