// Copyright 2009-2021 Intel Corporation
// SPDX-License-Identifier: Apache-2.0

#include "../common/tutorial/tutorial_device.h"
#include "../common/math/random_sampler.h"
#include "../common/core/differential_geometry.h"
#include "../common/tutorial/scene_device.h"

namespace embree {

struct TutorialData
{
  /* scene data */
  RTCScene g_scene;
  RTCScene g_scene1;
  
  AffineSpace3fa* instance_xfm;
  LinearSpace3fa* normal_xfm;
  
  Vec3fa* colors;
};

#if __SYCL_COMPILER_VERSION >= 20210801
}
namespace sycl {
  template<> struct is_device_copyable<embree::TutorialData> : std::true_type {};
  template<> struct is_device_copyable<const embree::TutorialData> : std::true_type {};
}
namespace embree {
#endif

void TutorialData_Constructor(TutorialData* This)
{
  This->g_scene  = nullptr;
  This->g_scene1 = nullptr;
  This->instance_xfm = (AffineSpace3fa*) alignedUSMMalloc((4)*sizeof(AffineSpace3fa),16);
  This->normal_xfm = (LinearSpace3fa*) alignedUSMMalloc((4)*sizeof(LinearSpace3fa),16);
  This->colors = (Vec3fa*) alignedUSMMalloc((4*4)*sizeof(Vec3fa),16);
}

void TutorialData_Destructor(TutorialData* This)
{
  rtcReleaseScene (This->g_scene); This->g_scene = nullptr;
  rtcReleaseScene (This->g_scene1); This->g_scene1 = nullptr;
  alignedUSMFree(This->instance_xfm); This->instance_xfm = nullptr;
  alignedUSMFree(This->normal_xfm); This->normal_xfm = nullptr;
  alignedUSMFree(This->colors); This->colors = nullptr;
}

} // namespace embree
