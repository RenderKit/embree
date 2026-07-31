// Copyright 2009-2021 Intel Corporation
// SPDX-License-Identifier: Apache-2.0

#include "../common/tutorial/tutorial_device.h"
#include "../common/math/linearspace.h"
#include "../common/math/random_sampler.h"
#include "../common/math/sampling.h"
#include "../../include/embree4/rtcore.h"
//#include "scene.h"

namespace embree {

struct InstanceLevels
{
  unsigned int numLevels;
  unsigned int* numInstancesOnLevel;
  LinearSpace3fa* * normalTransforms;
};

struct TutorialData
{
  RTCScene g_scene;
  RTCTraversable g_traversable;
  InstanceLevels g_instanceLevels;

  /* accumulation buffer */
  Vec3ff* g_accu;
  unsigned int g_accu_width;
  unsigned int g_accu_height;
  unsigned int g_accu_count;
  Vec3fa g_accu_vx;
  Vec3fa g_accu_vy;
  Vec3fa g_accu_vz;
  Vec3fa g_accu_p;

  LinearSpace3fa** g_normalTransforms;
};

#if __SYCL_COMPILER_VERSION >= 20210801
}
namespace sycl {
  template<> struct is_device_copyable<embree::TutorialData> : std::true_type {};
  template<> struct is_device_copyable<const embree::TutorialData> : std::true_type {};
}
namespace embree {
#endif

extern "C" RTCScene initializeScene(TutorialData& data, RTCDevice device);

extern "C" void cleanupScene(TutorialData& data);

inline void TutorialData_Constructor(TutorialData* This)
{
  This->g_scene = nullptr;
  This->g_traversable = nullptr;
  This->g_accu = nullptr;
  This->g_accu_width = 0;
  This->g_accu_height = 0;
  This->g_accu_count = 0;
  This->g_accu_vx = Vec3fa(0.0f);
  This->g_accu_vy = Vec3fa(0.0f);
  This->g_accu_vz = Vec3fa(0.0f);
  This->g_accu_p  = Vec3fa(0.0f);
  This->g_normalTransforms = nullptr;
  This->g_instanceLevels.numLevels = 0;
  This->g_instanceLevels.numInstancesOnLevel = nullptr;
  This->g_instanceLevels.normalTransforms = nullptr;
}

inline void TutorialData_Destructor(TutorialData* This)
{
  rtcReleaseScene (This->g_scene); This->g_scene = nullptr;
  cleanupScene(*This);
  alignedUSMFree(This->g_accu); This->g_accu = nullptr;
  This->g_accu_width = 0;
  This->g_accu_height = 0;
  This->g_accu_count = 0;
}

} // namespace embree
