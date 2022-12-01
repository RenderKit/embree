// Copyright 2009-2021 Intel Corporation
// SPDX-License-Identifier: Apache-2.0

#include "../common/tutorial/tutorial_device.h"
#include "../common/math/random_sampler.h"
#include "../common/core/differential_geometry.h"
#include "../common/tutorial/scene_device.h"

namespace embree {

#define NUM_SPHERE_INSTANCES_PHI   1000
#define NUM_SPHERE_INSTANCES_THETA 1000  
#define NUM_SPHERE_INSTANCES (NUM_SPHERE_INSTANCES_PHI*NUM_SPHERE_INSTANCES_THETA)
  
#define NUM_SPHERES (10*10)
  
struct TutorialData
{
  /* scene data */
  RTCScene g_scene;
  RTCScene *g_scene_spheres;
  RTCGeometry *g_instances;

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
  This->g_scene_spheres = (RTCScene*)alignedUSMMalloc(NUM_SPHERES*sizeof(RTCScene),64);
  This->g_instances = (RTCGeometry*)alignedUSMMalloc(NUM_SPHERE_INSTANCES*sizeof(RTCGeometry),64);
}

void TutorialData_Destructor(TutorialData* This)
{
  rtcReleaseScene (This->g_scene); This->g_scene = nullptr;
  if (This->g_scene_spheres)
  {
    for (uint i=0;i<NUM_SPHERES;i++)
    {
      rtcReleaseScene (This->g_scene_spheres[i]);
      This->g_scene_spheres[i] = nullptr;
    }
    alignedUSMFree(This->g_scene_spheres);
    This->g_scene_spheres = nullptr;    
  }
  if (This->g_instances)
  {
    alignedUSMFree(This->g_instances);
    This->g_instances = nullptr;
  }
}

} // namespace embree
