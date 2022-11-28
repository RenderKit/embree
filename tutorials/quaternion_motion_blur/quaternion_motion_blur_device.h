// Copyright 2009-2021 Intel Corporation
// SPDX-License-Identifier: Apache-2.0

#include "../common/tutorial/tutorial_device.h"

namespace embree {

struct Sphere
{
  ALIGNED_STRUCT_(16)
  Vec3fa p;                      //!< position of the sphere
  float r;                      //!< radius of the sphere
  RTCGeometry geometry;
  unsigned int geomID;
};

struct TutorialData
{  
  RTCScene g_scene;
  RTCScene g_scene0;
  Sphere* g_spheres;
  int g_spp;
  bool g_motion_blur;
  float g_time;
  float g_shutter_close;

  /* accumulation buffer */
  Vec3ff* g_accu;
  unsigned int g_accu_width;
  unsigned int g_accu_height;
  unsigned int g_accu_count;
  Vec3fa g_accu_vx;
  Vec3fa g_accu_vy;
  Vec3fa g_accu_vz;
  Vec3fa g_accu_p;
};

#if __SYCL_COMPILER_VERSION >= 20210801
}
namespace sycl {
  template<> struct is_device_copyable<embree::TutorialData> : std::true_type {};
  template<> struct is_device_copyable<const embree::TutorialData> : std::true_type {};
}
namespace embree {
#endif

inline void TutorialData_Constructor(TutorialData* This)
{
  This->g_scene = nullptr;
  This->g_scene0 = nullptr;
  This->g_spheres = nullptr;
  This->g_spp = 0;
  This->g_motion_blur = true;
  This->g_time = 0.f;
  This->g_shutter_close = 0.f;
  This->g_accu = nullptr;
  This->g_accu_width = 0;
  This->g_accu_height = 0;
  This->g_accu_count = 0;
  This->g_accu_vx = Vec3fa(0.0f);
  This->g_accu_vy = Vec3fa(0.0f);
  This->g_accu_vz = Vec3fa(0.0f);
  This->g_accu_p  = Vec3fa(0.0f);
}

inline void TutorialData_Destructor(TutorialData* This)
{
  rtcReleaseScene (This->g_scene); This->g_scene = nullptr;
  rtcReleaseScene (This->g_scene0); This->g_scene0 = nullptr;
  alignedUSMFree(This->g_spheres); This->g_spheres = nullptr;
  alignedUSMFree(This->g_accu); This-> g_accu = nullptr;

  This->g_accu_width = 0;
  This->g_accu_height = 0;
  This->g_accu_count = 0;
}

} // namespace embree
