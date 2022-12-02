// Copyright 2009-2021 Intel Corporation
// SPDX-License-Identifier: Apache-2.0

#include "../common/tutorial/tutorial_device.h"

namespace embree {

enum UserGeometryType
{
  USER_GEOMETRY_INSTANCE = 0,
  USER_GEOMETRY_SPHERE = 1
};

struct Sphere
{
  ALIGNED_STRUCT_(16)
  UserGeometryType type;
  Vec3fa p;                      //!< position of the sphere
  float r;                      //!< radius of the sphere
  RTCGeometry geometry;
  unsigned int geomID;
};

struct Instance
{
  ALIGNED_STRUCT_(16)
  UserGeometryType type;
  RTCGeometry geometry;
  RTCScene object;
  AffineSpace3fa local2world;
  AffineSpace3fa world2local;
  LinearSpace3fa normal2world;
  Vec3fa lower;
  Vec3fa upper;
};

struct TutorialData
{
  /* scene data */
  RTCScene g_scene;
  RTCScene g_scene0;
  RTCScene g_scene1;
  RTCScene g_scene2;
  Sphere* g_spheres;
  Sphere* g_sphere0;
  Sphere* g_sphere1;
  
  Instance* g_instance[4];
  
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

inline void TutorialData_Constructor(TutorialData* This)
{
  This->g_scene  = nullptr;
  This->g_scene0 = nullptr;
  This->g_scene1 = nullptr;
  This->g_scene2 = nullptr;
  This->g_spheres = nullptr;
  This->g_sphere0 = nullptr;
  This->g_sphere1 = nullptr;
  This->g_instance[0] = nullptr;
  This->g_instance[1] = nullptr;
  This->g_instance[2] = nullptr;
  This->g_instance[3] = nullptr;
  This->colors = (Vec3fa*) alignedUSMMalloc((5*4)*sizeof(Vec3fa),16);
}

inline void TutorialData_Destructor(TutorialData* This)
{
  rtcReleaseScene (This->g_scene); This->g_scene = nullptr;
  rtcReleaseScene (This->g_scene0); This->g_scene0 = nullptr;
  rtcReleaseScene (This->g_scene1); This->g_scene1 = nullptr;
  rtcReleaseScene (This->g_scene2); This->g_scene2 = nullptr;
  rtcReleaseDevice(g_device); g_device = nullptr;
  alignedUSMFree(This->g_spheres); This->g_spheres = nullptr;
  alignedUSMFree(This->g_sphere0); This->g_sphere0 = nullptr;
  alignedUSMFree(This->g_sphere1); This->g_sphere1 = nullptr;
  alignedUSMFree(This->colors); This->colors = nullptr;
}

} // namespace embree
