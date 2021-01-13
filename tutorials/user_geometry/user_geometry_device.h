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

struct Instance
{
  ALIGNED_STRUCT_(16)
  RTCGeometry geometry;
  RTCScene object;
  int userID;
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
  
  Vec3fa colors[5][4];
};

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
}

inline void TutorialData_Destructor(TutorialData* This)
{
  rtcReleaseScene (This->g_scene); This->g_scene = nullptr;
  rtcReleaseScene (This->g_scene0); This->g_scene0 = nullptr;
  rtcReleaseScene (This->g_scene1); This->g_scene1 = nullptr;
  rtcReleaseScene (This->g_scene2); This->g_scene2 = nullptr;
  rtcReleaseDevice(g_device); g_device = nullptr;
  alignedFree(This->g_spheres); This->g_spheres = nullptr;
  alignedFree(This->g_sphere0); This->g_sphere0 = nullptr;
  alignedFree(This->g_sphere1); This->g_sphere1 = nullptr;
}

} // namespace embree
