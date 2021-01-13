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
  
  RTCGeometry g_instance0;
  RTCGeometry g_instance1;
  RTCGeometry g_instance2;
  RTCGeometry g_instance3;
  AffineSpace3fa instance_xfm[4];
  LinearSpace3fa normal_xfm[4];
  
  Vec3fa colors[4][4];
};

void TutorialData_Constructor(TutorialData* This)
{
  This->g_scene  = nullptr;
  This->g_scene1 = nullptr;
  This->g_instance0 = nullptr;
  This->g_instance1 = nullptr;
  This->g_instance2 = nullptr;
  This->g_instance3 = nullptr;
}

void TutorialData_Destructor(TutorialData* This)
{
  rtcReleaseScene (This->g_scene); This->g_scene = nullptr;
  rtcReleaseScene (This->g_scene1); This->g_scene1 = nullptr;
}

} // namespace embree
