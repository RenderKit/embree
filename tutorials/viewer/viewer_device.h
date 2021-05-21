// Copyright 2009-2021 Intel Corporation
// SPDX-License-Identifier: Apache-2.0

#include "../common/tutorial/tutorial_device.h"
#include "../common/math/random_sampler.h"
#include "../common/core/differential_geometry.h"
#include "../common/tutorial/scene_device.h"

namespace embree {

extern "C" ISPCScene* g_ispc_scene;
extern "C" int g_instancing_mode;
extern "C" float g_min_width;
extern "C" int g_animation_mode;

struct TutorialData
{
  ISPCScene* ispc_scene;
  int instancing_mode;
  RTCIntersectContextFlags iflags_coherent;
  
  /* scene data */
  RTCScene scene;
  bool subdiv_mode;

  float min_width;
};

void TutorialData_Constructor(TutorialData* This)
{
  This->ispc_scene = g_ispc_scene;
  This->instancing_mode = g_instancing_mode;
  This->iflags_coherent = g_iflags_coherent;
  This->scene = nullptr;
  This->subdiv_mode = false;
  This->min_width = g_min_width;
}

void TutorialData_Destructor(TutorialData* This)
{
  rtcReleaseScene (This->scene); This->scene = nullptr;
}

} // namespace embree
