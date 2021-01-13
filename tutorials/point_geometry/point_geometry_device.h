// Copyright 2009-2021 Intel Corporation
// SPDX-License-Identifier: Apache-2.0

#include "../common/tutorial/tutorial_device.h"
#include "../common/tutorial/optics.h"
#include "../common/math/random_sampler.h"

namespace embree {

#define NUM_POINTS 512
  
struct TutorialData
{
  RTCScene g_scene;
  Vec3fa* point_colors;
};

inline void TutorialData_Constructor(TutorialData* This)
{
  This->g_scene = nullptr;
  This->point_colors = (Vec3fa*) alignedMalloc(NUM_POINTS*sizeof(Vec3fa),16);
}

inline void TutorialData_Destructor(TutorialData* This)
{
  rtcReleaseScene (This->g_scene); This->g_scene = nullptr;
  alignedFree(This->point_colors); This->point_colors = nullptr;
}

} // namespace embree
