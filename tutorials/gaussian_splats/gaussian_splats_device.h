// Copyright 2009-2021 Intel Corporation
// SPDX-License-Identifier: Apache-2.0

#include "../common/tutorial/tutorial_device.h"
#include "../common/tutorial/optics.h"
#include "../common/math/random_sampler.h"

namespace embree {

#define NUM_SPLATS 1024

struct GaussianSplat
{
  Vec3fa center;
  Vec3fa axisU;
  Vec3fa axisV;
  float sigmaU;
  float sigmaV;
  float opacity;
  unsigned int colorID;
};

struct TutorialData
{
  RTCScene g_scene;
  RTCTraversable g_traversable;
  GaussianSplat* splats;
  Vec3fa* colors;
};

inline void TutorialData_Constructor(TutorialData* This)
{
  This->g_scene = nullptr;
  This->g_traversable = nullptr;
  This->splats = (GaussianSplat*) alignedUSMMalloc(NUM_SPLATS * sizeof(GaussianSplat), 16);
  This->colors = (Vec3fa*) alignedUSMMalloc(NUM_SPLATS * sizeof(Vec3fa), 16);
}

inline void TutorialData_Destructor(TutorialData* This)
{
  rtcReleaseScene(This->g_scene);
  This->g_scene = nullptr;
  alignedUSMFree(This->splats);
  This->splats = nullptr;
  alignedUSMFree(This->colors);
  This->colors = nullptr;
}

} // namespace embree
