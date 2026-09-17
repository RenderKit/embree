// Copyright 2009-2021 Intel Corporation
// SPDX-License-Identifier: Apache-2.0

#include "../common/tutorial/tutorial_device.h"
#include "../common/tutorial/optics.h"
#include "../common/math/random_sampler.h"

namespace embree {

static constexpr unsigned int DEFAULT_NUM_SPLATS = 1024;

struct GaussianSplat
{
  Vec3fa center;
  Vec3fa scale;
  Vec4f rotation;
  float opacity;
  unsigned int colorID;
};

struct TutorialData
{
  RTCScene g_scene;
  RTCTraversable g_traversable;
  GaussianSplat* splats;
  Vec3fa* colors;
  unsigned int splatCount;
};

inline void TutorialData_Constructor(TutorialData* This)
{
  This->g_scene = nullptr;
  This->g_traversable = nullptr;
  This->splats = nullptr;
  This->colors = nullptr;
  This->splatCount = 0;
}

inline void TutorialData_ResizeSplats(TutorialData* This, unsigned int splatCount)
{
  alignedUSMFree(This->splats);
  alignedUSMFree(This->colors);

  This->splats = (GaussianSplat*) alignedUSMMalloc(splatCount * sizeof(GaussianSplat), 16);
  This->colors = (Vec3fa*) alignedUSMMalloc(splatCount * sizeof(Vec3fa), 16);
  This->splatCount = splatCount;
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
