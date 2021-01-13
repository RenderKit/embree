// Copyright 2009-2021 Intel Corporation
// SPDX-License-Identifier: Apache-2.0

#include "../common/tutorial/tutorial_device.h"

namespace embree {

struct TutorialData
{
  RTCScene g_scene;
  Vec3fa* colors;
};

inline void TutorialData_Constructor(TutorialData* This)
{
  This->g_scene = nullptr;
  This->colors = nullptr;
}

inline void TutorialData_Destructor(TutorialData* This)
{
  rtcReleaseScene (This->g_scene); This->g_scene = nullptr;
  alignedFree(This->colors); This->colors = nullptr;
}

} // namespace embree
