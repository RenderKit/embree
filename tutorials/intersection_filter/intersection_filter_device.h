// Copyright 2009-2021 Intel Corporation
// SPDX-License-Identifier: Apache-2.0

#include "../common/tutorial/tutorial_device.h"

namespace embree {

struct TutorialData
{
  RTCScene g_scene;
  Vec3fa* colors;

  RTCFilterFunctionN intersectionFilter;
  RTCFilterFunctionN occlusionFilter;
};

inline void TutorialData_Constructor(TutorialData* This)
{
  This->g_scene = nullptr;
  This->colors = nullptr;
  This->intersectionFilter = nullptr;
  This->occlusionFilter = nullptr;
}

inline void TutorialData_Destructor(TutorialData* This)
{
  rtcReleaseScene (This->g_scene); This->g_scene = nullptr;
  alignedUSMFree(This->colors); This->colors = nullptr;
}

} // namespace embree
