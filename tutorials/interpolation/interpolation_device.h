// Copyright 2009-2021 Intel Corporation
// SPDX-License-Identifier: Apache-2.0

#include "../common/tutorial/tutorial_device.h"

namespace embree {

struct TutorialData
{
  RTCScene scene;
};

inline void TutorialData_Constructor(TutorialData* This)
{
  This->scene = nullptr;
}

inline void TutorialData_Destructor(TutorialData* This)
{
  rtcReleaseScene (This->scene); This->scene = nullptr;
}

} // namespace embree
