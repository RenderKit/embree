// Copyright 2009-2021 Intel Corporation
// SPDX-License-Identifier: Apache-2.0

#include "../common/tutorial/tutorial_device.h"

namespace embree {

struct TutorialData
{
  RTCScene g_scene;
  RTCTraversable g_traversable;
  Vec3fa* face_colors;
  Vec3fa* vertex_colors;
  bool enable_ray_mask;
};

inline void TutorialData_Constructor(TutorialData* This)
{
  This->g_scene = nullptr;
  This->g_traversable = nullptr;
  This->face_colors = nullptr;
  This->vertex_colors = nullptr;
}

inline void TutorialData_Destructor(TutorialData* This)
{
  rtcReleaseScene (This->g_scene); This->g_scene = nullptr;
  alignedUSMFree(This->face_colors); This->face_colors = nullptr;
  alignedUSMFree(This->vertex_colors); This->vertex_colors = nullptr;
}

} // namespace embree
