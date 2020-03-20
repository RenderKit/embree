// Copyright 2009-2020 Intel Corporation
// SPDX-License-Identifier: Apache-2.0

#include "../common/tutorial/tutorial_device.h"

namespace embree {

struct TutorialData
{
  RTCScene g_scene = nullptr;
  Vec3fa face_colors[12];
  
  /* accumulation buffer */
  Vec3fa* g_accu = nullptr;
  unsigned int g_accu_width;
  unsigned int g_accu_height;
  unsigned int g_accu_count;
  Vec3fa g_accu_vx;
  Vec3fa g_accu_vy;
  Vec3fa g_accu_vz;
  Vec3fa g_accu_p;
  
  extern "C" float g_time;
  extern "C" unsigned int g_num_time_steps;
  extern "C" unsigned int g_num_time_steps2;
};

inline void TutorialData_Constructor(TutorialData* This)
{
  This->g_scene = nullptr;
  This->g_accu = nullptr;
  This->g_accu_width = 0;
  This->g_accu_height = 0;
  This->g_accu_count = 0;
  This->g_accu_vx = Vec3fa(0.0f);
  This->g_accu_vy = Vec3fa(0.0f);
  This->g_accu_vz = Vec3fa(0.0f);
  This->g_accu_p  = Vec3fa(0.0f);
  This->g_time = -1;
  This->g_num_time_steps = 8;
  This->g_num_time_steps2 = 30;
}

inline void TutorialData_Destructor(TutorialData* This)
{
  alignedFree(This->sphere0); This->sphere0 = nullptr;
  alignedFree(This->sphere1); This->sphere1 = nullptr;
  rtcReleaseScene(This->scene0); This->scene0 = nullptr;
  rtcReleaseScene(This->scene1); This->scene1 = nullptr;
  rtcReleaseScene(This->scene2); This->scene2 = nullptr;
  rtcReleaseScene(This->scene3); This->scene3 = nullptr;
  rtcReleaseScene (This->g_scene); This->g_scene = nullptr;
  alignedFree(This->g_accu); This->g_accu = nullptr;
  This->g_accu_width = 0;
  This->g_accu_height = 0;
  This->g_accu_count = 0;
}

} // namespace embree
