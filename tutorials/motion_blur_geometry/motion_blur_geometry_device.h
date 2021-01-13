// Copyright 2009-2021 Intel Corporation
// SPDX-License-Identifier: Apache-2.0

#include "../common/tutorial/tutorial_device.h"

namespace embree {

struct Sphere
{
  ALIGNED_STRUCT_(16)
  Vec3fa p;                      //!< position of the sphere
  float r;                      //!< radius of the sphere
  unsigned int geomID;
  unsigned int num_time_steps;
};

struct TutorialData
{
  RTCScene g_scene;
  Vec3fa face_colors[12];
  float g_time;
  
  /* accumulation buffer */
  Vec3ff* g_accu;
  unsigned int g_accu_width;
  unsigned int g_accu_height;
  unsigned int g_accu_count;
  Vec3fa g_accu_vx;
  Vec3fa g_accu_vy;
  Vec3fa g_accu_vz;
  Vec3fa g_accu_p;

  RTCScene scene0;
  RTCScene scene1;
  RTCScene scene2;
  RTCScene scene3;
  Sphere* sphere0;
  Sphere* sphere1;
};

inline void TutorialData_Constructor(TutorialData* This)
{
  This->g_scene = nullptr;
  This->g_time = 0;
  This->g_accu = nullptr;
  This->g_accu_width = 0;
  This->g_accu_height = 0;
  This->g_accu_count = 0;
  This->g_accu_vx = Vec3fa(0.0f);
  This->g_accu_vy = Vec3fa(0.0f);
  This->g_accu_vz = Vec3fa(0.0f);
  This->g_accu_p  = Vec3fa(0.0f);

  This->scene0 = nullptr;
  This->scene1 = nullptr;
  This->scene2 = nullptr;
  This->scene3 = nullptr;
  This->sphere0 = nullptr;
  This->sphere1 = nullptr;
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
