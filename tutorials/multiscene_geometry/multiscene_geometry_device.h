// Copyright 2009-2021 Intel Corporation
// SPDX-License-Identifier: Apache-2.0

#include "../common/tutorial/tutorial_device.h"

namespace embree {

#define NUM_SPHERES 20
#define NUM_PHI 120
#define NUM_THETA 240


struct TutorialData
{
  int numSpheres = NUM_SPHERES;
  int numPhi = NUM_PHI;
  int numTheta = NUM_THETA;

  
  /* scene data */
  RTCScene  g_scene_0;
  RTCScene  g_scene_1;
  RTCScene  g_scene_2;
  RTCScene  g_curr_scene;
  RTCTraversable g_traversable;
  Vec3fa* position;
  Vec3fa* colors0;
  Vec3fa* colors1;
  Vec3fa* colors2;
  Vec3fa* colors;
  float* radius;
  int disabledID = -1;
  int g_scene_id = 0;
};

inline void TutorialData_Constructor(TutorialData* This)
{
  This->g_scene_0 = nullptr;
  This->g_scene_1 = nullptr;
  This->g_scene_2 = nullptr;
  This->g_curr_scene = nullptr;
  This->g_traversable = nullptr;
  This->position = (Vec3fa*) alignedUSMMalloc(NUM_SPHERES*sizeof(Vec3fa),16);
  This->colors = nullptr;
  This->colors0 = (Vec3fa*) alignedUSMMalloc((NUM_SPHERES+1)*sizeof(Vec3fa),16);
  This->colors1 = (Vec3fa*) alignedUSMMalloc((NUM_SPHERES/2+1)*sizeof(Vec3fa),16);
  This->colors2 = (Vec3fa*) alignedUSMMalloc((NUM_SPHERES/2+1)*sizeof(Vec3fa),16);
  This->radius = (float*) alignedUSMMalloc((NUM_SPHERES)*sizeof(float),16);
}

inline void TutorialData_Destructor(TutorialData* This)
{
  rtcReleaseScene (This->g_scene_0); This->g_scene_0 = nullptr;
  rtcReleaseScene (This->g_scene_1); This->g_scene_1 = nullptr;
  rtcReleaseScene (This->g_scene_2); This->g_scene_2 = nullptr;
  alignedUSMFree(This->position); This->position = nullptr;
  alignedUSMFree(This->colors0); This->colors = nullptr;
  alignedUSMFree(This->colors1); This->colors = nullptr;
  alignedUSMFree(This->colors2); This->colors = nullptr;
  alignedUSMFree(This->radius); This->radius = nullptr;

}

} // namespace embree
