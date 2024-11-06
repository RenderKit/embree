// Copyright 2009-2021 Intel Corporation
// SPDX-License-Identifier: Apache-2.0

#include "../common/tutorial/tutorial_device.h"

namespace embree {

#define NUM_SPHERES 20
#define NUM_PHI 120
#define NUM_THETA 240

struct TutorialData
{
  int numSpheres;
  int numPhi;
  int numTheta;

  /* scene data */
  RTCScene g_scene;
  RTCTraversable g_traversable;

  Vec3fa* position;
  Vec3fa* colors;
  float* radius;
  // int disabledID = -1;
};

void TutorialData_Constructor(TutorialData* This)
{
  This->numSpheres = NUM_SPHERES;
  This->numPhi = NUM_PHI;
  This->numTheta = NUM_THETA;
  This->g_scene  = nullptr;
  This->g_traversable  = nullptr;
  This->position = (Vec3fa*) alignedUSMMalloc((NUM_SPHERES)*sizeof(Vec3fa),16);
  This->colors = (Vec3fa*) alignedUSMMalloc((NUM_SPHERES+1)*sizeof(Vec3fa),16);
  This->radius = (float*) alignedUSMMalloc((NUM_SPHERES)*sizeof(float),16);
}

void TutorialData_Destructor(TutorialData* This)
{
  rtcReleaseScene (This->g_scene); This->g_scene = nullptr;
  alignedUSMFree(This->position); This->position = nullptr;
  alignedUSMFree(This->colors); This->colors = nullptr;
  alignedUSMFree(This->radius); This->radius = nullptr;
}


} // namespace embree
