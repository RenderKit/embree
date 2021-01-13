// Copyright 2009-2021 Intel Corporation
// SPDX-License-Identifier: Apache-2.0

#include "../common/tutorial/tutorial_device.h"
#include "../common/tutorial/scene_device.h"

namespace embree {

struct GridMesh
{
  RTCGeometry geom;
  RTCGeometry geomNormals;
  RTCGrid* egrids;
  Vec3fa* vertices;
  Vec3fa* normals;
};

struct TutorialData
{
  /* scene data */
  RTCScene g_scene;
  GridMesh gmesh;
};

void TutorialData_Constructor(TutorialData* This)
{
  This->g_scene  = nullptr;
  This->gmesh.geom = nullptr;
  This->gmesh.geomNormals = nullptr;
  This->gmesh.egrids = nullptr;
  This->gmesh.vertices = nullptr;
  This->gmesh.normals = nullptr;
}

void TutorialData_Destructor(TutorialData* This)
{
  alignedFree(This->gmesh.normals);
  rtcReleaseGeometry(This->gmesh.geom);
  rtcReleaseGeometry(This->gmesh.geomNormals);
  rtcReleaseScene (This->g_scene); This->g_scene = nullptr;
}

} // namespace embree
