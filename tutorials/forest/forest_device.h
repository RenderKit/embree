// Copyright 2009-2021 Intel Corporation
// SPDX-License-Identifier: Apache-2.0

#include "../common/tutorial/tutorial_device.h"

namespace embree {

struct TutorialData
{
  RTCScene g_scene;
  Triangle* tree_triangles[6];
  Vec3fa* tree_vertex_colors[6];
  Triangle* terrain_triangles;
  unsigned int trees_selected[6];
  unsigned int* tree_ids;
  AffineSpace3fa* tree_transforms;
  bool use_instance_array;
  int spp;
};

inline void TutorialData_Constructor(TutorialData* This)
{
  This->g_scene = nullptr;
  This->tree_ids = nullptr;
  This->tree_transforms = nullptr;
  This->terrain_triangles = nullptr;
}

inline void TutorialData_Destructor(TutorialData* This)
{
  rtcReleaseScene (This->g_scene); This->g_scene = nullptr;

  if(This->tree_ids) alignedUSMFree(This->tree_ids);
  if(This->tree_transforms) alignedUSMFree(This->tree_transforms);
}

} // namespace embree
