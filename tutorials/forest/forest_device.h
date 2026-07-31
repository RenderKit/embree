// Copyright 2009-2021 Intel Corporation
// SPDX-License-Identifier: Apache-2.0

#include "../common/tutorial/tutorial_device.h"

namespace embree {

struct TutorialData
{
  RTCScene g_scene;
  RTCTraversable g_traversable;
  Triangle* tree_triangles[6];
  Vec3f* tree_vertex_colors[6];
  Triangle* terrain_triangles;
  unsigned int trees_selected[6];

  unsigned int* tree_ids_host;
  unsigned int* tree_ids_device;
  AffineSpace3fa* tree_transforms_host;
  AffineSpace3fa* tree_transforms_device;

  bool use_instance_array;
  int spp;
};

inline void TutorialData_Constructor(TutorialData* This)
{
  This->g_scene = nullptr;
  This->g_traversable = nullptr;
  This->tree_ids_host = nullptr;
  This->tree_ids_device = nullptr;
  This->tree_transforms_host = nullptr;
  This->tree_transforms_device = nullptr;
  This->terrain_triangles = nullptr;
}

inline void TutorialData_FreeTreeData(void* hptr, void* dptr)
{
  if(hptr == dptr) {
    // either CPU or unified memory mode
    if(hptr) alignedFree(hptr);
  } else {
    if(hptr) alignedFree(hptr);
    if(dptr) alignedUSMFree(dptr);
  }
  hptr = nullptr;
  dptr = nullptr;
}

inline void TutorialData_Destructor(TutorialData* This)
{
  rtcReleaseScene (This->g_scene); This->g_scene = nullptr;

  TutorialData_FreeTreeData((void*)This->tree_ids_host, (void*)This->tree_ids_device);
  TutorialData_FreeTreeData((void*)This->tree_transforms_host, (void*)This->tree_transforms_device);
}

} // namespace embree
