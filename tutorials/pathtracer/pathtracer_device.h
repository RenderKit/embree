// Copyright 2009-2021 Intel Corporation
// SPDX-License-Identifier: Apache-2.0

#include "../common/math/random_sampler.h"
#include "../common/math/sampling.h"
#include "../common/core/differential_geometry.h"
#include "../common/tutorial/tutorial_device.h"
#include "../common/tutorial/scene_device.h"
#include "../common/tutorial/optics.h"

namespace embree {

extern "C" ISPCScene* g_ispc_scene;
extern "C" int g_instancing_mode;
extern "C" int g_spp;
extern "C" int g_max_path_length;
extern "C" bool g_accumulate;
extern "C" bool g_changed;

struct TutorialData
{
  ISPCScene* ispc_scene;
  int instancing_mode;
  RTCRayQueryFlags iflags_coherent;
  RTCRayQueryFlags iflags_incoherent;

  RTCFilterFunctionN occlusionFilterOpaque;
  RTCFilterFunctionN occlusionFilterHair;
  
  int spp;
  int max_path_length;

  /* accumulation buffer */
  Vec3ff* accu;
  unsigned int accu_width;
  unsigned int accu_height;
  unsigned int accu_count;
  
  bool animation;
  bool use_smooth_normals;
  
  /* scene data */
  RTCScene scene;
};

void TutorialData_Constructor(TutorialData* This)
{
  This->ispc_scene = g_ispc_scene;
  This->instancing_mode = g_instancing_mode;
  This->iflags_coherent = g_iflags_coherent;
  This->iflags_incoherent = g_iflags_incoherent;

  This->occlusionFilterOpaque = nullptr;
  This->occlusionFilterHair = nullptr; 
  
  This->spp = g_spp;
  This->max_path_length = g_max_path_length;

  This->accu = nullptr;
  This->accu_width = 0;
  This->accu_height = 0;
  This->accu_count = 0;

  This->animation = true;
  This->use_smooth_normals = false;
        
  This->scene = nullptr;
}

void TutorialData_Destructor(TutorialData* This)
{
  rtcReleaseScene (This->scene); This->scene = nullptr;
  alignedUSMFree(This->accu); This->accu = nullptr;
  This->accu_width = 0;
  This->accu_height = 0;
  This->accu_count = 0;
}

} // namespace embree
