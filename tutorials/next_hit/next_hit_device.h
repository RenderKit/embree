// Copyright 2009-2021 Intel Corporation
// SPDX-License-Identifier: Apache-2.0

#include "../common/math/random_sampler.h"
#include "../common/core/differential_geometry.h"
#include "../common/tutorial/tutorial_device.h"
#include "../common/tutorial/scene_device.h"
#include <algorithm>
#include "../common/tutorial/tutorial.h"

namespace embree {

extern "C" ISPCScene* g_ispc_scene;
extern "C" int g_instancing_mode;
extern "C" int g_next_hit_mode;
extern "C" unsigned g_max_next_hits;
extern "C" unsigned g_max_total_hits;
extern "C" bool g_verify;
extern "C" bool g_visualize_errors;
extern "C" bool g_enable_opacity;
extern "C" float g_curve_opacity;

struct TutorialData
{
  RTCScene scene;
  ISPCScene* ispc_scene;
  
  int instancing_mode;
  int next_hit_mode;
  unsigned max_next_hits;
  unsigned max_total_hits;
  bool verify;
  bool visualize_errors;
  bool enable_opacity;
  float curve_opacity;

  int* num_prev_hits;
  unsigned int num_prev_hits_width = 0;
  unsigned int num_prev_hits_height = 0;
};

void TutorialData_Constructor(TutorialData* This)
{
  This->scene = nullptr;
  This->ispc_scene = g_ispc_scene;
  This->instancing_mode = g_instancing_mode;
  This->next_hit_mode = g_next_hit_mode;
  This->max_next_hits = g_max_next_hits;
  This->max_total_hits = g_max_total_hits;
  This->verify = g_verify;
  This->visualize_errors = g_visualize_errors;
  This->enable_opacity = g_enable_opacity;
  This->curve_opacity = g_curve_opacity;
  This->num_prev_hits = nullptr;
}

void TutorialData_Destructor(TutorialData* This)
{
  rtcReleaseScene (This->scene); This->scene = nullptr;
  alignedUSMFree(This->num_prev_hits); This->num_prev_hits = nullptr;
}

} // namespace embree



