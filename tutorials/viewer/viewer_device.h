// Copyright 2009-2021 Intel Corporation
// SPDX-License-Identifier: Apache-2.0

#include "../common/tutorial/tutorial_device.h"
#include "../common/math/random_sampler.h"
#include "../common/core/differential_geometry.h"
#include "../common/tutorial/scene_device.h"

namespace embree {

extern "C" ISPCScene* g_ispc_scene;
extern "C" int g_instancing_mode;
extern "C" float g_min_width;
extern "C" int g_animation_mode;
extern "C" bool g_motion_blur;

struct TutorialData
{
  ISPCScene* ispc_scene;
  int instancing_mode;
  RTCRayQueryFlags iflags_coherent;
  int spp;
  int max_path_length;
  
  /* scene data */
  RTCScene scene;
  bool subdiv_mode;
  bool motion_blur;

  float min_width;
};

#define ENABLE_FP16_GBUFFER 0

#if ENABLE_FP16_GBUFFER == 1
  typedef sycl::vec<cl::sycl::cl_half, 3>  Vec3fp16;

  __forceinline Vec3f    fp_convert(const Vec3fp16 &v) { return Vec3f((float)v.x(),(float)v.y(),(float)v.z()); }
  __forceinline Vec3fp16 fp_convert(const Vec3f    &v) { return Vec3fp16(v.x,v.y,v.z);  }

#else
  __forceinline Vec3f    fp_convert(const Vec3f    &v) { return v; }  
#endif


struct GBuffer
{
#if ENABLE_FP16_GBUFFER == 1
  Vec3fp16 color,normal,albedo;
#else
  Vec3f color;
  Vec3f normal;
  Vec3f albedo;
#endif  
};

#if ENABLE_FP16_GBUFFER == 1
  typedef Vec3fp16 GBufferOutput;
#else
  typedef Vec3f GBufferOutput;
#endif
  
inline void TutorialData_Constructor(TutorialData* This)
{
  This->ispc_scene = g_ispc_scene;
  This->instancing_mode = g_instancing_mode;
  This->iflags_coherent = g_iflags_coherent;
  This->scene = nullptr;
  This->subdiv_mode = false;
  This->motion_blur = g_motion_blur;
  This->min_width = g_min_width;
  This->spp = 1;
  This->max_path_length = 2;
  
}

inline void TutorialData_Destructor(TutorialData* This)
{
  rtcReleaseScene (This->scene); This->scene = nullptr;
}

} // namespace embree
