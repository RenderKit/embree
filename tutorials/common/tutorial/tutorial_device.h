// Copyright 2009-2021 Intel Corporation
// SPDX-License-Identifier: Apache-2.0

#pragma once

#define _CRT_SECURE_NO_WARNINGS

/* size of screen tiles */
#define TILE_SIZE_X 8
#define TILE_SIZE_Y 8

/* vertex and triangle layout */
struct Vertex   { float x,y,z,r;  }; // FIXME: rename to Vertex4f
struct Triangle { int v0, v1, v2; };

#include "../device_default.h"

/* include optional vector library */
#include "../math/math.h"
#include "../math/vec.h"
#include "../math/affinespace.h"
#include "../core/ray.h"
#include "camera.h"
#include "scene_device.h"
#include "noise.h"
#if !defined(ISPC)
#include "../../../common/algorithms/parallel_for.h"

namespace embree {
#endif

#if defined(EMBREE_SYCL_TUTORIAL)
inline sycl::nd_range<2> make_nd_range(unsigned int size0, unsigned int size1)
{
  const sycl::range<2> wg_size = sycl::range<2>(4,4);

  /* align iteration space to work group size */
  size0 = ((size0 + wg_size[0] - 1) / wg_size[0]) * wg_size[0];
  size1 = ((size1 + wg_size[1] - 1) / wg_size[1]) * wg_size[1];

  return sycl::nd_range(sycl::range(size0,size1),wg_size);
}
#endif

enum Shader { 
  SHADER_DEFAULT, 
  SHADER_EYELIGHT,
  SHADER_OCCLUSION,
  SHADER_UV,
  SHADER_TEXCOORDS,
  SHADER_TEXCOORDS_GRID,
  SHADER_NG,
  SHADER_CYCLES,
  SHADER_GEOMID,
  SHADER_GEOMID_PRIMID,
  SHADER_AO
};

extern "C" RTCDevice g_device;
extern "C" RTCRayQueryFlags g_iflags_coherent;
extern "C" RTCRayQueryFlags g_iflags_incoherent;
extern "C" Shader shader;

/* error reporting function */
void error_handler(void* userPtr, RTCError code, const char* str = nullptr);

/* returns time stamp counter */
#if defined(EMBREE_SYCL_SUPPORT) && defined(__SYCL_DEVICE_ONLY__)
inline int64_t get_tsc() { return 0; }
#else
extern "C" int64_t get_tsc();
#endif

/* face forward for shading normals */
inline Vec3fa faceforward( const Vec3fa& N, const Vec3fa& I, const Vec3fa& Ng ) {
  Vec3fa NN = N; return dot(I, Ng) < 0 ? NN : neg(NN);
}

/* GLFW keys codes */
#if !defined(GLFW_KEY_F1)
#define GLFW_KEY_F1                 290
#define GLFW_KEY_F2                 291
#define GLFW_KEY_F3                 292
#define GLFW_KEY_F4                 293
#define GLFW_KEY_F5                 294
#define GLFW_KEY_F6                 295
#define GLFW_KEY_F7                 296
#define GLFW_KEY_F8                 297
#define GLFW_KEY_F9                 298
#define GLFW_KEY_F10                299
#define GLFW_KEY_F11                300
#define GLFW_KEY_F12                301
#endif

extern "C" void device_key_pressed_default(int key);
extern "C" void (* key_pressed_handler)(int key);

extern "C" void renderFrameStandard(int* pixels,
                         const unsigned int width,
                         const unsigned int height,
                         const float time,
                         const ISPCCamera& camera);

unsigned int getNumHWThreads();

#if defined(ISPC)
#define ALIGNED_STRUCT_(x)
#define __aligned(x)
#define MAYBE_UNUSED
#endif

/* draws progress bar */
extern "C" void progressStart();
extern "C" bool progressMonitor(void* ptr, const double n);
extern "C" void progressEnd();

SYCL_EXTERNAL Vec2f  getTextureCoordinatesSubdivMesh(void* mesh, const unsigned int primID, const float u, const float v);
SYCL_EXTERNAL float  getTextureTexel1f(const Texture* texture, float u, float v);
SYCL_EXTERNAL Vec3fa  getTextureTexel3f(const Texture* texture, float u, float v);

enum ISPCInstancingMode { ISPC_INSTANCING_NONE, ISPC_INSTANCING_GEOMETRY, ISPC_INSTANCING_GROUP };

/* ray statistics */
#if !defined(TASKING_PPL) // not supported with PPL because threadIndex is not unique and atomics are too expensive
#define RAY_STATS
#endif

struct RayStats
{
  int numRays;
  int pad[32-1];
};

#if defined(RAY_STATS)
#if defined(ISPC)
inline void RayStats_addRay(RayStats& stats)       { stats.numRays += popcnt(1); }
inline void RayStats_addShadowRay(RayStats& stats) { stats.numRays += popcnt(1); }
#else // C++
__forceinline void RayStats_addRay(RayStats& stats)        { stats.numRays++; }
__forceinline void RayStats_addShadowRay(RayStats& stats)  { stats.numRays++; }
#endif
#else // disabled
inline void RayStats_addRay(RayStats& stats)       {}
inline void RayStats_addShadowRay(RayStats& stats) {}
#endif

extern "C" RayStats* g_stats;

inline bool nativePacketSupported(RTCDevice device)
{
  if (sizeof(float) == 1*4) return true;
  else if (sizeof(float) == 4*4) return rtcGetDeviceProperty(device,RTC_DEVICE_PROPERTY_NATIVE_RAY4_SUPPORTED);
  else if (sizeof(float) == 8*4) return rtcGetDeviceProperty(device,RTC_DEVICE_PROPERTY_NATIVE_RAY8_SUPPORTED);
  else if (sizeof(float) == 16*4) return rtcGetDeviceProperty(device,RTC_DEVICE_PROPERTY_NATIVE_RAY16_SUPPORTED);
  else return false;
}

} // namespace embree
