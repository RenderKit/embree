// ======================================================================== //
// Copyright 2009-2018 Intel Corporation                                    //
//                                                                          //
// Licensed under the Apache License, Version 2.0 (the "License");          //
// you may not use this file except in compliance with the License.         //
// You may obtain a copy of the License at                                  //
//                                                                          //
//     http://www.apache.org/licenses/LICENSE-2.0                           //
//                                                                          //
// Unless required by applicable law or agreed to in writing, software      //
// distributed under the License is distributed on an "AS IS" BASIS,        //
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. //
// See the License for the specific language governing permissions and      //
// limitations under the License.                                           //
// ======================================================================== //

#pragma once

#define _CRT_SECURE_NO_WARNINGS

/* size of screen tiles */
#define TILE_SIZE_X 8
#define TILE_SIZE_Y 8

/* vertex and triangle layout */
struct Vertex   { float x,y,z,r;  }; // FIXME: rename to Vertex4f
struct Triangle { int v0, v1, v2; };

/* include embree API */
#include "../../../include/embree3/rtcore.h"
RTC_NAMESPACE_OPEN

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

enum Mode {
  MODE_NORMAL = 0,
  MODE_STREAM = 1
};

extern "C" RTCDevice g_device;
extern "C" Mode g_mode;
extern "C" RTCIntersectContextFlags g_iflags_coherent;
extern "C" RTCIntersectContextFlags g_iflags_incoherent;

/* error reporting function */
void error_handler(void* userPtr, RTCError code, const char* str = nullptr);

/* returns time stamp counter */
extern "C" int64_t get_tsc();

/* declare some standard library functions */
extern "C" void abort ();
extern "C" void exit(int);
extern "C" int puts ( const char* str );
extern "C" int putchar ( int character );

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

/* standard shading function */
typedef void (* renderTileFunc)(int taskIndex,
                                        int threadIndex,
                                        int* pixels,
                                        const unsigned int width,
                                        const unsigned int height,
                                        const float time,
                                        const ISPCCamera& camera,
                                        const int numTilesX,
                                        const int numTilesY);
extern "C" renderTileFunc renderTile;

extern "C" void device_key_pressed_default(int key);
extern "C" void (* key_pressed_handler)(int key);

void renderTileStandard(int taskIndex,
                        int threadIndex,
                        int* pixels,
                        const unsigned int width,
                        const unsigned int height,
                        const float time,
                        const ISPCCamera& camera,
                        const int numTilesX,
                        const int numTilesY);

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

Vec2f  getTextureCoordinatesSubdivMesh(void* mesh, const unsigned int primID, const float u, const float v);

float  getTextureTexel1f(const Texture* texture, float u, float v);
Vec3fa  getTextureTexel3f(const Texture* texture, float u, float v);

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
inline void RayStats_addRay(RayStats& stats)       { stats.numRays += popcnt(lanemask()); }
inline void RayStats_addShadowRay(RayStats& stats) { stats.numRays += popcnt(lanemask()); }
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
