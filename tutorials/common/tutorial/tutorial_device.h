// ======================================================================== //
// Copyright 2009-2016 Intel Corporation                                    //
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

#include "../../../common/sys/platform.h"
#include "../../../kernels/algorithms/parallel_for.h"
#include "config.h"
#include "noise.h"

/* size of screen tiles */
#define TILE_SIZE_X 8
#define TILE_SIZE_Y 8

#define __RTCRay__
#define __RTCRay4__
#define __RTCRay8__
#define __RTCRay16__
#include "../../../include/embree2/rtcore.h"
#include "../../../include/embree2/rtcore_ray.h"

#include "../core/ray.h"
#include "camera.h"
#include "scene.h"

namespace embree
{
/* vertex and triangle layout */
#if !defined(__NO_VERTEX__)
struct Vertex   { float x,y,z,r; };
#endif
struct Triangle { int v0, v1, v2; };
struct Quad     { int v0, v1, v2, v3; };

enum Mode {
  MODE_NORMAL = 0,
  MODE_STREAM_COHERENT = 1,
  MODE_STREAM_INCOHERENT = 2
};

/* error reporting function */
void error_handler(const RTCError code, const char* str = nullptr);

extern "C" Mode g_mode;

/* returns time stamp counter */
extern "C" int64_t get_tsc();

/* face forward for shading normals */
__forceinline Vec3f faceforward( const Vec3f& N, const Vec3f& I, const Vec3f& Ng ) {
  return dot(I, Ng) < 0 ? N : -N;
}

/* glut keys codes */
#if !defined(GLUT_KEY_F1)
#define GLUT_KEY_F1 1
#define GLUT_KEY_F2 2
#define GLUT_KEY_F3 3
#define GLUT_KEY_F4 4
#define GLUT_KEY_F5 5
#define GLUT_KEY_F6 6
#define GLUT_KEY_F7 7
#define GLUT_KEY_F8 8
#define GLUT_KEY_F9 9
#define GLUT_KEY_F10 10
#define GLUT_KEY_F11 11
#define GLUT_KEY_F12 12
#endif

/* standard shading function */
typedef void (* renderTileFunc)(int taskIndex, int* pixels, const unsigned int width, const unsigned int height,
                                const float time, const ISPCCamera& camera,
                                const int numTilesX, const int numTilesY);
extern renderTileFunc renderTile;

extern "C" void device_key_pressed_default(int key);
extern "C" void (*key_pressed_handler)(int key);

void renderTileStandard(int taskIndex, int* pixels, const unsigned int width, const unsigned int height,
                        const float time, const ISPCCamera& camera,
                        const int numTilesX, const int numTilesY);

__forceinline Vec3f  neg(const Vec3f& a ) { return -a; }
__forceinline Vec3fa neg(const Vec3fa& a) { return -a; }
__forceinline bool   eq (const Vec3fa& a, const Vec3fa& b) { return a == b; }
__forceinline bool   ne (const Vec3fa& a, const Vec3fa& b) { return a != b; }
__forceinline bool   eq (const AffineSpace3fa& a, const AffineSpace3fa& b) { return a == b; }

struct Sample3f
{
  Sample3f () {}

  Sample3f (const Vec3fa& v, const float pdf)
    : v(v), pdf(pdf) {}

  Vec3fa v;
  float pdf;
};

/* draws progress bar */
extern "C" void progressStart();
extern "C" bool progressMonitor(void* ptr, const double dn);
extern "C" void progressEnd();

Vec2f getTextureCoordinatesSubdivMesh(void* mesh, const unsigned int primID, const float u, const float v);

float  getTextureTexel1f(const Texture* texture, float u, float v);
Vec3fa getTextureTexel3f(const Texture* texture, float u, float v);
}
