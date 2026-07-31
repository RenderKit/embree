// Copyright 2009-2021 Intel Corporation
// SPDX-License-Identifier: Apache-2.0

#include "../common/tutorial/tutorial_device.h"

namespace embree {

extern "C" float cube_vertices[8][4];
extern "C" unsigned int cube_triangle_indices[36];
extern "C" unsigned int cube_quad_indices[24];
  
struct Sphere
{
  ALIGNED_STRUCT_(16)
  Vec3fa p;                      //!< position of the sphere
  float r;                      //!< radius of the sphere
  unsigned int geomID;
  unsigned int num_time_steps;
};

struct TutorialData
{
  int frameID;
  
  RTCScene g_scene;
  RTCTraversable g_traversable;
  Vec3fa* face_colors;
  float g_time;
  
  /* accumulation buffer */
  Vec3ff* g_accu;
  unsigned int g_accu_width;
  unsigned int g_accu_height;
  unsigned int g_accu_count;
  Vec3fa g_accu_vx;
  Vec3fa g_accu_vy;
  Vec3fa g_accu_vz;
  Vec3fa g_accu_p;

  float* cube_vertices;
  int* cube_triangle_indices;
  int* cube_quad_indices;

  RTCScene scene0;
  RTCScene scene1;
  RTCScene scene2;
  RTCScene scene3;
  Sphere* sphere0;
  Sphere* sphere1;
};

#if __SYCL_COMPILER_VERSION >= 20210801
}
namespace sycl {
  template<> struct is_device_copyable<embree::TutorialData> : std::true_type {};
  template<> struct is_device_copyable<const embree::TutorialData> : std::true_type {};
}
namespace embree {
#endif

inline void TutorialData_Constructor(TutorialData* This)
{
  This->frameID = 50;
  This->g_scene = nullptr;
  This->g_traversable = nullptr;
  This->face_colors = (Vec3fa*) alignedUSMMalloc((12)*sizeof(Vec3fa),16);
  This->g_time = 0;
  This->g_accu = nullptr;
  This->g_accu_width = 0;
  This->g_accu_height = 0;
  This->g_accu_count = 0;
  This->g_accu_vx = Vec3fa(0.0f);
  This->g_accu_vy = Vec3fa(0.0f);
  This->g_accu_vz = Vec3fa(0.0f);
  This->g_accu_p  = Vec3fa(0.0f);

  This->scene0 = nullptr;
  This->scene1 = nullptr;
  This->scene2 = nullptr;
  This->scene3 = nullptr;
  This->sphere0 = nullptr;
  This->sphere1 = nullptr;

  This->cube_vertices = (float*) alignedUSMMalloc((8*4)*sizeof(float),16);
  This->cube_triangle_indices = (int*) alignedUSMMalloc((36)*sizeof(int),16);
  This->cube_quad_indices = (int*) alignedUSMMalloc((24)*sizeof(int),16);

  memcpy(This->cube_vertices, cube_vertices, 8*4*sizeof(float));
  memcpy(This->cube_triangle_indices, cube_triangle_indices, 36*sizeof(int));
  memcpy(This->cube_quad_indices, cube_quad_indices, 24*sizeof(int));
}

inline void TutorialData_Destructor(TutorialData* This)
{
  alignedUSMFree(This->face_colors); This->face_colors = nullptr;
  alignedUSMFree(This->sphere0); This->sphere0 = nullptr;
  alignedUSMFree(This->sphere1); This->sphere1 = nullptr;
  rtcReleaseScene(This->scene0); This->scene0 = nullptr;
  rtcReleaseScene(This->scene1); This->scene1 = nullptr;
  rtcReleaseScene(This->scene2); This->scene2 = nullptr;
  rtcReleaseScene(This->scene3); This->scene3 = nullptr;
  rtcReleaseScene (This->g_scene); This->g_scene = nullptr;
  alignedUSMFree(This->g_accu); This->g_accu = nullptr;
  This->g_accu_width = 0;
  This->g_accu_height = 0;
  This->g_accu_count = 0;
  alignedUSMFree(This->cube_vertices);
  alignedUSMFree(This->cube_triangle_indices);
  alignedUSMFree(This->cube_quad_indices);
}

} // namespace embree
