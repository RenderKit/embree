// Copyright 2009-2021 Intel Corporation
// SPDX-License-Identifier: Apache-2.0

#include "../common/tutorial/tutorial_device.h"

namespace embree {

#define NUM_VERTICES 9
#define NUM_CURVES 6

#define W 2.0f

float static_hair_vertices[NUM_VERTICES][4] =
{
{ -1.0f, 0.0f, -   W, 0.2f },

{ +0.0f,-1.0f, +0.0f, 0.2f },
{ +1.0f, 0.0f, +   W, 0.2f },
{ -1.0f, 0.0f, +   W, 0.2f },
{ +0.0f,+1.0f, +0.0f, 0.6f },
{ +1.0f, 0.0f, -   W, 0.2f },
{ -1.0f, 0.0f, -   W, 0.2f },

{ +0.0f,-1.0f, +0.0f, 0.2f },
{ +1.0f, 0.0f, +   W, 0.2f },
};

float static_hair_normals[NUM_VERTICES][3] =
{
{ -1.0f,  0.0f, 0.0f},

{  0.0f, +1.0f, 0.0f},
{ +1.0f,  0.0f, 0.0f},
{  0.0f, -1.0f, 0.0f},
{ -1.0f,  0.0f, 0.0f},
{  0.0f, +1.0f, 0.0f},
{ +1.0f,  0.0f, 0.0f},

{  0.0f, -1.0f, 0.0f},
{ -1.0f,  0.0f, 0.0f},
};

float static_hair_vertex_colors[NUM_VERTICES][3] =
{
{  1.0f,  1.0f,  0.0f},

{  1.0f,  0.0f,  0.0f},
{  1.0f,  1.0f,  0.0f},
{  0.0f,  0.0f,  1.0f},
{  1.0f,  1.0f,  1.0f},
{  1.0f,  0.0f,  0.0f},
{  1.0f,  1.0f,  0.0f},

{  1.0f,  0.0f,  0.0f},
{  1.0f,  1.0f,  0.0f},
};

unsigned int static_hair_indices[NUM_CURVES] = {
0, 1, 2, 3, 4, 5
};

unsigned int static_hair_indices_linear[NUM_CURVES] = {
1, 2, 3, 4, 5, 6
};

char static_hair_flags_linear[NUM_CURVES] = {
0x3, 0x3, 0x3, 0x3, 0x3, 0x3
};

struct TutorialData
{
  RTCScene g_scene;
  RTCTraversable g_traversable;
  Vec4f* hair_vertices;
  Vec3fa* hair_normals;
  Vec3fa* hair_vertex_colors;
  int* hair_indices;
  int* hair_indices_linear;
  char * hair_flags_linear;
};

inline void TutorialData_Constructor(TutorialData* This)
{
  This->g_scene = nullptr;
  This->g_traversable = nullptr;
  This->hair_vertices = (Vec4f*) alignedUSMMalloc((NUM_VERTICES)*sizeof(Vec4f),16);
  This->hair_normals = (Vec3fa*) alignedUSMMalloc((NUM_VERTICES)*sizeof(Vec3fa),16);
  This->hair_vertex_colors = (Vec3fa*) alignedUSMMalloc((NUM_VERTICES)*sizeof(Vec3fa),16);
  This->hair_indices = (int*) alignedUSMMalloc((NUM_CURVES)*sizeof(int),16);
  This->hair_indices_linear = (int*) alignedUSMMalloc((NUM_CURVES)*sizeof(int),16);
  This->hair_flags_linear = (char*) alignedUSMMalloc((NUM_CURVES)*sizeof(char),16);

  for (int i = 0; i < NUM_VERTICES; i++)
  {
    This->hair_vertices[i] = Vec4f(static_hair_vertices[i][0],
                                        static_hair_vertices[i][1],
                                        static_hair_vertices[i][2],
                                        static_hair_vertices[i][3]);
    This->hair_normals[i] = Vec3fa(static_hair_normals[i][0],
                                        static_hair_normals[i][1],
                                        static_hair_normals[i][2]);
    This->hair_vertex_colors[i] = Vec3fa(static_hair_vertex_colors[i][0],
                                              static_hair_vertex_colors[i][1],
                                              static_hair_vertex_colors[i][2]);
  }
  for (int i = 0; i < NUM_CURVES; i++) {
    This->hair_indices[i] = static_hair_indices[i];
    This->hair_indices_linear[i] = static_hair_indices_linear[i];
    This->hair_flags_linear[i] = static_hair_flags_linear[i];
  }
}

inline void TutorialData_Destructor(TutorialData* This)
{
  rtcReleaseScene (This->g_scene); This->g_scene = nullptr;
  alignedUSMFree(This->hair_vertices); This->hair_vertices = nullptr;
  alignedUSMFree(This->hair_normals); This->hair_normals = nullptr;
  alignedUSMFree(This->hair_vertex_colors); This->hair_vertex_colors = nullptr;
  alignedUSMFree(This->hair_indices); This->hair_indices = nullptr;
  alignedUSMFree(This->hair_indices_linear); This->hair_indices_linear = nullptr;
  alignedUSMFree(This->hair_flags_linear); This->hair_flags_linear = nullptr;
}

} // namespace embree
