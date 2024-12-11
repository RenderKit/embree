// Copyright 2009-2021 Intel Corporation
// SPDX-License-Identifier: Apache-2.0

#include "motion_blur_geometry_device.h"

namespace embree {

#define USE_ARGUMENT_CALLBACKS 1

/* all features required by this tutorial */
#if USE_ARGUMENT_CALLBACKS
#define FEATURE_MASK \
  RTC_FEATURE_FLAG_TRIANGLE | \
  RTC_FEATURE_FLAG_QUAD |            \
  RTC_FEATURE_FLAG_SUBDIVISION |     \
  RTC_FEATURE_FLAG_FLAT_LINEAR_CURVE |               \
  RTC_FEATURE_FLAG_FLAT_BSPLINE_CURVE |              \
  RTC_FEATURE_FLAG_ROUND_BSPLINE_CURVE |             \
  RTC_FEATURE_FLAG_DISC_POINT |                      \
  RTC_FEATURE_FLAG_SPHERE_POINT |                    \
  RTC_FEATURE_FLAG_ORIENTED_DISC_POINT |             \
  RTC_FEATURE_FLAG_INSTANCE |                        \
  RTC_FEATURE_FLAG_USER_GEOMETRY_CALLBACK_IN_ARGUMENTS | \
  RTC_FEATURE_FLAG_MOTION_BLUR
#else
#define FEATURE_MASK \
  RTC_FEATURE_FLAG_TRIANGLE | \
  RTC_FEATURE_FLAG_QUAD |            \
  RTC_FEATURE_FLAG_SUBDIVISION |     \
  RTC_FEATURE_FLAG_FLAT_LINEAR_CURVE |               \
  RTC_FEATURE_FLAG_FLAT_BSPLINE_CURVE |              \
  RTC_FEATURE_FLAG_ROUND_BSPLINE_CURVE |             \
  RTC_FEATURE_FLAG_DISC_POINT |                      \
  RTC_FEATURE_FLAG_SPHERE_POINT |                    \
  RTC_FEATURE_FLAG_ORIENTED_DISC_POINT |             \
  RTC_FEATURE_FLAG_INSTANCE |                        \
  RTC_FEATURE_FLAG_USER_GEOMETRY_CALLBACK_IN_GEOMETRY | \
  RTC_FEATURE_FLAG_MOTION_BLUR
#endif

/* scene data */
RTCScene g_scene = nullptr;
TutorialData data;

extern "C" bool g_changed;
extern "C" float g_time;
extern "C" unsigned int g_num_time_steps;
extern "C" unsigned int g_num_time_steps2;


float cube_vertices[8][4] =
{
  { -1.0f, -1.0f, -1.0f, 0.0f },
  {  1.0f, -1.0f, -1.0f, 0.0f },
  {  1.0f, -1.0f,  1.0f, 0.0f },
  { -1.0f, -1.0f,  1.0f, 0.0f },
  { -1.0f,  1.0f, -1.0f, 0.0f },
  {  1.0f,  1.0f, -1.0f, 0.0f },
  {  1.0f,  1.0f,  1.0f, 0.0f },
  { -1.0f,  1.0f,  1.0f, 0.0f }
};

unsigned int cube_triangle_indices[36] = {
  1, 4, 5,  0, 4, 1,
  2, 5, 6,  1, 5, 2,
  3, 6, 7,  2, 6, 3,
  4, 3, 7,  0, 3, 4,
  5, 7, 6,  4, 7, 5,
  3, 1, 2,  0, 1, 3
};

unsigned int cube_quad_indices[24] = {
  0, 4, 5, 1,
  1, 5, 6, 2,
  2, 6, 7, 3,
  0, 3, 7, 4,
  4, 7, 6, 5,
  0, 1, 2, 3,
};

float cube_vertex_crease_weights[8] = {
  inf, inf,inf, inf, inf, inf, inf, inf
};

unsigned int cube_vertex_crease_indices[8] = {
  0,1,2,3,4,5,6,7
};

float cube_edge_crease_weights[12] = {
  inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf
};

unsigned int cube_edge_crease_indices[24] =
{
  0,1, 1,2, 2,3, 3,0,
  4,5, 5,6, 6,7, 7,4,
  0,4, 1,5, 2,6, 3,7,
};

#define NUM_INDICES 24
#define NUM_FACES 6
#define FACE_SIZE 4

unsigned int cube_quad_faces[6] = {
  4, 4, 4, 4, 4, 4
};

unsigned int addSphere(RTCScene scene, const Vec3fa& pos, RTCGeometryType type, unsigned int num_time_steps)
{
  RTCGeometry geom = rtcNewGeometry(g_device, type);
  rtcSetGeometryTimeStepCount(geom, num_time_steps);

  for (unsigned int t = 0; t < num_time_steps; t++)
  {
    RTCBufferType bufType = RTC_BUFFER_TYPE_VERTEX;
    Vec3ff *vertex = (Vec3ff*)rtcSetNewGeometryBuffer(geom, bufType, t, RTC_FORMAT_FLOAT4, sizeof(Vec3ff), 1);
    AffineSpace3fa rotation = AffineSpace3fa::rotate(Vec3fa(0,0,0), Vec3fa(0,1,0), 2.0f*float(M_PI)*(float)t/(float)(num_time_steps-1));
    *vertex = Vec3ff(xfmPoint(rotation, Vec3fa(1, 0, 0)) + pos);
    vertex->w = 1.f;

    if (type == RTC_GEOMETRY_TYPE_ORIENTED_DISC_POINT) {
      Vec3fa *normal = (Vec3fa*)rtcSetNewGeometryBuffer(geom, RTC_BUFFER_TYPE_NORMAL, t, RTC_FORMAT_FLOAT3, sizeof(Vec3fa), 1);
      normal[0] = Vec3fa(1, 1, 0);
      normal[0] = normalize(normal[0]);
    }
  }

  rtcCommitGeometry(geom);
  unsigned int geomID = rtcAttachGeometry(scene, geom);
  rtcReleaseGeometry(geom);
  return geomID;
}

/* adds a cube to the scene */
unsigned int addTriangleCube (RTCScene scene, const Vec3fa& pos, unsigned int num_time_steps)
{
  /* create a triangulated cube with 12 triangles and 8 vertices */
  RTCGeometry geom = rtcNewGeometry (g_device, RTC_GEOMETRY_TYPE_TRIANGLE);
  rtcSetGeometryTimeStepCount(geom,num_time_steps);
  rtcSetSharedGeometryBuffer(geom, RTC_BUFFER_TYPE_INDEX, 0, RTC_FORMAT_UINT3, data.cube_triangle_indices, 0, 3*sizeof(unsigned int), 12);

  for (unsigned int t=0; t<num_time_steps; t++)
  {
    RTCBufferType bufType = RTC_BUFFER_TYPE_VERTEX;
    Vec3fa* vertices = (Vec3fa*) rtcSetNewGeometryBuffer(geom,bufType,t,RTC_FORMAT_FLOAT3,sizeof(Vec3fa), 8);

    AffineSpace3fa rotation = AffineSpace3fa::rotate(Vec3fa(0,0,0),Vec3fa(0,1,0),2.0f*float(M_PI)*(float)t/(float)(num_time_steps-1));
    AffineSpace3fa scale = AffineSpace3fa::scale(Vec3fa(2.0f,1.0f,1.0f));

    for (int i=0; i<8; i++) {
      Vec3fa v = Vec3fa(cube_vertices[i][0],cube_vertices[i][1],cube_vertices[i][2]);
      vertices[i] = Vec3fa(xfmPoint(rotation*scale,v)+pos);
    }
  }

  /* create face color array */
  data.face_colors[0] = Vec3fa(1,0,0);
  data.face_colors[1] = Vec3fa(1,0,0);
  data.face_colors[2] = Vec3fa(0,1,0);
  data.face_colors[3] = Vec3fa(0,1,0);
  data.face_colors[4] = Vec3fa(0.5f);
  data.face_colors[5] = Vec3fa(0.5f);
  data.face_colors[6] = Vec3fa(1.0f);
  data.face_colors[7] = Vec3fa(1.0f);
  data.face_colors[8] = Vec3fa(0,0,1);
  data.face_colors[9] = Vec3fa(0,0,1);
  data.face_colors[10] = Vec3fa(1,1,0);
  data.face_colors[11] = Vec3fa(1,1,0);

  rtcCommitGeometry(geom);
  unsigned int geomID = rtcAttachGeometry(scene,geom);
  rtcReleaseGeometry(geom);
  return geomID;
}

/* adds a cube to the scene */
unsigned int addQuadCube (RTCScene scene, const Vec3fa& pos, unsigned int num_time_steps)
{
  /* create a quad cube with 6 quads and 8 vertices */
  RTCGeometry geom = rtcNewGeometry (g_device, RTC_GEOMETRY_TYPE_QUAD);
  rtcSetGeometryTimeStepCount(geom,num_time_steps);
  rtcSetSharedGeometryBuffer(geom, RTC_BUFFER_TYPE_INDEX, 0, RTC_FORMAT_UINT4, data.cube_quad_indices, 0, 4*sizeof(unsigned int), 6);

  for (unsigned int t=0; t<num_time_steps; t++)
  {
    RTCBufferType bufType = RTC_BUFFER_TYPE_VERTEX;
    Vec3fa* vertices = (Vec3fa*) rtcSetNewGeometryBuffer(geom,bufType,t,RTC_FORMAT_FLOAT3,sizeof(Vec3fa), 8);

    AffineSpace3fa rotation = AffineSpace3fa::rotate(Vec3fa(0,0,0),Vec3fa(0,1,0),2.0f*float(M_PI)*(float)t/(float)(num_time_steps-1));
    AffineSpace3fa scale = AffineSpace3fa::scale(Vec3fa(2.0f,1.0f,1.0f));

    for (int i=0; i<8; i++) {
      Vec3fa v = Vec3fa(cube_vertices[i][0],cube_vertices[i][1],cube_vertices[i][2]);
      vertices[i] = Vec3fa(xfmPoint(rotation*scale,v)+pos);
    }
  }

  rtcCommitGeometry(geom);
  unsigned int geomID = rtcAttachGeometry(scene,geom);
  rtcReleaseGeometry(geom);
  return geomID;
}

/* adds a subdivision cube to the scene */
unsigned int addSubdivCube (RTCScene scene, const Vec3fa& pos, unsigned int num_time_steps)
{
  /* create a triangulated cube with 6 quads and 8 vertices */
  RTCGeometry geom = rtcNewGeometry(g_device, RTC_GEOMETRY_TYPE_SUBDIVISION);
  rtcSetGeometryTimeStepCount(geom,num_time_steps);

  //rtcSetSharedGeometryBuffer(geom, RTC_BUFFER_TYPE_VERTEX, 0, RTC_FORMAT_FLOAT3, cube_vertices, 0, sizeof(Vec3fa), 8);
  rtcSetSharedGeometryBuffer(geom, RTC_BUFFER_TYPE_INDEX, 0, RTC_FORMAT_UINT, cube_quad_indices, 0, sizeof(unsigned int), NUM_INDICES);
  rtcSetSharedGeometryBuffer(geom, RTC_BUFFER_TYPE_FACE,  0, RTC_FORMAT_UINT, cube_quad_faces,   0, sizeof(unsigned int), NUM_FACES);

  rtcSetSharedGeometryBuffer(geom, RTC_BUFFER_TYPE_EDGE_CREASE_INDEX,  0, RTC_FORMAT_UINT2, cube_edge_crease_indices,  0, 2*sizeof(unsigned int), 0);
  rtcSetSharedGeometryBuffer(geom, RTC_BUFFER_TYPE_EDGE_CREASE_WEIGHT, 0, RTC_FORMAT_FLOAT, cube_edge_crease_weights,  0, sizeof(float),          0);

  rtcSetSharedGeometryBuffer(geom, RTC_BUFFER_TYPE_VERTEX_CREASE_INDEX,  0, RTC_FORMAT_UINT,  cube_vertex_crease_indices,0, sizeof(unsigned int), 0);
  rtcSetSharedGeometryBuffer(geom, RTC_BUFFER_TYPE_VERTEX_CREASE_WEIGHT, 0, RTC_FORMAT_FLOAT, cube_vertex_crease_weights,0, sizeof(float),        0);

  float* level = (float*) rtcSetNewGeometryBuffer(geom, RTC_BUFFER_TYPE_LEVEL, 0, RTC_FORMAT_FLOAT, sizeof(float), NUM_INDICES);
  for (unsigned int i=0; i<NUM_INDICES; i++) level[i] = 16.0f;

  for (unsigned int t=0; t<num_time_steps; t++)
  {
    RTCBufferType bufType = RTC_BUFFER_TYPE_VERTEX;
    Vec3fa* vertices = (Vec3fa*) rtcSetNewGeometryBuffer(geom,bufType,t,RTC_FORMAT_FLOAT3,sizeof(Vec3fa),8);

    AffineSpace3fa rotation = AffineSpace3fa::rotate(Vec3fa(0,0,0),Vec3fa(0,1,0),2.0f*float(M_PI)*(float)t/(float)(num_time_steps-1));
    AffineSpace3fa scale = AffineSpace3fa::scale(Vec3fa(2.0f,1.0f,1.0f));

    for (int i=0; i<8; i++) {
      Vec3fa v = Vec3fa(cube_vertices[i][0],cube_vertices[i][1],cube_vertices[i][2]);
      vertices[i] = Vec3fa(xfmPoint(rotation*scale,v)+pos);
    }
  }

  rtcCommitGeometry(geom);
  unsigned int geomID = rtcAttachGeometry(scene,geom);
  rtcReleaseGeometry(geom);
  return geomID;
}

/* add hair geometry */
unsigned int addCurve (RTCScene scene, const Vec3fa& pos, RTCGeometryType type, unsigned int num_time_steps)
{
  RTCGeometry geom = rtcNewGeometry(g_device, type);
  rtcSetGeometryTimeStepCount(geom,num_time_steps);
  rtcSetGeometryTessellationRate (geom,16.0f);

  Vec3fa* bspline = (Vec3fa*) alignedUSMMalloc((16)*sizeof(Vec3fa),16);
  for (int i=0; i<16; i++) {
    float f = (float)(i)/16.0f;
    bspline[i] = Vec3fa(2.0f*f-1.0f,sin(12.0f*f),cos(12.0f*f));
  }

  for (unsigned int t=0; t<num_time_steps; t++)
  {
    RTCBufferType bufType = RTC_BUFFER_TYPE_VERTEX;
    Vec3ff* vertices = (Vec3ff*) rtcSetNewGeometryBuffer(geom,bufType,t,RTC_FORMAT_FLOAT4,sizeof(Vec3ff),16);

    AffineSpace3fa rotation = AffineSpace3fa::rotate(Vec3fa(0,0,0),Vec3fa(0,1,0),2.0f*float(M_PI)*(float)t/(float)(num_time_steps-1));
    AffineSpace3fa scale = AffineSpace3fa::scale(Vec3fa(2.0f,1.0f,1.0f));

    for (int i=0; i<16; i++)
      vertices[i] = Vec3ff(xfmPoint(rotation*scale,bspline[i])+pos,0.2f);
  }

  int* indices = (int*) rtcSetNewGeometryBuffer(geom,RTC_BUFFER_TYPE_INDEX,0,RTC_FORMAT_UINT,sizeof(int),13);
  for (int i=0; i<13; i++) indices[i] = i;

  alignedUSMFree(bspline);

  rtcCommitGeometry(geom);
  unsigned int geomID = rtcAttachGeometry(scene,geom);
  rtcReleaseGeometry(geom);
  return geomID;
}

/* add line geometry */
unsigned int addLines (RTCScene scene, const Vec3fa& pos, unsigned int num_time_steps)
{
  RTCGeometry geom = rtcNewGeometry (g_device, RTC_GEOMETRY_TYPE_FLAT_LINEAR_CURVE);
  rtcSetGeometryTimeStepCount(geom,num_time_steps);

  Vec3fa* bspline = (Vec3fa*) alignedUSMMalloc((16)*sizeof(Vec3fa),16);
  for (int i=0; i<16; i++) {
    float f = (float)(i)/16.0f;
    bspline[i] = Vec3fa(2.0f*f-1.0f,sin(12.0f*f),cos(12.0f*f));
  }

  for (unsigned int t=0; t<num_time_steps; t++)
  {
    RTCBufferType bufType = RTC_BUFFER_TYPE_VERTEX;
    Vec3ff* vertices = (Vec3ff*) rtcSetNewGeometryBuffer(geom,bufType,t,RTC_FORMAT_FLOAT4,sizeof(Vec3ff),16);

    AffineSpace3fa rotation = AffineSpace3fa::rotate(Vec3fa(0,0,0),Vec3fa(0,1,0),2.0f*float(M_PI)*(float)t/(float)(num_time_steps-1));
    AffineSpace3fa scale = AffineSpace3fa::scale(Vec3fa(2.0f,1.0f,1.0f));

    for (int i=0; i<16; i++)
      vertices[i] = Vec3ff(xfmPoint(rotation*scale,bspline[i])+pos,0.2f);
  }

  int* indices = (int*) rtcSetNewGeometryBuffer(geom,RTC_BUFFER_TYPE_INDEX,0,RTC_FORMAT_UINT,sizeof(int),15);
  for (int i=0; i<15; i++) indices[i] = i;

  alignedUSMFree(bspline);

  rtcCommitGeometry(geom);
  unsigned int geomID = rtcAttachGeometry(scene,geom);
  rtcReleaseGeometry(geom);
  return geomID;
}

/* adds an instanced triangle cube to the scene, rotate instance */
RTCScene addInstancedTriangleCube (RTCScene global_scene, const Vec3fa& pos, unsigned int num_time_steps)
{
  RTCScene scene = rtcNewScene(g_device);
  RTCGeometry geom = rtcNewGeometry (g_device, RTC_GEOMETRY_TYPE_TRIANGLE);
  rtcSetSharedGeometryBuffer(geom, RTC_BUFFER_TYPE_INDEX,  0, RTC_FORMAT_UINT3,  data.cube_triangle_indices, 0, 3*sizeof(unsigned int), 12);
  rtcSetSharedGeometryBuffer(geom, RTC_BUFFER_TYPE_VERTEX, 0, RTC_FORMAT_FLOAT3, data.cube_vertices, 0, 4*sizeof(float), 8);
  rtcCommitGeometry(geom);
  rtcAttachGeometry(scene,geom);
  rtcReleaseGeometry(geom);
  rtcCommitScene(scene);

  RTCGeometry inst = rtcNewGeometry (g_device, RTC_GEOMETRY_TYPE_INSTANCE);
   rtcSetGeometryInstancedScene(inst,scene);
   rtcSetGeometryTimeStepCount(inst,num_time_steps);
  
  for (unsigned int t=0; t<num_time_steps; t++)
  {
    AffineSpace3fa rotation = AffineSpace3fa::rotate(Vec3fa(0,0,0),Vec3fa(0,1,0),2.0f*float(M_PI)*(float)t/(float)(num_time_steps-1));
    AffineSpace3fa scale = AffineSpace3fa::scale(Vec3fa(2.0f,1.0f,1.0f));
    AffineSpace3fa translation = AffineSpace3fa::translate(pos);
    AffineSpace3fa xfm = translation*rotation*scale;
    rtcSetGeometryTransform(inst,t,RTC_FORMAT_FLOAT4X4_COLUMN_MAJOR,(float*)&xfm);
  }

  rtcCommitGeometry(inst);
  rtcAttachGeometry(global_scene,inst);
  rtcReleaseGeometry(inst);
  return scene;
}

/* adds an instanced quad cube to the scene, rotate instance and geometry */
RTCScene addInstancedQuadCube (RTCScene global_scene, const Vec3fa& pos, unsigned int num_time_steps)
{
  RTCScene scene = rtcNewScene(g_device);
  RTCGeometry geom = rtcNewGeometry (g_device, RTC_GEOMETRY_TYPE_QUAD);
  rtcSetGeometryTimeStepCount(geom,num_time_steps);
  rtcSetSharedGeometryBuffer(geom, RTC_BUFFER_TYPE_INDEX, 0, RTC_FORMAT_UINT4, data.cube_quad_indices, 0, 4*sizeof(unsigned int), 6);

  for (unsigned int t=0; t<num_time_steps; t++)
  {
    RTCBufferType bufType = RTC_BUFFER_TYPE_VERTEX;
    Vec3fa* vertices = (Vec3fa*) rtcSetNewGeometryBuffer(geom,bufType,t,RTC_FORMAT_FLOAT3,sizeof(Vec3fa),8);

    AffineSpace3fa rotation = AffineSpace3fa::rotate(Vec3fa(0,0,0),Vec3fa(0,1,0),0.5f*2.0f*float(M_PI)*(float)t/(float)(num_time_steps-1));
    AffineSpace3fa scale = AffineSpace3fa::scale(Vec3fa(2.0f,1.0f,1.0f));

    for (int i=0; i<8; i++) {
      Vec3fa v = Vec3fa(cube_vertices[i][0],cube_vertices[i][1],cube_vertices[i][2]);
      vertices[i] = Vec3fa(xfmPoint(rotation*scale,v));
    }
  }

  rtcCommitGeometry(geom);
  rtcAttachGeometry(scene,geom);
  rtcReleaseGeometry(geom);  
  rtcCommitScene(scene);

  RTCGeometry inst = rtcNewGeometry (g_device, RTC_GEOMETRY_TYPE_INSTANCE);
   rtcSetGeometryInstancedScene(inst,scene);
   rtcSetGeometryTimeStepCount(inst,num_time_steps);

  for (unsigned int t=0; t<num_time_steps; t++)
  {
    AffineSpace3fa rotation = AffineSpace3fa::rotate(Vec3fa(0,0,0),Vec3fa(0,1,0),0.5f*2.0f*float(M_PI)*(float)t/(float)(num_time_steps-1));
    AffineSpace3fa translation = AffineSpace3fa::translate(pos);
    AffineSpace3fa xfm = translation*rotation;
    rtcSetGeometryTransform(inst,t,RTC_FORMAT_FLOAT4X4_COLUMN_MAJOR,(float*)&xfm);
  }

  rtcCommitGeometry(inst);
  rtcAttachGeometry(global_scene,inst);
  rtcReleaseGeometry(inst);
  return scene;
}

// ======================================================================== //
//                     User defined sphere geometry                         //
// ======================================================================== //

void sphereBoundsFunc(const struct RTCBoundsFunctionArguments* args)
{
  const Sphere* spheres = (const Sphere*) args->geometryUserPtr;
  RTCBounds* bounds_o = args->bounds_o;
  const unsigned int time = args->timeStep;
  const Sphere& sphere = spheres[args->primID];
  float ft = 2.0f*float(M_PI) * (float) time / (float) (sphere.num_time_steps-1);
  Vec3fa p = sphere.p + Vec3fa(cos(ft),0.0f,sin(ft));
  bounds_o->lower_x = p.x-sphere.r;
  bounds_o->lower_y = p.y-sphere.r;
  bounds_o->lower_z = p.z-sphere.r;
  bounds_o->upper_x = p.x+sphere.r;
  bounds_o->upper_y = p.y+sphere.r;
  bounds_o->upper_z = p.z+sphere.r;
}

RTCIntersectFunctionN sphereIntersectFuncPtr = nullptr;

RTC_SYCL_INDIRECTLY_CALLABLE void sphereIntersectFuncN(const RTCIntersectFunctionNArguments* args)
{
  int* valid = args->valid;
  void* ptr  = args->geometryUserPtr;
  RTCRayHitN* rays = (RTCRayHitN*)args->rayhit;
  unsigned int primID = args->primID;
  assert(args->N == 1);
  const Sphere* spheres = (const Sphere*)ptr;
  const Sphere& sphere = spheres[primID];

  if (!valid[0]) return;
  valid[0] = 0;
  
  Ray *ray = (Ray *)rays;
  
  const int time_segments = sphere.num_time_steps-1;
  const float time = ray->time()*(float)(time_segments);
  const int itime = clamp((int)(floor(time)),(int)0,time_segments-1);
  const float ftime = time - (float)(itime);
  const float ft0 = 2.0f*float(M_PI) * (float) (itime+0) / (float) (sphere.num_time_steps-1);
  const float ft1 = 2.0f*float(M_PI) * (float) (itime+1) / (float) (sphere.num_time_steps-1);
  const Vec3fa p0 = sphere.p + Vec3fa(cos(ft0),0.0f,sin(ft0));
  const Vec3fa p1 = sphere.p + Vec3fa(cos(ft1),0.0f,sin(ft1));
  const Vec3fa sphere_p = (1.0f-ftime)*p0 + ftime*p1;
  
  const Vec3fa v = ray->org-sphere_p;
  const float A = dot(ray->dir,ray->dir);
  const float B = 2.0f*dot(v,ray->dir);
  const float C = dot(v,v) - sqr(sphere.r);
  const float D = B*B - 4.0f*A*C;
  if (D < 0.0f) return;
  const float Q = sqrt(D);
  const float rcpA = rcp(A);
  const float t0 = 0.5f*rcpA*(-B-Q);
  const float t1 = 0.5f*rcpA*(-B+Q);
  if ((ray->tnear() < t0) & (t0 < ray->tfar)) {
    ray->u = 0.0f;
    ray->v = 0.0f;
    ray->tfar = t0;
    ray->geomID = sphere.geomID;
    ray->primID = (unsigned int) primID;
    ray->Ng = ray->org+t0*ray->dir-sphere_p;
    valid[0] = -1;
  }
  if ((ray->tnear() < t1) & (t1 < ray->tfar)) {
    ray->u = 0.0f;
    ray->v = 0.0f;
    ray->tfar = t1;
    ray->geomID = sphere.geomID;
    ray->primID = (unsigned int) primID;
    ray->Ng = ray->org+t1*ray->dir-sphere_p;
    valid[0] = -1;
  }
}

RTCOccludedFunctionN sphereOccludedFuncPtr = nullptr;

RTC_SYCL_INDIRECTLY_CALLABLE void sphereOccludedFuncN(const RTCOccludedFunctionNArguments* args)
{
  int* valid = args->valid;
  void* ptr  = args->geometryUserPtr;
  RTCRayHitN* rays = (RTCRayHitN*)args->ray;
  unsigned int primID = args->primID;
  assert(args->N == 1);
  const Sphere* spheres = (const Sphere*)ptr;
  const Sphere& sphere = spheres[primID];

  if (!valid[0]) return;
  valid[0] = 0;
  
  Ray *ray = (Ray *)rays;
  const int time_segments = sphere.num_time_steps-1;
  const float time = ray->time()*(float)(time_segments);
  const int itime = clamp((int)(floor(time)),(int)0,time_segments-1);
  const float ftime = time - (float)(itime);
  const float ft0 = 2.0f*float(M_PI) * (float) (itime+0) / (float) (sphere.num_time_steps-1);
  const float ft1 = 2.0f*float(M_PI) * (float) (itime+1) / (float) (sphere.num_time_steps-1);
  const Vec3fa p0 = sphere.p + Vec3fa(cos(ft0),0.0f,sin(ft0));
  const Vec3fa p1 = sphere.p + Vec3fa(cos(ft1),0.0f,sin(ft1));
  const Vec3fa sphere_p = (1.0f-ftime)*p0 + ftime*p1;
  
  const Vec3fa v = ray->org-sphere_p;
  const float A = dot(ray->dir,ray->dir);
  const float B = 2.0f*dot(v,ray->dir);
  const float C = dot(v,v) - sqr(sphere.r);
  const float D = B*B - 4.0f*A*C;
  if (D < 0.0f) return;
  const float Q = sqrt(D);
  const float rcpA = rcp(A);
  const float t0 = 0.5f*rcpA*(-B-Q);
  const float t1 = 0.5f*rcpA*(-B+Q);
  if ((ray->tnear() < t0) & (t0 < ray->tfar)) {
    ray->tfar = neg_inf;
    valid[0] = -1;
  }
  if ((ray->tnear() < t1) & (t1 < ray->tfar)) {
    ray->tfar = neg_inf;
    valid[0] = -1;
  }
}

Sphere* addUserGeometrySphere (RTCScene scene, const Vec3fa& p, float r, unsigned int num_time_steps)
{
  RTCGeometry geom = rtcNewGeometry(g_device, RTC_GEOMETRY_TYPE_USER);
  Sphere* sphere = (Sphere*) alignedUSMMalloc(sizeof(Sphere),16);
  sphere->p = p;
  sphere->r = r;
  sphere->geomID = rtcAttachGeometry(scene,geom);
  sphere->num_time_steps = num_time_steps;
  rtcSetGeometryUserPrimitiveCount(geom,1);
  rtcSetGeometryTimeStepCount(geom,num_time_steps);
  rtcSetGeometryUserData(geom,sphere);
  rtcSetGeometryBoundsFunction(geom,sphereBoundsFunc,nullptr);
#if !USE_ARGUMENT_CALLBACKS
  rtcSetGeometryIntersectFunction(geom,sphereIntersectFuncPtr);
  rtcSetGeometryOccludedFunction (geom,sphereOccludedFuncPtr);
#endif
  rtcCommitGeometry(geom);
  rtcReleaseGeometry(geom);
  return sphere;
}

/* adds a ground plane to the scene */
unsigned int addGroundPlane (RTCScene scene)
{
  /* create a triangulated plane with 2 triangles and 4 vertices */
  RTCGeometry geom = rtcNewGeometry (g_device, RTC_GEOMETRY_TYPE_TRIANGLE);

  /* set vertices */
  Vertex* vertices = (Vertex*) rtcSetNewGeometryBuffer(geom,RTC_BUFFER_TYPE_VERTEX,0,RTC_FORMAT_FLOAT3,sizeof(Vertex),4);
  vertices[0].x = -10; vertices[0].y = -2; vertices[0].z = -10;
  vertices[1].x = -10; vertices[1].y = -2; vertices[1].z = +15;
  vertices[2].x = +10; vertices[2].y = -2; vertices[2].z = -10;
  vertices[3].x = +10; vertices[3].y = -2; vertices[3].z = +15;

  /* set triangles */
  Triangle* triangles = (Triangle*) rtcSetNewGeometryBuffer(geom,RTC_BUFFER_TYPE_INDEX,0,RTC_FORMAT_UINT3,sizeof(Triangle),2);
  triangles[0].v0 = 0; triangles[0].v1 = 1; triangles[0].v2 = 2;
  triangles[1].v0 = 1; triangles[1].v1 = 3; triangles[1].v2 = 2;

  rtcCommitGeometry(geom);
  unsigned int geomID = rtcAttachGeometry(scene,geom);
  rtcReleaseGeometry(geom);
  return geomID;
}

/* called by the C++ code for initialization */
extern "C" void device_init (char* cfg)
{
  sphereIntersectFuncPtr = GET_FUNCTION_POINTER(sphereIntersectFuncN);
  sphereOccludedFuncPtr  = GET_FUNCTION_POINTER(sphereOccludedFuncN);
  
  /* create scene */
  TutorialData_Constructor(&data);
  g_scene = data.g_scene = rtcNewScene(g_device);
  data.g_time = g_time;

  /* add geometry to the scene */
  addTriangleCube(g_scene,Vec3fa(-5,1,-5),g_num_time_steps);
  addTriangleCube(g_scene,Vec3fa(-5,5,-5),g_num_time_steps2);

  addQuadCube    (g_scene,Vec3fa( 0,1,-5),g_num_time_steps);
  addQuadCube    (g_scene,Vec3fa( 0,5,-5),g_num_time_steps2);

  addQuadCube    (g_scene,Vec3fa( +5,1,-5),g_num_time_steps);
  addQuadCube    (g_scene,Vec3fa( +5,5,-5),g_num_time_steps2);
  //addSubdivCube  (g_scene,Vec3fa(+5,1,-5),g_num_time_steps);
  //addSubdivCube  (g_scene,Vec3fa(+5,5,-5),g_num_time_steps2);

  addLines       (g_scene,Vec3fa(-5,1, 0),g_num_time_steps);
  addLines       (g_scene,Vec3fa(-5,5, 0),g_num_time_steps2);

  addCurve (g_scene,Vec3fa( 0,1, 0),RTC_GEOMETRY_TYPE_FLAT_BSPLINE_CURVE,g_num_time_steps);
  addCurve (g_scene,Vec3fa( 0,5, 0),RTC_GEOMETRY_TYPE_FLAT_BSPLINE_CURVE,g_num_time_steps2);

  addCurve (g_scene,Vec3fa(+5,1, 0),RTC_GEOMETRY_TYPE_ROUND_BSPLINE_CURVE,g_num_time_steps);
  addCurve (g_scene,Vec3fa(+5,5, 0),RTC_GEOMETRY_TYPE_ROUND_BSPLINE_CURVE,g_num_time_steps2);

  data.scene0 = addInstancedTriangleCube(g_scene,Vec3fa(-5,1,+5),g_num_time_steps);
  data.scene1 = addInstancedTriangleCube(g_scene,Vec3fa(-5,5,+5),g_num_time_steps2);

  data.scene2 = addInstancedQuadCube    (g_scene,Vec3fa( 0,1,+5),g_num_time_steps);
  data.scene3 = addInstancedQuadCube    (g_scene,Vec3fa( 0,5,+5),g_num_time_steps2);

  data.sphere0 = addUserGeometrySphere   (g_scene,Vec3fa(+5,1,+5),1.0f,g_num_time_steps);
  data.sphere1 = addUserGeometrySphere   (g_scene,Vec3fa(+5,5,+5),1.0f,g_num_time_steps2);

  addSphere(g_scene, Vec3fa(-5, 1, +10), RTC_GEOMETRY_TYPE_ORIENTED_DISC_POINT, g_num_time_steps);
  addSphere(g_scene, Vec3fa(-5, 5, +10), RTC_GEOMETRY_TYPE_ORIENTED_DISC_POINT, g_num_time_steps2);
  addSphere(g_scene, Vec3fa( 0, 1, +10), RTC_GEOMETRY_TYPE_DISC_POINT, g_num_time_steps);
  addSphere(g_scene, Vec3fa( 0, 5, +10), RTC_GEOMETRY_TYPE_DISC_POINT, g_num_time_steps2);
  addSphere(g_scene, Vec3fa(+5, 1, +10), RTC_GEOMETRY_TYPE_SPHERE_POINT, g_num_time_steps);
  addSphere(g_scene, Vec3fa(+5, 5, +10), RTC_GEOMETRY_TYPE_SPHERE_POINT, g_num_time_steps2);

  addGroundPlane(g_scene);

  /* commit changes to scene */
  rtcCommitScene (g_scene);
  data.g_traversable = rtcGetSceneTraversable(data.g_scene);
}

/* task that renders a single screen tile */
Vec3fa renderPixel(const TutorialData& data, float x, float y, const ISPCCamera& camera, RayStats& stats)
{
  float time = abs((int)(0.01f*data.frameID) - 0.01f*data.frameID);
  if (data.g_time != -1) time = data.g_time;

  /* initialize ray */
  Ray ray(Vec3fa(camera.xfm.p), Vec3fa(normalize(x*camera.xfm.l.vx + y*camera.xfm.l.vy + camera.xfm.l.vz)), 0.0f, inf, time);

  /* intersect ray with scene */
  RTCIntersectArguments iargs;
  rtcInitIntersectArguments(&iargs);
  iargs.feature_mask = (RTCFeatureFlags) (FEATURE_MASK);
#if USE_ARGUMENT_CALLBACKS
  iargs.intersect = sphereIntersectFuncN;
#endif
  
  rtcTraversableIntersect1(data.g_traversable,RTCRayHit_(ray),&iargs);
  RayStats_addRay(stats);

  /* shade pixels */
  Vec3fa color = Vec3fa(0.0f);
  if (ray.geomID != RTC_INVALID_GEOMETRY_ID)
  {
    Vec3fa diffuse = Vec3fa(0.5f,0.5f,0.5f);
    if (ray.instID[0] == RTC_INVALID_GEOMETRY_ID)
      ray.instID[0] = ray.geomID;
    switch (ray.instID[0] / 2) {
    case 0: diffuse = data.face_colors[ray.primID]; break;
    case 1: diffuse = data.face_colors[2*ray.primID]; break;
    case 2: diffuse = data.face_colors[2*ray.primID]; break;

    case 3: diffuse = Vec3fa(0.5f,0.0f,0.0f); break;
    case 4: diffuse = Vec3fa(0.0f,0.5f,0.0f); break;
    case 5: diffuse = Vec3fa(0.0f,0.0f,0.5f); break;

    case 6: diffuse = data.face_colors[ray.primID]; break;
    case 7: diffuse = data.face_colors[2*ray.primID]; break;
    case 8: diffuse = Vec3fa(0.5f,0.5f,0.0f); break;
    default: diffuse = Vec3fa(0.5f,0.5f,0.5f); break;
    }
    color = color + diffuse*0.5f;
    Vec3fa lightDir = normalize(Vec3fa(-1,-4,-1));

    /* initialize shadow ray */
    Ray shadow(ray.org + ray.tfar*ray.dir, neg(lightDir), 0.001f, inf, time);

    /* trace shadow ray */
    RTCOccludedArguments sargs;
    rtcInitOccludedArguments(&sargs);
    sargs.feature_mask = (RTCFeatureFlags) (FEATURE_MASK);
#if USE_ARGUMENT_CALLBACKS
    sargs.occluded = sphereOccludedFuncN;
#endif
    rtcTraversableOccluded1(data.g_traversable,RTCRay_(shadow),&sargs);
    RayStats_addShadowRay(stats);

    /* add light contribution */
    if (shadow.tfar >= 0.0f)
      color = color + diffuse*clamp(-dot(lightDir,normalize(ray.Ng)),0.0f,1.0f);
  }
  return color;
}

void renderPixelStandard(const TutorialData& data, int x, int y,
                  int* pixels,
                  const unsigned int width,
                  const unsigned int height,
                  const float time,
                  const ISPCCamera& camera, RayStats& stats)
{
  /* calculate pixel color */
  Vec3fa color = renderPixel(data,(float)x,(float)y,camera,stats);
  
  /* write color to framebuffer */
  Vec3ff accu_color = data.g_accu[y*width+x] + Vec3ff(color.x,color.y,color.z,1.0f); data.g_accu[y*width+x] = accu_color;
  float f = rcp(max(0.001f,accu_color.w));
  unsigned int r = (unsigned int) (255.0f * clamp(accu_color.x*f,0.0f,1.0f));
  unsigned int g = (unsigned int) (255.0f * clamp(accu_color.y*f,0.0f,1.0f));
  unsigned int b = (unsigned int) (255.0f * clamp(accu_color.z*f,0.0f,1.0f));
  pixels[y*width+x] = (b << 16) + (g << 8) + r;
}

/* renders a single screen tile */
void renderTileStandard(int taskIndex,
                        int threadIndex,
                        int* pixels,
                        const unsigned int width,
                        const unsigned int height,
                        const float time,
                        const ISPCCamera& camera,
                        const int numTilesX,
                        const int numTilesY)
{
  const unsigned int tileY = taskIndex / numTilesX;
  const unsigned int tileX = taskIndex - tileY * numTilesX;
  const unsigned int x0 = tileX * TILE_SIZE_X;
  const unsigned int x1 = min(x0+TILE_SIZE_X,width);
  const unsigned int y0 = tileY * TILE_SIZE_Y;
  const unsigned int y1 = min(y0+TILE_SIZE_Y,height);

  for (unsigned int y=y0; y<y1; y++) for (unsigned int x=x0; x<x1; x++)
  {
    renderPixelStandard(data,x,y,pixels,width,height,time,camera,g_stats[threadIndex]);
  }
}

/* task that renders a single screen tile */
void renderTileTask (int taskIndex, int threadIndex, int* pixels,
                         const unsigned int width,
                         const unsigned int height,
                         const float time,
                         const ISPCCamera& camera,
                         const int numTilesX,
                         const int numTilesY)
{
  renderTileStandard(taskIndex,threadIndex,pixels,width,height,time,camera,numTilesX,numTilesY);
}

extern "C" void renderFrameStandard (int* pixels,
                          const unsigned int width,
                          const unsigned int height,
                          const float time,
                          const ISPCCamera& camera)
{
  /* render next frame */
#if defined(EMBREE_SYCL_TUTORIAL) && !defined(EMBREE_SYCL_RT_SIMULATION)
  TutorialData ldata = data;
  sycl::event event = global_gpu_queue->submit([=](sycl::handler& cgh){
    const sycl::nd_range<2> nd_range = make_nd_range(height,width);
    cgh.parallel_for(nd_range,[=](sycl::nd_item<2> item) {
      const unsigned int x = item.get_global_id(1); if (x >= width ) return;
      const unsigned int y = item.get_global_id(0); if (y >= height) return;
      RayStats stats;
      renderPixelStandard(ldata,x,y,pixels,width,height,time,camera,stats);
    });
  });
  global_gpu_queue->wait_and_throw();

  const auto t0 = event.template get_profiling_info<sycl::info::event_profiling::command_start>();
  const auto t1 = event.template get_profiling_info<sycl::info::event_profiling::command_end>();
  const double dt = (t1-t0)*1E-9;
  ((ISPCCamera*)&camera)->render_time = dt;
#else
  const int numTilesX = (width +TILE_SIZE_X-1)/TILE_SIZE_X;
  const int numTilesY = (height+TILE_SIZE_Y-1)/TILE_SIZE_Y;
  parallel_for(size_t(0),size_t(numTilesX*numTilesY),[&](const range<size_t>& range) {
    const int threadIndex = (int)TaskScheduler::threadIndex();
    for (size_t i=range.begin(); i<range.end(); i++)
      renderTileTask((int)i,threadIndex,pixels,width,height,time,camera,numTilesX,numTilesY);
  }); 
#endif
}

/* called by the C++ code to render */
extern "C" void device_render (int* pixels,
                           const unsigned int width,
                           const unsigned int height,
                           const float time,
                           const ISPCCamera& camera)
{
  /* create accumulator */
  if (data.g_accu_width != width || data.g_accu_height != height) {
    alignedUSMFree(data.g_accu);
    data.g_accu = (Vec3ff*) alignedUSMMalloc((width*height)*sizeof(Vec3ff),16,EMBREE_USM_SHARED_DEVICE_READ_WRITE);
    data.g_accu_width = width;
    data.g_accu_height = height;
    for (unsigned int i=0; i<width*height; i++)
      data.g_accu[i] = Vec3ff(0.0f);
  }

  /* reset accumulator */
  bool camera_changed = g_changed; g_changed = false;
  camera_changed |= ne(data.g_accu_vx,camera.xfm.l.vx); data.g_accu_vx = camera.xfm.l.vx;
  camera_changed |= ne(data.g_accu_vy,camera.xfm.l.vy); data.g_accu_vy = camera.xfm.l.vy;
  camera_changed |= ne(data.g_accu_vz,camera.xfm.l.vz); data.g_accu_vz = camera.xfm.l.vz;
  camera_changed |= ne(data.g_accu_p, camera.xfm.p);    data.g_accu_p  = camera.xfm.p;
  //camera_changed = true;
  if (camera_changed) {
    data.g_accu_count=0;
    for (unsigned int i=0; i<width*height; i++)
      data.g_accu[i] = Vec3ff(0.0f);
  }

  /* render next frame */
  data.frameID++;
}

/* called by the C++ code for cleanup */
extern "C" void device_cleanup ()
{
  TutorialData_Destructor(&data);
}

} // namespace embree
