// Copyright 2009-2021 Intel Corporation
// SPDX-License-Identifier: Apache-2.0

#include "motion_blur_geometry_device.isph"

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
RTCScene g_scene = NULL;
uniform TutorialData data;

extern uniform bool g_changed;
extern uniform float g_time;
extern uniform unsigned int g_num_time_steps;
extern uniform unsigned int g_num_time_steps2;


uniform float cube_vertices[8][4] =
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

uniform unsigned int cube_triangle_indices[36] = {
  1, 4, 5,  0, 4, 1,
  2, 5, 6,  1, 5, 2,
  3, 6, 7,  2, 6, 3,
  4, 3, 7,  0, 3, 4,
  5, 7, 6,  4, 7, 5,
  3, 1, 2,  0, 1, 3
};

uniform unsigned int cube_quad_indices[24] = {
  0, 4, 5, 1,
  1, 5, 6, 2,
  2, 6, 7, 3,
  0, 3, 7, 4,
  4, 7, 6, 5,
  0, 1, 2, 3,
};

uniform float cube_vertex_crease_weights[8] = {
  inf, inf,inf, inf, inf, inf, inf, inf
};

uniform unsigned int cube_vertex_crease_indices[8] = {
  0,1,2,3,4,5,6,7
};

uniform float cube_edge_crease_weights[12] = {
  inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf
};

uniform unsigned int cube_edge_crease_indices[24] =
{
  0,1, 1,2, 2,3, 3,0,
  4,5, 5,6, 6,7, 7,4,
  0,4, 1,5, 2,6, 3,7,
};

#define NUM_INDICES 24
#define NUM_FACES 6
#define FACE_SIZE 4

uniform unsigned int cube_quad_faces[6] = {
  4, 4, 4, 4, 4, 4
};

uniform unsigned int addSphere(RTCScene scene, const uniform Vec3f& pos, uniform RTCGeometryType type, uniform unsigned int num_time_steps)
{
  RTCGeometry geom = rtcNewGeometry(g_device, type);
  rtcSetGeometryTimeStepCount(geom, num_time_steps);

  for (uniform unsigned int t = 0; t < num_time_steps; t++)
  {
    uniform RTCBufferType bufType = RTC_BUFFER_TYPE_VERTEX;
    uniform Vec3ff *vertex = (uniform Vec3ff*)rtcSetNewGeometryBuffer(geom, bufType, t, RTC_FORMAT_FLOAT4, sizeof(Vec3ff), 1);
    uniform AffineSpace3f rotation = make_AffineSpace3f_rotate(make_Vec3f(0,0,0), make_Vec3f(0,1,0), 2.0f*M_PI*(float)t/(float)(num_time_steps-1));
    *vertex = make_Vec3ff(xfmPoint(rotation, make_Vec3f(1, 0, 0)) + pos);
    vertex->w = 1.f;

    if (type == RTC_GEOMETRY_TYPE_ORIENTED_DISC_POINT) {
      uniform Vec3fa *normal = (uniform Vec3fa*)rtcSetNewGeometryBuffer(geom, RTC_BUFFER_TYPE_NORMAL, t, RTC_FORMAT_FLOAT3, sizeof(Vec3fa), 1);
      normal[0] = make_Vec3fa(1, 1, 0);
      normal[0] = normalize(normal[0]);
    }
  }

  rtcCommitGeometry(geom);
  uniform unsigned int geomID = rtcAttachGeometry(scene, geom);
  rtcReleaseGeometry(geom);
  return geomID;
}

/* adds a cube to the scene */
uniform unsigned int addTriangleCube (RTCScene scene, const uniform Vec3f& pos, uniform unsigned int num_time_steps)
{
  /* create a triangulated cube with 12 triangles and 8 vertices */
  RTCGeometry geom = rtcNewGeometry (g_device, RTC_GEOMETRY_TYPE_TRIANGLE);
  rtcSetGeometryTimeStepCount(geom,num_time_steps);
  rtcSetSharedGeometryBuffer(geom, RTC_BUFFER_TYPE_INDEX, 0, RTC_FORMAT_UINT3, data.cube_triangle_indices, 0, 3*sizeof(uniform unsigned int), 12);

  for (uniform unsigned int t=0; t<num_time_steps; t++)
  {
    uniform RTCBufferType bufType = RTC_BUFFER_TYPE_VERTEX;
    uniform Vec3fa* uniform vertices = (uniform Vec3fa* uniform) rtcSetNewGeometryBuffer(geom,bufType,t,RTC_FORMAT_FLOAT3,sizeof(uniform Vec3fa), 8);

    uniform AffineSpace3f rotation = make_AffineSpace3f_rotate(make_Vec3f(0,0,0),make_Vec3f(0,1,0),2.0f*M_PI*(float)t/(float)(num_time_steps-1));
    uniform AffineSpace3f scale = make_AffineSpace3f_scale(make_Vec3f(2.0f,1.0f,1.0f));

    for (uniform int i=0; i<8; i++) {
      uniform Vec3f v = make_Vec3f(cube_vertices[i][0],cube_vertices[i][1],cube_vertices[i][2]);
      vertices[i] = make_Vec3fa(xfmPoint(rotation*scale,v)+pos);
    }
  }

  /* create face color array */
  data.face_colors[0] = make_Vec3f(1,0,0);
  data.face_colors[1] = make_Vec3f(1,0,0);
  data.face_colors[2] = make_Vec3f(0,1,0);
  data.face_colors[3] = make_Vec3f(0,1,0);
  data.face_colors[4] = make_Vec3f(0.5f);
  data.face_colors[5] = make_Vec3f(0.5f);
  data.face_colors[6] = make_Vec3f(1.0f);
  data.face_colors[7] = make_Vec3f(1.0f);
  data.face_colors[8] = make_Vec3f(0,0,1);
  data.face_colors[9] = make_Vec3f(0,0,1);
  data.face_colors[10] = make_Vec3f(1,1,0);
  data.face_colors[11] = make_Vec3f(1,1,0);

  rtcCommitGeometry(geom);
  uniform unsigned int geomID = rtcAttachGeometry(scene,geom);
  rtcReleaseGeometry(geom);
  return geomID;
}

/* adds a cube to the scene */
uniform unsigned int addQuadCube (RTCScene scene, const uniform Vec3f& pos, uniform unsigned int num_time_steps)
{
  /* create a quad cube with 6 quads and 8 vertices */
  RTCGeometry geom = rtcNewGeometry (g_device, RTC_GEOMETRY_TYPE_QUAD);
  rtcSetGeometryTimeStepCount(geom,num_time_steps);
  rtcSetSharedGeometryBuffer(geom, RTC_BUFFER_TYPE_INDEX, 0, RTC_FORMAT_UINT4, data.cube_quad_indices, 0, 4*sizeof(uniform unsigned int), 6);

  for (uniform unsigned int t=0; t<num_time_steps; t++)
  {
    uniform RTCBufferType bufType = RTC_BUFFER_TYPE_VERTEX;
    uniform Vec3fa* uniform vertices = (uniform Vec3fa* uniform) rtcSetNewGeometryBuffer(geom,bufType,t,RTC_FORMAT_FLOAT3,sizeof(uniform Vec3fa), 8);

    uniform AffineSpace3f rotation = make_AffineSpace3f_rotate(make_Vec3f(0,0,0),make_Vec3f(0,1,0),2.0f*M_PI*(float)t/(float)(num_time_steps-1));
    uniform AffineSpace3f scale = make_AffineSpace3f_scale(make_Vec3f(2.0f,1.0f,1.0f));

    for (uniform int i=0; i<8; i++) {
      uniform Vec3f v = make_Vec3f(cube_vertices[i][0],cube_vertices[i][1],cube_vertices[i][2]);
      vertices[i] = make_Vec3fa(xfmPoint(rotation*scale,v)+pos);
    }
  }

  rtcCommitGeometry(geom);
  uniform unsigned int geomID = rtcAttachGeometry(scene,geom);
  rtcReleaseGeometry(geom);
  return geomID;
}

/* adds a subdivision cube to the scene */
uniform unsigned int addSubdivCube (RTCScene scene, const uniform Vec3f& pos, uniform unsigned int num_time_steps)
{
  /* create a triangulated cube with 6 quads and 8 vertices */
  RTCGeometry geom = rtcNewGeometry(g_device, RTC_GEOMETRY_TYPE_SUBDIVISION);
  rtcSetGeometryTimeStepCount(geom,num_time_steps);

  //rtcSetSharedGeometryBuffer(geom, RTC_BUFFER_TYPE_VERTEX, 0, RTC_FORMAT_FLOAT3, cube_vertices, 0, sizeof(uniform Vec3fa), 8);
  rtcSetSharedGeometryBuffer(geom, RTC_BUFFER_TYPE_INDEX, 0, RTC_FORMAT_UINT, cube_quad_indices, 0, sizeof(uniform unsigned int), NUM_INDICES);
  rtcSetSharedGeometryBuffer(geom, RTC_BUFFER_TYPE_FACE,  0, RTC_FORMAT_UINT, cube_quad_faces,   0, sizeof(uniform unsigned int), NUM_FACES);

  rtcSetSharedGeometryBuffer(geom, RTC_BUFFER_TYPE_EDGE_CREASE_INDEX,  0, RTC_FORMAT_UINT2, cube_edge_crease_indices,  0, 2*sizeof(uniform unsigned int), 0);
  rtcSetSharedGeometryBuffer(geom, RTC_BUFFER_TYPE_EDGE_CREASE_WEIGHT, 0, RTC_FORMAT_FLOAT, cube_edge_crease_weights,  0, sizeof(uniform float),          0);

  rtcSetSharedGeometryBuffer(geom, RTC_BUFFER_TYPE_VERTEX_CREASE_INDEX,  0, RTC_FORMAT_UINT,  cube_vertex_crease_indices,0, sizeof(uniform unsigned int), 0);
  rtcSetSharedGeometryBuffer(geom, RTC_BUFFER_TYPE_VERTEX_CREASE_WEIGHT, 0, RTC_FORMAT_FLOAT, cube_vertex_crease_weights,0, sizeof(uniform float),        0);

  uniform float* uniform level = (uniform float* uniform) rtcSetNewGeometryBuffer(geom, RTC_BUFFER_TYPE_LEVEL, 0, RTC_FORMAT_FLOAT, sizeof(uniform float), NUM_INDICES);
  for (uniform unsigned int i=0; i<NUM_INDICES; i++) level[i] = 16.0f;

  for (uniform unsigned int t=0; t<num_time_steps; t++)
  {
    uniform RTCBufferType bufType = RTC_BUFFER_TYPE_VERTEX;
    uniform Vec3fa* uniform vertices = (uniform Vec3fa* uniform) rtcSetNewGeometryBuffer(geom,bufType,t,RTC_FORMAT_FLOAT3,sizeof(uniform Vec3fa),8);

    uniform AffineSpace3f rotation = make_AffineSpace3f_rotate(make_Vec3f(0,0,0),make_Vec3f(0,1,0),2.0f*M_PI*(float)t/(float)(num_time_steps-1));
    uniform AffineSpace3f scale = make_AffineSpace3f_scale(make_Vec3f(2.0f,1.0f,1.0f));

    for (uniform int i=0; i<8; i++) {
      uniform Vec3f v = make_Vec3f(cube_vertices[i][0],cube_vertices[i][1],cube_vertices[i][2]);
      vertices[i] = make_Vec3fa(xfmPoint(rotation*scale,v)+pos);
    }
  }

  rtcCommitGeometry(geom);
  uniform unsigned int geomID = rtcAttachGeometry(scene,geom);
  rtcReleaseGeometry(geom);
  return geomID;
}

/* add hair geometry */
uniform unsigned int addCurve (RTCScene scene, const uniform Vec3f& pos, uniform RTCGeometryType type, uniform unsigned int num_time_steps)
{
  RTCGeometry geom = rtcNewGeometry(g_device, type);
  rtcSetGeometryTimeStepCount(geom,num_time_steps);
  rtcSetGeometryTessellationRate (geom,16.0f);

  uniform Vec3f* uniform bspline = uniform new uniform Vec3f[16];
  for (uniform int i=0; i<16; i++) {
    uniform float f = (uniform float)(i)/16.0f;
    bspline[i] = make_Vec3f(2.0f*f-1.0f,sin(12.0f*f),cos(12.0f*f));
  }

  for (uniform unsigned int t=0; t<num_time_steps; t++)
  {
    uniform RTCBufferType bufType = RTC_BUFFER_TYPE_VERTEX;
    uniform Vec3ff* uniform vertices = (uniform Vec3ff* uniform) rtcSetNewGeometryBuffer(geom,bufType,t,RTC_FORMAT_FLOAT4,sizeof(uniform Vec3ff),16);

    uniform AffineSpace3f rotation = make_AffineSpace3f_rotate(make_Vec3f(0,0,0),make_Vec3f(0,1,0),2.0f*M_PI*(float)t/(float)(num_time_steps-1));
    uniform AffineSpace3f scale = make_AffineSpace3f_scale(make_Vec3f(2.0f,1.0f,1.0f));

    for (uniform int i=0; i<16; i++)
      vertices[i] = make_Vec3ff(xfmPoint(rotation*scale,bspline[i])+pos,0.2f);
  }

  uniform int* uniform indices = (uniform int* uniform) rtcSetNewGeometryBuffer(geom,RTC_BUFFER_TYPE_INDEX,0,RTC_FORMAT_UINT,sizeof(uniform int),13);
  for (uniform int i=0; i<13; i++) indices[i] = i;

  delete[] bspline;

  rtcCommitGeometry(geom);
  uniform unsigned int geomID = rtcAttachGeometry(scene,geom);
  rtcReleaseGeometry(geom);
  return geomID;
}

/* add line geometry */
uniform unsigned int addLines (RTCScene scene, const uniform Vec3f& pos, uniform unsigned int num_time_steps)
{
  RTCGeometry geom = rtcNewGeometry (g_device, RTC_GEOMETRY_TYPE_FLAT_LINEAR_CURVE);
  rtcSetGeometryTimeStepCount(geom,num_time_steps);

  uniform Vec3f* uniform bspline = uniform new uniform Vec3f[16];
  for (uniform int i=0; i<16; i++) {
    uniform float f = (uniform float)(i)/16.0f;
    bspline[i] = make_Vec3f(2.0f*f-1.0f,sin(12.0f*f),cos(12.0f*f));
  }

  for (uniform unsigned int t=0; t<num_time_steps; t++)
  {
    uniform RTCBufferType bufType = RTC_BUFFER_TYPE_VERTEX;
    uniform Vec3ff* uniform vertices = (uniform Vec3ff* uniform) rtcSetNewGeometryBuffer(geom,bufType,t,RTC_FORMAT_FLOAT4,sizeof(uniform Vec3ff),16);

    uniform AffineSpace3f rotation = make_AffineSpace3f_rotate(make_Vec3f(0,0,0),make_Vec3f(0,1,0),2.0f*M_PI*(float)t/(float)(num_time_steps-1));
    uniform AffineSpace3f scale = make_AffineSpace3f_scale(make_Vec3f(2.0f,1.0f,1.0f));

    for (uniform int i=0; i<16; i++)
      vertices[i] = make_Vec3ff(xfmPoint(rotation*scale,bspline[i])+pos,0.2f);
  }

  uniform int* uniform indices = (uniform int* uniform) rtcSetNewGeometryBuffer(geom,RTC_BUFFER_TYPE_INDEX,0,RTC_FORMAT_UINT,sizeof(uniform int),15);
  for (uniform int i=0; i<15; i++) indices[i] = i;

  delete[] bspline;

  rtcCommitGeometry(geom);
  uniform unsigned int geomID = rtcAttachGeometry(scene,geom);
  rtcReleaseGeometry(geom);
  return geomID;
}

/* adds an instanced triangle cube to the scene, rotate instance */
RTCScene addInstancedTriangleCube (RTCScene global_scene, const uniform Vec3f& pos, uniform unsigned int num_time_steps)
{
  RTCScene scene = rtcNewScene(g_device);
  RTCGeometry geom = rtcNewGeometry (g_device, RTC_GEOMETRY_TYPE_TRIANGLE);
  rtcSetSharedGeometryBuffer(geom, RTC_BUFFER_TYPE_INDEX,  0, RTC_FORMAT_UINT3,  data.cube_triangle_indices, 0, 3*sizeof(uniform unsigned int), 12);
  rtcSetSharedGeometryBuffer(geom, RTC_BUFFER_TYPE_VERTEX, 0, RTC_FORMAT_FLOAT3, data.cube_vertices, 0, 4*sizeof(uniform float), 8);
  rtcCommitGeometry(geom);
  rtcAttachGeometry(scene,geom);
  rtcReleaseGeometry(geom);
  rtcCommitScene(scene);

  RTCGeometry inst = rtcNewGeometry (g_device, RTC_GEOMETRY_TYPE_INSTANCE);
   rtcSetGeometryInstancedScene(inst,scene);
   rtcSetGeometryTimeStepCount(inst,num_time_steps);
  
  for (uniform unsigned int t=0; t<num_time_steps; t++)
  {
    uniform AffineSpace3f rotation = make_AffineSpace3f_rotate(make_Vec3f(0,0,0),make_Vec3f(0,1,0),2.0f*M_PI*(float)t/(float)(num_time_steps-1));
    uniform AffineSpace3f scale = make_AffineSpace3f_scale(make_Vec3f(2.0f,1.0f,1.0f));
    uniform AffineSpace3f translation = make_AffineSpace3f_translate(pos);
    uniform AffineSpace3f xfm = translation*rotation*scale;
    rtcSetGeometryTransform(inst,t,RTC_FORMAT_FLOAT3X4_COLUMN_MAJOR,(uniform float* uniform)&xfm);
  }

  rtcCommitGeometry(inst);
  rtcAttachGeometry(global_scene,inst);
  rtcReleaseGeometry(inst);
  return scene;
}

/* adds an instanced quad cube to the scene, rotate instance and geometry */
RTCScene addInstancedQuadCube (RTCScene global_scene, const uniform Vec3f& pos, uniform unsigned int num_time_steps)
{
  RTCScene scene = rtcNewScene(g_device);
  RTCGeometry geom = rtcNewGeometry (g_device, RTC_GEOMETRY_TYPE_QUAD);
  rtcSetGeometryTimeStepCount(geom,num_time_steps);
  rtcSetSharedGeometryBuffer(geom, RTC_BUFFER_TYPE_INDEX, 0, RTC_FORMAT_UINT4, data.cube_quad_indices, 0, 4*sizeof(uniform unsigned int), 6);

  for (uniform unsigned int t=0; t<num_time_steps; t++)
  {
    uniform RTCBufferType bufType = RTC_BUFFER_TYPE_VERTEX;
    uniform Vec3fa* uniform vertices = (uniform Vec3fa* uniform) rtcSetNewGeometryBuffer(geom,bufType,t,RTC_FORMAT_FLOAT3,sizeof(uniform Vec3fa),8);

    uniform AffineSpace3f rotation = make_AffineSpace3f_rotate(make_Vec3f(0,0,0),make_Vec3f(0,1,0),0.5f*2.0f*M_PI*(float)t/(float)(num_time_steps-1));
    uniform AffineSpace3f scale = make_AffineSpace3f_scale(make_Vec3f(2.0f,1.0f,1.0f));

    for (uniform int i=0; i<8; i++) {
      uniform Vec3f v = make_Vec3f(cube_vertices[i][0],cube_vertices[i][1],cube_vertices[i][2]);
      vertices[i] = make_Vec3fa(xfmPoint(rotation*scale,v));
    }
  }

  rtcCommitGeometry(geom);
  rtcAttachGeometry(scene,geom);
  rtcReleaseGeometry(geom);  
  rtcCommitScene(scene);

  RTCGeometry inst = rtcNewGeometry (g_device, RTC_GEOMETRY_TYPE_INSTANCE);
   rtcSetGeometryInstancedScene(inst,scene);
   rtcSetGeometryTimeStepCount(inst,num_time_steps);

  for (uniform unsigned int t=0; t<num_time_steps; t++)
  {
    uniform AffineSpace3f rotation = make_AffineSpace3f_rotate(make_Vec3f(0,0,0),make_Vec3f(0,1,0),0.5f*2.0f*M_PI*(float)t/(float)(num_time_steps-1));
    uniform AffineSpace3f translation = make_AffineSpace3f_translate(pos);
    uniform AffineSpace3f xfm = translation*rotation;
    rtcSetGeometryTransform(inst,t,RTC_FORMAT_FLOAT3X4_COLUMN_MAJOR,(uniform float* uniform)&xfm);
  }

  rtcCommitGeometry(inst);
  rtcAttachGeometry(global_scene,inst);
  rtcReleaseGeometry(inst);
  return scene;
}

// ======================================================================== //
//                     User defined sphere geometry                         //
// ======================================================================== //

unmasked void sphereBoundsFunc(const struct RTCBoundsFunctionArguments* uniform args)
{
  const uniform Sphere* uniform spheres = (const uniform Sphere* uniform) args->geometryUserPtr;
  uniform RTCBounds* uniform bounds_o = args->bounds_o;
  const uniform unsigned int time = args->timeStep;
  const uniform Sphere& sphere = spheres[args->primID];
  uniform float ft = 2.0f*M_PI * (uniform float) time / (uniform float) (sphere.num_time_steps-1);
  uniform Vec3f p = sphere.p + make_Vec3f(cos(ft),0.0f,sin(ft));
  bounds_o->lower_x = p.x-sphere.r;
  bounds_o->lower_y = p.y-sphere.r;
  bounds_o->lower_z = p.z-sphere.r;
  bounds_o->upper_x = p.x+sphere.r;
  bounds_o->upper_y = p.y+sphere.r;
  bounds_o->upper_z = p.z+sphere.r;
}

uniform RTCIntersectFunctionN sphereIntersectFuncPtr = NULL;

RTC_SYCL_INDIRECTLY_CALLABLE unmasked void sphereIntersectFuncN(const RTCIntersectFunctionNArguments* uniform args)
{
  uniform int* uniform valid = args->valid;
  void* uniform ptr  = args->geometryUserPtr;
  RTCRayHitN* uniform rays = (RTCRayHitN* uniform)args->rayhit;
  uniform unsigned int primID = args->primID;
  assert(args->N == programCount);
  const uniform Sphere* uniform spheres = (const uniform Sphere* uniform)ptr;
  const uniform Sphere& sphere = spheres[primID];

  if (!valid[programIndex]) return;
  valid[programIndex] = 0;
  
  varying Ray *uniform ray = (varying Ray *uniform)rays;
  
  const int time_segments = sphere.num_time_steps-1;
  const float time = ray->time*(float)(time_segments);
  const int itime = clamp((int)(floor(time)),(varying int)0,time_segments-1);
  const float ftime = time - (float)(itime);
  const float ft0 = 2.0f*M_PI * (float) (itime+0) / (float) (sphere.num_time_steps-1);
  const float ft1 = 2.0f*M_PI * (float) (itime+1) / (float) (sphere.num_time_steps-1);
  const Vec3f p0 = sphere.p + make_Vec3f(cos(ft0),0.0f,sin(ft0));
  const Vec3f p1 = sphere.p + make_Vec3f(cos(ft1),0.0f,sin(ft1));
  const Vec3f sphere_p = (1.0f-ftime)*p0 + ftime*p1;
  
  const Vec3f v = ray->org-sphere_p;
  const float A = dot(ray->dir,ray->dir);
  const float B = 2.0f*dot(v,ray->dir);
  const float C = dot(v,v) - sqr(sphere.r);
  const float D = B*B - 4.0f*A*C;
  if (D < 0.0f) return;
  const float Q = sqrt(D);
  const float rcpA = rcp(A);
  const float t0 = 0.5f*rcpA*(-B-Q);
  const float t1 = 0.5f*rcpA*(-B+Q);
  if ((ray->tnear < t0) & (t0 < ray->tfar)) {
    ray->u = 0.0f;
    ray->v = 0.0f;
    ray->tfar = t0;
    ray->geomID = sphere.geomID;
    ray->primID = (unsigned int) primID;
    ray->Ng = ray->org+t0*ray->dir-sphere_p;
    valid[programIndex] = -1;
  }
  if ((ray->tnear < t1) & (t1 < ray->tfar)) {
    ray->u = 0.0f;
    ray->v = 0.0f;
    ray->tfar = t1;
    ray->geomID = sphere.geomID;
    ray->primID = (unsigned int) primID;
    ray->Ng = ray->org+t1*ray->dir-sphere_p;
    valid[programIndex] = -1;
  }
}

uniform RTCOccludedFunctionN sphereOccludedFuncPtr = NULL;

RTC_SYCL_INDIRECTLY_CALLABLE unmasked void sphereOccludedFuncN(const RTCOccludedFunctionNArguments* uniform args)
{
  uniform int* uniform valid = args->valid;
  void* uniform ptr  = args->geometryUserPtr;
  RTCRayHitN* uniform rays = (RTCRayHitN* uniform)args->ray;
  uniform unsigned int primID = args->primID;
  assert(args->N == programCount);
  const uniform Sphere* uniform spheres = (const uniform Sphere* uniform)ptr;
  const uniform Sphere& sphere = spheres[primID];

  if (!valid[programIndex]) return;
  valid[programIndex] = 0;
  
  varying Ray *uniform ray = (varying Ray *uniform)rays;
  const int time_segments = sphere.num_time_steps-1;
  const float time = ray->time*(float)(time_segments);
  const int itime = clamp((int)(floor(time)),(varying int)0,time_segments-1);
  const float ftime = time - (float)(itime);
  const float ft0 = 2.0f*M_PI * (float) (itime+0) / (float) (sphere.num_time_steps-1);
  const float ft1 = 2.0f*M_PI * (float) (itime+1) / (float) (sphere.num_time_steps-1);
  const Vec3f p0 = sphere.p + make_Vec3f(cos(ft0),0.0f,sin(ft0));
  const Vec3f p1 = sphere.p + make_Vec3f(cos(ft1),0.0f,sin(ft1));
  const Vec3f sphere_p = (1.0f-ftime)*p0 + ftime*p1;
  
  const Vec3f v = ray->org-sphere_p;
  const float A = dot(ray->dir,ray->dir);
  const float B = 2.0f*dot(v,ray->dir);
  const float C = dot(v,v) - sqr(sphere.r);
  const float D = B*B - 4.0f*A*C;
  if (D < 0.0f) return;
  const float Q = sqrt(D);
  const float rcpA = rcp(A);
  const float t0 = 0.5f*rcpA*(-B-Q);
  const float t1 = 0.5f*rcpA*(-B+Q);
  if ((ray->tnear < t0) & (t0 < ray->tfar)) {
    ray->tfar = neg_inf;
    valid[programIndex] = -1;
  }
  if ((ray->tnear < t1) & (t1 < ray->tfar)) {
    ray->tfar = neg_inf;
    valid[programIndex] = -1;
  }
}

uniform Sphere* uniform addUserGeometrySphere (RTCScene scene, const uniform Vec3f& p, uniform float r, uniform unsigned int num_time_steps)
{
  RTCGeometry geom = rtcNewGeometry(g_device, RTC_GEOMETRY_TYPE_USER);
  uniform Sphere* uniform sphere = uniform new uniform Sphere;
  sphere->p = p;
  sphere->r = r;
  sphere->geomID = rtcAttachGeometry(scene,geom);
  sphere->num_time_steps = num_time_steps;
  rtcSetGeometryUserPrimitiveCount(geom,1);
  rtcSetGeometryTimeStepCount(geom,num_time_steps);
  rtcSetGeometryUserData(geom,sphere);
  rtcSetGeometryBoundsFunction(geom,sphereBoundsFunc,NULL);
#if !USE_ARGUMENT_CALLBACKS
  rtcSetGeometryIntersectFunction(geom,sphereIntersectFuncPtr);
  rtcSetGeometryOccludedFunction (geom,sphereOccludedFuncPtr);
#endif
  rtcCommitGeometry(geom);
  rtcReleaseGeometry(geom);
  return sphere;
}

/* adds a ground plane to the scene */
uniform unsigned int addGroundPlane (RTCScene scene)
{
  /* create a triangulated plane with 2 triangles and 4 vertices */
  RTCGeometry geom = rtcNewGeometry (g_device, RTC_GEOMETRY_TYPE_TRIANGLE);

  /* set vertices */
  uniform Vertex* uniform vertices = (uniform Vertex* uniform) rtcSetNewGeometryBuffer(geom,RTC_BUFFER_TYPE_VERTEX,0,RTC_FORMAT_FLOAT3,sizeof(uniform Vertex),4);
  vertices[0].x = -10; vertices[0].y = -2; vertices[0].z = -10;
  vertices[1].x = -10; vertices[1].y = -2; vertices[1].z = +15;
  vertices[2].x = +10; vertices[2].y = -2; vertices[2].z = -10;
  vertices[3].x = +10; vertices[3].y = -2; vertices[3].z = +15;

  /* set triangles */
  uniform Triangle* uniform triangles = (uniform Triangle* uniform) rtcSetNewGeometryBuffer(geom,RTC_BUFFER_TYPE_INDEX,0,RTC_FORMAT_UINT3,sizeof(uniform Triangle),2);
  triangles[0].v0 = 0; triangles[0].v1 = 1; triangles[0].v2 = 2;
  triangles[1].v0 = 1; triangles[1].v1 = 3; triangles[1].v2 = 2;

  rtcCommitGeometry(geom);
  uniform unsigned int geomID = rtcAttachGeometry(scene,geom);
  rtcReleaseGeometry(geom);
  return geomID;
}

/* called by the C++ code for initialization */
export void device_init (uniform int8* uniform cfg)
{
  sphereIntersectFuncPtr = GET_FUNCTION_POINTER(sphereIntersectFuncN);
  sphereOccludedFuncPtr  = GET_FUNCTION_POINTER(sphereOccludedFuncN);
  
  /* create scene */
  TutorialData_Constructor(&data);
  g_scene = data.g_scene = rtcNewScene(g_device);
  data.g_time = g_time;

  /* add geometry to the scene */
  addTriangleCube(g_scene,make_Vec3f(-5,1,-5),g_num_time_steps);
  addTriangleCube(g_scene,make_Vec3f(-5,5,-5),g_num_time_steps2);

  addQuadCube    (g_scene,make_Vec3f( 0,1,-5),g_num_time_steps);
  addQuadCube    (g_scene,make_Vec3f( 0,5,-5),g_num_time_steps2);

  addQuadCube    (g_scene,make_Vec3f( +5,1,-5),g_num_time_steps);
  addQuadCube    (g_scene,make_Vec3f( +5,5,-5),g_num_time_steps2);
  //addSubdivCube  (g_scene,make_Vec3f(+5,1,-5),g_num_time_steps);
  //addSubdivCube  (g_scene,make_Vec3f(+5,5,-5),g_num_time_steps2);

  addLines       (g_scene,make_Vec3f(-5,1, 0),g_num_time_steps);
  addLines       (g_scene,make_Vec3f(-5,5, 0),g_num_time_steps2);

  addCurve (g_scene,make_Vec3f( 0,1, 0),RTC_GEOMETRY_TYPE_FLAT_BSPLINE_CURVE,g_num_time_steps);
  addCurve (g_scene,make_Vec3f( 0,5, 0),RTC_GEOMETRY_TYPE_FLAT_BSPLINE_CURVE,g_num_time_steps2);

  addCurve (g_scene,make_Vec3f(+5,1, 0),RTC_GEOMETRY_TYPE_ROUND_BSPLINE_CURVE,g_num_time_steps);
  addCurve (g_scene,make_Vec3f(+5,5, 0),RTC_GEOMETRY_TYPE_ROUND_BSPLINE_CURVE,g_num_time_steps2);

  data.scene0 = addInstancedTriangleCube(g_scene,make_Vec3f(-5,1,+5),g_num_time_steps);
  data.scene1 = addInstancedTriangleCube(g_scene,make_Vec3f(-5,5,+5),g_num_time_steps2);

  data.scene2 = addInstancedQuadCube    (g_scene,make_Vec3f( 0,1,+5),g_num_time_steps);
  data.scene3 = addInstancedQuadCube    (g_scene,make_Vec3f( 0,5,+5),g_num_time_steps2);

  data.sphere0 = addUserGeometrySphere   (g_scene,make_Vec3f(+5,1,+5),1.0f,g_num_time_steps);
  data.sphere1 = addUserGeometrySphere   (g_scene,make_Vec3f(+5,5,+5),1.0f,g_num_time_steps2);

  addSphere(g_scene, make_Vec3f(-5, 1, +10), RTC_GEOMETRY_TYPE_ORIENTED_DISC_POINT, g_num_time_steps);
  addSphere(g_scene, make_Vec3f(-5, 5, +10), RTC_GEOMETRY_TYPE_ORIENTED_DISC_POINT, g_num_time_steps2);
  addSphere(g_scene, make_Vec3f( 0, 1, +10), RTC_GEOMETRY_TYPE_DISC_POINT, g_num_time_steps);
  addSphere(g_scene, make_Vec3f( 0, 5, +10), RTC_GEOMETRY_TYPE_DISC_POINT, g_num_time_steps2);
  addSphere(g_scene, make_Vec3f(+5, 1, +10), RTC_GEOMETRY_TYPE_SPHERE_POINT, g_num_time_steps);
  addSphere(g_scene, make_Vec3f(+5, 5, +10), RTC_GEOMETRY_TYPE_SPHERE_POINT, g_num_time_steps2);

  addGroundPlane(g_scene);

  /* commit changes to scene */
  rtcCommitScene (g_scene);
}

/* task that renders a single screen tile */
Vec3f renderPixel(const uniform TutorialData& data, float x, float y, const uniform ISPCCamera& camera, uniform RayStats& stats)
{
  uniform float time = abs((uniform int)(0.01f*data.frameID) - 0.01f*data.frameID);
  if (data.g_time != -1) time = data.g_time;

  /* initialize ray */
  Ray ray = make_Ray(make_Vec3f(camera.xfm.p), make_Vec3f(normalize(x*camera.xfm.l.vx + y*camera.xfm.l.vy + camera.xfm.l.vz)), 0.0f, inf, time);

  /* intersect ray with scene */
  uniform RTCIntersectArguments iargs;
  rtcInitIntersectArguments(&iargs);
  iargs.feature_mask = (uniform RTCFeatureFlags) (FEATURE_MASK);
#if USE_ARGUMENT_CALLBACKS
  iargs.intersect = sphereIntersectFuncN;
#endif
  
  rtcIntersectV(data.g_scene,RTCRayHit_(ray),&iargs);
  RayStats_addRay(stats);

  /* shade pixels */
  Vec3f color = make_Vec3f(0.0f);
  if (ray.geomID != RTC_INVALID_GEOMETRY_ID)
  {
    Vec3f diffuse = make_Vec3f(0.5f,0.5f,0.5f);
    if (ray.instID[0] == RTC_INVALID_GEOMETRY_ID)
      ray.instID[0] = ray.geomID;
    switch (ray.instID[0] / 2) {
    case 0: diffuse = data.face_colors[ray.primID]; break;
    case 1: diffuse = data.face_colors[2*ray.primID]; break;
    case 2: diffuse = data.face_colors[2*ray.primID]; break;

    case 3: diffuse = make_Vec3f(0.5f,0.0f,0.0f); break;
    case 4: diffuse = make_Vec3f(0.0f,0.5f,0.0f); break;
    case 5: diffuse = make_Vec3f(0.0f,0.0f,0.5f); break;

    case 6: diffuse = data.face_colors[ray.primID]; break;
    case 7: diffuse = data.face_colors[2*ray.primID]; break;
    case 8: diffuse = make_Vec3f(0.5f,0.5f,0.0f); break;
    default: diffuse = make_Vec3f(0.5f,0.5f,0.5f); break;
    }
    color = color + diffuse*0.5f;
    Vec3f lightDir = normalize(make_Vec3f(-1,-4,-1));

    /* initialize shadow ray */
    Ray shadow = make_Ray(ray.org + ray.tfar*ray.dir, neg(lightDir), 0.001f, inf, time);

    /* trace shadow ray */
    uniform RTCOccludedArguments sargs;
    rtcInitOccludedArguments(&sargs);
    sargs.feature_mask = (uniform RTCFeatureFlags) (FEATURE_MASK);
#if USE_ARGUMENT_CALLBACKS
    sargs.occluded = sphereOccludedFuncN;
#endif
    rtcOccludedV(data.g_scene,RTCRay_(shadow),&sargs);
    RayStats_addShadowRay(stats);

    /* add light contribution */
    if (shadow.tfar >= 0.0f)
      color = color + diffuse*clamp(-dot(lightDir,normalize(ray.Ng)),0.0f,1.0f);
  }
  return color;
}

void renderPixelStandard(const uniform TutorialData& data, int x, int y,
                  uniform int* uniform pixels,
                  const uniform unsigned int width,
                  const uniform unsigned int height,
                  const uniform float time,
                  const uniform ISPCCamera& camera, uniform RayStats& stats)
{
  /* calculate pixel color */
  Vec3f color = renderPixel(data,(float)x,(float)y,camera,stats);
  
  /* write color to framebuffer */
  Vec3ff accu_color = data.g_accu[y*width+x] + make_Vec3ff(color.x,color.y,color.z,1.0f); data.g_accu[y*width+x] = accu_color;
  float f = rcp(max(0.001f,accu_color.w));
  unsigned int r = (unsigned int) (255.0f * clamp(accu_color.x*f,0.0f,1.0f));
  unsigned int g = (unsigned int) (255.0f * clamp(accu_color.y*f,0.0f,1.0f));
  unsigned int b = (unsigned int) (255.0f * clamp(accu_color.z*f,0.0f,1.0f));
  pixels[y*width+x] = (b << 16) + (g << 8) + r;
}

/* renders a single screen tile */
void renderTileStandard(uniform int taskIndex,
                        uniform int threadIndex,
                        uniform int* uniform pixels,
                        const uniform unsigned int width,
                        const uniform unsigned int height,
                        const uniform float time,
                        const uniform ISPCCamera& camera,
                        const uniform int numTilesX,
                        const uniform int numTilesY)
{
  const uniform unsigned int tileY = taskIndex / numTilesX;
  const uniform unsigned int tileX = taskIndex - tileY * numTilesX;
  const uniform unsigned int x0 = tileX * TILE_SIZE_X;
  const uniform unsigned int x1 = min(x0+TILE_SIZE_X,width);
  const uniform unsigned int y0 = tileY * TILE_SIZE_Y;
  const uniform unsigned int y1 = min(y0+TILE_SIZE_Y,height);

  foreach_tiled (y = y0 ... y1, x = x0 ... x1)
  {
    renderPixelStandard(data,x,y,pixels,width,height,time,camera,g_stats[threadIndex]);
  }
}

/* task that renders a single screen tile */
task void renderTileTask(uniform int* uniform pixels,
                         const uniform unsigned int width,
                         const uniform unsigned int height,
                         const uniform float time,
                         const uniform ISPCCamera& camera,
                         const uniform int numTilesX,
                         const uniform int numTilesY)
{
  renderTileStandard(taskIndex,threadIndex,pixels,width,height,time,camera,numTilesX,numTilesY);
}

export void renderFrameStandard (uniform int* uniform pixels,
                          const uniform unsigned int width,
                          const uniform unsigned int height,
                          const uniform float time,
                          const uniform ISPCCamera& camera)
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
  const uniform int numTilesX = (width +TILE_SIZE_X-1)/TILE_SIZE_X;
  const uniform int numTilesY = (height+TILE_SIZE_Y-1)/TILE_SIZE_Y;
  launch[numTilesX*numTilesY] renderTileTask(pixels,width,height,time,camera,numTilesX,numTilesY); sync;
#endif
}

/* called by the C++ code to render */
export void device_render (uniform int* uniform pixels,
                           const uniform unsigned int width,
                           const uniform unsigned int height,
                           const uniform float time,
                           const uniform ISPCCamera& camera)
{
  /* create accumulator */
  if (data.g_accu_width != width || data.g_accu_height != height) {
    delete[] data.g_accu;
    data.g_accu = uniform new uniform Vec3ff[width*height]; // EMBREE_USM_SHARED_DEVICE_READ_WRITE
    data.g_accu_width = width;
    data.g_accu_height = height;
    for (uniform unsigned int i=0; i<width*height; i++)
      data.g_accu[i] = make_Vec3ff(0.0f);
  }

  /* reset accumulator */
  uniform bool camera_changed = g_changed; g_changed = false;
  camera_changed |= ne(data.g_accu_vx,camera.xfm.l.vx); data.g_accu_vx = camera.xfm.l.vx;
  camera_changed |= ne(data.g_accu_vy,camera.xfm.l.vy); data.g_accu_vy = camera.xfm.l.vy;
  camera_changed |= ne(data.g_accu_vz,camera.xfm.l.vz); data.g_accu_vz = camera.xfm.l.vz;
  camera_changed |= ne(data.g_accu_p, camera.xfm.p);    data.g_accu_p  = camera.xfm.p;
  //camera_changed = true;
  if (camera_changed) {
    data.g_accu_count=0;
    for (uniform unsigned int i=0; i<width*height; i++)
      data.g_accu[i] = make_Vec3ff(0.0f);
  }

  /* render next frame */
  data.frameID++;
}

/* called by the C++ code for cleanup */
export void device_cleanup ()
{
  TutorialData_Destructor(&data);
}
