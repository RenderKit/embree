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

#include "../common/tutorial/tutorial_device.isph"

/* scene data */
RTCDevice g_device = NULL;
RTCScene g_scene = NULL;
uniform Vec3f face_colors[12];

/* accumulation buffer */
uniform Vec3fa* uniform g_accu = NULL;
uniform unsigned int g_accu_width = 0;
uniform unsigned int g_accu_height = 0;
uniform unsigned int g_accu_count = 0;
uniform Vec3fa g_accu_vx;
uniform Vec3fa g_accu_vy;
uniform Vec3fa g_accu_vz;
uniform Vec3fa g_accu_p;
extern uniform bool g_changed;
extern uniform float g_time;
extern uniform unsigned int g_num_time_steps;
extern uniform unsigned int g_num_time_steps2;


__aligned(16) uniform float cube_vertices[8][4] =
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
  1, 5, 4,  0, 1, 4,
  2, 6, 5,  1, 2, 5,
  3, 7, 6,  2, 3, 6,
  4, 7, 3,  0, 4, 3,
  5, 6, 7,  4, 5, 7,
  3, 2, 1,  0, 3, 1
};

uniform unsigned int cube_quad_indices[24] = {
  0, 1, 5, 4,
  1, 2, 6, 5,
  2, 3, 7, 6,
  0, 4, 7, 3,
  4, 5, 6, 7,
  0, 3, 2, 1,
};

__aligned(16) uniform float cube_vertex_crease_weights[8] = {
  inf, inf,inf, inf, inf, inf, inf, inf
};

__aligned(16) uniform unsigned int cube_vertex_crease_indices[8] = {
  0,1,2,3,4,5,6,7
};

__aligned(16) uniform float cube_edge_crease_weights[12] = {
  inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf
};

__aligned(16) uniform unsigned int cube_edge_crease_indices[24] =
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

/* adds a cube to the scene */
uniform unsigned int addTriangleCube (RTCScene scene, const uniform Vec3f& pos, uniform unsigned int num_time_steps)
{
  /* create a triangulated cube with 12 triangles and 8 vertices */
  uniform unsigned int geomID = rtcNewTriangleMesh (scene, RTC_GEOMETRY_STATIC, 12, 8, num_time_steps);
  rtcSetBuffer(scene, geomID, RTC_INDEX_BUFFER,  cube_triangle_indices , 0, 3*sizeof(uniform unsigned int));

  for (uniform size_t t=0; t<num_time_steps; t++)
  {
    uniform RTCBufferType bufID = (uniform RTCBufferType)(RTC_VERTEX_BUFFER0+t);
    uniform Vec3fa* uniform vertices = (uniform Vec3fa* uniform) rtcMapBuffer(scene,geomID,bufID);

    uniform AffineSpace3f rotation = make_AffineSpace3f_rotate(make_Vec3f(0,0,0),make_Vec3f(0,1,0),2.0f*M_PI*(float)t/(float)(num_time_steps-1));
    uniform AffineSpace3f scale = make_AffineSpace3f_scale(make_Vec3f(2.0f,1.0f,1.0f));

    for (uniform int i=0; i<8; i++) {
      uniform Vec3f v = make_Vec3f(cube_vertices[i][0],cube_vertices[i][1],cube_vertices[i][2]);
      vertices[i] = make_Vec3fa(xfmPoint(rotation*scale,v)+pos);
    }
    rtcUnmapBuffer(scene,geomID,bufID);
  }

  /* create face color array */
  face_colors[0] = make_Vec3f(1,0,0);
  face_colors[1] = make_Vec3f(1,0,0);
  face_colors[2] = make_Vec3f(0,1,0);
  face_colors[3] = make_Vec3f(0,1,0);
  face_colors[4] = make_Vec3f(0.5f);
  face_colors[5] = make_Vec3f(0.5f);
  face_colors[6] = make_Vec3f(1.0f);
  face_colors[7] = make_Vec3f(1.0f);
  face_colors[8] = make_Vec3f(0,0,1);
  face_colors[9] = make_Vec3f(0,0,1);
  face_colors[10] = make_Vec3f(1,1,0);
  face_colors[11] = make_Vec3f(1,1,0);
  return geomID;
}

/* adds a cube to the scene */
uniform unsigned int addQuadCube (RTCScene scene, const uniform Vec3f& pos, uniform unsigned int num_time_steps)
{
  /* create a quad cube with 6 quads and 8 vertices */
  uniform unsigned int geomID = rtcNewQuadMesh (scene, RTC_GEOMETRY_STATIC, 6, 8, num_time_steps);
  rtcSetBuffer(scene, geomID, RTC_INDEX_BUFFER,  cube_quad_indices , 0, 4*sizeof(uniform unsigned int));

  for (uniform size_t t=0; t<num_time_steps; t++)
  {
    uniform RTCBufferType bufID = (uniform RTCBufferType)(RTC_VERTEX_BUFFER0+t);
    uniform Vec3fa* uniform vertices = (uniform Vec3fa* uniform) rtcMapBuffer(scene,geomID,bufID);

    uniform AffineSpace3f rotation = make_AffineSpace3f_rotate(make_Vec3f(0,0,0),make_Vec3f(0,1,0),2.0f*M_PI*(float)t/(float)(num_time_steps-1));
    uniform AffineSpace3f scale = make_AffineSpace3f_scale(make_Vec3f(2.0f,1.0f,1.0f));

    for (uniform int i=0; i<8; i++) {
      uniform Vec3f v = make_Vec3f(cube_vertices[i][0],cube_vertices[i][1],cube_vertices[i][2]);
      vertices[i] = make_Vec3fa(xfmPoint(rotation*scale,v)+pos);
    }
    rtcUnmapBuffer(scene,geomID,bufID);
  }

  return geomID;
}

/* adds a subdivision cube to the scene */
uniform unsigned int addSubdivCube (RTCScene scene, const uniform Vec3f& pos, uniform unsigned int num_time_steps)
{
  /* create a triangulated cube with 6 quads and 8 vertices */
  uniform unsigned int geomID = rtcNewSubdivisionMesh(scene, RTC_GEOMETRY_STATIC, NUM_FACES, NUM_INDICES, 8, 0, 0, 0, num_time_steps);

  //rtcSetBuffer(scene, geomID, RTC_VERTEX_BUFFER, cube_vertices,  0, sizeof(uniform Vec3fa  ));
  rtcSetBuffer(scene, geomID, RTC_INDEX_BUFFER,  cube_quad_indices, 0, sizeof(uniform unsigned int));
  rtcSetBuffer(scene, geomID, RTC_FACE_BUFFER,   cube_quad_faces,0, sizeof(uniform unsigned int));

  rtcSetBuffer(scene, geomID, RTC_EDGE_CREASE_INDEX_BUFFER,   cube_edge_crease_indices,  0, 2*sizeof(uniform unsigned int));
  rtcSetBuffer(scene, geomID, RTC_EDGE_CREASE_WEIGHT_BUFFER,  cube_edge_crease_weights,  0, sizeof(uniform float));

  rtcSetBuffer(scene, geomID, RTC_VERTEX_CREASE_INDEX_BUFFER, cube_vertex_crease_indices,0, sizeof(uniform unsigned int));
  rtcSetBuffer(scene, geomID, RTC_VERTEX_CREASE_WEIGHT_BUFFER,cube_vertex_crease_weights,0, sizeof(uniform float));

  uniform float* uniform level = (uniform float* uniform) rtcMapBuffer(scene, geomID, RTC_LEVEL_BUFFER);
  for (uniform size_t i=0; i<NUM_INDICES; i++) level[i] = 16.0f;
  rtcUnmapBuffer(scene, geomID, RTC_LEVEL_BUFFER);

  for (uniform size_t t=0; t<num_time_steps; t++)
  {
    uniform RTCBufferType bufID = (uniform RTCBufferType)(RTC_VERTEX_BUFFER0+t);
    uniform Vec3fa* uniform vertices = (uniform Vec3fa* uniform) rtcMapBuffer(scene,geomID,bufID);

    uniform AffineSpace3f rotation = make_AffineSpace3f_rotate(make_Vec3f(0,0,0),make_Vec3f(0,1,0),2.0f*M_PI*(float)t/(float)(num_time_steps-1));
    uniform AffineSpace3f scale = make_AffineSpace3f_scale(make_Vec3f(2.0f,1.0f,1.0f));

    for (uniform int i=0; i<8; i++) {
      uniform Vec3f v = make_Vec3f(cube_vertices[i][0],cube_vertices[i][1],cube_vertices[i][2]);
      vertices[i] = make_Vec3fa(xfmPoint(rotation*scale,v)+pos);
    }
    rtcUnmapBuffer(scene,geomID,bufID);
  }

  return geomID;
}

/* add hair geometry */
uniform unsigned int addCurveOrHair (RTCScene scene, const uniform Vec3f& pos, bool curve, uniform unsigned int num_time_steps)
{
  uniform unsigned int geomID = 0;
  if (curve)
    geomID = rtcNewBSplineCurveGeometry (scene, RTC_GEOMETRY_STATIC, 13, 16, num_time_steps);
  else
  {
    geomID = rtcNewBSplineHairGeometry (scene, RTC_GEOMETRY_STATIC, 13, 16, num_time_steps);
    rtcSetTessellationRate (scene,geomID,16.0f);
  }

  uniform Vec3f* uniform bspline = uniform new uniform Vec3f[16];
  for (uniform int i=0; i<16; i++) {
    uniform float f = (uniform float)(i)/16.0f;
    bspline[i] = make_Vec3f(2.0f*f-1.0f,sin(12.0f*f),cos(12.0f*f));
  }

  for (uniform size_t t=0; t<num_time_steps; t++)
  {
    uniform RTCBufferType bufID = (uniform RTCBufferType)(RTC_VERTEX_BUFFER0+t);
    uniform Vec3fa* uniform vertices = (uniform Vec3fa* uniform) rtcMapBuffer(scene,geomID,bufID);

    uniform AffineSpace3f rotation = make_AffineSpace3f_rotate(make_Vec3f(0,0,0),make_Vec3f(0,1,0),2.0f*M_PI*(float)t/(float)(num_time_steps-1));
    uniform AffineSpace3f scale = make_AffineSpace3f_scale(make_Vec3f(2.0f,1.0f,1.0f));

    for (uniform int i=0; i<16; i++)
      vertices[i] = make_Vec3fa(xfmPoint(rotation*scale,bspline[i])+pos,0.2f);

    rtcUnmapBuffer(scene,geomID,bufID);
  }

  uniform int* uniform indices = (uniform int* uniform) rtcMapBuffer(scene,geomID,RTC_INDEX_BUFFER);
  for (uniform int i=0; i<13; i++) indices[i] = i;
  rtcUnmapBuffer(scene,geomID,RTC_INDEX_BUFFER);

  delete[] bspline;
  return geomID;
}

/* add line geometry */
uniform unsigned int addLines (RTCScene scene, const uniform Vec3f& pos, uniform unsigned int num_time_steps)
{
  uniform unsigned int geomID = rtcNewLineSegments (scene, RTC_GEOMETRY_STATIC, 15, 16, num_time_steps);

  uniform Vec3f* uniform bspline = uniform new uniform Vec3f[16];
  for (uniform int i=0; i<16; i++) {
    uniform float f = (uniform float)(i)/16.0f;
    bspline[i] = make_Vec3f(2.0f*f-1.0f,sin(12.0f*f),cos(12.0f*f));
  }

  for (uniform size_t t=0; t<num_time_steps; t++)
  {
    uniform RTCBufferType bufID = (uniform RTCBufferType)(RTC_VERTEX_BUFFER0+t);
    uniform Vec3fa* uniform vertices = (uniform Vec3fa* uniform) rtcMapBuffer(scene,geomID,bufID);

    uniform AffineSpace3f rotation = make_AffineSpace3f_rotate(make_Vec3f(0,0,0),make_Vec3f(0,1,0),2.0f*M_PI*(float)t/(float)(num_time_steps-1));
    uniform AffineSpace3f scale = make_AffineSpace3f_scale(make_Vec3f(2.0f,1.0f,1.0f));

    for (uniform int i=0; i<16; i++)
      vertices[i] = make_Vec3fa(xfmPoint(rotation*scale,bspline[i])+pos,0.2f);

    rtcUnmapBuffer(scene,geomID,bufID);
  }

  uniform int* uniform indices = (uniform int* uniform) rtcMapBuffer(scene,geomID,RTC_INDEX_BUFFER);
  for (uniform int i=0; i<15; i++) indices[i] = i;
  rtcUnmapBuffer(scene,geomID,RTC_INDEX_BUFFER);

  delete[] bspline;
  return geomID;
}

/* adds an instanced triangle cube to the scene, rotate instance */
RTCScene addInstancedTriangleCube (RTCScene global_scene, const uniform Vec3f& pos, uniform unsigned int num_time_steps)
{
  RTCScene scene = rtcDeviceNewScene(g_device, RTC_SCENE_STATIC,RTC_INTERSECT_UNIFORM | RTC_INTERSECT_VARYING);
  uniform unsigned int meshID = rtcNewTriangleMesh (scene, RTC_GEOMETRY_STATIC, 12, 8, 1);
  rtcSetBuffer(scene, meshID, RTC_INDEX_BUFFER,  cube_triangle_indices , 0, 3*sizeof(uniform unsigned int));
  rtcSetBuffer(scene, meshID, RTC_VERTEX_BUFFER, cube_vertices, 0, 4*sizeof(uniform float));
  rtcCommit(scene);

  uniform unsigned int instID = rtcNewInstance2(global_scene,scene,num_time_steps);

  for (uniform size_t t=0; t<num_time_steps; t++)
  {
    uniform AffineSpace3f rotation = make_AffineSpace3f_rotate(make_Vec3f(0,0,0),make_Vec3f(0,1,0),2.0f*M_PI*(float)t/(float)(num_time_steps-1));
    uniform AffineSpace3f scale = make_AffineSpace3f_scale(make_Vec3f(2.0f,1.0f,1.0f));
    uniform AffineSpace3f translation = make_AffineSpace3f_translate(pos);
    uniform AffineSpace3f xfm = translation*rotation*scale;
    rtcSetTransform2(global_scene,instID,RTC_MATRIX_COLUMN_MAJOR,(uniform float* uniform)&xfm,t);
  }
  return scene;
}

/* adds an instanced quad cube to the scene, rotate instance and geometry */
RTCScene addInstancedQuadCube (RTCScene global_scene, const uniform Vec3f& pos, uniform unsigned int num_time_steps)
{
  RTCScene scene = rtcDeviceNewScene(g_device, RTC_SCENE_STATIC,RTC_INTERSECT_UNIFORM | RTC_INTERSECT_VARYING);
  uniform unsigned int geomID = rtcNewQuadMesh (scene, RTC_GEOMETRY_STATIC, 6, 8, num_time_steps);
  rtcSetBuffer(scene, geomID, RTC_INDEX_BUFFER,  cube_quad_indices , 0, 4*sizeof(uniform unsigned int));

  for (uniform size_t t=0; t<num_time_steps; t++)
  {
    uniform RTCBufferType bufID = (uniform RTCBufferType)(RTC_VERTEX_BUFFER0+t);
    uniform Vec3fa* uniform vertices = (uniform Vec3fa* uniform) rtcMapBuffer(scene,geomID,bufID);

    uniform AffineSpace3f rotation = make_AffineSpace3f_rotate(make_Vec3f(0,0,0),make_Vec3f(0,1,0),0.5f*2.0f*M_PI*(float)t/(float)(num_time_steps-1));
    uniform AffineSpace3f scale = make_AffineSpace3f_scale(make_Vec3f(2.0f,1.0f,1.0f));

    for (uniform int i=0; i<8; i++) {
      uniform Vec3f v = make_Vec3f(cube_vertices[i][0],cube_vertices[i][1],cube_vertices[i][2]);
      vertices[i] = make_Vec3fa(xfmPoint(rotation*scale,v));
    }
    rtcUnmapBuffer(scene,geomID,bufID);
  }
  rtcCommit(scene);

  uniform unsigned int instID = rtcNewInstance2(global_scene,scene,num_time_steps);

  for (uniform size_t t=0; t<num_time_steps; t++)
  {
    uniform AffineSpace3f rotation = make_AffineSpace3f_rotate(make_Vec3f(0,0,0),make_Vec3f(0,1,0),0.5f*2.0f*M_PI*(float)t/(float)(num_time_steps-1));
    uniform AffineSpace3f translation = make_AffineSpace3f_translate(pos);
    uniform AffineSpace3f xfm = translation*rotation;
    rtcSetTransform2(global_scene,instID,RTC_MATRIX_COLUMN_MAJOR,(uniform float* uniform)&xfm,t);
  }
  return scene;
}

// ======================================================================== //
//                     User defined sphere geometry                         //
// ======================================================================== //

struct Sphere
{
  ALIGNED_STRUCT
  Vec3f p;                      //!< position of the sphere
  float r;                      //!< radius of the sphere
  unsigned int geomID;
  unsigned int num_time_steps;
};

unmasked void sphereBoundsFunc(void* uniform userPtr, void* uniform spheres_i, uniform uintptr_t item, uniform uintptr_t time, uniform RTCBounds& bounds_o)
{
  const uniform Sphere* uniform spheres = (const uniform Sphere* uniform) spheres_i;
  const uniform Sphere& sphere = spheres[item];
  uniform float ft = 2.0f*M_PI * (uniform float) time / (uniform float) (sphere.num_time_steps-1);
  uniform Vec3f p = sphere.p + make_Vec3f(cos(ft),0.0f,sin(ft));
  bounds_o.lower_x = p.x-sphere.r;
  bounds_o.lower_y = p.y-sphere.r;
  bounds_o.lower_z = p.z-sphere.r;
  bounds_o.upper_x = p.x+sphere.r;
  bounds_o.upper_y = p.y+sphere.r;
  bounds_o.upper_z = p.z+sphere.r;
}

void sphereIntersectFunc(void* uniform spheres_i, varying RTCRay& ray, uniform uintptr_t item)
{
  const uniform Sphere* uniform spheres = (const uniform Sphere* uniform) spheres_i;
  const uniform Sphere& sphere = spheres[item];

  const int time_segments = sphere.num_time_steps-1;
  const float time = ray.time*(float)(time_segments);
  const int itime = clamp((int)(floor(time)),(varying int)0,time_segments-1);
  const float ftime = time - (float)(itime);
  const float ft0 = 2.0f*M_PI * (float) (itime+0) / (float) (sphere.num_time_steps-1);
  const float ft1 = 2.0f*M_PI * (float) (itime+1) / (float) (sphere.num_time_steps-1);
  const Vec3f p0 = sphere.p + make_Vec3f(cos(ft0),0.0f,sin(ft0));
  const Vec3f p1 = sphere.p + make_Vec3f(cos(ft1),0.0f,sin(ft1));
  const Vec3f sphere_p = (1.0f-ftime)*p0 + ftime*p1;

  const Vec3f v = ray.org-sphere_p;
  const float A = dot(ray.dir,ray.dir);
  const float B = 2.0f*dot(v,ray.dir);
  const float C = dot(v,v) - sqr(sphere.r);
  const float D = B*B - 4.0f*A*C;
  if (D < 0.0f) return;
  const float Q = sqrt(D);
  const float rcpA = rcp(A);
  const float t0 = 0.5f*rcpA*(-B-Q);
  const float t1 = 0.5f*rcpA*(-B+Q);
  if ((ray.tnear < t0) & (t0 < ray.tfar)) {
    ray.u = 0.0f;
    ray.v = 0.0f;
    ray.tfar = t0;
    ray.geomID = sphere.geomID;
    ray.primID = (unsigned int) item;
    ray.Ng = ray.org+t0*ray.dir-sphere_p;
  }
  if ((ray.tnear < t1) & (t1 < ray.tfar)) {
    ray.u = 0.0f;
    ray.v = 0.0f;
    ray.tfar = t1;
    ray.geomID = sphere.geomID;
    ray.primID = (unsigned int) item;
    ray.Ng = ray.org+t1*ray.dir-sphere_p;
  }
}

void sphereOccludedFunc(void* uniform spheres_i, varying RTCRay& ray, uniform uintptr_t item)
{
  const uniform Sphere* uniform spheres = (const uniform Sphere* uniform) spheres_i;
  const uniform Sphere& sphere = spheres[item];

  const int time_segments = sphere.num_time_steps-1;
  const float time = ray.time*(float)(time_segments);
  const int itime = clamp((int)(floor(time)),(varying int)0,time_segments-1);
  const float ftime = time - (float)(itime);
  const float ft0 = 2.0f*M_PI * (float) (itime+0) / (float) (sphere.num_time_steps-1);
  const float ft1 = 2.0f*M_PI * (float) (itime+1) / (float) (sphere.num_time_steps-1);
  const Vec3f p0 = sphere.p + make_Vec3f(cos(ft0),0.0f,sin(ft0));
  const Vec3f p1 = sphere.p + make_Vec3f(cos(ft1),0.0f,sin(ft1));
  const Vec3f sphere_p = (1.0f-ftime)*p0 + ftime*p1;

  const Vec3f v = ray.org-sphere_p;
  const float A = dot(ray.dir,ray.dir);
  const float B = 2.0f*dot(v,ray.dir);
  const float C = dot(v,v) - sqr(sphere.r);
  const float D = B*B - 4.0f*A*C;
  if (D < 0.0f) return;
  const float Q = sqrt(D);
  const float rcpA = rcp(A);
  const float t0 = 0.5f*rcpA*(-B-Q);
  const float t1 = 0.5f*rcpA*(-B+Q);
  if ((ray.tnear < t0) & (t0 < ray.tfar)) {
    ray.geomID = 0;
  }
  if ((ray.tnear < t1) & (t1 < ray.tfar)) {
    ray.geomID = 0;
  }
}

uniform Sphere* uniform addUserGeometrySphere (RTCScene scene, const uniform Vec3f& p, uniform float r, uniform unsigned int num_time_steps)
{
  uniform unsigned int geomID = rtcNewUserGeometry2(scene,1,num_time_steps);
  uniform Sphere* uniform sphere = uniform new uniform Sphere;
  sphere->p = p;
  sphere->r = r;
  sphere->geomID = geomID;
  sphere->num_time_steps = num_time_steps;
  rtcSetUserData(scene,geomID,sphere);
  rtcSetBoundsFunction3(scene,geomID,sphereBoundsFunc,NULL);
  rtcSetIntersectFunction(scene,geomID,sphereIntersectFunc);
  rtcSetOccludedFunction (scene,geomID,sphereOccludedFunc);
  return sphere;
}

/* adds a ground plane to the scene */
uniform unsigned int addGroundPlane (RTCScene scene)
{
  /* create a triangulated plane with 2 triangles and 4 vertices */
  uniform unsigned int mesh = rtcNewTriangleMesh (scene, RTC_GEOMETRY_STATIC, 2, 4);

  /* set vertices */
  uniform Vertex* uniform vertices = (uniform Vertex* uniform) rtcMapBuffer(scene,mesh,RTC_VERTEX_BUFFER);
  vertices[0].x = -10; vertices[0].y = -2; vertices[0].z = -10;
  vertices[1].x = -10; vertices[1].y = -2; vertices[1].z = +10;
  vertices[2].x = +10; vertices[2].y = -2; vertices[2].z = -10;
  vertices[3].x = +10; vertices[3].y = -2; vertices[3].z = +10;
  rtcUnmapBuffer(scene,mesh,RTC_VERTEX_BUFFER);

  /* set triangles */
  uniform Triangle* uniform triangles = (uniform Triangle* uniform) rtcMapBuffer(scene,mesh,RTC_INDEX_BUFFER);
  triangles[0].v0 = 0; triangles[0].v1 = 2; triangles[0].v2 = 1;
  triangles[1].v0 = 1; triangles[1].v1 = 2; triangles[1].v2 = 3;
  rtcUnmapBuffer(scene,mesh,RTC_INDEX_BUFFER);

  return mesh;
}

RTCScene scene0 = NULL;
RTCScene scene1 = NULL;
RTCScene scene2 = NULL;
RTCScene scene3 = NULL;
uniform Sphere* uniform sphere0 = NULL;
uniform Sphere* uniform sphere1 = NULL;

/* called by the C++ code for initialization */
export void device_init (uniform int8* uniform cfg)
{
  /* initialize last seen camera */
  g_accu_vx = make_Vec3fa(0.0f);
  g_accu_vy = make_Vec3fa(0.0f);
  g_accu_vz = make_Vec3fa(0.0f);
  g_accu_p  = make_Vec3fa(0.0f);

  /* create new Embree device */
  g_device = rtcNewDevice(cfg);
  error_handler(NULL,rtcDeviceGetError(g_device));

  /* set error handler */
  rtcDeviceSetErrorFunction2(g_device,error_handler,NULL);

  /* create scene */
  g_scene = rtcDeviceNewScene(g_device, RTC_SCENE_STATIC,RTC_INTERSECT_UNIFORM | RTC_INTERSECT_VARYING);

  /* add geometry to the scene */
  addTriangleCube(g_scene,make_Vec3f(-5,1,-5),g_num_time_steps);
  addTriangleCube(g_scene,make_Vec3f(-5,5,-5),g_num_time_steps2);

  addQuadCube    (g_scene,make_Vec3f( 0,1,-5),g_num_time_steps);
  addQuadCube    (g_scene,make_Vec3f( 0,5,-5),g_num_time_steps2);

  addSubdivCube  (g_scene,make_Vec3f(+5,1,-5),g_num_time_steps);
  addSubdivCube  (g_scene,make_Vec3f(+5,5,-5),g_num_time_steps2);

  addLines       (g_scene,make_Vec3f(-5,1, 0),g_num_time_steps);
  addLines       (g_scene,make_Vec3f(-5,5, 0),g_num_time_steps2);

  addCurveOrHair (g_scene,make_Vec3f( 0,1, 0),false,g_num_time_steps);
  addCurveOrHair (g_scene,make_Vec3f( 0,5, 0),false,g_num_time_steps2);

  addCurveOrHair (g_scene,make_Vec3f(+5,1, 0),true,g_num_time_steps);
  addCurveOrHair (g_scene,make_Vec3f(+5,5, 0),true,g_num_time_steps2);

  scene0 = addInstancedTriangleCube(g_scene,make_Vec3f(-5,1,+5),g_num_time_steps);
  scene1 = addInstancedTriangleCube(g_scene,make_Vec3f(-5,5,+5),g_num_time_steps2);

  scene2 = addInstancedQuadCube    (g_scene,make_Vec3f( 0,1,+5),g_num_time_steps);
  scene3 = addInstancedQuadCube    (g_scene,make_Vec3f( 0,5,+5),g_num_time_steps2);

  sphere0 = addUserGeometrySphere   (g_scene,make_Vec3f(+5,1,+5),1.0f,g_num_time_steps);
  sphere1 = addUserGeometrySphere   (g_scene,make_Vec3f(+5,5,+5),1.0f,g_num_time_steps2);

  addGroundPlane(g_scene);

  /* commit changes to scene */
  rtcCommit (g_scene);

  /* set start render mode */
  renderTile = renderTileStandard;
  key_pressed_handler = device_key_pressed_default;
}

uniform int frameID = 50;

/* task that renders a single screen tile */
Vec3f renderPixelStandard(float x, float y, const uniform ISPCCamera& camera, uniform RayStats& stats)
{
  uniform float time = abs((uniform int)(0.01f*frameID) - 0.01f*frameID);
  if (g_time != -1) time = g_time;

  /* initialize ray */
  RTCRay ray;
  ray.org = make_Vec3f(camera.xfm.p);
  ray.dir = make_Vec3f(normalize(x*camera.xfm.l.vx + y*camera.xfm.l.vy + camera.xfm.l.vz));
  ray.tnear = 0.0f;
  ray.tfar = inf;
  ray.geomID = RTC_INVALID_GEOMETRY_ID;
  ray.primID = RTC_INVALID_GEOMETRY_ID;
  ray.instID = RTC_INVALID_GEOMETRY_ID;
  ray.mask = -1;
  ray.time = time;

  /* intersect ray with scene */
  rtcIntersect(g_scene,ray);
  RayStats_addRay(stats);

  /* shade pixels */
  Vec3f color = make_Vec3f(0.0f);
  if (ray.geomID != RTC_INVALID_GEOMETRY_ID)
  {
    Vec3f diffuse = make_Vec3f(0.5f,0.5f,0.5f);
    if (ray.instID == RTC_INVALID_GEOMETRY_ID)
      ray.instID = ray.geomID;
    switch (ray.instID / 2) {
    case 0: diffuse = face_colors[ray.primID]; break;
    case 1: diffuse = face_colors[2*ray.primID]; break;
    case 2: diffuse = face_colors[2*ray.primID]; break;

    case 3: diffuse = make_Vec3f(0.5f,0.0f,0.0f); break;
    case 4: diffuse = make_Vec3f(0.0f,0.5f,0.0f); break;
    case 5: diffuse = make_Vec3f(0.0f,0.0f,0.5f); break;

    case 6: diffuse = face_colors[ray.primID]; break;
    case 7: diffuse = face_colors[2*ray.primID]; break;
    case 8: diffuse = make_Vec3f(0.5f,0.5f,0.0f); break;
    default: diffuse = make_Vec3f(0.5f,0.5f,0.5f); break;
    }
    color = color + diffuse*0.5f;
    Vec3f lightDir = normalize(make_Vec3f(-1,-4,-1));

    /* initialize shadow ray */
    RTCRay shadow;
    shadow.org = ray.org + ray.tfar*ray.dir;
    shadow.dir = neg(lightDir);
    shadow.tnear = 0.001f;
    shadow.tfar = inf;
    shadow.geomID = 1;
    shadow.primID = 0;
    shadow.instID = RTC_INVALID_GEOMETRY_ID;
    shadow.mask = -1;
    shadow.time = time;

    /* trace shadow ray */
    rtcOccluded(g_scene,shadow);
    RayStats_addShadowRay(stats);

    /* add light contribution */
    if (shadow.geomID)
      color = color + diffuse*clamp(-dot(lightDir,normalize(ray.Ng)),0.0f,1.0f);
  }
  return color;
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
    /* calculate pixel color */
    Vec3f color = renderPixelStandard((float)x,(float)y,camera,g_stats[threadIndex]);

    /* write color to framebuffer */
    Vec3fa accu_color = g_accu[y*width+x] + make_Vec3fa(color.x,color.y,color.z,1.0f); g_accu[y*width+x] = accu_color;
    float f = rcp(max(0.001f,accu_color.w));
    unsigned int r = (unsigned int) (255.0f * clamp(accu_color.x*f,0.0f,1.0f));
    unsigned int g = (unsigned int) (255.0f * clamp(accu_color.y*f,0.0f,1.0f));
    unsigned int b = (unsigned int) (255.0f * clamp(accu_color.z*f,0.0f,1.0f));
    pixels[y*width+x] = (b << 16) + (g << 8) + r;
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
  renderTile(taskIndex,threadIndex,pixels,width,height,time,camera,numTilesX,numTilesY);
}

/* called by the C++ code to render */
export void device_render (uniform int* uniform pixels,
                           const uniform unsigned int width,
                           const uniform unsigned int height,
                           const uniform float time,
                           const uniform ISPCCamera& camera)
{
  /* create accumulator */
  if (g_accu_width != width || g_accu_height != height) {
    delete[] g_accu;
    g_accu = uniform new uniform Vec3fa[width*height];
    g_accu_width = width;
    g_accu_height = height;
    for (uniform size_t i=0; i<width*height; i++)
      g_accu[i] = make_Vec3fa(0.0f);
  }

  /* reset accumulator */
  uniform bool camera_changed = g_changed; g_changed = false;
  camera_changed |= ne(g_accu_vx,camera.xfm.l.vx); g_accu_vx = camera.xfm.l.vx;
  camera_changed |= ne(g_accu_vy,camera.xfm.l.vy); g_accu_vy = camera.xfm.l.vy;
  camera_changed |= ne(g_accu_vz,camera.xfm.l.vz); g_accu_vz = camera.xfm.l.vz;
  camera_changed |= ne(g_accu_p, camera.xfm.p);    g_accu_p  = camera.xfm.p;
  //camera_changed = true;
  if (camera_changed) {
    g_accu_count=0;
    for (uniform size_t i=0; i<width*height; i++)
      g_accu[i] = make_Vec3fa(0.0f);
  }

  /* render next frame */
  frameID++;
  const uniform int numTilesX = (width +TILE_SIZE_X-1)/TILE_SIZE_X;
  const uniform int numTilesY = (height+TILE_SIZE_Y-1)/TILE_SIZE_Y;
  launch[numTilesX*numTilesY] renderTileTask(pixels,width,height,time,camera,numTilesX,numTilesY); sync;
}

/* called by the C++ code for cleanup */
export void device_cleanup ()
{
  delete sphere0; sphere0 = NULL;
  delete sphere1; sphere1 = NULL;
  rtcDeleteScene(scene0); scene0 = NULL;
  rtcDeleteScene(scene1); scene1 = NULL;
  rtcDeleteScene(scene2); scene2 = NULL;
  rtcDeleteScene(scene3); scene3 = NULL;
  rtcDeleteScene (g_scene); g_scene = NULL;
  rtcDeleteDevice(g_device); g_device = NULL;
  delete[] g_accu; g_accu = NULL;
  g_accu_width = 0;
  g_accu_height = 0;
  g_accu_count = 0;
}
