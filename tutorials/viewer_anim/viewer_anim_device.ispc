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

#include "../common/math/random_sampler.isph"
#include "../common/tutorial/tutorial_device.isph"
#include "../common/tutorial/scene_device.h"
#include "../common/math/sampling.isph"

#define ANIM_FPS 15.0f
#define ENABLE_ANIM 1
#define VERTEX_NORMALS 0
#define SHADOWS 1
#define VERTEX_INTERPOLATION_BLOCK_SIZE 1024

extern uniform ISPCScene* uniform g_ispc_scene;

/* scene data */
uniform RTCScene g_scene   = NULL;
varying Vec3f* uniform ls_positions = NULL;

/* animation data */
uniform double animTime        = -1.0f; // global time counter


  // ==================================================================================================
  // ==================================================================================================
  // ==================================================================================================

void convertTriangleMesh(uniform ISPCTriangleMesh* uniform mesh, RTCScene scene_out)
{
  /* if more than a single timestep, mark object as dynamic */
  uniform RTCBuildQuality quality = mesh->numTimeSteps > 1 ? RTC_BUILD_QUALITY_LOW : RTC_BUILD_QUALITY_MEDIUM;
  RTCGeometry geom = rtcNewGeometry (g_device, RTC_GEOMETRY_TYPE_TRIANGLE);
  rtcSetGeometryBuildQuality(geom, quality);
  uniform Vec3fa* uniform vertices = (uniform Vec3fa* uniform) rtcSetNewGeometryBuffer(geom, RTC_BUFFER_TYPE_VERTEX, 0, RTC_FORMAT_FLOAT3, sizeof(uniform Vec3fa), mesh->numVertices);
  for (uniform unsigned int i=0;i<mesh->numVertices;i++) vertices[i] = mesh->positions[0][i];
  rtcSetSharedGeometryBuffer(geom, RTC_BUFFER_TYPE_INDEX, 0, RTC_FORMAT_UINT3, mesh->triangles, 0, sizeof(uniform ISPCTriangle), mesh->numTriangles);
  rtcCommitGeometry(geom);
  mesh->geom.geometry = geom;
  mesh->geom.geomID = rtcAttachGeometry(scene_out,geom);
}

void convertQuadMesh(uniform ISPCQuadMesh* uniform mesh, RTCScene scene_out)
{
  /* if more than a single timestep, mark object as dynamic */
  uniform RTCBuildQuality quality = mesh->numTimeSteps > 1 ? RTC_BUILD_QUALITY_LOW : RTC_BUILD_QUALITY_MEDIUM;
  RTCGeometry geom = rtcNewGeometry (g_device, RTC_GEOMETRY_TYPE_QUAD);
  rtcSetGeometryBuildQuality(geom, quality);
  uniform Vec3fa* uniform vertices = (uniform Vec3fa* uniform) rtcSetNewGeometryBuffer(geom, RTC_BUFFER_TYPE_VERTEX, 0, RTC_FORMAT_FLOAT3, sizeof(uniform Vec3fa), mesh->numVertices);
  for (unsigned int i=0;i<mesh->numVertices;i++) vertices[i] = mesh->positions[0][i];
  rtcSetSharedGeometryBuffer(geom, RTC_BUFFER_TYPE_INDEX, 0, RTC_FORMAT_UINT4, mesh->quads, 0, sizeof(uniform ISPCQuad), mesh->numQuads);
  rtcCommitGeometry(geom);
  mesh->geom.geometry = geom;
  mesh->geom.geomID = rtcAttachGeometry(scene_out,geom);
}

void convertSubdivMesh(uniform ISPCSubdivMesh* uniform mesh, RTCScene scene_out)
{
  /* if more than a single timestep, mark object as dynamic */
  uniform RTCBuildQuality quality = mesh->numTimeSteps > 1 ? RTC_BUILD_QUALITY_LOW : RTC_BUILD_QUALITY_MEDIUM;
  RTCGeometry geom = rtcNewGeometry(g_device, RTC_GEOMETRY_TYPE_SUBDIVISION);
  rtcSetGeometryBuildQuality(geom, quality);
  for (unsigned int i=0; i<mesh->numEdges; i++) mesh->subdivlevel[i] = 4.0f;
  uniform Vec3fa* uniform vertices = (uniform Vec3fa* uniform) rtcSetNewGeometryBuffer(geom, RTC_BUFFER_TYPE_VERTEX, 0, RTC_FORMAT_FLOAT3, sizeof(uniform Vec3fa), mesh->numVertices);
  for (unsigned int i=0;i<mesh->numVertices;i++) vertices[i] = mesh->positions[0][i];
  rtcSetSharedGeometryBuffer(geom, RTC_BUFFER_TYPE_LEVEL, 0, RTC_FORMAT_FLOAT, mesh->subdivlevel,      0, sizeof(uniform float),        mesh->numEdges);
  rtcSetSharedGeometryBuffer(geom, RTC_BUFFER_TYPE_INDEX, 0, RTC_FORMAT_UINT,  mesh->position_indices, 0, sizeof(uniform unsigned int), mesh->numEdges);
  rtcSetSharedGeometryBuffer(geom, RTC_BUFFER_TYPE_FACE,  0, RTC_FORMAT_UINT,  mesh->verticesPerFace,  0, sizeof(uniform unsigned int), mesh->numFaces);
  rtcSetSharedGeometryBuffer(geom, RTC_BUFFER_TYPE_HOLE,  0, RTC_FORMAT_UINT,  mesh->holes,            0, sizeof(uniform unsigned int), mesh->numFaces);
  rtcSetSharedGeometryBuffer(geom, RTC_BUFFER_TYPE_EDGE_CREASE_INDEX,    0, RTC_FORMAT_UINT2, mesh->edge_creases,          0, 2*sizeof(uniform unsigned int), mesh->numEdgeCreases);
  rtcSetSharedGeometryBuffer(geom, RTC_BUFFER_TYPE_EDGE_CREASE_WEIGHT,   0, RTC_FORMAT_FLOAT, mesh->edge_crease_weights,   0, sizeof(uniform float),          mesh->numEdgeCreases);
  rtcSetSharedGeometryBuffer(geom, RTC_BUFFER_TYPE_VERTEX_CREASE_INDEX,  0, RTC_FORMAT_UINT,  mesh->vertex_creases,        0, sizeof(uniform unsigned int),   mesh->numVertexCreases);
  rtcSetSharedGeometryBuffer(geom, RTC_BUFFER_TYPE_VERTEX_CREASE_WEIGHT, 0, RTC_FORMAT_FLOAT, mesh->vertex_crease_weights, 0, sizeof(uniform float),          mesh->numVertexCreases);
  rtcSetGeometrySubdivisionMode(geom, 0, mesh->position_subdiv_mode);
  rtcCommitGeometry(geom);
  mesh->geom.geometry = geom;
  mesh->geom.geomID = rtcAttachGeometry(scene_out,geom);
}

void convertCurveGeometry(uniform ISPCHairSet* uniform hair, RTCScene scene_out)
{
  /* if more than a single timestep, mark object as dynamic */
  uniform RTCBuildQuality quality = hair->numTimeSteps > 1 ? RTC_BUILD_QUALITY_LOW : RTC_BUILD_QUALITY_MEDIUM;
  /* create object */
  RTCGeometry geom = rtcNewGeometry (g_device, hair->type);
  rtcSetGeometryBuildQuality(geom, quality);
  /* generate vertex buffer */
  uniform Vec3fa* uniform vertices = (uniform Vec3fa* uniform) rtcSetNewGeometryBuffer(geom, RTC_BUFFER_TYPE_VERTEX, 0, RTC_FORMAT_FLOAT4, sizeof(uniform Vec3fa), hair->numVertices);
  for (unsigned int i=0;i<hair->numVertices;i++) vertices[i] = hair->positions[0][i];
  rtcSetSharedGeometryBuffer(geom, RTC_BUFFER_TYPE_INDEX, 0, RTC_FORMAT_UINT, hair->hairs, 0, sizeof(uniform ISPCHair), hair->numHairs);
  if (hair->type != RTC_GEOMETRY_TYPE_FLAT_LINEAR_CURVE)
    rtcSetGeometryTessellationRate(geom,(float)hair->tessellation_rate);
  rtcCommitGeometry(geom);
  hair->geom.geometry = geom;
  hair->geom.geomID = rtcAttachGeometry(scene_out,geom);
}

uniform unsigned int getNumObjects(uniform ISPCScene* uniform scene_in) {
  return scene_in->numGeometries;
}

uniform RTCScene createScene(uniform ISPCScene* uniform scene_in)
{
  RTCScene scene = rtcNewScene(g_device);
  rtcSetSceneBuildQuality(scene,RTC_BUILD_QUALITY_LOW);
  rtcSetSceneFlags(scene, RTC_SCENE_FLAG_DYNAMIC);
  return scene;
}

void createObject(const uniform unsigned int i, uniform ISPCScene* uniform scene_in, uniform RTCScene scene_out)
{
  uniform ISPCGeometry* uniform geometry = scene_in->geometries[i];
  
  if (geometry->type == SUBDIV_MESH) {
    convertSubdivMesh((uniform ISPCSubdivMesh* uniform) geometry, scene_out);
  }
  else if (geometry->type == TRIANGLE_MESH) {
    convertTriangleMesh((uniform ISPCTriangleMesh* uniform) geometry, scene_out);
  }
  else if (geometry->type == QUAD_MESH) {
    convertQuadMesh((uniform ISPCQuadMesh* uniform) geometry, scene_out);
  }
  else if (geometry->type == CURVES) {
    convertCurveGeometry((uniform ISPCHairSet* uniform) geometry, scene_out);
  }
  else
    assert(false);
}

uniform Vec3fa lerp(const uniform Vec3fa& v0, const uniform Vec3fa& v1, const uniform float t) {
  return v0*(1.0f-t)+v1*t;
}


 task void interpolateVertexBlock(const uniform unsigned int numVertices,
                                  uniform Vec3fa* uniform vertices,
                                  const uniform Vec3fa* uniform const input0,
                                  const uniform Vec3fa* uniform const input1,
                                  const uniform float tt)
 {
   const uniform unsigned int b = taskIndex;
   const uniform unsigned int startID = b*VERTEX_INTERPOLATION_BLOCK_SIZE;
   const uniform unsigned int endID = min(startID + VERTEX_INTERPOLATION_BLOCK_SIZE,numVertices);
   for (uniform unsigned int i=startID; i<endID; i++)
     vertices[i] = lerp(input0[i],input1[i],tt);
 }


void interpolateVertices(RTCGeometry geom,
                         const uniform unsigned int numVertices,
                         const uniform Vec3fa* uniform const input0,
                         const uniform Vec3fa* uniform const input1,
                         const uniform float tt)
  {
    uniform Vec3fa* uniform vertices = (Vec3fa*) rtcGetGeometryBufferData(geom, RTC_BUFFER_TYPE_VERTEX, 0);
#if 1
    const uniform unsigned int blocks = (numVertices+VERTEX_INTERPOLATION_BLOCK_SIZE-1) / VERTEX_INTERPOLATION_BLOCK_SIZE;
    launch[blocks] interpolateVertexBlock(numVertices,vertices,input0,input1,tt); sync;
#else
    for (uniform unsigned int i=0; i<numVertices; i++)
      vertices[i] = lerp(input0[i],input1[i],tt);
#endif
    rtcUpdateGeometryBuffer(geom,RTC_BUFFER_TYPE_VERTEX, 0);
    rtcCommitGeometry(geom);
  }

  void updateVertexData(const uniform unsigned int ID,
                        uniform ISPCScene* uniform scene_in,
                        uniform RTCScene uniform scene_out,
                        const uniform unsigned int keyFrameID,
                        const uniform float tt)
  {
    uniform ISPCGeometry* uniform geometry = scene_in->geometries[ID];

    if (geometry->type == SUBDIV_MESH) {
      /* if static do nothing */
      if (((uniform ISPCSubdivMesh* uniform)geometry)->numTimeSteps <= 1) return;
      rtcCommitGeometry(geometry->geometry);
    }
    else if (geometry->type == TRIANGLE_MESH) {
      uniform ISPCTriangleMesh* uniform mesh = (uniform ISPCTriangleMesh* uniform)geometry;
      /* if static do nothing */
      if (mesh->numTimeSteps <= 1) return;
      /* interpolate two vertices from two timesteps */
      const uniform unsigned int t0 = (keyFrameID+0) % mesh->numTimeSteps;
      const uniform unsigned int t1 = (keyFrameID+1) % mesh->numTimeSteps;
      const uniform Vec3fa* uniform const input0 = mesh->positions[t0];
      const uniform Vec3fa* uniform const input1 = mesh->positions[t1];
      interpolateVertices(geometry->geometry, mesh->numVertices, input0, input1, tt);
    }
    else if (geometry->type == QUAD_MESH) {
      uniform ISPCQuadMesh* uniform mesh = (uniform ISPCQuadMesh* uniform)geometry;
      /* if static do nothing */
      if (mesh->numTimeSteps <= 1) return;
      /* interpolate two vertices from two timesteps */
      const uniform unsigned int t0 = (keyFrameID+0) % mesh->numTimeSteps;
      const uniform unsigned int t1 = (keyFrameID+1) % mesh->numTimeSteps;
      const uniform Vec3fa* uniform const input0 = mesh->positions[t0];
      const uniform Vec3fa* uniform const input1 = mesh->positions[t1];
      interpolateVertices(geometry->geometry, mesh->numVertices, input0, input1, tt);
    }
    else if (geometry->type == CURVES) {
      /* if static do nothing */
      if (((uniform ISPCHairSet* uniform)geometry)->numTimeSteps <= 1) return;
      rtcCommitGeometry(geometry->geometry);
    }
    else
      assert(false);
  }


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

  uniform RayStats& stats = g_stats[threadIndex];

  Ray rays[TILE_SIZE_X*TILE_SIZE_Y];

  /* generate stream of primary rays */
  uniform unsigned int N = 0;
  foreach_tiled (y = y0 ... y1, x = x0 ... x1)
  {
    /* ISPC workaround for mask == 0 */
    if (all(__mask == 0)) continue;

    /* initialize ray */
    Ray& ray = rays[N++];
    bool mask = __mask; unmasked { // invalidates inactive rays
      ray.tnear = mask ? 0.0f         : (float)(pos_inf);
      ray.tfar  = mask ? (float)(inf) : (float)(neg_inf);
    }
    init_Ray(ray, make_Vec3f(camera.xfm.p), make_Vec3f(normalize((float)x*camera.xfm.l.vx + (float)y*camera.xfm.l.vy + camera.xfm.l.vz)), ray.tnear, ray.tfar);

    RayStats_addRay(stats);
  }

  uniform RTCIntersectContext context;
  rtcInitIntersectContext(&context);
  context.flags = g_iflags_coherent;

  /* trace stream of rays */
  rtcIntersectVM(g_scene,&context,(varying RTCRayHit* uniform)&rays,N,sizeof(Ray));

  /* shade stream of rays */
  Vec3f colors[TILE_SIZE_X*TILE_SIZE_Y];
  N = 0;
  foreach_tiled (y = y0 ... y1, x = x0 ... x1)
  {
    /* ISPC workaround for mask == 0 */
    if (all(__mask == 0)) continue;
    Ray& ray = rays[N];

    Vec3f Ng = ray.Ng;

    /* shading */
    Vec3f color = make_Vec3f(0.0f,1.0f,0.0f);
    if (ray.geomID != RTC_INVALID_GEOMETRY_ID)
    {
      /* vertex normals */
#if VERTEX_NORMALS == 1
      ISPCGeometry* geometry = g_ispc_scene->geometries[ray.geomID];
      if (geometry->type == TRIANGLE_MESH)
      {
        ISPCTriangleMesh* mesh = (ISPCTriangleMesh*) geometry;
        if (mesh->normals)
        {
          ISPCTriangle* tri = &mesh->triangles[ray.primID];

          const Vec3fa n0 = mesh->normals[tri->v0];
          const Vec3fa n1 = mesh->normals[tri->v1];
          const Vec3fa n2 = mesh->normals[tri->v2];
          const Vec3fa n = n0*(1.0f-ray.u-ray.v) + n1*ray.u + n2*ray.v;
          Ng = make_Vec3f(n.x,n.y,n.z);
        }
      }
#endif
      color = make_Vec3f(abs(dot(ray.dir,normalize(Ng))));
    }
    colors[N++] = color;
  }


#if SHADOWS == 1
    /* do some hard shadows to point lights */
    if (g_ispc_scene->numLights)
    {
      for (uniform unsigned int i=0; i<g_ispc_scene->numLights; i++)
      {
        /* init shadow/occlusion rays */
        for (uniform unsigned int n=0;n<N;n++)
        {
          Ray& ray = rays[n];
          const bool valid = ray.geomID != RTC_INVALID_GEOMETRY_ID;
          const Vec3f hitpos = ray.org + ray.dir * ray.tfar;
          const Vec3f shadow_org = hitpos - ray.org;
          init_Ray(ray, ls_positions[i], shadow_org, 1E-4f, valid ? 0.99f : -1.0f);
          RayStats_addShadowRay(stats);
        }
        /* trace shadow rays */
#if 0
        for (uniform unsigned int n=0;n<N;n++)
          rtcOccludedV(g_scene,&context,RTCRay_(rays[n]));
#else
        rtcOccludedVM(g_scene,&context,(varying RTCRay* uniform)&rays,N,sizeof(Ray));
#endif
        /* modify pixel color based on occlusion */
        for (uniform unsigned int n=0;n<N;n++)
          if (rays[n].tfar >= 0.0f)
            colors[n] = colors[n] * 0.1f;

      }
    }
#endif

  N = 0;
  foreach_tiled (y = y0 ... y1, x = x0 ... x1)
  {
    /* ISPC workaround for mask == 0 */
    if (all(__mask == 0)) continue;
    Vec3f& color = colors[N++];
    /* write color to framebuffer */
    unsigned int r = (unsigned int) (255.0f * clamp(color.x,0.0f,1.0f));
    unsigned int g = (unsigned int) (255.0f * clamp(color.y,0.0f,1.0f));
    unsigned int b = (unsigned int) (255.0f * clamp(color.z,0.0f,1.0f));
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

void device_key_pressed_handler(uniform int key)
{
  if (key == 111 /*o*/) {
  }
  else if (key == 112 /*p*/) {
  }
  else device_key_pressed_default(key);
}

/* called by the C++ code for initialization */
export void device_init (uniform int8* uniform cfg)
{
  /* create scene */
  g_scene = createScene(g_ispc_scene);

  /* create objects */
  uniform unsigned int numObjects = getNumObjects(g_ispc_scene);

  for (uniform unsigned int i=0;i<numObjects;i++)
    createObject(i,g_ispc_scene,g_scene);

  rtcCommitScene (g_scene);

  /* set render tile function to use */
  renderTile = renderTileStandard;
  key_pressed_handler = device_key_pressed_handler;
}

#define TICKS_PER_SECOND 2000000000

inline uniform double getTime() { return (double)clock() / TICKS_PER_SECOND; }

/* called by the C++ code to render */
export void device_render (uniform int* uniform pixels,
                           const uniform unsigned int width,
                           const uniform unsigned int height,
                           const uniform float time,
                           const uniform ISPCCamera& camera)
{

  /* =================================== */
  /* samples LS positions as pointlights */
  /* =================================== */

  if (g_ispc_scene->numLights)
  {
    if (ls_positions == NULL) ls_positions = uniform new varying Vec3f[g_ispc_scene->numLights];
    DifferentialGeometry dg;
    dg.geomID = 0;
    dg.primID = 0;
    dg.u = 0.0f;
    dg.v = 0.0f;
    dg.P  = make_Vec3f(0.0f,0.0f,0.0f);
    dg.Ng = make_Vec3f(0.0f,0.0f,0.0f);
    dg.Ns = dg.Ng;
    for (uniform unsigned int i=0; i<g_ispc_scene->numLights; i++)
    {
      const uniform Light* uniform l = g_ispc_scene->lights[i];
      const Vec2f sample = make_Vec2f(0.0f,0.0f);
      Light_SampleRes ls = l->sample(l,dg,sample);
      ls_positions[i] = ls.dir * ls.dist;
    }
  }

  /* ============ */
  /* render image */
  /* ============ */

  const uniform int numTilesX = (width +TILE_SIZE_X-1)/TILE_SIZE_X;
  const uniform int numTilesY = (height+TILE_SIZE_Y-1)/TILE_SIZE_Y;
  launch[numTilesX*numTilesY] renderTileTask(pixels,width,height,time,camera,numTilesX,numTilesY); sync;

  /* =============== */
  /* update geometry */
  /* =============== */

#if ENABLE_ANIM == 1

  if (animTime < 0.0f) animTime = getTime();
  const uniform float atime = (float)((getTime() - animTime) * ANIM_FPS);
  const uniform unsigned int intpart = (unsigned int)floor(atime);
  const uniform double fracpart = atime - (double)intpart;
  const uniform unsigned int keyFrameID = intpart;

  uniform unsigned int numObjects = getNumObjects(g_ispc_scene);
  for (uniform unsigned int i=0;i<numObjects;i++)
    updateVertexData(i, g_ispc_scene, g_scene, keyFrameID, (float)fracpart);

  /* =========== */
  /* rebuild bvh */
  /* =========== */

  rtcCommitScene(g_scene);

#endif
}

/* called by the C++ code for cleanup */
export void device_cleanup ()
{
  rtcReleaseScene (g_scene); g_scene = NULL;
}
