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

#include "../common/math/random_sampler.h"
#include "../common/tutorial/tutorial_device.h"
#include "../common/tutorial/scene_device.h"
#include "../common/math/sampling.h"

namespace embree {

#define ANIM_FPS 15.0f
#define ENABLE_ANIM 1
#define VERTEX_NORMALS 0
#define SHADOWS 1
#define VERTEX_INTERPOLATION_BLOCK_SIZE 1024

extern "C" ISPCScene* g_ispc_scene;

/* scene data */
RTCScene g_scene   = nullptr;
Vec3fa* ls_positions = nullptr;

/* animation data */
double animTime        = -1.0f; // global time counter


  // ==================================================================================================
  // ==================================================================================================
  // ==================================================================================================

void convertTriangleMesh(ISPCTriangleMesh* mesh, RTCScene scene_out)
{
  /* if more than a single timestep, mark object as dynamic */
  RTCBuildQuality quality = mesh->numTimeSteps > 1 ? RTC_BUILD_QUALITY_LOW : RTC_BUILD_QUALITY_MEDIUM;
  RTCGeometry geom = rtcNewGeometry (g_device, RTC_GEOMETRY_TYPE_TRIANGLE);
  rtcSetGeometryBuildQuality(geom, quality);
  Vec3fa* vertices = (Vec3fa*) rtcSetNewGeometryBuffer(geom, RTC_BUFFER_TYPE_VERTEX, 0, RTC_FORMAT_FLOAT3, sizeof(Vec3fa), mesh->numVertices);
  for (unsigned int i=0;i<mesh->numVertices;i++) vertices[i] = mesh->positions[0][i];
  rtcSetSharedGeometryBuffer(geom, RTC_BUFFER_TYPE_INDEX, 0, RTC_FORMAT_UINT3, mesh->triangles, 0, sizeof(ISPCTriangle), mesh->numTriangles);
  rtcCommitGeometry(geom);
  mesh->geom.geometry = geom;
  mesh->geom.geomID = rtcAttachGeometry(scene_out,geom);
}

void convertQuadMesh(ISPCQuadMesh* mesh, RTCScene scene_out)
{
  /* if more than a single timestep, mark object as dynamic */
  RTCBuildQuality quality = mesh->numTimeSteps > 1 ? RTC_BUILD_QUALITY_LOW : RTC_BUILD_QUALITY_MEDIUM;
  RTCGeometry geom = rtcNewGeometry (g_device, RTC_GEOMETRY_TYPE_QUAD);
  rtcSetGeometryBuildQuality(geom, quality);
  Vec3fa* vertices = (Vec3fa*) rtcSetNewGeometryBuffer(geom, RTC_BUFFER_TYPE_VERTEX, 0, RTC_FORMAT_FLOAT3, sizeof(Vec3fa), mesh->numVertices);
  for (unsigned int i=0;i<mesh->numVertices;i++) vertices[i] = mesh->positions[0][i];
  rtcSetSharedGeometryBuffer(geom, RTC_BUFFER_TYPE_INDEX, 0, RTC_FORMAT_UINT4, mesh->quads, 0, sizeof(ISPCQuad), mesh->numQuads);
  rtcCommitGeometry(geom);
  mesh->geom.geometry = geom;
  mesh->geom.geomID = rtcAttachGeometry(scene_out,geom);
}

void convertSubdivMesh(ISPCSubdivMesh* mesh, RTCScene scene_out)
{
  /* if more than a single timestep, mark object as dynamic */
  RTCBuildQuality quality = mesh->numTimeSteps > 1 ? RTC_BUILD_QUALITY_LOW : RTC_BUILD_QUALITY_MEDIUM;
  RTCGeometry geom = rtcNewGeometry(g_device, RTC_GEOMETRY_TYPE_SUBDIVISION);
  rtcSetGeometryBuildQuality(geom, quality);
  for (unsigned int i=0; i<mesh->numEdges; i++) mesh->subdivlevel[i] = 4.0f;
  Vec3fa* vertices = (Vec3fa*) rtcSetNewGeometryBuffer(geom, RTC_BUFFER_TYPE_VERTEX, 0, RTC_FORMAT_FLOAT3, sizeof(Vec3fa), mesh->numVertices);
  for (unsigned int i=0;i<mesh->numVertices;i++) vertices[i] = mesh->positions[0][i];
  rtcSetSharedGeometryBuffer(geom, RTC_BUFFER_TYPE_LEVEL, 0, RTC_FORMAT_FLOAT, mesh->subdivlevel,      0, sizeof(float),        mesh->numEdges);
  rtcSetSharedGeometryBuffer(geom, RTC_BUFFER_TYPE_INDEX, 0, RTC_FORMAT_UINT,  mesh->position_indices, 0, sizeof(unsigned int), mesh->numEdges);
  rtcSetSharedGeometryBuffer(geom, RTC_BUFFER_TYPE_FACE,  0, RTC_FORMAT_UINT,  mesh->verticesPerFace,  0, sizeof(unsigned int), mesh->numFaces);
  rtcSetSharedGeometryBuffer(geom, RTC_BUFFER_TYPE_HOLE,  0, RTC_FORMAT_UINT,  mesh->holes,            0, sizeof(unsigned int), mesh->numFaces);
  rtcSetSharedGeometryBuffer(geom, RTC_BUFFER_TYPE_EDGE_CREASE_INDEX,    0, RTC_FORMAT_UINT2, mesh->edge_creases,          0, 2*sizeof(unsigned int), mesh->numEdgeCreases);
  rtcSetSharedGeometryBuffer(geom, RTC_BUFFER_TYPE_EDGE_CREASE_WEIGHT,   0, RTC_FORMAT_FLOAT, mesh->edge_crease_weights,   0, sizeof(float),          mesh->numEdgeCreases);
  rtcSetSharedGeometryBuffer(geom, RTC_BUFFER_TYPE_VERTEX_CREASE_INDEX,  0, RTC_FORMAT_UINT,  mesh->vertex_creases,        0, sizeof(unsigned int),   mesh->numVertexCreases);
  rtcSetSharedGeometryBuffer(geom, RTC_BUFFER_TYPE_VERTEX_CREASE_WEIGHT, 0, RTC_FORMAT_FLOAT, mesh->vertex_crease_weights, 0, sizeof(float),          mesh->numVertexCreases);
  rtcSetGeometrySubdivisionMode(geom, 0, mesh->position_subdiv_mode);
  rtcCommitGeometry(geom);
  mesh->geom.geometry = geom;
  mesh->geom.geomID = rtcAttachGeometry(scene_out,geom);
}

void convertCurveGeometry(ISPCHairSet* hair, RTCScene scene_out)
{
  /* if more than a single timestep, mark object as dynamic */
  RTCBuildQuality quality = hair->numTimeSteps > 1 ? RTC_BUILD_QUALITY_LOW : RTC_BUILD_QUALITY_MEDIUM;
  /* create object */
  RTCGeometry geom = rtcNewGeometry (g_device, hair->type);
  rtcSetGeometryBuildQuality(geom, quality);
  /* generate vertex buffer */
  Vec3fa* vertices = (Vec3fa*) rtcSetNewGeometryBuffer(geom, RTC_BUFFER_TYPE_VERTEX, 0, RTC_FORMAT_FLOAT4, sizeof(Vec3fa), hair->numVertices);
  for (unsigned int i=0;i<hair->numVertices;i++) vertices[i] = hair->positions[0][i];
  rtcSetSharedGeometryBuffer(geom, RTC_BUFFER_TYPE_INDEX, 0, RTC_FORMAT_UINT, hair->hairs, 0, sizeof(ISPCHair), hair->numHairs);
  if (hair->type != RTC_GEOMETRY_TYPE_FLAT_LINEAR_CURVE)
    rtcSetGeometryTessellationRate(geom,(float)hair->tessellation_rate);
  rtcCommitGeometry(geom);
  hair->geom.geometry = geom;
  hair->geom.geomID = rtcAttachGeometry(scene_out,geom);
}

unsigned int getNumObjects(ISPCScene* scene_in) {
  return scene_in->numGeometries;
}

RTCScene createScene(ISPCScene* scene_in)
{
  RTCScene scene = rtcNewScene(g_device);
  rtcSetSceneBuildQuality(scene,RTC_BUILD_QUALITY_LOW);
  rtcSetSceneFlags(scene, RTC_SCENE_FLAG_DYNAMIC);
  return scene;
}

void createObject(const unsigned int i, ISPCScene* scene_in, RTCScene scene_out)
{
  ISPCGeometry* geometry = scene_in->geometries[i];
  
  if (geometry->type == SUBDIV_MESH) {
    convertSubdivMesh((ISPCSubdivMesh*) geometry, scene_out);
  }
  else if (geometry->type == TRIANGLE_MESH) {
    convertTriangleMesh((ISPCTriangleMesh*) geometry, scene_out);
  }
  else if (geometry->type == QUAD_MESH) {
    convertQuadMesh((ISPCQuadMesh*) geometry, scene_out);
  }
  else if (geometry->type == CURVES) {
    convertCurveGeometry((ISPCHairSet*) geometry, scene_out);
  }
  else
    assert(false);
}

Vec3fa lerpr(const Vec3fa& v0, const Vec3fa& v1, const float t) {
  return v0*(1.0f-t)+v1*t;
}


 void interpolateVertexBlock (int taskIndex, int threadIndex, const unsigned int numVertices,
                                  Vec3fa* vertices,
                                  const Vec3fa* const input0,
                                  const Vec3fa* const input1,
                                  const float tt)
 {
   const unsigned int b = taskIndex;
   const unsigned int startID = b*VERTEX_INTERPOLATION_BLOCK_SIZE;
   const unsigned int endID = min(startID + VERTEX_INTERPOLATION_BLOCK_SIZE,numVertices);
   for (unsigned int i=startID; i<endID; i++)
     vertices[i] = lerpr(input0[i],input1[i],tt);
 }


void interpolateVertices(RTCGeometry geom,
                         const unsigned int numVertices,
                         const Vec3fa* const input0,
                         const Vec3fa* const input1,
                         const float tt)
  {
    Vec3fa* vertices = (Vec3fa*) rtcGetGeometryBufferData(geom, RTC_BUFFER_TYPE_VERTEX, 0);
#if 1
    const unsigned int blocks = (numVertices+VERTEX_INTERPOLATION_BLOCK_SIZE-1) / VERTEX_INTERPOLATION_BLOCK_SIZE;
    parallel_for(size_t(0),size_t(blocks),[&](const range<size_t>& range) {
    const int threadIndex = (int)TaskScheduler::threadIndex();
    for (size_t i=range.begin(); i<range.end(); i++)
      interpolateVertexBlock((int)i,threadIndex,numVertices,vertices,input0,input1,tt);
  }); 
#else
    for (unsigned int i=0; i<numVertices; i++)
      vertices[i] = lerpr(input0[i],input1[i],tt);
#endif
    rtcUpdateGeometryBuffer(geom,RTC_BUFFER_TYPE_VERTEX, 0);
    rtcCommitGeometry(geom);
  }

  void updateVertexData(const unsigned int ID,
                        ISPCScene* scene_in,
                        RTCScene scene_out,
                        const unsigned int keyFrameID,
                        const float tt)
  {
    ISPCGeometry* geometry = scene_in->geometries[ID];

    if (geometry->type == SUBDIV_MESH) {
      /* if static do nothing */
      if (((ISPCSubdivMesh*)geometry)->numTimeSteps <= 1) return;
      rtcCommitGeometry(geometry->geometry);
    }
    else if (geometry->type == TRIANGLE_MESH) {
      ISPCTriangleMesh* mesh = (ISPCTriangleMesh*)geometry;
      /* if static do nothing */
      if (mesh->numTimeSteps <= 1) return;
      /* interpolate two vertices from two timesteps */
      const unsigned int t0 = (keyFrameID+0) % mesh->numTimeSteps;
      const unsigned int t1 = (keyFrameID+1) % mesh->numTimeSteps;
      const Vec3fa* const input0 = mesh->positions[t0];
      const Vec3fa* const input1 = mesh->positions[t1];
      interpolateVertices(geometry->geometry, mesh->numVertices, input0, input1, tt);
    }
    else if (geometry->type == QUAD_MESH) {
      ISPCQuadMesh* mesh = (ISPCQuadMesh*)geometry;
      /* if static do nothing */
      if (mesh->numTimeSteps <= 1) return;
      /* interpolate two vertices from two timesteps */
      const unsigned int t0 = (keyFrameID+0) % mesh->numTimeSteps;
      const unsigned int t1 = (keyFrameID+1) % mesh->numTimeSteps;
      const Vec3fa* const input0 = mesh->positions[t0];
      const Vec3fa* const input1 = mesh->positions[t1];
      interpolateVertices(geometry->geometry, mesh->numVertices, input0, input1, tt);
    }
    else if (geometry->type == CURVES) {
      /* if static do nothing */
      if (((ISPCHairSet*)geometry)->numTimeSteps <= 1) return;
      rtcCommitGeometry(geometry->geometry);
    }
    else
      assert(false);
  }


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

  RayStats& stats = g_stats[threadIndex];

  Ray rays[TILE_SIZE_X*TILE_SIZE_Y];

  /* generate stream of primary rays */
  unsigned int N = 0;
  for (unsigned int y=y0; y<y1; y++) for (unsigned int x=x0; x<x1; x++)
  {
    /* ISPC workaround for mask == 0 */
    

    /* initialize ray */
    Ray& ray = rays[N++];
    bool mask = 1; { // invalidates inactive rays
      ray.tnear() = mask ? 0.0f         : (float)(pos_inf);
      ray.tfar  = mask ? (float)(inf) : (float)(neg_inf);
    }
    init_Ray(ray, Vec3fa(camera.xfm.p), Vec3fa(normalize((float)x*camera.xfm.l.vx + (float)y*camera.xfm.l.vy + camera.xfm.l.vz)), ray.tnear(), ray.tfar);

    RayStats_addRay(stats);
  }

  RTCIntersectContext context;
  rtcInitIntersectContext(&context);
  context.flags = g_iflags_coherent;

  /* trace stream of rays */
  rtcIntersect1M(g_scene,&context,(RTCRayHit*)&rays,N,sizeof(Ray));

  /* shade stream of rays */
  Vec3fa colors[TILE_SIZE_X*TILE_SIZE_Y];
  N = 0;
  for (unsigned int y=y0; y<y1; y++) for (unsigned int x=x0; x<x1; x++)
  {
    /* ISPC workaround for mask == 0 */
    
    Ray& ray = rays[N];

    Vec3fa Ng = ray.Ng;

    /* shading */
    Vec3fa color = Vec3fa(0.0f,1.0f,0.0f);
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
          Ng = Vec3fa(n.x,n.y,n.z);
        }
      }
#endif
      color = Vec3fa(abs(dot(ray.dir,normalize(Ng))));
    }
    colors[N++] = color;
  }


#if SHADOWS == 1
    /* do some hard shadows to point lights */
    if (g_ispc_scene->numLights)
    {
      for (unsigned int i=0; i<g_ispc_scene->numLights; i++)
      {
        /* init shadow/occlusion rays */
        for (unsigned int n=0;n<N;n++)
        {
          Ray& ray = rays[n];
          const bool valid = ray.geomID != RTC_INVALID_GEOMETRY_ID;
          const Vec3fa hitpos = ray.org + ray.dir * ray.tfar;
          const Vec3fa shadow_org = hitpos - ray.org;
          init_Ray(ray, ls_positions[i], shadow_org, 1E-4f, valid ? 0.99f : -1.0f);
          RayStats_addShadowRay(stats);
        }
        /* trace shadow rays */
#if 0
        for (unsigned int n=0;n<N;n++)
          rtcOccluded1(g_scene,&context,RTCRay_(rays[n]));
#else
        rtcOccluded1M(g_scene,&context,(RTCRay*)&rays,N,sizeof(Ray));
#endif
        /* modify pixel color based on occlusion */
        for (unsigned int n=0;n<N;n++)
          if (rays[n].tfar >= 0.0f)
            colors[n] = colors[n] * 0.1f;

      }
    }
#endif

  N = 0;
  for (unsigned int y=y0; y<y1; y++) for (unsigned int x=x0; x<x1; x++)
  {
    /* ISPC workaround for mask == 0 */
    
    Vec3fa& color = colors[N++];
    /* write color to framebuffer */
    unsigned int r = (unsigned int) (255.0f * clamp(color.x,0.0f,1.0f));
    unsigned int g = (unsigned int) (255.0f * clamp(color.y,0.0f,1.0f));
    unsigned int b = (unsigned int) (255.0f * clamp(color.z,0.0f,1.0f));
    pixels[y*width+x] = (b << 16) + (g << 8) + r;
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
  renderTile(taskIndex,threadIndex,pixels,width,height,time,camera,numTilesX,numTilesY);
}

void device_key_pressed_handler(int key)
{
  if (key == 111 /*o*/) {
  }
  else if (key == 112 /*p*/) {
  }
  else device_key_pressed_default(key);
}

/* called by the C++ code for initialization */
extern "C" void device_init (char* cfg)
{
  /* create scene */
  g_scene = createScene(g_ispc_scene);

  /* create objects */
  unsigned int numObjects = getNumObjects(g_ispc_scene);

  for (unsigned int i=0;i<numObjects;i++)
    createObject(i,g_ispc_scene,g_scene);

  rtcCommitScene (g_scene);

  /* set render tile function to use */
  renderTile = renderTileStandard;
  key_pressed_handler = device_key_pressed_handler;
}

#define TICKS_PER_SECOND 2000000000

inline double getTime() { return (double)clock() / TICKS_PER_SECOND; }

/* called by the C++ code to render */
extern "C" void device_render (int* pixels,
                           const unsigned int width,
                           const unsigned int height,
                           const float time,
                           const ISPCCamera& camera)
{

  /* =================================== */
  /* samples LS positions as pointlights */
  /* =================================== */

  if (g_ispc_scene->numLights)
  {
    if (ls_positions == nullptr) ls_positions = (Vec3fa*) alignedMalloc(g_ispc_scene->numLights*sizeof(Vec3fa),16);
    DifferentialGeometry dg;
    dg.geomID = 0;
    dg.primID = 0;
    dg.u = 0.0f;
    dg.v = 0.0f;
    dg.P  = Vec3fa(0.0f,0.0f,0.0f);
    dg.Ng = Vec3fa(0.0f,0.0f,0.0f);
    dg.Ns = dg.Ng;
    for (unsigned int i=0; i<g_ispc_scene->numLights; i++)
    {
      const Light* l = g_ispc_scene->lights[i];
      const Vec2f sample = Vec2f(0.0f,0.0f);
      Light_SampleRes ls = l->sample(l,dg,sample);
      ls_positions[i] = ls.dir * ls.dist;
    }
  }

  /* ============ */
  /* render image */
  /* ============ */

  const int numTilesX = (width +TILE_SIZE_X-1)/TILE_SIZE_X;
  const int numTilesY = (height+TILE_SIZE_Y-1)/TILE_SIZE_Y;
  parallel_for(size_t(0),size_t(numTilesX*numTilesY),[&](const range<size_t>& range) {
    const int threadIndex = (int)TaskScheduler::threadIndex();
    for (size_t i=range.begin(); i<range.end(); i++)
      renderTileTask((int)i,threadIndex,pixels,width,height,time,camera,numTilesX,numTilesY);
  }); 

  /* =============== */
  /* update geometry */
  /* =============== */

#if ENABLE_ANIM == 1

  if (animTime < 0.0f) animTime = getTime();
  const float atime = (float)((getTime() - animTime) * ANIM_FPS);
  const unsigned int intpart = (unsigned int)floor(atime);
  const double fracpart = atime - (double)intpart;
  const unsigned int keyFrameID = intpart;

  unsigned int numObjects = getNumObjects(g_ispc_scene);
  for (unsigned int i=0;i<numObjects;i++)
    updateVertexData(i, g_ispc_scene, g_scene, keyFrameID, (float)fracpart);

  /* =========== */
  /* rebuild bvh */
  /* =========== */

  rtcCommitScene(g_scene);

#endif
}

/* called by the C++ code for cleanup */
extern "C" void device_cleanup ()
{
  rtcReleaseScene (g_scene); g_scene = nullptr;
}

} // namespace embree
