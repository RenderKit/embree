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

#include "../common/math/random_sampler.h"
#include "../common/math/sampling.h"
#include "../common/tutorial/tutorial_device.h"
#include "../common/tutorial/scene_device.h"

namespace embree {

#define USE_INTERFACE 0 // 0 = stream, 1 = single rays/packets, 2 = single rays/packets using stream interface
#define AMBIENT_OCCLUSION_SAMPLES 64
//#define rtcOccluded rtcIntersect
//#define rtcOccluded1M rtcIntersect1M
#define RAYN_FLAGS RTC_INTERSECT_COHERENT
//#define RAYN_FLAGS RTC_INTERSECT_INCOHERENT

#define SIMPLE_SHADING 0
#define DYNAMIC_BENCHMARK 0

extern "C" ISPCScene* g_ispc_scene;

/* scene data */
RTCDevice g_device = nullptr;
RTCScene g_scene = nullptr;

unsigned int convertTriangleMesh(ISPCTriangleMesh* mesh, RTCScene scene_out)
{
#if DYNAMIC_BENCHMARK == 1
  RTCGeometryFlags flags = RTC_GEOMETRY_DEFORMABLE;
#else
  RTCGeometryFlags flags = RTC_GEOMETRY_STATIC;
#endif
  unsigned int geomID = rtcNewTriangleMesh (scene_out, flags, mesh->numTriangles, mesh->numVertices, mesh->numTimeSteps);
  for (size_t t=0; t<mesh->numTimeSteps; t++) {
    rtcSetBuffer(scene_out, geomID, (RTCBufferType)(RTC_VERTEX_BUFFER+t),mesh->positions+t*mesh->numVertices, 0, sizeof(Vec3fa      ));
  }
  rtcSetBuffer(scene_out, geomID, RTC_INDEX_BUFFER,  mesh->triangles, 0, sizeof(ISPCTriangle));
  mesh->geomID = geomID;
  return geomID;
}

unsigned int convertQuadMesh(ISPCQuadMesh* mesh, RTCScene scene_out)
{
#if DYNAMIC_BENCHMARK == 1
  RTCGeometryFlags flags = RTC_GEOMETRY_DEFORMABLE;
#else
  RTCGeometryFlags flags = RTC_GEOMETRY_STATIC;
#endif
  unsigned int geomID = rtcNewQuadMesh (scene_out, flags, mesh->numQuads, mesh->numVertices, mesh->numTimeSteps);
  for (size_t t=0; t<mesh->numTimeSteps; t++) {
    rtcSetBuffer(scene_out, geomID, (RTCBufferType)(RTC_VERTEX_BUFFER+t),mesh->positions+t*mesh->numVertices, 0, sizeof(Vec3fa      ));
  }
  rtcSetBuffer(scene_out, geomID, RTC_INDEX_BUFFER,  mesh->quads, 0, sizeof(ISPCQuad));
  mesh->geomID = geomID;
  return geomID;
}

unsigned int convertSubdivMesh(ISPCSubdivMesh* mesh, RTCScene scene_out)
{
  unsigned int geomID = rtcNewSubdivisionMesh(scene_out, RTC_GEOMETRY_STATIC, mesh->numFaces, mesh->numEdges, mesh->numVertices,
                                                      mesh->numEdgeCreases, mesh->numVertexCreases, mesh->numHoles, mesh->numTimeSteps);
  mesh->geomID = geomID;
  for (size_t i=0; i<mesh->numEdges; i++) mesh->subdivlevel[i] = 16.0f;
  for (size_t t=0; t<mesh->numTimeSteps; t++) {
    rtcSetBuffer(scene_out, geomID, (RTCBufferType)(RTC_VERTEX_BUFFER+t),mesh->positions+t*mesh->numVertices, 0, sizeof(Vec3fa  ));
  }
  rtcSetBuffer(scene_out, geomID, RTC_LEVEL_BUFFER,  mesh->subdivlevel, 0, sizeof(float));
  rtcSetBuffer(scene_out, geomID, RTC_INDEX_BUFFER,  mesh->position_indices  , 0, sizeof(unsigned int));
  rtcSetBuffer(scene_out, geomID, RTC_FACE_BUFFER,   mesh->verticesPerFace, 0, sizeof(unsigned int));
  rtcSetBuffer(scene_out, geomID, RTC_HOLE_BUFFER,   mesh->holes, 0, sizeof(unsigned int));
  rtcSetBuffer(scene_out, geomID, RTC_EDGE_CREASE_INDEX_BUFFER,    mesh->edge_creases,          0, 2*sizeof(unsigned int));
  rtcSetBuffer(scene_out, geomID, RTC_EDGE_CREASE_WEIGHT_BUFFER,   mesh->edge_crease_weights,   0, sizeof(float));
  rtcSetBuffer(scene_out, geomID, RTC_VERTEX_CREASE_INDEX_BUFFER,  mesh->vertex_creases,        0, sizeof(unsigned int));
  rtcSetBuffer(scene_out, geomID, RTC_VERTEX_CREASE_WEIGHT_BUFFER, mesh->vertex_crease_weights, 0, sizeof(float));
  return geomID;
}

unsigned int convertLineSegments(ISPCLineSegments* mesh, RTCScene scene_out)
{
  unsigned int geomID = rtcNewLineSegments (scene_out, RTC_GEOMETRY_STATIC, mesh->numSegments, mesh->numVertices, mesh->numTimeSteps);
  for (size_t t=0; t<mesh->numTimeSteps; t++) {
    rtcSetBuffer(scene_out,geomID,(RTCBufferType)(RTC_VERTEX_BUFFER+t),mesh->positions+t*mesh->numVertices,0,sizeof(Vertex));
  }
  rtcSetBuffer(scene_out,geomID,RTC_INDEX_BUFFER,mesh->indices,0,sizeof(int));
  return geomID;
}

unsigned int convertHairSet(ISPCHairSet* hair, RTCScene scene_out)
{
  unsigned int geomID = rtcNewHairGeometry (scene_out, RTC_GEOMETRY_STATIC, hair->numHairs, hair->numVertices, hair->numTimeSteps);
  for (size_t t=0; t<hair->numTimeSteps; t++) {
    rtcSetBuffer(scene_out,geomID,(RTCBufferType)(RTC_VERTEX_BUFFER+t),hair->positions+t*hair->numVertices,0,sizeof(Vertex));
  }
  rtcSetBuffer(scene_out,geomID,RTC_INDEX_BUFFER,hair->hairs,0,sizeof(ISPCHair));
  return geomID;
}

unsigned int convertCurveGeometry(ISPCHairSet* hair, RTCScene scene_out)
{
  unsigned int geomID = rtcNewCurveGeometry (scene_out, RTC_GEOMETRY_STATIC, hair->numHairs, hair->numVertices, hair->numTimeSteps);
  for (size_t t=0; t<hair->numTimeSteps; t++) {
    rtcSetBuffer(scene_out,geomID,(RTCBufferType)(RTC_VERTEX_BUFFER+t),hair->positions+t*hair->numVertices,0,sizeof(Vertex));
  }
  rtcSetBuffer(scene_out,geomID,RTC_INDEX_BUFFER,hair->hairs,0,sizeof(ISPCHair));
  return geomID;
}

RTCScene convertScene(ISPCScene* scene_in)
{
  size_t numGeometries = scene_in->numGeometries;
#if DYNAMIC_BENCHMARK == 1
  int scene_flags = RTC_SCENE_DYNAMIC | RTC_SCENE_INCOHERENT;
#else
  int scene_flags = RTC_SCENE_STATIC | RTC_SCENE_INCOHERENT;
#endif

  int scene_aflags = RTC_INTERSECT1 | RTC_INTERSECT_STREAM | RTC_INTERPOLATE;
  RTCScene scene_out = rtcDeviceNewScene(g_device, (RTCSceneFlags)scene_flags,(RTCAlgorithmFlags) scene_aflags);

  for (size_t i=0; i<numGeometries; i++)
  {
    ISPCGeometry* geometry = scene_in->geometries[i];
    if (geometry->type == SUBDIV_MESH) {
      unsigned int geomID MAYBE_UNUSED = convertSubdivMesh((ISPCSubdivMesh*) geometry, scene_out);
      assert(geomID == i);
    }
    else if (geometry->type == TRIANGLE_MESH) {
      unsigned int geomID MAYBE_UNUSED = convertTriangleMesh((ISPCTriangleMesh*) geometry, scene_out);
      assert(geomID == i);
    }
    else if (geometry->type == QUAD_MESH) {
      unsigned int geomID MAYBE_UNUSED = convertQuadMesh((ISPCQuadMesh*) geometry, scene_out);
      assert(geomID == i);
    }
    else if (geometry->type == LINE_SEGMENTS) {
      unsigned int geomID MAYBE_UNUSED = convertLineSegments((ISPCLineSegments*) geometry, scene_out);
      assert(geomID == i);
    }
    else if (geometry->type == HAIR_SET) {
      unsigned int geomID MAYBE_UNUSED = convertHairSet((ISPCHairSet*) geometry, scene_out);
      assert(geomID == i);
    }
    else if (geometry->type == CURVES) {
      unsigned int geomID MAYBE_UNUSED = convertCurveGeometry((ISPCHairSet*) geometry, scene_out);
      assert(geomID == i);
    }
    else
      assert(false);
  }
  return scene_out;
}

  void updateScene(ISPCScene* scene_in, RTCScene g_scene)
{
  size_t numGeometries = scene_in->numGeometries;
  for (size_t i=0; i<numGeometries; i++)
  {
    ISPCGeometry* geometry = scene_in->geometries[i];
    if (geometry->type == SUBDIV_MESH) {
      rtcUpdate (g_scene,((ISPCSubdivMesh*)geometry)->geomID);
    }
    else if (geometry->type == TRIANGLE_MESH) {
      rtcUpdate (g_scene,((ISPCTriangleMesh*)geometry)->geomID);
    }
    else if (geometry->type == QUAD_MESH) {
      rtcUpdate (g_scene,((ISPCQuadMesh*)geometry)->geomID);
    }
    else
      assert(false);
  }
}
/* renders a single pixel casting with ambient occlusion */
Vec3fa ambientOcclusionShading(int x, int y, RTCRay& ray)
{
  RTCRay rays[AMBIENT_OCCLUSION_SAMPLES];

  Vec3fa Ng = normalize(ray.Ng);
  if (dot(ray.dir,Ng) > 0.0f) Ng = neg(Ng);

  Vec3fa col = Vec3fa(min(1.0f,0.3f+0.8f*abs(dot(Ng,normalize(ray.dir)))));

  /* calculate hit point */
  float intensity = 0;
  Vec3fa hitPos = ray.org + ray.tfar * ray.dir;

  RandomSampler sampler;
  RandomSampler_init(sampler,x,y,0);

  /* enable only valid rays */
  for (int i=0; i<AMBIENT_OCCLUSION_SAMPLES; i++)
  {
    /* sample random direction */
    Vec2f s = RandomSampler_get2D(sampler);
    Sample3f dir;
    dir.v = cosineSampleHemisphere(s);
    dir.pdf = cosineSampleHemispherePDF(dir.v);
    dir.v = frame(Ng) * dir.v;

    /* initialize shadow ray */
    RTCRay& shadow = rays[i];
    shadow.org = hitPos;
    shadow.dir = dir.v;
    bool mask = 1; { // invalidate inactive rays
      shadow.tnear = mask ? 0.001f       : (float)(pos_inf);
      shadow.tfar  = mask ? (float)(inf) : (float)(neg_inf);
    }
    shadow.geomID = RTC_INVALID_GEOMETRY_ID;
    shadow.primID = RTC_INVALID_GEOMETRY_ID;
    shadow.mask = -1;
    shadow.time = 0;    // FIXME: invalidate inactive rays
  }

  RTCIntersectContext context;
  context.flags = RAYN_FLAGS;

  /* trace occlusion rays */
#if USE_INTERFACE == 0
  rtcOccluded1M(g_scene,&context,rays,AMBIENT_OCCLUSION_SAMPLES,sizeof(RTCRay));
#elif USE_INTERFACE == 1
  for (size_t i=0; i<AMBIENT_OCCLUSION_SAMPLES; i++)
    rtcOccluded(g_scene,rays[i]);
#else
  for (size_t i=0; i<AMBIENT_OCCLUSION_SAMPLES; i++)
    rtcOccluded1M(g_scene,&context,&rays[i],1,sizeof(RTCRay));
#endif

  /* accumulate illumination */
  for (int i=0; i<AMBIENT_OCCLUSION_SAMPLES; i++) {
    if (rays[i].geomID == RTC_INVALID_GEOMETRY_ID)
      intensity += 1.0f;
  }

  /* shade pixel */
  return col * (intensity/AMBIENT_OCCLUSION_SAMPLES);
}

/* renders a single screen tile */
void renderTileStandard(int taskIndex,
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

  RTCRay rays[TILE_SIZE_X*TILE_SIZE_Y];

  /* generate stream of primary rays */
  int N = 0;
  for (unsigned int y=y0; y<y1; y++) for (unsigned int x=x0; x<x1; x++)
  {
    /* ISPC workaround for mask == 0 */
    if (all(1 == 0)) continue;

    RandomSampler sampler;
    RandomSampler_init(sampler, x, y, 0);

    /* initialize ray */
    RTCRay& ray = rays[N++];

    ray.org = Vec3fa(camera.xfm.p);
    ray.dir = Vec3fa(normalize((float)x*camera.xfm.l.vx + (float)y*camera.xfm.l.vy + camera.xfm.l.vz));
    bool mask = 1; { // invalidates inactive rays
      ray.tnear = mask ? 0.0f         : (float)(pos_inf);
      ray.tfar  = mask ? (float)(inf) : (float)(neg_inf);
    }
    ray.geomID = RTC_INVALID_GEOMETRY_ID;
    ray.primID = RTC_INVALID_GEOMETRY_ID;
    ray.mask = -1;
    ray.time = RandomSampler_get1D(sampler);
  }

  RTCIntersectContext context;
  context.flags = RAYN_FLAGS;

  /* trace stream of rays */
#if USE_INTERFACE == 0
  rtcIntersect1M(g_scene,&context,rays,N,sizeof(RTCRay));
#elif USE_INTERFACE == 1
  for (size_t i=0; i<N; i++)
    rtcIntersect(g_scene,rays[i]);
#else
  for (size_t i=0; i<N; i++)
    rtcIntersect1M(g_scene,&context,&rays[i],1,sizeof(RTCRay));
#endif

  /* shade stream of rays */
  N = 0;
  for (unsigned int y=y0; y<y1; y++) for (unsigned int x=x0; x<x1; x++)
  {
    /* ISPC workaround for mask == 0 */
    if (all(1 == 0)) continue;
    RTCRay& ray = rays[N++];

    /* eyelight shading */
    Vec3fa color = Vec3fa(0.0f);
    if (ray.geomID != RTC_INVALID_GEOMETRY_ID)
#if SIMPLE_SHADING == 1
      color = Vec3fa(abs(dot(ray.dir,normalize(ray.Ng))));
#else
      color = ambientOcclusionShading(x,y,ray);
#endif

    /* write color to framebuffer */
    unsigned int r = (unsigned int) (255.0f * clamp(color.x,0.0f,1.0f));
    unsigned int g = (unsigned int) (255.0f * clamp(color.y,0.0f,1.0f));
    unsigned int b = (unsigned int) (255.0f * clamp(color.z,0.0f,1.0f));
    pixels[y*width+x] = (b << 16) + (g << 8) + r;
  }
}

/* task that renders a single screen tile */
void renderTileTask (int taskIndex, int* pixels,
                         const unsigned int width,
                         const unsigned int height,
                         const float time,
                         const ISPCCamera& camera,
                         const int numTilesX,
                         const int numTilesY)
{
  renderTile(taskIndex,pixels,width,height,time,camera,numTilesX,numTilesY);
}

/* called by the C++ code for initialization */
extern "C" void device_init (char* cfg)
{
  /* create new Embree device */
  g_device = rtcNewDevice(cfg);
  error_handler(rtcDeviceGetError(g_device));

  /* set error handler */
  rtcDeviceSetErrorFunction(g_device,error_handler);

  /* create scene */
  g_scene = convertScene(g_ispc_scene);

#if DYNAMIC_BENCHMARK == 1
  while(1)
  {
    double t0 = getSeconds();
    updateScene (g_ispc_scene,g_scene);
    rtcCommit(g_scene);
    double t1 = getSeconds();
    PRINT(t1-t0);
  }
#endif
  rtcCommit (g_scene);

  /* set render tile function to use */
  renderTile = renderTileStandard;
  key_pressed_handler = device_key_pressed_default;
}

/* called by the C++ code to render */
extern "C" void device_render (int* pixels,
                           const unsigned int width,
                           const unsigned int height,
                           const float time,
                           const ISPCCamera& camera)
{
  /* render image */
  const int numTilesX = (width +TILE_SIZE_X-1)/TILE_SIZE_X;
  const int numTilesY = (height+TILE_SIZE_Y-1)/TILE_SIZE_Y;
  parallel_for(size_t(0),size_t(numTilesX*numTilesY),[&](const range<size_t>& range) {
    for (size_t i=range.begin(); i<range.end(); i++)
      renderTileTask((int)i,pixels,width,height,time,camera,numTilesX,numTilesY);
  }); 
}

/* called by the C++ code for cleanup */
extern "C" void device_cleanup ()
{
  rtcDeleteScene (g_scene); g_scene = nullptr;
  rtcDeleteDevice(g_device); g_device = nullptr;
}

} // namespace embree
