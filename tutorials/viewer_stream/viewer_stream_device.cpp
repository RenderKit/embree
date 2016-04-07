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

#include "../common/tutorial/tutorial_device.h"
#include "../common/tutorial/scene_device.h"
#include "../common/tutorial/random_sampler.h"
#include "../pathtracer/shapesampler.h"

#define USE_INTERFACE 0 // 0 = stream, 1 = single rays/packets, 2 = single rays/packets using stream interface
#define AMBIENT_OCCLUSION_SAMPLES 64
//#define AMBIENT_OCCLUSION_SAMPLES 256

//#define rtcOccluded rtcIntersect
//#define rtcOccludedN rtcIntersectN
//#define RAYN_FLAGS RTC_RAYN_COHERENT
#define RAYN_FLAGS RTC_RAYN_INCOHERENT

extern "C" ISPCScene* g_ispc_scene;

/* scene data */
RTCDevice g_device = nullptr;
RTCScene g_scene = nullptr;

/* error reporting function */
void error_handler(const RTCError code, const char* str = nullptr)
{
  if (code == RTC_NO_ERROR) 
    return;

  printf("Embree: ");
  switch (code) {
  case RTC_UNKNOWN_ERROR    : printf("RTC_UNKNOWN_ERROR"); break;
  case RTC_INVALID_ARGUMENT : printf("RTC_INVALID_ARGUMENT"); break;
  case RTC_INVALID_OPERATION: printf("RTC_INVALID_OPERATION"); break;
  case RTC_OUT_OF_MEMORY    : printf("RTC_OUT_OF_MEMORY"); break;
  case RTC_UNSUPPORTED_CPU  : printf("RTC_UNSUPPORTED_CPU"); break;
  case RTC_CANCELLED        : printf("RTC_CANCELLED"); break;
  default                   : printf("invalid error code"); break;
  }
  if (str) { 
    printf(" ("); 
    while (*str) putchar(*str++); 
    printf(")\n"); 
  }
  exit(1);
}

unsigned int convertTriangleMesh(ISPCTriangleMesh* mesh, RTCScene scene_out)
{
  unsigned int geomID = rtcNewTriangleMesh (scene_out, RTC_GEOMETRY_STATIC, mesh->numTriangles, mesh->numVertices, mesh->positions2 ? 2 : 1);
  rtcSetBuffer(scene_out, geomID, RTC_VERTEX_BUFFER, mesh->positions, 0, sizeof(Vec3fa      ));
  if (mesh->positions2) rtcSetBuffer(scene_out, geomID, RTC_VERTEX_BUFFER1, mesh->positions2, 0, sizeof(Vec3fa      ));
  rtcSetBuffer(scene_out, geomID, RTC_INDEX_BUFFER,  mesh->triangles, 0, sizeof(ISPCTriangle));
  mesh->geomID = geomID;
  return geomID;
}

unsigned int convertQuadMesh(ISPCQuadMesh* mesh, RTCScene scene_out)
{
  unsigned int geomID = rtcNewQuadMesh (scene_out, RTC_GEOMETRY_STATIC, mesh->numQuads, mesh->numVertices, mesh->positions2 ? 2 : 1);
  rtcSetBuffer(scene_out, geomID, RTC_VERTEX_BUFFER, mesh->positions, 0, sizeof(Vec3fa      ));
  if (mesh->positions2) rtcSetBuffer(scene_out, geomID, RTC_VERTEX_BUFFER1, mesh->positions2, 0, sizeof(Vec3fa      ));
  rtcSetBuffer(scene_out, geomID, RTC_INDEX_BUFFER,  mesh->quads, 0, sizeof(ISPCQuad));
  mesh->geomID = geomID;
  return geomID;
}

unsigned int convertSubdivMesh(ISPCSubdivMesh* mesh, RTCScene scene_out)
{
  unsigned int geomID = rtcNewSubdivisionMesh(scene_out, RTC_GEOMETRY_STATIC, mesh->numFaces, mesh->numEdges, mesh->numVertices, 
                                                      mesh->numEdgeCreases, mesh->numVertexCreases, mesh->numHoles);
  mesh->geomID = geomID;												
  for (size_t i=0; i<mesh->numEdges; i++) mesh->subdivlevel[i] = 16.0f;
  rtcSetBuffer(scene_out, geomID, RTC_VERTEX_BUFFER, mesh->positions, 0, sizeof(Vec3fa  ));
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
  unsigned int geomID = rtcNewLineSegments (scene_out, RTC_GEOMETRY_STATIC, mesh->numSegments, mesh->numVertices, mesh->v2 ? 2 : 1);
  rtcSetBuffer(scene_out,geomID,RTC_VERTEX_BUFFER,mesh->v,0,sizeof(Vertex));
  if (mesh->v2) rtcSetBuffer(scene_out,geomID,RTC_VERTEX_BUFFER1,mesh->v2,0,sizeof(Vertex));
  rtcSetBuffer(scene_out,geomID,RTC_INDEX_BUFFER,mesh->indices,0,sizeof(int));
  return geomID;
}

unsigned int convertHairSet(ISPCHairSet* hair, RTCScene scene_out)
{
  unsigned int geomID = rtcNewHairGeometry (scene_out, RTC_GEOMETRY_STATIC, hair->numHairs, hair->numVertices, hair->v2 ? 2 : 1);
  rtcSetBuffer(scene_out,geomID,RTC_VERTEX_BUFFER,hair->v,0,sizeof(Vertex));
  if (hair->v2) rtcSetBuffer(scene_out,geomID,RTC_VERTEX_BUFFER1,hair->v2,0,sizeof(Vertex));
  rtcSetBuffer(scene_out,geomID,RTC_INDEX_BUFFER,hair->hairs,0,sizeof(ISPCHair));
  return geomID;
}

unsigned int convertCurveGeometry(ISPCHairSet* hair, RTCScene scene_out)
{
  unsigned int geomID = rtcNewCurveGeometry (scene_out, RTC_GEOMETRY_STATIC, hair->numHairs, hair->numVertices, hair->v2 ? 2 : 1);
  rtcSetBuffer(scene_out,geomID,RTC_VERTEX_BUFFER,hair->v,0,sizeof(Vertex));
  if (hair->v2) rtcSetBuffer(scene_out,geomID,RTC_VERTEX_BUFFER1,hair->v2,0,sizeof(Vertex));
  rtcSetBuffer(scene_out,geomID,RTC_INDEX_BUFFER,hair->hairs,0,sizeof(ISPCHair));
  return geomID;
}

RTCScene convertScene(ISPCScene* scene_in)
{
  size_t numGeometries = scene_in->numGeometries;
  int scene_flags = RTC_SCENE_STATIC | RTC_SCENE_INCOHERENT;
  int scene_aflags = RTC_INTERSECT1 | RTC_INTERSECTN | RTC_INTERPOLATE;
  RTCScene scene_out = rtcDeviceNewScene(g_device, (RTCSceneFlags)scene_flags,(RTCAlgorithmFlags) scene_aflags);

  for (size_t i=0; i<scene_in->numGeometries; i++)
  {
    ISPCGeometry* geometry = scene_in->geometries[i];
    if (geometry->type == SUBDIV_MESH) {
      unsigned int geomID = convertSubdivMesh((ISPCSubdivMesh*) geometry, scene_out);
      assert(geomID == i);
    }
    else if (geometry->type == TRIANGLE_MESH) {
      unsigned int geomID = convertTriangleMesh((ISPCTriangleMesh*) geometry, scene_out);
      assert(geomID == i);
    }
    else if (geometry->type == QUAD_MESH) {
      unsigned int geomID = convertQuadMesh((ISPCQuadMesh*) geometry, scene_out);
      assert(geomID == i);
    }
    else if (geometry->type == LINE_SEGMENTS) {
      unsigned int geomID = convertLineSegments((ISPCLineSegments*) geometry, scene_out);
      assert(geomID == i);
    }
    else if (geometry->type == HAIR_SET) {
      unsigned int geomID = convertHairSet((ISPCHairSet*) geometry, scene_out);
      assert(geomID == i);
    }
    else if (geometry->type == CURVES) {
      unsigned int geomID = convertCurveGeometry((ISPCHairSet*) geometry, scene_out);
      assert(geomID == i);
    }
    else
      assert(false);
  }
  return scene_out;
}

/* renders a single pixel casting with ambient occlusion */
Vec3fa ambientOcclusionShading(int x, int y, RTCRay& ray)
{
  RTCRay rays[AMBIENT_OCCLUSION_SAMPLES];

  /* disable all rays first bet setting tnear > tfar */
  {
    for (int i=0; i<AMBIENT_OCCLUSION_SAMPLES; i++)
    {
      RTCRay& shadow = rays[i];
      shadow.tnear = inf;
      shadow.tfar = 0;      
    }  
  }

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
    float sx = RandomSampler_get1D(sampler);
    float sy = RandomSampler_get1D(sampler);
    Sample3f dir = cosineSampleHemisphere(sx,sy,Ng);

    /* initialize shadow ray */
    RTCRay& shadow = rays[i];
    shadow.org = hitPos;
    shadow.dir = dir.v;
    shadow.tnear = 0.001f;
    shadow.tfar = inf;
    shadow.geomID = RTC_INVALID_GEOMETRY_ID;
    shadow.primID = RTC_INVALID_GEOMETRY_ID;
    shadow.mask = -1;
    shadow.time = 0;    // FIXME: invalidate inactive rays
  } 
  
  /* trace occlusion rays */
#if USE_INTERFACE == 0
  rtcOccludedN(g_scene,rays,AMBIENT_OCCLUSION_SAMPLES,sizeof(RTCRay),RAYN_FLAGS);
#elif USE_INTERFACE == 1
  for (size_t i=0; i<AMBIENT_OCCLUSION_SAMPLES; i++)
    rtcOccluded(g_scene,rays[i]);
#else
  for (size_t i=0; i<AMBIENT_OCCLUSION_SAMPLES; i++)
    rtcOccludedN(g_scene,&rays[i],1,sizeof(RTCRay),RAYN_FLAGS);
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
                        const int width,
                        const int height, 
                        const float time,
                        const ISPCCamera& camera,
                        const int numTilesX, 
                        const int numTilesY)
{
  const int tileY = taskIndex / numTilesX;
  const int tileX = taskIndex - tileY * numTilesX;
  const int x0 = tileX * TILE_SIZE_X;
  const int x1 = min(x0+TILE_SIZE_X,width);
  const int y0 = tileY * TILE_SIZE_Y;
  const int y1 = min(y0+TILE_SIZE_Y,height);

  RTCRay rays[TILE_SIZE_X*TILE_SIZE_Y];

  /* generate stream of primary rays */
  int N = 0;
  for (int y=y0; y<y1; y++) for (int x=x0; x<x1; x++)
  {
    /* ISPC workaround for mask == 0 */
    if (all(1 == 0)) continue;

    RandomSampler sampler;
    RandomSampler_init(sampler, x, y, 0);
    
    /* initialize ray */
    RTCRay& ray = rays[N++];

    ray.org = Vec3fa(camera.xfm.p); // FIXME: make invalid rays empty
    ray.dir = Vec3fa(normalize(x*camera.xfm.l.vx + y*camera.xfm.l.vy + camera.xfm.l.vz));
    ray.tnear = 0.0f;
    ray.tfar = inf;
    ray.geomID = RTC_INVALID_GEOMETRY_ID;
    ray.primID = RTC_INVALID_GEOMETRY_ID;
    ray.mask = -1;
    ray.time = RandomSampler_get1D(sampler);
  }

  /* trace stream of rays */
#if USE_INTERFACE == 0
  rtcIntersectN(g_scene,rays,N,sizeof(RTCRay),RAYN_FLAGS);
#elif USE_INTERFACE == 1
  for (size_t i=0; i<N; i++)
    rtcIntersect(g_scene,rays[i]);
#else
  for (size_t i=0; i<N; i++)
    rtcIntersectN(g_scene,&rays[i],1,sizeof(RTCRay),RAYN_FLAGS);
#endif

  /* shade stream of rays */
  N = 0;
  for (int y=y0; y<y1; y++) for (int x=x0; x<x1; x++)
  {
    /* ISPC workaround for mask == 0 */
    if (all(1 == 0)) continue;
    RTCRay& ray = rays[N++];


    /* eyelight shading */
    Vec3fa color = Vec3fa(0.0f);
    if (ray.geomID != RTC_INVALID_GEOMETRY_ID) 
      //color = Vec3fa(abs(dot(ray.dir,normalize(ray.Ng))));
      color = ambientOcclusionShading(x,y,ray);

    /* write color to framebuffer */
    unsigned int r = (unsigned int) (255.0f * clamp(color.x,0.0f,1.0f));
    unsigned int g = (unsigned int) (255.0f * clamp(color.y,0.0f,1.0f));
    unsigned int b = (unsigned int) (255.0f * clamp(color.z,0.0f,1.0f));
    pixels[y*width+x] = (b << 16) + (g << 8) + r;
  }
}

/* task that renders a single screen tile */
void renderTileTask (int taskIndex, int* pixels,
                         const int width,
                         const int height, 
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
  rtcCommit (g_scene);

  /* set render tile function to use */
  renderTile = renderTileStandard;
  key_pressed_handler = device_key_pressed_default;
}

/* called by the C++ code to render */
extern "C" void device_render (int* pixels,
                           const int width,
                           const int height, 
                           const float time,
                           const ISPCCamera& camera)
{
  /* render image */
  const int numTilesX = (width +TILE_SIZE_X-1)/TILE_SIZE_X;
  const int numTilesY = (height+TILE_SIZE_Y-1)/TILE_SIZE_Y;
  parallel_for(size_t(0),size_t(numTilesX*numTilesY),[&](const range<size_t>& range) {
    for (size_t i=range.begin(); i<range.end(); i++)
      renderTileTask(i,pixels,width,height,time,camera,numTilesX,numTilesY);
  }); 
}

/* called by the C++ code for cleanup */
extern "C" void device_cleanup ()
{
  rtcDeleteScene (g_scene);
  rtcDeleteDevice(g_device);
}
