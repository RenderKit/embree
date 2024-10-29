// Copyright 2009-2021 Intel Corporation
// SPDX-License-Identifier: Apache-2.0

#include "debug_device_memory_device.h"

#include <stdlib.h>

namespace embree {

AffineSpace3fa axfm0;
AffineSpace3fa axfm1;

/* all features required by this tutorial */
#define FEATURE_MASK \
  RTC_FEATURE_FLAG_TRIANGLE | \
  RTC_FEATURE_FLAG_INSTANCE | \
  RTC_FEATURE_FLAG_MOTION_BLUR

RTCScene g_scene = nullptr;
RTCScene g_scene1 = nullptr;
TutorialData data;

/* adds a cube to the scene */
unsigned int addCube (RTCScene scene_i)
{
  /* create a triangulated cube with 12 triangles and 8 vertices */
  RTCGeometry mesh = rtcNewGeometry(g_device, RTC_GEOMETRY_TYPE_TRIANGLE);
  
  rtcSetGeometryTimeStepCount(mesh, 2);

  /* create face and vertex color arrays */
  data.face_colors = (Vec3fa*) alignedUSMMalloc((12)*sizeof(Vec3fa),16);
  data.vertex_colors = (Vec3fa*) alignedUSMMalloc((8)*sizeof(Vec3fa),16);

  /* set vertices and vertex colors */
  Vertex* vertices_device;
  Vertex* vertices1_device;
  Vertex* vertices = (Vertex*) rtcSetNewGeometryBufferXPU(mesh,RTC_BUFFER_TYPE_VERTEX,0,RTC_FORMAT_FLOAT3,sizeof(Vertex),8,(void**)&vertices_device);
  Vertex* vertices1 = (Vertex*) rtcSetNewGeometryBufferXPU(mesh,RTC_BUFFER_TYPE_VERTEX,1,RTC_FORMAT_FLOAT3,sizeof(Vertex),8,(void**)&vertices1_device);
  
#if defined(EMBREE_SYCL_TUTORIAL)
  sycl::usm::alloc allocType;
  allocType = sycl::get_pointer_type(vertices, global_gpu_queue->get_context());
  if (allocType != sycl::usm::alloc::host)
    std::cout << "cube vertices have wrong alloc type!" <<std::endl;
  allocType = sycl::get_pointer_type(vertices_device, global_gpu_queue->get_context());
  if (allocType != sycl::usm::alloc::device)
    std::cout << "cube vertices have wrong alloc type!" <<std::endl;
#endif

  data.vertex_colors[0] = Vec3fa(0,0,0);
  data.vertex_colors[1] = Vec3fa(0,0,1);
  data.vertex_colors[2] = Vec3fa(0,1,0);
  data.vertex_colors[3] = Vec3fa(0,1,1);
  data.vertex_colors[4] = Vec3fa(1,0,0);
  data.vertex_colors[5] = Vec3fa(1,0,1);
  data.vertex_colors[6] = Vec3fa(1,1,0);
  data.vertex_colors[7] = Vec3fa(1,1,1);

  vertices[0].x = -1; vertices[0].y = -1; vertices[0].z = -1;
  vertices[1].x = -1; vertices[1].y = -1; vertices[1].z = +1;
  vertices[2].x = -1; vertices[2].y = +1; vertices[2].z = -1;
  vertices[3].x = -1; vertices[3].y = +1; vertices[3].z = +1;
  vertices[4].x = +1; vertices[4].y = -1; vertices[4].z = -1;
  vertices[5].x = +1; vertices[5].y = -1; vertices[5].z = +1;
  vertices[6].x = +1; vertices[6].y = +1; vertices[6].z = -1;
  vertices[7].x = +1; vertices[7].y = +1; vertices[7].z = +1;

  vertices1[0].x = -1; vertices1[0].y = -1; vertices1[0].z = -1 + 3.f;
  vertices1[1].x = -1; vertices1[1].y = -1; vertices1[1].z = +1 + 3.f;
  vertices1[2].x = -1; vertices1[2].y = +1; vertices1[2].z = -1 + 3.f;
  vertices1[3].x = -1; vertices1[3].y = +1; vertices1[3].z = +1 + 3.f;
  vertices1[4].x = +1; vertices1[4].y = -1; vertices1[4].z = -1 + 3.f;
  vertices1[5].x = +1; vertices1[5].y = -1; vertices1[5].z = +1 + 3.f;
  vertices1[6].x = +1; vertices1[6].y = +1; vertices1[6].z = -1 + 3.f;
  vertices1[7].x = +1; vertices1[7].y = +1; vertices1[7].z = +1 + 3.f;

  /* set triangles and face colors */
  int tri = 0;
  Triangle* triangles_device;
  Triangle* triangles = (Triangle*) rtcSetNewGeometryBufferXPU(mesh,RTC_BUFFER_TYPE_INDEX,0,RTC_FORMAT_UINT3,sizeof(Triangle),12,(void**)&triangles_device);

  // left side
  data.face_colors[tri] = Vec3fa(1,0,0); triangles[tri].v0 = 0; triangles[tri].v1 = 1; triangles[tri].v2 = 2; tri++;
  data.face_colors[tri] = Vec3fa(1,0,0); triangles[tri].v0 = 1; triangles[tri].v1 = 3; triangles[tri].v2 = 2; tri++;

  // right side
  data.face_colors[tri] = Vec3fa(0,1,0); triangles[tri].v0 = 4; triangles[tri].v1 = 6; triangles[tri].v2 = 5; tri++;
  data.face_colors[tri] = Vec3fa(0,1,0); triangles[tri].v0 = 5; triangles[tri].v1 = 6; triangles[tri].v2 = 7; tri++;

  // bottom side
  data.face_colors[tri] = Vec3fa(0.5f);  triangles[tri].v0 = 0; triangles[tri].v1 = 4; triangles[tri].v2 = 1; tri++;
  data.face_colors[tri] = Vec3fa(0.5f);  triangles[tri].v0 = 1; triangles[tri].v1 = 4; triangles[tri].v2 = 5; tri++;

  // top side
  data.face_colors[tri] = Vec3fa(1.0f);  triangles[tri].v0 = 2; triangles[tri].v1 = 3; triangles[tri].v2 = 6; tri++;
  data.face_colors[tri] = Vec3fa(1.0f);  triangles[tri].v0 = 3; triangles[tri].v1 = 7; triangles[tri].v2 = 6; tri++;

  // front side
  data.face_colors[tri] = Vec3fa(0,0,1); triangles[tri].v0 = 0; triangles[tri].v1 = 2; triangles[tri].v2 = 4; tri++;
  data.face_colors[tri] = Vec3fa(0,0,1); triangles[tri].v0 = 2; triangles[tri].v1 = 6; triangles[tri].v2 = 4; tri++;

  // back side
  data.face_colors[tri] = Vec3fa(1,1,0); triangles[tri].v0 = 1; triangles[tri].v1 = 5; triangles[tri].v2 = 3; tri++;
  data.face_colors[tri] = Vec3fa(1,1,0); triangles[tri].v0 = 3; triangles[tri].v1 = 5; triangles[tri].v2 = 7; tri++;

  rtcSetGeometryVertexAttributeCount(mesh,1);
  rtcSetSharedGeometryBuffer(mesh,RTC_BUFFER_TYPE_VERTEX_ATTRIBUTE,0,RTC_FORMAT_FLOAT3,data.vertex_colors,0,sizeof(Vec3fa),8);

#if defined(EMBREE_SYCL_TUTORIAL)
  global_gpu_queue->memcpy(vertices_device, vertices, 8 * sizeof(Vertex));
  global_gpu_queue->memcpy(vertices1_device, vertices1, 8 * sizeof(Vertex));
  global_gpu_queue->memcpy(triangles_device, triangles, 12 * sizeof(Triangle));
  global_gpu_queue->wait_and_throw();
#endif

  rtcCommitGeometry(mesh);
  unsigned int geomID = rtcAttachGeometry(scene_i,mesh);
  rtcReleaseGeometry(mesh);
  return geomID;
}

unsigned int addCubeShared (RTCScene scene_i)
{
  /* create a triangulated cube with 12 triangles and 8 vertices */
  RTCGeometry mesh = rtcNewGeometry(g_device, RTC_GEOMETRY_TYPE_TRIANGLE);
  
  rtcSetGeometryTimeStepCount(mesh, 2);

  /* create face and vertex color arrays */
  data.face_colors = (Vec3fa*) alignedUSMMalloc((12)*sizeof(Vec3fa),16);
  data.vertex_colors = (Vec3fa*) alignedUSMMalloc((8)*sizeof(Vec3fa),16);

  Vertex* vertices = (Vertex*) alignedMalloc(8*sizeof(Vertex), 16);
  Vertex* vertices1 = (Vertex*) alignedMalloc(8*sizeof(Vertex), 16);
  Triangle* triangles = (Triangle*) alignedMalloc(12*sizeof(Triangle), 16);

#if defined(EMBREE_SYCL_TUTORIAL)
  Vertex* vertices_device = sycl::aligned_alloc_device<Vertex>(16, 8, *global_gpu_queue);
  Vertex* vertices1_device = sycl::aligned_alloc_device<Vertex>(16, 8, *global_gpu_queue);
  Triangle* triangles_device = sycl::aligned_alloc_device<Triangle>(16, 12, *global_gpu_queue);
#else
  Vertex* vertices_device = nullptr;
  Vertex* vertices1_device = nullptr;
  Triangle* triangles_device = nullptr;
#endif

  rtcSetSharedGeometryBufferXPU(mesh,RTC_BUFFER_TYPE_VERTEX,0,RTC_FORMAT_FLOAT3,vertices,vertices_device, 0, sizeof(Vertex),8);
  rtcSetSharedGeometryBufferXPU(mesh,RTC_BUFFER_TYPE_VERTEX,1,RTC_FORMAT_FLOAT3,vertices1,vertices1_device, 0, sizeof(Vertex),8);

  data.vertex_colors[0] = Vec3fa(0,0,0);
  data.vertex_colors[1] = Vec3fa(0,0,1);
  data.vertex_colors[2] = Vec3fa(0,1,0);
  data.vertex_colors[3] = Vec3fa(0,1,1);
  data.vertex_colors[4] = Vec3fa(1,0,0);
  data.vertex_colors[5] = Vec3fa(1,0,1);
  data.vertex_colors[6] = Vec3fa(1,1,0);
  data.vertex_colors[7] = Vec3fa(1,1,1);

  vertices[0].x = -1; vertices[0].y = -1; vertices[0].z = -1;
  vertices[1].x = -1; vertices[1].y = -1; vertices[1].z = +1;
  vertices[2].x = -1; vertices[2].y = +1; vertices[2].z = -1;
  vertices[3].x = -1; vertices[3].y = +1; vertices[3].z = +1;
  vertices[4].x = +1; vertices[4].y = -1; vertices[4].z = -1;
  vertices[5].x = +1; vertices[5].y = -1; vertices[5].z = +1;
  vertices[6].x = +1; vertices[6].y = +1; vertices[6].z = -1;
  vertices[7].x = +1; vertices[7].y = +1; vertices[7].z = +1;

  vertices1[0].x = -1 + 0.5f; vertices1[0].y = -1 + 0.5f; vertices1[0].z = -1 + 0.5f;
  vertices1[1].x = -1 + 0.5f; vertices1[1].y = -1 + 0.5f; vertices1[1].z = +1 + 0.5f;
  vertices1[2].x = -1 + 0.5f; vertices1[2].y = +1 + 0.5f; vertices1[2].z = -1 + 0.5f;
  vertices1[3].x = -1 + 0.5f; vertices1[3].y = +1 + 0.5f; vertices1[3].z = +1 + 0.5f;
  vertices1[4].x = +1 + 0.5f; vertices1[4].y = -1 + 0.5f; vertices1[4].z = -1 + 0.5f;
  vertices1[5].x = +1 + 0.5f; vertices1[5].y = -1 + 0.5f; vertices1[5].z = +1 + 0.5f;
  vertices1[6].x = +1 + 0.5f; vertices1[6].y = +1 + 0.5f; vertices1[6].z = -1 + 0.5f;
  vertices1[7].x = +1 + 0.5f; vertices1[7].y = +1 + 0.5f; vertices1[7].z = +1 + 0.5f;

  /* set triangles and face colors */
  int tri = 0;
  rtcSetSharedGeometryBufferXPU(mesh,RTC_BUFFER_TYPE_INDEX,0,RTC_FORMAT_UINT3,triangles,triangles_device,0,sizeof(Triangle),12);

  // left side
  data.face_colors[tri] = Vec3fa(1,0,0); triangles[tri].v0 = 0; triangles[tri].v1 = 1; triangles[tri].v2 = 2; tri++;
  data.face_colors[tri] = Vec3fa(1,0,0); triangles[tri].v0 = 1; triangles[tri].v1 = 3; triangles[tri].v2 = 2; tri++;

  // right side
  data.face_colors[tri] = Vec3fa(0,1,0); triangles[tri].v0 = 4; triangles[tri].v1 = 6; triangles[tri].v2 = 5; tri++;
  data.face_colors[tri] = Vec3fa(0,1,0); triangles[tri].v0 = 5; triangles[tri].v1 = 6; triangles[tri].v2 = 7; tri++;

  // bottom side
  data.face_colors[tri] = Vec3fa(0.5f);  triangles[tri].v0 = 0; triangles[tri].v1 = 4; triangles[tri].v2 = 1; tri++;
  data.face_colors[tri] = Vec3fa(0.5f);  triangles[tri].v0 = 1; triangles[tri].v1 = 4; triangles[tri].v2 = 5; tri++;

  // top side
  data.face_colors[tri] = Vec3fa(1.0f);  triangles[tri].v0 = 2; triangles[tri].v1 = 3; triangles[tri].v2 = 6; tri++;
  data.face_colors[tri] = Vec3fa(1.0f);  triangles[tri].v0 = 3; triangles[tri].v1 = 7; triangles[tri].v2 = 6; tri++;

  // front side
  data.face_colors[tri] = Vec3fa(0,0,1); triangles[tri].v0 = 0; triangles[tri].v1 = 2; triangles[tri].v2 = 4; tri++;
  data.face_colors[tri] = Vec3fa(0,0,1); triangles[tri].v0 = 2; triangles[tri].v1 = 6; triangles[tri].v2 = 4; tri++;

  // back side
  data.face_colors[tri] = Vec3fa(1,1,0); triangles[tri].v0 = 1; triangles[tri].v1 = 5; triangles[tri].v2 = 3; tri++;
  data.face_colors[tri] = Vec3fa(1,1,0); triangles[tri].v0 = 3; triangles[tri].v1 = 5; triangles[tri].v2 = 7; tri++;

  rtcSetGeometryVertexAttributeCount(mesh,1);
  rtcSetSharedGeometryBuffer(mesh,RTC_BUFFER_TYPE_VERTEX_ATTRIBUTE,0,RTC_FORMAT_FLOAT3,data.vertex_colors,0,sizeof(Vec3fa),8);

#if defined(EMBREE_SYCL_TUTORIAL)
  global_gpu_queue->memcpy(vertices_device, vertices, 8 * sizeof(Vertex));
  global_gpu_queue->memcpy(vertices1_device, vertices1, 8 * sizeof(Vertex));
  global_gpu_queue->memcpy(triangles_device, triangles, 12 * sizeof(Triangle));
#endif

  rtcCommitGeometry(mesh);
  unsigned int geomID = rtcAttachGeometry(scene_i,mesh);
  rtcReleaseGeometry(mesh);
  return geomID;
}

/* adds a ground plane to the scene */
unsigned int addGroundPlane (RTCScene scene_i)
{
  /* create a triangulated plane with 2 triangles and 4 vertices */
  RTCGeometry mesh = rtcNewGeometry (g_device, RTC_GEOMETRY_TYPE_TRIANGLE);

  /* set vertices */
  Vertex* vertices = (Vertex*) rtcSetNewGeometryBuffer(mesh,RTC_BUFFER_TYPE_VERTEX,0,RTC_FORMAT_FLOAT3,sizeof(Vertex),4);
  vertices[0].x = -10; vertices[0].y = -2; vertices[0].z = -10;
  vertices[1].x = -10; vertices[1].y = -2; vertices[1].z = +10;
  vertices[2].x = +10; vertices[2].y = -2; vertices[2].z = -10;
  vertices[3].x = +10; vertices[3].y = -2; vertices[3].z = +10;

  /* set triangles */
  Triangle* triangles = (Triangle*) rtcSetNewGeometryBuffer(mesh,RTC_BUFFER_TYPE_INDEX,0,RTC_FORMAT_UINT3,sizeof(Triangle),2);
  triangles[0].v0 = 0; triangles[0].v1 = 1; triangles[0].v2 = 2;
  triangles[1].v0 = 1; triangles[1].v1 = 3; triangles[1].v2 = 2;

  rtcCommitGeometry(mesh);
  unsigned int geomID = rtcAttachGeometry(scene_i,mesh);
  rtcReleaseGeometry(mesh);
  return geomID;
}

/* called by the C++ code for initialization */
extern "C" void device_init (char* cfg)
{ 
  /* create scene */
  TutorialData_Constructor(&data);
  g_scene = data.g_scene = rtcNewScene(g_device);
  g_scene1 = rtcNewScene(g_device);
  rtcSetSceneFlags(data.g_scene, RTC_SCENE_FLAG_PREFETCH_USM_SHARED_ON_GPU);

  /* add cube */
  addCube(g_scene1);

  RTCGeometry inst = rtcNewGeometry(g_device, RTC_GEOMETRY_TYPE_INSTANCE);

  rtcSetGeometryTimeStepCount(inst, 2);
  rtcSetGeometryInstancedScene(inst, g_scene1);

  LinearSpace3fa xfm = one;
  axfm0 = AffineSpace3fa(xfm,Vec3fa(0.f, 0.f, 0.f));
  axfm1 = AffineSpace3fa(xfm,Vec3fa(3.f, 0.f, 0.f));
  rtcSetGeometryTransform(inst,0,RTC_FORMAT_FLOAT4X4_COLUMN_MAJOR,(float*)&(axfm0.l.vx.x));
  rtcSetGeometryTransform(inst,1,RTC_FORMAT_FLOAT4X4_COLUMN_MAJOR,(float*)&(axfm1.l.vx.x));
  
  rtcAttachGeometry(data.g_scene,inst);
  rtcReleaseGeometry(inst);
  rtcCommitGeometry(inst);

  /* add ground plane */
  addGroundPlane(data.g_scene);

  /* commit changes to scene */
#if defined(EMBREE_SYCL_SUPPORT) && defined(SYCL_LANGUAGE_VERSION)
  rtcCommitSceneWithQueue (g_scene1, *global_gpu_queue);
  rtcCommitSceneWithQueue (data.g_scene, *global_gpu_queue);
#else
  rtcCommitScene (g_scene1);
  rtcCommitScene (data.g_scene);
#endif
}

static inline uint32_t doodle(uint32_t x)
{
  x ^= x << 13;
  x ^= x >> 17;
  x ^= x << 5;
  return x;
}

static inline float doodlef(uint32_t x)
{
  return ((float)doodle(x)) / (float)(uint32_t(-1));
}

/* task that renders a single screen tile */
void renderPixelStandard(const TutorialData& data,
                         int x, int y, 
                         int* pixels,
                         const unsigned int width,
                         const unsigned int height,
                         const float time,
                         const ISPCCamera& camera, RayStats& stats)
{
  /* initialize ray */
  uint32_t state = doodle(x + y * width);
  state = doodle(state);
  float t = doodlef(state);
  Ray ray(Vec3fa(camera.xfm.p), Vec3fa(normalize(x*camera.xfm.l.vx + y*camera.xfm.l.vy + camera.xfm.l.vz)), 0.0f, inf, t);

  /* intersect ray with scene */
  RTCIntersectArguments iargs;
  rtcInitIntersectArguments(&iargs);
  iargs.feature_mask = (RTCFeatureFlags) (FEATURE_MASK);
  rtcIntersect1(data.g_scene,RTCRayHit_(ray),&iargs);
  RayStats_addRay(stats);

  /* shade pixels */
  Vec3fa color = Vec3fa(0.0f);
  if (ray.geomID != RTC_INVALID_GEOMETRY_ID || ray.instID[0] != RTC_INVALID_GEOMETRY_ID)
  {
#if 1
    Vec3fa diffuse = data.face_colors[ray.primID];
    color = color + diffuse*0.5f;
    Vec3fa lightDir = normalize(Vec3fa(-1,-1,-1));

    /* initialize shadow ray */
    Ray shadow(ray.org + ray.tfar*ray.dir, neg(lightDir), 0.001f, inf, 1.f - t);

    /* trace shadow ray */
    RTCOccludedArguments sargs;
    rtcInitOccludedArguments(&sargs);
    sargs.feature_mask = (RTCFeatureFlags) (FEATURE_MASK);
    rtcOccluded1(data.g_scene,RTCRay_(shadow),&sargs);
    RayStats_addShadowRay(stats);

    /* add light contribution */
    if (shadow.tfar >= 0.0f)
      color = color + diffuse*clamp(-dot(lightDir,normalize(ray.Ng)),0.0f,1.0f);
#else

#if 0
    if (ray.geomID == 0)
      color = Vec3fa(0.f, 0.f, 1.f);
    else if (ray.geomID == 1)
      color = Vec3fa(0.f, 1.f, 0.f);
    else if (ray.geomID == 2)
      color = Vec3fa(1.f, 0.f, 0.f);
    else
      color = Vec3fa(1.f);
#endif

#if 0
    if (ray.primID == 0)
      color = Vec3fa(0.f, 0.f, 1.f);
    else if (ray.primID == 1)
      color = Vec3fa(0.f, 1.f, 0.f);
    else if (ray.primID == 2)
      color = Vec3fa(1.f, 0.f, 0.f);
    else
      color = Vec3fa(1.f);
#endif

#if 0
    color = Vec3fa(ray.u, ray.v, 0.f);
#endif

    color = Vec3fa(0.f, 1.f, 0.f);

#endif

  }

  /* write color to framebuffer */
  unsigned int r = (unsigned int) (255.0f * clamp(color.x,0.0f,1.0f));
  unsigned int g = (unsigned int) (255.0f * clamp(color.y,0.0f,1.0f));
  unsigned int b = (unsigned int) (255.0f * clamp(color.z,0.0f,1.0f));
  pixels[y*width+x] = (b << 16) + (g << 8) + r;
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

/* called by the C++ code to render */
extern "C" void renderFrameStandard (int* pixels,
                          const unsigned int width,
                          const unsigned int height,
                          const float time,
                          const ISPCCamera& camera)
{
#if defined(EMBREE_SYCL_TUTORIAL) && !defined(EMBREE_SYCL_RT_SIMULATION) && !defined(EMBREE_SYCL_RT_SIMULATION)
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
}

/* called by the C++ code for cleanup */
extern "C" void device_cleanup ()
{
  TutorialData_Destructor(&data);
}

} // namespace embree
