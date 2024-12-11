// Copyright 2009-2021 Intel Corporation
// SPDX-License-Identifier: Apache-2.0

#include "dynamic_scene_device.h"

namespace embree {

/* all features required by this tutorial */
#define FEATURE_MASK \
  RTC_FEATURE_FLAG_TRIANGLE

RTCScene g_scene = nullptr;
TutorialData data;

/* adds a sphere to the scene */
unsigned int createSphere (RTCBuildQuality quality, const Vec3fa& pos, const float r)
{
  /* create a triangulated sphere */
  RTCGeometry geom = rtcNewGeometry (g_device, RTC_GEOMETRY_TYPE_TRIANGLE);
  rtcSetGeometryBuildQuality(geom, quality);

  /* map triangle and vertex buffer */
  RTCBuffer vertexBuffer = rtcNewBufferHostDevice(g_device, sizeof(Vertex) * data.numTheta*(data.numPhi+1));
  Vertex* vertices = (Vertex*)rtcGetBufferData(vertexBuffer);
  rtcSetGeometryBuffer(geom,RTC_BUFFER_TYPE_VERTEX,0,RTC_FORMAT_FLOAT3,vertexBuffer,0,sizeof(Vertex),data.numTheta*(data.numPhi+1));
  rtcReleaseBuffer(vertexBuffer);

  RTCBuffer indexBuffer = rtcNewBufferHostDevice(g_device, sizeof(Triangle) * 2*data.numTheta*(data.numPhi-1));
  Triangle* triangles = (Triangle*)rtcGetBufferData(indexBuffer);
  rtcSetGeometryBuffer(geom,RTC_BUFFER_TYPE_INDEX,0,RTC_FORMAT_UINT3,indexBuffer,0,sizeof(Triangle),2*data.numTheta*(data.numPhi-1));
  rtcReleaseBuffer(indexBuffer);

  /* create sphere geometry */
  int tri = 0;
  const float rcpNumTheta = rcp((float)data.numTheta);
  const float rcpNumPhi   = rcp((float)data.numPhi);
  for (int phi=0; phi<=data.numPhi; phi++)
  {
    for (int theta=0; theta<data.numTheta; theta++)
    {
      const float phif   = phi*float(pi)*rcpNumPhi;
      const float thetaf = theta*2.0f*float(pi)*rcpNumTheta;
      Vertex& v = vertices[phi*data.numTheta+theta];
      v.x = pos.x + r*sin(phif)*sin(thetaf);
      v.y = pos.y + r*cos(phif);
      v.z = pos.z + r*sin(phif)*cos(thetaf);
    }
    if (phi == 0) continue;

    for (int theta=1; theta<=data.numTheta; theta++)
    {
      int p00 = (phi-1)*data.numTheta+theta-1;
      int p01 = (phi-1)*data.numTheta+theta%data.numTheta;
      int p10 = phi*data.numTheta+theta-1;
      int p11 = phi*data.numTheta+theta%data.numTheta;

      if (phi > 1) {
        triangles[tri].v0 = p10;
        triangles[tri].v1 = p01;
        triangles[tri].v2 = p00;
        tri++;
      }

      if (phi < data.numPhi) {
        triangles[tri].v0 = p11;
        triangles[tri].v1 = p01;
        triangles[tri].v2 = p10;
        tri++;
      }
    }
  }
  
  rtcCommitBuffer(vertexBuffer);
  rtcCommitBuffer(indexBuffer);

  rtcCommitGeometry(geom);
  unsigned int geomID = rtcAttachGeometry(data.g_scene,geom);
  rtcReleaseGeometry(geom);
  return geomID;
}

/* adds a ground plane to the scene */
unsigned int addGroundPlane (RTCScene scene_i)
{
  /* create a triangulated plane with 2 triangles and 4 vertices */
  RTCGeometry geom = rtcNewGeometry (g_device, RTC_GEOMETRY_TYPE_TRIANGLE);

  /* set vertices */
  RTCBuffer vertexBuffer = rtcNewBufferHostDevice(g_device, sizeof(Vertex)*4);
  Vertex* vertices = (Vertex*) rtcGetBufferData(vertexBuffer);
  rtcSetGeometryBuffer(geom,RTC_BUFFER_TYPE_VERTEX,0,RTC_FORMAT_FLOAT3,vertexBuffer,0,sizeof(Vertex),4);
  vertices[0].x = -10; vertices[0].y = -2; vertices[0].z = -10;
  vertices[1].x = -10; vertices[1].y = -2; vertices[1].z = +10;
  vertices[2].x = +10; vertices[2].y = -2; vertices[2].z = -10;
  vertices[3].x = +10; vertices[3].y = -2; vertices[3].z = +10;
  rtcReleaseBuffer(vertexBuffer);
  rtcCommitBuffer(vertexBuffer);

  /* set triangles */
  RTCBuffer indexBuffer = rtcNewBufferHostDevice(g_device, sizeof(Triangle)*2);
  Triangle* triangles = (Triangle*) rtcGetBufferData(indexBuffer);
  rtcSetGeometryBuffer(geom,RTC_BUFFER_TYPE_INDEX,0,RTC_FORMAT_UINT3,indexBuffer,0,sizeof(Triangle),2);
  triangles[0].v0 = 0; triangles[0].v1 = 1; triangles[0].v2 = 2;
  triangles[1].v0 = 1; triangles[1].v1 = 3; triangles[1].v2 = 2;
  rtcReleaseBuffer(indexBuffer);
  rtcCommitBuffer(indexBuffer);

  rtcCommitGeometry(geom);
  unsigned int geomID = rtcAttachGeometry(scene_i,geom);
  rtcReleaseGeometry(geom);
  return geomID;
}

/* called by the C++ code for initialization */
extern "C" void device_init (char* cfg)
{
  TutorialData_Constructor(&data);
  
  /* create scene */
  data.g_scene = g_scene = rtcNewScene(g_device);
  rtcSetSceneFlags(data.g_scene,RTC_SCENE_FLAG_DYNAMIC | RTC_SCENE_FLAG_ROBUST);
  rtcSetSceneBuildQuality(data.g_scene,RTC_BUILD_QUALITY_LOW);

  /* create some triangulated spheres */
  for (int i=0; i<data.numSpheres; i++)
  {
    const float phi = i*2.0f*float(pi)/data.numSpheres;
    const float r = 2.0f*float(pi)/data.numSpheres;
    const Vec3fa p = 2.0f*Vec3fa(sin(phi),0.0f,-cos(phi));
    //RTCBuildQuality quality = i%3 == 0 ? RTC_BUILD_QUALITY_MEDIUM : i%3 == 1 ? RTC_BUILD_QUALITY_REFIT : RTC_BUILD_QUALITY_LOW;
    RTCBuildQuality quality = i%2 ? RTC_BUILD_QUALITY_REFIT : RTC_BUILD_QUALITY_LOW;
    //RTCBuildQuality quality = RTC_BUILD_QUALITY_REFIT;
    int id = createSphere(quality,p,r);
    data.position[id] = Vec3fa(p);
    data.radius[id] = r;
    data.colors[id].x = (i%16+1)/17.0f;
    data.colors[id].y = (i%8+1)/9.0f;
    data.colors[id].z = (i%4+1)/5.0f;
  }

  /* add ground plane to scene */
  int id = addGroundPlane(data.g_scene);
  data.colors[id] = Vec3fa(1.0f,1.0f,1.0f);

  /* commit changes to scene */
  rtcCommitScene (data.g_scene);
}

/* animates the sphere */
void animateSphere (int taskIndex, int threadIndex, Vertex* vertices,
                         const float rcpNumTheta,
                         const float rcpNumPhi,
                         const Vec3fa& pos,
                         const float r,
                         const float f)
{
  int phi = taskIndex;
  for (unsigned int theta=0; theta<data.numTheta; theta++)
  {
    Vertex* v = &vertices[phi*data.numTheta+theta];
    const float phif   = phi*float(pi)*rcpNumPhi;
    const float thetaf = theta*2.0f*float(pi)*rcpNumTheta;
    v->x = pos.x + r*sin(f*phif)*sin(thetaf);
    v->y = pos.y + r*cos(phif);
    v->z = pos.z + r*sin(f*phif)*cos(thetaf);
  }
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
  Ray ray(Vec3fa(camera.xfm.p), Vec3fa(normalize(x*camera.xfm.l.vx + y*camera.xfm.l.vy + camera.xfm.l.vz)), 0.0f, inf);

  /* intersect ray with scene */
  RTCIntersectArguments iargs;
  rtcInitIntersectArguments(&iargs);
  iargs.feature_mask = (RTCFeatureFlags) (FEATURE_MASK);
  
  rtcTraversableIntersect1(data.g_traversable,RTCRayHit_(ray),&iargs);
  RayStats_addRay(stats);

  /* shade pixels */
  Vec3fa color = Vec3fa(0.0f);
  if (ray.geomID != RTC_INVALID_GEOMETRY_ID)
  {
    Vec3fa diffuse = Vec3fa(data.colors[ray.geomID]);
    color = color + diffuse*0.1f;
    Vec3fa lightDir = normalize(Vec3fa(-1,-1,-1));

    /* initialize shadow ray */
    Ray shadow(ray.org + ray.tfar*ray.dir, neg(lightDir), 0.001f, inf);

    /* trace shadow ray */
    RTCOccludedArguments sargs;
    rtcInitOccludedArguments(&sargs);
    sargs.feature_mask = (RTCFeatureFlags) (FEATURE_MASK);
    rtcTraversableOccluded1(data.g_traversable,RTCRay_(shadow),&sargs);
    RayStats_addShadowRay(stats);

    /* add light contribution */
    if (shadow.tfar >= 0.0f)
      color = color + diffuse*clamp(-dot(lightDir,normalize(ray.Ng)),0.0f,1.0f);
  }

  /* write color to framebuffer */
  unsigned int r = (unsigned int) (255.0f * clamp(color.x,0.0f,1.0f));
  unsigned int g = (unsigned int) (255.0f * clamp(color.y,0.0f,1.0f));
  unsigned int b = (unsigned int) (255.0f * clamp(color.z,0.0f,1.0f));
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

/* animates a sphere */
void animateSphere (int id, float time)
{
  /* animate vertices */
  RTCGeometry geom = rtcGetGeometry(data.g_scene,id);
  Vertex* vertices = (Vertex*) rtcGetGeometryBufferData(geom,RTC_BUFFER_TYPE_VERTEX,0);
  const float rcpNumTheta = rcp((float)data.numTheta);
  const float rcpNumPhi   = rcp((float)data.numPhi);
  const Vec3fa pos = Vec3fa(data.position[id]);
  const float r = data.radius[id];
  const float f = 2.0f*(1.0f+0.5f*sin(time));

  /* loop over all vertices */
//#if !defined(EMBREE_SYCL_TUTORIAL) // enables parallel execution
  parallel_for(size_t(0),size_t(data.numPhi+1),[&](const range<size_t>& range) {
    const int threadIndex = (int)TaskScheduler::threadIndex();
    for (size_t i=range.begin(); i<range.end(); i++)
      animateSphere((int)i,threadIndex,vertices,rcpNumTheta,rcpNumPhi,pos,r,f);
  }); 
//#else
//  for (unsigned int phi=0; phi<data.numPhi+1; phi++) for (int theta=0; theta<data.numTheta; theta++)
//  {
//    Vertex* v = &vertices[phi*data.numTheta+theta];
//    const float phif   = phi*float(pi)*rcpNumPhi;
//    const float thetaf = theta*2.0f*float(pi)*rcpNumTheta;
//    v->x = pos.x+r*sin(f*phif)*sin(thetaf);
//    v->y = pos.y+r*cos(phif);
//    v->z = pos.z+r*sin(f*phif)*cos(thetaf);
//  }
//#endif

  /* commit mesh */
  rtcUpdateGeometryBuffer(geom,RTC_BUFFER_TYPE_VERTEX,0);
  rtcCommitGeometry(geom);
}

extern "C" void renderFrameStandard (int* pixels,
                          const unsigned int width,
                          const unsigned int height,
                          const float time,
                          const ISPCCamera& camera)
{
  /* render all pixels */
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
  auto start_animate = std::chrono::high_resolution_clock::now();
  /* animate sphere */
  for (int i=0; i<data.numSpheres; i++)
    animateSphere(i,time+i);
  auto end_animate = std::chrono::high_resolution_clock::now();

  /* commit changes to scene */
  auto start_commit = std::chrono::high_resolution_clock::now();
#if defined(EMBREE_SYCL_TUTORIAL)
  rtcCommitSceneWithQueue (data.g_scene, *global_gpu_queue);
#else
  rtcCommitScene (data.g_scene);
#endif
  auto end_commit = std::chrono::high_resolution_clock::now();

  auto duration_animate = std::chrono::duration_cast<std::chrono::milliseconds>(end_animate - start_animate);
  auto duration_commit = std::chrono::duration_cast<std::chrono::milliseconds>(end_commit - start_commit);

  std::cout << "animate took " << duration_animate.count() << " milliseconds to execute." << std::endl;
  std::cout << "commit took " << duration_commit.count() << " milliseconds to execute." << std::endl;
  data.g_traversable = rtcGetSceneTraversable(data.g_scene);
}

/* called by the C++ code for cleanup */
extern "C" void device_cleanup ()
{
  TutorialData_Destructor(&data);
}

} // namespace embree
