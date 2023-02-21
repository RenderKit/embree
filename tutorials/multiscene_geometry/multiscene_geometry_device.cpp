// Copyright 2009-2021 Intel Corporation
// SPDX-License-Identifier: Apache-2.0

#include "multiscene_geometry_device.h"

namespace embree {

  /* all features required by this tutorial */
  #define FEATURE_MASK \
    RTC_FEATURE_FLAG_TRIANGLE
  
  /* scene data */
  RTCScene  g_scene = nullptr;
  RTCScene  g_scene_0 = nullptr;
  RTCScene  g_scene_1 = nullptr;
  RTCScene  g_scene_2 = nullptr;
  int g_scene_id = 0;
  TutorialData data;
  
  /* adds a sphere to the scene */
  unsigned int createSphere(RTCScene scene, RTCBuildQuality quality, const Vec3fa& pos, const float r)
  {
    /* create a triangulated sphere */
    RTCGeometry geom = rtcNewGeometry(g_device, RTC_GEOMETRY_TYPE_TRIANGLE);
    rtcSetGeometryBuildQuality(geom, quality);
    
    /* map triangle and vertex buffer */
    Vertex* vertices = (Vertex*)rtcSetNewGeometryBuffer(geom, RTC_BUFFER_TYPE_VERTEX, 0, RTC_FORMAT_FLOAT3, sizeof(Vertex), data.numTheta * (data.numPhi + 1));
    Triangle* triangles = (Triangle*)rtcSetNewGeometryBuffer(geom, RTC_BUFFER_TYPE_INDEX, 0, RTC_FORMAT_UINT3, sizeof(Triangle), 2 * data.numTheta * (data.numPhi - 1));
    
    /* create sphere geometry */
    int tri = 0;
    const float rcpNumTheta = rcp((float)data.numTheta);
    const float rcpNumPhi = rcp((float)data.numPhi);
    for (int phi = 0; phi <= data.numPhi; phi++)
    {
      for (int theta = 0; theta < data.numTheta; theta++)
      {
        const float phif = phi * float(pi) * rcpNumPhi;
        const float thetaf = theta * 2.0f * float(pi) * rcpNumTheta;
        Vertex& v = vertices[phi * data.numTheta + theta];
        v.x = pos.x + r * sin(phif) * sin(thetaf);
        v.y = pos.y + r * cos(phif);
        v.z = pos.z + r * sin(phif) * cos(thetaf);
      }
      if (phi == 0) continue;
      
      for (int theta = 1; theta <= data.numTheta; theta++)
      {
        int p00 = (phi - 1) * data.numTheta + theta - 1;
        int p01 = (phi - 1) * data.numTheta + theta % data.numTheta;
        int p10 = phi * data.numTheta + theta - 1;
        int p11 = phi * data.numTheta + theta % data.numTheta;
        
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
    
    rtcCommitGeometry(geom);
    unsigned int geomID = rtcAttachGeometry(scene, geom);
    rtcReleaseGeometry(geom);
    return geomID;
  }
  
  /* adds a ground plane to the scene */
  unsigned int addGroundPlane(RTCScene scene_i)
  {
    /* create a triangulated plane with 2 triangles and 4 vertices */
    RTCGeometry geom = rtcNewGeometry(g_device, RTC_GEOMETRY_TYPE_TRIANGLE);
    
    /* set vertices */
    Vertex* vertices = (Vertex*)rtcSetNewGeometryBuffer(geom, RTC_BUFFER_TYPE_VERTEX, 0, RTC_FORMAT_FLOAT3, sizeof(Vertex), 4);
    vertices[0].x = -10; vertices[0].y = -2; vertices[0].z = -10;
    vertices[1].x = -10; vertices[1].y = -2; vertices[1].z = +10;
    vertices[2].x = +10; vertices[2].y = -2; vertices[2].z = -10;
    vertices[3].x = +10; vertices[3].y = -2; vertices[3].z = +10;
    
    /* set triangles */
    Triangle* triangles = (Triangle*)rtcSetNewGeometryBuffer(geom, RTC_BUFFER_TYPE_INDEX, 0, RTC_FORMAT_UINT3, sizeof(Triangle), 2);
    triangles[0].v0 = 0; triangles[0].v1 = 1; triangles[0].v2 = 2;
    triangles[1].v0 = 1; triangles[1].v1 = 3; triangles[1].v2 = 2;
    
    rtcCommitGeometry(geom);
    unsigned int geomID = rtcAttachGeometry(scene_i, geom);
    rtcReleaseGeometry(geom);
    return geomID;
  }
  
  /* called by the C++ code for initialization */
  extern "C" void device_init(char* cfg)
  {
    /* create scene */
    TutorialData_Constructor(&data);
    // g_scene_id = g_scene_id;
    g_scene_0 = data.g_scene_0 = rtcNewScene(g_device);
    g_scene_1 = data.g_scene_1 = rtcNewScene(g_device);
    g_scene_2 = data.g_scene_2 = rtcNewScene(g_device);

    rtcSetSceneFlags(data.g_scene_0,RTC_SCENE_FLAG_DYNAMIC | RTC_SCENE_FLAG_ROBUST);
    rtcSetSceneBuildQuality(data.g_scene_0,RTC_BUILD_QUALITY_LOW);

    rtcSetSceneFlags(data.g_scene_1,RTC_SCENE_FLAG_DYNAMIC | RTC_SCENE_FLAG_ROBUST);
    rtcSetSceneBuildQuality(data.g_scene_1,RTC_BUILD_QUALITY_LOW);
    
    //rtcSetSceneFlags(data.g_scene_2,RTC_SCENE_FLAG_DYNAMIC | RTC_SCENE_FLAG_ROBUST);
    //rtcSetSceneBuildQuality(data.g_scene_2,RTC_BUILD_QUALITY_LOW);
    
    /* create some triangulated spheres */
    for (int i = 0; i < data.numSpheres; i++)
    {
      const float phi = i * 2.0f * float(pi) / data.numSpheres;
      const float r = 2.0f * float(pi) / data.numSpheres;
      const Vec3fa p = 2.0f * Vec3fa(sin(phi), 0.0f, -cos(phi));
      //RTCBuildQuality quality = i%3 == 0 ? RTC_BUILD_QUALITY_MEDIUM : i%3 == 1 ? RTC_BUILD_QUALITY_REFIT : RTC_BUILD_QUALITY_LOW;
      RTCBuildQuality quality = i % 2 ? RTC_BUILD_QUALITY_REFIT : RTC_BUILD_QUALITY_LOW;
      //RTCBuildQuality quality = RTC_BUILD_QUALITY_REFIT;
      //RTCBuildQuality quality = RTC_BUILD_QUALITY_LOW;
      int id0 = createSphere(data.g_scene_0, quality, p, r);
      data.position[id0] = p;
      data.radius[id0] = r;
      data.colors0[id0].x = (i % 16 + 1) / 17.0f;
      data.colors0[id0].y = (i % 8 + 1) / 9.0f;
      data.colors0[id0].z = (i % 4 + 1) / 5.0f;
      if (i < (data.numSpheres / 2)) {
        int id1 = rtcAttachGeometry(data.g_scene_1, rtcGetGeometry (data.g_scene_0, id0));
        data.colors1[id1] = data.colors0[id0];
      }
      else {
        int id2 = rtcAttachGeometry(data.g_scene_2, rtcGetGeometry(data.g_scene_0, id0));
        data.colors2[id2] = data.colors0[id0];
      }
    }
    
    /* add ground plane to scene */
    int id0 = addGroundPlane(data.g_scene_0);
    int id1 = rtcAttachGeometry(data.g_scene_1, rtcGetGeometry(data.g_scene_0, id0));
    int id2 = rtcAttachGeometry(data.g_scene_2, rtcGetGeometry(data.g_scene_0, id0));
    data.colors0[id0] = Vec3fa(1.0f, 1.0f, 1.0f);
    data.colors1[id1] = data.colors0[id0];
    data.colors2[id2] = data.colors0[id0];
    
    /* commit changes to scene */
    rtcCommitScene(data.g_scene_0);
    rtcCommitScene(data.g_scene_1);
    rtcCommitScene(data.g_scene_2);
    
    /* First render scene1 to check if it got properly
     * committed even though scene_0 containing all
     * geometries got already build. */
    g_scene_id = 1;
  }
  
  /* animates the sphere */
  void animateSphere(int taskIndex, int threadIndex, Vertex* vertices,
                     const float rcpNumTheta,
                     const float rcpNumPhi,
                     const Vec3fa& pos,
                     const float r,
                     const float f)
  {
    int phi = taskIndex;
    for (unsigned int theta = 0; theta < data.numTheta; theta++)
    {
      Vertex* v = &vertices[phi * data.numTheta + theta];
      const float phif = phi * float(pi) * rcpNumPhi;
      const float thetaf = theta * 2.0f * float(pi) * rcpNumTheta;
      v->x = pos.x + r * sin(f * phif) * sin(thetaf);
      v->y = pos.y + r * cos(phif);
      v->z = pos.z + r * sin(f * phif) * cos(thetaf);
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
    Ray ray(Vec3fa(camera.xfm.p), Vec3fa(normalize(x * camera.xfm.l.vx + y * camera.xfm.l.vy + camera.xfm.l.vz)), 0.0f, inf);
    
    /* intersect ray with scene */
    RTCIntersectArguments iargs;
    rtcInitIntersectArguments(&iargs);
    iargs.feature_mask = (RTCFeatureFlags) (FEATURE_MASK);
  
    rtcIntersect1(data.g_curr_scene, RTCRayHit_(ray),&iargs);
    RayStats_addRay(stats);
    
    /* shade pixels */
    Vec3fa color = Vec3fa(0.0f);
    if (ray.geomID != RTC_INVALID_GEOMETRY_ID)
    {
      Vec3fa diffuse = data.colors[ray.geomID];
      color = color + diffuse * 0.1f;
      Vec3fa lightDir = normalize(Vec3fa(-1, -1, -1));
      
      /* initialize shadow ray */
      Ray shadow(ray.org + ray.tfar * ray.dir, neg(lightDir), 0.001f, inf);
      
      /* trace shadow ray */
      RTCOccludedArguments sargs;
      rtcInitOccludedArguments(&sargs);
      sargs.feature_mask = (RTCFeatureFlags) (FEATURE_MASK);
      
      rtcOccluded1(data.g_curr_scene, RTCRay_(shadow),&sargs);
      RayStats_addShadowRay(stats);
      
      /* add light contribution */
      if (shadow.tfar >= 0.0f) {
        color = color + diffuse * clamp(-dot(lightDir, normalize(ray.Ng)), 0.0f, 1.0f);
      }
    }

    /* write color to framebuffer */
    unsigned int r = (unsigned int) (255.0f * clamp(color.x,0.0f,1.0f));
    unsigned int g = (unsigned int) (255.0f * clamp(color.y,0.0f,1.0f));
    unsigned int b = (unsigned int) (255.0f * clamp(color.z,0.0f,1.0f));
    pixels[y*width+x] = (b << 16) + (g << 8) + r;
  }
  
  /* task that renders a single screen tile */
  void renderTileTask(int taskIndex, int threadIndex, int* pixels,
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
  
  /* animates a sphere */
  void animateSphere(int id, float time)
  {
    /* animate vertices */
    RTCGeometry geom = rtcGetGeometry(data.g_scene_0, id);
    Vertex* vertices = (Vertex*)rtcGetGeometryBufferData(geom, RTC_BUFFER_TYPE_VERTEX, 0);
    const float rcpNumTheta = rcp((float)data.numTheta);
    const float rcpNumPhi = rcp((float)data.numPhi);
    const Vec3fa pos = data.position[id];
    const float r = data.radius[id];
    const float f = 2.0f * (1.0f + 0.5f * sin(time));
    
    /* loop over all vertices */
#if 0 // enables parallel execution
    parallel_for(size_t(0), size_t(data.numPhi + 1), [&](const range<size_t>& range) {
        const int threadIndex = (int)TaskScheduler::threadIndex();
        for (size_t i = range.begin(); i < range.end(); i++)
          animateSphere((int)i, threadIndex, vertices, rcpNumTheta, rcpNumPhi, pos, r, f);
      });
#else
    for (unsigned int phi = 0; phi < data.numPhi + 1; phi++) for (int theta = 0; theta < data.numTheta; theta++)
                                                        {
                                                          Vertex* v = &vertices[phi * data.numTheta + theta];
                                                          const float phif = phi * float(pi) * rcpNumPhi;
                                                          const float thetaf = theta * 2.0f * float(pi) * rcpNumTheta;
                                                          v->x = pos.x + r * sin(f * phif) * sin(thetaf);
                                                          v->y = pos.y + r * cos(phif);
                                                          v->z = pos.z + r * sin(f * phif) * cos(thetaf);
                                                        }
#endif
    
    /* commit mesh */
    rtcUpdateGeometryBuffer(geom, RTC_BUFFER_TYPE_VERTEX, 0);
    rtcCommitGeometry(geom);
  }

  extern "C" void renderFrameStandard(int* pixels,
                           const unsigned int width,
                           const unsigned int height,
                           const float time,
                           const ISPCCamera & camera)
  {
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

    /* render all pixels */
    const int numTilesX = (width + TILE_SIZE_X - 1) / TILE_SIZE_X;
    const int numTilesY = (height + TILE_SIZE_Y - 1) / TILE_SIZE_Y;
    parallel_for(size_t(0), size_t(numTilesX * numTilesY), [&](const range<size_t>& range) {
        const int threadIndex = (int)TaskScheduler::threadIndex();
        for (size_t i = range.begin(); i < range.end(); i++)
          renderTileTask((int)i, threadIndex, pixels, width, height, time, camera, numTilesX, numTilesY);
      });
#endif
  }
  
  /* called by the C++ code to render */
  extern "C" void device_render(int* pixels,
                                const unsigned int width,
                                const unsigned int height,
                                const float time,
                                const ISPCCamera & camera)
  {
    /* configure rendering of selected scene */
    switch (g_scene_id)
    {
    case 0:
      data.g_curr_scene = data.g_scene_0;
      data.colors = &(data.colors0[0]);
      break;
    case 1:
      data.g_curr_scene = data.g_scene_1;
      data.colors = &(data.colors1[0]);
      break;
    case 2:
      data.g_curr_scene = data.g_scene_2;
      data.colors = &(data.colors2[0]);
      break;
    }
    
    /* animate sphere */
    for (int i = 0; i < data.numSpheres; i++)
      animateSphere(i, time + i);
    
    /* commit changes to scene */
    //rtcCommitScene(*data.g_curr_scene);
    rtcCommitScene(data.g_scene_0);
    rtcCommitScene(data.g_scene_1);
    rtcCommitScene(data.g_scene_2);
  }
  
  /* called by the C++ code for cleanup */
  extern "C" void device_cleanup()
  {
    TutorialData_Destructor(&data);
  }
  
} // namespace embree
