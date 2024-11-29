// Copyright 2009-2021 Intel Corporation
// SPDX-License-Identifier: Apache-2.0

#include "dynamic_scene_device.isph"

/* all features required by this tutorial */
#define FEATURE_MASK \
  RTC_FEATURE_FLAG_TRIANGLE

RTCScene g_scene = NULL;
uniform TutorialData data;

/* adds a sphere to the scene */
uniform unsigned int createSphere (uniform RTCBuildQuality quality, const uniform Vec3f& pos, const uniform float r)
{
  /* create a triangulated sphere */
  RTCGeometry geom = rtcNewGeometry (g_device, RTC_GEOMETRY_TYPE_TRIANGLE);
  rtcSetGeometryBuildQuality(geom, quality);

  /* map triangle and vertex buffer */
  uniform Vertex*   uniform vertices  = (uniform Vertex*   uniform) rtcSetNewGeometryBuffer(geom,RTC_BUFFER_TYPE_VERTEX,0,RTC_FORMAT_FLOAT3,sizeof(uniform Vertex),data.numTheta*(data.numPhi+1));
  uniform Triangle* uniform triangles = (uniform Triangle* uniform) rtcSetNewGeometryBuffer(geom,RTC_BUFFER_TYPE_INDEX,0,RTC_FORMAT_UINT3,sizeof(uniform Triangle),2*data.numTheta*(data.numPhi-1));

  /* create sphere geometry */
  uniform int tri = 0;
  const uniform float rcpNumTheta = rcp((uniform float)data.numTheta);
  const uniform float rcpNumPhi   = rcp((uniform float)data.numPhi);
  for (uniform int phi=0; phi<=data.numPhi; phi++)
  {
    for (uniform int theta=0; theta<data.numTheta; theta++)
    {
      const uniform float phif   = phi*pi*rcpNumPhi;
      const uniform float thetaf = theta*2.0f*pi*rcpNumTheta;
      uniform Vertex& v = vertices[phi*data.numTheta+theta];
      v.x = pos.x + r*sin(phif)*sin(thetaf);
      v.y = pos.y + r*cos(phif);
      v.z = pos.z + r*sin(phif)*cos(thetaf);
    }
    if (phi == 0) continue;

    for (uniform int theta=1; theta<=data.numTheta; theta++)
    {
      uniform int p00 = (phi-1)*data.numTheta+theta-1;
      uniform int p01 = (phi-1)*data.numTheta+theta%data.numTheta;
      uniform int p10 = phi*data.numTheta+theta-1;
      uniform int p11 = phi*data.numTheta+theta%data.numTheta;

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
  uniform unsigned int geomID = rtcAttachGeometry(data.g_scene,geom);
  rtcReleaseGeometry(geom);
  return geomID;
}

/* adds a ground plane to the scene */
uniform unsigned int addGroundPlane (RTCScene scene_i)
{
  /* create a triangulated plane with 2 triangles and 4 vertices */
  RTCGeometry geom = rtcNewGeometry (g_device, RTC_GEOMETRY_TYPE_TRIANGLE);

  /* set vertices */
  uniform Vertex* uniform vertices = (uniform Vertex* uniform) rtcSetNewGeometryBuffer(geom,RTC_BUFFER_TYPE_VERTEX,0,RTC_FORMAT_FLOAT3,sizeof(uniform Vertex),4);
  vertices[0].x = -10; vertices[0].y = -2; vertices[0].z = -10;
  vertices[1].x = -10; vertices[1].y = -2; vertices[1].z = +10;
  vertices[2].x = +10; vertices[2].y = -2; vertices[2].z = -10;
  vertices[3].x = +10; vertices[3].y = -2; vertices[3].z = +10;

  /* set triangles */
  uniform Triangle* uniform triangles = (uniform Triangle* uniform) rtcSetNewGeometryBuffer(geom,RTC_BUFFER_TYPE_INDEX,0,RTC_FORMAT_UINT3,sizeof(uniform Triangle),2);
  triangles[0].v0 = 0; triangles[0].v1 = 1; triangles[0].v2 = 2;
  triangles[1].v0 = 1; triangles[1].v1 = 3; triangles[1].v2 = 2;

  rtcCommitGeometry(geom);
  uniform unsigned int geomID = rtcAttachGeometry(scene_i,geom);
  rtcReleaseGeometry(geom);
  return geomID;
}

/* called by the C++ code for initialization */
export void device_init (uniform int8* uniform cfg)
{
  TutorialData_Constructor(&data);
  
  /* create scene */
  data.g_scene = g_scene = rtcNewScene(g_device);
  rtcSetSceneFlags(data.g_scene,RTC_SCENE_FLAG_DYNAMIC | RTC_SCENE_FLAG_ROBUST);
  rtcSetSceneBuildQuality(data.g_scene,RTC_BUILD_QUALITY_LOW);

  /* create some triangulated spheres */
  for (uniform int i=0; i<data.numSpheres; i++)
  {
    const uniform float phi = i*2.0f*pi/data.numSpheres;
    const uniform float r = 2.0f*pi/data.numSpheres;
    const uniform Vec3f p = 2.0f*make_Vec3f(sin(phi),0.0f,-cos(phi));
    //uniform RTCBuildQuality quality = i%3 == 0 ? RTC_BUILD_QUALITY_MEDIUM : i%3 == 1 ? RTC_BUILD_QUALITY_REFIT : RTC_BUILD_QUALITY_LOW;
    uniform RTCBuildQuality quality = i%2 ? RTC_BUILD_QUALITY_REFIT : RTC_BUILD_QUALITY_LOW;
    //uniform RTCBuildQuality quality = RTC_BUILD_QUALITY_REFIT;
    uniform int id = createSphere(quality,p,r);
    data.position[id] = make_Vec3fa(p);
    data.radius[id] = r;
    data.colors[id].x = (i%16+1)/17.0f;
    data.colors[id].y = (i%8+1)/9.0f;
    data.colors[id].z = (i%4+1)/5.0f;
  }

  /* add ground plane to scene */
  uniform int id = addGroundPlane(data.g_scene);
  data.colors[id] = make_Vec3fa(1.0f,1.0f,1.0f);

  /* commit changes to scene */
  rtcCommitScene (data.g_scene);
}

/* animates the sphere */
task void animateSphere (uniform Vertex* uniform vertices,
                         const uniform float rcpNumTheta,
                         const uniform float rcpNumPhi,
                         const uniform Vec3f& pos,
                         const uniform float r,
                         const uniform float f)
{
  uniform int phi = taskIndex;
  foreach (theta = 0 ... data.numTheta)
  {
    uniform Vertex* v = &vertices[phi*data.numTheta+theta];
    const float phif   = phi*pi*rcpNumPhi;
    const float thetaf = theta*2.0f*pi*rcpNumTheta;
    v->x = pos.x + r*sin(f*phif)*sin(thetaf);
    v->y = pos.y + r*cos(phif);
    v->z = pos.z + r*sin(f*phif)*cos(thetaf);
  }
}

/* task that renders a single screen tile */
void renderPixelStandard(const uniform TutorialData& data,
                         int x, int y, 
                         uniform int* uniform pixels,
                         const uniform unsigned int width,
                         const uniform unsigned int height,
                         const uniform float time,
                         const uniform ISPCCamera& camera, uniform RayStats& stats)
{
  /* initialize ray */
  Ray ray = make_Ray(make_Vec3f(camera.xfm.p), make_Vec3f(normalize(x*camera.xfm.l.vx + y*camera.xfm.l.vy + camera.xfm.l.vz)), 0.0f, inf);

  /* intersect ray with scene */
  uniform RTCIntersectArguments iargs;
  rtcInitIntersectArguments(&iargs);
  iargs.feature_mask = (uniform RTCFeatureFlags) (FEATURE_MASK);
  
  rtcTraversableIntersectV(data.g_traversable,RTCRayHit_(ray),&iargs);
  RayStats_addRay(stats);

  /* shade pixels */
  Vec3f color = make_Vec3f(0.0f);
  if (ray.geomID != RTC_INVALID_GEOMETRY_ID)
  {
    Vec3f diffuse = make_Vec3f(data.colors[ray.geomID]);
    color = color + diffuse*0.1f;
    Vec3f lightDir = normalize(make_Vec3f(-1,-1,-1));

    /* initialize shadow ray */
    Ray shadow = make_Ray(ray.org + ray.tfar*ray.dir, neg(lightDir), 0.001f, inf);

    /* trace shadow ray */
    uniform RTCOccludedArguments sargs;
    rtcInitOccludedArguments(&sargs);
    sargs.feature_mask = (uniform RTCFeatureFlags) (FEATURE_MASK);
    rtcTraversableOccludedV(data.g_traversable,RTCRay_(shadow),&sargs);
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

/* animates a sphere */
void animateSphere (uniform int id, uniform float time)
{
  /* animate vertices */
  RTCGeometry geom = rtcGetGeometry(data.g_scene,id);
  uniform Vertex* uniform vertices = (uniform Vertex* uniform) rtcGetGeometryBufferData(geom,RTC_BUFFER_TYPE_VERTEX,0);
  const uniform float rcpNumTheta = rcp((uniform float)data.numTheta);
  const uniform float rcpNumPhi   = rcp((uniform float)data.numPhi);
  const uniform Vec3f pos = make_Vec3f(data.position[id]);
  const uniform float r = data.radius[id];
  const uniform float f = 2.0f*(1.0f+0.5f*sin(time));

  /* loop over all vertices */
#if !defined(EMBREE_SYCL_TUTORIAL) // enables parallel execution
  launch[data.numPhi+1] animateSphere(vertices,rcpNumTheta,rcpNumPhi,pos,r,f); sync;
#else
  foreach (phi = 0 ... data.numPhi+1, theta = 0 ... data.numTheta)
  {
    uniform Vertex* v = &vertices[phi*data.numTheta+theta];
    const float phif   = phi*pi*rcpNumPhi;
    const float thetaf = theta*2.0f*pi*rcpNumTheta;
    v->x = pos.x+r*sin(f*phif)*sin(thetaf);
    v->y = pos.y+r*cos(phif);
    v->z = pos.z+r*sin(f*phif)*cos(thetaf);
  }
#endif

  /* commit mesh */
  rtcUpdateGeometryBuffer(geom,RTC_BUFFER_TYPE_VERTEX,0);
  rtcCommitGeometry(geom);
}

export void renderFrameStandard (uniform int* uniform pixels,
                          const uniform unsigned int width,
                          const uniform unsigned int height,
                          const uniform float time,
                          const uniform ISPCCamera& camera)
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
  /* animate sphere */
  for (uniform int i=0; i<data.numSpheres; i++)
    animateSphere(i,time+i);

  /* commit changes to scene */
  rtcCommitScene (data.g_scene);
  data.g_traversable = rtcGetSceneTraversable(data.g_scene);
}

/* called by the C++ code for cleanup */
export void device_cleanup ()
{
  TutorialData_Destructor(&data);
}
