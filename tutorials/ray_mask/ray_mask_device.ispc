// Copyright 2009-2021 Intel Corporation
// SPDX-License-Identifier: Apache-2.0

#include "ray_mask_device.isph"

/* all features required by this tutorial */
#define FEATURE_MASK \
  RTC_FEATURE_FLAG_TRIANGLE | \
  RTC_FEATURE_FLAG_32_BIT_RAY_MASK
  
uniform const unsigned int MASK_PV_SV = (1 <<  0); // geometry mask, primary rays visible,   secondary rays visible
uniform const unsigned int MASK_PI_SV = (1 <<  2); // geometry mask, primary rays invisible, secondary rays visible
uniform const unsigned int MASK_PV_SI = (1 << 10); // geometry mask, primary rays visible,   secondary rays invisible

RTCScene g_scene = NULL;
uniform TutorialData data;

extern uniform bool g_ray_mask;

/* adds a cube to the scene */
uniform unsigned int addCube (RTCScene scene_i, const uniform Vec3f& d, uniform unsigned int mask)
{
  /* create a triangulated cube with 12 triangles and 8 vertices */
  RTCGeometry mesh = rtcNewGeometry(g_device, RTC_GEOMETRY_TYPE_TRIANGLE);

  /* set vertices and vertex colors */
  uniform Vertex* uniform vertices = (uniform Vertex* uniform) rtcSetNewGeometryBuffer(mesh,RTC_BUFFER_TYPE_VERTEX,0,RTC_FORMAT_FLOAT3,sizeof(uniform Vertex),8);
  data.vertex_colors[0] = make_Vec3fa(0,0,0); vertices[0].x = -1 + d.x; vertices[0].y = -1 + d.y; vertices[0].z = -1 + d.z;
  data.vertex_colors[1] = make_Vec3fa(0,0,1); vertices[1].x = -1 + d.x; vertices[1].y = -1 + d.y; vertices[1].z = +1 + d.z;
  data.vertex_colors[2] = make_Vec3fa(0,1,0); vertices[2].x = -1 + d.x; vertices[2].y = +1 + d.y; vertices[2].z = -1 + d.z;
  data.vertex_colors[3] = make_Vec3fa(0,1,1); vertices[3].x = -1 + d.x; vertices[3].y = +1 + d.y; vertices[3].z = +1 + d.z;
  data.vertex_colors[4] = make_Vec3fa(1,0,0); vertices[4].x = +1 + d.x; vertices[4].y = -1 + d.y; vertices[4].z = -1 + d.z;
  data.vertex_colors[5] = make_Vec3fa(1,0,1); vertices[5].x = +1 + d.x; vertices[5].y = -1 + d.y; vertices[5].z = +1 + d.z;
  data.vertex_colors[6] = make_Vec3fa(1,1,0); vertices[6].x = +1 + d.x; vertices[6].y = +1 + d.y; vertices[6].z = -1 + d.z;
  data.vertex_colors[7] = make_Vec3fa(1,1,1); vertices[7].x = +1 + d.x; vertices[7].y = +1 + d.y; vertices[7].z = +1 + d.z;

  /* set triangles and face colors */
  uniform int tri = 0;
  uniform Triangle* uniform triangles = (uniform Triangle* uniform) rtcSetNewGeometryBuffer(mesh,RTC_BUFFER_TYPE_INDEX,0,RTC_FORMAT_UINT3,sizeof(uniform Triangle),12);

  // left side
  data.face_colors[tri] = make_Vec3f(1,0,0); triangles[tri].v0 = 0; triangles[tri].v1 = 1; triangles[tri].v2 = 2; tri++;
  data.face_colors[tri] = make_Vec3f(1,0,0); triangles[tri].v0 = 1; triangles[tri].v1 = 3; triangles[tri].v2 = 2; tri++;

  // right side
  data.face_colors[tri] = make_Vec3f(0,1,0); triangles[tri].v0 = 4; triangles[tri].v1 = 6; triangles[tri].v2 = 5; tri++;
  data.face_colors[tri] = make_Vec3f(0,1,0); triangles[tri].v0 = 5; triangles[tri].v1 = 6; triangles[tri].v2 = 7; tri++;

  // bottom side
  data.face_colors[tri] = make_Vec3f(0.5f);  triangles[tri].v0 = 0; triangles[tri].v1 = 4; triangles[tri].v2 = 1; tri++;
  data.face_colors[tri] = make_Vec3f(0.5f);  triangles[tri].v0 = 1; triangles[tri].v1 = 4; triangles[tri].v2 = 5; tri++;

  // top side
  data.face_colors[tri] = make_Vec3f(1.0f);  triangles[tri].v0 = 2; triangles[tri].v1 = 3; triangles[tri].v2 = 6; tri++;
  data.face_colors[tri] = make_Vec3f(1.0f);  triangles[tri].v0 = 3; triangles[tri].v1 = 7; triangles[tri].v2 = 6; tri++;

  // front side
  data.face_colors[tri] = make_Vec3f(0,0,1); triangles[tri].v0 = 0; triangles[tri].v1 = 2; triangles[tri].v2 = 4; tri++;
  data.face_colors[tri] = make_Vec3f(0,0,1); triangles[tri].v0 = 2; triangles[tri].v1 = 6; triangles[tri].v2 = 4; tri++;

  // back side
  data.face_colors[tri] = make_Vec3f(1,1,0); triangles[tri].v0 = 1; triangles[tri].v1 = 5; triangles[tri].v2 = 3; tri++;
  data.face_colors[tri] = make_Vec3f(1,1,0); triangles[tri].v0 = 3; triangles[tri].v1 = 5; triangles[tri].v2 = 7; tri++;

  rtcSetGeometryVertexAttributeCount(mesh,1);
  rtcSetSharedGeometryBuffer(mesh,RTC_BUFFER_TYPE_VERTEX_ATTRIBUTE,0,RTC_FORMAT_FLOAT3,data.vertex_colors,0,sizeof(uniform Vec3fa),8);
  rtcSetGeometryMask(mesh,mask);
  
  rtcCommitGeometry(mesh);
  uniform unsigned int geomID = rtcAttachGeometry(scene_i,mesh);
  rtcReleaseGeometry(mesh);
  return geomID;
}

/* adds a ground plane to the scene */
uniform unsigned int addGroundPlane (RTCScene scene_i)
{
  /* create a triangulated plane with 2 triangles and 4 vertices */
  RTCGeometry mesh = rtcNewGeometry (g_device, RTC_GEOMETRY_TYPE_TRIANGLE);

  /* set vertices */
  uniform Vertex* uniform vertices = (uniform Vertex* uniform) rtcSetNewGeometryBuffer(mesh,RTC_BUFFER_TYPE_VERTEX,0,RTC_FORMAT_FLOAT3,sizeof(uniform Vertex),4);
  vertices[0].x = -10; vertices[0].y = -2; vertices[0].z = -10;
  vertices[1].x = -10; vertices[1].y = -2; vertices[1].z = +10;
  vertices[2].x = +10; vertices[2].y = -2; vertices[2].z = -10;
  vertices[3].x = +10; vertices[3].y = -2; vertices[3].z = +10;

  /* set triangles */
  uniform Triangle* uniform triangles = (uniform Triangle* uniform) rtcSetNewGeometryBuffer(mesh,RTC_BUFFER_TYPE_INDEX,0,RTC_FORMAT_UINT3,sizeof(uniform Triangle),2);
  triangles[0].v0 = 0; triangles[0].v1 = 1; triangles[0].v2 = 2;
  triangles[1].v0 = 1; triangles[1].v1 = 3; triangles[1].v2 = 2;
  
  rtcCommitGeometry(mesh);
  uniform unsigned int geomID = rtcAttachGeometry(scene_i,mesh);
  rtcReleaseGeometry(mesh);
  return geomID;
}

/* called by the C++ code for initialization */
export void device_init (uniform int8* uniform cfg)
{ 
  /* create scene */
  TutorialData_Constructor(&data);
  g_scene = data.g_scene = rtcNewScene(g_device);

  /* create face and vertex color arrays */
  data.face_colors = uniform new uniform Vec3f[12];
  data.vertex_colors = uniform new uniform Vec3fa[8];

  /* add cube */
  addCube(data.g_scene,make_Vec3f(-3.f, 0.f, 0.f), MASK_PI_SV);
  addCube(data.g_scene,make_Vec3f( 0.f, 0.f, 0.f), MASK_PV_SV);
  addCube(data.g_scene,make_Vec3f( 3.f, 0.f, 0.f), MASK_PV_SI);

  /* add ground plane */
  addGroundPlane(data.g_scene);

  /* commit changes to scene */
  rtcCommitScene (data.g_scene);
  data.g_traversable = rtcGetSceneTraversable(data.g_scene);
}

/* task that renders a single screen tile */
void renderPixelStandard(const uniform TutorialData& data,
                         int x, int y, 
                         uniform int* uniform pixels,
                         const uniform unsigned int width,
                         const uniform unsigned int height,
                         const float time,
                         const uniform ISPCCamera& camera, uniform RayStats& stats)
{
  /* initialize ray */
  Ray ray = make_Ray(make_Vec3f(camera.xfm.p), make_Vec3f(normalize(x*camera.xfm.l.vx + y*camera.xfm.l.vy + camera.xfm.l.vz)), 0.0f, inf);
  if (data.enable_ray_mask)
    ray.mask = MASK_PV_SV + MASK_PV_SI;

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
    Vec3f diffuse = data.face_colors[ray.primID];
    color = color + diffuse*0.5f;
    Vec3f lightDir = normalize(make_Vec3f(-1,-1,-1));

    /* initialize shadow ray */
    Ray shadow = make_Ray(ray.org + ray.tfar*ray.dir, neg(lightDir), 0.001f, inf);
    if (data.enable_ray_mask)
      shadow.mask = MASK_PV_SV + MASK_PI_SV;

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

/* task that renders a single screen tile */
task void renderTileTask(uniform int* uniform pixels,
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

/* called by the C++ code to render */
export void renderFrameStandard (uniform int* uniform pixels,
                          const uniform unsigned int width,
                          const uniform unsigned int height,
                          const uniform float time,
                          const uniform ISPCCamera& camera)
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
  const uniform uint64 numTilesX = (width +TILE_SIZE_X-1)/TILE_SIZE_X;
  const uniform uint64 numTilesY = (height+TILE_SIZE_Y-1)/TILE_SIZE_Y;
  const uniform uint64 numTiles = numTilesX * numTilesY;
  launch[numTiles] renderTileTask(pixels,width,height,time,camera,numTilesX,numTilesY); sync;
#endif
}

/* called by the C++ code to render */
export void device_render (uniform int* uniform pixels,
                           const uniform unsigned int width,
                           const uniform unsigned int height,
                           const uniform float time,
                           const uniform ISPCCamera& camera)
{
  data.enable_ray_mask = g_ray_mask;
}

/* called by the C++ code for cleanup */
export void device_cleanup ()
{
  TutorialData_Destructor(&data);
}
