// Copyright 2009-2021 Intel Corporation
// SPDX-License-Identifier: Apache-2.0

#include "point_geometry_device.h"

namespace embree {

/* all features required by this tutorial */
#define FEATURE_MASK \
  RTC_FEATURE_FLAG_TRIANGLE | \
  RTC_FEATURE_FLAG_ORIENTED_DISC_POINT | \
  RTC_FEATURE_FLAG_DISC_POINT | \
  RTC_FEATURE_FLAG_SPHERE_POINT

RTCScene  g_scene  = nullptr;
TutorialData data;

/* add point geometry */
void addPoints (RTCScene scene, RTCGeometryType gtype, const Vec3fa& pos)
{
  RandomSampler rng;
  RandomSampler_init(rng, 42);

#define COORD RandomSampler_get1D(rng) * 4.f - 2.f
#define RADIUS RandomSampler_get1D(rng) * 0.13f + 0.02f
#define COLOR RandomSampler_get1D(rng)
#define NORMAL RandomSampler_get1D(rng) * 2.f - 1.f

  RTCGeometry geom = rtcNewGeometry (g_device, gtype);
  Vec4f* point_vertices = (Vec4f*)rtcSetNewGeometryBuffer(geom,RTC_BUFFER_TYPE_VERTEX, 0, RTC_FORMAT_FLOAT4, sizeof(Vec4f), NUM_POINTS);

  for (int i = 0; i < NUM_POINTS; i++)
  {
    const float vx = COORD;
    const float vy = COORD;
    const float vz = COORD;
    const float vr = RADIUS;
    point_vertices[i] = Vec4f(pos.x,pos.y,pos.z,0.0f) + Vec4f(vx, vy, vz, vr);
    const float cr = COLOR;
    const float cg = COLOR;
    const float cb = COLOR;
    data.point_colors[i] = Vec3fa(cr,cg,cb);
  }

  if (gtype == RTC_GEOMETRY_TYPE_ORIENTED_DISC_POINT) {
    Vec3fa* point_normals = (Vec3fa*)rtcSetNewGeometryBuffer(geom,RTC_BUFFER_TYPE_NORMAL, 0, RTC_FORMAT_FLOAT3, sizeof(Vec3fa), NUM_POINTS);
    for (int i = 0; i < NUM_POINTS; i++) {
      const float nx = NORMAL;
      const float ny = NORMAL;
      const float nz = NORMAL;
      point_normals[i] = Vec3fa(nx,ny,nz);
      point_normals[i] = normalize(point_normals[i]);
    }
  }

  rtcCommitGeometry(geom);
  rtcAttachGeometry(scene,geom);
  rtcReleaseGeometry(geom);
}

/* adds a ground plane to the scene */
unsigned int addGroundPlane (RTCScene scene_i)
{
  /* create a triangulated plane with 2 triangles and 4 vertices */
  RTCGeometry geom = rtcNewGeometry (g_device, RTC_GEOMETRY_TYPE_TRIANGLE);

  /* set vertices */
  Vertex* vertices = (Vertex*) rtcSetNewGeometryBuffer(geom, RTC_BUFFER_TYPE_VERTEX, 0, RTC_FORMAT_FLOAT3, sizeof(Vertex), 4);
  vertices[0].x = -10; vertices[0].y = -2; vertices[0].z = -10;
  vertices[1].x = -10; vertices[1].y = -2; vertices[1].z = +10;
  vertices[2].x = +10; vertices[2].y = -2; vertices[2].z = -10;
  vertices[3].x = +10; vertices[3].y = -2; vertices[3].z = +10;

  /* set triangles */
  Triangle* triangles = (Triangle*) rtcSetNewGeometryBuffer(geom, RTC_BUFFER_TYPE_INDEX, 0, RTC_FORMAT_UINT3, sizeof(Triangle), 2);
  triangles[0].v0 = 0; triangles[0].v1 = 1; triangles[0].v2 = 2;
  triangles[1].v0 = 1; triangles[1].v1 = 3; triangles[1].v2 = 2;

  rtcCommitGeometry(geom);
  unsigned int geomID = rtcAttachGeometry(scene_i,geom);
  rtcReleaseGeometry(geom);
  return geomID;
}

/* called by the C++ code for initialization */
extern "C" void device_init (char* cfg)
{
  /* create scene */
  TutorialData_Constructor(&data);
  g_scene = data.g_scene = rtcNewScene(g_device);

  /* add ground plane */
  addGroundPlane(g_scene);

  /* add curve */
  addPoints(g_scene, RTC_GEOMETRY_TYPE_SPHERE_POINT,        Vec3fa( 0.0f, 0.0f, 0.0f));
  addPoints(g_scene, RTC_GEOMETRY_TYPE_DISC_POINT,          Vec3fa( 5.0f, 0.0f, 0.0f));
  addPoints(g_scene, RTC_GEOMETRY_TYPE_ORIENTED_DISC_POINT, Vec3fa(-5.0f, 0.0f, 0.0f));

  /* commit changes to scene */
  rtcCommitScene (g_scene);
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
  iargs.feature_mask = (RTCFeatureFlags)(FEATURE_MASK);
  
  rtcIntersect1(data.g_scene,RTCRayHit_(ray),&iargs);
  RayStats_addRay(stats);

  /* shade pixels */
  Vec3fa color = Vec3fa(0.0f);
  if (ray.geomID != RTC_INVALID_GEOMETRY_ID)
  {
    /* interpolate diffuse color */
    Vec3fa diffuse = data.point_colors[ray.geomID ? ray.primID : 0];

    /* calculate smooth shading normal */
    Vec3fa Ng = normalize(ray.Ng);
    color = color + diffuse*0.5f;
    Vec3fa lightDir = normalize(Vec3fa(-1,-1,-1));

    /* initialize shadow ray */
    Ray shadow(ray.org + ray.tfar*ray.dir, neg(lightDir), 0.001f, inf, 0.0f);

    /* trace shadow ray */
    RTCOccludedArguments sargs;
    rtcInitOccludedArguments(&sargs);
    sargs.feature_mask = (RTCFeatureFlags)(FEATURE_MASK);
  
    rtcOccluded1(data.g_scene,RTCRay_(shadow),&sargs);
    RayStats_addShadowRay(stats);

    /* add light contribution */
    if (shadow.tfar >= 0.0f) {
      Vec3fa r = normalize(reflect(ray.dir,Ng));
      float s = pow(clamp(dot(r,lightDir),0.0f,1.0f),10.0f);
      float d = clamp(-dot(lightDir,Ng),0.0f,1.0f);
      color = color + diffuse*d + 0.5f*Vec3fa(s);
    }
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

extern "C" void renderFrameStandard (int* pixels,
                          const unsigned int width,
                          const unsigned int height,
                          const float time,
                          const ISPCCamera& camera)
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
