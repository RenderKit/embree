// Copyright 2009-2021 Intel Corporation
// SPDX-License-Identifier: Apache-2.0

#include "../common/tutorial/tutorial_device.isph"
#include "../common/math/random_sampler.isph"
#include "../common/math/sampling.isph"
#include "../common/tutorial/scene_device.h"

/* the scene to render */
extern RTCScene g_scene;
extern ISPCScene* uniform g_ispc_scene;

/* intensity scaling for traversal cost visualization */
extern uniform float scale;
extern uniform bool g_changed;

extern uniform float g_debug;

#if defined(EMBREE_SYCL_TUTORIAL) && !defined(EMBREE_SYCL_RT_SIMULATION)
static const sycl::specialization_id<RTCFeatureFlags> spec_feature_mask;
#endif

extern uniform RTCFeatureFlags g_feature_mask;

struct DebugShaderData
{
  RTCScene scene;
  ISPCScene* uniform ispc_scene;

  /* intensity scaling for traversal cost visualization */
  uniform float scale;

  uniform float debug;

  uniform Shader shader;
};

void DebugShaderData_Constructor(uniform DebugShaderData* uniform This)
{
  This->scene = g_scene;
  This->ispc_scene = g_ispc_scene;
  This->scale = scale;
  This->debug = g_debug;
  This->shader = shader;
}

#define RENDER_FRAME_FUNCTION_ISPC(Name)                             \
  void renderTile##Name(uniform int taskIndex,                  \
                        uniform int threadIndex,                \
                        const uniform DebugShaderData& data,    \
                        uniform int* uniform pixels,            \
                        const uniform unsigned int width,       \
                        const uniform unsigned int height,      \
                        const uniform float time,               \
                        const uniform ISPCCamera& camera,       \
                        const uniform int numTilesX,            \
                        const uniform int numTilesY)            \
  {                                                             \
    const uniform int t = taskIndex;                            \
    const uniform unsigned int tileY = t / numTilesX;           \
    const uniform unsigned int tileX = t - tileY * numTilesX;   \
    const uniform unsigned int x0 = tileX * TILE_SIZE_X;        \
    const uniform unsigned int x1 = min(x0+TILE_SIZE_X,width);  \
    const uniform unsigned int y0 = tileY * TILE_SIZE_Y;        \
    const uniform unsigned int y1 = min(y0+TILE_SIZE_Y,height); \
                                                                \
    foreach_tiled (y = y0 ... y1, x = x0 ... x1)                        \
    {                                                                   \
      Vec3f color = renderPixel##Name(data,(float)x,(float)y,camera,g_stats[threadIndex],RTC_FEATURE_FLAG_ALL); \
                                                                        \
      /* write color to framebuffer */                                  \
      unsigned int r = (unsigned int) (255.0f * clamp(color.x,0.0f,1.0f)); \
      unsigned int g = (unsigned int) (255.0f * clamp(color.y,0.0f,1.0f)); \
      unsigned int b = (unsigned int) (255.0f * clamp(color.z,0.0f,1.0f)); \
      pixels[y*width+x] = (b << 16) + (g << 8) + r;                     \
    }                                                                   \
  }                                                                     \
                                                                        \
  task void renderTileTask##Name(const uniform DebugShaderData& data,   \
                                 uniform int* uniform pixels,           \
                                 const uniform unsigned int width,      \
                                 const uniform unsigned int height,     \
                                 const uniform float time,              \
                                 const uniform ISPCCamera& camera,      \
                                 const uniform int numTilesX,           \
                                 const uniform int numTilesY)           \
  {                                                                     \
    renderTile##Name(taskIndex,threadIndex,data,pixels,width,height,time,camera,numTilesX,numTilesY); \
  }                                                                     \
                                                                        \
  export void renderFrame##Name (uniform int* uniform pixels,                  \
                          const uniform unsigned int width,             \
                          const uniform unsigned int height,            \
                          const uniform float time,                     \
                          const uniform ISPCCamera& camera)             \
  {                                                                     \
    uniform DebugShaderData data;                                       \
    DebugShaderData_Constructor(&data);                                 \
    const uniform int numTilesX = (width +TILE_SIZE_X-1)/TILE_SIZE_X;   \
    const uniform int numTilesY = (height+TILE_SIZE_Y-1)/TILE_SIZE_Y;   \
    launch[numTilesX*numTilesY] renderTileTask##Name(data,pixels,width,height,time,camera,numTilesX,numTilesY); sync; \
  }

#define RENDER_FRAME_FUNCTION_CPP(Name)                             \
  void renderTile##Name(uniform int taskIndex,                  \
                        uniform int threadIndex,                \
                        const DebugShaderData& data,            \
                        uniform int* uniform pixels,            \
                        const uniform unsigned int width,       \
                        const uniform unsigned int height,      \
                        const uniform float time,               \
                        const uniform ISPCCamera& camera,       \
                        const uniform int numTilesX,            \
                        const uniform int numTilesY)            \
  {                                                             \
    const uniform int t = taskIndex;                            \
    const uniform unsigned int tileY = t / numTilesX;           \
    const uniform unsigned int tileX = t - tileY * numTilesX;   \
    const uniform unsigned int x0 = tileX * TILE_SIZE_X;        \
    const uniform unsigned int x1 = min(x0+TILE_SIZE_X,width);  \
    const uniform unsigned int y0 = tileY * TILE_SIZE_Y;        \
    const uniform unsigned int y1 = min(y0+TILE_SIZE_Y,height); \
                                                                \
    foreach_tiled (y = y0 ... y1, x = x0 ... x1)                        \
    {                                                                   \
      Vec3f color = renderPixel##Name(data,(float)x,(float)y,camera,g_stats[threadIndex],RTC_FEATURE_FLAG_ALL); \
                                                                        \
      /* write color to framebuffer */                                  \
      unsigned int r = (unsigned int) (255.0f * clamp(color.x,0.0f,1.0f)); \
      unsigned int g = (unsigned int) (255.0f * clamp(color.y,0.0f,1.0f)); \
      unsigned int b = (unsigned int) (255.0f * clamp(color.z,0.0f,1.0f)); \
      pixels[y*width+x] = (b << 16) + (g << 8) + r;                     \
    }                                                                   \
  }                                                                     \
                                                                        \
  void renderTileTask##Name(int taskIndex, int threadIndex,             \
                            const DebugShaderData& data,                \
                            uniform int* uniform pixels,                \
                            const uniform unsigned int width,           \
                            const uniform unsigned int height,          \
                            const uniform float time,                   \
                            const uniform ISPCCamera& camera,           \
                            const uniform int numTilesX,                \
                            const uniform int numTilesY)                \
  {                                                                     \
    renderTile##Name(taskIndex,threadIndex,data,pixels,width,height,time,camera,numTilesX,numTilesY); \
  }                                                                     \
                                                                        \
  extern "C" void renderFrame##Name (uniform int* uniform pixels,                  \
                          const uniform unsigned int width,             \
                          const uniform unsigned int height,            \
                          const uniform float time,                     \
                          const uniform ISPCCamera& camera)             \
  {                                                                     \
    DebugShaderData data;                                               \
    DebugShaderData_Constructor(&data);                                 \
    const uniform int numTilesX = (width +TILE_SIZE_X-1)/TILE_SIZE_X;   \
    const uniform int numTilesY = (height+TILE_SIZE_Y-1)/TILE_SIZE_Y;   \
    parallel_for(size_t(0),size_t(numTilesX*numTilesY),[&](const range<size_t>& range) { \
      const int threadIndex = (int)TaskScheduler::threadIndex();        \
      for (size_t i=range.begin(); i<range.end(); i++)                  \
        renderTileTask##Name((int)i,threadIndex,data,pixels,width,height,time,camera,numTilesX,numTilesY); \
      });                                                               \
  }

#define RENDER_FRAME_FUNCTION_SYCL(Name)                                \
  extern "C" void renderFrame##Name (uniform int* uniform pixels,       \
                          const uniform unsigned int width,             \
                          const uniform unsigned int height,            \
                          const uniform float time,                     \
                          const uniform ISPCCamera& camera)             \
  {                                                                     \
    DebugShaderData data;                                               \
    DebugShaderData_Constructor(&data);                                 \
    sycl::event event;                                                  \
  \
  event = global_gpu_queue->submit([=](sycl::handler& cgh) {\
    cgh.set_specialization_constant<spec_feature_mask>(g_feature_mask);\
    const sycl::nd_range<2> nd_range = make_nd_range(height,width);   \
    cgh.parallel_for(nd_range,[=](sycl::nd_item<2> item, sycl::kernel_handler kh) { \
      const unsigned int x = item.get_global_id(1); if (x >= width ) return; \
      const unsigned int y = item.get_global_id(0); if (y >= height) return; \
      RayStats stats;                                                 \
      const RTCFeatureFlags feature_mask = kh.get_specialization_constant<spec_feature_mask>(); \
      Vec3fa color = renderPixel##Name(data,x,y,camera,stats,feature_mask); \
      unsigned int r = (unsigned int) (255.0f * clamp(color.x,0.0f,1.0f)); \
      unsigned int g = (unsigned int) (255.0f * clamp(color.y,0.0f,1.0f)); \
      unsigned int b = (unsigned int) (255.0f * clamp(color.z,0.0f,1.0f)); \
      pixels[y*width+x] = (b << 16) + (g << 8) + r;                   \
    });                                     \
  });                                \
  global_gpu_queue->wait_and_throw();\
  \
  const auto t0 = event.template get_profiling_info<sycl::info::event_profiling::command_start>();\
  const auto t1 = event.template get_profiling_info<sycl::info::event_profiling::command_end>();\
  const double dt = (t1-t0)*1E-9;\
  ((ISPCCamera*)&camera)->render_time = dt;\
}

Vec3f randomColor(const int ID)
{
  int r = ((ID+13)*17*23) & 255;
  int g = ((ID+15)*11*13) & 255;
  int b = ((ID+17)* 7*19) & 255;
  const float oneOver255f = 1.f/255.f;
  return make_Vec3f(r*oneOver255f,g*oneOver255f,b*oneOver255f);
}

/* renders a single pixel with eyelight shading */
Vec3f renderPixelDebugShader(const uniform DebugShaderData& data, float x, float y, const uniform ISPCCamera& camera, uniform RayStats& stats, const uniform RTCFeatureFlags feature_mask)
{
  /* initialize ray */
  Ray ray;
  ray.org = make_Vec3f_(camera.xfm.p);
  ray.dir = make_Vec3f_(normalize(x*camera.xfm.l.vx + y*camera.xfm.l.vy + camera.xfm.l.vz));
  ray.tnear = 0.0f;
  ray.tfar = inf;
  ray.geomID = RTC_INVALID_GEOMETRY_ID;
  ray.primID = RTC_INVALID_GEOMETRY_ID;
  ray.mask = -1;
  ray.time = data.debug;

  /* intersect ray with scene */
  uniform int64 c0 = get_tsc();
  if (data.shader == SHADER_OCCLUSION)
  {
    uniform RTCOccludedArguments args;
    rtcInitOccludedArguments(&args);
    args.feature_mask = feature_mask;
    rtcOccludedV(data.scene,RTCRay_(ray),&args);
  }
  else
  {
    uniform RTCIntersectArguments args;
    rtcInitIntersectArguments(&args);
    args.feature_mask = feature_mask;
    rtcIntersectV(data.scene,RTCRayHit_(ray),&args);
  }

  uniform int64 c1 = get_tsc();
  RayStats_addRay(stats);

  /* shade pixel */
  switch (data.shader)
  {
  case SHADER_EYELIGHT:
    if (ray.geomID == RTC_INVALID_GEOMETRY_ID)
      return make_Vec3f(0.0f);
    else if (dot(ray.dir,ray.Ng) < 0.0f)
      return make_Vec3f(0.0f,abs(dot(ray.dir,normalize(ray.Ng))),0.0f);
    else
      return make_Vec3f(abs(dot(ray.dir,normalize(ray.Ng))),0.0f,0.0f);

  case SHADER_OCCLUSION:
    if (ray.tfar >= 0.0f) 
      return make_Vec3f(0.0f,0.0f,0.0f);
    else 
      return make_Vec3f(1.0f,1.0f,1.0f);

  case SHADER_UV:
    if (ray.geomID == RTC_INVALID_GEOMETRY_ID) return make_Vec3f(0.0f,0.0f,0.0f);
    else return make_Vec3f(ray.u,ray.v,1.0f-ray.u-ray.v);

  case SHADER_TEXCOORDS:
  case SHADER_TEXCOORDS_GRID:

#if !defined(__SYCL_DEVICE_ONLY__)
    if (ray.geomID == RTC_INVALID_GEOMETRY_ID)
      return make_Vec3f(0.0f,0.0f,1.0f);

    else if (data.ispc_scene)
    {
      Vec2f st = make_Vec2f(0,0);
      foreach_unique (geomID in ray.geomID) {
        RTCGeometry geometry = rtcGetGeometry(data.scene,geomID);
        rtcInterpolateV0(geometry,ray.primID,ray.u,ray.v,RTC_BUFFER_TYPE_VERTEX_ATTRIBUTE,2,&st.x,2);
      }
      if (data.shader == SHADER_TEXCOORDS)
        return make_Vec3f(st.x,st.y,0.0f);
      else
        return ((int)(10.0f*st.x)+(int)(10.0f*st.y)) % 2 == 0 ? make_Vec3f(1,0,0) : make_Vec3f(0,1,0);
    }
#endif
    
    return make_Vec3f(1.0f);

  case SHADER_NG:

    if (ray.geomID == RTC_INVALID_GEOMETRY_ID) return make_Vec3f(0.0f,0.0f,0.0f);
    else return abs(normalize(make_Vec3f(ray.Ng.x,ray.Ng.y,ray.Ng.z)));
    //else return normalize(make_Vec3f(ray.Ng.x,ray.Ng.y,ray.Ng.z));

  case SHADER_GEOMID:

    if (ray.geomID == RTC_INVALID_GEOMETRY_ID) return make_Vec3f(0.0f);
    else return randomColor(ray.geomID);
    
  case SHADER_GEOMID_PRIMID:

    if (ray.geomID == RTC_INVALID_GEOMETRY_ID) return make_Vec3f(0.0f);
    else return randomColor(ray.geomID ^ ray.primID)*make_Vec3f(abs(dot(ray.dir,normalize(ray.Ng))));
    
  case SHADER_CYCLES:
    return make_Vec3f((uniform float)(c1-c0)*data.scale,0.0f,0.0f);
    
  case SHADER_DEFAULT:
    return make_Vec3f(0,0,0);
  }
  
  return make_Vec3f(0,0,0);
}

/* renders a single pixel with eyelight shading */
Vec3f renderPixelAOShader(const uniform DebugShaderData& data, float x, float y, const uniform ISPCCamera& camera, uniform RayStats& stats, const uniform RTCFeatureFlags feature_mask)
{
  /* initialize ray */
  Ray ray;
  ray.org = make_Vec3f_(camera.xfm.p);
  ray.dir = make_Vec3f_(normalize(x*camera.xfm.l.vx + y*camera.xfm.l.vy + camera.xfm.l.vz));
  ray.tnear = 0.0f;
  ray.tfar = inf;
  ray.geomID = RTC_INVALID_GEOMETRY_ID;
  ray.primID = RTC_INVALID_GEOMETRY_ID;
  ray.mask = -1;
  ray.time = data.debug;

  /* intersect ray with scene */
  uniform RTCIntersectArguments args;
  rtcInitIntersectArguments(&args);
  args.feature_mask = feature_mask;
  rtcIntersectV(data.scene,RTCRayHit_(ray),&args);
  RayStats_addRay(stats);

  /* shade pixel */
  if (ray.geomID == RTC_INVALID_GEOMETRY_ID) return make_Vec3f(0.0f);

  Vec3f Ng = normalize(ray.Ng);
  Vec3f Nf = faceforward(Ng,ray.dir,Ng);
  Vec3f col = make_Vec3f(min(1.f,.3f+.8f*abs(dot(Ng,normalize(ray.dir)))));

  /* calculate hit point */
  float intensity = 0;
  Vec3f hitPos = ray.org + ray.tfar * ray.dir;

#define AMBIENT_OCCLUSION_SAMPLES 64
  /* trace some ambient occlusion rays */
  RandomSampler sampler;
  RandomSampler_init(sampler, (int)x, (int)y, 0);
  for (uniform int i=0; i<AMBIENT_OCCLUSION_SAMPLES; i++)
  {
    Vec2f sample = RandomSampler_get2D(sampler);
    Sample3f dir = cosineSampleHemisphere(sample.x,sample.y,Nf);

    /* initialize shadow ray */
    Ray shadow;
    shadow.org = make_Vec3f_(hitPos);
    shadow.dir = make_Vec3f_(dir.v);
    shadow.tnear = 0.001f;
    shadow.tfar = inf;
    shadow.geomID = RTC_INVALID_GEOMETRY_ID;
    shadow.primID = RTC_INVALID_GEOMETRY_ID;
    shadow.mask = -1;
    shadow.time = data.debug;

    /* trace shadow ray */
    uniform RTCOccludedArguments args;
    rtcInitOccludedArguments(&args);
    args.feature_mask = feature_mask;
    rtcOccludedV(data.scene,RTCRay_(shadow),&args);
    RayStats_addShadowRay(stats);

    /* add light contribution */
    if (shadow.tfar >= 0.0f)
      intensity += 1.0f;
  }
  intensity *= 1.0f/AMBIENT_OCCLUSION_SAMPLES;

  /* shade pixel */
  return col * intensity;
}

RENDER_FRAME_FUNCTION_ISPC(DebugShader)
RENDER_FRAME_FUNCTION_ISPC(AOShader)
