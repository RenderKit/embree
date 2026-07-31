// Copyright 2009-2021 Intel Corporation
// SPDX-License-Identifier: Apache-2.0

#include "../common/tutorial/tutorial_device.h"
#include "../common/math/random_sampler.h"
#include "../common/math/sampling.h"
#include "../common/tutorial/scene_device.h"

namespace embree {

/* the scene to render */
extern RTCScene g_scene;
extern "C" ISPCScene* g_ispc_scene;

/* intensity scaling for traversal cost visualization */
extern "C" float scale;
extern "C" bool g_changed;

extern "C" float g_debug;

#if defined(EMBREE_SYCL_TUTORIAL) && !defined(EMBREE_SYCL_RT_SIMULATION)
static const sycl::specialization_id<RTCFeatureFlags> spec_feature_mask;
#endif

extern "C" RTCFeatureFlags g_feature_mask;

struct DebugShaderData
{
  RTCScene scene;
  RTCTraversable traversable;
  ISPCScene* ispc_scene;

  /* intensity scaling for traversal cost visualization */
  float scale;

  float debug;

  Shader shader;
};

void DebugShaderData_Constructor(DebugShaderData* This)
{
  This->scene = g_scene;
  This->traversable = rtcGetSceneTraversable(g_scene);
  This->ispc_scene = g_ispc_scene;
  This->scale = scale;
  This->debug = g_debug;
  This->shader = shader;
}

#define RENDER_FRAME_FUNCTION_ISPC(Name)                             \
  void renderTile##Name(int taskIndex,                  \
                        int threadIndex,                \
                        const DebugShaderData& data,    \
                        int* pixels,            \
                        const unsigned int width,       \
                        const unsigned int height,      \
                        const float time,               \
                        const ISPCCamera& camera,       \
                        const int numTilesX,            \
                        const int numTilesY)            \
  {                                                             \
    const int t = taskIndex;                            \
    const unsigned int tileY = t / numTilesX;           \
    const unsigned int tileX = t - tileY * numTilesX;   \
    const unsigned int x0 = tileX * TILE_SIZE_X;        \
    const unsigned int x1 = min(x0+TILE_SIZE_X,width);  \
    const unsigned int y0 = tileY * TILE_SIZE_Y;        \
    const unsigned int y1 = min(y0+TILE_SIZE_Y,height); \
                                                                \
    for (unsigned int y=y0; y<y1; y++) for (unsigned int x=x0; x<x1; x++)                        \
    {                                                                   \
      Vec3fa color = renderPixel##Name(data,(float)x,(float)y,camera,g_stats[threadIndex],RTC_FEATURE_FLAG_ALL); \
                                                                        \
      /* write color to framebuffer */                                  \
      unsigned int r = (unsigned int) (255.0f * clamp(color.x,0.0f,1.0f)); \
      unsigned int g = (unsigned int) (255.0f * clamp(color.y,0.0f,1.0f)); \
      unsigned int b = (unsigned int) (255.0f * clamp(color.z,0.0f,1.0f)); \
      pixels[y*width+x] = (b << 16) + (g << 8) + r;                     \
    }                                                                   \
  }                                                                     \
                                                                        \
  task void renderTileTask##Name(const DebugShaderData& data,   \
                                 int* pixels,           \
                                 const unsigned int width,      \
                                 const unsigned int height,     \
                                 const float time,              \
                                 const ISPCCamera& camera,      \
                                 const int numTilesX,           \
                                 const int numTilesY)           \
  {                                                                     \
    renderTile##Name(taskIndex,threadIndex,data,pixels,width,height,time,camera,numTilesX,numTilesY); \
  }                                                                     \
                                                                        \
  extern "C" void renderFrame##Name (int* pixels,                  \
                          const unsigned int width,             \
                          const unsigned int height,            \
                          const float time,                     \
                          const ISPCCamera& camera)             \
  {                                                                     \
    DebugShaderData data;                                       \
    DebugShaderData_Constructor(&data);                                 \
    const int numTilesX = (width +TILE_SIZE_X-1)/TILE_SIZE_X;   \
    const int numTilesY = (height+TILE_SIZE_Y-1)/TILE_SIZE_Y;   \
    launch[numTilesX*numTilesY] renderTileTask##Name(data,pixels,width,height,time,camera,numTilesX,numTilesY);  \
  }

#define RENDER_FRAME_FUNCTION_CPP(Name)                             \
  void renderTile##Name(int taskIndex,                  \
                        int threadIndex,                \
                        const DebugShaderData& data,            \
                        int* pixels,            \
                        const unsigned int width,       \
                        const unsigned int height,      \
                        const float time,               \
                        const ISPCCamera& camera,       \
                        const int numTilesX,            \
                        const int numTilesY)            \
  {                                                             \
    const int t = taskIndex;                            \
    const unsigned int tileY = t / numTilesX;           \
    const unsigned int tileX = t - tileY * numTilesX;   \
    const unsigned int x0 = tileX * TILE_SIZE_X;        \
    const unsigned int x1 = min(x0+TILE_SIZE_X,width);  \
    const unsigned int y0 = tileY * TILE_SIZE_Y;        \
    const unsigned int y1 = min(y0+TILE_SIZE_Y,height); \
                                                                \
    for (unsigned int y=y0; y<y1; y++) for (unsigned int x=x0; x<x1; x++)                        \
    {                                                                   \
      Vec3fa color = renderPixel##Name(data,(float)x,(float)y,camera,g_stats[threadIndex],RTC_FEATURE_FLAG_ALL); \
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
                            int* pixels,                \
                            const unsigned int width,           \
                            const unsigned int height,          \
                            const float time,                   \
                            const ISPCCamera& camera,           \
                            const int numTilesX,                \
                            const int numTilesY)                \
  {                                                                     \
    renderTile##Name(taskIndex,threadIndex,data,pixels,width,height,time,camera,numTilesX,numTilesY); \
  }                                                                     \
                                                                        \
  extern "C" void renderFrame##Name (int* pixels,                  \
                          const unsigned int width,             \
                          const unsigned int height,            \
                          const float time,                     \
                          const ISPCCamera& camera)             \
  {                                                                     \
    DebugShaderData data;                                               \
    DebugShaderData_Constructor(&data);                                 \
    const int numTilesX = (width +TILE_SIZE_X-1)/TILE_SIZE_X;   \
    const int numTilesY = (height+TILE_SIZE_Y-1)/TILE_SIZE_Y;   \
    parallel_for(size_t(0),size_t(numTilesX*numTilesY),[&](const range<size_t>& range) { \
      const int threadIndex = (int)TaskScheduler::threadIndex();        \
      for (size_t i=range.begin(); i<range.end(); i++)                  \
        renderTileTask##Name((int)i,threadIndex,data,pixels,width,height,time,camera,numTilesX,numTilesY); \
      });                                                               \
  }

#define RENDER_FRAME_FUNCTION_SYCL(Name)                                \
  extern "C" void renderFrame##Name (int* pixels,       \
                          const unsigned int width,             \
                          const unsigned int height,            \
                          const float time,                     \
                          const ISPCCamera& camera)             \
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

Vec3fa randomColor(const int ID)
{
  int r = ((ID+13)*17*23) & 255;
  int g = ((ID+15)*11*13) & 255;
  int b = ((ID+17)* 7*19) & 255;
  const float oneOver255f = 1.f/255.f;
  return Vec3fa(r*oneOver255f,g*oneOver255f,b*oneOver255f);
}

/* renders a single pixel with eyelight shading */
Vec3fa renderPixelDebugShader(const DebugShaderData& data, float x, float y, const ISPCCamera& camera, RayStats& stats, const RTCFeatureFlags feature_mask)
{
  /* initialize ray */
  Ray ray;
  ray.org = Vec3ff(camera.xfm.p);
  ray.dir = Vec3ff(normalize(x*camera.xfm.l.vx + y*camera.xfm.l.vy + camera.xfm.l.vz));
  ray.tnear() = 0.0f;
  ray.tfar = inf;
  ray.geomID = RTC_INVALID_GEOMETRY_ID;
  ray.primID = RTC_INVALID_GEOMETRY_ID;
  ray.mask = -1;
  ray.time() = data.debug;

  /* intersect ray with scene */
  int64_t c0 = get_tsc();
  if (data.shader == SHADER_OCCLUSION)
  {
    RTCOccludedArguments args;
    rtcInitOccludedArguments(&args);
    args.feature_mask = feature_mask;
    rtcTraversableOccluded1(data.traversable,RTCRay_(ray),&args);
  }
  else
  {
    RTCIntersectArguments args;
    rtcInitIntersectArguments(&args);
    args.feature_mask = feature_mask;
    rtcTraversableIntersect1(data.traversable,RTCRayHit_(ray),&args);
  }

  int64_t c1 = get_tsc();
  RayStats_addRay(stats);

  /* shade pixel */
  switch (data.shader)
  {
  case SHADER_EYELIGHT:
    if (ray.geomID == RTC_INVALID_GEOMETRY_ID)
      return Vec3fa(0.0f);
    else if (dot(ray.dir,ray.Ng) < 0.0f)
      return Vec3fa(0.0f,abs(dot(ray.dir,normalize(ray.Ng))),0.0f);
    else
      return Vec3fa(abs(dot(ray.dir,normalize(ray.Ng))),0.0f,0.0f);

  case SHADER_OCCLUSION:
    if (ray.tfar >= 0.0f) 
      return Vec3fa(0.0f,0.0f,0.0f);
    else 
      return Vec3fa(1.0f,1.0f,1.0f);

  case SHADER_UV:
    if (ray.geomID == RTC_INVALID_GEOMETRY_ID) return Vec3fa(0.0f,0.0f,0.0f);
    else return Vec3fa(ray.u,ray.v,1.0f-ray.u-ray.v);

  case SHADER_TEXCOORDS:
  case SHADER_TEXCOORDS_GRID:

#if !defined(__SYCL_DEVICE_ONLY__)
    if (ray.geomID == RTC_INVALID_GEOMETRY_ID)
      return Vec3fa(0.0f,0.0f,1.0f);

    else if (data.ispc_scene)
    {
      Vec2f st = Vec2f(0,0);
      auto geomID = ray.geomID; {
        RTCGeometry geometry = rtcGetGeometry(data.scene,geomID);
        rtcInterpolate0(geometry,ray.primID,ray.u,ray.v,RTC_BUFFER_TYPE_VERTEX_ATTRIBUTE,2,&st.x,2);
      }
      if (data.shader == SHADER_TEXCOORDS)
        return Vec3fa(st.x,st.y,0.0f);
      else
        return ((int)(10.0f*st.x)+(int)(10.0f*st.y)) % 2 == 0 ? Vec3fa(1,0,0) : Vec3fa(0,1,0);
    }
#endif
    
    return Vec3fa(1.0f);

  case SHADER_NG:

    if (ray.geomID == RTC_INVALID_GEOMETRY_ID) return Vec3fa(0.0f,0.0f,0.0f);
    else return abs(normalize(Vec3fa(ray.Ng.x,ray.Ng.y,ray.Ng.z)));
    //else return normalize(Vec3fa(ray.Ng.x,ray.Ng.y,ray.Ng.z));

  case SHADER_GEOMID:

    if (ray.geomID == RTC_INVALID_GEOMETRY_ID) return Vec3fa(0.0f);
    else return randomColor(ray.geomID);
    
  case SHADER_GEOMID_PRIMID:

    if (ray.geomID == RTC_INVALID_GEOMETRY_ID) return Vec3fa(0.0f);
    else return randomColor(ray.geomID ^ ray.primID)*Vec3fa(abs(dot(ray.dir,normalize(ray.Ng))));
    
  case SHADER_CYCLES:
    return Vec3fa((float)(c1-c0)*data.scale,0.0f,0.0f);
    
  case SHADER_AO:
    return Vec3fa(0,0,0);

  case SHADER_DEFAULT:
    return Vec3fa(0,0,0);
  }
  
  return Vec3fa(0,0,0);
}

/* renders a single pixel with eyelight shading */
Vec3fa renderPixelAOShader(const DebugShaderData& data, float x, float y, const ISPCCamera& camera, RayStats& stats, const RTCFeatureFlags feature_mask)
{
  /* initialize ray */
  Ray ray;
  ray.org = Vec3ff(camera.xfm.p);
  ray.dir = Vec3ff(normalize(x*camera.xfm.l.vx + y*camera.xfm.l.vy + camera.xfm.l.vz));
  ray.tnear() = 0.0f;
  ray.tfar = inf;
  ray.geomID = RTC_INVALID_GEOMETRY_ID;
  ray.primID = RTC_INVALID_GEOMETRY_ID;
  ray.mask = -1;
  ray.time() = data.debug;

  /* intersect ray with scene */
  RTCIntersectArguments args;
  rtcInitIntersectArguments(&args);
  args.feature_mask = feature_mask;
  rtcTraversableIntersect1(data.traversable,RTCRayHit_(ray),&args);
  RayStats_addRay(stats);

  /* shade pixel */
  if (ray.geomID == RTC_INVALID_GEOMETRY_ID) return Vec3fa(0.0f);

  Vec3fa Ng = normalize(ray.Ng);
  Vec3fa Nf = faceforward(Ng,ray.dir,Ng);
  Vec3fa col = Vec3fa(min(1.f,.3f+.8f*abs(dot(Ng,normalize(ray.dir)))));

  /* calculate hit point */
  float intensity = 0;
  Vec3fa hitPos = ray.org + ray.tfar * ray.dir;

#define AMBIENT_OCCLUSION_SAMPLES 64
  /* trace some ambient occlusion rays */
  RandomSampler sampler;
  RandomSampler_init(sampler, (int)x, (int)y, 0);
  for (int i=0; i<AMBIENT_OCCLUSION_SAMPLES; i++)
  {
    Vec2f sample = RandomSampler_get2D(sampler);
    Sample3f dir = cosineSampleHemisphere(sample.x,sample.y,Nf);

    /* initialize shadow ray */
    Ray shadow;
    shadow.org = Vec3ff(hitPos);
    shadow.dir = Vec3ff(dir.v);
    shadow.tnear() = 0.001f;
    shadow.tfar = inf;
    shadow.geomID = RTC_INVALID_GEOMETRY_ID;
    shadow.primID = RTC_INVALID_GEOMETRY_ID;
    shadow.mask = -1;
    shadow.time() = data.debug;

    /* trace shadow ray */
    RTCOccludedArguments args;
    rtcInitOccludedArguments(&args);
    args.feature_mask = feature_mask;
    rtcTraversableOccluded1(data.traversable,RTCRay_(shadow),&args);
    RayStats_addShadowRay(stats);

    /* add light contribution */
    if (shadow.tfar >= 0.0f)
      intensity += 1.0f;
  }
  intensity *= 1.0f/AMBIENT_OCCLUSION_SAMPLES;

  /* shade pixel */
  return col * intensity;
}

#if defined(EMBREE_SYCL_TUTORIAL) && !defined(EMBREE_SYCL_RT_SIMULATION)
RENDER_FRAME_FUNCTION_SYCL(DebugShader)
RENDER_FRAME_FUNCTION_SYCL(AOShader)
#else
RENDER_FRAME_FUNCTION_CPP(DebugShader)
RENDER_FRAME_FUNCTION_CPP(AOShader)
#endif

} // namespace embree
