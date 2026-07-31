// Copyright 2009-2021 Intel Corporation
// SPDX-License-Identifier: Apache-2.0

#include "multi_instanced_geometry_device.h"

namespace embree {

/* all features required by this tutorial */
#define FEATURE_MASK \
  RTC_FEATURE_FLAG_TRIANGLE | \
  RTC_FEATURE_FLAG_INSTANCE

RTCScene g_scene = nullptr;
TutorialData g_data;
extern "C" bool g_changed;

/*
 * There is an issue in ISPC where foreach_tiled can generate
 * empty gangs. This is problematic when scalar operations
 * (e.g. increment) run inside the loop.
 */
#define FOREACH_TILED_MITIGATION if (1 == 0) { continue; }

/* 
 * Accumulate an instance transformation given an instance stack. 
 * We use this only for normal transformations in this example.
 */
LinearSpace3fa accumulateNormalTransform(const TutorialData& data,
                                        const Ray& ray,
                                        float time)
{
  LinearSpace3fa transform = LinearSpace3fa(one);
  for (unsigned int level = 0; level < RTC_MAX_INSTANCE_LEVEL_COUNT && ray.instID[level] != RTC_INVALID_GEOMETRY_ID; ++level)
  {
    assert(level < data.g_instanceLevels.numLevels);
    const unsigned int instId = ray.instID[level];
    assert(instId < data.g_instanceLevels.numInstancesOnLevel[level]);
    LinearSpace3fa M = data.g_instanceLevels.normalTransforms[level][instId];
    transform = transform * LinearSpace3fa(M);
  }
  return transform;
}


/*
 * A simplistic sky model consisting of a directional sun
 * and a constant sky.
 */
void sampleLightDirection(const Vec3fa& xi,
                          Vec3fa& dir,
                          Vec3fa& emission)
{
  const Vec3fa sunDir = normalize(Vec3fa(-1.f, 1.f, 1.f));
  const Vec3fa sunEmission = Vec3fa(1.f);
  const Vec3fa skyEmission = Vec3fa(.2f);
  const float skyPdf = 1.f/4.f/float(M_PI);
  const float sunWeight = .1f; // Put most samples into the sky, the sun will converge instantly.

  if (xi.z < sunWeight)
    dir = sunDir;
  else
  {
    // Uniform sphere sampling around +Y axis.
    const float theta = acos(1.f - 2.f * xi.x);
    const float phi = 2.f * float(M_PI) * xi.y;
    const float st = sin(theta);
    dir = Vec3fa(st * cos(phi), cos(theta), -st * sin(phi));
  }

  emission = skyEmission;
  float pdf = (1.f-sunWeight) * skyPdf;
  if (sunDir.x == dir.x && sunDir.y == dir.y && sunDir.z == dir.z)
  {
    emission = emission + sunEmission;
    pdf = pdf + sunWeight;
  }

  emission = emission * rcp(pdf);
}

/*
 * Pixel filter importance sampling.
 * This uses the Box-Mueller transform to obtain Gaussian samples.
 */
Vec2f sampleGaussianPixelFilter(RandomSampler& sampler)
{
  const float phi = 2.f * float(M_PI) * RandomSampler_get1D(sampler);
  const float threeSigma = 1.f;
  const float sigma = threeSigma / 3.f;

  // Rejection sampling: we don't want any samples outside 3 sigma.
  float radius = (float)inf;
  while (radius <= 0.f || radius > threeSigma)
  {
    const float xi = RandomSampler_get1D(sampler);
    radius = sqrtf(sigma * (-2.f) * log(xi));
  }

  return Vec2f(radius * cos(phi), radius * sin(phi));
}

/*
 * Sample a primary ray.
 */
Ray samplePrimaryRay(const TutorialData& data,
                     unsigned int x,
                     unsigned int x0,
                     unsigned int y,
                     unsigned int y0,
                     const ISPCCamera& camera,
                     RandomSampler& sampler,
                     RayStats& stats)
{
  RandomSampler_init(sampler, (int)x, (int)y, data.g_accu_count);

  const unsigned int id = (y-y0) * TILE_SIZE_X + (x-x0);
  const Vec2f offset = sampleGaussianPixelFilter(sampler);
  const float fx = (float)x + 0.5f + offset.x;
  const float fy = (float)y + 0.5f + offset.y;
  const Vec3fa o = Vec3fa(camera.xfm.p);
  const Vec3fa w = Vec3fa(normalize(fx*camera.xfm.l.vx 
                                     + fy*camera.xfm.l.vy 
                                     + camera.xfm.l.vz));

  Ray ray;
  init_Ray(ray, o, w, 0.f, (float)inf);
  ray.id = id;
  RayStats_addRay(stats);
  return ray;
}

/* 
 * Make a new shadow ray.
 */
inline Ray makeShadowRay(const Ray& primary,
                         const Vec3fa& lightDir,
                         RayStats& stats)
{
  const Vec3fa o = primary.org + primary.tfar * primary.dir;

  Ray ray;
  init_Ray(ray, o, lightDir, 0.001f, (float)inf);
  ray.id = -1;
  RayStats_addShadowRay(stats);
  return ray;
} 

/*
 * Our shader for this scene: Lambertian shading with normal display.
 */
Vec3fa shade(const TutorialData& data,
             const Ray& primaryRay, 
             const Ray& shadowRay,
             const Vec3fa& lightDir,
             const Vec3fa& emission)
{
  if (primaryRay.geomID == RTC_INVALID_GEOMETRY_ID || shadowRay.tfar < 0.f)
    return Vec3fa(0.f);

  const LinearSpace3fa xfm = accumulateNormalTransform(data,primaryRay, 0.f);
  Vec3fa Ns = normalize(xfmVector(xfm, Vec3fa(primaryRay.Ng)));

  const float cosThetaOut = dot(Ns, lightDir);
  const float cosThetaIn = -dot(Ns, primaryRay.dir);

  // Block transmission.
  if ((cosThetaOut >= 0) != (cosThetaIn >= 0))
    return Vec3fa(0.f);

  // Make sure backfaces shade correctly.
  if (cosThetaIn < 0.f)
    Ns = -1.f * Ns;

  return emission
       * clamp(abs(cosThetaOut), 0.f, 1.f) 
       * (Vec3fa(0.5f) + 0.5f * Ns);
}

/*
 * Convert a floating-point value in [0, 1] to 8 bit.
 */
inline unsigned int floatToByte(float channel)
{
  channel = 255.1f * clamp(channel, 0.f, 1.f);
  return 0xff & (unsigned int) channel;
}

/*
 * Pack an RGB8 color value from three floats.
 */
inline unsigned int packRGB8(const Vec3fa& color)
{
  const unsigned int r = floatToByte(color.x);
  const unsigned int g = floatToByte(color.y);
  const unsigned int b = floatToByte(color.z);
  return (b << 16) + (g << 8) + r;
}

/*
 * Splat a color into the framebuffer.
 */
void splat(const TutorialData& data,
           int* pixels,
           const unsigned int width,
           unsigned int x,
           unsigned int y,
           const Vec3fa& color)
{
  const unsigned int pixIdx = y * width + x;
  const Vec3ff accu_color = data.g_accu[pixIdx] + Vec3ff(color.x,color.y,color.z,1.0f); 
  data.g_accu[pixIdx] = accu_color;
  if (accu_color.w > 0)
  {
    float f = rcp(accu_color.w);
    pixels[pixIdx] = packRGB8(Vec3fa(accu_color * f));
  }
}

void renderPixelStandard(const TutorialData& data, int x, int y,
                         int* pixels,
                         const unsigned int width,
                         const unsigned int height,
                         const float time,
                         const ISPCCamera& camera, RayStats& stats)
{
  RTCIntersectArguments iargs;
  rtcInitIntersectArguments(&iargs);
  iargs.feature_mask = (RTCFeatureFlags) (FEATURE_MASK);

  RTCOccludedArguments sargs;
  rtcInitOccludedArguments(&sargs);
  sargs.feature_mask = (RTCFeatureFlags) (FEATURE_MASK);

  RandomSampler sampler;
  Ray primaryRay = samplePrimaryRay(data, x, 0, y, 0, camera, sampler, stats);
  rtcTraversableIntersect1(data.g_traversable, RTCRayHit_(primaryRay), &iargs);
  
  Vec3fa color = Vec3fa(0.f);
  if (primaryRay.geomID != RTC_INVALID_GEOMETRY_ID)
  {
    Vec3fa lightDir;
    Vec3fa emission;
    sampleLightDirection(RandomSampler_get3D(sampler), lightDir, emission);
    Ray shadowRay = makeShadowRay(primaryRay, lightDir, stats);
    rtcTraversableOccluded1(data.g_traversable, RTCRay_(shadowRay), &sargs);
    color = shade(data, primaryRay, shadowRay, lightDir, emission);
  }
  
  splat(data, pixels, width, x, y, color);
}

/*
 * Render a single tile.
 */
void renderTileNormal(int* pixels,
                      const unsigned int width,
                      const unsigned int height,
                      const float time,
                      const ISPCCamera& camera,
                      const unsigned int x0,
                      const unsigned int x1,
                      const unsigned int y0,
                      const unsigned int y1,
                      RayStats& stats)
{
  for (unsigned int y=y0; y<y1; y++) for (unsigned int x=x0; x<x1; x++)
  {
    FOREACH_TILED_MITIGATION;
    renderPixelStandard(g_data,x,y,pixels,width,height,time,camera,stats);
  }
}

// ======================================================================== //
// TUTORIAL API.
// ======================================================================== //

/*
 * A task that renders a single screen tile.
 */
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
  const unsigned int x1 = min(x0 + TILE_SIZE_X, width);
  const unsigned int y0 = tileY * TILE_SIZE_Y;
  const unsigned int y1 = min(y0 + TILE_SIZE_Y, height);

  renderTileNormal(pixels, width, height,
                   time, camera,
                   x0, x1, y0, y1,
                   g_stats[threadIndex]);
}

/* 
 * Called by the C++ code for initialization. 
 */
extern "C" void device_init(char* cfg)
{
  TutorialData_Constructor(&g_data);
  g_scene = g_data.g_scene = initializeScene(g_data, g_device);
  g_data.g_traversable = rtcGetSceneTraversable(g_scene);
}


extern "C" void renderFrameStandard(int* pixels,
                                const unsigned int width,
                                const unsigned int height,
                                const float time,
                                const ISPCCamera& camera)
{
#if defined(EMBREE_SYCL_TUTORIAL) && !defined(EMBREE_SYCL_RT_SIMULATION)
  TutorialData ldata = g_data;
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

/* 
 * Called by the C++ code to render. 
 */
extern "C" void device_render(int* pixels,
                          const unsigned int width,
                          const unsigned int height,
                          const float time,
                          const ISPCCamera& camera)
{
  if (g_data.g_accu_width != width || g_data.g_accu_height != height) {
    alignedUSMFree(g_data.g_accu);
    g_data.g_accu = (Vec3ff*) alignedUSMMalloc((width*height)*sizeof(Vec3ff),16,EmbreeUSMMode::DEVICE_READ_WRITE);
    g_data.g_accu_width = width;
    g_data.g_accu_height = height;
    for (unsigned int i=0; i<width*height; i++)
      g_data.g_accu[i] = Vec3ff(0.0f);
  }
  
  bool camera_changed = g_changed; 
  g_changed = false;
  camera_changed |= ne(g_data.g_accu_vx,camera.xfm.l.vx); g_data.g_accu_vx = camera.xfm.l.vx;
  camera_changed |= ne(g_data.g_accu_vy,camera.xfm.l.vy); g_data.g_accu_vy = camera.xfm.l.vy;
  camera_changed |= ne(g_data.g_accu_vz,camera.xfm.l.vz); g_data.g_accu_vz = camera.xfm.l.vz;
  camera_changed |= ne(g_data.g_accu_p, camera.xfm.p);    g_data.g_accu_p  = camera.xfm.p;

  if (camera_changed)
  {
    g_data.g_accu_count=0;
    for (unsigned int i=0; i<width*height; i++)
      g_data.g_accu[i] = Vec3ff(0.0f);
  }
  else
    g_data.g_accu_count++;
}

/*
 * Called by the C++ code for cleanup.
 */
extern "C" void device_cleanup ()
{
  TutorialData_Destructor(&g_data);
}

/*
 * This must be here for the linker to find, but we will not use it.
 */
void renderTileStandard(int taskIndex,
                        int threadIndex,
                        int* pixels,
                        const unsigned int width,
                        const unsigned int height,
                        const float time,
                        const ISPCCamera& camera,
                        const int numTilesX,
                        const int numTilesY) { }

} // namespace embree
