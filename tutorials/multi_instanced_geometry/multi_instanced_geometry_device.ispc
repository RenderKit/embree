// Copyright 2009-2021 Intel Corporation
// SPDX-License-Identifier: Apache-2.0

#include "../common/tutorial/tutorial_device.isph"
#include "../common/math/random_sampler.isph"
#include "../common/math/sampling.isph"
#include "scene.isph"

RTCScene g_scene = NULL;
uniform InstanceLevels g_instanceLevels;

/* accumulation buffer */
uniform Vec3fa* uniform g_accu = NULL;
uniform unsigned int g_accu_width = 0;
uniform unsigned int g_accu_height = 0;
uniform unsigned int g_accu_count = 0;
uniform Vec3fa g_accu_vx;
uniform Vec3fa g_accu_vy;
uniform Vec3fa g_accu_vz;
uniform Vec3fa g_accu_p;
extern uniform bool g_changed;

#define STREAM_SIZE (TILE_SIZE_X * TILE_SIZE_Y)

/*
 * There is an issue in ISPC where foreach_tiled can generate
 * empty gangs. This is problematic when scalar operations
 * (e.g. uniform increment) run inside the loop.
 */
#define FOREACH_TILED_MITIGATION if (lanemask() == 0) { continue; }

/* 
 * Accumulate an instance transformation given an instance stack. 
 * We use this only for normal transformations in this example.
 */
LinearSpace3f accumulateNormalTransform(const Ray& ray,
                                        float time)
{
  LinearSpace3f transform = make_LinearSpace3f_identity();
  for (unsigned int level = 0; level < RTC_MAX_INSTANCE_LEVEL_COUNT && ray.instID[level] != RTC_INVALID_GEOMETRY_ID; ++level)
  {
    assert(level < g_instanceLevels.numLevels);
    const unsigned int instId = ray.instID[level];
    assert(instId < g_instanceLevels.numInstancesOnLevel[level]);
    LinearSpace3fa M = g_instanceLevels.normalTransforms[level][instId];
    transform = transform * make_LinearSpace3f(M);
  }
  return transform;
}


/*
 * A simplistic sky model consisting of a directional sun
 * and a constant sky.
 */
void sampleLightDirection(const Vec3fa& xi,
                          Vec3f& dir,
                          Vec3f& emission)
{
  const Vec3f sunDir = normalize(make_Vec3f(-1.f, 1.f, 1.f));
  const Vec3f sunEmission = make_Vec3f(1.f);
  const Vec3f skyEmission = make_Vec3f(.2f);
  const float skyPdf = 1.f/4.f/M_PI;
  const float sunWeight = .1f; // Put most samples into the sky, the sun will converge instantly.

  if (xi.z < sunWeight)
    dir = sunDir;
  else
  {
    // Uniform sphere sampling around +Y axis.
    const float theta = acos(1.f - 2.f * xi.x);
    const float phi = 2.f * M_PI * xi.y;
    const float st = sin(theta);
    dir = make_Vec3f(st * cos(phi), cos(theta), -st * sin(phi));
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
 * Safely invalidate a packet of rays.
 */
inline void invalidateRay(Ray& ray)
{
  // Initialize the whole ray so that it is invalid. This is important because
  // in streamed mode, active state of lanes is forgotten between ray
  // generation and traversal.
  unmasked {
    ray.org = make_Vec3f(0.f);
    ray.dir = make_Vec3f(0.f);
    ray.tnear = pos_inf;
    ray.tfar = neg_inf;
  }
}

/*
 * Pixel filter importance sampling.
 * This uses the Box-Mueller transform to obtain Gaussian samples.
 */
Vec2f sampleGaussianPixelFilter(RandomSampler& sampler)
{
  const float phi = 2.f * M_PI * RandomSampler_get1D(sampler);
  const float threeSigma = 1.f;
  const float sigma = threeSigma / 3.f;

  // Rejection sampling: we don't want any samples outside 3 sigma.
  float radius = (float)inf;
  while (radius <= 0.f || radius > threeSigma)
  {
    const float xi = RandomSampler_get1D(sampler);
    radius = sqrtf(sigma * (-2.f) * log(xi));
  }

  return make_Vec2f(radius * cos(phi), radius * sin(phi));
}

/*
 * Sample a primary ray.
 */
Ray samplePrimaryRay(unsigned int x,
                     unsigned int x0,
                     unsigned int y,
                     unsigned int y0,
                     const uniform ISPCCamera& camera,
                     RandomSampler& sampler,
                     uniform RayStats& stats)
{
  RandomSampler_init(sampler, (int)x, (int)y, g_accu_count);

  const unsigned int id = (y-y0) * TILE_SIZE_X + (x-x0);
  const Vec2f offset = sampleGaussianPixelFilter(sampler);
  const float fx = (float)x + 0.5f + offset.x;
  const float fy = (float)y + 0.5f + offset.y;
  const Vec3f o = make_Vec3f(camera.xfm.p);
  const Vec3f w = make_Vec3f(normalize(fx*camera.xfm.l.vx 
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
                         const Vec3f& lightDir,
                         uniform RayStats& stats)
{
  const Vec3f o = primary.org + primary.tfar * primary.dir;

  Ray ray;
  init_Ray(ray, o, lightDir, 0.001f, (float)inf);
  ray.id = -1;
  RayStats_addShadowRay(stats);
  return ray;
} 

/*
 * Our shader for this scene: Lambertian shading with normal display.
 */
Vec3f shader(const Ray& primaryRay, 
             const Ray& shadowRay,
             const Vec3f& lightDir,
             const Vec3f& emission)
{
  if (primaryRay.geomID == RTC_INVALID_GEOMETRY_ID || shadowRay.tfar < 0.f)
    return make_Vec3f(0.f);

  const LinearSpace3f xfm = accumulateNormalTransform(primaryRay, 0.f);
  Vec3f Ns = normalize(xfmVector(xfm, make_Vec3f(primaryRay.Ng)));

  const float cosThetaOut = dot(Ns, lightDir);
  const float cosThetaIn = -dot(Ns, primaryRay.dir);

  // Block transmission.
  if ((cosThetaOut >= 0) != (cosThetaIn >= 0))
    return make_Vec3f(0.f);

  // Make sure backfaces shade correctly.
  if (cosThetaIn < 0.f)
    Ns = -1.f * Ns;

  return emission
       * clamp(abs(cosThetaOut), 0.f, 1.f) 
       * (make_Vec3f(0.5f) + 0.5f * Ns);
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
inline unsigned int packRGB8(const Vec3f& color)
{
  const unsigned int r = floatToByte(color.x);
  const unsigned int g = floatToByte(color.y);
  const unsigned int b = floatToByte(color.z);
  return (b << 16) + (g << 8) + r;
}

/*
 * Splat a color into the framebuffer.
 */
void splat(uniform int* uniform pixels,
           const uniform unsigned int width,
           unsigned int x,
           unsigned int y,
           const Vec3f& color)
{
  const unsigned int pixIdx = y * width + x;
  const Vec3fa accu_color = g_accu[pixIdx] + make_Vec3fa(color.x,color.y,color.z,1.0f); 
  g_accu[pixIdx] = accu_color;
  if (accu_color.w > 0)
  {
    float f = rcp(accu_color.w);
    pixels[pixIdx] = packRGB8(make_Vec3f(accu_color * f));
  }
}

/* 
 * Renders a single screen tile.
 */
void renderTileStream(uniform int* uniform pixels,
                      const uniform unsigned int width,
                      const uniform float time,
                      const uniform ISPCCamera& camera,
                      const uniform unsigned int x0,
                      const uniform unsigned int x1,
                      const uniform unsigned int y0,
                      const uniform unsigned int y1,
                      uniform IntersectContext& primaryContext,
                      uniform IntersectContext& shadowContext,
                      uniform RayStats& stats)
{
  RandomSampler sampler;
  Ray primary[STREAM_SIZE];
  Ray shadow[STREAM_SIZE];
  Vec3f lightDir[STREAM_SIZE];
  Vec3f emission[STREAM_SIZE];

  uniform unsigned int numPackets = 0;
  foreach_tiled(y = y0 ... y1, x = x0 ... x1)
  {
    FOREACH_TILED_MITIGATION

    primary[numPackets] = samplePrimaryRay(x, x0, y, y0, camera, sampler, stats);

    ++numPackets;
  }

  rtcIntersectVM(g_scene, 
                 &primaryContext.context,
                 (varying RTCRayHit* uniform)&primary,
                 numPackets,
                 sizeof(Ray));

  numPackets = 0;
  foreach_tiled(y = y0 ... y1, x = x0 ... x1)
  {
    FOREACH_TILED_MITIGATION

    // This is only needed for streaming mode, as we keep invalid
    // rays in flight. This call will invalidate instances that 
    // are not currently active.
    invalidateRay(shadow[numPackets]);

    if (primary[numPackets].geomID != RTC_INVALID_GEOMETRY_ID)
    {
      sampleLightDirection(RandomSampler_get3D(sampler),
                           lightDir[numPackets],
                           emission[numPackets]);
      shadow[numPackets] = makeShadowRay(primary[numPackets], lightDir[numPackets], stats);
    }

    ++numPackets;
  }

  rtcOccludedVM(g_scene, 
                &shadowContext.context, 
                (varying RTCRay* uniform)&shadow,
                numPackets, 
                sizeof(Ray));

  numPackets = 0;
  foreach_tiled(y = y0 ... y1, x = x0 ... x1)
  {
    FOREACH_TILED_MITIGATION

    const Vec3f color = shader(primary[numPackets],
                               shadow[numPackets],
                               lightDir[numPackets],
                               emission[numPackets]);

    splat(pixels, width, x, y, color);

    ++numPackets;
  }
}

/*
 * Render a single tile without streams.
 */
void renderTileNormal(uniform int* uniform pixels,
                      const uniform unsigned int width,
                      const uniform float time,
                      const uniform ISPCCamera& camera,
                      const uniform unsigned int x0,
                      const uniform unsigned int x1,
                      const uniform unsigned int y0,
                      const uniform unsigned int y1,
                      uniform IntersectContext& primaryContext,
                      uniform IntersectContext& shadowContext,
                      uniform RayStats& stats)
{
  RandomSampler sampler;

  foreach_tiled(y = y0 ... y1, x = x0 ... x1)
  {
    FOREACH_TILED_MITIGATION

    Ray primaryRay = samplePrimaryRay(x, x0, y, y0, camera, sampler, stats);
    rtcIntersectV(g_scene, &primaryContext.context, RTCRayHit_(primaryRay));

    Vec3f color = make_Vec3f(0.f);
    if (primaryRay.geomID != RTC_INVALID_GEOMETRY_ID)
    {
      Vec3f lightDir;
      Vec3f emission;
      sampleLightDirection(RandomSampler_get3D(sampler), lightDir, emission);
      Ray shadowRay = makeShadowRay(primaryRay, lightDir, stats);
      rtcOccludedV(g_scene, &shadowContext.context, RTCRay_(shadowRay));
      color = shader(primaryRay, shadowRay, lightDir, emission);
    }

    splat(pixels, width, x, y, color);
  }
}

// ======================================================================== //
// TUTORIAL API.
// ======================================================================== //

/*
 * A task that renders a single screen tile.
 */
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
  const uniform unsigned int x1 = min(x0 + TILE_SIZE_X, width);
  const uniform unsigned int y0 = tileY * TILE_SIZE_Y;
  const uniform unsigned int y1 = min(y0 + TILE_SIZE_Y, height);

  uniform IntersectContext primaryContext;
  uniform IntersectContext shadowContext;

  InitIntersectionContext(&primaryContext);
  InitIntersectionContext(&shadowContext);

  // Primary rays in this scene are coherent. This is not the case
  // for shadow rays, since there is a spherical environment light.
  primaryContext.context.flags = g_iflags_coherent; 

  if (g_mode == MODE_NORMAL) 
    renderTileNormal(pixels, width,
                     time, camera,
                     x0, x1, y0, y1,
                     primaryContext,
                     shadowContext,
                     g_stats[threadIndex]);
  else                       
    renderTileStream(pixels, width,
                     time, camera,
                     x0, x1, y0, y1,
                     primaryContext,
                     shadowContext,
                     g_stats[threadIndex]);
}

/* 
 * Called by the C++ code for initialization. 
 */
export void device_init(uniform int8* uniform cfg)
{
  g_scene = initializeScene(g_device, &g_instanceLevels);
}


export void renderFrameStandard(uniform int* uniform pixels,
                                const uniform unsigned int width,
                                const uniform unsigned int height,
                                const uniform float time,
                                const uniform ISPCCamera& camera)
{
  const uniform int numTilesX = (width +TILE_SIZE_X-1)/TILE_SIZE_X;
  const uniform int numTilesY = (height+TILE_SIZE_Y-1)/TILE_SIZE_Y;
  launch[numTilesX*numTilesY] renderTileTask(pixels,width,height,time,camera,numTilesX,numTilesY); sync;
}

/* 
 * Called by the C++ code to render. 
 */
export void device_render(uniform int* uniform pixels,
                          const uniform unsigned int width,
                          const uniform unsigned int height,
                          const uniform float time,
                          const uniform ISPCCamera& camera)
{
  if (g_accu_width != width || g_accu_height != height) {
    delete[] g_accu;
    g_accu = uniform new uniform Vec3fa[width*height];
    g_accu_width = width;
    g_accu_height = height;
    for (uniform unsigned int i=0; i<width*height; i++)
      g_accu[i] = make_Vec3fa(0.0f);
  }

  uniform bool camera_changed = g_changed; 
  g_changed = false;
  camera_changed |= ne(g_accu_vx,camera.xfm.l.vx); g_accu_vx = camera.xfm.l.vx;
  camera_changed |= ne(g_accu_vy,camera.xfm.l.vy); g_accu_vy = camera.xfm.l.vy;
  camera_changed |= ne(g_accu_vz,camera.xfm.l.vz); g_accu_vz = camera.xfm.l.vz;
  camera_changed |= ne(g_accu_p, camera.xfm.p);    g_accu_p  = camera.xfm.p;

  if (camera_changed)
  {
    g_accu_count=0;
    for (uniform unsigned int i=0; i<width*height; i++)
      g_accu[i] = make_Vec3fa(0.0f);
  }
  else
    g_accu_count++;
}

/*
 * Called by the C++ code for cleanup.
 */
export void device_cleanup ()
{
  rtcReleaseScene(g_scene);
  g_scene = NULL;
  cleanupScene();
  delete[] g_accu; 
  g_accu = NULL;
  g_accu_width = 0;
  g_accu_height = 0;
  g_accu_count = 0;
}

/*
 * This must be here for the linker to find, but we will not use it.
 */
void renderTileStandard(uniform int taskIndex,
                        uniform int threadIndex,
                        uniform int* uniform pixels,
                        const uniform unsigned int width,
                        const uniform unsigned int height,
                        const uniform float time,
                        const uniform ISPCCamera& camera,
                        const uniform int numTilesX,
                        const uniform int numTilesY) { }
