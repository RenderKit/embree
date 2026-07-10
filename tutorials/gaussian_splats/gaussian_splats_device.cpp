// Copyright 2009-2021 Intel Corporation
// SPDX-License-Identifier: Apache-2.0

#include "gaussian_splats_device.h"

namespace embree {

#define FEATURE_MASK \
  RTC_FEATURE_FLAG_TRIANGLE | \
  RTC_FEATURE_FLAG_USER_GEOMETRY_CALLBACK_IN_GEOMETRY

RTCScene g_scene = nullptr;
TutorialData data;

void splatBoundsFunc(const RTCBoundsFunctionArguments* args)
{
  const GaussianSplat* splats = (const GaussianSplat*) args->geometryUserPtr;
  const GaussianSplat& s = splats[args->primID];
  RTCBounds* bounds = args->bounds_o;

  const float radiusScale = 3.0f;
  const Vec3fa du = radiusScale * s.sigmaU * s.axisU;
  const Vec3fa dv = radiusScale * s.sigmaV * s.axisV;

  const Vec3fa p0 = s.center + du + dv;
  const Vec3fa p1 = s.center + du - dv;
  const Vec3fa p2 = s.center - du + dv;
  const Vec3fa p3 = s.center - du - dv;

  const Vec3fa lower = min(min(p0, p1), min(p2, p3));
  const Vec3fa upper = max(max(p0, p1), max(p2, p3));

  bounds->lower_x = lower.x;
  bounds->lower_y = lower.y;
  bounds->lower_z = lower.z;
  bounds->upper_x = upper.x;
  bounds->upper_y = upper.y;
  bounds->upper_z = upper.z;
}

RTC_SYCL_INDIRECTLY_CALLABLE void splatIntersectFunc(const RTCIntersectFunctionNArguments* args)
{
  int* valid = args->valid;
  Ray* ray = (Ray*) args->rayhit;
  RTCHit* hit = (RTCHit*) &ray->Ng.x;

  if (args->N != 1 || !valid[0])
    return;

  const GaussianSplat* splats = (const GaussianSplat*) args->geometryUserPtr;
  const GaussianSplat& s = splats[args->primID];

  const Vec3fa n = normalize(cross(s.axisU, s.axisV));
  const float denom = dot(ray->dir, n);
  if (abs(denom) < 1.0e-6f)
    return;

  const float t = dot(s.center - ray->org, n) / denom;
  if (t <= ray->tnear() || t >= ray->tfar)
    return;

  const Vec3fa p = ray->org + t * ray->dir;
  const Vec3fa d = p - s.center;
  const float u = dot(d, s.axisU);
  const float v = dot(d, s.axisV);

  const float invSigmaU2 = rcp(max(1.0e-8f, s.sigmaU * s.sigmaU));
  const float invSigmaV2 = rcp(max(1.0e-8f, s.sigmaV * s.sigmaV));
  const float r2 = u * u * invSigmaU2 + v * v * invSigmaV2;

  if (r2 > 9.0f)
    return;

  const float w = s.opacity * exp(-0.5f * r2);
  if (w < 0.01f)
    return;

  ray->tfar = t;
  hit->geomID = args->geomID;
  hit->primID = args->primID;
  hit->Ng_x = n.x;
  hit->Ng_y = n.y;
  hit->Ng_z = n.z;
  hit->u = w;
  hit->v = 0.0f;
  valid[0] = -1;
}

RTC_SYCL_INDIRECTLY_CALLABLE void splatOccludedFunc(const RTCOccludedFunctionNArguments* args)
{
  int* valid = args->valid;
  Ray* ray = (Ray*) args->ray;

  if (args->N != 1 || !valid[0])
    return;

  const GaussianSplat* splats = (const GaussianSplat*) args->geometryUserPtr;
  const GaussianSplat& s = splats[args->primID];

  const Vec3fa n = normalize(cross(s.axisU, s.axisV));
  const float denom = dot(ray->dir, n);
  if (abs(denom) < 1.0e-6f)
    return;

  const float t = dot(s.center - ray->org, n) / denom;
  if (t <= ray->tnear() || t >= ray->tfar)
    return;

  const Vec3fa p = ray->org + t * ray->dir;
  const Vec3fa d = p - s.center;
  const float u = dot(d, s.axisU);
  const float v = dot(d, s.axisV);

  const float invSigmaU2 = rcp(max(1.0e-8f, s.sigmaU * s.sigmaU));
  const float invSigmaV2 = rcp(max(1.0e-8f, s.sigmaV * s.sigmaV));
  const float r2 = u * u * invSigmaU2 + v * v * invSigmaV2;
  const float w = s.opacity * exp(-0.5f * r2);

  if (r2 <= 9.0f && w >= 0.05f)
    ray->tfar = neg_inf;
}

unsigned int addGroundPlane(RTCScene scene)
{
  RTCGeometry geom = rtcNewGeometry(g_device, RTC_GEOMETRY_TYPE_TRIANGLE);

  Vertex* vertices = (Vertex*) rtcSetNewGeometryBuffer(geom, RTC_BUFFER_TYPE_VERTEX, 0, RTC_FORMAT_FLOAT3, sizeof(Vertex), 4);
  vertices[0].x = -12.0f; vertices[0].y = -2.0f; vertices[0].z = -12.0f;
  vertices[1].x = -12.0f; vertices[1].y = -2.0f; vertices[1].z = +12.0f;
  vertices[2].x = +12.0f; vertices[2].y = -2.0f; vertices[2].z = -12.0f;
  vertices[3].x = +12.0f; vertices[3].y = -2.0f; vertices[3].z = +12.0f;

  Triangle* triangles = (Triangle*) rtcSetNewGeometryBuffer(geom, RTC_BUFFER_TYPE_INDEX, 0, RTC_FORMAT_UINT3, sizeof(Triangle), 2);
  triangles[0].v0 = 0; triangles[0].v1 = 1; triangles[0].v2 = 2;
  triangles[1].v0 = 1; triangles[1].v1 = 3; triangles[1].v2 = 2;

  rtcCommitGeometry(geom);
  unsigned int geomID = rtcAttachGeometry(scene, geom);
  rtcReleaseGeometry(geom);
  return geomID;
}

void addGaussianSplats(RTCScene scene)
{
  RandomSampler rng;
  RandomSampler_init(rng, 1337);

  for (unsigned int i = 0; i < NUM_SPLATS; ++i)
  {
    const float px = 8.0f * RandomSampler_get1D(rng) - 4.0f;
    const float py = 2.5f * RandomSampler_get1D(rng) - 0.5f;
    const float pz = 8.0f * RandomSampler_get1D(rng) - 4.0f;

    Vec3fa axisU(RandomSampler_get1D(rng) * 2.0f - 1.0f,
                 RandomSampler_get1D(rng) * 2.0f - 1.0f,
                 RandomSampler_get1D(rng) * 2.0f - 1.0f);
    axisU = normalize(axisU);

    Vec3fa tangent(RandomSampler_get1D(rng) * 2.0f - 1.0f,
                   RandomSampler_get1D(rng) * 2.0f - 1.0f,
                   RandomSampler_get1D(rng) * 2.0f - 1.0f);
    tangent = normalize(tangent);

    Vec3fa axisV = normalize(cross(axisU, tangent));
    if (dot(axisV, axisV) < 1.0e-6f) {
      axisV = normalize(cross(axisU, Vec3fa(0.0f, 1.0f, 0.0f)));
      if (dot(axisV, axisV) < 1.0e-6f)
        axisV = normalize(cross(axisU, Vec3fa(1.0f, 0.0f, 0.0f)));
    }

    data.splats[i].center = Vec3fa(px, py, pz);
    data.splats[i].axisU = axisU;
    data.splats[i].axisV = axisV;
    data.splats[i].sigmaU = 0.06f + 0.18f * RandomSampler_get1D(rng);
    data.splats[i].sigmaV = 0.06f + 0.18f * RandomSampler_get1D(rng);
    data.splats[i].opacity = 0.25f + 0.75f * RandomSampler_get1D(rng);
    data.splats[i].colorID = i;

    const float cr = 0.2f + 0.8f * RandomSampler_get1D(rng);
    const float cg = 0.2f + 0.8f * RandomSampler_get1D(rng);
    const float cb = 0.2f + 0.8f * RandomSampler_get1D(rng);
    data.colors[i] = Vec3fa(cr, cg, cb);
  }

  RTCGeometry geom = rtcNewGeometry(g_device, RTC_GEOMETRY_TYPE_USER_ORIENTED);
  rtcSetGeometryUserPrimitiveCount(geom, NUM_SPLATS);
  rtcSetGeometryUserData(geom, data.splats);
  rtcSetGeometryOrientedBoundsFunction(geom, splatBoundsFunc, nullptr);
  rtcSetGeometryIntersectFunction(geom, splatIntersectFunc);
  rtcSetGeometryOccludedFunction(geom, splatOccludedFunc);
  rtcCommitGeometry(geom);
  rtcAttachGeometry(scene, geom);
  rtcReleaseGeometry(geom);
}

void renderPixelStandard(const TutorialData& data,
                         int x, int y,
                         int* pixels,
                         const unsigned int width,
                         const unsigned int height,
                         const float time,
                         const ISPCCamera& camera,
                         RayStats& stats)
{
  Ray ray(Vec3fa(camera.xfm.p), Vec3fa(normalize(x * camera.xfm.l.vx + y * camera.xfm.l.vy + camera.xfm.l.vz)), 0.0f, inf);

  RTCIntersectArguments iargs;
  rtcInitIntersectArguments(&iargs);
  iargs.feature_mask = (RTCFeatureFlags)(FEATURE_MASK);
  rtcTraversableIntersect1(data.g_traversable, RTCRayHit_(ray), &iargs);
  RayStats_addRay(stats);

  Vec3fa color(0.02f, 0.03f, 0.05f);
  if (ray.geomID != RTC_INVALID_GEOMETRY_ID)
  {
    if (ray.geomID == 0) {
      const Vec3fa Ng = normalize(ray.Ng);
      const float checker = ((int)floor(0.5f * ray.org.x + ray.tfar * ray.dir.x) ^ (int)floor(0.5f * ray.org.z + ray.tfar * ray.dir.z)) & 1;
      const Vec3fa base = checker ? Vec3fa(0.15f, 0.15f, 0.18f) : Vec3fa(0.35f, 0.35f, 0.4f);
      const Vec3fa L = normalize(Vec3fa(-1.0f, -1.0f, -1.0f));
      const float d = clamp(-dot(L, Ng), 0.0f, 1.0f);
      color = base * (0.2f + 0.8f * d);
    } else {
      const float w = clamp(ray.u, 0.0f, 1.0f);
      const Vec3fa base = data.colors[ray.primID];
      const Vec3fa Ng = normalize(ray.Ng);
      const Vec3fa L = normalize(Vec3fa(-1.0f, -1.0f, -1.0f));
      const float d = clamp(-dot(L, Ng), 0.0f, 1.0f);
      color = base * (0.1f + 0.9f * d) * w;

      Ray shadow(ray.org + ray.tfar * ray.dir, neg(L), 0.001f, inf, 0.0f);
      RTCOccludedArguments sargs;
      rtcInitOccludedArguments(&sargs);
      sargs.feature_mask = (RTCFeatureFlags)(FEATURE_MASK);
      rtcTraversableOccluded1(data.g_traversable, RTCRay_(shadow), &sargs);
      RayStats_addShadowRay(stats);

      if (shadow.tfar < 0.0f)
        color *= 0.55f;
    }
  }

  const unsigned int r = (unsigned int)(255.0f * clamp(color.x, 0.0f, 1.0f));
  const unsigned int g = (unsigned int)(255.0f * clamp(color.y, 0.0f, 1.0f));
  const unsigned int b = (unsigned int)(255.0f * clamp(color.z, 0.0f, 1.0f));
  pixels[y * width + x] = (b << 16) + (g << 8) + r;
}

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
  const unsigned int x1 = min(x0 + TILE_SIZE_X, width);
  const unsigned int y0 = tileY * TILE_SIZE_Y;
  const unsigned int y1 = min(y0 + TILE_SIZE_Y, height);

  for (unsigned int y = y0; y < y1; y++) {
    for (unsigned int x = x0; x < x1; x++)
      renderPixelStandard(data, x, y, pixels, width, height, time, camera, g_stats[threadIndex]);
  }
}

extern "C" void renderFrameStandard(int* pixels,
                                     const unsigned int width,
                                     const unsigned int height,
                                     const float time,
                                     const ISPCCamera& camera)
{
  const int numTilesX = (width + TILE_SIZE_X - 1) / TILE_SIZE_X;
  const int numTilesY = (height + TILE_SIZE_Y - 1) / TILE_SIZE_Y;
  parallel_for(size_t(0), size_t(numTilesX * numTilesY), [&](const range<size_t>& range) {
    const int threadIndex = (int)TaskScheduler::threadIndex();
    for (size_t i = range.begin(); i < range.end(); i++)
      renderTileTask((int)i, threadIndex, pixels, width, height, time, camera, numTilesX, numTilesY);
  });
}

extern "C" void device_init(char* cfg)
{
  TutorialData_Constructor(&data);
  g_scene = data.g_scene = rtcNewScene(g_device);

  addGroundPlane(g_scene);
  addGaussianSplats(g_scene);

  rtcCommitScene(g_scene);
  data.g_traversable = rtcGetSceneTraversable(data.g_scene);
}

extern "C" void device_render(int* pixels,
                               const unsigned int width,
                               const unsigned int height,
                               const float time,
                               const ISPCCamera& camera)
{
  renderFrameStandard(pixels, width, height, time, camera);
}

extern "C" void device_cleanup()
{
  TutorialData_Destructor(&data);
}

} // namespace embree
