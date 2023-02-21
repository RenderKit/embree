// Copyright 2009-2021 Intel Corporation
// SPDX-License-Identifier: Apache-2.0

#include "quaternion_motion_blur_device.h"
#include "../common/math/random_sampler.h"
#include "../common/math/sampling.h"

namespace embree {

#define USE_ARGUMENT_CALLBACKS 1

/* all features required by this tutorial */
#if USE_ARGUMENT_CALLBACKS
#define FEATURE_MASK \
  RTC_FEATURE_FLAG_TRIANGLE | \
  RTC_FEATURE_FLAG_INSTANCE | \
  RTC_FEATURE_FLAG_USER_GEOMETRY | \
  RTC_FEATURE_FLAG_MOTION_BLUR | \
  RTC_FEATURE_FLAG_USER_GEOMETRY_CALLBACK_IN_ARGUMENTS
#else
  #define FEATURE_MASK \
  RTC_FEATURE_FLAG_TRIANGLE | \
  RTC_FEATURE_FLAG_INSTANCE | \
  RTC_FEATURE_FLAG_USER_GEOMETRY | \
  RTC_FEATURE_FLAG_MOTION_BLUR | \
  RTC_FEATURE_FLAG_USER_GEOMETRY_CALLBACK_IN_GEOMETRY
#endif

/* scene data */
RTCScene g_scene = nullptr;
TutorialData data;

RTCGeometry g_instance_linear_0 = nullptr;
RTCGeometry g_instance_linear_1 = nullptr;
RTCGeometry g_instance_quaternion_0 = nullptr;
RTCGeometry g_instance_quaternion_1 = nullptr;
RTCQuaternionDecomposition qdc[10];

extern "C" bool g_changed;
extern "C" float g_time;
extern "C" int g_spp;
extern "C" int g_numTimeSteps;
extern "C" float g_shutter_close;
extern "C" bool g_animate;
extern "C" bool g_motion_blur;
extern "C" bool g_reset;

RTCIntersectFunctionN sphereIntersectFuncPtr = nullptr;

AffineSpace3fa fromQuaternionDecomposition(const RTCQuaternionDecomposition& qdc)
{
  AffineSpace3fa T = AffineSpace3fa::scale(Vec3fa(1.f, 1.f, 1.f));
  T.p = Vec3fa(qdc.translation_x, qdc.translation_y, qdc.translation_z);

  AffineSpace3fa S = AffineSpace3fa::scale(Vec3fa(1.f, 1.f, 1.f));
  S.l.vx.x = qdc.scale_x; S.l.vy.x = qdc.skew_xy; S.l.vz.x = qdc.skew_xz; S.p.x = qdc.shift_x;
                          S.l.vy.y = qdc.scale_y; S.l.vz.y = qdc.skew_yz; S.p.y = qdc.shift_y;
                                                  S.l.vz.z = qdc.scale_z; S.p.z = qdc.shift_z;

  Quaternion3f q = Quaternion3f(Vec4f(
    qdc.quaternion_r, qdc.quaternion_i, qdc.quaternion_j, qdc.quaternion_k));

  AffineSpace3fa R = AffineSpace3fa(LinearSpace3fa(q));
  return T * R * S;
}

void updateTransformation()
{
  // transformation matrizes for instance 0 (rotation around axis through sphere center)
  rtcSetGeometryTimeStepCount(g_instance_linear_0, g_numTimeSteps);
  rtcSetGeometryTimeStepCount(g_instance_quaternion_0, g_numTimeSteps);
  for (int i = 0; i < g_numTimeSteps; ++i)
  {
    // scale/skew, rotation, transformation data for quaternion motion blur
    float K = g_numTimeSteps > 0 ? ((float)i)/(g_numTimeSteps-1) : 0.f;
    float R = K * 2.0f * float(M_PI);
    if (g_numTimeSteps == 3) R = K * (2.0f - 1e-6f) * float(M_PI);

    Quaternion3f q = Quaternion3f::rotate(Vec3fa(0.f, 1.f, 0.f), R);
    rtcInitQuaternionDecomposition(qdc+i);
    rtcQuaternionDecompositionSetQuaternion(qdc+i, q.r, q.i, q.j, q.k);
    rtcQuaternionDecompositionSetScale(qdc+i, 3.f, 3.f, 3.f);
    rtcQuaternionDecompositionSetTranslation(qdc+i, -5.5f, 0.f, -5.5f);
    rtcSetGeometryTransformQuaternion(g_instance_quaternion_0, i, qdc+i);

    rtcQuaternionDecompositionSetTranslation(qdc+i, -5.5f, 0.f, 5.5f);
    AffineSpace3fa xfm = fromQuaternionDecomposition(qdc[i]);
    rtcSetGeometryTransform(g_instance_linear_0, i, RTC_FORMAT_FLOAT4X4_COLUMN_MAJOR,(float*)&(xfm.l.vx.x));
  }

  // transformation matrizes for instance 1 (translation and rotation around origin)
  rtcSetGeometryTimeStepCount(g_instance_linear_1, g_numTimeSteps);
  rtcSetGeometryTimeStepCount(g_instance_quaternion_1, g_numTimeSteps);
  for (int i = 0; i < g_numTimeSteps; ++i)
  {
    // scale/skew, rotation, transformation data for quaternion motion blur
    float K = g_numTimeSteps > 0 ? ((float)i)/(g_numTimeSteps-1) : 0.f;
    float R = K * 2.0f * float(M_PI);
    if (g_numTimeSteps == 3) R = K * (2.0f - 1e-6f) * float(M_PI);

    Quaternion3f q = Quaternion3f::rotate(Vec3fa(0.f, 1.f, 0.f), R);
    rtcInitQuaternionDecomposition(qdc+i);
    rtcQuaternionDecompositionSetQuaternion(qdc+i, q.r, q.i, q.j, q.k);
    rtcQuaternionDecompositionSetShift(qdc+i, 3.f, 0.f, 3.f);
    rtcQuaternionDecompositionSetTranslation(qdc+i, 5.5f, 0.f, -5.5f);
    rtcSetGeometryTransformQuaternion(g_instance_quaternion_1, i, qdc+i);

    rtcQuaternionDecompositionSetTranslation(qdc+i, 5.5f, 0.f, 5.5f);
    AffineSpace3fa xfm = fromQuaternionDecomposition(qdc[i]);
    rtcSetGeometryTransform(g_instance_linear_1, i, RTC_FORMAT_FLOAT4X4_COLUMN_MAJOR,(float*)&(xfm.l.vx.x));
  }

  rtcCommitGeometry(g_instance_linear_0);
  rtcCommitGeometry(g_instance_linear_1);
  rtcCommitGeometry(g_instance_quaternion_0);
  rtcCommitGeometry(g_instance_quaternion_1);
}

// ======================================================================== //
//                     User defined sphere geometry                         //
// ======================================================================== //

void sphereBoundsFunc(const struct RTCBoundsFunctionArguments* args)
{
  const Sphere* spheres = (const Sphere*) args->geometryUserPtr;
  RTCBounds* bounds_o = args->bounds_o;
  const Sphere& sphere = spheres[args->primID];
  bounds_o->lower_x = sphere.p.x-sphere.r;
  bounds_o->lower_y = sphere.p.y-sphere.r;
  bounds_o->lower_z = sphere.p.z-sphere.r;
  bounds_o->upper_x = sphere.p.x+sphere.r;
  bounds_o->upper_y = sphere.p.y+sphere.r;
  bounds_o->upper_z = sphere.p.z+sphere.r;
}

RTC_SYCL_INDIRECTLY_CALLABLE void sphereIntersectFunc(const RTCIntersectFunctionNArguments* args)
{
  int* valid = args->valid;
  void* ptr  = args->geometryUserPtr;
  Ray *ray = (Ray*)args->rayhit;
  RTCHit* hit = (RTCHit *)&ray->Ng.x;
  unsigned int primID = args->primID;

  if (args->N != 1)
    return;
  const Sphere* spheres = (const Sphere*)ptr;
  const Sphere& sphere = spheres[primID];

  if (!valid[0]) return;
  valid[0] = 0;

  const Vec3fa v = ray->org-sphere.p;
  const float A = dot(ray->dir,ray->dir);
  const float B = 2.0f*dot(v,ray->dir);
  const float C = dot(v,v) - sqr(sphere.r);
  const float D = B*B - 4.0f*A*C;
  if (D < 0.0f) return;
  const float Q = sqrt(D);
  const float rcpA = rcp(A);
  const float t0 = 0.5f*rcpA*(-B-Q);
  const float t1 = 0.5f*rcpA*(-B+Q);

  RTCHit potentialHit;
  potentialHit.u = 0.0f;
  potentialHit.v = 0.0f;
  potentialHit.geomID = sphere.geomID;
  potentialHit.primID = primID;
  if ((ray->tnear() < t0) & (t0 < ray->tfar))
  {
    const Vec3fa Ng = ray->org+t0*ray->dir-sphere.p;
    potentialHit.Ng_x = Ng.x;
    potentialHit.Ng_y = Ng.y;
    potentialHit.Ng_z = Ng.z;
    ray->tfar = t0;
    *hit = potentialHit;
    valid[0] = -1;
  }

  if ((ray->tnear() < t1) & (t1 < ray->tfar))
  {
    const Vec3fa Ng = ray->org+t1*ray->dir-sphere.p;
    potentialHit.Ng_x = Ng.x;
    potentialHit.Ng_y = Ng.y;
    potentialHit.Ng_z = Ng.z;
    ray->tfar = t1;
    *hit = potentialHit;
    valid[0] = -1;
  }
}

Sphere* createAnalyticalSpheres (RTCScene scene, unsigned int N)
{
  RTCGeometry geom = rtcNewGeometry(g_device, RTC_GEOMETRY_TYPE_USER);
  Sphere* spheres = (Sphere*) alignedUSMMalloc((N)*sizeof(Sphere),16);
  unsigned int geomID = rtcAttachGeometry(scene,geom);
  for (unsigned int i=0; i<N; i++) {
    spheres[i].geometry = geom;
    spheres[i].geomID = geomID;
  }
  rtcSetGeometryUserPrimitiveCount(geom,N);
  rtcSetGeometryUserData(geom,spheres);
  rtcSetGeometryBoundsFunction(geom,sphereBoundsFunc,nullptr);
#if !USE_ARGUMENT_CALLBACKS
  rtcSetGeometryIntersectFunction(geom,sphereIntersectFuncPtr);
#endif
  rtcCommitGeometry(geom);
  rtcReleaseGeometry(geom);
  return spheres;
}

/* called by the C++ code for initialization */
extern "C" void device_init (char* cfg)
{
  sphereIntersectFuncPtr = GET_FUNCTION_POINTER(sphereIntersectFunc);
  
  /* create scene */
  TutorialData_Constructor(&data);
  g_scene = data.g_scene = rtcNewScene(g_device);

  /* create scene with 4 analytical spheres */
  data.g_scene0 = rtcNewScene(g_device);
  data.g_spheres = createAnalyticalSpheres(data.g_scene0, 1);
  data.g_spheres[0].p = Vec3fa(0, 0, 0);
  data.g_spheres[0].r = 1.0f;
  rtcCommitScene(data.g_scene0);

  // attach multiple times otherwise Embree will optimize and not use
  // internal instancing (magic!)
  g_instance_linear_0 = rtcNewGeometry(g_device, RTC_GEOMETRY_TYPE_INSTANCE);
  g_instance_linear_1 = rtcNewGeometry(g_device, RTC_GEOMETRY_TYPE_INSTANCE);
  g_instance_quaternion_0 = rtcNewGeometry(g_device, RTC_GEOMETRY_TYPE_INSTANCE);
  g_instance_quaternion_1 = rtcNewGeometry(g_device, RTC_GEOMETRY_TYPE_INSTANCE);
  rtcSetGeometryInstancedScene(g_instance_linear_0, data.g_scene0);
  rtcSetGeometryInstancedScene(g_instance_linear_1, data.g_scene0);
  rtcSetGeometryInstancedScene(g_instance_quaternion_0, data.g_scene0);
  rtcSetGeometryInstancedScene(g_instance_quaternion_1, data.g_scene0);
  updateTransformation();
  for (int i = 0; i < 2; ++i)
  {
    rtcAttachGeometry(data.g_scene, g_instance_linear_0);
    rtcAttachGeometry(data.g_scene, g_instance_linear_1);
    rtcAttachGeometry(data.g_scene, g_instance_quaternion_0);
    rtcAttachGeometry(data.g_scene, g_instance_quaternion_1);
  }
  rtcReleaseGeometry(g_instance_linear_0);
  rtcReleaseGeometry(g_instance_linear_1);
  rtcReleaseGeometry(g_instance_quaternion_0);
  rtcReleaseGeometry(g_instance_quaternion_1);
  rtcCommitGeometry(g_instance_linear_0);
  rtcCommitGeometry(g_instance_linear_1);
  rtcCommitGeometry(g_instance_quaternion_0);
  rtcCommitGeometry(g_instance_quaternion_1);

  rtcCommitScene (data.g_scene);
}

inline Vec3fa face_forward(const Vec3fa& dir, const Vec3fa& _Ng) {
  const Vec3fa Ng = _Ng;
  return dot(dir,Ng) < 0.0f ? Ng : neg(Ng);
}

Vec3fa renderPixelFunction(const TutorialData& data,
                          float x, float y,
                          RandomSampler& sampler,
                          const ISPCCamera& camera,
                          RayStats& stats)
{
  RTCIntersectArguments args;
  rtcInitIntersectArguments(&args);
  args.feature_mask = (RTCFeatureFlags) (FEATURE_MASK);
#if USE_ARGUMENT_CALLBACKS
  args.intersect = sphereIntersectFunc;
#endif

  float time = data.g_motion_blur ? RandomSampler_get1D(sampler) * data.g_shutter_close: data.g_time;

  /* initialize ray */
  Ray ray(Vec3fa(camera.xfm.p),
                     Vec3fa(normalize(x*camera.xfm.l.vx + y*camera.xfm.l.vy + camera.xfm.l.vz)),
                     0.0f, inf, time, -1,
                     RTC_INVALID_GEOMETRY_ID, RTC_INVALID_GEOMETRY_ID);

  /* intersect ray with scene */
  rtcIntersect1(data.g_scene,RTCRayHit_(ray),&args);
  RayStats_addRay(stats);

  /* shade pixels */
  Vec3fa color = Vec3fa(1.0f);
  if (ray.geomID != RTC_INVALID_GEOMETRY_ID)
  {
    Vec3fa Ns = ray.Ng;
    Ns = face_forward(ray.dir,normalize(Ns));

    // shade sphere
    Vec3fa Ng = normalize(ray.Ng);
    float u = (atan2(Ng.z, Ng.x) + float(M_PI)) / (2.f * float(M_PI));
    float v = acos(Ng.y) / float(M_PI);
    u = 16*u+0.5f;
    v = 19*v+0.5f;
    color = ((u-(int)u) < 0.9f && (v-(int)v) < 0.9f) ? Vec3fa(0.5f) : Vec3fa(0.2f);
  }
  return color;
}

/* task that renders a single screen tile */
Vec3fa renderPixelStandard(const TutorialData& data,
                           float x, float y,
                           const ISPCCamera& camera,
                           RayStats& stats)
{
  RandomSampler sampler;

  Vec3fa L = Vec3fa(0.0f);

  for (int i=0; i<data.g_spp; i++)
  {
    RandomSampler_init(sampler, (int)x, (int)y, data.g_accu_count*data.g_spp+i);

    /* calculate pixel color */
    float fx = x + RandomSampler_get1D(sampler);
    float fy = y + RandomSampler_get1D(sampler);
    L = L + renderPixelFunction(data,fx,fy,sampler,camera,stats);
  }
  L = L/(float)data.g_spp;
  return L;
}

void renderPixelStandard(const TutorialData& data,
                         int x, int y,
                         int* pixels,
                         const unsigned int width,
                         const unsigned int height,
                         const float time,
                         const ISPCCamera& camera, RayStats& stats)
{
  Vec3fa color = renderPixelStandard(data,(float)x,(float)y,camera,stats);

  /* write color to framebuffer */
  Vec3ff accu_color = data.g_accu[y*width+x] + Vec3ff(color.x,color.y,color.z,1.0f); data.g_accu[y*width+x] = accu_color;
  float f = rcp(max(1.f,accu_color.w));
  unsigned int r = (unsigned int) (255.0f * clamp(accu_color.x*f,0.0f,1.0f));
  unsigned int g = (unsigned int) (255.0f * clamp(accu_color.y*f,0.0f,1.0f));
  unsigned int b = (unsigned int) (255.0f * clamp(accu_color.z*f,0.0f,1.0f));
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
  data.g_spp = g_spp;
  data.g_motion_blur = g_motion_blur;
  data.g_time = g_time;
  data.g_shutter_close = g_shutter_close;

  if (g_animate) {
    data.g_time = 0.5f * cos(time) + 0.5f;
    data.g_shutter_close = pow(data.g_time, 4.f);
    g_reset = true;
  }

  if (data.g_accu_width != width || data.g_accu_height != height) {
    alignedUSMFree(data.g_accu);
    data.g_accu = (Vec3ff*) alignedUSMMalloc((width*height)*sizeof(Vec3ff),16,EMBREE_USM_SHARED_DEVICE_READ_WRITE);
    data.g_accu_width = width;
    data.g_accu_height = height;
    for (unsigned int i=0; i<width*height; i++)
      data.g_accu[i] = Vec3ff(0.0f);
  }

  if (g_changed || g_reset)
  {
    updateTransformation();
    rtcCommitScene(data.g_scene);
  }


  /* reset accumulator */
  bool camera_changed = g_changed || g_reset; g_changed = false; g_reset = false;
  camera_changed |= ne(data.g_accu_vx,camera.xfm.l.vx); data.g_accu_vx = camera.xfm.l.vx;
  camera_changed |= ne(data.g_accu_vy,camera.xfm.l.vy); data.g_accu_vy = camera.xfm.l.vy;
  camera_changed |= ne(data.g_accu_vz,camera.xfm.l.vz); data.g_accu_vz = camera.xfm.l.vz;
  camera_changed |= ne(data.g_accu_p, camera.xfm.p);    data.g_accu_p  = camera.xfm.p;
  if (camera_changed) {
    data.g_accu_count=0;
    for (unsigned int i=0; i<width*height; i++)
      data.g_accu[i] = Vec3ff(0.0f);
  }
  else {
    data.g_accu_count++;
  }
}

/* called by the C++ code for cleanup */
extern "C" void device_cleanup ()
{
  TutorialData_Destructor(&data);
}

} // namespace embree
