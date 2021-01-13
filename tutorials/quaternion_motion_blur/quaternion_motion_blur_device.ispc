// Copyright 2009-2021 Intel Corporation
// SPDX-License-Identifier: Apache-2.0

#include "../common/tutorial/tutorial_device.isph"
#include "../common/math/random_sampler.isph"
#include "../common/math/sampling.isph"

/* accumulation buffer */
uniform Vec3ff* uniform g_accu = NULL;
uniform unsigned int g_accu_width = 0;
uniform unsigned int g_accu_height = 0;
uniform unsigned int g_accu_count = 0;
uniform Vec3fa g_accu_vx;
uniform Vec3fa g_accu_vy;
uniform Vec3fa g_accu_vz;
uniform Vec3fa g_accu_p;
extern uniform bool g_changed;
extern uniform float g_time;
extern uniform int g_spp;
extern uniform int g_numTimeSteps;
extern uniform float g_time;
extern uniform float g_shutter_close;
extern uniform bool g_animate;
extern uniform bool g_accumulate;
extern uniform bool g_motion_blur;
extern uniform bool g_reset;

RTCGeometry g_instance_linear_0 = NULL;
RTCGeometry g_instance_linear_1 = NULL;
RTCGeometry g_instance_quaternion_0 = NULL;
RTCGeometry g_instance_quaternion_1 = NULL;
uniform RTCQuaternionDecomposition qdc[10];

uniform AffineSpace3f fromQuaternionDecomposition(uniform const RTCQuaternionDecomposition& qdc)
{
  uniform AffineSpace3f T = make_AffineSpace3f_scale(make_Vec3f(1.f, 1.f, 1.f));
  T.p = make_Vec3f(qdc.translation_x, qdc.translation_y, qdc.translation_z);

  uniform AffineSpace3f S = make_AffineSpace3f_scale(make_Vec3f(1.f, 1.f, 1.f));
  S.l.vx.x = qdc.scale_x; S.l.vy.x = qdc.skew_xy; S.l.vz.x = qdc.skew_xz; S.p.x = qdc.shift_x;
                          S.l.vy.y = qdc.scale_y; S.l.vz.y = qdc.skew_yz; S.p.y = qdc.shift_y;
                                                  S.l.vz.z = qdc.scale_z; S.p.z = qdc.shift_z;

  uniform Quaternion3f q = make_Quaternion3f(make_Vec4f(
    qdc.quaternion_r, qdc.quaternion_i, qdc.quaternion_j, qdc.quaternion_k));

  uniform AffineSpace3f R = make_AffineSpace3f(make_LinearSpace3f(q));
  return T * R * S;
}

void updateTransformation()
{
  // transformation matrizes for instance 0 (rotation around axis through sphere center)
  rtcSetGeometryTimeStepCount(g_instance_linear_0, g_numTimeSteps);
  rtcSetGeometryTimeStepCount(g_instance_quaternion_0, g_numTimeSteps);
  for (uniform int i = 0; i < g_numTimeSteps; ++i)
  {
    // scale/skew, rotation, transformation data for quaternion motion blur
    uniform float K = g_numTimeSteps > 0 ? ((float)i)/(g_numTimeSteps-1) : 0.f;
    uniform float R = K * 2.0 * M_PI;
    if (g_numTimeSteps == 3) R = K * (2.0 - 1e-6f) * M_PI;

    uniform Quaternion3f q = make_Quaternion3f_rotate(make_Vec3f(0.f, 1.f, 0.f), R);
    rtcInitQuaternionDecomposition(qdc+i);
    rtcQuaternionDecompositionSetQuaternion(qdc+i, q.r, q.i, q.j, q.k);
    rtcQuaternionDecompositionSetScale(qdc+i, 3.f, 3.f, 3.f);
    rtcQuaternionDecompositionSetTranslation(qdc+i, -5.5f, 0.f, -5.5f);
    rtcSetGeometryTransformQuaternion(g_instance_quaternion_0, i, qdc+i);

    rtcQuaternionDecompositionSetTranslation(qdc+i, -5.5f, 0.f, 5.5f);
    uniform AffineSpace3f xfm = fromQuaternionDecomposition(qdc[i]);
    rtcSetGeometryTransform(g_instance_linear_0, i, RTC_FORMAT_FLOAT3X4_COLUMN_MAJOR,(uniform float* uniform)&(xfm.l.vx.x));
  }

  // transformation matrizes for instance 1 (translation and rotation around origin)
  rtcSetGeometryTimeStepCount(g_instance_linear_1, g_numTimeSteps);
  rtcSetGeometryTimeStepCount(g_instance_quaternion_1, g_numTimeSteps);
  for (uniform int i = 0; i < g_numTimeSteps; ++i)
  {
    // scale/skew, rotation, transformation data for quaternion motion blur
    uniform float K = g_numTimeSteps > 0 ? ((float)i)/(g_numTimeSteps-1) : 0.f;
    uniform float R = K * 2.0 * M_PI;
    if (g_numTimeSteps == 3) R = K * (2.0 - 1e-6f) * M_PI;

    uniform Quaternion3f q = make_Quaternion3f_rotate(make_Vec3f(0.f, 1.f, 0.f), R);
    rtcInitQuaternionDecomposition(qdc+i);
    rtcQuaternionDecompositionSetQuaternion(qdc+i, q.r, q.i, q.j, q.k);
    rtcQuaternionDecompositionSetShift(qdc+i, 3.f, 0.f, 3.f);
    rtcQuaternionDecompositionSetTranslation(qdc+i, 5.5f, 0.f, -5.5f);
    rtcSetGeometryTransformQuaternion(g_instance_quaternion_1, i, qdc+i);

    rtcQuaternionDecompositionSetTranslation(qdc+i, 5.5f, 0.f, 5.5f);
    uniform AffineSpace3f xfm = fromQuaternionDecomposition(qdc[i]);
    rtcSetGeometryTransform(g_instance_linear_1, i, RTC_FORMAT_FLOAT3X4_COLUMN_MAJOR,(uniform float* uniform)&(xfm.l.vx.x));
  }

  rtcCommitGeometry(g_instance_linear_0);
  rtcCommitGeometry(g_instance_linear_1);
  rtcCommitGeometry(g_instance_quaternion_0);
  rtcCommitGeometry(g_instance_quaternion_1);
}

// ======================================================================== //
//                     User defined sphere geometry                         //
// ======================================================================== //

struct Sphere
{
  ALIGNED_STRUCT_(16)
  Vec3f p;                      //!< position of the sphere
  float r;                      //!< radius of the sphere
  RTCGeometry geometry;
  uniform unsigned int geomID;
};

unmasked void sphereBoundsFunc(const struct RTCBoundsFunctionArguments* uniform args)
{
  const uniform Sphere* uniform spheres = (const uniform Sphere* uniform) args->geometryUserPtr;
  uniform RTCBounds* uniform bounds_o = args->bounds_o;
  const uniform Sphere& sphere = spheres[args->primID];
  bounds_o->lower_x = sphere.p.x-sphere.r;
  bounds_o->lower_y = sphere.p.y-sphere.r;
  bounds_o->lower_z = sphere.p.z-sphere.r;
  bounds_o->upper_x = sphere.p.x+sphere.r;
  bounds_o->upper_y = sphere.p.y+sphere.r;
  bounds_o->upper_z = sphere.p.z+sphere.r;
}

unmasked void sphereIntersectFunc(const RTCIntersectFunctionNArguments* uniform args)
{
  uniform int* uniform valid = args->valid;
  void* uniform ptr  = args->geometryUserPtr;
  varying Ray *uniform ray = (varying Ray*uniform)args->rayhit;
  varying RTCHit* uniform hit = (varying RTCHit *uniform)&ray->Ng.x;
  uniform unsigned int primID = args->primID;

  if (args->N != programCount)
    return;
  const uniform Sphere* uniform spheres = (const uniform Sphere* uniform)ptr;
  const uniform Sphere& sphere = spheres[primID];

  if (!valid[programIndex]) return;

  const Vec3f v = ray->org-sphere.p;
  const float A = dot(ray->dir,ray->dir);
  const float B = 2.0f*dot(v,ray->dir);
  const float C = dot(v,v) - sqr(sphere.r);
  const float D = B*B - 4.0f*A*C;
  if (D < 0.0f) return;
  const float Q = sqrt(D);
  const float rcpA = rcp(A);
  const float t0 = 0.5f*rcpA*(-B-Q);
  const float t1 = 0.5f*rcpA*(-B+Q);

  varying RTCHit potentialHit;
  potentialHit.u = 0.0f;
  potentialHit.v = 0.0f;
  potentialHit.geomID = sphere.geomID;
  potentialHit.primID = primID;
  if ((ray->tnear < t0) & (t0 < ray->tfar))
  {
    varying int imask;
    varying bool mask = __mask;
    unmasked {
      imask = mask ? -1 : 0;
    }

    const Vec3f Ng = ray->org+t0*ray->dir-sphere.p;
    potentialHit.Ng_x = Ng.x;
    potentialHit.Ng_y = Ng.y;
    potentialHit.Ng_z = Ng.z;

    uniform RTCFilterFunctionNArguments fargs;
    fargs.valid = (int* uniform)&imask;
    fargs.geometryUserPtr = ptr;
    fargs.context = args->context;
    fargs.ray = (RTCRayN *uniform)args->rayhit;
    fargs.hit = (RTCHitN* uniform)&potentialHit;
    fargs.N = programCount;

    const float old_t = ray->tfar;
    ray->tfar = t0;
    rtcFilterIntersection(args,&fargs);

    if (imask == -1)
      *hit = potentialHit;
    else
      ray->tfar = old_t;
  }

  if ((ray->tnear < t1) & (t1 < ray->tfar))
  {
    varying int imask;
    varying bool mask = __mask;
    unmasked {
      imask = mask ? -1 : 0;
    }

    const Vec3f Ng = ray->org+t1*ray->dir-sphere.p;
    potentialHit.Ng_x = Ng.x;
    potentialHit.Ng_y = Ng.y;
    potentialHit.Ng_z = Ng.z;

    uniform RTCFilterFunctionNArguments fargs;
    fargs.valid = (int* uniform)&imask;
    fargs.geometryUserPtr = ptr;
    fargs.context = args->context;
    fargs.ray = (RTCRayN *uniform)args->rayhit;
    fargs.hit = (RTCHitN* uniform)&potentialHit;
    fargs.N = programCount;

    const float old_t = ray->tfar;
    ray->tfar = t1;
    rtcFilterIntersection(args,&fargs);

    if (imask == -1)
      *hit = potentialHit;
    else
      ray->tfar = old_t;
  }
}

uniform Sphere* uniform createAnalyticalSpheres (RTCScene scene, uniform unsigned int N)
{
  RTCGeometry geom = rtcNewGeometry(g_device, RTC_GEOMETRY_TYPE_USER);
  uniform Sphere* uniform spheres = uniform new uniform Sphere[N];
  uniform unsigned int geomID = rtcAttachGeometry(scene,geom);
  for (uniform unsigned int i=0; i<N; i++) {
    spheres[i].geometry = geom;
    spheres[i].geomID = geomID;
  }
  rtcSetGeometryUserPrimitiveCount(geom,N);
  rtcSetGeometryUserData(geom,spheres);
  rtcSetGeometryBoundsFunction(geom,sphereBoundsFunc,NULL);
  rtcSetGeometryIntersectFunction(geom,sphereIntersectFunc);
  rtcCommitGeometry(geom);
  rtcReleaseGeometry(geom);
  return spheres;
}

/* scene data */
RTCScene g_scene  = NULL;
RTCScene g_scene0 = NULL;
uniform Sphere* uniform g_spheres = NULL;

/* called by the C++ code for initialization */
export void device_init (uniform int8* uniform cfg)
{
  /* create scene */
  g_scene = rtcNewScene(g_device);

  /* create scene with 4 analytical spheres */
  g_scene0 = rtcNewScene(g_device);
  g_spheres = createAnalyticalSpheres(g_scene0, 1);
  g_spheres[0].p = make_Vec3f(0, 0, 0);
  g_spheres[0].r = 1.0f;
  rtcCommitScene(g_scene0);

  // attach multiple times otherwise Embree will optimize and not use
  // internal instancing (magic!)
  g_instance_linear_0 = rtcNewGeometry(g_device, RTC_GEOMETRY_TYPE_INSTANCE);
  g_instance_linear_1 = rtcNewGeometry(g_device, RTC_GEOMETRY_TYPE_INSTANCE);
  g_instance_quaternion_0 = rtcNewGeometry(g_device, RTC_GEOMETRY_TYPE_INSTANCE);
  g_instance_quaternion_1 = rtcNewGeometry(g_device, RTC_GEOMETRY_TYPE_INSTANCE);
  rtcSetGeometryInstancedScene(g_instance_linear_0, g_scene0);
  rtcSetGeometryInstancedScene(g_instance_linear_1, g_scene0);
  rtcSetGeometryInstancedScene(g_instance_quaternion_0, g_scene0);
  rtcSetGeometryInstancedScene(g_instance_quaternion_1, g_scene0);
  updateTransformation();
  for (uniform int i = 0; i < 2; ++i)
  {
    rtcAttachGeometry(g_scene, g_instance_linear_0);
    rtcAttachGeometry(g_scene, g_instance_linear_1);
    rtcAttachGeometry(g_scene, g_instance_quaternion_0);
    rtcAttachGeometry(g_scene, g_instance_quaternion_1);
  }
  rtcReleaseGeometry(g_instance_linear_0);
  rtcReleaseGeometry(g_instance_linear_1);
  rtcReleaseGeometry(g_instance_quaternion_0);
  rtcReleaseGeometry(g_instance_quaternion_1);
  rtcCommitGeometry(g_instance_linear_0);
  rtcCommitGeometry(g_instance_linear_1);
  rtcCommitGeometry(g_instance_quaternion_0);
  rtcCommitGeometry(g_instance_quaternion_1);

  rtcCommitScene (g_scene);
}

inline Vec3f face_forward(const Vec3f& dir, const Vec3f& _Ng) {
  const Vec3f Ng = _Ng;
  return dot(dir,Ng) < 0.0f ? Ng : neg(Ng);
}

Vec3f renderPixelFunction(float x, float y, RandomSampler& sampler, const uniform ISPCCamera& camera, uniform RayStats& stats)
{
  uniform RTCIntersectContext context;
  rtcInitIntersectContext(&context);

  float time = g_motion_blur ? RandomSampler_get1D(sampler) * g_shutter_close: g_time;

  /* initialize ray */
  Ray ray = make_Ray(make_Vec3f(camera.xfm.p),
                     make_Vec3f(normalize(x*camera.xfm.l.vx + y*camera.xfm.l.vy + camera.xfm.l.vz)),
                     0.0f, inf, time, -1,
                     RTC_INVALID_GEOMETRY_ID, RTC_INVALID_GEOMETRY_ID);

  /* intersect ray with scene */
  rtcIntersectV(g_scene,&context,RTCRayHit_(ray));
  RayStats_addRay(stats);

  /* shade pixels */
  Vec3f color = make_Vec3f(1.0f);
  if (ray.geomID != RTC_INVALID_GEOMETRY_ID)
  {
    Vec3f Ns = ray.Ng;
    Ns = face_forward(ray.dir,normalize(Ns));

    // shade sphere
    Vec3f Ng = normalize(ray.Ng);
    float u = (atan2(Ng.z, Ng.x) + M_PI) / (2.f * M_PI);
    float v = acos(Ng.y) / M_PI;
    u = 16*u+0.5f;
    v = 19*v+0.5f;
    color = ((u-(int)u) < 0.9 && (v-(int)v) < 0.9) ? make_Vec3f(0.5f) : make_Vec3f(0.2f);
  }
  return color;
}

/* task that renders a single screen tile */
Vec3f renderPixelStandard(float x, float y, const uniform ISPCCamera& camera, uniform RayStats& stats)
{
  RandomSampler sampler;

  Vec3f L = make_Vec3f(0.0f);

  for (uniform int i=0; i<g_spp; i++)
  {
    RandomSampler_init(sampler, (int)x, (int)y, g_accu_count*g_spp+i);

    /* calculate pixel color */
    float fx = x + RandomSampler_get1D(sampler);
    float fy = y + RandomSampler_get1D(sampler);
    L = L + renderPixelFunction(fx,fy,sampler,camera,stats);
  }
  L = L/(uniform float)g_spp;
  return L;
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
    if (all(__mask == 0)) continue;

    /* calculate pixel color */
    Vec3f color = renderPixelStandard((float)x,(float)y,camera,g_stats[threadIndex]);

    /* write color to framebuffer */
    Vec3ff accu_color = g_accu[y*width+x] + make_Vec3ff(color.x,color.y,color.z,1.0f); g_accu[y*width+x] = accu_color;
    float f = rcp(max(1.f,accu_color.w));
    unsigned int r = (unsigned int) (255.0f * clamp(accu_color.x*f,0.0f,1.0f));
    unsigned int g = (unsigned int) (255.0f * clamp(accu_color.y*f,0.0f,1.0f));
    unsigned int b = (unsigned int) (255.0f * clamp(accu_color.z*f,0.0f,1.0f));
    pixels[y*width+x] = (b << 16) + (g << 8) + r;
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

export void renderFrameStandard (uniform int* uniform pixels,
                          const uniform unsigned int width,
                          const uniform unsigned int height,
                          const uniform float time,
                          const uniform ISPCCamera& camera)
{
  const uniform int numTilesX = (width +TILE_SIZE_X-1)/TILE_SIZE_X;
  const uniform int numTilesY = (height+TILE_SIZE_Y-1)/TILE_SIZE_Y;
  launch[numTilesX*numTilesY] renderTileTask(pixels,width,height,time,camera,numTilesX,numTilesY); sync;
}


/* called by the C++ code to render */
export void device_render (uniform int* uniform pixels,
                           const uniform unsigned int width,
                           const uniform unsigned int height,
                           const uniform float time,
                           const uniform ISPCCamera& camera)
{
  if (g_animate) {
    g_time = 0.5f * cos(time) + 0.5f;
    g_shutter_close = pow(g_time, 4.f);
    g_reset = true;
  }

  if (g_accu_width != width || g_accu_height != height) {
    delete[] g_accu;
    g_accu = uniform new uniform Vec3ff[width*height];
    g_accu_width = width;
    g_accu_height = height;
    for (uniform unsigned int i=0; i<width*height; i++)
      g_accu[i] = make_Vec3ff(0.0f);
  }

  if (g_changed || g_reset)
  {
    updateTransformation();
    rtcCommitScene(g_scene);
  }


  /* reset accumulator */
  uniform bool camera_changed = g_changed || g_reset; g_changed = false; g_reset = false;
  camera_changed |= ne(g_accu_vx,camera.xfm.l.vx); g_accu_vx = camera.xfm.l.vx;
  camera_changed |= ne(g_accu_vy,camera.xfm.l.vy); g_accu_vy = camera.xfm.l.vy;
  camera_changed |= ne(g_accu_vz,camera.xfm.l.vz); g_accu_vz = camera.xfm.l.vz;
  camera_changed |= ne(g_accu_p, camera.xfm.p);    g_accu_p  = camera.xfm.p;
  if (camera_changed) {
    g_accu_count=0;
    for (uniform unsigned int i=0; i<width*height; i++)
      g_accu[i] = make_Vec3ff(0.0f);
  }
  else {
    g_accu_count++;
  }
}

/* called by the C++ code for cleanup */
export void device_cleanup ()
{
  rtcReleaseScene (g_scene); g_scene = NULL;
  rtcReleaseScene (g_scene0); g_scene0 = NULL;
  rtcReleaseDevice(g_device); g_device = NULL;
  delete[] g_spheres; g_spheres = NULL;
  delete[] g_accu; g_accu = NULL;
}
