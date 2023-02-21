// Copyright 2009-2021 Intel Corporation
// SPDX-License-Identifier: Apache-2.0

#include "user_geometry_device.isph"

#if EMBREE_SYCL_TUTORIAL
#define USE_ARGUMENT_CALLBACKS 1
#else
#define USE_ARGUMENT_CALLBACKS 0
#endif

/* all features required by this tutorial */
#if USE_ARGUMENT_CALLBACKS
#define FEATURE_MASK \
  RTC_FEATURE_FLAG_TRIANGLE | \
  RTC_FEATURE_FLAG_USER_GEOMETRY_CALLBACK_IN_ARGUMENTS | \
  RTC_FEATURE_FLAG_FILTER_FUNCTION_IN_ARGUMENTS
#else
#define FEATURE_MASK     \
  RTC_FEATURE_FLAG_TRIANGLE | \
  RTC_FEATURE_FLAG_USER_GEOMETRY_CALLBACK_IN_GEOMETRY | \
  RTC_FEATURE_FLAG_FILTER_FUNCTION_IN_GEOMETRY
#endif

#define ENABLE_NATIVE_INSTANCING 0

RTCScene g_scene = NULL;
uniform TutorialData data;

const uniform int numPhi = 5;
const uniform int numTheta = 2*numPhi;

RTC_SYCL_INDIRECTLY_CALLABLE unmasked void contextFilterFunction(const RTCFilterFunctionNArguments* uniform args);

uniform RTCIntersectFunctionN instanceIntersectFuncPtr = NULL;
uniform RTCOccludedFunctionN instanceOccludedFuncPtr = NULL;
uniform RTCIntersectFunctionN sphereIntersectFuncPtr = NULL;
uniform RTCOccludedFunctionN sphereOccludedFuncPtr = NULL;
uniform RTCFilterFunctionN sphereFilterFuncPtr = NULL;

inline void pushInstanceId(uniform RTCRayQueryContext* uniform ctx, uniform unsigned int id)
{
#if RTC_MAX_INSTANCE_LEVEL_COUNT > 1
  ctx->instID[ctx->instStackSize++] = id;
#else
  ctx->instID[0] = id;
#endif
}

inline void popInstanceId(uniform RTCRayQueryContext* uniform ctx)
{
#if RTC_MAX_INSTANCE_LEVEL_COUNT > 1
  ctx->instID[--ctx->instStackSize] = RTC_INVALID_GEOMETRY_ID;
#else
  ctx->instID[0] = RTC_INVALID_GEOMETRY_ID;
#endif
}

inline void copyInstanceIdStack(uniform const RTCRayQueryContext* uniform ctx, varying unsigned* uniform tgt)
{
  tgt[0] = ctx->instID[0];
#if (RTC_MAX_INSTANCE_LEVEL_COUNT > 1)
  for (unsigned l = 1; l < RTC_MAX_INSTANCE_LEVEL_COUNT && l < ctx->instStackSize; ++l)
    tgt[l] = ctx->instID[l];
#endif
}

// ======================================================================== //
//                         User defined instancing                          //
// ======================================================================== //

unmasked void instanceBoundsFunc(const struct RTCBoundsFunctionArguments* uniform args)
{
  const uniform Instance* uniform instance = (const uniform Instance* uniform) args->geometryUserPtr;
  uniform RTCBounds* uniform bounds_o = args->bounds_o;
  uniform Vec3f l = instance->lower;
  uniform Vec3f u = instance->upper;
  uniform Vec3f p000 = xfmPoint(instance->local2world,make_Vec3f(l.x,l.y,l.z));
  uniform Vec3f p001 = xfmPoint(instance->local2world,make_Vec3f(l.x,l.y,u.z));
  uniform Vec3f p010 = xfmPoint(instance->local2world,make_Vec3f(l.x,u.y,l.z));
  uniform Vec3f p011 = xfmPoint(instance->local2world,make_Vec3f(l.x,u.y,u.z));
  uniform Vec3f p100 = xfmPoint(instance->local2world,make_Vec3f(u.x,l.y,l.z));
  uniform Vec3f p101 = xfmPoint(instance->local2world,make_Vec3f(u.x,l.y,u.z));
  uniform Vec3f p110 = xfmPoint(instance->local2world,make_Vec3f(u.x,u.y,l.z));
  uniform Vec3f p111 = xfmPoint(instance->local2world,make_Vec3f(u.x,u.y,u.z));
  uniform Vec3f lower = min(min(min(p000,p001),min(p010,p011)),min(min(p100,p101),min(p110,p111)));
  uniform Vec3f upper = max(max(max(p000,p001),max(p010,p011)),max(max(p100,p101),max(p110,p111)));
  bounds_o->lower_x = lower.x;
  bounds_o->lower_y = lower.y;
  bounds_o->lower_z = lower.z;
  bounds_o->upper_x = upper.x;
  bounds_o->upper_y = upper.y;
  bounds_o->upper_z = upper.z;
}

RTC_SYCL_INDIRECTLY_CALLABLE unmasked void instanceIntersectFunc(const RTCIntersectFunctionNArguments* uniform args)
{
  const uniform int* uniform valid = args->valid;
  void* uniform ptr  = args->geometryUserPtr;
  RTCRayHitN* uniform rays = (RTCRayHitN* uniform)args->rayhit;
  
  assert(args->N == programCount);
  if (!valid[programIndex])
    return;
  
  varying Ray *uniform ray = (varying Ray*uniform)rays;
  const uniform Instance* uniform instance = (const uniform Instance* uniform)ptr;
  const Vec3f_ ray_org = ray->org;
  const Vec3f_ ray_dir = ray->dir;
  const float ray_tnear = ray->tnear;
  const float ray_tfar  = ray->tfar;

#if 0

  uniform RTCRayQueryContext* uniform context = args->context;
  ray->org = make_Vec3f_(xfmPoint (instance->world2local,ray_org));
  ray->dir = make_Vec3f_(xfmVector(instance->world2local,ray_dir));
  ray->tnear = ray_tnear;
  ray->tfar  = ray_tfar;
  pushInstanceId(context, args->geomID);
  uniform RTCIntersectArguments args;
  rtcInitIntersectArguments(&args);
  args.context = context;
  rtcIntersectV(instance->object,RTCRayHit_(*ray),&args);
  popInstanceId(context);
  const float updated_tfar = ray->tfar;
  ray->org = ray_org;
  ray->dir = ray_dir;
  ray->tfar = updated_tfar;

#else
  
  RTCRay xray;
  const Vec3f org = xfmPoint (instance->world2local,ray_org);
  const Vec3f dir = xfmVector(instance->world2local,ray_dir);
  xray.org_x = org.x; xray.org_y = org.y; xray.org_z = org.z;
  xray.dir_x = dir.x; xray.dir_y = dir.y; xray.dir_z = dir.z;
  xray.tnear = ray_tnear;
  xray.tfar  = ray_tfar;
  xray.time = 0.0f;
  xray.mask = -1;
  xray.id = 0;
  xray.flags = 0;
  
  rtcForwardIntersectV(args,instance->object,&xray,args->geomID);
  
#endif
}

RTC_SYCL_INDIRECTLY_CALLABLE unmasked void instanceOccludedFunc(const RTCOccludedFunctionNArguments* uniform args)
{
  const uniform int* uniform valid = args->valid;
  void* uniform ptr  = args->geometryUserPtr;
  RTCRayHitN* uniform rays = (RTCRayHitN* uniform)args->ray;
  assert(args->N == programCount);
  if (!valid[programIndex])
    return;
  
  varying Ray *uniform ray = (varying Ray*uniform)rays;
  const uniform Instance* uniform instance = (const uniform Instance* uniform)ptr;
  const Vec3f_ ray_org = ray->org;
  const Vec3f_ ray_dir = ray->dir;
  const float ray_tnear = ray->tnear;
  const float ray_tfar  = ray->tfar;

#if 0

  uniform RTCRayQueryContext* uniform context = args->context;
  ray->org    = make_Vec3f_(xfmPoint (instance->world2local,ray_org));
  ray->dir    = make_Vec3f_(xfmVector(instance->world2local,ray_dir));
  ray->tnear  = ray_tnear;
  ray->tfar   = ray_tfar;
  pushInstanceId(context, args->geomID);
  uniform RTCOccludedArguments args;
  rtcInitOccludedArguments(&args);
  args.context = context;
  rtcOccludedV(instance->object,RTCRay_(*ray),&args);
  popInstanceId(context);
  const float updated_tfar = ray->tfar;
  ray->org    = ray_org;
  ray->dir    = ray_dir;
  ray->tnear  = ray_tnear;
  ray->tfar   = updated_tfar;

#else

  RTCRay xray;
  const Vec3f org = xfmPoint (instance->world2local,ray_org);
  const Vec3f dir = xfmVector(instance->world2local,ray_dir);
  xray.org_x = org.x; xray.org_y = org.y; xray.org_z = org.z;
  xray.dir_x = dir.x; xray.dir_y = dir.y; xray.dir_z = dir.z;
  xray.tnear = ray_tnear;
  xray.tfar  = ray_tfar;
  xray.time = 0.0f;
  xray.mask = -1;
  xray.id = 0;
  xray.flags = 0;
  
  rtcForwardOccludedV(args,instance->object,&xray,args->geomID);
  
#endif
}

uniform Instance* uniform createInstance (RTCScene scene, RTCScene object, uniform int geomID, const uniform Vec3f& lower, const uniform Vec3f& upper)
{
#if !ENABLE_NATIVE_INSTANCING
  uniform Instance* uniform instance = uniform new uniform Instance;
  instance->type = USER_GEOMETRY_INSTANCE;
  instance->object = object;
  instance->lower = lower;
  instance->upper = upper;
  instance->local2world.l.vx = make_Vec3f(1,0,0);
  instance->local2world.l.vy = make_Vec3f(0,1,0);
  instance->local2world.l.vz = make_Vec3f(0,0,1);
  instance->local2world.p    = make_Vec3f(0,0,0);
  instance->geometry = rtcNewGeometry(g_device, RTC_GEOMETRY_TYPE_USER);
  rtcSetGeometryUserPrimitiveCount(instance->geometry,1);
  rtcSetGeometryUserData(instance->geometry,instance);
  rtcSetGeometryBoundsFunction(instance->geometry,instanceBoundsFunc,NULL);
  rtcSetGeometryIntersectFunction(instance->geometry,instanceIntersectFuncPtr);
  rtcSetGeometryOccludedFunction (instance->geometry,instanceOccludedFuncPtr);
  rtcCommitGeometry(instance->geometry);
  rtcAttachGeometry(scene,instance->geometry);
  rtcReleaseGeometry(instance->geometry);
  return instance;
#else
  uniform Instance* uniform instance = uniform new uniform Instance;
  instance->type = USER_GEOMETRY_INSTANCE;
  instance->geometry = rtcNewGeometry (g_device, RTC_GEOMETRY_TYPE_INSTANCE);
  rtcSetGeometryInstancedScene(instance->geometry,object);
  rtcSetGeometryTimeStepCount(instance->geometry,1);
  rtcCommitGeometry(instance->geometry);
  rtcAttachGeometryByID(scene,instance->geometry,geomID);
  rtcReleaseGeometry(instance->geometry);
  return instance;
#endif
}

void updateInstance (RTCScene scene, uniform Instance* uniform instance)
{
#if ENABLE_NATIVE_INSTANCING
  rtcSetGeometryTransform(instance->geometry,0,RTC_FORMAT_FLOAT4X4_COLUMN_MAJOR,(float* uniform)&instance->local2world);
#endif
  
  instance->world2local = rcp(instance->local2world);
  instance->normal2world = transposed(rcp(instance->local2world.l));
  rtcCommitGeometry(instance->geometry);
}

// ======================================================================== //
//                     User defined sphere geometry                         //
// ======================================================================== //

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

RTC_SYCL_INDIRECTLY_CALLABLE unmasked void sphereIntersectFunc(const RTCIntersectFunctionNArguments* uniform args)
{
  uniform int* uniform valid = args->valid;
  void* uniform ptr  = args->geometryUserPtr;
  varying Ray *uniform ray = (varying Ray*uniform)args->rayhit;
  varying RTCHit* uniform hit = (varying RTCHit *uniform)&ray->Ng.x;
  uniform unsigned int primID = args->primID;
  
  assert(args->N == programCount);
  const uniform Sphere* uniform spheres = (const uniform Sphere* uniform)ptr;
  const uniform Sphere& sphere = spheres[primID];
  
  if (!valid[programIndex]) return;
  valid[programIndex] = 0;

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
  copyInstanceIdStack(args->context, potentialHit.instID);
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
#if USE_ARGUMENT_CALLBACKS
    contextFilterFunction(&fargs);
#else
    rtcInvokeIntersectFilterFromGeometry(args,&fargs);
#endif

    if (imask == -1) {
      *hit = potentialHit;
      valid[programIndex] = -1;
    }
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
#if USE_ARGUMENT_CALLBACKS
    contextFilterFunction(&fargs);
#else
    rtcInvokeIntersectFilterFromGeometry(args,&fargs);
#endif

    if (imask == -1) {
      *hit = potentialHit;
      valid[programIndex] = -1;
    }
    else
      ray->tfar = old_t;
  }
}

RTC_SYCL_INDIRECTLY_CALLABLE unmasked void sphereOccludedFunc(const RTCOccludedFunctionNArguments* uniform args)
{
  uniform int* uniform valid = args->valid;
  void* uniform ptr  = args->geometryUserPtr;
  varying Ray *uniform ray = (varying Ray*uniform)args->ray;
  uniform unsigned int primID = args->primID;
  
  assert(args->N == programCount);
  const uniform Sphere* uniform spheres = (const uniform Sphere* uniform) ptr;
  const uniform Sphere& sphere = spheres[primID];
  
  if (!valid[programIndex]) return;
  valid[programIndex] = 0;

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
  copyInstanceIdStack(args->context, potentialHit.instID);
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
    fargs.ray = args->ray;
    fargs.hit = (RTCHitN* uniform)&potentialHit;
    fargs.N = programCount;

    const float old_t = ray->tfar;
    ray->tfar = t0;
#if USE_ARGUMENT_CALLBACKS
    contextFilterFunction(&fargs);
#else
    rtcInvokeOccludedFilterFromGeometry(args,&fargs);
#endif
    
    if (imask == -1) {
      ray->tfar = neg_inf;
      valid[programIndex] = -1;
    }
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
    fargs.ray = args->ray;
    fargs.hit = (RTCHitN* uniform)&potentialHit;
    fargs.N = programCount;

    const float old_t = ray->tfar;
    ray->tfar = t1;
#if USE_ARGUMENT_CALLBACKS
    contextFilterFunction(&fargs);
#else
    rtcInvokeOccludedFilterFromGeometry(args,&fargs);
#endif

    if (imask == -1) {
      ray->tfar = neg_inf;
      valid[programIndex] = -1;
    }
    else
      ray->tfar = old_t;
  }
}

RTC_SYCL_INDIRECTLY_CALLABLE unmasked void contextIntersectFunc(const RTCIntersectFunctionNArguments* uniform args)
{
  UserGeometryType* uniform type = (UserGeometryType* uniform) args->geometryUserPtr;
  if (*type == USER_GEOMETRY_INSTANCE) instanceIntersectFunc(args);
  else                                 sphereIntersectFunc(args);
}

RTC_SYCL_INDIRECTLY_CALLABLE unmasked void contextOccludedFunc(const RTCOccludedFunctionNArguments* uniform args)
{
  UserGeometryType* uniform type = (UserGeometryType* uniform) args->geometryUserPtr;
  if (*type == USER_GEOMETRY_INSTANCE) instanceOccludedFunc(args);
  else                                 sphereOccludedFunc(args);
}

/* intersection filter function */

RTC_SYCL_INDIRECTLY_CALLABLE unmasked void sphereFilterFunction(const RTCFilterFunctionNArguments* uniform args)
{
  uniform int* uniform valid = args->valid;
  const uniform RayQueryContext* uniform context = (const uniform RayQueryContext* uniform) args->context;
  varying struct Ray* uniform ray    = (varying struct Ray* uniform)args->ray;
  //varying struct RTCHit* uniform hit = (varying struct RTCHit* uniform)args->hit;
  const uniform unsigned int N = args->N;
  assert(N == programCount);

  /* avoid crashing when debug visualizations are used */
  if (context == NULL)
    return;

  /* ignore inactive rays */
  if (valid[programIndex] != -1) return;
  
  /* carve out parts of the sphere */
  const Vec3f h = ray->org+ray->dir*ray->tfar;
  float v = abs(sin(10.0f*h.x)*cos(10.0f*h.y)*sin(10.0f*h.z));
  float T = clamp((v-0.1f)*3.0f,0.0f,1.0f);

  /* reject some hits */
  if (T < 0.5f) valid[programIndex] = 0;
}

RTC_SYCL_INDIRECTLY_CALLABLE unmasked void contextFilterFunction(const RTCFilterFunctionNArguments* uniform args)
{
  uniform int* uniform valid = args->valid;
  if (!valid[programIndex]) return;
  
  varying RTCHit* uniform potential_hit = (varying RTCHit* uniform) args->hit;
  if (potential_hit->instID[0] == 0)
    sphereFilterFunction(args);
}

uniform Sphere* uniform createAnalyticalSphere (RTCScene scene, const uniform Vec3f& p, uniform float r)
{
  RTCGeometry geom = rtcNewGeometry(g_device, RTC_GEOMETRY_TYPE_USER);
  uniform Sphere* uniform sphere = uniform new uniform Sphere;
  sphere->type = USER_GEOMETRY_SPHERE;
  sphere->p = p;
  sphere->r = r;
  sphere->geometry = geom;
  sphere->geomID = rtcAttachGeometry(scene,geom);
  rtcSetGeometryUserPrimitiveCount(geom,1);
  rtcSetGeometryUserData(geom,sphere);
  rtcSetGeometryBoundsFunction(geom,sphereBoundsFunc,NULL);
#if !USE_ARGUMENT_CALLBACKS
    rtcSetGeometryIntersectFunction(geom,sphereIntersectFuncPtr);
    rtcSetGeometryOccludedFunction (geom,sphereOccludedFuncPtr);
#endif
  rtcCommitGeometry(geom);
  rtcReleaseGeometry(geom);
  return sphere;
}

uniform Sphere* uniform createAnalyticalSpheres (RTCScene scene, uniform unsigned int N)
{
  RTCGeometry geom = rtcNewGeometry(g_device, RTC_GEOMETRY_TYPE_USER);
  uniform Sphere* uniform spheres = uniform new uniform Sphere[N];
  uniform unsigned int geomID = rtcAttachGeometry(scene,geom);
  for (uniform unsigned int i=0; i<N; i++) {
    spheres[i].type = USER_GEOMETRY_SPHERE;
    spheres[i].geometry = geom;
    spheres[i].geomID = geomID;
  }
  rtcSetGeometryUserPrimitiveCount(geom,N);
  rtcSetGeometryUserData(geom,spheres);
  rtcSetGeometryBoundsFunction(geom,sphereBoundsFunc,NULL);
#if !USE_ARGUMENT_CALLBACKS
  rtcSetGeometryIntersectFunction(geom,sphereIntersectFuncPtr);
  rtcSetGeometryOccludedFunction (geom,sphereOccludedFuncPtr);
#endif
#if !USE_ARGUMENT_CALLBACKS
  rtcSetGeometryIntersectFilterFunction(geom,sphereFilterFuncPtr);
  rtcSetGeometryOccludedFilterFunction(geom,sphereFilterFuncPtr);
#endif
  rtcCommitGeometry(geom);
  rtcReleaseGeometry(geom);
  return spheres;
}

// ======================================================================== //
//                      Triangular sphere geometry                          //
// ======================================================================== //

uniform unsigned int createTriangulatedSphere (RTCScene scene, const uniform Vec3f& p, uniform float r)
{
  /* create triangle mesh */
  RTCGeometry geom = rtcNewGeometry (g_device, RTC_GEOMETRY_TYPE_TRIANGLE);

  /* map triangle and vertex buffers */
  uniform Vertex* uniform vertices = (uniform Vertex* uniform) rtcSetNewGeometryBuffer(geom,RTC_BUFFER_TYPE_VERTEX,0,RTC_FORMAT_FLOAT3,sizeof(uniform Vertex),numTheta*(numPhi+1));
  uniform Triangle* uniform triangles = (uniform Triangle* uniform) rtcSetNewGeometryBuffer(geom,RTC_BUFFER_TYPE_INDEX,0,RTC_FORMAT_UINT3,sizeof(uniform Triangle),2*numTheta*(numPhi-1));

  /* create sphere */
  uniform int tri = 0;
  const uniform float rcpNumTheta = rcp((uniform float)numTheta);
  const uniform float rcpNumPhi   = rcp((uniform float)numPhi);
  for (uniform int phi=0; phi<=numPhi; phi++)
  {
    for (uniform int theta=0; theta<numTheta; theta++)
    {
      const uniform float phif   = phi*pi*rcpNumPhi;
      const uniform float thetaf = theta*2.0f*pi*rcpNumTheta;

      uniform Vertex& v = vertices[phi*numTheta+theta];
      v.x = p.x + r*sin(phif)*sin(thetaf);
      v.y = p.y + r*cos(phif);
      v.z = p.z + r*sin(phif)*cos(thetaf);
    }
    if (phi == 0) continue;

    for (uniform int theta=1; theta<=numTheta; theta++)
    {
      uniform int p00 = (phi-1)*numTheta+theta-1;
      uniform int p01 = (phi-1)*numTheta+theta%numTheta;
      uniform int p10 = phi*numTheta+theta-1;
      uniform int p11 = phi*numTheta+theta%numTheta;

      if (phi > 1) {
        triangles[tri].v0 = p10;
        triangles[tri].v1 = p00;
        triangles[tri].v2 = p01;
        tri++;
      }

      if (phi < numPhi) {
        triangles[tri].v0 = p11;
        triangles[tri].v1 = p10;
        triangles[tri].v2 = p01;
        tri++;
      }
    }
  }

  rtcCommitGeometry(geom);
  uniform unsigned int geomID = rtcAttachGeometry(scene,geom);
  rtcReleaseGeometry(geom);
  return geomID;
}

/* creates a ground plane */
uniform unsigned int createGroundPlane (RTCScene scene)
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
  triangles[0].v0 = 0; triangles[0].v1 = 2; triangles[0].v2 = 1;
  triangles[1].v0 = 1; triangles[1].v1 = 2; triangles[1].v2 = 3;

  rtcCommitGeometry(geom);
  uniform unsigned int geomID = rtcAttachGeometry(scene,geom);
  rtcReleaseGeometry(geom);
  return geomID;
}

/* called by the C++ code for initialization */
export void device_init (uniform int8* uniform cfg)
{
  instanceIntersectFuncPtr = GET_FUNCTION_POINTER(instanceIntersectFunc);
  instanceOccludedFuncPtr  = GET_FUNCTION_POINTER(instanceOccludedFunc );
  sphereIntersectFuncPtr = GET_FUNCTION_POINTER(sphereIntersectFunc);
  sphereOccludedFuncPtr  = GET_FUNCTION_POINTER(sphereOccludedFunc );
  sphereFilterFuncPtr    = GET_FUNCTION_POINTER(sphereFilterFunction);
  
  /* create scene */
  TutorialData_Constructor(&data);
  g_scene = data.g_scene = rtcNewScene(g_device);

  /* create scene with 4 analytical spheres */
  data.g_scene0 = rtcNewScene(g_device);
  rtcSetSceneBuildQuality(data.g_scene0,RTC_BUILD_QUALITY_LOW);
  data.g_spheres = createAnalyticalSpheres(data.g_scene0,4);
  data.g_spheres[0].p = make_Vec3f( 0, 0,+1); data.g_spheres[0].r = 0.5f;
  data.g_spheres[1].p = make_Vec3f(+1, 0, 0); data.g_spheres[1].r = 0.5f;
  data.g_spheres[2].p = make_Vec3f( 0, 0,-1); data.g_spheres[2].r = 0.5f;
  data.g_spheres[3].p = make_Vec3f(-1, 0, 0); data.g_spheres[3].r = 0.5f;
  rtcCommitScene(data.g_scene0);

  /* create scene with 4 triangulated spheres */
  data.g_scene1 = rtcNewScene(g_device);
  createTriangulatedSphere(data.g_scene1,make_Vec3f( 0, 0,+1),0.5f);
  createTriangulatedSphere(data.g_scene1,make_Vec3f(+1, 0, 0),0.5f);
  createTriangulatedSphere(data.g_scene1,make_Vec3f( 0, 0,-1),0.5f);
  createTriangulatedSphere(data.g_scene1,make_Vec3f(-1, 0, 0),0.5f);
  rtcCommitScene(data.g_scene1);

  /* create scene with 2 triangulated and 2 analytical spheres */
  data.g_scene2 = rtcNewScene(g_device);
  createTriangulatedSphere(data.g_scene2,make_Vec3f( 0, 0,+1),0.5f);
  data.g_sphere0 = createAnalyticalSphere  (data.g_scene2,make_Vec3f(+1, 0, 0),0.5f);
  createTriangulatedSphere(data.g_scene2,make_Vec3f( 0, 0,-1),0.5f);
  data.g_sphere1 = createAnalyticalSphere  (data.g_scene2,make_Vec3f(-1, 0, 0),0.5f);
  rtcCommitScene(data.g_scene2);

  /* instantiate geometry */
  data.g_instance[0] = createInstance(data.g_scene,data.g_scene0,0,make_Vec3f(-2,-2,-2),make_Vec3f(+2,+2,+2));
  data.g_instance[1] = createInstance(data.g_scene,data.g_scene1,1,make_Vec3f(-2,-2,-2),make_Vec3f(+2,+2,+2));
  data.g_instance[2] = createInstance(data.g_scene,data.g_scene2,2,make_Vec3f(-2,-2,-2),make_Vec3f(+2,+2,+2));
  data.g_instance[3] = createInstance(data.g_scene,data.g_scene2,3,make_Vec3f(-2,-2,-2),make_Vec3f(+2,+2,+2));
  createGroundPlane(data.g_scene);
  rtcCommitScene(data.g_scene);

  /* set all colors */
  data.colors[4*0+0] = make_Vec3f(0.25f, 0.00f, 0.00f);
  data.colors[4*0+1] = make_Vec3f(0.50f, 0.00f, 0.00f);
  data.colors[4*0+2] = make_Vec3f(0.75f, 0.00f, 0.00f);
  data.colors[4*0+3] = make_Vec3f(1.00f, 0.00f, 0.00f);

  data.colors[4*1+0] = make_Vec3f(0.00f, 0.25f, 0.00f);
  data.colors[4*1+1] = make_Vec3f(0.00f, 0.50f, 0.00f);
  data.colors[4*1+2] = make_Vec3f(0.00f, 0.75f, 0.00f);
  data.colors[4*1+3] = make_Vec3f(0.00f, 1.00f, 0.00f);

  data.colors[4*2+0] = make_Vec3f(0.00f, 0.00f, 0.25f);
  data.colors[4*2+1] = make_Vec3f(0.00f, 0.00f, 0.50f);
  data.colors[4*2+2] = make_Vec3f(0.00f, 0.00f, 0.75f);
  data.colors[4*2+3] = make_Vec3f(0.00f, 0.00f, 1.00f);

  data.colors[4*3+0] = make_Vec3f(0.25f, 0.25f, 0.00f);
  data.colors[4*3+1] = make_Vec3f(0.50f, 0.50f, 0.00f);
  data.colors[4*3+2] = make_Vec3f(0.75f, 0.75f, 0.00f);
  data.colors[4*3+3] = make_Vec3f(1.00f, 1.00f, 0.00f);

  data.colors[4*4+0] = make_Vec3f(1.0f, 1.0f, 1.0f);
  data.colors[4*4+1] = make_Vec3f(1.0f, 1.0f, 1.0f);
  data.colors[4*4+2] = make_Vec3f(1.0f, 1.0f, 1.0f);
  data.colors[4*4+3] = make_Vec3f(1.0f, 1.0f, 1.0f);
}

inline Vec3f face_forward(const Vec3f& dir, const Vec3f& _Ng) {
  const Vec3f Ng = _Ng;
  return dot(dir,Ng) < 0.0f ? Ng : neg(Ng);
}

/* task that renders a single screen tile */
Vec3f renderPixelStandard(const uniform TutorialData& data,
                          float x, float y, const uniform ISPCCamera& camera,
                          uniform RayStats& stats)
{
  /* initialize ray */
  Ray ray = make_Ray(make_Vec3f(camera.xfm.p), 
                     make_Vec3f(normalize(x*camera.xfm.l.vx + y*camera.xfm.l.vy + camera.xfm.l.vz)), 
                     0.0f, inf, 0.0f, -1,
                     RTC_INVALID_GEOMETRY_ID, RTC_INVALID_GEOMETRY_ID);

  /* intersect ray with scene */
  uniform RTCIntersectArguments iargs;
  rtcInitIntersectArguments(&iargs);
#if USE_ARGUMENT_CALLBACKS
  iargs.filter = contextFilterFunction;
#endif
#if USE_ARGUMENT_CALLBACKS
  iargs.intersect = contextIntersectFunc;
#endif
  iargs.feature_mask = (uniform RTCFeatureFlags) (FEATURE_MASK);
  
  rtcIntersectV(data.g_scene,RTCRayHit_(ray),&iargs);
  RayStats_addRay(stats);

  /* shade pixels */
  Vec3f color = make_Vec3f(0.0f);
  if (ray.geomID != RTC_INVALID_GEOMETRY_ID)
  {
    /* calculate shading normal in world space */
    Vec3f Ns = ray.Ng;

    if (ray.instID[0] != RTC_INVALID_GEOMETRY_ID) {
      Ns = xfmVector(data.g_instance[ray.instID[0]]->normal2world,make_Vec3f(Ns));
    }
    Ns = face_forward(ray.dir,normalize(Ns));

    /* calculate diffuse color of geometries */
    Vec3f diffuse = make_Vec3f(0.0f);
    if      (ray.instID[0] ==  0) diffuse = data.colors[4*ray.instID[0]+ray.primID];
    else if (ray.instID[0] == -1) diffuse = data.colors[4*4+ray.primID];
    else                          diffuse = data.colors[4*ray.instID[0]+ray.geomID];
    color = color + diffuse*0.5f;

    /* initialize shadow ray */
    Vec3f lightDir = normalize(make_Vec3f(-1,-1,-1));
    Ray shadow = make_Ray(ray.org + 0.999f*ray.tfar*ray.dir, neg(lightDir), 0.001f, inf);

    /* trace shadow ray */
    uniform RTCOccludedArguments sargs;
    rtcInitOccludedArguments(&sargs);
#if USE_ARGUMENT_CALLBACKS
    sargs.filter = contextFilterFunction;
#endif
#if USE_ARGUMENT_CALLBACKS
    sargs.occluded = contextOccludedFunc;
#endif
    sargs.feature_mask = (uniform RTCFeatureFlags) (FEATURE_MASK);
    
    rtcOccludedV(data.g_scene,RTCRay_(shadow),&sargs);
    RayStats_addShadowRay(stats);

    /* add light contribution */
    if (shadow.tfar >= 0.0f)
      color = color + diffuse*clamp(-dot(lightDir,Ns),0.0f,1.0f);
  }
  return color;
}

void renderPixelStandard(const uniform TutorialData& data,
                         int x, int y, 
                         uniform int* uniform pixels,
                         const uniform unsigned int width,
                         const uniform unsigned int height,
                         const float time,
                         const uniform ISPCCamera& camera, uniform RayStats& stats)
{
  Vec3f color = renderPixelStandard(data,x,y,camera,stats);
  
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
  /* render all pixels */
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
  uniform float t0 = 0.7f*time;
  uniform float t1 = 1.5f*time;

  /* rotate instances around themselves */
  uniform LinearSpace3f xfm;
  xfm.vx = make_Vec3f(cos(t1),0,sin(t1));
  xfm.vy = make_Vec3f(0,1,0);
  xfm.vz = make_Vec3f(-sin(t1),0,cos(t1));

  /* calculate transformations to move instances in circles */
  data.g_instance[0]->local2world = make_AffineSpace3f(xfm,2.2f*make_Vec3f(+cos(t0),0.0f,+sin(t0)));
  data.g_instance[1]->local2world = make_AffineSpace3f(xfm,2.2f*make_Vec3f(-cos(t0),0.0f,-sin(t0)));
  data.g_instance[2]->local2world = make_AffineSpace3f(xfm,2.2f*make_Vec3f(-sin(t0),0.0f,+cos(t0)));
  data.g_instance[3]->local2world = make_AffineSpace3f(xfm,2.2f*make_Vec3f(+sin(t0),0.0f,-cos(t0)));

  /* update scene */
  updateInstance(data.g_scene,data.g_instance[0]);
  updateInstance(data.g_scene,data.g_instance[1]);
  updateInstance(data.g_scene,data.g_instance[2]);
  updateInstance(data.g_scene,data.g_instance[3]);
  rtcCommitScene (data.g_scene);
}

/* called by the C++ code for cleanup */
export void device_cleanup ()
{
  TutorialData_Destructor(&data);
}
