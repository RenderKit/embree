// Copyright 2009-2021 Intel Corporation
// SPDX-License-Identifier: Apache-2.0

#include "user_geometry_device.h"

namespace embree {

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

RTCScene g_scene = nullptr;
TutorialData data;

const int numPhi = 5;
const int numTheta = 2*numPhi;

RTC_SYCL_INDIRECTLY_CALLABLE void contextFilterFunction(const RTCFilterFunctionNArguments* args);

RTCIntersectFunctionN instanceIntersectFuncPtr = nullptr;
RTCOccludedFunctionN instanceOccludedFuncPtr = nullptr;
RTCIntersectFunctionN sphereIntersectFuncPtr = nullptr;
RTCOccludedFunctionN sphereOccludedFuncPtr = nullptr;
RTCFilterFunctionN sphereFilterFuncPtr = nullptr;

inline void pushInstanceId(RTCRayQueryContext* ctx, unsigned int id)
{
#if RTC_MAX_INSTANCE_LEVEL_COUNT > 1
  ctx->instID[ctx->instStackSize++] = id;
#else
  ctx->instID[0] = id;
#endif
}

inline void popInstanceId(RTCRayQueryContext* ctx)
{
#if RTC_MAX_INSTANCE_LEVEL_COUNT > 1
  ctx->instID[--ctx->instStackSize] = RTC_INVALID_GEOMETRY_ID;
#else
  ctx->instID[0] = RTC_INVALID_GEOMETRY_ID;
#endif
}

inline void copyInstanceIdStack(const RTCRayQueryContext* ctx, unsigned* tgt)
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

void instanceBoundsFunc(const struct RTCBoundsFunctionArguments* args)
{
  const Instance* instance = (const Instance*) args->geometryUserPtr;
  RTCBounds* bounds_o = args->bounds_o;
  Vec3fa l = instance->lower;
  Vec3fa u = instance->upper;
  Vec3fa p000 = xfmPoint(instance->local2world,Vec3fa(l.x,l.y,l.z));
  Vec3fa p001 = xfmPoint(instance->local2world,Vec3fa(l.x,l.y,u.z));
  Vec3fa p010 = xfmPoint(instance->local2world,Vec3fa(l.x,u.y,l.z));
  Vec3fa p011 = xfmPoint(instance->local2world,Vec3fa(l.x,u.y,u.z));
  Vec3fa p100 = xfmPoint(instance->local2world,Vec3fa(u.x,l.y,l.z));
  Vec3fa p101 = xfmPoint(instance->local2world,Vec3fa(u.x,l.y,u.z));
  Vec3fa p110 = xfmPoint(instance->local2world,Vec3fa(u.x,u.y,l.z));
  Vec3fa p111 = xfmPoint(instance->local2world,Vec3fa(u.x,u.y,u.z));
  Vec3fa lower = min(min(min(p000,p001),min(p010,p011)),min(min(p100,p101),min(p110,p111)));
  Vec3fa upper = max(max(max(p000,p001),max(p010,p011)),max(max(p100,p101),max(p110,p111)));
  bounds_o->lower_x = lower.x;
  bounds_o->lower_y = lower.y;
  bounds_o->lower_z = lower.z;
  bounds_o->upper_x = upper.x;
  bounds_o->upper_y = upper.y;
  bounds_o->upper_z = upper.z;
}

RTC_SYCL_INDIRECTLY_CALLABLE void instanceIntersectFunc(const RTCIntersectFunctionNArguments* args)
{
  const int* valid = args->valid;
  void* ptr  = args->geometryUserPtr;
  RTCRayHitN* rays = (RTCRayHitN*)args->rayhit;
  
  assert(args->N == 1);
  if (!valid[0])
    return;
  
  Ray *ray = (Ray*)rays;
  const Instance* instance = (const Instance*)ptr;
  const Vec3ff ray_org = ray->org;
  const Vec3ff ray_dir = ray->dir;
  const float ray_tnear = ray->tnear();
  const float ray_tfar  = ray->tfar;

#if 0

  RTCRayQueryContext* context = args->context;
  ray->org = Vec3ff(xfmPoint (instance->world2local,ray_org));
  ray->dir = Vec3ff(xfmVector(instance->world2local,ray_dir));
  ray->tnear() = ray_tnear;
  ray->tfar  = ray_tfar;
  pushInstanceId(context, args->geomID);
  RTCIntersectArguments args;
  rtcInitIntersectArguments(&args);
  args.context = context;
  rtcIntersect1(instance->object,RTCRayHit_(*ray),&args);
  popInstanceId(context);
  const float updated_tfar = ray->tfar;
  ray->org = ray_org;
  ray->dir = ray_dir;
  ray->tfar = updated_tfar;

#else
  
  RTCRay xray;
  const Vec3fa org = xfmPoint (instance->world2local,ray_org);
  const Vec3fa dir = xfmVector(instance->world2local,ray_dir);
  xray.org_x = org.x; xray.org_y = org.y; xray.org_z = org.z;
  xray.dir_x = dir.x; xray.dir_y = dir.y; xray.dir_z = dir.z;
  xray.tnear = ray_tnear;
  xray.tfar  = ray_tfar;
  xray.time = 0.0f;
  xray.mask = -1;
  xray.id = 0;
  xray.flags = 0;
  
  rtcForwardIntersect1(args,instance->object,&xray,args->geomID);
  
#endif
}

RTC_SYCL_INDIRECTLY_CALLABLE void instanceOccludedFunc(const RTCOccludedFunctionNArguments* args)
{
  const int* valid = args->valid;
  void* ptr  = args->geometryUserPtr;
  RTCRayHitN* rays = (RTCRayHitN*)args->ray;
  assert(args->N == 1);
  if (!valid[0])
    return;
  
  Ray *ray = (Ray*)rays;
  const Instance* instance = (const Instance*)ptr;
  const Vec3ff ray_org = ray->org;
  const Vec3ff ray_dir = ray->dir;
  const float ray_tnear = ray->tnear();
  const float ray_tfar  = ray->tfar;

#if 0

  RTCRayQueryContext* context = args->context;
  ray->org    = Vec3ff(xfmPoint (instance->world2local,ray_org));
  ray->dir    = Vec3ff(xfmVector(instance->world2local,ray_dir));
  ray->tnear()  = ray_tnear;
  ray->tfar   = ray_tfar;
  pushInstanceId(context, args->geomID);
  RTCOccludedArguments args;
  rtcInitOccludedArguments(&args);
  args.context = context;
  rtcOccluded1(instance->object,RTCRay_(*ray),&args);
  popInstanceId(context);
  const float updated_tfar = ray->tfar;
  ray->org    = ray_org;
  ray->dir    = ray_dir;
  ray->tnear()  = ray_tnear;
  ray->tfar   = updated_tfar;

#else

  RTCRay xray;
  const Vec3fa org = xfmPoint (instance->world2local,ray_org);
  const Vec3fa dir = xfmVector(instance->world2local,ray_dir);
  xray.org_x = org.x; xray.org_y = org.y; xray.org_z = org.z;
  xray.dir_x = dir.x; xray.dir_y = dir.y; xray.dir_z = dir.z;
  xray.tnear = ray_tnear;
  xray.tfar  = ray_tfar;
  xray.time = 0.0f;
  xray.mask = -1;
  xray.id = 0;
  xray.flags = 0;
  
  rtcForwardOccluded1(args,instance->object,&xray,args->geomID);
  
#endif
}

Instance* createInstance (RTCScene scene, RTCScene object, int geomID, const Vec3fa& lower, const Vec3fa& upper)
{
#if !ENABLE_NATIVE_INSTANCING
  Instance* instance = (Instance*) alignedUSMMalloc(sizeof(Instance),16);
  instance->type = USER_GEOMETRY_INSTANCE;
  instance->object = object;
  instance->lower = lower;
  instance->upper = upper;
  instance->local2world.l.vx = Vec3fa(1,0,0);
  instance->local2world.l.vy = Vec3fa(0,1,0);
  instance->local2world.l.vz = Vec3fa(0,0,1);
  instance->local2world.p    = Vec3fa(0,0,0);
  instance->geometry = rtcNewGeometry(g_device, RTC_GEOMETRY_TYPE_USER);
  rtcSetGeometryUserPrimitiveCount(instance->geometry,1);
  rtcSetGeometryUserData(instance->geometry,instance);
  rtcSetGeometryBoundsFunction(instance->geometry,instanceBoundsFunc,nullptr);
  rtcSetGeometryIntersectFunction(instance->geometry,instanceIntersectFuncPtr);
  rtcSetGeometryOccludedFunction (instance->geometry,instanceOccludedFuncPtr);
  rtcCommitGeometry(instance->geometry);
  rtcAttachGeometry(scene,instance->geometry);
  rtcReleaseGeometry(instance->geometry);
  return instance;
#else
  Instance* instance = (Instance*) alignedUSMMalloc(sizeof(Instance),16);
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

void updateInstance (RTCScene scene, Instance* instance)
{
#if ENABLE_NATIVE_INSTANCING
  rtcSetGeometryTransform(instance->geometry,0,RTC_FORMAT_FLOAT4X4_COLUMN_MAJOR,(float*)&instance->local2world);
#endif
  
  instance->world2local = rcp(instance->local2world);
  instance->normal2world = transposed(rcp(instance->local2world.l));
  rtcCommitGeometry(instance->geometry);
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
  
  assert(args->N == 1);
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
  copyInstanceIdStack(args->context, potentialHit.instID);
  potentialHit.geomID = sphere.geomID;
  potentialHit.primID = primID;

  if ((ray->tnear() < t0) & (t0 < ray->tfar))
  {
    int imask;
    bool mask = 1;
    {
      imask = mask ? -1 : 0;
    }
    
    const Vec3fa Ng = ray->org+t0*ray->dir-sphere.p;
    potentialHit.Ng_x = Ng.x;
    potentialHit.Ng_y = Ng.y;
    potentialHit.Ng_z = Ng.z;

    RTCFilterFunctionNArguments fargs;
    fargs.valid = (int*)&imask;
    fargs.geometryUserPtr = ptr;
    fargs.context = args->context;
    fargs.ray = (RTCRayN *)args->rayhit;
    fargs.hit = (RTCHitN*)&potentialHit;
    fargs.N = 1;

    const float old_t = ray->tfar;
    ray->tfar = t0;
#if USE_ARGUMENT_CALLBACKS
    contextFilterFunction(&fargs);
#else
    rtcInvokeIntersectFilterFromGeometry(args,&fargs);
#endif

    if (imask == -1) {
      *hit = potentialHit;
      valid[0] = -1;
    }
    else
      ray->tfar = old_t;
  }

  if ((ray->tnear() < t1) & (t1 < ray->tfar))
  {
    int imask;
    bool mask = 1;
    {
      imask = mask ? -1 : 0;
    }
    
    const Vec3fa Ng = ray->org+t1*ray->dir-sphere.p;
    potentialHit.Ng_x = Ng.x;
    potentialHit.Ng_y = Ng.y;
    potentialHit.Ng_z = Ng.z;

    RTCFilterFunctionNArguments fargs;
    fargs.valid = (int*)&imask;
    fargs.geometryUserPtr = ptr;
    fargs.context = args->context;
    fargs.ray = (RTCRayN *)args->rayhit;
    fargs.hit = (RTCHitN*)&potentialHit;
    fargs.N = 1;

    const float old_t = ray->tfar;
    ray->tfar = t1;
#if USE_ARGUMENT_CALLBACKS
    contextFilterFunction(&fargs);
#else
    rtcInvokeIntersectFilterFromGeometry(args,&fargs);
#endif

    if (imask == -1) {
      *hit = potentialHit;
      valid[0] = -1;
    }
    else
      ray->tfar = old_t;
  }
}

RTC_SYCL_INDIRECTLY_CALLABLE void sphereOccludedFunc(const RTCOccludedFunctionNArguments* args)
{
  int* valid = args->valid;
  void* ptr  = args->geometryUserPtr;
  Ray *ray = (Ray*)args->ray;
  unsigned int primID = args->primID;
  
  assert(args->N == 1);
  const Sphere* spheres = (const Sphere*) ptr;
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
  copyInstanceIdStack(args->context, potentialHit.instID);
  potentialHit.geomID = sphere.geomID;
  potentialHit.primID = primID;
  if ((ray->tnear() < t0) & (t0 < ray->tfar))
  {
    int imask;
    bool mask = 1;
    {
      imask = mask ? -1 : 0;
    }
    
    const Vec3fa Ng = ray->org+t0*ray->dir-sphere.p;
    potentialHit.Ng_x = Ng.x;
    potentialHit.Ng_y = Ng.y;
    potentialHit.Ng_z = Ng.z;

    RTCFilterFunctionNArguments fargs;
    fargs.valid = (int*)&imask;
    fargs.geometryUserPtr = ptr;
    fargs.context = args->context;
    fargs.ray = args->ray;
    fargs.hit = (RTCHitN*)&potentialHit;
    fargs.N = 1;

    const float old_t = ray->tfar;
    ray->tfar = t0;
#if USE_ARGUMENT_CALLBACKS
    contextFilterFunction(&fargs);
#else
    rtcInvokeOccludedFilterFromGeometry(args,&fargs);
#endif
    
    if (imask == -1) {
      ray->tfar = neg_inf;
      valid[0] = -1;
    }
    else
      ray->tfar = old_t;
  }

  if ((ray->tnear() < t1) & (t1 < ray->tfar))
  {
    int imask;
    bool mask = 1;
    {
      imask = mask ? -1 : 0;
    }
    
    const Vec3fa Ng = ray->org+t1*ray->dir-sphere.p;
    potentialHit.Ng_x = Ng.x;
    potentialHit.Ng_y = Ng.y;
    potentialHit.Ng_z = Ng.z;

    RTCFilterFunctionNArguments fargs;
    fargs.valid = (int*)&imask;
    fargs.geometryUserPtr = ptr;
    fargs.context = args->context;
    fargs.ray = args->ray;
    fargs.hit = (RTCHitN*)&potentialHit;
    fargs.N = 1;

    const float old_t = ray->tfar;
    ray->tfar = t1;
#if USE_ARGUMENT_CALLBACKS
    contextFilterFunction(&fargs);
#else
    rtcInvokeOccludedFilterFromGeometry(args,&fargs);
#endif

    if (imask == -1) {
      ray->tfar = neg_inf;
      valid[0] = -1;
    }
    else
      ray->tfar = old_t;
  }
}

RTC_SYCL_INDIRECTLY_CALLABLE void contextIntersectFunc(const RTCIntersectFunctionNArguments* args)
{
  UserGeometryType* type = (UserGeometryType*) args->geometryUserPtr;
  if (*type == USER_GEOMETRY_INSTANCE) instanceIntersectFunc(args);
  else                                 sphereIntersectFunc(args);
}

RTC_SYCL_INDIRECTLY_CALLABLE void contextOccludedFunc(const RTCOccludedFunctionNArguments* args)
{
  UserGeometryType* type = (UserGeometryType*) args->geometryUserPtr;
  if (*type == USER_GEOMETRY_INSTANCE) instanceOccludedFunc(args);
  else                                 sphereOccludedFunc(args);
}

/* intersection filter function */

RTC_SYCL_INDIRECTLY_CALLABLE void sphereFilterFunction(const RTCFilterFunctionNArguments* args)
{
  int* valid = args->valid;
  const RayQueryContext* context = (const RayQueryContext*) args->context;
  struct Ray* ray    = (struct Ray*)args->ray;
  //struct RTCHit* hit = (struct RTCHit*)args->hit;
  const unsigned int N = args->N;
  assert(N == 1);
  _unused(N);

  /* avoid crashing when debug visualizations are used */
  if (context == nullptr)
    return;

  /* ignore inactive rays */
  if (valid[0] != -1) return;
  
  /* carve out parts of the sphere */
  const Vec3fa h = ray->org+ray->dir*ray->tfar;
  float v = abs(sin(10.0f*h.x)*cos(10.0f*h.y)*sin(10.0f*h.z));
  float T = clamp((v-0.1f)*3.0f,0.0f,1.0f);

  /* reject some hits */
  if (T < 0.5f) valid[0] = 0;
}

RTC_SYCL_INDIRECTLY_CALLABLE void contextFilterFunction(const RTCFilterFunctionNArguments* args)
{
  int* valid = args->valid;
  if (!valid[0]) return;
  
  RTCHit* potential_hit = (RTCHit*) args->hit;
  if (potential_hit->instID[0] == 0)
    sphereFilterFunction(args);
}

Sphere* createAnalyticalSphere (RTCScene scene, const Vec3fa& p, float r)
{
  RTCGeometry geom = rtcNewGeometry(g_device, RTC_GEOMETRY_TYPE_USER);
  Sphere* sphere = (Sphere*) alignedUSMMalloc(sizeof(Sphere),16);
  sphere->type = USER_GEOMETRY_SPHERE;
  sphere->p = p;
  sphere->r = r;
  sphere->geometry = geom;
  sphere->geomID = rtcAttachGeometry(scene,geom);
  rtcSetGeometryUserPrimitiveCount(geom,1);
  rtcSetGeometryUserData(geom,sphere);
  rtcSetGeometryBoundsFunction(geom,sphereBoundsFunc,nullptr);
#if !USE_ARGUMENT_CALLBACKS
    rtcSetGeometryIntersectFunction(geom,sphereIntersectFuncPtr);
    rtcSetGeometryOccludedFunction (geom,sphereOccludedFuncPtr);
#endif
  rtcCommitGeometry(geom);
  rtcReleaseGeometry(geom);
  return sphere;
}

Sphere* createAnalyticalSpheres (RTCScene scene, unsigned int N)
{
  RTCGeometry geom = rtcNewGeometry(g_device, RTC_GEOMETRY_TYPE_USER);
  Sphere* spheres = (Sphere*) alignedUSMMalloc((N)*sizeof(Sphere),16);
  unsigned int geomID = rtcAttachGeometry(scene,geom);
  for (unsigned int i=0; i<N; i++) {
    spheres[i].type = USER_GEOMETRY_SPHERE;
    spheres[i].geometry = geom;
    spheres[i].geomID = geomID;
  }
  rtcSetGeometryUserPrimitiveCount(geom,N);
  rtcSetGeometryUserData(geom,spheres);
  rtcSetGeometryBoundsFunction(geom,sphereBoundsFunc,nullptr);
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

unsigned int createTriangulatedSphere (RTCScene scene, const Vec3fa& p, float r)
{
  /* create triangle mesh */
  RTCGeometry geom = rtcNewGeometry (g_device, RTC_GEOMETRY_TYPE_TRIANGLE);

  /* map triangle and vertex buffers */
  Vertex* vertices = (Vertex*) rtcSetNewGeometryBuffer(geom,RTC_BUFFER_TYPE_VERTEX,0,RTC_FORMAT_FLOAT3,sizeof(Vertex),numTheta*(numPhi+1));
  Triangle* triangles = (Triangle*) rtcSetNewGeometryBuffer(geom,RTC_BUFFER_TYPE_INDEX,0,RTC_FORMAT_UINT3,sizeof(Triangle),2*numTheta*(numPhi-1));

  /* create sphere */
  int tri = 0;
  const float rcpNumTheta = rcp((float)numTheta);
  const float rcpNumPhi   = rcp((float)numPhi);
  for (int phi=0; phi<=numPhi; phi++)
  {
    for (int theta=0; theta<numTheta; theta++)
    {
      const float phif   = phi*float(pi)*rcpNumPhi;
      const float thetaf = theta*2.0f*float(pi)*rcpNumTheta;

      Vertex& v = vertices[phi*numTheta+theta];
      v.x = p.x + r*sin(phif)*sin(thetaf);
      v.y = p.y + r*cos(phif);
      v.z = p.z + r*sin(phif)*cos(thetaf);
    }
    if (phi == 0) continue;

    for (int theta=1; theta<=numTheta; theta++)
    {
      int p00 = (phi-1)*numTheta+theta-1;
      int p01 = (phi-1)*numTheta+theta%numTheta;
      int p10 = phi*numTheta+theta-1;
      int p11 = phi*numTheta+theta%numTheta;

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
  unsigned int geomID = rtcAttachGeometry(scene,geom);
  rtcReleaseGeometry(geom);
  return geomID;
}

/* creates a ground plane */
unsigned int createGroundPlane (RTCScene scene)
{
  /* create a triangulated plane with 2 triangles and 4 vertices */
  RTCGeometry geom = rtcNewGeometry (g_device, RTC_GEOMETRY_TYPE_TRIANGLE);

  /* set vertices */
  Vertex* vertices = (Vertex*) rtcSetNewGeometryBuffer(geom,RTC_BUFFER_TYPE_VERTEX,0,RTC_FORMAT_FLOAT3,sizeof(Vertex),4);
  vertices[0].x = -10; vertices[0].y = -2; vertices[0].z = -10;
  vertices[1].x = -10; vertices[1].y = -2; vertices[1].z = +10;
  vertices[2].x = +10; vertices[2].y = -2; vertices[2].z = -10;
  vertices[3].x = +10; vertices[3].y = -2; vertices[3].z = +10;

  /* set triangles */
  Triangle* triangles = (Triangle*) rtcSetNewGeometryBuffer(geom,RTC_BUFFER_TYPE_INDEX,0,RTC_FORMAT_UINT3,sizeof(Triangle),2);
  triangles[0].v0 = 0; triangles[0].v1 = 2; triangles[0].v2 = 1;
  triangles[1].v0 = 1; triangles[1].v1 = 2; triangles[1].v2 = 3;

  rtcCommitGeometry(geom);
  unsigned int geomID = rtcAttachGeometry(scene,geom);
  rtcReleaseGeometry(geom);
  return geomID;
}

/* called by the C++ code for initialization */
extern "C" void device_init (char* cfg)
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
  data.g_spheres[0].p = Vec3fa( 0, 0,+1); data.g_spheres[0].r = 0.5f;
  data.g_spheres[1].p = Vec3fa(+1, 0, 0); data.g_spheres[1].r = 0.5f;
  data.g_spheres[2].p = Vec3fa( 0, 0,-1); data.g_spheres[2].r = 0.5f;
  data.g_spheres[3].p = Vec3fa(-1, 0, 0); data.g_spheres[3].r = 0.5f;
  rtcCommitScene(data.g_scene0);

  /* create scene with 4 triangulated spheres */
  data.g_scene1 = rtcNewScene(g_device);
  createTriangulatedSphere(data.g_scene1,Vec3fa( 0, 0,+1),0.5f);
  createTriangulatedSphere(data.g_scene1,Vec3fa(+1, 0, 0),0.5f);
  createTriangulatedSphere(data.g_scene1,Vec3fa( 0, 0,-1),0.5f);
  createTriangulatedSphere(data.g_scene1,Vec3fa(-1, 0, 0),0.5f);
  rtcCommitScene(data.g_scene1);

  /* create scene with 2 triangulated and 2 analytical spheres */
  data.g_scene2 = rtcNewScene(g_device);
  createTriangulatedSphere(data.g_scene2,Vec3fa( 0, 0,+1),0.5f);
  data.g_sphere0 = createAnalyticalSphere  (data.g_scene2,Vec3fa(+1, 0, 0),0.5f);
  createTriangulatedSphere(data.g_scene2,Vec3fa( 0, 0,-1),0.5f);
  data.g_sphere1 = createAnalyticalSphere  (data.g_scene2,Vec3fa(-1, 0, 0),0.5f);
  rtcCommitScene(data.g_scene2);

  /* instantiate geometry */
  data.g_instance[0] = createInstance(data.g_scene,data.g_scene0,0,Vec3fa(-2,-2,-2),Vec3fa(+2,+2,+2));
  data.g_instance[1] = createInstance(data.g_scene,data.g_scene1,1,Vec3fa(-2,-2,-2),Vec3fa(+2,+2,+2));
  data.g_instance[2] = createInstance(data.g_scene,data.g_scene2,2,Vec3fa(-2,-2,-2),Vec3fa(+2,+2,+2));
  data.g_instance[3] = createInstance(data.g_scene,data.g_scene2,3,Vec3fa(-2,-2,-2),Vec3fa(+2,+2,+2));
  createGroundPlane(data.g_scene);
  rtcCommitScene(data.g_scene);

  /* set all colors */
  data.colors[4*0+0] = Vec3fa(0.25f, 0.00f, 0.00f);
  data.colors[4*0+1] = Vec3fa(0.50f, 0.00f, 0.00f);
  data.colors[4*0+2] = Vec3fa(0.75f, 0.00f, 0.00f);
  data.colors[4*0+3] = Vec3fa(1.00f, 0.00f, 0.00f);

  data.colors[4*1+0] = Vec3fa(0.00f, 0.25f, 0.00f);
  data.colors[4*1+1] = Vec3fa(0.00f, 0.50f, 0.00f);
  data.colors[4*1+2] = Vec3fa(0.00f, 0.75f, 0.00f);
  data.colors[4*1+3] = Vec3fa(0.00f, 1.00f, 0.00f);

  data.colors[4*2+0] = Vec3fa(0.00f, 0.00f, 0.25f);
  data.colors[4*2+1] = Vec3fa(0.00f, 0.00f, 0.50f);
  data.colors[4*2+2] = Vec3fa(0.00f, 0.00f, 0.75f);
  data.colors[4*2+3] = Vec3fa(0.00f, 0.00f, 1.00f);

  data.colors[4*3+0] = Vec3fa(0.25f, 0.25f, 0.00f);
  data.colors[4*3+1] = Vec3fa(0.50f, 0.50f, 0.00f);
  data.colors[4*3+2] = Vec3fa(0.75f, 0.75f, 0.00f);
  data.colors[4*3+3] = Vec3fa(1.00f, 1.00f, 0.00f);

  data.colors[4*4+0] = Vec3fa(1.0f, 1.0f, 1.0f);
  data.colors[4*4+1] = Vec3fa(1.0f, 1.0f, 1.0f);
  data.colors[4*4+2] = Vec3fa(1.0f, 1.0f, 1.0f);
  data.colors[4*4+3] = Vec3fa(1.0f, 1.0f, 1.0f);
}

inline Vec3fa face_forward(const Vec3fa& dir, const Vec3fa& _Ng) {
  const Vec3fa Ng = _Ng;
  return dot(dir,Ng) < 0.0f ? Ng : neg(Ng);
}

/* task that renders a single screen tile */
Vec3fa renderPixelStandard(const TutorialData& data,
                          float x, float y, const ISPCCamera& camera,
                          RayStats& stats)
{
  /* initialize ray */
  Ray ray(Vec3fa(camera.xfm.p), 
                     Vec3fa(normalize(x*camera.xfm.l.vx + y*camera.xfm.l.vy + camera.xfm.l.vz)), 
                     0.0f, inf, 0.0f, -1,
                     RTC_INVALID_GEOMETRY_ID, RTC_INVALID_GEOMETRY_ID);

  /* intersect ray with scene */
  RTCIntersectArguments iargs;
  rtcInitIntersectArguments(&iargs);
#if USE_ARGUMENT_CALLBACKS
  iargs.filter = contextFilterFunction;
#endif
#if USE_ARGUMENT_CALLBACKS
  iargs.intersect = contextIntersectFunc;
#endif
  iargs.feature_mask = (RTCFeatureFlags) (FEATURE_MASK);
  
  rtcIntersect1(data.g_scene,RTCRayHit_(ray),&iargs);
  RayStats_addRay(stats);

  /* shade pixels */
  Vec3fa color = Vec3fa(0.0f);
  if (ray.geomID != RTC_INVALID_GEOMETRY_ID)
  {
    /* calculate shading normal in world space */
    Vec3fa Ns = ray.Ng;

    if (ray.instID[0] != RTC_INVALID_GEOMETRY_ID) {
      Ns = xfmVector(data.g_instance[ray.instID[0]]->normal2world,Vec3fa(Ns));
    }
    Ns = face_forward(ray.dir,normalize(Ns));

    /* calculate diffuse color of geometries */
    Vec3fa diffuse = Vec3fa(0.0f);
    if      (ray.instID[0] ==  0) diffuse = data.colors[4*ray.instID[0]+ray.primID];
    else if (ray.instID[0] == -1) diffuse = data.colors[4*4+ray.primID];
    else                          diffuse = data.colors[4*ray.instID[0]+ray.geomID];
    color = color + diffuse*0.5f;

    /* initialize shadow ray */
    Vec3fa lightDir = normalize(Vec3fa(-1,-1,-1));
    Ray shadow(ray.org + 0.999f*ray.tfar*ray.dir, neg(lightDir), 0.001f, inf);

    /* trace shadow ray */
    RTCOccludedArguments sargs;
    rtcInitOccludedArguments(&sargs);
#if USE_ARGUMENT_CALLBACKS
    sargs.filter = contextFilterFunction;
#endif
#if USE_ARGUMENT_CALLBACKS
    sargs.occluded = contextOccludedFunc;
#endif
    sargs.feature_mask = (RTCFeatureFlags) (FEATURE_MASK);
    
    rtcOccluded1(data.g_scene,RTCRay_(shadow),&sargs);
    RayStats_addShadowRay(stats);

    /* add light contribution */
    if (shadow.tfar >= 0.0f)
      color = color + diffuse*clamp(-dot(lightDir,Ns),0.0f,1.0f);
  }
  return color;
}

void renderPixelStandard(const TutorialData& data,
                         int x, int y, 
                         int* pixels,
                         const unsigned int width,
                         const unsigned int height,
                         const float time,
                         const ISPCCamera& camera, RayStats& stats)
{
  Vec3fa color = renderPixelStandard(data,x,y,camera,stats);
  
  /* write color to framebuffer */
  unsigned int r = (unsigned int) (255.0f * clamp(color.x,0.0f,1.0f));
  unsigned int g = (unsigned int) (255.0f * clamp(color.y,0.0f,1.0f));
  unsigned int b = (unsigned int) (255.0f * clamp(color.z,0.0f,1.0f));
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
  /* render all pixels */
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
  float t0 = 0.7f*time;
  float t1 = 1.5f*time;

  /* rotate instances around themselves */
  LinearSpace3fa xfm;
  xfm.vx = Vec3fa(cos(t1),0,sin(t1));
  xfm.vy = Vec3fa(0,1,0);
  xfm.vz = Vec3fa(-sin(t1),0,cos(t1));

  /* calculate transformations to move instances in circles */
  data.g_instance[0]->local2world = AffineSpace3fa(xfm,2.2f*Vec3fa(+cos(t0),0.0f,+sin(t0)));
  data.g_instance[1]->local2world = AffineSpace3fa(xfm,2.2f*Vec3fa(-cos(t0),0.0f,-sin(t0)));
  data.g_instance[2]->local2world = AffineSpace3fa(xfm,2.2f*Vec3fa(-sin(t0),0.0f,+cos(t0)));
  data.g_instance[3]->local2world = AffineSpace3fa(xfm,2.2f*Vec3fa(+sin(t0),0.0f,-cos(t0)));

  /* update scene */
  updateInstance(data.g_scene,data.g_instance[0]);
  updateInstance(data.g_scene,data.g_instance[1]);
  updateInstance(data.g_scene,data.g_instance[2]);
  updateInstance(data.g_scene,data.g_instance[3]);
  rtcCommitScene (data.g_scene);
}

/* called by the C++ code for cleanup */
extern "C" void device_cleanup ()
{
  TutorialData_Destructor(&data);
}

} // namespace embree
