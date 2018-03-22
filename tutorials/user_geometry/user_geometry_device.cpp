// ======================================================================== //
// Copyright 2009-2018 Intel Corporation                                    //
//                                                                          //
// Licensed under the Apache License, Version 2.0 (the "License");          //
// you may not use this file except in compliance with the License.         //
// You may obtain a copy of the License at                                  //
//                                                                          //
//     http://www.apache.org/licenses/LICENSE-2.0                           //
//                                                                          //
// Unless required by applicable law or agreed to in writing, software      //
// distributed under the License is distributed on an "AS IS" BASIS,        //
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. //
// See the License for the specific language governing permissions and      //
// limitations under the License.                                           //
// ======================================================================== //

#include "../common/tutorial/tutorial_device.h"

namespace embree {

const int numPhi = 5;
const int numTheta = 2*numPhi;

void renderTileStandardStream(int taskIndex,
                              int threadIndex,
                              int* pixels,
                              const unsigned int width,
                              const unsigned int height,
                              const float time,
                              const ISPCCamera& camera,
                              const int numTilesX,
                              const int numTilesY);

// ======================================================================== //
//                         User defined instancing                          //
// ======================================================================== //

struct Instance
{
  ALIGNED_STRUCT_(16)
  RTCGeometry geometry;
  RTCScene object;
  int userID;
  AffineSpace3fa local2world;
  AffineSpace3fa world2local;
  LinearSpace3fa normal2world;
  Vec3fa lower;
  Vec3fa upper;
};

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

void instanceIntersectFunc(const RTCIntersectFunctionNArguments* args)
{
  
  const int* valid = args->valid;
  void* ptr  = args->geometryUserPtr;
  RTCIntersectContext* context = args->context;
  RTCRayHitN* rays = (RTCRayHitN*)args->rayhit;
                                    
  assert(args->N == 1);
  if (!valid[0])
    return;
  
  Ray *ray = (Ray*)rays;
  const Instance* instance = (const Instance*)ptr;
  const Vec3fa ray_org = ray->org;
  const Vec3fa ray_dir = ray->dir;
  const float ray_tnear = ray->tnear();
  const float ray_tfar  = ray->tfar;
  ray->org = xfmPoint (instance->world2local,ray_org);
  ray->dir = xfmVector(instance->world2local,ray_dir);
  ray->tnear() = ray_tnear;
  ray->tfar  = ray_tfar;
  context->instID[0] = instance->userID;
  rtcIntersect1(instance->object,context,RTCRayHit_(*ray));
  context->instID[0] = -1;
  const float updated_tfar = ray->tfar;
  ray->org = ray_org;
  ray->dir = ray_dir;
  ray->tfar = updated_tfar;
}

void instanceOccludedFunc(const RTCOccludedFunctionNArguments* args)
{
  const int* valid = args->valid;
  void* ptr  = args->geometryUserPtr;
  RTCIntersectContext* context = args->context;
  RTCRayHitN* rays = (RTCRayHitN*)args->ray;
  assert(args->N == 1);
  if (!valid[0])
    return;
  
  Ray *ray = (Ray*)rays;
  const Instance* instance = (const Instance*)ptr;
  const Vec3fa ray_org = ray->org;
  const Vec3fa ray_dir = ray->dir;
  const float ray_tnear = ray->tnear();
  const float ray_tfar  = ray->tfar;
  ray->org    = xfmPoint (instance->world2local,ray_org);
  ray->dir    = xfmVector(instance->world2local,ray_dir);
  ray->tnear()  = ray_tnear;
  ray->tfar   = ray_tfar;
  context->instID[0] = instance->userID;
  rtcOccluded1(instance->object,context,RTCRay_(*ray));
  context->instID[0] = -1;
  const float updated_tfar = ray->tfar;
  ray->org    = ray_org;
  ray->dir    = ray_dir;
  ray->tnear()  = ray_tnear;
  ray->tfar   = updated_tfar;
}

void instanceIntersectFuncN(const RTCIntersectFunctionNArguments* args)
{
  /* avoid crashing when debug visualizations are used */
  if (args->context == nullptr)
    return;

  const int* valid = args->valid;
  void* ptr  = args->geometryUserPtr;
  RTCIntersectContext* context = args->context;
  unsigned int N = args->N;
  RTCRayHitN* rayhit = (RTCRayHitN*)args->rayhit;
  RTCRayN* rays = RTCRayHitN_RayN(rayhit,N);
  RTCHitN* hits = RTCRayHitN_HitN(rayhit,N);
  const Instance* instance = (const Instance*) ptr;
  
  /* iterate over all rays in ray packet */
  for (unsigned int ui=0; ui<N; ui+=1)
  {
    /* calculate loop and execution mask */
    unsigned int vi = ui+0;
    if (vi>=N) continue;

    /* ignore inactive rays */
    if (valid[vi] != -1) continue;

    /* create transformed ray */
    Ray ray;
    const Vec3fa ray_org = Vec3fa(RTCRayN_org_x(rays,N,ui),RTCRayN_org_y(rays,N,ui),RTCRayN_org_z(rays,N,ui));
    const Vec3fa ray_dir = Vec3fa(RTCRayN_dir_x(rays,N,ui),RTCRayN_dir_y(rays,N,ui),RTCRayN_dir_z(rays,N,ui));
    ray.org = xfmPoint (instance->world2local,ray_org);
    ray.dir = xfmVector(instance->world2local,ray_dir);
    bool mask = 1; {
      ray.tnear() = mask ? RTCRayN_tnear(rays,N,ui) : (float)(pos_inf);
      ray.tfar  = mask ? RTCRayN_tfar(rays,N,ui ) : (float)(neg_inf);
    }
    ray.time()  = RTCRayN_time(rays,N,ui);
    ray.mask  = RTCRayN_mask(rays,N,ui);
    ray.geomID = RTC_INVALID_GEOMETRY_ID;

    /* trace ray through object */
    context->instID[0] = instance->userID;
    rtcIntersect1(instance->object,context,RTCRayHit_(ray));
    context->instID[0] = -1;
    if (ray.geomID == RTC_INVALID_GEOMETRY_ID) continue;

    /* update hit */
    RTCRayN_tfar(rays,N,ui) = ray.tfar;
    rtcCopyHitToHitN(hits,RTCHit_(ray),N,ui);
  }
}

void instanceOccludedFuncN(const RTCOccludedFunctionNArguments* args)
{
  /* avoid crashing when debug visualizations are used */
  if (args->context == nullptr)
    return;
  
  const int* valid = args->valid;
  void* ptr  = args->geometryUserPtr;
  RTCIntersectContext* context = args->context;
  RTCRayN* rays = (RTCRayN*)args->ray;
  unsigned int N = args->N;
  const Instance* instance = (const Instance*) ptr;

  /* iterate over all rays in ray packet */
  for (unsigned int ui=0; ui<N; ui+=1)
  {
    /* calculate loop and execution mask */
    unsigned int vi = ui+0;
    if (vi>=N) continue;

    /* ignore inactive rays */
    if (valid[vi] != -1) continue;

    /* create transformed ray */
    Ray ray;
    const Vec3fa ray_org = Vec3fa(RTCRayN_org_x(rays,N,ui),RTCRayN_org_y(rays,N,ui),RTCRayN_org_z(rays,N,ui));
    const Vec3fa ray_dir = Vec3fa(RTCRayN_dir_x(rays,N,ui),RTCRayN_dir_y(rays,N,ui),RTCRayN_dir_z(rays,N,ui));
    ray.org = xfmPoint (instance->world2local,ray_org);
    ray.dir = xfmVector(instance->world2local,ray_dir);
    bool mask = 1; {
      ray.tnear() = mask ? RTCRayN_tnear(rays,N,ui) : (float)(pos_inf);
      ray.tfar  = mask ? RTCRayN_tfar(rays,N,ui)  : (float)(neg_inf);
    }
    ray.time()  = RTCRayN_time(rays,N,ui);
    ray.mask  = RTCRayN_mask(rays,N,ui);
    ray.geomID = RTC_INVALID_GEOMETRY_ID;

    /* trace ray through object */
    context->instID[0] = instance->userID;
    rtcOccluded1(instance->object,context,RTCRay_(ray));
    context->instID[0] = -1;
    if (ray.tfar >= 0.0f) continue;

    /* update hit */
    RTCRayN_tfar(rays,N,ui) = ray.tfar;
  }
}

Instance* createInstance (RTCScene scene, RTCScene object, int userID, const Vec3fa& lower, const Vec3fa& upper)
{
  Instance* instance = (Instance*) alignedMalloc(sizeof(Instance),16);
  instance->object = object;
  instance->userID = userID;
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
  if (g_mode == MODE_NORMAL && nativePacketSupported(g_device)) 
  {
    rtcSetGeometryIntersectFunction(instance->geometry,instanceIntersectFunc);
    rtcSetGeometryOccludedFunction (instance->geometry,instanceOccludedFunc);
  }
  else
  {
    rtcSetGeometryIntersectFunction(instance->geometry,instanceIntersectFuncN);
    rtcSetGeometryOccludedFunction (instance->geometry,instanceOccludedFuncN);
  }
  rtcCommitGeometry(instance->geometry);
  rtcAttachGeometry(scene,instance->geometry);
  rtcReleaseGeometry(instance->geometry);
  return instance;
}

void updateInstance (RTCScene scene, Instance* instance)
{
  instance->world2local = rcp(instance->local2world);
  instance->normal2world = transposed(rcp(instance->local2world.l));
  rtcCommitGeometry(instance->geometry);
}

// ======================================================================== //
//                     User defined sphere geometry                         //
// ======================================================================== //

struct Sphere
{
  ALIGNED_STRUCT_(16)
  Vec3fa p;                      //!< position of the sphere
  float r;                      //!< radius of the sphere
  RTCGeometry geometry;
  unsigned int geomID;
};

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

void sphereIntersectFunc(const RTCIntersectFunctionNArguments* args)
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
  potentialHit.instID[0] = args->context->instID[0];
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
    rtcFilterIntersection(args,&fargs);

    if (imask == -1)
      *hit = potentialHit;
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
    rtcFilterIntersection(args,&fargs);

    if (imask == -1)
      *hit = potentialHit;
    else
      ray->tfar = old_t;
  }
}

void sphereOccludedFunc(const RTCOccludedFunctionNArguments* args)
{
  int* valid = args->valid;
  void* ptr  = args->geometryUserPtr;
  Ray *ray = (Ray*)args->ray;
  unsigned int primID = args->primID;
  
  assert(args->N == 1);
  const Sphere* spheres = (const Sphere*) ptr;
  const Sphere& sphere = spheres[primID];
  
  if (!valid[0])
    return;

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
  potentialHit.instID[0] = args->context->instID[0];
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
    rtcFilterOcclusion(args,&fargs);
    
    if (imask == -1)
      ray->tfar = neg_inf;
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
    rtcFilterOcclusion(args,&fargs);

    if (imask == -1)
      ray->tfar = neg_inf;
    else
      ray->tfar = old_t;
  }
}

void sphereIntersectFuncN(const RTCIntersectFunctionNArguments* args)
{
  int* valid = (int*) args->valid;
  void* ptr  = args->geometryUserPtr;
  unsigned int N = args->N;
  RTCRayHitN* rayhit = (RTCRayHitN*)args->rayhit;
  RTCRayN* rays = RTCRayHitN_RayN(rayhit,N);
  RTCHitN* hits = RTCRayHitN_HitN(rayhit,N);
  unsigned int primID = args->primID;
  const Sphere* spheres = (const Sphere*) ptr;

  /* iterate over all rays in ray packet */
  for (unsigned int ui=0; ui<N; ui+=1)
  {
    /* calculate loop and execution mask */
    unsigned int vi = ui+0;
    if (vi>=N) continue;

    /* ignore inactive rays */
    if (valid[vi] != -1) continue;

    const Vec3fa ray_org = Vec3fa(RTCRayN_org_x(rays,N,ui),RTCRayN_org_y(rays,N,ui),RTCRayN_org_z(rays,N,ui));
    const Vec3fa ray_dir = Vec3fa(RTCRayN_dir_x(rays,N,ui),RTCRayN_dir_y(rays,N,ui),RTCRayN_dir_z(rays,N,ui));
    float& ray_tnear = RTCRayN_tnear(rays,N,ui);
    float& ray_tfar = RTCRayN_tfar(rays,N,ui);

    const Sphere& sphere = spheres[primID];
    const Vec3fa v = ray_org-sphere.p;
    const float A = dot(ray_dir,ray_dir);
    const float B = 2.0f*dot(v,ray_dir);
    const float C = dot(v,v) - sqr(sphere.r);
    const float D = B*B - 4.0f*A*C;
    if (D < 0.0f) continue;
    const float Q = sqrt(D);
    const float rcpA = rcp(A);
    const float t0 = 0.5f*rcpA*(-B-Q);
    const float t1 = 0.5f*rcpA*(-B+Q);

    RTCRayHit rtc_ray = rtcGetRayHitFromRayHitN(rayhit,N,ui);
    Ray *ray = (Ray*)&rtc_ray;

    RTCHit potentialhit;
    potentialhit.u = 0.0f;
    potentialhit.v = 0.0f;
    potentialhit.instID[0] = args->context->instID[0];
    potentialhit.geomID = sphere.geomID;
    potentialhit.primID = primID;

    if ((ray_tnear < t0) & (t0 < ray_tfar))
    {
      int imask;
      bool mask = 1;
      {
        imask = mask ? -1 : 0;
      }
      const Vec3fa Ng = ray->org+t0*ray->dir-sphere.p;
      potentialhit.Ng_x = Ng.x;
      potentialhit.Ng_y = Ng.y;
      potentialhit.Ng_z = Ng.z;

      RTCFilterFunctionNArguments fargs;
      fargs.valid = (int*)&imask;
      fargs.geometryUserPtr = ptr;
      fargs.context = args->context;
      fargs.ray = (RTCRayN*)ray;
      fargs.hit = (RTCHitN*)&potentialhit;
      fargs.N = 1;

      ray->tfar = t0;
      rtcFilterIntersection(args,&fargs);
      
      if (imask == -1) {
        ray_tfar = t0;
        rtcCopyHitToHitN(hits,&potentialhit,N,ui);
      }
    }

    if ((ray_tnear < t1) & (t1 < ray_tfar))
    {
      int imask;
      bool mask = 1;
      {
        imask = mask ? -1 : 0;
      }
      const Vec3fa Ng = ray->org+t1*ray->dir-sphere.p;
      potentialhit.Ng_x = Ng.x;
      potentialhit.Ng_y = Ng.y;
      potentialhit.Ng_z = Ng.z;

      RTCFilterFunctionNArguments fargs;
      fargs.valid = (int*)&imask;
      fargs.geometryUserPtr = ptr;
      fargs.context = args->context;
      fargs.ray = (RTCRayN*)ray;
      fargs.hit = (RTCHitN*)&potentialhit;
      fargs.N = 1;

      ray->tfar = t1; 
      rtcFilterIntersection(args,&fargs);
      
      if (imask == -1) {
        ray_tfar = t1;
        rtcCopyHitToHitN(hits,&potentialhit,N,ui);
      }
    }
  }
}

void sphereOccludedFuncN(const RTCOccludedFunctionNArguments* args)
{
  int* valid = args->valid;
  void* ptr  = args->geometryUserPtr;
  RTCRayN* rays = (RTCRayN*)args->ray;
  unsigned int N = args->N;
  unsigned int primID = args->primID;
  const Sphere* spheres = (const Sphere*) ptr;

  /* iterate over all rays in ray packet */
  for (unsigned int ui=0; ui<N; ui+=1)
  {
    /* calculate loop and execution mask */
    unsigned int vi = ui+0;
    if (vi>=N) continue;

    /* ignore inactive rays */
    if (valid[vi] != -1) continue;

    const Vec3fa ray_org = Vec3fa(RTCRayN_org_x(rays,N,ui),RTCRayN_org_y(rays,N,ui),RTCRayN_org_z(rays,N,ui));
    const Vec3fa ray_dir = Vec3fa(RTCRayN_dir_x(rays,N,ui),RTCRayN_dir_y(rays,N,ui),RTCRayN_dir_z(rays,N,ui));
    float& ray_tnear = RTCRayN_tnear(rays,N,ui);
    float& ray_tfar = RTCRayN_tfar(rays,N,ui);

    const Sphere& sphere = spheres[primID];
    const Vec3fa v = ray_org-sphere.p;
    const float A = dot(ray_dir,ray_dir);
    const float B = 2.0f*dot(v,ray_dir);
    const float C = dot(v,v) - sqr(sphere.r);
    const float D = B*B - 4.0f*A*C;
    if (D < 0.0f) continue;
    const float Q = sqrt(D);
    const float rcpA = rcp(A);
    const float t0 = 0.5f*rcpA*(-B-Q);
    const float t1 = 0.5f*rcpA*(-B+Q);

    RTCRay rtc_ray = rtcGetRayFromRayN(rays,N,ui);
    Ray *ray = (Ray*)&rtc_ray;

    RTCHit potentialhit;

    potentialhit.u = 0.0f;
    potentialhit.v = 0.0f;
    potentialhit.instID[0] = args->context->instID[0];
    potentialhit.geomID = sphere.geomID;
    potentialhit.primID = primID;
    if ((ray_tnear < t0) & (t0 < ray_tfar))
    {
      int imask;
      bool mask = 1;
      {
        imask = mask ? -1 : 0;
      }
      const Vec3fa Ng = ray->org+t0*ray->dir-sphere.p;
      potentialhit.Ng_x = Ng.x;
      potentialhit.Ng_y = Ng.y;
      potentialhit.Ng_z = Ng.z;

      RTCFilterFunctionNArguments fargs;
      fargs.valid = (int*)&imask;
      fargs.geometryUserPtr = ptr;
      fargs.context = args->context;
      fargs.ray = (RTCRayN*)ray;
      fargs.hit = (RTCHitN*)&potentialhit;
      fargs.N = 1;

      ray->tfar = t0;  
      rtcFilterOcclusion(args,&fargs);

      if (imask == -1)
        RTCRayN_tfar(rays,N,ui) = neg_inf;
    }

    /* ignore rays that have just found a hit */
    if (RTCRayN_tfar(rays,N,ui) < 0.0f)
      continue;
 
    if ((ray_tnear < t1) & (t1 < ray_tfar))
    {
      int imask;
      bool mask = 1;
      {
        imask = mask ? -1 : 0;
      }
      const Vec3fa Ng = ray_org+t1*ray_dir-sphere.p;
      potentialhit.Ng_x = Ng.x;
      potentialhit.Ng_y = Ng.y;
      potentialhit.Ng_z = Ng.z;

      RTCFilterFunctionNArguments fargs;
      fargs.valid = (int*)&imask;
      fargs.geometryUserPtr = ptr;
      fargs.context = args->context;
      fargs.ray = (RTCRayN*)ray;
      fargs.hit = (RTCHitN*)&potentialhit;
      fargs.N = 1;

      ray->tfar = t1;    
      rtcFilterOcclusion(args,&fargs);

      if (imask == -1)
        RTCRayN_tfar(rays,N,ui) = neg_inf;
    }
  }
}

/* intersection filter function */

void sphereFilterFunction(const RTCFilterFunctionNArguments* args)
{
  int* valid = args->valid;
  const IntersectContext* context = (const IntersectContext*) args->context;
  struct Ray* ray    = (struct Ray*)args->ray;
  //struct RTCHit* hit = (struct RTCHit*)args->hit;
  const unsigned int N = args->N;
  assert(N == 1);


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

void sphereFilterFunctionN(const RTCFilterFunctionNArguments* args)
{
  int* valid = args->valid;
  const IntersectContext* context = (const IntersectContext*) args->context;
  struct RTCRayN* ray = (RTCRayN*)args->ray;
  //struct RTCHitN* hit = args->hit;
  const unsigned int N = args->N;
               
  /* avoid crashing when debug visualizations are used */
  if (context == nullptr)
    return;

  /* iterate over all rays in ray packet */
  for (unsigned int ui=0; ui<N; ui+=1)
  {
    /* calculate loop and execution mask */
    unsigned int vi = ui+0;
    if (vi>=N) continue;

    /* ignore inactive rays */
    if (valid[vi] != -1) continue;
    
    /* calculate hit point */
    Vec3fa ray_org = Vec3fa(RTCRayN_org_x(ray,N,ui),RTCRayN_org_y(ray,N,ui),RTCRayN_org_z(ray,N,ui));
    Vec3fa ray_dir = Vec3fa(RTCRayN_dir_x(ray,N,ui),RTCRayN_dir_y(ray,N,ui),RTCRayN_dir_z(ray,N,ui));
    float hit_t = RTCRayN_tfar(ray,N,ui);

    /* carve out parts of the sphere */
    const Vec3fa h = ray_org+hit_t*ray_dir;
    float v = abs(sin(10.0f*h.x)*cos(10.0f*h.y)*sin(10.0f*h.z));
    float T = clamp((v-0.1f)*3.0f,0.0f,1.0f);

    /* reject some hits */
    if (T < 0.5f) valid[vi] = 0;
  }
}

Sphere* createAnalyticalSphere (RTCScene scene, const Vec3fa& p, float r)
{
  RTCGeometry geom = rtcNewGeometry(g_device, RTC_GEOMETRY_TYPE_USER);
  Sphere* sphere = (Sphere*) alignedMalloc(sizeof(Sphere),16);
  sphere->p = p;
  sphere->r = r;
  sphere->geometry = geom;
  sphere->geomID = rtcAttachGeometry(scene,geom);
  rtcSetGeometryUserPrimitiveCount(geom,1);
  rtcSetGeometryUserData(geom,sphere);
  rtcSetGeometryBoundsFunction(geom,sphereBoundsFunc,nullptr);
  if (g_mode == MODE_NORMAL && nativePacketSupported(g_device))
  {
    rtcSetGeometryIntersectFunction(geom,sphereIntersectFunc);
    rtcSetGeometryOccludedFunction (geom,sphereOccludedFunc);
  }
  else
  {
    rtcSetGeometryIntersectFunction(geom,sphereIntersectFuncN);
    rtcSetGeometryOccludedFunction (geom,sphereOccludedFuncN);
  }
  rtcCommitGeometry(geom);
  rtcReleaseGeometry(geom);
  return sphere;
}

Sphere* createAnalyticalSpheres (RTCScene scene, unsigned int N)
{
  RTCGeometry geom = rtcNewGeometry(g_device, RTC_GEOMETRY_TYPE_USER);
  Sphere* spheres = (Sphere*) alignedMalloc(N*sizeof(Sphere),16);
  unsigned int geomID = rtcAttachGeometry(scene,geom);
  for (unsigned int i=0; i<N; i++) {
    spheres[i].geometry = geom;
    spheres[i].geomID = geomID;
  }
  rtcSetGeometryUserPrimitiveCount(geom,N);
  rtcSetGeometryUserData(geom,spheres);
  rtcSetGeometryBoundsFunction(geom,sphereBoundsFunc,nullptr);
  if (g_mode == MODE_NORMAL && nativePacketSupported(g_device))
  {
    rtcSetGeometryIntersectFunction(geom,sphereIntersectFunc);
    rtcSetGeometryOccludedFunction (geom,sphereOccludedFunc);
    rtcSetGeometryIntersectFilterFunction(geom,sphereFilterFunction);
    rtcSetGeometryOccludedFilterFunction(geom,sphereFilterFunction);
  }
  else
  {
    rtcSetGeometryIntersectFunction(geom,sphereIntersectFuncN);
    rtcSetGeometryOccludedFunction (geom,sphereOccludedFuncN);
    rtcSetGeometryIntersectFilterFunction(geom,sphereFilterFunctionN);
    rtcSetGeometryOccludedFilterFunction(geom,sphereFilterFunctionN);
  }
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

/* scene data */
RTCScene g_scene  = nullptr;
RTCScene g_scene0 = nullptr;
RTCScene g_scene1 = nullptr;
RTCScene g_scene2 = nullptr;
Sphere* g_spheres = nullptr;
Sphere* g_sphere0 = nullptr;
Sphere* g_sphere1 = nullptr;

Instance* g_instance[4] = { nullptr, nullptr, nullptr, nullptr };

Vec3fa colors[5][4];

/* called by the C++ code for initialization */
extern "C" void device_init (char* cfg)
{
  /* create scene */
  g_scene = rtcNewScene(g_device);

  /* create scene with 4 analytical spheres */
  g_scene0 = rtcNewScene(g_device);
  g_spheres = createAnalyticalSpheres(g_scene0,4);
  g_spheres[0].p = Vec3fa( 0, 0,+1); g_spheres[0].r = 0.5f;
  g_spheres[1].p = Vec3fa(+1, 0, 0); g_spheres[1].r = 0.5f;
  g_spheres[2].p = Vec3fa( 0, 0,-1); g_spheres[2].r = 0.5f;
  g_spheres[3].p = Vec3fa(-1, 0, 0); g_spheres[3].r = 0.5f;
  rtcCommitScene(g_scene0);

  /* create scene with 4 triangulated spheres */
  g_scene1 = rtcNewScene(g_device);
  createTriangulatedSphere(g_scene1,Vec3fa( 0, 0,+1),0.5f);
  createTriangulatedSphere(g_scene1,Vec3fa(+1, 0, 0),0.5f);
  createTriangulatedSphere(g_scene1,Vec3fa( 0, 0,-1),0.5f);
  createTriangulatedSphere(g_scene1,Vec3fa(-1, 0, 0),0.5f);
  rtcCommitScene(g_scene1);

  /* create scene with 2 triangulated and 2 analytical spheres */
  g_scene2 = rtcNewScene(g_device);
  createTriangulatedSphere(g_scene2,Vec3fa( 0, 0,+1),0.5f);
  g_sphere0 = createAnalyticalSphere  (g_scene2,Vec3fa(+1, 0, 0),0.5f);
  createTriangulatedSphere(g_scene2,Vec3fa( 0, 0,-1),0.5f);
  g_sphere1 = createAnalyticalSphere  (g_scene2,Vec3fa(-1, 0, 0),0.5f);
  rtcCommitScene(g_scene2);

  /* instantiate geometry */
  g_instance[0] = createInstance(g_scene,g_scene0,0,Vec3fa(-2,-2,-2),Vec3fa(+2,+2,+2));
  g_instance[1] = createInstance(g_scene,g_scene1,1,Vec3fa(-2,-2,-2),Vec3fa(+2,+2,+2));
  g_instance[2] = createInstance(g_scene,g_scene2,2,Vec3fa(-2,-2,-2),Vec3fa(+2,+2,+2));
  g_instance[3] = createInstance(g_scene,g_scene2,3,Vec3fa(-2,-2,-2),Vec3fa(+2,+2,+2));
  createGroundPlane(g_scene);
  rtcCommitScene(g_scene);

  /* set all colors */
  colors[0][0] = Vec3fa(0.25f, 0.00f, 0.00f);
  colors[0][1] = Vec3fa(0.50f, 0.00f, 0.00f);
  colors[0][2] = Vec3fa(0.75f, 0.00f, 0.00f);
  colors[0][3] = Vec3fa(1.00f, 0.00f, 0.00f);

  colors[1][0] = Vec3fa(0.00f, 0.25f, 0.00f);
  colors[1][1] = Vec3fa(0.00f, 0.50f, 0.00f);
  colors[1][2] = Vec3fa(0.00f, 0.75f, 0.00f);
  colors[1][3] = Vec3fa(0.00f, 1.00f, 0.00f);

  colors[2][0] = Vec3fa(0.00f, 0.00f, 0.25f);
  colors[2][1] = Vec3fa(0.00f, 0.00f, 0.50f);
  colors[2][2] = Vec3fa(0.00f, 0.00f, 0.75f);
  colors[2][3] = Vec3fa(0.00f, 0.00f, 1.00f);

  colors[3][0] = Vec3fa(0.25f, 0.25f, 0.00f);
  colors[3][1] = Vec3fa(0.50f, 0.50f, 0.00f);
  colors[3][2] = Vec3fa(0.75f, 0.75f, 0.00f);
  colors[3][3] = Vec3fa(1.00f, 1.00f, 0.00f);

  colors[4][0] = Vec3fa(1.0f, 1.0f, 1.0f);
  colors[4][1] = Vec3fa(1.0f, 1.0f, 1.0f);
  colors[4][2] = Vec3fa(1.0f, 1.0f, 1.0f);
  colors[4][3] = Vec3fa(1.0f, 1.0f, 1.0f);

  /* set start render mode */
  if (g_mode == MODE_NORMAL) renderTile = renderTileStandard;
  else                       renderTile = renderTileStandardStream;
  key_pressed_handler = device_key_pressed_default;
}

inline Vec3fa face_forward(const Vec3fa& dir, const Vec3fa& _Ng) {
  const Vec3fa Ng = _Ng;
  return dot(dir,Ng) < 0.0f ? Ng : neg(Ng);
}

/* task that renders a single screen tile */
Vec3fa renderPixelStandard(float x, float y, const ISPCCamera& camera, RayStats& stats)
{
  RTCIntersectContext context;
  rtcInitIntersectContext(&context);
  
  /* initialize ray */
  Ray ray(Vec3fa(camera.xfm.p), 
                     Vec3fa(normalize(x*camera.xfm.l.vx + y*camera.xfm.l.vy + camera.xfm.l.vz)), 
                     0.0f, inf, 0.0f, -1,
                     RTC_INVALID_GEOMETRY_ID, RTC_INVALID_GEOMETRY_ID, RTC_INVALID_GEOMETRY_ID);

  /* intersect ray with scene */
  rtcIntersect1(g_scene,&context,RTCRayHit_(ray));
  RayStats_addRay(stats);

  /* shade pixels */
  Vec3fa color = Vec3fa(0.0f);
  if (ray.geomID != RTC_INVALID_GEOMETRY_ID)
  {
    /* calculate shading normal in world space */
    Vec3fa Ns = ray.Ng;

    if (ray.instID != RTC_INVALID_GEOMETRY_ID) {
      Ns = xfmVector(g_instance[ray.instID]->normal2world,Vec3fa(Ns));
    }
    Ns = face_forward(ray.dir,normalize(Ns));

    /* calculate diffuse color of geometries */
    Vec3fa diffuse = Vec3fa(0.0f);
    if      (ray.instID ==  0) diffuse = colors[ray.instID][ray.primID];
    else if (ray.instID == -1) diffuse = colors[4][ray.primID];
    else                       diffuse = colors[ray.instID][ray.geomID];
    color = color + diffuse*0.5;

    /* initialize shadow ray */
    Vec3fa lightDir = normalize(Vec3fa(-1,-1,-1));
    Ray shadow(ray.org + 0.999f*ray.tfar*ray.dir, neg(lightDir), 0.001f, inf);

    /* trace shadow ray */
    rtcOccluded1(g_scene,&context,RTCRay_(shadow));
    RayStats_addShadowRay(stats);

    /* add light contribution */
    if (shadow.tfar >= 0.0f)
      color = color + diffuse*clamp(-dot(lightDir,Ns),0.0f,1.0f);
  }
  return color;
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
    /* calculate pixel color */
    Vec3fa color = renderPixelStandard((float)x,(float)y,camera,g_stats[threadIndex]);

    /* write color to framebuffer */
    unsigned int r = (unsigned int) (255.0f * clamp(color.x,0.0f,1.0f));
    unsigned int g = (unsigned int) (255.0f * clamp(color.y,0.0f,1.0f));
    unsigned int b = (unsigned int) (255.0f * clamp(color.z,0.0f,1.0f));
    pixels[y*width+x] = (b << 16) + (g << 8) + r;
  }
}

/* renders a single screen tile */
void renderTileStandardStream(int taskIndex,
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

  RayStats& stats = g_stats[threadIndex];

  Ray primary_stream[TILE_SIZE_X*TILE_SIZE_Y];
  Ray shadow_stream[TILE_SIZE_X*TILE_SIZE_Y];
  Vec3fa color_stream[TILE_SIZE_X*TILE_SIZE_Y];
  bool valid_stream[TILE_SIZE_X*TILE_SIZE_Y];

  /* generate stream of primary rays */
  int N = 0;
  for (unsigned int y=y0; y<y1; y++) for (unsigned int x=x0; x<x1; x++)
  {
    /* ISPC workaround for mask == 0 */
    

    /* initialize variables */
    color_stream[N] = Vec3fa(0.0f);
    bool mask = 1; { valid_stream[N] = mask; }

    /* initialize ray */
    Ray& primary = primary_stream[N];
    mask = 1; { // invalidates inactive rays
      primary.tnear() = mask ? 0.0f         : (float)(pos_inf);
      primary.tfar  = mask ? (float)(inf) : (float)(neg_inf);
    }

    init_Ray(primary, Vec3fa(camera.xfm.p), Vec3fa(normalize((float)x*camera.xfm.l.vx + (float)y*camera.xfm.l.vy + camera.xfm.l.vz)), primary.tnear(), primary.tfar, 0.0f, -1,
             RTC_INVALID_GEOMETRY_ID, RTC_INVALID_GEOMETRY_ID, RTC_INVALID_GEOMETRY_ID);
    N++;
    RayStats_addRay(stats);
  }

  Vec3fa lightDir = normalize(Vec3fa(-1,-1,-1));

  /* trace rays */
  RTCIntersectContext primary_context;
  rtcInitIntersectContext(&primary_context);
  primary_context.flags = g_iflags_coherent;
  rtcIntersect1M(g_scene,&primary_context,(RTCRayHit*)&primary_stream,N,sizeof(Ray));

  /* terminate rays and update color */
  N = -1;
  for (unsigned int y=y0; y<y1; y++) for (unsigned int x=x0; x<x1; x++)
  {
    N++;
    /* ISPC workaround for mask == 0 */
    

    /* invalidate shadow rays by default */
    Ray& shadow = shadow_stream[N];
    {
      shadow.tnear() = (float)(pos_inf);
      shadow.tfar  = (float)(neg_inf);
    }

    Ray& primary = primary_stream[N];

    /* ignore invalid rays */
    if (valid_stream[N] == false) continue;

    /* terminate rays that hit nothing */
    if (primary_stream[N].geomID == RTC_INVALID_GEOMETRY_ID) {
      valid_stream[N] = false;
      continue;
    }

    /* calculate diffuse color of geometries */
    Vec3fa diffuse = Vec3fa(0.0f);
    if      (primary.instID ==  0) diffuse = colors[primary.instID][primary.primID];
    else if (primary.instID == -1) diffuse = colors[4][primary.primID];      
    else                           diffuse = colors[primary.instID][primary.geomID];
    color_stream[N] = color_stream[N] + diffuse*0.5;

    /* initialize shadow ray */
    bool mask = 1; {
      shadow.tnear() = mask ? 0.001f       : (float)(pos_inf);
      shadow.tfar  = mask ? (float)(inf) : (float)(neg_inf);
    }
    init_Ray(shadow,primary.org + 0.999f*primary.tfar*primary.dir, neg(lightDir), shadow.tnear(), shadow.tfar, 0.0f, N*1 + 0);

    RayStats_addShadowRay(stats);
  }
  N++;

  /* trace shadow rays */
  RTCIntersectContext shadow_context;
  rtcInitIntersectContext(&shadow_context);
  shadow_context.flags = g_iflags_coherent;
  rtcOccluded1M(g_scene,&shadow_context,(RTCRay*)&shadow_stream,N,sizeof(Ray));

  /* add light contribution */
  N = -1;
  for (unsigned int y=y0; y<y1; y++) for (unsigned int x=x0; x<x1; x++)
  {
    N++;
    /* ISPC workaround for mask == 0 */
    

    /* ignore invalid rays */
    if (valid_stream[N] == false) continue;

    /* calculate shading normal in world space */
    Ray& primary = primary_stream[N];
    Vec3fa Ns = primary.Ng;
    if (primary.instID != RTC_INVALID_GEOMETRY_ID) {
      Ns = xfmVector(g_instance[primary.instID]->normal2world,Vec3fa(Ns));
    }
    Ns = face_forward(primary.dir,normalize(Ns));
    
    /* add light contrinution */
    Vec3fa diffuse = Vec3fa(0.0f);
    if      (primary.instID ==  0) diffuse = colors[primary.instID][primary.primID];
    else if (primary.instID == -1) diffuse = colors[4][primary.primID];      
    else                           diffuse = colors[primary.instID][primary.geomID];
    Ray& shadow = shadow_stream[N];
    if (shadow.tfar >= 0.0f) {
      color_stream[N] = color_stream[N] + diffuse*clamp(-dot(lightDir,Ns),0.0f,1.0f);
    }
  }
  N++;

  /* framebuffer writeback */
  N = 0;
  for (unsigned int y=y0; y<y1; y++) for (unsigned int x=x0; x<x1; x++)
  {
    /* ISPC workaround for mask == 0 */
    

    /* write color to framebuffer */
    unsigned int r = (unsigned int) (255.0f * clamp(color_stream[N].x,0.0f,1.0f));
    unsigned int g = (unsigned int) (255.0f * clamp(color_stream[N].y,0.0f,1.0f));
    unsigned int b = (unsigned int) (255.0f * clamp(color_stream[N].z,0.0f,1.0f));
    pixels[y*width+x] = (b << 16) + (g << 8) + r;
    N++;
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
  renderTile(taskIndex,threadIndex,pixels,width,height,time,camera,numTilesX,numTilesY);
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
  g_instance[0]->local2world = AffineSpace3fa(xfm,2.2f*Vec3fa(+cos(t0),0.0f,+sin(t0)));
  g_instance[1]->local2world = AffineSpace3fa(xfm,2.2f*Vec3fa(-cos(t0),0.0f,-sin(t0)));
  g_instance[2]->local2world = AffineSpace3fa(xfm,2.2f*Vec3fa(-sin(t0),0.0f,+cos(t0)));
  g_instance[3]->local2world = AffineSpace3fa(xfm,2.2f*Vec3fa(+sin(t0),0.0f,-cos(t0)));

  /* update scene */
  updateInstance(g_scene,g_instance[0]);
  updateInstance(g_scene,g_instance[1]);
  updateInstance(g_scene,g_instance[2]);
  updateInstance(g_scene,g_instance[3]);
  rtcCommitScene (g_scene);

  /* render all pixels */
  const int numTilesX = (width +TILE_SIZE_X-1)/TILE_SIZE_X;
  const int numTilesY = (height+TILE_SIZE_Y-1)/TILE_SIZE_Y;
  parallel_for(size_t(0),size_t(numTilesX*numTilesY),[&](const range<size_t>& range) {
    const int threadIndex = (int)TaskScheduler::threadIndex();
    for (size_t i=range.begin(); i<range.end(); i++)
      renderTileTask((int)i,threadIndex,pixels,width,height,time,camera,numTilesX,numTilesY);
  }); 
}

/* called by the C++ code for cleanup */
extern "C" void device_cleanup ()
{
  rtcReleaseScene (g_scene); g_scene = nullptr;
  rtcReleaseScene (g_scene0); g_scene0 = nullptr;
  rtcReleaseScene (g_scene1); g_scene1 = nullptr;
  rtcReleaseScene (g_scene2); g_scene2 = nullptr;
  rtcReleaseDevice(g_device); g_device = nullptr;
  alignedFree(g_spheres); g_spheres = nullptr;
  alignedFree(g_sphere0); g_sphere0 = nullptr;
  alignedFree(g_sphere1); g_sphere1 = nullptr;
}

} // namespace embree
