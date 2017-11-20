// ======================================================================== //
// Copyright 2009-2017 Intel Corporation                                    //
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

#include "../common/tutorial/tutorial_device.isph"

const uniform int numPhi = 5;
const uniform int numTheta = 2*numPhi;

RTCDevice g_device = NULL;

void renderTileStandardStream(uniform int taskIndex,
                              uniform int threadIndex,
                              uniform int* uniform pixels,
                              const uniform unsigned int width,
                              const uniform unsigned int height,
                              const uniform float time,
                              const uniform ISPCCamera& camera,
                              const uniform int numTilesX,
                              const uniform int numTilesY);

// ======================================================================== //
//                         User defined instancing                          //
// ======================================================================== //

struct Instance
{
  ALIGNED_STRUCT
  RTCGeometry geometry;
  RTCScene object;
  uniform int userID;
  AffineSpace3f local2world;
  AffineSpace3f world2local;
  LinearSpace3f normal2world;
  Vec3f lower;
  Vec3f upper;
};

unmasked void instanceBoundsFunc(const struct RTCBoundsFunctionArguments* uniform const args)
{
  const uniform Instance* uniform instance = (const uniform Instance* uniform) args->geomUserPtr;
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

unmasked void instanceIntersectFunc(const RTCIntersectFunctionNArguments* uniform const args)
{
  
  const uniform int* uniform valid = args->valid;
  void* uniform ptr  = args->geomUserPtr;
  uniform RTCIntersectContext* uniform context = args->context;
  RTCRayN* uniform rays = args->ray;
                                    
  assert(args->N == programCount);
  if (!valid[programIndex])
    return;
  
  varying Ray *uniform ray = (varying Ray*uniform)rays;
  const uniform Instance* uniform instance = (const uniform Instance* uniform)ptr;
  const Vec3f ray_org = ray->org;
  const Vec3f ray_dir = ray->dir;
  const float ray_tnear = ray->tnear;
  const float ray_tfar  = ray->tfar;
  ray->org = xfmPoint (instance->world2local,ray_org);
  ray->dir = xfmVector(instance->world2local,ray_dir);
  ray->tnear = ray_tnear;
  ray->tfar  = ray_tfar;
  context->instID = instance->userID;
  rtcIntersect(instance->object,context,RTCRay_(*ray));
  context->instID = -1;
  const float updated_tfar = ray->tfar;
  ray->org = ray_org;
  ray->dir = ray_dir;
  ray->tfar = updated_tfar;
}

unmasked void instanceOccludedFunc(const RTCOccludedFunctionNArguments* uniform const args)
{
  const uniform int* uniform valid = args->valid;
  void* uniform ptr  = args->geomUserPtr;
  uniform RTCIntersectContext* uniform context = args->context;
  RTCRayN* uniform rays = args->ray;
  assert(args->N == programCount);
  if (!valid[programIndex])
    return;
  
  varying Ray *uniform ray = (varying Ray*uniform)rays;
  const uniform Instance* uniform instance = (const uniform Instance* uniform)ptr;
  const Vec3f ray_org = ray->org;
  const Vec3f ray_dir = ray->dir;
  const float ray_tnear = ray->tnear;
  const float ray_tfar  = ray->tfar;
  ray->org    = xfmPoint (instance->world2local,ray_org);
  ray->dir    = xfmVector(instance->world2local,ray_dir);
  ray->tnear  = ray_tnear;
  ray->tfar   = ray_tfar;
  context->instID = instance->userID;
  rtcOccluded(instance->object,context,RTCRay_(*ray));
  context->instID = -1;
  ray->org    = ray_org;
  ray->dir    = ray_dir;
  ray->tnear  = ray_tnear;
  ray->tfar   = ray_tfar;
}

unmasked void instanceIntersectFuncN(const RTCIntersectFunctionNArguments* uniform const args)
{
  /* avoid crashing when debug visualizations are used */
  if (args->context == NULL)
    return;

  /* fast path */
  if (args->N == programCount) {
    instanceIntersectFunc(args);
    return;
  }

  const uniform int* uniform valid = args->valid;
  void* uniform ptr  = args->geomUserPtr;
  uniform RTCIntersectContext* uniform context = args->context;
  RTCRayN* uniform rays = args->ray;
  uniform unsigned int N = args->N;
  const uniform Instance* uniform instance = (const uniform Instance* uniform) ptr;

  /* iterate over all rays in ray packet */
  for (uniform unsigned int ui=0; ui<N; ui+=programCount)
  {
    /* calculate varying loop and execution mask */
    unsigned int vi = ui+programIndex;
    if (vi>=N) continue;

    /* ignore inactive rays */
    if (valid[vi] != -1) continue;

    /* create transformed ray */
    Ray ray;
    const Vec3f ray_org = make_Vec3f(RTCRayN_org_x(rays,N,ui),RTCRayN_org_y(rays,N,ui),RTCRayN_org_z(rays,N,ui));
    const Vec3f ray_dir = make_Vec3f(RTCRayN_dir_x(rays,N,ui),RTCRayN_dir_y(rays,N,ui),RTCRayN_dir_z(rays,N,ui));
    ray.org = xfmPoint (instance->world2local,ray_org);
    ray.dir = xfmVector(instance->world2local,ray_dir);
    bool mask = __mask; unmasked {
      ray.tnear = mask ? RTCRayN_tnear(rays,N,ui) : (float)(pos_inf);
      ray.tfar  = mask ? RTCRayN_tfar(rays,N,ui ) : (float)(neg_inf);
    }
    ray.time  = RTCRayN_time(rays,N,ui);
    ray.mask  = RTCRayN_mask(rays,N,ui);
    ray.geomID = RTC_INVALID_GEOMETRY_ID;

    /* trace ray through object */
    context->instID = instance->userID;
    rtcIntersect(instance->object,context,RTCRay_(ray));
    context->instID = -1;
    if (ray.geomID == RTC_INVALID_GEOMETRY_ID) continue;

    /* update hit */
    RTCRayN_u(rays,N,ui) = ray.u;
    RTCRayN_v(rays,N,ui) = ray.v;
    RTCRayN_tfar(rays,N,ui) = ray.tfar;
    RTCRayN_instID(rays,N,ui) = instance->userID;
    RTCRayN_geomID(rays,N,ui) = ray.geomID;
    RTCRayN_primID(rays,N,ui) = ray.primID;
    RTCRayN_Ng_x(rays,N,ui) = ray.Ng.x;
    RTCRayN_Ng_y(rays,N,ui) = ray.Ng.y;
    RTCRayN_Ng_z(rays,N,ui) = ray.Ng.z;
  }
}

unmasked void instanceOccludedFuncN(const RTCOccludedFunctionNArguments* uniform const args)
{
  /* avoid crashing when debug visualizations are used */
  if (args->context == NULL)
    return;
  
  /* fast path */
  if (args->N == programCount) {
    instanceOccludedFunc(args);
    return;
  }

  const uniform int* uniform valid = args->valid;
  void* uniform ptr  = args->geomUserPtr;
  uniform RTCIntersectContext* uniform context = args->context;
  RTCRayN* uniform rays = args->ray;
  uniform unsigned int N = args->N;
  const uniform Instance* uniform instance = (const uniform Instance* uniform) ptr;

  /* iterate over all rays in ray packet */
  for (uniform unsigned int ui=0; ui<N; ui+=programCount)
  {
    /* calculate varying loop and execution mask */
    unsigned int vi = ui+programIndex;
    if (vi>=N) continue;

    /* ignore inactive rays */
    if (valid[vi] != -1) continue;

    /* create transformed ray */
    Ray ray;
    const Vec3f ray_org = make_Vec3f(RTCRayN_org_x(rays,N,ui),RTCRayN_org_y(rays,N,ui),RTCRayN_org_z(rays,N,ui));
    const Vec3f ray_dir = make_Vec3f(RTCRayN_dir_x(rays,N,ui),RTCRayN_dir_y(rays,N,ui),RTCRayN_dir_z(rays,N,ui));
    ray.org = xfmPoint (instance->world2local,ray_org);
    ray.dir = xfmVector(instance->world2local,ray_dir);
    bool mask = __mask; unmasked {
      ray.tnear = mask ? RTCRayN_tnear(rays,N,ui) : (float)(pos_inf);
      ray.tfar  = mask ? RTCRayN_tfar(rays,N,ui)  : (float)(neg_inf);
    }
    ray.time  = RTCRayN_time(rays,N,ui);
    ray.mask  = RTCRayN_mask(rays,N,ui);
    ray.geomID = RTC_INVALID_GEOMETRY_ID;

    /* trace ray through object */
    context->instID = instance->userID;
    rtcOccluded(instance->object,context,RTCRay_(ray));
    context->instID = -1;
    if (ray.geomID == RTC_INVALID_GEOMETRY_ID) continue;

    /* update hit */
    RTCRayN_geomID(rays,N,ui) = ray.geomID;
  }
}

uniform Instance* uniform createInstance (RTCScene scene, RTCScene object, uniform int userID, const uniform Vec3f& lower, const uniform Vec3f& upper)
{
  uniform Instance* uniform instance = uniform new uniform Instance;
  instance->object = object;
  instance->userID = userID;
  instance->lower = lower;
  instance->upper = upper;
  instance->local2world.l.vx = make_Vec3f(1,0,0);
  instance->local2world.l.vy = make_Vec3f(0,1,0);
  instance->local2world.l.vz = make_Vec3f(0,0,1);
  instance->local2world.p    = make_Vec3f(0,0,0);
  instance->geometry = rtcNewUserGeometry(g_device);
  rtcSetNumPrimitives(instance->geometry,1);
  rtcSetUserData(instance->geometry,instance);
  rtcSetBoundsFunction(instance->geometry,instanceBoundsFunc,NULL);
  rtcSetIntersectFunction(instance->geometry,instanceIntersectFuncN);
  rtcSetOccludedFunction (instance->geometry,instanceOccludedFuncN);
  rtcCommitGeometry(instance->geometry);
  rtcAttachGeometry(scene,instance->geometry);
  rtcReleaseGeometry(instance->geometry);
  return instance;
}

void updateInstance (RTCScene scene, uniform Instance* uniform instance)
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
  ALIGNED_STRUCT
  Vec3f p;                      //!< position of the sphere
  float r;                      //!< radius of the sphere
  RTCGeometry geometry;
  uniform unsigned int geomID;
};

unmasked void sphereBoundsFunc(const struct RTCBoundsFunctionArguments* uniform const args)
{
  const uniform Sphere* uniform spheres = (const uniform Sphere* uniform) args->geomUserPtr;
  uniform RTCBounds* uniform bounds_o = args->bounds_o;
  const uniform Sphere& sphere = spheres[args->item];
  bounds_o->lower_x = sphere.p.x-sphere.r;
  bounds_o->lower_y = sphere.p.y-sphere.r;
  bounds_o->lower_z = sphere.p.z-sphere.r;
  bounds_o->upper_x = sphere.p.x+sphere.r;
  bounds_o->upper_y = sphere.p.y+sphere.r;
  bounds_o->upper_z = sphere.p.z+sphere.r;
}

unmasked void sphereIntersectFunc(const RTCIntersectFunctionNArguments* uniform const args)
{
  uniform int* uniform valid = args->valid;
  void* uniform ptr  = args->geomUserPtr;
  varying Ray *uniform ray = (varying Ray*uniform)args->ray;
  uniform unsigned int item = args->item;
  
  assert(args->N == programCount);
  const uniform Sphere* uniform spheres = (const uniform Sphere* uniform)ptr;
  const uniform Sphere& sphere = spheres[item];
  
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

#if 1
  varying RTCHit hit;
  hit.u = 0.0f;
  hit.v = 0.0f;
  hit.instID = args->context->instID;
  hit.geomID = sphere.geomID;
  hit.primID = item;
  if ((ray->tnear < t0) & (t0 < ray->tfar))
  {
    varying int imask;
    varying bool mask = __mask;
    unmasked {
      imask = mask ? -1 : 0;
    }
    
    const Vec3f Ng = ray->org+t0*ray->dir-sphere.p;
    hit.t = t0;
    hit.Ngx = Ng.x;
    hit.Ngy = Ng.y;
    hit.Ngz = Ng.z;

    uniform RTCFilterFunctionNArguments fargs;
    fargs.valid = (int* uniform)&imask;
    fargs.geomUserPtr = ptr;
    fargs.context = args->context;
    fargs.ray = args->ray;
    fargs.potentialHit = (RTCHitN* uniform)&hit;
    fargs.N = programCount;
    rtcReportIntersection(args,&fargs);

    if (imask == -1)
    {
      ray->u = hit.u;
      ray->v = hit.v;
      ray->tfar = hit.t;
      ray->instID = hit.instID;
      ray->geomID = hit.geomID;
      ray->primID = hit.primID;
      ray->Ng.x = hit.Ngx;
      ray->Ng.y = hit.Ngy;
      ray->Ng.z = hit.Ngz;
    }
  }

  if ((ray->tnear < t1) & (t1 < ray->tfar))
  {
    varying int imask;
    varying bool mask = __mask;
    unmasked {
      imask = mask ? -1 : 0;
    }
    
    const Vec3f Ng = ray->org+t0*ray->dir-sphere.p;
    hit.t = t1;
    hit.Ngx = Ng.x;
    hit.Ngy = Ng.y;
    hit.Ngz = Ng.z;

    uniform RTCFilterFunctionNArguments fargs;
    fargs.valid = (int* uniform)&imask;
    fargs.geomUserPtr = ptr;
    fargs.context = args->context;
    fargs.ray = args->ray;
    fargs.potentialHit = (RTCHitN* uniform)&hit;
    fargs.N = programCount;
    rtcReportIntersection(args,&fargs);

    if (imask == -1)
    {
      ray->u = hit.u;
      ray->v = hit.v;
      ray->tfar = hit.t;
      ray->instID = hit.instID;
      ray->geomID = hit.geomID;
      ray->primID = hit.primID;
      ray->Ng.x = hit.Ngx;
      ray->Ng.y = hit.Ngy;
      ray->Ng.z = hit.Ngz;
    }
  }

#else
  if ((ray->tnear < t0) & (t0 < ray->tfar)) {
    ray->u = 0.0f;
    ray->v = 0.0f;
    ray->tfar = t0;
    ray->instID = args->context->instID;
    ray->geomID = sphere.geomID;
    ray->primID = (unsigned int) item;
    ray->Ng = ray->org+t0*ray->dir-sphere.p;
  }
  if ((ray->tnear < t1) & (t1 < ray->tfar)) {
    ray->u = 0.0f;
    ray->v = 0.0f;
    ray->tfar = t1;
    ray->instID = args->context->instID;
    ray->geomID = sphere.geomID;
    ray->primID = (unsigned int) item;
    ray->Ng = ray->org+t1*ray->dir-sphere.p;
  }
#endif
}

unmasked void sphereOccludedFunc(const RTCOccludedFunctionNArguments* uniform const args)
{
  uniform int* uniform valid = args->valid;
  void* uniform ptr  = args->geomUserPtr;
  varying Ray *uniform ray = (varying Ray*uniform)args->ray;
  uniform unsigned int item = args->item;
  
  assert(args->N == programCount);
  const uniform Sphere* uniform spheres = (const uniform Sphere* uniform) ptr;
  const uniform Sphere& sphere = spheres[item];
  
  if (!valid[programIndex])
    return;
  
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
#if 1
  varying RTCHit hit;
  hit.u = 0.0f;
  hit.v = 0.0f;
  hit.instID = args->context->instID;
  hit.geomID = sphere.geomID;
  hit.primID = item;
  if ((ray->tnear < t0) & (t0 < ray->tfar))
  {
    varying int imask;
    varying bool mask = __mask;
    unmasked {
      imask = mask ? -1 : 0;
    }
    
    const Vec3f Ng = ray->org+t0*ray->dir-sphere.p;
    hit.t = t0;
    hit.Ngx = Ng.x;
    hit.Ngy = Ng.y;
    hit.Ngz = Ng.z;

    uniform RTCFilterFunctionNArguments fargs;
    fargs.valid = (int* uniform)&imask;
    fargs.geomUserPtr = ptr;
    fargs.context = args->context;
    fargs.ray = args->ray;
    fargs.potentialHit = (RTCHitN* uniform)&hit;
    fargs.N = programCount;
    rtcReportOcclusion(args,&fargs);

    if (imask == -1)
    {
      ray->geomID = 0;
    }
  }

  if ((ray->tnear < t1) & (t1 < ray->tfar))
  {
    varying int imask;
    varying bool mask = __mask;
    unmasked {
      imask = mask ? -1 : 0;
    }
    
    const Vec3f Ng = ray->org+t0*ray->dir-sphere.p;
    hit.t = t1;
    hit.Ngx = Ng.x;
    hit.Ngy = Ng.y;
    hit.Ngz = Ng.z;

    uniform RTCFilterFunctionNArguments fargs;
    fargs.valid = (int* uniform)&imask;
    fargs.geomUserPtr = ptr;
    fargs.context = args->context;
    fargs.ray = args->ray;
    fargs.potentialHit = (RTCHitN* uniform)&hit;
    fargs.N = programCount;
    rtcReportOcclusion(args,&fargs);

    if (imask == -1)
    {
      ray->geomID = 0;
    }

  }

#else
  if ((ray->tnear < t0) & (t0 < ray->tfar)) {
    ray->geomID = 0;
  }
  if ((ray->tnear < t1) & (t1 < ray->tfar)) {
    ray->geomID = 0;
  }
#endif
}

unmasked void sphereIntersectFuncN(const RTCIntersectFunctionNArguments* uniform const args)
{
  uniform int* uniform valid = (uniform int*) args->valid;
  void* uniform ptr  = args->geomUserPtr;
  RTCRayN* uniform rays = args->ray;
  uniform unsigned int N = args->N;
  uniform unsigned int item = args->item;
  const uniform Sphere* uniform spheres = (const uniform Sphere* uniform) ptr;

  /* iterate over all rays in ray packet */
  for (uniform unsigned int ui=0; ui<N; ui+=programCount)
  {
    /* calculate varying loop and execution mask */
    unsigned int vi = ui+programIndex;
    if (vi>=N) continue;

    /* ignore inactive rays */
    if (valid[vi] != -1) continue;

    const Vec3f ray_org = make_Vec3f(RTCRayN_org_x(rays,N,ui),RTCRayN_org_y(rays,N,ui),RTCRayN_org_z(rays,N,ui));
    const Vec3f ray_dir = make_Vec3f(RTCRayN_dir_x(rays,N,ui),RTCRayN_dir_y(rays,N,ui),RTCRayN_dir_z(rays,N,ui));
    float& ray_tnear = RTCRayN_tnear(rays,N,ui);
    float& ray_tfar = RTCRayN_tfar(rays,N,ui);

    const uniform Sphere& sphere = spheres[item];
    const Vec3f v = ray_org-sphere.p;
    const float A = dot(ray_dir,ray_dir);
    const float B = 2.0f*dot(v,ray_dir);
    const float C = dot(v,v) - sqr(sphere.r);
    const float D = B*B - 4.0f*A*C;
    if (D < 0.0f) continue;
    const float Q = sqrt(D);
    const float rcpA = rcp(A);
    const float t0 = 0.5f*rcpA*(-B-Q);
    const float t1 = 0.5f*rcpA*(-B+Q);

#if 1
    RTCRay ray = RTCRayNtoRTCRay(args->ray,N,ui);
    
    RTCHit potentialhit;
    potentialhit.u = 0.0f;
    potentialhit.v = 0.0f;
    potentialhit.instID = args->context->instID;
    potentialhit.geomID = sphere.geomID;
    potentialhit.primID = item;
    if ((ray_tnear < t0) & (t0 < ray_tfar))
    {
      varying int imask;
      varying bool mask = __mask;
      unmasked {
        imask = mask ? -1 : 0;
      }
      const Vec3f Ng = ray_org+t0*ray_dir-sphere.p;
      potentialhit.t = t0;
      potentialhit.Ng_x = Ng.x;
      potentialhit.Ng_y = Ng.y;
      potentialhit.Ng_z = Ng.z;

      uniform RTCFilterFunctionNArguments fargs;
      fargs.valid = (int* uniform)&imask;
      fargs.geomUserPtr = ptr;
      fargs.context = args->context;
      fargs.ray = (RTCRayN* uniform)&ray;
      fargs.potentialHit = (RTCHitN* uniform)&potentialhit;
      fargs.N = programCount;
  
      rtcReportIntersection(args,&fargs);

      /* update for all accepted hits */
      if (imask == -1)
      {
        RTCRayN_u(rays,N,ui) = potentialhit.u;
        RTCRayN_v(rays,N,ui) = potentialhit.v;
        RTCRayN_tfar(rays,N,ui) = potentialhit.t;
        RTCRayN_instID(rays,N,ui) = potentialhit.instID;
        RTCRayN_geomID(rays,N,ui) = potentialhit.geomID;
        RTCRayN_primID(rays,N,ui) = potentialhit.primID;
        RTCRayN_Ng_x(rays,N,ui) = potentialhit.Ng_x;
        RTCRayN_Ng_y(rays,N,ui) = potentialhit.Ng_y;
        RTCRayN_Ng_z(rays,N,ui) = potentialhit.Ng_z;
      }
    }

    if ((ray_tnear < t1) & (t1 < ray_tfar))
    {
      varying int imask;
      varying bool mask = __mask;
      unmasked {
        imask = mask ? -1 : 0;
      }
      const Vec3f Ng = ray_org+t1*ray_dir-sphere.p;
      potentialhit.t = t1;
      potentialhit.Ng_x = Ng.x;
      potentialhit.Ng_y = Ng.y;
      potentialhit.Ng_z = Ng.z;

      uniform RTCFilterFunctionNArguments fargs;
      fargs.valid = (int* uniform)&imask;
      fargs.geomUserPtr = ptr;
      fargs.context = args->context;
      fargs.ray = (RTCRayN* uniform)&ray;
      fargs.potentialHit = (RTCHitN* uniform)&potentialhit;
      fargs.N = programCount;
  
      rtcReportIntersection(args,&fargs);

      /* update for all accepted hits */
      if (imask == -1)
      {
        RTCRayN_u(rays,N,ui) = potentialhit.u;
        RTCRayN_v(rays,N,ui) = potentialhit.v;
        RTCRayN_tfar(rays,N,ui) = potentialhit.t;
        RTCRayN_instID(rays,N,ui) = potentialhit.instID;
        RTCRayN_geomID(rays,N,ui) = potentialhit.geomID;
        RTCRayN_primID(rays,N,ui) = potentialhit.primID;
        RTCRayN_Ng_x(rays,N,ui) = potentialhit.Ng_x;
        RTCRayN_Ng_y(rays,N,ui) = potentialhit.Ng_y;
        RTCRayN_Ng_z(rays,N,ui) = potentialhit.Ng_z;
      }
    }
    
#else
    
    if ((ray_tnear < t0) & (t0 < ray_tfar))
    {
      RTCRayN_u(rays,N,ui) = 0.0f;
      RTCRayN_v(rays,N,ui) = 0.0f;
      RTCRayN_tfar(rays,N,ui) = t0;
      RTCRayN_instID(rays,N,ui) = args->context->instID;
      RTCRayN_geomID(rays,N,ui) = sphere.geomID;
      RTCRayN_primID(rays,N,ui) = (unsigned int)item;
      const Vec3f Ng = ray_org+t0*ray_dir-sphere.p;
      RTCRayN_Ng_x(rays,N,ui) = Ng.x;
      RTCRayN_Ng_y(rays,N,ui) = Ng.y;
      RTCRayN_Ng_z(rays,N,ui) = Ng.z;
    }
    if ((ray_tnear < t1) & (t1 < ray_tfar))
    {
      RTCRayN_u(rays,N,ui) = 0.0f;
      RTCRayN_v(rays,N,ui) = 0.0f;
      RTCRayN_tfar(rays,N,ui) = t1;
      RTCRayN_instID(rays,N,ui) = args->context->instID;
      RTCRayN_geomID(rays,N,ui) = sphere.geomID;
      RTCRayN_primID(rays,N,ui) = (unsigned int)item;
      const Vec3f Ng = ray_org+t1*ray_dir-sphere.p;
      RTCRayN_Ng_x(rays,N,ui) = Ng.x;
      RTCRayN_Ng_y(rays,N,ui) = Ng.y;
      RTCRayN_Ng_z(rays,N,ui) = Ng.z;
    }
    
#endif
  }
}

unmasked void sphereOccludedFuncN(const RTCOccludedFunctionNArguments* uniform const args)
{
  uniform int* uniform valid = args->valid;
  void* uniform ptr  = args->geomUserPtr;
  RTCRayN* uniform rays = args->ray;
  uniform unsigned int N = args->N;
  uniform unsigned int item = args->item;
  const uniform Sphere* uniform spheres = (const uniform Sphere* uniform) ptr;

  /* iterate over all rays in ray packet */
  for (uniform unsigned int ui=0; ui<N; ui+=programCount)
  {
    /* calculate varying loop and execution mask */
    unsigned int vi = ui+programIndex;
    if (vi>=N) continue;

    /* ignore inactive rays */
    if (valid[vi] != -1) continue;

    const Vec3f ray_org = make_Vec3f(RTCRayN_org_x(rays,N,ui),RTCRayN_org_y(rays,N,ui),RTCRayN_org_z(rays,N,ui));
    const Vec3f ray_dir = make_Vec3f(RTCRayN_dir_x(rays,N,ui),RTCRayN_dir_y(rays,N,ui),RTCRayN_dir_z(rays,N,ui));
    float& ray_tnear = RTCRayN_tnear(rays,N,ui);
    float& ray_tfar = RTCRayN_tfar(rays,N,ui);

    const uniform Sphere& sphere = spheres[item];
    const Vec3f v = ray_org-sphere.p;
    const float A = dot(ray_dir,ray_dir);
    const float B = 2.0f*dot(v,ray_dir);
    const float C = dot(v,v) - sqr(sphere.r);
    const float D = B*B - 4.0f*A*C;
    if (D < 0.0f) continue;
    const float Q = sqrt(D);
    const float rcpA = rcp(A);
    const float t0 = 0.5f*rcpA*(-B-Q);
    const float t1 = 0.5f*rcpA*(-B+Q);

#if 1
    RTCRay ray = RTCRayNtoRTCRay(args->ray,N,ui);

    RTCHit potentialhit;
    potentialhit.u = 0.0f;
    potentialhit.v = 0.0f;
    potentialhit.instID = args->context->instID;
    potentialhit.geomID = sphere.geomID;
    potentialhit.primID = item;
    if ((ray_tnear < t0) & (t0 < ray_tfar))
    {
      varying int imask;
      varying bool mask = __mask;
      unmasked {
        imask = mask ? -1 : 0;
      }
      const Vec3f Ng = ray_org+t0*ray_dir-sphere.p;
      potentialhit.t = t0;
      potentialhit.Ng_x = Ng.x;
      potentialhit.Ng_y = Ng.y;
      potentialhit.Ng_z = Ng.z;

      uniform RTCFilterFunctionNArguments fargs;
      fargs.valid = (int* uniform)&imask;
      fargs.geomUserPtr = ptr;
      fargs.context = args->context;
      fargs.ray = (RTCRayN* uniform)&ray;
      fargs.potentialHit = (RTCHitN* uniform)&potentialhit;
      fargs.N = programCount;
  
      rtcReportOcclusion(args,&fargs);

      /* update for all accepted hits */
      if (imask == -1)
        RTCRayN_geomID(rays,N,ui) = 0;
    }

    /* ignore rays that have just found a hit */
    if (RTCRayN_geomID(rays,N,ui) == 0)
      continue;
 
    if ((ray_tnear < t1) & (t1 < ray_tfar))
    {
      varying int imask;
      varying bool mask = __mask;
      unmasked {
        imask = mask ? -1 : 0;
      }
      const Vec3f Ng = ray_org+t1*ray_dir-sphere.p;
      potentialhit.t = t1;
      potentialhit.Ng_x = Ng.x;
      potentialhit.Ng_y = Ng.y;
      potentialhit.Ng_z = Ng.z;

      uniform RTCFilterFunctionNArguments fargs;
      fargs.valid = (int* uniform)&imask;
      fargs.geomUserPtr = ptr;
      fargs.context = args->context;
      fargs.ray = (RTCRayN* uniform)&ray;
      fargs.potentialHit = (RTCHitN* uniform)&potentialhit;
      fargs.N = programCount;
  
      rtcReportOcclusion(args,&fargs);

      /* update for all accepted hits */
      if (imask == -1)
        RTCRayN_geomID(rays,N,ui) = 0;
    }
 
#else
    
    if ((ray_tnear < t0) & (t0 < ray_tfar)) {
      RTCRayN_geomID(rays,N,ui) = 0;
    }
    if ((ray_tnear < t1) & (t1 < ray_tfar)) {
      RTCRayN_geomID(rays,N,ui) = 0;
    }
#endif
  }
}

/* intersection filter function */

unmasked void sphereFilterFunction(const RTCFilterFunctionNArguments* uniform const args)
{
  uniform int* uniform valid = args->valid;
  const uniform IntersectContext* uniform context = (const uniform IntersectContext* uniform) args->context;
  varying struct Ray* uniform ray    = (varying struct Ray* uniform)args->ray;
  varying struct RTCHit* uniform hit = (varying struct RTCHit* uniform)args->potentialHit;
  const uniform unsigned int N = args->N;
  assert(N == programCount);


  /* avoid crashing when debug visualizations are used */
  if (context == NULL)
    return;

  /* ignore inactive rays */
  if (valid[programIndex] != -1) return;
  
  /* carve out parts of the sphere */
  const Vec3f h = ray->org+hit->t*ray->dir;
  float v = abs(sin(10.0f*h.x)*cos(10.0f*h.y)*sin(10.0f*h.z));
  float T = clamp((v-0.1f)*3.0f,0.0f,1.0f);

  /* reject some hits */
  if (T < 0.5f) valid[programIndex] = 0;
}

unmasked void sphereFilterFunctionN(const RTCFilterFunctionNArguments* uniform const args)
{
  uniform int* uniform valid = args->valid;
  const uniform IntersectContext* uniform context = (const uniform IntersectContext* uniform) args->context;
  struct RTCRayN* uniform ray = args->ray;
  struct RTCHitN* uniform potentialHit = args->potentialHit;
  const uniform unsigned int N = args->N;
               
  /* avoid crashing when debug visualizations are used */
  if (context == NULL)
    return;

  /* iterate over all rays in ray packet */
  for (uniform unsigned int ui=0; ui<N; ui+=programCount)
  {
    /* calculate varying loop and execution mask */
    unsigned int vi = ui+programIndex;
    if (vi>=N) continue;

    /* ignore inactive rays */
    if (valid[vi] != -1) continue;
    
    /* calculate hit point */
    Vec3f ray_org = make_Vec3f(RTCRayN_org_x(ray,N,ui),RTCRayN_org_y(ray,N,ui),RTCRayN_org_z(ray,N,ui));
    Vec3f ray_dir = make_Vec3f(RTCRayN_dir_x(ray,N,ui),RTCRayN_dir_y(ray,N,ui),RTCRayN_dir_z(ray,N,ui));
    float hit_t = RTCHitN_t(potentialHit,N,ui);

    /* carve out parts of the sphere */
    const Vec3f h = ray_org+hit_t*ray_dir;
    float v = abs(sin(10.0f*h.x)*cos(10.0f*h.y)*sin(10.0f*h.z));
    float T = clamp((v-0.1f)*3.0f,0.0f,1.0f);

    /* reject some hits */
    if (T < 0.5f) valid[vi] = 0;
  }
}

uniform Sphere* uniform createAnalyticalSphere (RTCScene scene, const uniform Vec3f& p, uniform float r)
{
  RTCGeometry geom = rtcNewUserGeometry(g_device);
  uniform Sphere* uniform sphere = uniform new uniform Sphere;
  sphere->p = p;
  sphere->r = r;
  sphere->geometry = geom;
  sphere->geomID = rtcAttachGeometry(scene,geom);
  rtcSetNumPrimitives(geom,1);
  rtcSetUserData(geom,sphere);
  rtcSetBoundsFunction(geom,sphereBoundsFunc,NULL);
  rtcSetIntersectFunction(geom,sphereIntersectFunc);
  rtcSetOccludedFunction (geom,sphereOccludedFunc);
  rtcCommitGeometry(geom);
  rtcReleaseGeometry(geom);
  return sphere;
}

uniform Sphere* uniform createAnalyticalSpheres (RTCScene scene, uniform size_t N)
{
  RTCGeometry geom = rtcNewUserGeometry(g_device);
  uniform Sphere* uniform spheres = uniform new uniform Sphere[N];
  uniform unsigned int geomID = rtcAttachGeometry(scene,geom);
  for (uniform size_t i=0; i<N; i++) {
    spheres[i].geometry = geom;
    spheres[i].geomID = geomID;
  }
  rtcSetNumPrimitives(geom,N);
  rtcSetUserData(geom,spheres);
  rtcSetBoundsFunction(geom,sphereBoundsFunc,NULL);
  if (g_mode == MODE_NORMAL)
  {
    rtcSetIntersectFunction(geom,sphereIntersectFunc);
    rtcSetOccludedFunction (geom,sphereOccludedFunc);
    rtcSetIntersectionFilterFunction(geom,sphereFilterFunction);
    rtcSetOcclusionFilterFunction(geom,sphereFilterFunction);
  }
  else
  {
    rtcSetIntersectFunction(geom,sphereIntersectFuncN);
    rtcSetOccludedFunction (geom,sphereOccludedFuncN);
    rtcSetIntersectionFilterFunction(geom,sphereFilterFunctionN);
    rtcSetOcclusionFilterFunction(geom,sphereFilterFunctionN);
  }
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
  RTCGeometry geom = rtcNewTriangleMesh (g_device);

  /* map triangle and vertex buffers */
  uniform Vertex* uniform vertices = (uniform Vertex* uniform) rtcNewBuffer(geom,RTC_VERTEX_BUFFER,sizeof(uniform Vertex),numTheta*(numPhi+1));
  uniform Triangle* uniform triangles = (uniform Triangle* uniform) rtcNewBuffer(geom,RTC_INDEX_BUFFER,sizeof(uniform Triangle),2*numTheta*(numPhi-1));

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
  RTCGeometry geom = rtcNewTriangleMesh (g_device);

  /* set vertices */
  uniform Vertex* uniform vertices = (uniform Vertex* uniform) rtcNewBuffer(geom,RTC_VERTEX_BUFFER,sizeof(uniform Vertex),4);
  vertices[0].x = -10; vertices[0].y = -2; vertices[0].z = -10;
  vertices[1].x = -10; vertices[1].y = -2; vertices[1].z = +10;
  vertices[2].x = +10; vertices[2].y = -2; vertices[2].z = -10;
  vertices[3].x = +10; vertices[3].y = -2; vertices[3].z = +10;

  /* set triangles */
  uniform Triangle* uniform triangles = (uniform Triangle* uniform) rtcNewBuffer(geom,RTC_INDEX_BUFFER,sizeof(uniform Triangle),2);
  triangles[0].v0 = 0; triangles[0].v1 = 2; triangles[0].v2 = 1;
  triangles[1].v0 = 1; triangles[1].v1 = 2; triangles[1].v2 = 3;

  rtcCommitGeometry(geom);
  uniform unsigned int geomID = rtcAttachGeometry(scene,geom);
  rtcReleaseGeometry(geom);
  return geomID;
}

/* scene data */
RTCScene g_scene  = NULL;
RTCScene g_scene0 = NULL;
RTCScene g_scene1 = NULL;
RTCScene g_scene2 = NULL;
uniform Sphere* uniform g_spheres = NULL;
uniform Sphere* uniform g_sphere0 = NULL;
uniform Sphere* uniform g_sphere1 = NULL;

uniform Instance* uniform g_instance[4] = { NULL, NULL, NULL, NULL };

uniform Vec3f colors[5][4];

/* called by the C++ code for initialization */
export void device_init (uniform int8* uniform cfg)
{
  /* create new Embree device */
  g_device = rtcNewDevice(cfg);
  error_handler(NULL,rtcDeviceGetError(g_device));

  /* set error handler */
  rtcDeviceSetErrorFunction(g_device,error_handler,NULL);

  /* create scene */
  g_scene = rtcDeviceNewScene(g_device);

  /* create scene with 4 analytical spheres */
  g_scene0 = rtcDeviceNewScene(g_device);
  g_spheres = createAnalyticalSpheres(g_scene0,4);
  g_spheres[0].p = make_Vec3f( 0, 0,+1); g_spheres[0].r = 0.5f;
  g_spheres[1].p = make_Vec3f(+1, 0, 0); g_spheres[1].r = 0.5f;
  g_spheres[2].p = make_Vec3f( 0, 0,-1); g_spheres[2].r = 0.5f;
  g_spheres[3].p = make_Vec3f(-1, 0, 0); g_spheres[3].r = 0.5f;
  rtcCommit(g_scene0);

  /* create scene with 4 triangulated spheres */
  g_scene1 = rtcDeviceNewScene(g_device);
  createTriangulatedSphere(g_scene1,make_Vec3f( 0, 0,+1),0.5f);
  createTriangulatedSphere(g_scene1,make_Vec3f(+1, 0, 0),0.5f);
  createTriangulatedSphere(g_scene1,make_Vec3f( 0, 0,-1),0.5f);
  createTriangulatedSphere(g_scene1,make_Vec3f(-1, 0, 0),0.5f);
  rtcCommit(g_scene1);

  /* create scene with 2 triangulated and 2 analytical spheres */
  g_scene2 = rtcDeviceNewScene(g_device);
  createTriangulatedSphere(g_scene2,make_Vec3f( 0, 0,+1),0.5f);
  g_sphere0 = createAnalyticalSphere  (g_scene2,make_Vec3f(+1, 0, 0),0.5f);
  createTriangulatedSphere(g_scene2,make_Vec3f( 0, 0,-1),0.5f);
  g_sphere1 = createAnalyticalSphere  (g_scene2,make_Vec3f(-1, 0, 0),0.5f);
  rtcCommit(g_scene2);

  /* instantiate geometry */
  g_instance[0] = createInstance(g_scene,g_scene0,0,make_Vec3f(-2,-2,-2),make_Vec3f(+2,+2,+2));
  g_instance[1] = createInstance(g_scene,g_scene1,1,make_Vec3f(-2,-2,-2),make_Vec3f(+2,+2,+2));
  g_instance[2] = createInstance(g_scene,g_scene2,2,make_Vec3f(-2,-2,-2),make_Vec3f(+2,+2,+2));
  g_instance[3] = createInstance(g_scene,g_scene2,3,make_Vec3f(-2,-2,-2),make_Vec3f(+2,+2,+2));
  createGroundPlane(g_scene);
  rtcCommit(g_scene);

  /* set all colors */
  colors[0][0] = make_Vec3f(0.25f, 0.00f, 0.00f);
  colors[0][1] = make_Vec3f(0.50f, 0.00f, 0.00f);
  colors[0][2] = make_Vec3f(0.75f, 0.00f, 0.00f);
  colors[0][3] = make_Vec3f(1.00f, 0.00f, 0.00f);

  colors[1][0] = make_Vec3f(0.00f, 0.25f, 0.00f);
  colors[1][1] = make_Vec3f(0.00f, 0.50f, 0.00f);
  colors[1][2] = make_Vec3f(0.00f, 0.75f, 0.00f);
  colors[1][3] = make_Vec3f(0.00f, 1.00f, 0.00f);

  colors[2][0] = make_Vec3f(0.00f, 0.00f, 0.25f);
  colors[2][1] = make_Vec3f(0.00f, 0.00f, 0.50f);
  colors[2][2] = make_Vec3f(0.00f, 0.00f, 0.75f);
  colors[2][3] = make_Vec3f(0.00f, 0.00f, 1.00f);

  colors[3][0] = make_Vec3f(0.25f, 0.25f, 0.00f);
  colors[3][1] = make_Vec3f(0.50f, 0.50f, 0.00f);
  colors[3][2] = make_Vec3f(0.75f, 0.75f, 0.00f);
  colors[3][3] = make_Vec3f(1.00f, 1.00f, 0.00f);

  colors[4][0] = make_Vec3f(1.0f, 1.0f, 1.0f);
  colors[4][1] = make_Vec3f(1.0f, 1.0f, 1.0f);
  colors[4][2] = make_Vec3f(1.0f, 1.0f, 1.0f);
  colors[4][3] = make_Vec3f(1.0f, 1.0f, 1.0f);

  /* set start render mode */
  if (g_mode == MODE_NORMAL) renderTile = renderTileStandard;
  else                       renderTile = renderTileStandardStream;
  key_pressed_handler = device_key_pressed_default;
}

inline Vec3f face_forward(const Vec3f& dir, const Vec3f& _Ng) {
  const Vec3f Ng = _Ng;
  return dot(dir,Ng) < 0.0f ? Ng : neg(Ng);
}

/* task that renders a single screen tile */
Vec3f renderPixelStandard(float x, float y, const uniform ISPCCamera& camera, uniform RayStats& stats)
{
  uniform RTCIntersectContext context;
  rtcInitIntersectionContext(&context);
  
  /* initialize ray */
  Ray ray = make_Ray(make_Vec3f(camera.xfm.p), 
                     make_Vec3f(normalize(x*camera.xfm.l.vx + y*camera.xfm.l.vy + camera.xfm.l.vz)), 
                     0.0f, inf, 0.0f, -1,
                     RTC_INVALID_GEOMETRY_ID, RTC_INVALID_GEOMETRY_ID, RTC_INVALID_GEOMETRY_ID);

  /* intersect ray with scene */
  rtcIntersect(g_scene,&context,RTCRay_(ray));
  RayStats_addRay(stats);

  /* shade pixels */
  Vec3f color = make_Vec3f(0.0f);
  if (ray.geomID != RTC_INVALID_GEOMETRY_ID)
  {
    /* calculate shading normal in world space */
    Vec3f Ns = ray.Ng;

    if (ray.instID != RTC_INVALID_GEOMETRY_ID) {
      Ns = xfmVector(g_instance[ray.instID]->normal2world,make_Vec3f(Ns));
    }
    Ns = face_forward(ray.dir,normalize(Ns));

    /* calculate diffuse color of geometries */
    Vec3f diffuse = make_Vec3f(0.0f);
    if      (ray.instID ==  0) diffuse = colors[ray.instID][ray.primID];
    else if (ray.instID == -1) diffuse = colors[4][ray.primID];
    else                       diffuse = colors[ray.instID][ray.geomID];
    color = color + diffuse*0.5;

    /* initialize shadow ray */
    Vec3f lightDir = normalize(make_Vec3f(-1,-1,-1));
    Ray shadow = make_Ray(ray.org + 0.999f*ray.tfar*ray.dir, neg(lightDir), 0.001f, inf);

    /* trace shadow ray */
    rtcOccluded(g_scene,&context,RTCRay_(shadow));
    RayStats_addShadowRay(stats);

    /* add light contribution */
    if (shadow.geomID)
      color = color + diffuse*clamp(-dot(lightDir,Ns),0.0f,1.0f);
  }
  return color;
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
    /* calculate pixel color */
    Vec3f color = renderPixelStandard((float)x,(float)y,camera,g_stats[threadIndex]);

    /* write color to framebuffer */
    unsigned int r = (unsigned int) (255.0f * clamp(color.x,0.0f,1.0f));
    unsigned int g = (unsigned int) (255.0f * clamp(color.y,0.0f,1.0f));
    unsigned int b = (unsigned int) (255.0f * clamp(color.z,0.0f,1.0f));
    pixels[y*width+x] = (b << 16) + (g << 8) + r;
  }
}

/* renders a single screen tile */
void renderTileStandardStream(uniform int taskIndex,
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

  uniform RayStats& stats = g_stats[threadIndex];

  Ray primary_stream[TILE_SIZE_X*TILE_SIZE_Y];
  Ray shadow_stream[TILE_SIZE_X*TILE_SIZE_Y];
  Vec3f color_stream[TILE_SIZE_X*TILE_SIZE_Y];
  bool valid_stream[TILE_SIZE_X*TILE_SIZE_Y];

  /* generate stream of primary rays */
  uniform int N = 0;
  foreach_tiled (y = y0 ... y1, x = x0 ... x1)
  {
    /* ISPC workaround for mask == 0 */
    if (all(__mask == 0)) continue;

    /* initialize variables */
    color_stream[N] = make_Vec3f(0.0f);
    bool mask = __mask; unmasked { valid_stream[N] = mask; }

    /* initialize ray */
    Ray& primary = primary_stream[N];
    mask = __mask; unmasked { // invalidates inactive rays
      primary.tnear = mask ? 0.0f         : (float)(pos_inf);
      primary.tfar  = mask ? (float)(inf) : (float)(neg_inf);
    }

    init_Ray(primary, make_Vec3f(camera.xfm.p), make_Vec3f(normalize((float)x*camera.xfm.l.vx + (float)y*camera.xfm.l.vy + camera.xfm.l.vz)), primary.tnear, primary.tfar, 0.0f, -1,
             RTC_INVALID_GEOMETRY_ID, RTC_INVALID_GEOMETRY_ID, RTC_INVALID_GEOMETRY_ID);
    N++;
    RayStats_addRay(stats);
  }

  Vec3f lightDir = normalize(make_Vec3f(-1,-1,-1));

  /* trace rays */
  uniform RTCIntersectContext primary_context;
  rtcInitIntersectionContext(&primary_context);
  primary_context.flags = g_iflags_coherent;
  rtcIntersectVM(g_scene,&primary_context,(varying RTCRay* uniform)&primary_stream,N,sizeof(Ray));

  /* terminate rays and update color */
  N = -1;
  foreach_tiled (y = y0 ... y1, x = x0 ... x1)
  {
    N++;
    /* ISPC workaround for mask == 0 */
    if (all(__mask == 0)) continue;

    /* invalidate shadow rays by default */
    Ray& shadow = shadow_stream[N];
    unmasked {
      shadow.tnear = (float)(pos_inf);
      shadow.tfar  = (float)(neg_inf);
    }

    /* ignore invalid rays */
    if (valid_stream[N] == false) continue;

    /* terminate rays that hit nothing */
    if (primary_stream[N].geomID == RTC_INVALID_GEOMETRY_ID) {
      valid_stream[N] = false;
      continue;
    }

    /* calculate diffuse color of geometries */
    Ray& primary = primary_stream[N];
    Vec3f diffuse = make_Vec3f(0.0f);
    if      (primary.instID ==  0) diffuse = colors[primary.instID][primary.primID];
    else if (primary.instID == -1) diffuse = colors[4][primary.primID];      
    else                           diffuse = colors[primary.instID][primary.geomID];
    color_stream[N] = color_stream[N] + diffuse*0.5;

    /* initialize shadow ray */
    bool mask = __mask; unmasked {
      shadow.tnear = mask ? 0.001f       : (float)(pos_inf);
      shadow.tfar  = mask ? (float)(inf) : (float)(neg_inf);
    }
    init_Ray(shadow,primary.org + primary.tfar*primary.dir, neg(lightDir), shadow.tnear, shadow.tfar, 0.0f, N*programCount + programIndex);

    RayStats_addShadowRay(stats);
  }
  N++;

  /* trace shadow rays */
  uniform RTCIntersectContext shadow_context;
  rtcInitIntersectionContext(&shadow_context);
  shadow_context.flags = g_iflags_coherent;
  rtcOccludedVM(g_scene,&shadow_context,(varying RTCRay* uniform)&shadow_stream,N,sizeof(Ray));

  /* add light contribution */
  N = -1;
  foreach_tiled (y = y0 ... y1, x = x0 ... x1)
  {
    N++;
    /* ISPC workaround for mask == 0 */
    if (all(__mask == 0)) continue;

    /* ignore invalid rays */
    if (valid_stream[N] == false) continue;

    /* calculate shading normal in world space */
    Ray& primary = primary_stream[N];
    Vec3f Ns = primary.Ng;
    if (primary.instID != RTC_INVALID_GEOMETRY_ID && primary.instID != 4) {
      Ns = xfmVector(g_instance[primary.instID]->normal2world,make_Vec3f(Ns));
    }
    Ns = face_forward(primary.dir,normalize(Ns));
    
    /* add light contrinution */
    Vec3f diffuse = make_Vec3f(0.0f);
    if      (primary.instID ==  0) diffuse = colors[primary.instID][primary.primID];
    else if (primary.instID == -1) diffuse = colors[4][primary.primID];      
    else                           diffuse = colors[primary.instID][primary.geomID];
    Ray& shadow = shadow_stream[N];
    if (shadow.geomID) {
      color_stream[N] = color_stream[N] + diffuse*clamp(-dot(lightDir,Ns),0.0f,1.0f);
    }
  }
  N++;

  /* framebuffer writeback */
  N = 0;
  foreach_tiled (y = y0 ... y1, x = x0 ... x1)
  {
    /* ISPC workaround for mask == 0 */
    if (all(__mask == 0)) continue;

    /* write color to framebuffer */
    unsigned int r = (unsigned int) (255.0f * clamp(color_stream[N].x,0.0f,1.0f));
    unsigned int g = (unsigned int) (255.0f * clamp(color_stream[N].y,0.0f,1.0f));
    unsigned int b = (unsigned int) (255.0f * clamp(color_stream[N].z,0.0f,1.0f));
    pixels[y*width+x] = (b << 16) + (g << 8) + r;
    N++;
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
  renderTile(taskIndex,threadIndex,pixels,width,height,time,camera,numTilesX,numTilesY);
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
  g_instance[0]->local2world = make_AffineSpace3f(xfm,2.2f*make_Vec3f(+cos(t0),0.0f,+sin(t0)));
  g_instance[1]->local2world = make_AffineSpace3f(xfm,2.2f*make_Vec3f(-cos(t0),0.0f,-sin(t0)));
  g_instance[2]->local2world = make_AffineSpace3f(xfm,2.2f*make_Vec3f(-sin(t0),0.0f,+cos(t0)));
  g_instance[3]->local2world = make_AffineSpace3f(xfm,2.2f*make_Vec3f(+sin(t0),0.0f,-cos(t0)));

  /* update scene */
  updateInstance(g_scene,g_instance[0]);
  updateInstance(g_scene,g_instance[1]);
  updateInstance(g_scene,g_instance[2]);
  updateInstance(g_scene,g_instance[3]);
  rtcCommit (g_scene);

  /* render all pixels */
  const uniform int numTilesX = (width +TILE_SIZE_X-1)/TILE_SIZE_X;
  const uniform int numTilesY = (height+TILE_SIZE_Y-1)/TILE_SIZE_Y;
  launch[numTilesX*numTilesY] renderTileTask(pixels,width,height,time,camera,numTilesX,numTilesY); sync;
}

/* called by the C++ code for cleanup */
export void device_cleanup ()
{
  rtcReleaseScene (g_scene); g_scene = NULL;
  rtcReleaseScene (g_scene0); g_scene0 = NULL;
  rtcReleaseScene (g_scene1); g_scene1 = NULL;
  rtcReleaseScene (g_scene2); g_scene2 = NULL;
  rtcReleaseDevice(g_device); g_device = NULL;
  delete[] g_spheres; g_spheres = NULL;
  delete g_sphere0; g_sphere0 = NULL;
  delete g_sphere1; g_sphere1 = NULL;
}
