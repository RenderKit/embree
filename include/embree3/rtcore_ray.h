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

#pragma once

#include "rtcore_common.h"

#if defined(__cplusplus)
extern "C" {
#endif
  
/* Ray structure for an individual ray */
struct RTC_ALIGN(16) RTCRay
{
  /* ray data */
  float org_x;         // x coordinate of ray origin
  float org_y;         // y coordinate of ray origin
  float org_z;         // z coordinate of ray origin
  float tnear;         // start of ray segment

  float dir_x;         // x coordinate of ray direction
  float dir_y;         // y coordinate of ray direction
  float dir_z;         // z coordinate of ray direction
  float tfar;          // end of ray segment (set to hit distance)

  float time;          // time of this ray for motion blur
  unsigned int mask;   // ray mask
  unsigned int id;     // ray ID
  unsigned int flags;  // ray flags
  
  /* hit data */
  float Ng_x;          // x coordinate of geometry normal
  float Ng_y;          // y coordinate of geometry normal
  float Ng_z;          // z coordinate of geometry normal
   
  float u;             // barycentric u coordinate of hit
  float v;             // barycentric v coordinate of hit

  unsigned int primID; // geometry ID
  unsigned int geomID; // primitive ID
  unsigned int instID[RTC_MAX_INSTANCE_LEVELS];   // instance ID
};

/* Ray structure for packets of 4 rays. */
struct RTC_ALIGN(16) RTCRay4
{
  /* ray data */
  float org_x[4];
  float org_y[4];
  float org_z[4];
  float tnear[4];
  
  float dir_x[4];
  float dir_y[4];
  float dir_z[4];
  float tfar[4];

  float time[4];
  unsigned int mask[4];
  unsigned int id[4];
  unsigned int flags[4];
  
  /* hit data */
  float Ng_x[4];
  float Ng_y[4];
  float Ng_z[4];
  
  float u[4];
  float v[4];
  
  unsigned int primID[4];
  unsigned int geomID[4];
  unsigned int instID[RTC_MAX_INSTANCE_LEVELS][4];
};

/* Ray structure for packets of 8 rays. */
struct RTC_ALIGN(32) RTCRay8
{
  /* ray data */
  float org_x[8];
  float org_y[8];
  float org_z[8];
  float tnear[8];
  
  float dir_x[8];
  float dir_y[8];
  float dir_z[8];
  float tfar[8];

  float time[8];
  unsigned int mask[8];
  unsigned int id[8];
  unsigned int flags[8];
  
  /* hit data */
  float Ng_x[8];
  float Ng_y[8];
  float Ng_z[8];
  
  float u[8];
  float v[8];
  
  unsigned int primID[8];
  unsigned int geomID[8];
  unsigned int instID[RTC_MAX_INSTANCE_LEVELS][8];
};

/* Ray structure for packets of 16 rays. */
struct RTC_ALIGN(64) RTCRay16
{
  /* ray data */
  float org_x[16];
  float org_y[16];
  float org_z[16];
  float tnear[16];
  
  float dir_x[16];
  float dir_y[16];
  float dir_z[16];
  float tfar[16];

  float time[16];
  unsigned int mask[16];
  unsigned int id[16];
  unsigned int flags[16];

  /* hit data */
  float Ng_x[16];
  float Ng_y[16];
  float Ng_z[16];
  
  float u[16];
  float v[16];
  
  unsigned int primID[16];
  unsigned int geomID[16];
  unsigned int instID[RTC_MAX_INSTANCE_LEVELS][16];
};

/* Ray structure template for packets of N rays in pointer SOA layout. */
struct RTCRayNp
{
  /* ray data */
  float* org_x;
  float* org_y;
  float* org_z;
  float* tnear;

  float* dir_x;
  float* dir_y;
  float* dir_z;
  float* tfar;
 
  float* time;
  unsigned int* mask;
  unsigned int* id;
  unsigned int* flags;

  /* hit data */
  float* Ng_x;
  float* Ng_y;
  float* Ng_z;

  float* u;
  float* v;

  unsigned int* primID;
  unsigned int* geomID;
  unsigned int* instID[RTC_MAX_INSTANCE_LEVELS];
};

struct RTCHit
{
  float Ng_x;
  float Ng_y;
  float Ng_z;

  float u;
  float v;

  unsigned primID;
  unsigned geomID;
  unsigned instID[RTC_MAX_INSTANCE_LEVELS];
};
  
struct RTCRayN;
struct RTCHitN;

#if defined(__cplusplus)
}
#endif

#if defined(__cplusplus)

/* Helper functions to access ray packets of runtime size N */
RTC_FORCEINLINE float& RTCRayN_org_x(RTCRayN* ptr, unsigned int N, unsigned int i) { return ((float*)ptr)[0*N+i]; }
RTC_FORCEINLINE float& RTCRayN_org_y(RTCRayN* ptr, unsigned int N, unsigned int i) { return ((float*)ptr)[1*N+i]; }
RTC_FORCEINLINE float& RTCRayN_org_z(RTCRayN* ptr, unsigned int N, unsigned int i) { return ((float*)ptr)[2*N+i]; }
RTC_FORCEINLINE float& RTCRayN_tnear(RTCRayN* ptr, unsigned int N, unsigned int i) { return ((float*)ptr)[3*N+i]; }

RTC_FORCEINLINE float& RTCRayN_dir_x(RTCRayN* ptr, unsigned int N, unsigned int i) { return ((float*)ptr)[4*N+i]; }
RTC_FORCEINLINE float& RTCRayN_dir_y(RTCRayN* ptr, unsigned int N, unsigned int i) { return ((float*)ptr)[5*N+i]; }
RTC_FORCEINLINE float& RTCRayN_dir_z(RTCRayN* ptr, unsigned int N, unsigned int i) { return ((float*)ptr)[6*N+i]; }
RTC_FORCEINLINE float& RTCRayN_tfar (RTCRayN* ptr, unsigned int N, unsigned int i) { return ((float*)ptr)[7*N+i]; }

RTC_FORCEINLINE float&    RTCRayN_time(RTCRayN* ptr, unsigned int N, unsigned int i) { return ((float*)   ptr)[8*N+i]; }
RTC_FORCEINLINE unsigned& RTCRayN_mask(RTCRayN* ptr, unsigned int N, unsigned int i) { return ((unsigned*)ptr)[9*N+i]; }

RTC_FORCEINLINE unsigned& RTCRayN_id   (RTCRayN* ptr, unsigned int N, unsigned int i) { return ((unsigned*)ptr)[10*N+i]; }
RTC_FORCEINLINE unsigned& RTCRayN_flags(RTCRayN* ptr, unsigned int N, unsigned int i) { return ((unsigned*)ptr)[11*N+i]; }

RTC_FORCEINLINE float& RTCRayN_Ng_x(RTCRayN* ptr, unsigned int N, unsigned int i) { return ((float*)ptr)[12*N+i]; }
RTC_FORCEINLINE float& RTCRayN_Ng_y(RTCRayN* ptr, unsigned int N, unsigned int i) { return ((float*)ptr)[13*N+i]; }
RTC_FORCEINLINE float& RTCRayN_Ng_z(RTCRayN* ptr, unsigned int N, unsigned int i) { return ((float*)ptr)[14*N+i]; }

RTC_FORCEINLINE float& RTCRayN_u(RTCRayN* ptr, unsigned int N, unsigned int i) { return ((float*)ptr)[15*N+i]; }
RTC_FORCEINLINE float& RTCRayN_v(RTCRayN* ptr, unsigned int N, unsigned int i) { return ((float*)ptr)[16*N+i]; }

RTC_FORCEINLINE unsigned& RTCRayN_primID(RTCRayN* ptr, unsigned int N, unsigned int i) { return ((unsigned*)ptr)[17*N+i]; }
RTC_FORCEINLINE unsigned& RTCRayN_geomID(RTCRayN* ptr, unsigned int N, unsigned int i) { return ((unsigned*)ptr)[18*N+i]; }
RTC_FORCEINLINE unsigned& RTCRayN_instID(RTCRayN* ptr, unsigned int N, unsigned int i, unsigned int level) { return ((unsigned*)ptr)[19*N+i+N*level]; }

/* Helper functions to access hit packets of size N */
RTC_FORCEINLINE float& RTCHitN_Ng_x(const RTCHitN* ptr, unsigned int N, unsigned int i) { return ((float*)ptr)[0*N+i]; }
RTC_FORCEINLINE float& RTCHitN_Ng_y(const RTCHitN* ptr, unsigned int N, unsigned int i) { return ((float*)ptr)[1*N+i]; }
RTC_FORCEINLINE float& RTCHitN_Ng_z(const RTCHitN* ptr, unsigned int N, unsigned int i) { return ((float*)ptr)[2*N+i]; }

RTC_FORCEINLINE float& RTCHitN_u(const RTCHitN* ptr, unsigned int N, unsigned int i) { return ((float*)ptr)[3*N+i]; }
RTC_FORCEINLINE float& RTCHitN_v(const RTCHitN* ptr, unsigned int N, unsigned int i) { return ((float*)ptr)[4*N+i]; }

RTC_FORCEINLINE unsigned& RTCHitN_primID(const RTCHitN* ptr, unsigned int N, unsigned int i) { return ((unsigned*)ptr)[5*N+i]; }
RTC_FORCEINLINE unsigned& RTCHitN_geomID(const RTCHitN* ptr, unsigned int N, unsigned int i) { return ((unsigned*)ptr)[6*N+i]; }
RTC_FORCEINLINE unsigned& RTCHitN_instID(const RTCHitN* ptr, unsigned int N, unsigned int i, unsigned int l) { return ((unsigned*)ptr)[7*N+i+N*l]; }


/* Helper structure to create a ray packet of compile time size N */
template<int N>
struct RTCRayNt
{
  /* ray data */
public:
  float org_x[N];
  float org_y[N];
  float org_z[N];
  float tnear[N];
  
  float dir_x[N];
  float dir_y[N];
  float dir_z[N];
  float tfar[N];

  float time[N];
  unsigned mask[N];
  
  unsigned id[N];
  unsigned flags[N];

  /* hit data */
public:
  float Ng_x[N];
  float Ng_y[N];
  float Ng_z[N];
  
  float u[N];
  float v[N];

  unsigned primID[N];
  unsigned geomID[N];
  unsigned instID[RTC_MAX_INSTANCE_LEVELS][N];
};

/* Helper structure to create a hit packet of compile time size N */
template<int N>
struct RTCHitNt
{
  float Ng_x[N];
  float Ng_y[N];
  float Ng_z[N];

  float u[N];
  float v[N];

  unsigned primID[N];
  unsigned geomID[N];
  unsigned instID[RTC_MAX_INSTANCE_LEVELS][N];
};

RTC_FORCEINLINE RTCRay rtcGetRayFromRayN(RTCRayN* rays, unsigned int N, unsigned int i)
{
  RTCRay ray;
  ray.org_x  = RTCRayN_org_x(rays,N,i);
  ray.org_y  = RTCRayN_org_y(rays,N,i);
  ray.org_z  = RTCRayN_org_z(rays,N,i);
  ray.tnear  = RTCRayN_tnear(rays,N,i);
  ray.dir_x  = RTCRayN_dir_x(rays,N,i);
  ray.dir_y  = RTCRayN_dir_y(rays,N,i);
  ray.dir_z  = RTCRayN_dir_z(rays,N,i);
  ray.tfar   = RTCRayN_tfar(rays,N,i);
  ray.time   = RTCRayN_time(rays,N,i);
  ray.mask   = RTCRayN_mask(rays,N,i);
  ray.id     = RTCRayN_id(rays,N,i);
  ray.flags  = RTCRayN_flags(rays,N,i);
  ray.Ng_x   = RTCRayN_Ng_x(rays,N,i);
  ray.Ng_y   = RTCRayN_Ng_y(rays,N,i);
  ray.Ng_z   = RTCRayN_Ng_z(rays,N,i);
  ray.u      = RTCRayN_u(rays,N,i);
  ray.v      = RTCRayN_v(rays,N,i);
  ray.primID = RTCRayN_primID(rays,N,i);
  ray.geomID = RTCRayN_geomID(rays,N,i);
  for (size_t l = 0; l < RTC_MAX_INSTANCE_LEVELS; l++)
    ray.instID[l] = RTCRayN_instID(rays,N,i,l);
  return ray;
};

RTC_FORCEINLINE RTCHit rtcGetHitFromHitN(RTCHitN* hits, unsigned int N, unsigned int i)
{
  RTCHit hit;
  hit.Ng_x   = RTCHitN_Ng_x(hits,N,i);
  hit.Ng_y   = RTCHitN_Ng_y(hits,N,i);
  hit.Ng_z   = RTCHitN_Ng_z(hits,N,i);
  hit.u      = RTCHitN_u(hits,N,i);
  hit.v      = RTCHitN_v(hits,N,i);
  hit.primID = RTCHitN_primID(hits,N,i);
  hit.geomID = RTCHitN_geomID(hits,N,i);
  for (size_t l = 0; l < RTC_MAX_INSTANCE_LEVELS; l++)
    hit.instID[l] = RTCHitN_instID(hits,N,i,l);
  return hit;
};

RTC_FORCEINLINE void rtcCopyHitToRay(RTCRay* ray, const RTCHit* hit, float t)
{
  ray->Ng_x = hit->Ng_x;
  ray->Ng_y = hit->Ng_y;
  ray->Ng_z = hit->Ng_z;  
  ray->tfar = t;
  ray->u = hit->u;
  ray->v = hit->v;
  ray->primID = hit->primID;
  ray->geomID = hit->geomID;
  for (size_t l = 0; l < RTC_MAX_INSTANCE_LEVELS; l++)
    ray->instID[l] = hit->instID[l];
}

RTC_FORCEINLINE void rtcCopyHitToRayN(RTCRayN* rays, const RTCHit* hit, float t, unsigned int N, unsigned int i)
{
  RTCRayN_Ng_x(rays,N,i)   = hit->Ng_x;
  RTCRayN_Ng_y(rays,N,i)   = hit->Ng_y;
  RTCRayN_Ng_z(rays,N,i)   = hit->Ng_z;
  RTCRayN_tfar(rays,N,i)   = t;
  RTCRayN_u(rays,N,i)      = hit->u;
  RTCRayN_v(rays,N,i)      = hit->v;
  RTCRayN_primID(rays,N,i) = hit->primID;
  RTCRayN_geomID(rays,N,i) = hit->geomID;
  for (size_t l = 0; l < RTC_MAX_INSTANCE_LEVELS; l++)
    RTCRayN_instID(rays,N,i,l) = hit->instID[l];
}

RTC_FORCEINLINE void rtcCopyHitFromRayToRayN(RTCRayN* rays, const RTCRay* ray, unsigned int N, unsigned int i)
{
  RTCRayN_Ng_x(rays,N,i)   = ray->Ng_x;
  RTCRayN_Ng_y(rays,N,i)   = ray->Ng_y;
  RTCRayN_Ng_z(rays,N,i)   = ray->Ng_z;
  RTCRayN_u(rays,N,i)      = ray->u;
  RTCRayN_v(rays,N,i)      = ray->v;
  RTCRayN_primID(rays,N,i) = ray->primID;
  RTCRayN_geomID(rays,N,i) = ray->geomID;
  for (size_t l = 0 ; l < RTC_MAX_INSTANCE_LEVELS; l++)
    RTCRayN_instID(rays,N,i,l) = ray->instID[l];
}

#endif
