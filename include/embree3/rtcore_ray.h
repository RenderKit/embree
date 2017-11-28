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
  
/*! \ingroup embree_kernel_api */
/*! \{ */

/*! \brief Ray structure for an individual ray */
struct RTCORE_ALIGN(16) RTCRay
{
  /* ray data */
  float org_x;        //!< x coordinate of ray origin
  float org_y;        //!< y coordinate of ray origin
  float org_z;        //!< z coordinate of ray origin
  float tnear;       //!< Start of ray segment

  float dir_x;        //!< x coordinate of ray direction
  float dir_y;        //!< y coordinate of ray direction
  float dir_z;        //!< z coordinate of ray direction
  float tfar;        //!< End of ray segment (set to hit distance)

  float time;        //!< Time of this ray for motion blur
  unsigned mask;     //!< Used to mask out objects during traversal

  unsigned id;       //!< ray ID
  unsigned flags;    //!< per flags
  
  /* hit data */
  float Ng_x;         //!< x coordinate of geometry normal
  float Ng_y;         //!< y coordinate of geometry normal
  float Ng_z;         //!< z coordinate of geometry normal
   
  float u;           //!< Barycentric u coordinate of hit
  float v;           //!< Barycentric v coordinate of hit

  unsigned primID;   //!< geometry ID
  unsigned geomID;   //!< primitive ID
  unsigned instID;   //!< instance ID
};

/*! Ray structure for packets of 4 rays. */
struct RTCORE_ALIGN(16) RTCRay4
{
  /* ray data */
  float org_x[4];  //!< x coordinate of ray origin
  float org_y[4];  //!< y coordinate of ray origin
  float org_z[4];  //!< z coordinate of ray origin
  float tnear[4]; //!< Start of ray segment 
  
  float dir_x[4];  //!< x coordinate of ray direction
  float dir_y[4];  //!< y coordinate of ray direction
  float dir_z[4];  //!< z coordinate of ray direction
  float tfar[4];  //!< End of ray segment (set to hit distance)

  float time[4];  //!< Time of this ray for motion blur
  unsigned mask[4];  //!< Used to mask out objects during traversal

  unsigned id[4];       //!< ray ID
  unsigned flags[4];    //!< per flags
  
  /* hit data */
  float Ng_x[4];   //!< x coordinate of geometry normal
  float Ng_y[4];   //!< y coordinate of geometry normal
  float Ng_z[4];   //!< z coordinate of geometry normal
  
  float u[4];     //!< Barycentric u coordinate of hit
  float v[4];     //!< Barycentric v coordinate of hit
  
  unsigned primID[4];  //!< geometry ID
  unsigned geomID[4];  //!< primitive ID
  unsigned instID[4];  //!< instance ID
};

/*! Ray structure for packets of 8 rays. */
struct RTCORE_ALIGN(32) RTCRay8
{
  /* ray data */
  float org_x[8];  //!< x coordinate of ray origin
  float org_y[8];  //!< y coordinate of ray origin
  float org_z[8];  //!< z coordinate of ray origin
  float tnear[8]; //!< Start of ray segment 
  
  float dir_x[8];  //!< x coordinate of ray direction
  float dir_y[8];  //!< y coordinate of ray direction
  float dir_z[8];  //!< z coordinate of ray direction  
  float tfar[8];  //!< End of ray segment (set to hit distance)

  float time[8];  //!< Time of this ray for motion blur
  unsigned mask[8];  //!< Used to mask out objects during traversal

  unsigned id[8];       //!< ray ID
  unsigned flags[8];    //!< per flags
  
  /* hit data */
  float Ng_x[8];   //!< x coordinate of geometry normal
  float Ng_y[8];   //!< y coordinate of geometry normal
  float Ng_z[8];   //!< z coordinate of geometry normal
  
  float u[8];     //!< Barycentric u coordinate of hit
  float v[8];     //!< Barycentric v coordinate of hit
  
  unsigned primID[8];  //!< geometry ID
  unsigned geomID[8];  //!< primitive ID
  unsigned instID[8];  //!< instance ID
};

/*! \brief Ray structure for packets of 16 rays. */
struct RTCORE_ALIGN(64) RTCRay16
{
  /* ray data */
  float org_x[16];  //!< x coordinate of ray origin
  float org_y[16];  //!< y coordinate of ray origin
  float org_z[16];  //!< z coordinate of ray origin
  float tnear[16]; //!< Start of ray segment 
  
  float dir_x[16];  //!< x coordinate of ray direction
  float dir_y[16];  //!< y coordinate of ray direction
  float dir_z[16];  //!< z coordinate of ray direction  
  float tfar[16];  //!< End of ray segment (set to hit distance)

  float time[16];  //!< Time of this ray for motion blur
  unsigned mask[16];  //!< Used to mask out objects during traversal
  
  unsigned id[16];    //!< ray ID
  unsigned flags[16]; //!< per flags

  /* hit data */
  float Ng_x[16];   //!< x coordinate of geometry normal
  float Ng_y[16];   //!< y coordinate of geometry normal
  float Ng_z[16];   //!< z coordinate of geometry normal
  
  float u[16];     //!< Barycentric u coordinate of hit
  float v[16];     //!< Barycentric v coordinate of hit
  
  unsigned primID[16];  //!< geometry ID
  unsigned geomID[16];  //!< primitive ID
  unsigned instID[16];  //!< instance ID
};

/*! \brief Ray structure template for packets of N rays in pointer SOA layout. */
struct RTCRayNp
{
  /* ray data */
  float* org_x;  //!< x coordinate of ray origin
  float* org_y;  //!< y coordinate of ray origin
  float* org_z;  //!< z coordinate of ray origin
  float* tnear; //!< Start of ray segment (optional)

  float* dir_x;  //!< x coordinate of ray direction
  float* dir_y;  //!< y coordinate of ray direction
  float* dir_z;  //!< z coordinate of ray direction  
  float* tfar;  //!< End of ray segment (set to hit distance)
 
  float* time;     //!< Time of this ray for motion blur (optional)
  unsigned* mask;  //!< Used to mask out objects during traversal (optional)

  unsigned* id;    //!< ray ID
  unsigned* flags; //!< per flags

  /* hit data */
  float* Ng_x;   //!< x coordinate of geometry normal (optional)
  float* Ng_y;   //!< y coordinate of geometry normal (optional)
  float* Ng_z;   //!< z coordinate of geometry normal (optional)

  float* u;     //!< Barycentric u coordinate of hit
  float* v;     //!< Barycentric v coordinate of hit

  unsigned* primID;  //!< primitive ID 
  unsigned* geomID;  //!< geometry ID
  unsigned* instID;  //!< instance ID (optional)
};

struct RTCRayN;
struct RTCHitN;

#if defined(__cplusplus)
}
#endif

#if defined(__cplusplus)

/* Helper functions to access ray packets of runtime size N */
RTCORE_FORCEINLINE float& RTCRayN_org_x(RTCRayN* ptr, unsigned int N, unsigned int i) { return ((float*)ptr)[0*N+i]; }  //!< x coordinate of ray origin
RTCORE_FORCEINLINE float& RTCRayN_org_y(RTCRayN* ptr, unsigned int N, unsigned int i) { return ((float*)ptr)[1*N+i]; }  //!< y coordinate of ray origin
RTCORE_FORCEINLINE float& RTCRayN_org_z(RTCRayN* ptr, unsigned int N, unsigned int i) { return ((float*)ptr)[2*N+i]; }  //!< z coordinate of ray origin
RTCORE_FORCEINLINE float& RTCRayN_tnear(RTCRayN* ptr, unsigned int N, unsigned int i) { return ((float*)ptr)[3*N+i]; }  //!< Start of ray segment 

RTCORE_FORCEINLINE float& RTCRayN_dir_x(RTCRayN* ptr, unsigned int N, unsigned int i) { return ((float*)ptr)[4*N+i]; }  //!< x coordinate of ray direction
RTCORE_FORCEINLINE float& RTCRayN_dir_y(RTCRayN* ptr, unsigned int N, unsigned int i) { return ((float*)ptr)[5*N+i]; }  //!< y coordinate of ray direction
RTCORE_FORCEINLINE float& RTCRayN_dir_z(RTCRayN* ptr, unsigned int N, unsigned int i) { return ((float*)ptr)[6*N+i]; }  //!< z coordinate of ray direction
RTCORE_FORCEINLINE float& RTCRayN_tfar (RTCRayN* ptr, unsigned int N, unsigned int i) { return ((float*)ptr)[7*N+i]; }  //!< End of ray segment (set to hit distance)

RTCORE_FORCEINLINE float&    RTCRayN_time(RTCRayN* ptr, unsigned int N, unsigned int i) { return ((float*)   ptr)[8*N+i]; }   //!< Time of this ray for motion blur 
RTCORE_FORCEINLINE unsigned& RTCRayN_mask(RTCRayN* ptr, unsigned int N, unsigned int i) { return ((unsigned*)ptr)[9*N+i]; }   //!< Used to mask out objects during traversal

RTCORE_FORCEINLINE unsigned& RTCRayN_id   (RTCRayN* ptr, unsigned int N, unsigned int i) { return ((unsigned*)ptr)[10*N+i]; }   //!< Time of this ray for motion blur 
RTCORE_FORCEINLINE unsigned& RTCRayN_flags(RTCRayN* ptr, unsigned int N, unsigned int i) { return ((unsigned*)ptr)[11*N+i]; }   //!< Used to mask out objects during traversal

RTCORE_FORCEINLINE float& RTCRayN_Ng_x(RTCRayN* ptr, unsigned int N, unsigned int i) { return ((float*)ptr)[12*N+i]; }  //!< x coordinate of geometry normal
RTCORE_FORCEINLINE float& RTCRayN_Ng_y(RTCRayN* ptr, unsigned int N, unsigned int i) { return ((float*)ptr)[13*N+i]; }  //!< y coordinate of geometry normal
RTCORE_FORCEINLINE float& RTCRayN_Ng_z(RTCRayN* ptr, unsigned int N, unsigned int i) { return ((float*)ptr)[14*N+i]; }  //!< z coordinate of geometry normal

RTCORE_FORCEINLINE float& RTCRayN_u   (RTCRayN* ptr, unsigned int N, unsigned int i) { return ((float*)ptr)[15*N+i]; }  //!< Barycentric u coordinate of hit
RTCORE_FORCEINLINE float& RTCRayN_v   (RTCRayN* ptr, unsigned int N, unsigned int i) { return ((float*)ptr)[16*N+i]; }  //!< Barycentric v coordinate of hit

RTCORE_FORCEINLINE unsigned& RTCRayN_primID(RTCRayN* ptr, unsigned int N, unsigned int i) { return ((unsigned*)ptr)[17*N+i]; }  //!< primitive ID
RTCORE_FORCEINLINE unsigned& RTCRayN_geomID(RTCRayN* ptr, unsigned int N, unsigned int i) { return ((unsigned*)ptr)[18*N+i]; }  //!< geometry ID
RTCORE_FORCEINLINE unsigned& RTCRayN_instID(RTCRayN* ptr, unsigned int N, unsigned int i) { return ((unsigned*)ptr)[19*N+i]; }  //!< instance ID

/* Helper functions to access hit packets of size N */
RTCORE_FORCEINLINE float& RTCHitN_Ng_x(const RTCHitN* ptr, unsigned int N, unsigned int i) { return ((float*)ptr)[0*N+i]; }  //!< x coordinate of geometry normal
RTCORE_FORCEINLINE float& RTCHitN_Ng_y(const RTCHitN* ptr, unsigned int N, unsigned int i) { return ((float*)ptr)[1*N+i]; }  //!< y coordinate of geometry normal
RTCORE_FORCEINLINE float& RTCHitN_Ng_z(const RTCHitN* ptr, unsigned int N, unsigned int i) { return ((float*)ptr)[2*N+i]; }  //!< z coordinate of geometry normal

RTCORE_FORCEINLINE float& RTCHitN_t   (const RTCHitN* ptr, unsigned int N, unsigned int i) { return ((float*)ptr)[3*N+i]; } //!< hit distance
RTCORE_FORCEINLINE float& RTCHitN_u   (const RTCHitN* ptr, unsigned int N, unsigned int i) { return ((float*)ptr)[4*N+i]; } //!< Barycentric u coordinate of hit
RTCORE_FORCEINLINE float& RTCHitN_v   (const RTCHitN* ptr, unsigned int N, unsigned int i) { return ((float*)ptr)[5*N+i]; } //!< Barycentric v coordinate of hit

RTCORE_FORCEINLINE unsigned& RTCHitN_primID(const RTCHitN* ptr, unsigned int N, unsigned int i) { return ((unsigned*)ptr)[6*N+i]; }  //!< primitive ID
RTCORE_FORCEINLINE unsigned& RTCHitN_geomID(const RTCHitN* ptr, unsigned int N, unsigned int i) { return ((unsigned*)ptr)[7*N+i]; }  //!< geometry ID
RTCORE_FORCEINLINE unsigned& RTCHitN_instID(const RTCHitN* ptr, unsigned int N, unsigned int i) { return ((unsigned*)ptr)[8*N+i]; }  //!< instance ID


/* Helper structure to create a ray packet of compile time size N */
template<int N>
struct RTCRayNt
{
  /* ray data */
public:
  float org_x[N];  //!< x coordinate of ray origin
  float org_y[N];  //!< y coordinate of ray origin
  float org_z[N];  //!< z coordinate of ray origin
  float tnear[N]; //!< Start of ray segment 
  
  float dir_x[N];  //!< x coordinate of ray direction
  float dir_y[N];  //!< y coordinate of ray direction
  float dir_z[N];  //!< z coordinate of ray direction  
  float tfar[N];  //!< End of ray segment (set to hit distance)

  float time[N];  //!< Time of this ray for motion blur
  unsigned mask[N];  //!< Used to mask out objects during traversal
  
  unsigned id[N];    //!< ray ID
  unsigned flags[N]; //!< per flags

  /* hit data */
public:
  float Ng_x[N];   //!< x coordinate of geometry normal
  float Ng_y[N];   //!< y coordinate of geometry normal
  float Ng_z[N];   //!< z coordinate of geometry normal
  
  float u[N];     //!< Barycentric u coordinate of hit
  float v[N];     //!< Barycentric v coordinate of hit

  unsigned primID[N];  //!< primitive ID  
  unsigned geomID[N];  //!< geometry ID
  unsigned instID[N];  //!< instance ID
};

/* Helper structure to create a hit packet of compile time size N */
template<int N>
struct RTCHitNt
{
  float Ng_x[N];        //!< x coordinate of geometry normal
  float Ng_y[N];        //!< y coordinate of geometry normal
  float Ng_z[N];        //!< z coordinate of geometry normal

  float t[N];          //!< hit distance
  float u[N];          //!< Barycentric u coordinate of hit
  float v[N];          //!< Barycentric v coordinate of hit

  unsigned primID[N];  //!< primitive ID
  unsigned geomID[N];  //!< geometry ID
  unsigned instID[N];  //!< instance ID
};

struct RTCHit
{
  float Ng_x;        //!< x coordinate of geometry normal
  float Ng_y;        //!< y coordinate of geometry normal
  float Ng_z;        //!< z coordinate of geometry normal

  float t;          //!< hit distance
  float u;          //!< Barycentric u coordinate of hit
  float v;          //!< Barycentric v coordinate of hit

  unsigned primID;  //!< primitive ID
  unsigned geomID;  //!< geometry ID
  unsigned instID;  //!< instance ID
};


RTCORE_FORCEINLINE RTCRay RTCRayNtoRTCRay(RTCRayN* ptr, unsigned int N, unsigned int i)
{
  RTCRay ray;
  ray.org_x   = RTCRayN_org_x(ptr,N,i);
  ray.org_y   = RTCRayN_org_y(ptr,N,i);
  ray.org_z   = RTCRayN_org_z(ptr,N,i);
  ray.tnear   = RTCRayN_tnear(ptr,N,i);
  ray.dir_x   = RTCRayN_dir_x(ptr,N,i);
  ray.dir_y   = RTCRayN_dir_y(ptr,N,i);
  ray.dir_z   = RTCRayN_dir_z(ptr,N,i);
  ray.tfar    = RTCRayN_tfar(ptr,N,i);
  ray.time    = RTCRayN_time(ptr,N,i);
  ray.mask    = RTCRayN_mask(ptr,N,i);
  ray.id      = RTCRayN_id(ptr,N,i);
  ray.flags   = RTCRayN_flags(ptr,N,i);
  ray.Ng_x    = RTCRayN_Ng_x(ptr,N,i);
  ray.Ng_y    = RTCRayN_Ng_y(ptr,N,i);
  ray.Ng_z    = RTCRayN_Ng_z(ptr,N,i);
  ray.u       = RTCRayN_u(ptr,N,i);
  ray.v       = RTCRayN_v(ptr,N,i);
  ray.primID  = RTCRayN_primID(ptr,N,i);
  ray.geomID  = RTCRayN_geomID(ptr,N,i);
  ray.instID  = RTCRayN_instID(ptr,N,i);
  return ray;
};

RTCORE_FORCEINLINE RTCHit RTCHitNtoRTCHit(const RTCHitN* ptr, unsigned int N, unsigned int i)
{
  RTCHit hit;
  hit.Ng_x   = RTCHitN_Ng_x(ptr,N,i);
  hit.Ng_y   = RTCHitN_Ng_y(ptr,N,i);
  hit.Ng_z   = RTCHitN_Ng_z(ptr,N,i);
  hit.t      = RTCHitN_t(ptr,N,i);
  hit.u      = RTCHitN_u(ptr,N,i);
  hit.v      = RTCHitN_v(ptr,N,i);
  hit.primID = RTCHitN_primID(ptr,N,i);
  hit.geomID = RTCHitN_geomID(ptr,N,i);
  hit.instID = RTCHitN_instID(ptr,N,i);
  return hit;
};


RTCORE_FORCEINLINE void copyRTCHitToRTCRay(RTCRay *ray, RTCHit *hit)
{
  ray->Ng_x = hit->Ng_x;
  ray->Ng_y = hit->Ng_y;
  ray->Ng_z = hit->Ng_z;  
  ray->tfar = hit->t;
  ray->u = hit->u;
  ray->v = hit->v;
  ray->primID = hit->primID;
  ray->geomID = hit->geomID;
  ray->instID = hit->instID;
}

RTCORE_FORCEINLINE void copyRTCHitToRTCRayN(RTCRayN *rays,  RTCHit *hit, unsigned int N, unsigned int ui)
{
  RTCRayN_Ng_x(rays,N,ui)   = hit->Ng_x;
  RTCRayN_Ng_y(rays,N,ui)   = hit->Ng_y;
  RTCRayN_Ng_z(rays,N,ui)   = hit->Ng_z;
  RTCRayN_tfar(rays,N,ui)   = hit->t;
  RTCRayN_u(rays,N,ui)      = hit->u;
  RTCRayN_v(rays,N,ui)      = hit->v;
  RTCRayN_primID(rays,N,ui) = hit->primID;
  RTCRayN_geomID(rays,N,ui) = hit->geomID;
  RTCRayN_instID(rays,N,ui) = hit->instID;
}

RTCORE_FORCEINLINE void copyHitFromRTCRayToRTCRayN(RTCRayN *rays, RTCRay *ray, unsigned int N, unsigned int ui)
{
  RTCRayN_Ng_x(rays,N,ui)   = ray->Ng_x;
  RTCRayN_Ng_y(rays,N,ui)   = ray->Ng_y;
  RTCRayN_Ng_z(rays,N,ui)   = ray->Ng_z;
  RTCRayN_tfar(rays,N,ui)   = ray->tfar;
  RTCRayN_u(rays,N,ui)      = ray->u;
  RTCRayN_v(rays,N,ui)      = ray->v;
  RTCRayN_primID(rays,N,ui) = ray->primID;
  RTCRayN_geomID(rays,N,ui) = ray->geomID;
  RTCRayN_instID(rays,N,ui) = ray->instID;
}

#endif

/*! @} */
