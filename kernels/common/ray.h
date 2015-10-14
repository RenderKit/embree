// ======================================================================== //
// Copyright 2009-2015 Intel Corporation                                    //
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

#include "default.h"

namespace embree
{
  /* Ray structure for K rays */
  template<int K>
  struct RayK
  {
    /* Default construction does nothing */
    __forceinline RayK() {}

    /* Constructs a ray from origin, direction, and ray segment. Near
     * has to be smaller than far */
    __forceinline RayK(const Vec3<vfloat<K>>& org, const Vec3<vfloat<K>>& dir,
                       const vfloat<K>& tnear = zero, const vfloat<K>& tfar = inf,
                       const vfloat<K>& time = zero, const vint<K>& mask = -1)
      : org(org), dir(dir), tnear(tnear), tfar(tfar), geomID(-1), primID(-1), instID(-1), mask(mask), time(time) {}

    /* Returns the size of the ray */
    static __forceinline size_t size() { return K; }

    /* Tests if we hit something */
    __forceinline operator vbool<K>() const { return geomID != vint<K>(-1); }

    /* Calculates if this is a valid ray that does not cause issues during traversal */
    __forceinline vbool<K> valid() const
    {
      const vbool<K> vx = abs(org.x) <= vfloat<K>(FLT_LARGE) & abs(dir.x) <= vfloat<K>(FLT_LARGE);
      const vbool<K> vy = abs(org.y) <= vfloat<K>(FLT_LARGE) & abs(dir.y) <= vfloat<K>(FLT_LARGE);
      const vbool<K> vz = abs(org.z) <= vfloat<K>(FLT_LARGE) & abs(dir.z) <= vfloat<K>(FLT_LARGE);
      const vbool<K> vn = abs(tnear) <= vfloat<K>(inf);
      const vbool<K> vf = abs(tfar) <= vfloat<K>(inf);
      return vx & vy & vz & vn & vf;
    }

    __forceinline void get(RayK<1>* ray) const;
    __forceinline void set(const RayK<1>* ray);

#if defined(__MIC__)
    template<int PFHINT>
    __forceinline void prefetchHitData() const
    {
      prefetch<PFHINT>(&geomID);
      prefetch<PFHINT>(&primID);
      prefetch<PFHINT>(&tfar);
      prefetch<PFHINT>(&u);
      prefetch<PFHINT>(&v);
      prefetch<PFHINT>(&Ng.x);
      prefetch<PFHINT>(&Ng.y);
      prefetch<PFHINT>(&Ng.z);
    }
#endif

    __forceinline void update(const vbool<K>& m_mask,
                              const vfloat<K>& new_t,
                              const vfloat<K>& new_u,
                              const vfloat<K>& new_v,
                              const vfloat<K>& new_gnormalx,
                              const vfloat<K>& new_gnormaly,
                              const vfloat<K>& new_gnormalz,
                              const vint<K>& new_geomID,
                              const vint<K>& new_primID)
    {
      vfloat<K>::store(m_mask,(float*)&tfar,new_t);
      vfloat<K>::store(m_mask,(float*)&u,new_u);
      vfloat<K>::store(m_mask,(float*)&v,new_v);
      vfloat<K>::store(m_mask,(float*)&Ng.x,new_gnormalx);
      vfloat<K>::store(m_mask,(float*)&Ng.y,new_gnormaly);
      vfloat<K>::store(m_mask,(float*)&Ng.z,new_gnormalz);
      vint<K>::store(m_mask,(int*)&geomID,new_geomID);
      vint<K>::store(m_mask,(int*)&primID,new_primID);
    }

    __forceinline void update(const vbool<K>& m_mask,
                              const size_t rayIndex,
                              const vfloat<K>& new_t,
                              const vfloat<K>& new_u,
                              const vfloat<K>& new_v,
                              const vfloat<K>& new_gnormalx,
                              const vfloat<K>& new_gnormaly,
                              const vfloat<K>& new_gnormalz,
                              const int new_geomID,
                              const int new_primID);

    /* Ray data */
    Vec3<vfloat<K>> org; // ray origin
    Vec3<vfloat<K>> dir; // ray direction
    vfloat<K> tnear;     // start of ray segment
    vfloat<K> tfar;      // end of ray segment
    vfloat<K> time;      // time of this ray for motion blur.
    vint<K> mask;        // used to mask out objects during traversal

    /* Hit data */
    Vec3<vfloat<K>> Ng;  // geometry normal
    vfloat<K> u;         // barycentric u coordinate of hit
    vfloat<K> v;         // barycentric v coordinate of hit
    vint<K> geomID;      // geometry ID
    vint<K> primID;      // primitive ID
    vint<K> instID;      // instance ID
  };

#if defined(__AVX512F__) || defined(__MIC__)
  template<>
  __forceinline void RayK<16>::update(const vbool16& m_mask,
                                      const size_t rayIndex,
                                      const vfloat16& new_t,
                                      const vfloat16& new_u,
                                      const vfloat16& new_v,
                                      const vfloat16& new_gnormalx,
                                      const vfloat16& new_gnormaly,
                                      const vfloat16& new_gnormalz,
                                      const int new_geomID,
                                      const int new_primID)
  {
    geomID[rayIndex] = new_geomID;
    primID[rayIndex] = new_primID;

    compactustore16f_low(m_mask,&tfar[rayIndex],new_t);
    compactustore16f_low(m_mask,&u[rayIndex],new_u);
    compactustore16f_low(m_mask,&v[rayIndex],new_v);
    compactustore16f_low(m_mask,&Ng.x[rayIndex],new_gnormalx);
    compactustore16f_low(m_mask,&Ng.y[rayIndex],new_gnormaly);
    compactustore16f_low(m_mask,&Ng.z[rayIndex],new_gnormalz);
  }
#endif

  /* Specialization for a single ray */
  template<>
  struct RayK<1>
  {
    /* Default construction does nothing */
    __forceinline RayK() {}

    /* Constructs a ray from origin, direction, and ray segment. Near
     *  has to be smaller than far */
    __forceinline RayK(const Vec3fa& org, const Vec3fa& dir, float tnear = zero, float tfar = inf, float time = zero, int mask = -1)
      : org(org), dir(dir), tnear(tnear), tfar(tfar), geomID(-1), primID(-1), instID(-1), mask(mask), time(time) {}

    /* Tests if we hit something */
    __forceinline operator bool() const { return geomID != -1; }

    /* Calculates if this is a valid ray that does not cause issues during traversal */
    __forceinline bool valid() const {
      return all(le_mask(abs(org),Vec3fa(FLT_LARGE)) & le_mask(abs(dir),Vec3fa(FLT_LARGE))) && fabs(tnear) <= float(inf) && fabs(tfar) <= float(inf);
    }

    /* Ray data */
    Vec3fa org;  // ray origin
    Vec3fa dir;  // ray direction
    float tnear; // start of ray segment
    float tfar;  // end of ray segment
    float time;  // time of this ray for motion blur.
    int mask;    // used to mask out objects during traversal

    /* Hit data */
    Vec3fa Ng;   // not normalized geometry normal
    float u;     // barycentric u coordinate of hit
    float v;     // barycentric v coordinate of hit
    int geomID;  // geometry ID
    int primID;  // primitive ID
    int instID;  // instance ID

#if defined(__AVX512F__) || defined(__MIC__)
    __forceinline void update(const vbool16& m_mask,
                              const vfloat16& new_t,
                              const vfloat16& new_u,
                              const vfloat16& new_v,
                              const vfloat16& new_gnormalx,
                              const vfloat16& new_gnormaly,
                              const vfloat16& new_gnormalz,
			      const int new_geomID,
			      const int new_primID)
    {
      geomID = new_geomID;
      primID = new_primID;

      compactustore16f_low(m_mask,&tfar,new_t);
      compactustore16f_low(m_mask,&u,new_u); 
      compactustore16f_low(m_mask,&v,new_v); 
      compactustore16f_low(m_mask,&Ng.x,new_gnormalx); 
      compactustore16f_low(m_mask,&Ng.y,new_gnormaly); 
      compactustore16f_low(m_mask,&Ng.z,new_gnormalz);       
    }
#endif
  };

  /* Converts ray packet to single rays */
  template<int K>
  __forceinline void RayK<K>::get(RayK<1>* ray) const
  {
    for (size_t i=0; i<K; i++) // FIXME: use SIMD transpose
    {
      ray[i].org.x = org.x[i]; ray[i].org.y = org.y[i]; ray[i].org.z = org.z[i];
      ray[i].dir.x = dir.x[i]; ray[i].dir.y = dir.y[i]; ray[i].dir.z = dir.z[i];
      ray[i].tnear = tnear[i]; ray[i].tfar  = tfar [i]; ray[i].time  = time[i]; ray[i].mask = mask[i];
      ray[i].Ng.x = Ng.x[i]; ray[i].Ng.y = Ng.y[i]; ray[i].Ng.z = Ng.z[i];
      ray[i].u = u[i]; ray[i].v = v[i];
      ray[i].geomID = geomID[i]; ray[i].primID = primID[i]; ray[i].instID = instID[i];
    }
  }

  /* Converts single rays to ray packet */
  template<int K>
  __forceinline void RayK<K>::set(const RayK<1>* ray)
  {
    for (size_t i=0; i<K; i++)
    {
      org.x[i] = ray[i].org.x; org.y[i] = ray[i].org.y; org.z[i] = ray[i].org.z;
      dir.x[i] = ray[i].dir.x; dir.y[i] = ray[i].dir.y; dir.z[i] = ray[i].dir.z;
      tnear[i] = ray[i].tnear; tfar [i] = ray[i].tfar;  time[i] = ray[i].time; mask[i] = ray[i].mask;
      Ng.x[i] = ray[i].Ng.x; Ng.y[i] = ray[i].Ng.y; Ng.z[i] = ray[i].Ng.z;
      u[i] = ray[i].u; v[i] = ray[i].v;
      geomID[i] = ray[i].geomID; primID[i] = ray[i].primID; instID[i] = ray[i].instID;
    }
  }

  /* Shortcuts */
  typedef RayK<1>  Ray;
  typedef RayK<4>  Ray4;
  typedef RayK<8>  Ray8;
  typedef RayK<16> Ray16;

  /* Outputs ray to stream */
  template<int K>
  inline std::ostream& operator<<(std::ostream& cout, const RayK<K>& ray)
  {
    return cout << "{ " << std::endl
                << "org = " << ray.org << std::endl
                << " dir = " << ray.dir << std::endl
                << " near = " << ray.tnear << std::endl
                << " far = " << ray.tfar << std::endl
                << " time = " << ray.time << std::endl
                << " instID = " << ray.instID << std::endl
                << " geomID = " << ray.geomID << std::endl
                << " primID = " << ray.primID <<  std::endl
                << " u = " << ray.u <<  std::endl
                << " v = " << ray.v << std::endl
                << " Ng = " << ray.Ng
                << " }";
  }
}
