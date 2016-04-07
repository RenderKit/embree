// ======================================================================== //
// Copyright 2009-2016 Intel Corporation                                    //
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

    template<int M>
    __forceinline void updateK(const size_t i,
                               const size_t rayIndex,
                               const vfloat<M>& new_t,
                               const vfloat<M>& new_u,
                               const vfloat<M>& new_v,
                               const vfloat<M>& new_gnormalx,
                               const vfloat<M>& new_gnormaly,
                               const vfloat<M>& new_gnormalz,
                               const int new_geomID,
                               const vint<M> &new_primID)
    {
      u[rayIndex] = new_u[i];
      v[rayIndex] = new_v[i];
      tfar[rayIndex] = new_t[i];
      Ng.x[rayIndex] = new_gnormalx[i];
      Ng.y[rayIndex] = new_gnormaly[i];
      Ng.z[rayIndex] = new_gnormalz[i];
      geomID[rayIndex] = new_geomID;
      primID[rayIndex] = new_primID[i];
    }


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
      __forceinline void RayK<16>::updateK<16>(const size_t i,
                                             const size_t rayIndex,
                                             const vfloat16& new_t,
                                             const vfloat16& new_u,
                                             const vfloat16& new_v,
                                             const vfloat16& new_gnormalx,
                                             const vfloat16& new_gnormaly,
                                             const vfloat16& new_gnormalz,
                                             const int new_geomID,
                                             const vint16 &new_primID)
    {
      const vbool16 m_mask((unsigned int)1 << i);
      vfloat16::storeu_compact_single(m_mask,&tfar[rayIndex],new_t);
      vfloat16::storeu_compact_single(m_mask,&Ng.x[rayIndex],new_gnormalx);
      vfloat16::storeu_compact_single(m_mask,&Ng.y[rayIndex],new_gnormaly);
      vfloat16::storeu_compact_single(m_mask,&Ng.z[rayIndex],new_gnormalz);
      vfloat16::storeu_compact_single(m_mask,&u[rayIndex],new_u);
      vfloat16::storeu_compact_single(m_mask,&v[rayIndex],new_v);
      vint16::storeu_compact_single(m_mask,&primID[rayIndex],new_primID);
      geomID[rayIndex] = new_geomID;
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

    /* filter out all occluded rays from a stream of rays */
    __forceinline static void filterOutOccluded(RayK<1>** ray, size_t& N)
    {
      size_t l=0, r=N;
      while (l<r) {
        if (ray[l]->geomID != 0) l++;
        else std::swap(ray[l],ray[--r]); 
      }
      N = r;
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

      vfloat16::storeu_compact_single(m_mask,&tfar,new_t);
      vfloat16::storeu_compact_single(m_mask,&u,new_u); 
      vfloat16::storeu_compact_single(m_mask,&v,new_v); 
      vfloat16::storeu_compact_single(m_mask,&Ng.x,new_gnormalx); 
      vfloat16::storeu_compact_single(m_mask,&Ng.y,new_gnormaly); 
      vfloat16::storeu_compact_single(m_mask,&Ng.z,new_gnormalz);       
    }

    __forceinline void update(const vbool16& m_mask,
                              const vfloat16& new_t,
                              const vfloat16& new_u,
                              const vfloat16& new_v,
                              const vfloat16& new_gnormalx,
                              const vfloat16& new_gnormaly,
                              const vfloat16& new_gnormalz,
			      const vint16 &new_geomID,
			      const vint16 &new_primID)
    {
      vint16::storeu_compact_single(m_mask,&geomID,new_geomID);
      vint16::storeu_compact_single(m_mask,&primID,new_primID);
      vfloat16::storeu_compact_single(m_mask,&tfar,new_t);
      vfloat16::storeu_compact_single(m_mask,&u,new_u); 
      vfloat16::storeu_compact_single(m_mask,&v,new_v); 
      vfloat16::storeu_compact_single(m_mask,&Ng.x,new_gnormalx); 
      vfloat16::storeu_compact_single(m_mask,&Ng.y,new_gnormaly); 
      vfloat16::storeu_compact_single(m_mask,&Ng.z,new_gnormalz);       
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


  struct RaySOA
  {
    /* ray data */
  public:

    float* __restrict__ orgx;  //!< x coordinate of ray origin
    float* __restrict__ orgy;  //!< y coordinate of ray origin
    float* __restrict__ orgz;  //!< z coordinate of ray origin

    float* __restrict__ dirx;  //!< x coordinate of ray direction
    float* __restrict__ diry;  //!< y coordinate of ray direction
    float* __restrict__ dirz;  //!< z coordinate of ray direction

    float* __restrict__ tnear; //!< Start of ray segment (optional)
    float* __restrict__ tfar;  //!< End of ray segment (set to hit distance)

 
    float* __restrict__ time;     //!< Time of this ray for motion blur (optional)
    unsigned* __restrict__ mask;  //!< Used to mask out objects during traversal (optional)

    /* hit data */

    float* __restrict__ Ngx;   //!< x coordinate of geometry normal (optional)
    float* __restrict__ Ngy;   //!<y coordinate of geometry normal (optional)
    float* __restrict__ Ngz;   //!< z coordinate of geometry normal (optional)

 

    float* __restrict__ u;     //!< Barycentric u coordinate of hit
    float* __restrict__ v;     //!< Barycentric v coordinate of hit

 
    unsigned* __restrict__ geomID;  //!< geometry ID
    unsigned* __restrict__ primID;  //!< primitive ID
    unsigned* __restrict__ instID;  //!< instance ID (optional)

    template<class T>
    __forceinline void init(T &t)
    {
      orgx   = (float*)&t.org.x;
      orgy   = (float*)&t.org.y;
      orgz   = (float*)&t.org.z;
      dirx   = (float*)&t.dir.x;
      diry   = (float*)&t.dir.y;
      dirz   = (float*)&t.dir.z;
      tnear  = (float*)&t.tnear;
      tfar   = (float*)&t.tfar;
      time   = (float*)&t.time;
      mask   = (unsigned *)&t.mask;
      Ngx    = (float*)&t.Ng.x;
      Ngy    = (float*)&t.Ng.y;
      Ngz    = (float*)&t.Ng.z;
      u      = (float*)&t.u;
      v      = (float*)&t.v;
      geomID = (unsigned *)&t.geomID;
      primID = (unsigned *)&t.primID;
      instID = (unsigned *)&t.instID;
    }

    __forceinline Ray gatherByOffset(const size_t offset)
    {
      Ray ray;
      ray.org.x = *(float* __restrict__ )((char*)orgx + offset);
      ray.org.y = *(float* __restrict__ )((char*)orgy + offset);
      ray.org.z = *(float* __restrict__ )((char*)orgz + offset);
      ray.dir.x = *(float* __restrict__ )((char*)dirx + offset);
      ray.dir.y = *(float* __restrict__ )((char*)diry + offset);
      ray.dir.z = *(float* __restrict__ )((char*)dirz + offset);
      ray.tfar  = *(float* __restrict__ )((char*)tfar + offset);
      /* optional inputs */
      ray.tnear = tnear ? *(float* __restrict__ )((char*)tnear + offset) : 0.0f;
      ray.time  = time  ? *(float* __restrict__ )((char*)time  + offset) : 0.0f;
      ray.mask  = mask  ? *(unsigned * __restrict__ )((char*)mask  + offset) : -1;
      /* init geomID */
      ray.geomID = RTC_INVALID_GEOMETRY_ID;
      return ray;
    }

    template<int K>
    __forceinline RayK<K> gather(const size_t offset)
    {
      RayK<K> ray;
      ray.org.x = vfloat<K>::load((float* __restrict__ )((char*)orgx + offset));
      ray.org.y = vfloat<K>::load((float* __restrict__ )((char*)orgy + offset));
      ray.org.z = vfloat<K>::load((float* __restrict__ )((char*)orgz + offset));
      ray.dir.x = vfloat<K>::load((float* __restrict__ )((char*)dirx + offset));
      ray.dir.y = vfloat<K>::load((float* __restrict__ )((char*)diry + offset));
      ray.dir.z = vfloat<K>::load((float* __restrict__ )((char*)dirz + offset));
      ray.tfar  = vfloat<K>::load((float* __restrict__ )((char*)tfar + offset));
      /* optional inputs */
      ray.tnear = tnear ? vfloat<K>::load((float* __restrict__ )((char*)tnear + offset)) : 0.0f;
      ray.time  = time  ? vfloat<K>::load((float* __restrict__ )((char*)time  + offset)) : 0.0f;
      ray.mask  = mask  ? vint<K>::load((int * __restrict__ )((char*)mask  + offset)) : -1;
      /* init geomID */
      ray.geomID = RTC_INVALID_GEOMETRY_ID;
      return ray;
    }

    __forceinline void scatterByOffset(const size_t offset, const Ray& ray, const bool all=true)
    {
      *(unsigned * __restrict__ )((char*)geomID + offset) = ray.geomID;
      if (all)
        if (ray.geomID !=  RTC_INVALID_GEOMETRY_ID)
        {
          *(float* __restrict__ )((char*)tfar + offset) = ray.tfar;
          *(float* __restrict__ )((char*)u + offset) = ray.u;
          *(float* __restrict__ )((char*)v + offset) = ray.v;
          *(unsigned * __restrict__ )((char*)primID + offset) = ray.primID;
          if (likely(Ngx)) *(float* __restrict__ )((char*)Ngx + offset) = ray.Ng.x;
          if (likely(Ngy)) *(float* __restrict__ )((char*)Ngy + offset) = ray.Ng.y;
          if (likely(Ngz)) *(float* __restrict__ )((char*)Ngz + offset) = ray.Ng.z;
          if (likely(instID)) *(unsigned * __restrict__ )((char*)instID + offset) = ray.instID;
        }
    }

    template<int K>
    __forceinline void scatter(const size_t offset, const RayK<K>& ray, const bool all=true)
    {
      vint<K>::store((int * __restrict__ )((char*)geomID + offset), ray.geomID);
      if (!all) return;

      vbool<K> mask = ray.geomID !=  RTC_INVALID_GEOMETRY_ID;
      
      vfloat<K>::store((float* __restrict__ )((char*)tfar + offset), ray.tfar);
      vfloat<K>::store((float* __restrict__ )((char*)u + offset), ray.u);
      vfloat<K>::store((float* __restrict__ )((char*)v + offset), ray.v);
      vint<K>::store((int * __restrict__ )((char*)primID + offset), ray.primID);
      if (likely(Ngx)) vfloat<K>::store((float* __restrict__ )((char*)Ngx + offset), ray.Ng.x);
      if (likely(Ngy)) vfloat<K>::store((float* __restrict__ )((char*)Ngy + offset), ray.Ng.y);
      if (likely(Ngz)) vfloat<K>::store((float* __restrict__ )((char*)Ngz + offset), ray.Ng.z);
      if (likely(instID)) vint<K>::store((int * __restrict__ )((char*)instID + offset), ray.instID);
    }

    __forceinline size_t getOctantByOffset(const size_t offset)
    {
      const float dx = *(float* __restrict__ )((char*)dirx + offset);
      const float dy = *(float* __restrict__ )((char*)diry + offset);
      const float dz = *(float* __restrict__ )((char*)dirz + offset);
      const size_t octantID = (dx < 0.0f ? 1 : 0) + (dy < 0.0f ? 2 : 0) + (dz < 0.0f ? 4 : 0);
      return octantID;
    }

    __forceinline bool isValidByOffset(const size_t offset)
    {
      const float nnear = tnear ? *(float* __restrict__ )((char*)tnear + offset) : 0.0f;
      const float ffar  = *(float* __restrict__ )((char*)tfar  + offset);
      return nnear <= ffar;
    }

  };

}
