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

#include "default.h"

// FIXME: if ray gets seperated into ray* and hit, uload4 needs to be adjusted

namespace embree
{
  static const size_t MAX_INTERNAL_STREAM_SIZE = 32;

  /* Ray structure for K rays */
  template<int K>
  struct RayK
  {
    /* Default construction does nothing */
    __forceinline RayK() {}

    /* Constructs a ray from origin, direction, and ray segment. Near
     * has to be smaller than far */
    __forceinline RayK(const Vec3vf<K>& org, const Vec3vf<K>& dir,
                       const vfloat<K>& tnear = zero, const vfloat<K>& tfar = inf,
                       const vfloat<K>& time = zero, const vint<K>& mask = -1, const vint<K>& id = 0, const vint<K>& flags = 0)
      : org(org), dir(dir), _tnear(tnear), _tfar(tfar), time(time), mask(mask), id(id), flags(flags), primID(-1), geomID(-1), instID(-1) {}

    /* Returns the size of the ray */
    static __forceinline size_t size() { return K; }

    /* Tests if we hit something */
    __forceinline operator vbool<K>() const { return geomID != vint<K>(-1); }

    /* Calculates if this is a valid ray that does not cause issues during traversal */
    __forceinline vbool<K> valid() const
    {
      const vbool<K> vx = (abs(org.x) <= vfloat<K>(FLT_LARGE)) & (abs(dir.x) <= vfloat<K>(FLT_LARGE));
      const vbool<K> vy = (abs(org.y) <= vfloat<K>(FLT_LARGE)) & (abs(dir.y) <= vfloat<K>(FLT_LARGE));
      const vbool<K> vz = (abs(org.z) <= vfloat<K>(FLT_LARGE)) & (abs(dir.z) <= vfloat<K>(FLT_LARGE));
      const vbool<K> vn = abs(tnear()) <= vfloat<K>(inf);
      const vbool<K> vf = abs(tfar()) <= vfloat<K>(inf);
      return vx & vy & vz & vn & vf;
    }

    /* Calculates if the hit is valid */
    __forceinline void verifyHit(const vbool<K>& valid0) const
    {
      vbool<K> valid = valid0 & geomID != vint<K>(RTC_INVALID_GEOMETRY_ID);
      const vbool<K> vt = (abs(tfar()) <= vfloat<K>(FLT_LARGE)) | (tfar() == vfloat<K>(neg_inf));
      const vbool<K> vu = (abs(u) <= vfloat<K>(FLT_LARGE));
      const vbool<K> vv = (abs(u) <= vfloat<K>(FLT_LARGE));
      const vbool<K> vnx = abs(Ng.x) <= vfloat<K>(FLT_LARGE);
      const vbool<K> vny = abs(Ng.y) <= vfloat<K>(FLT_LARGE);
      const vbool<K> vnz = abs(Ng.z) <= vfloat<K>(FLT_LARGE);
      if (any(valid & !vt)) throw_RTCError(RTC_ERROR_UNKNOWN,"invalid t");
      if (any(valid & !vu)) throw_RTCError(RTC_ERROR_UNKNOWN,"invalid u");
      if (any(valid & !vv)) throw_RTCError(RTC_ERROR_UNKNOWN,"invalid v");
      if (any(valid & !vnx)) throw_RTCError(RTC_ERROR_UNKNOWN,"invalid Ng.x");
      if (any(valid & !vny)) throw_RTCError(RTC_ERROR_UNKNOWN,"invalid Ng.y");
      if (any(valid & !vnz)) throw_RTCError(RTC_ERROR_UNKNOWN,"invalid Ng.z");
    }

    __forceinline void get(RayK<1>* ray) const;
    __forceinline void get(size_t i, RayK<1>& ray) const;
    __forceinline void set(const RayK<1>* ray);
    __forceinline void set(size_t i, const RayK<1>& ray);

    __forceinline void copy(size_t dest, size_t source);

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
      vfloat<K>::store(m_mask, (float*)&tfar, new_t);
      vfloat<K>::store(m_mask, (float*)&u, new_u);
      vfloat<K>::store(m_mask, (float*)&v, new_v);
      vfloat<K>::store(m_mask, (float*)&Ng.x, new_gnormalx);
      vfloat<K>::store(m_mask, (float*)&Ng.y, new_gnormaly);
      vfloat<K>::store(m_mask, (float*)&Ng.z, new_gnormalz);
      vint<K>::store(m_mask, (int*)&geomID, new_geomID);
      vint<K>::store(m_mask, (int*)&primID, new_primID);
    }

    template<int M>
    __forceinline void updateK(size_t i,
                               size_t rayIndex,
                               const vfloat<M>& new_t,
                               const vfloat<M>& new_u,
                               const vfloat<M>& new_v,
                               const vfloat<M>& new_gnormalx,
                               const vfloat<M>& new_gnormaly,
                               const vfloat<M>& new_gnormalz,
                               int new_geomID,
                               const vint<M> &new_primID)
    {
      u[rayIndex] = new_u[i];
      v[rayIndex] = new_v[i];
      tfar()[rayIndex] = new_t[i];
      Ng.x[rayIndex] = new_gnormalx[i];
      Ng.y[rayIndex] = new_gnormaly[i];
      Ng.z[rayIndex] = new_gnormalz[i];
      geomID[rayIndex] = new_geomID;
      primID[rayIndex] = new_primID[i];
    }

    __forceinline vint<K> octant() const
    {
      return select(dir.x < 0.0f, vint<K>(1), vint<K>(zero)) |
             select(dir.y < 0.0f, vint<K>(2), vint<K>(zero)) |
             select(dir.z < 0.0f, vint<K>(4), vint<K>(zero));
    }

    /* Ray data */
    Vec3vf<K> org;    // ray origin
    vfloat<K> _tnear; // start of ray segment
    Vec3vf<K> dir;    // ray direction
    vfloat<K> _tfar;  // end of ray segment
    vfloat<K> time;   // time of this ray for motion blur.
    vint<K> mask;     // used to mask out objects during traversal
    vint<K> id;      
    vint<K> flags;  

    /* Hit data */
    Vec3vf<K> Ng;   // geometry normal
    vfloat<K> u;    // barycentric u coordinate of hit
    vfloat<K> v;    // barycentric v coordinate of hit
    vint<K> primID; // primitive ID
    vint<K> geomID; // geometry ID
    vint<K> instID; // instance ID

    __forceinline vfloat<K>& tnear() { return _tnear; }
    __forceinline vfloat<K>& tfar()  { return _tfar; }
    __forceinline const vfloat<K>& tnear() const { return _tnear; }
    __forceinline const vfloat<K>& tfar()  const { return _tfar; }

  };

#if defined(__AVX512F__)
  template<> template<>
  __forceinline void RayK<16>::updateK<16>(size_t i,
                                           size_t rayIndex,
                                           const vfloat16& new_t,
                                           const vfloat16& new_u,
                                           const vfloat16& new_v,
                                           const vfloat16& new_gnormalx,
                                           const vfloat16& new_gnormaly,
                                           const vfloat16& new_gnormalz,
                                           int new_geomID,
                                           const vint16& new_primID)
  {
    const vbool16 m_mask((unsigned int)1 << i);
    vfloat16::storeu_compact_single(m_mask, &tfar()[rayIndex], new_t);
    vfloat16::storeu_compact_single(m_mask, &Ng.x[rayIndex], new_gnormalx);
    vfloat16::storeu_compact_single(m_mask, &Ng.y[rayIndex], new_gnormaly);
    vfloat16::storeu_compact_single(m_mask, &Ng.z[rayIndex], new_gnormalz);
    vfloat16::storeu_compact_single(m_mask, &u[rayIndex], new_u);
    vfloat16::storeu_compact_single(m_mask, &v[rayIndex], new_v);
    vint16::storeu_compact_single(m_mask, &primID[rayIndex], new_primID);
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
    __forceinline RayK(const Vec3fa& org, const Vec3fa& dir, float tnear = zero, float tfar = inf, float time = zero, int mask = -1, int id = 0, int flags = 0, unsigned int geomID = RTC_INVALID_GEOMETRY_ID, unsigned int primID = RTC_INVALID_GEOMETRY_ID, unsigned int instID = RTC_INVALID_GEOMETRY_ID)
      : org(org,tnear), dir(dir,tfar), time(time), mask(mask), id(id), flags(flags), primID(primID), geomID(geomID), instID(instID) {}

    /* Tests if we hit something */
    __forceinline operator bool() const { return geomID != RTC_INVALID_GEOMETRY_ID; }

    /* Calculates if this is a valid ray that does not cause issues during traversal */
    __forceinline bool valid() const {
      return all(le_mask(abs(Vec3fa(org,0.0f)), Vec3fa(FLT_LARGE)) & le_mask(abs(Vec3fa(dir,0.0f)), Vec3fa(FLT_LARGE))) && abs(tnear()) <= float(inf) && abs(tfar()) <= float(inf);
    }

    /* Calculates if the hit is valid */
    __forceinline void verifyHit() const
    {
      if (geomID == RTC_INVALID_GEOMETRY_ID) return;
      const bool vt = (abs(tfar()) <= FLT_LARGE) || (tfar() == float(neg_inf));
      const bool vu = (abs(u) <= FLT_LARGE);
      const bool vv = (abs(u) <= FLT_LARGE);
      const bool vnx = abs(Ng.x) <= FLT_LARGE;
      const bool vny = abs(Ng.y) <= FLT_LARGE;
      const bool vnz = abs(Ng.z) <= FLT_LARGE;
      if (!vt) throw_RTCError(RTC_ERROR_UNKNOWN, "invalid t");
      if (!vu) throw_RTCError(RTC_ERROR_UNKNOWN, "invalid u");
      if (!vv) throw_RTCError(RTC_ERROR_UNKNOWN, "invalid v");
      if (!vnx) throw_RTCError(RTC_ERROR_UNKNOWN, "invalid Ng.x");
      if (!vny) throw_RTCError(RTC_ERROR_UNKNOWN, "invalid Ng.y");
      if (!vnz) throw_RTCError(RTC_ERROR_UNKNOWN, "invalid Ng.z");
    }

    /* Ray data */
    Vec3fa org;  // 3 floats for ray origin, 1 float for tnear
    //float tnear; // start of ray segment
    Vec3fa dir;  // 3 floats for ray direction, 1 float for tfar
    //float tfar;  // end of ray segment
    float time;  // time of this ray for motion blur.
    int mask;    // used to mask out objects during traversal

    int id;      // ray ID
    int flags;   // ray flags

    /* Hit data */
    Vec3f Ng;            // not normalized geometry normal
    float u;             // barycentric u coordinate of hit
    float v;             // barycentric v coordinate of hit
    unsigned int primID; // primitive ID
    unsigned int geomID; // geometry ID
    unsigned int instID; // instance ID

    __forceinline float& tnear() { return org.w; };
    __forceinline float& tfar()  { return dir.w; };
    __forceinline const float& tnear() const { return org.w; };
    __forceinline const float& tfar()  const { return dir.w; };


#if defined(__AVX512F__)
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

      vfloat16::storeu_compact_single(m_mask, &tfar(), new_t);
      vfloat16::storeu_compact_single(m_mask, &u, new_u);
      vfloat16::storeu_compact_single(m_mask, &v, new_v);
      vfloat16::storeu_compact_single(m_mask, &Ng.x, new_gnormalx);
      vfloat16::storeu_compact_single(m_mask, &Ng.y, new_gnormaly);
      vfloat16::storeu_compact_single(m_mask, &Ng.z, new_gnormalz);
    }

    __forceinline void update(const vbool16& m_mask,
                              const vfloat16& new_t,
                              const vfloat16& new_u,
                              const vfloat16& new_v,
                              const vfloat16& new_gnormalx,
                              const vfloat16& new_gnormaly,
                              const vfloat16& new_gnormalz,
                              const vint16& new_geomID,
                              const vint16& new_primID)
    {
      vfloat16::storeu_compact_single(m_mask, &tfar(), new_t);
      vfloat16::storeu_compact_single(m_mask, &u, new_u);
      vfloat16::storeu_compact_single(m_mask, &v, new_v);
      vfloat16::storeu_compact_single(m_mask, &Ng.x, new_gnormalx);
      vfloat16::storeu_compact_single(m_mask, &Ng.y, new_gnormaly);
      vfloat16::storeu_compact_single(m_mask, &Ng.z, new_gnormalz);
      vint16::storeu_compact_single(m_mask, &primID, new_primID);
      vint16::storeu_compact_single(m_mask, &geomID, new_geomID);
    }

#endif
  };

  /* Converts ray packet to single rays */
  template<int K>
  __forceinline void RayK<K>::get(RayK<1>* ray) const
  {
    for (size_t i = 0; i < K; i++) // FIXME: use SIMD transpose
    {
      ray[i].org.x = org.x[i]; ray[i].org.y = org.y[i]; ray[i].org.z = org.z[i]; ray[i].tnear() = tnear()[i];
      ray[i].dir.x = dir.x[i]; ray[i].dir.y = dir.y[i]; ray[i].dir.z = dir.z[i]; ray[i].tfar()  = tfar()[i]; 
      ray[i].time  = time[i]; ray[i].mask = mask[i]; ray[i].id = id[i]; ray[i].flags = flags[i];
      ray[i].Ng.x = Ng.x[i]; ray[i].Ng.y = Ng.y[i]; ray[i].Ng.z = Ng.z[i];
      ray[i].u = u[i]; ray[i].v = v[i];
      ray[i].primID = primID[i]; ray[i].geomID = geomID[i]; ray[i].instID = instID[i];
    }
  }

  /* Extracts a single ray out of a ray packet*/
  template<int K>
  __forceinline void RayK<K>::get(size_t i, RayK<1>& ray) const
  {
    ray.org.x = org.x[i]; ray.org.y = org.y[i]; ray.org.z = org.z[i]; ray.tnear() = tnear()[i]; 
    ray.dir.x = dir.x[i]; ray.dir.y = dir.y[i]; ray.dir.z = dir.z[i]; ray.tfar()  = tfar()[i]; 
    ray.time  = time[i];  ray.mask = mask[i];  ray.id = id[i]; ray.flags = flags[i];
    ray.Ng.x = Ng.x[i]; ray.Ng.y = Ng.y[i]; ray.Ng.z = Ng.z[i];
    ray.u = u[i]; ray.v = v[i];
    ray.primID = primID[i]; ray.geomID = geomID[i]; ray.instID = instID[i];
  }

  /* Converts single rays to ray packet */
  template<int K>
  __forceinline void RayK<K>::set(const RayK<1>* ray)
  {
    for (size_t i = 0; i < K; i++)
    {
      org.x[i] = ray[i].org.x; org.y[i] = ray[i].org.y; org.z[i] = ray[i].org.z; tnear()[i] = ray[i].tnear();
      dir.x[i] = ray[i].dir.x; dir.y[i] = ray[i].dir.y; dir.z[i] = ray[i].dir.z; tfar()[i] = ray[i].tfar();  
      time[i] = ray[i].time; mask[i] = ray[i].mask; id[i] = ray[i].id; flags[i] = ray[i].flags;
      Ng.x[i] = ray[i].Ng.x; Ng.y[i] = ray[i].Ng.y; Ng.z[i] = ray[i].Ng.z;
      u[i] = ray[i].u; v[i] = ray[i].v;
      primID[i] = ray[i].primID; geomID[i] = ray[i].geomID;  instID[i] = ray[i].instID;
    }
  }

  /* inserts a single ray into a ray packet element */
  template<int K>
  __forceinline void RayK<K>::set(size_t i, const RayK<1>& ray)
  {
    org.x[i] = ray.org.x; org.y[i] = ray.org.y; org.z[i] = ray.org.z; tnear()[i] = ray.tnear();
    dir.x[i] = ray.dir.x; dir.y[i] = ray.dir.y; dir.z[i] = ray.dir.z; tfar()[i] = ray.tfar();  
    time[i] = ray.time; mask[i] = ray.mask; id[i] = ray.id; flags[i] = ray.flags;
    Ng.x[i] = ray.Ng.x; Ng.y[i] = ray.Ng.y; Ng.z[i] = ray.Ng.z;
    u[i] = ray.u; v[i] = ray.v;
    primID[i] = ray.primID; geomID[i] = ray.geomID; instID[i] = ray.instID;
  }

  /* copies a ray packet element into another element*/
  template<int K>
  __forceinline void RayK<K>::copy(size_t dest, size_t source)
  {
    org.x[dest] = org.x[source]; org.y[dest] = org.y[source]; org.z[dest] = org.z[source]; tnear()[dest] = tnear()[source];
    dir.x[dest] = dir.x[source]; dir.y[dest] = dir.y[source]; dir.z[dest] = dir.z[source]; tfar() [dest] = tfar()[source];  
    time[dest] = time[source]; mask[dest] = mask[source]; id[dest] = id[source]; flags[dest] = flags[source]; 
    Ng.x[dest] = Ng.x[source]; Ng.y[dest] = Ng.y[source]; Ng.z[dest] = Ng.z[source];
    u[dest] = u[source]; v[dest] = v[source];
    primID[dest] = primID[source]; geomID[dest] = geomID[source];  instID[dest] = instID[source];
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
                << "  org = " << ray.org << std::endl
                << "  dir = " << ray.dir << std::endl
                << "  near = " << ray.tnear() << std::endl
                << "  far = " << ray.tfar() << std::endl
                << "  time = " << ray.time << std::endl
                << "  mask = " << ray.mask << std::endl
                << "  id = " << ray.id << std::endl
                << "  flags = " << ray.flags << std::endl
                << "  Ng = " << ray.Ng
                << "  u = " << ray.u <<  std::endl
                << "  v = " << ray.v << std::endl
                << "  primID = " << ray.primID <<  std::endl
                << "  geomID = " << ray.geomID << std::endl
                << "  instID = " << ray.instID << std::endl
                << "}";
  }


  struct RayStreamSOA
  {
    __forceinline RayStreamSOA(void* rays, size_t N)
      : ptr((char*)rays), N(N) {}

    /* ray data access functions */
    __forceinline float* org_x(size_t offset = 0) { return (float*)&ptr[0*4*N+offset]; }  //!< x coordinate of ray origin
    __forceinline float* org_y(size_t offset = 0) { return (float*)&ptr[1*4*N+offset]; }  //!< y coordinate of ray origin
    __forceinline float* org_z(size_t offset = 0) { return (float*)&ptr[2*4*N+offset]; }; //!< z coordinate of ray origin
    __forceinline float* tnear(size_t offset = 0) { return (float*)&ptr[3*4*N+offset]; }; //!< start of ray segment

    __forceinline float* dir_x(size_t offset = 0) { return (float*)&ptr[4*4*N+offset]; }; //!< x coordinate of ray direction
    __forceinline float* dir_y(size_t offset = 0) { return (float*)&ptr[5*4*N+offset]; }; //!< y coordinate of ray direction
    __forceinline float* dir_z(size_t offset = 0) { return (float*)&ptr[6*4*N+offset]; }; //!< z coordinate of ray direction
    __forceinline float* tfar (size_t offset = 0) { return (float*)&ptr[7*4*N+offset]; }; //!< end of ray segment (set to hit distance)

    __forceinline float* time(size_t offset = 0) { return (float*)&ptr[8*4*N+offset]; };  //!< time of this ray for motion blur
    __forceinline int*   mask(size_t offset = 0) { return (int*)  &ptr[9*4*N+offset]; };  //!< used to mask out objects during traversal (optional)

    __forceinline int*   id   (size_t offset = 0) { return (int*)&ptr[10*4*N+offset]; };  //!< id
    __forceinline int*   flags(size_t offset = 0) { return (int*)&ptr[11*4*N+offset]; };  //!< flags

    /* hit data access functions */
    __forceinline float* Ng_x(size_t offset = 0) { return (float*)&ptr[12*4*N+offset]; }; //!< x coordinate of geometry normal
    __forceinline float* Ng_y(size_t offset = 0) { return (float*)&ptr[13*4*N+offset]; }; //!< y coordinate of geometry normal
    __forceinline float* Ng_z(size_t offset = 0) { return (float*)&ptr[14*4*N+offset]; }; //!< z coordinate of geometry normal

    __forceinline float* u(size_t offset = 0) { return (float*)&ptr[15*4*N+offset]; };    //!< barycentric u coordinate of hit
    __forceinline float* v(size_t offset = 0) { return (float*)&ptr[16*4*N+offset]; };    //!< barycentric v coordinate of hit

    __forceinline int* primID(size_t offset = 0) { return (int*)&ptr[17*4*N+offset]; };   //!< primitive ID
    __forceinline int* geomID(size_t offset = 0) { return (int*)&ptr[18*4*N+offset]; };   //!< geometry ID
    __forceinline int* instID(size_t offset = 0) { return (int*)&ptr[19*4*N+offset]; };   //!< instance ID

    __forceinline Ray getRayByOffset(size_t offset)
    {
      Ray ray;
      ray.org.x   = org_x(offset)[0];
      ray.org.y   = org_y(offset)[0];
      ray.org.z   = org_z(offset)[0];
      ray.tnear() = tnear(offset)[0];
      ray.dir.x   = dir_x(offset)[0];
      ray.dir.y   = dir_y(offset)[0];
      ray.dir.z   = dir_z(offset)[0];
      ray.tfar()  = tfar(offset)[0];
      ray.time    = time(offset)[0];
      ray.mask    = mask(offset)[0];
      ray.id      = id(offset)[0];
      ray.flags   = flags(offset)[0];
      ray.instID  = instID(offset)[0];
      ray.geomID  = RTC_INVALID_GEOMETRY_ID;
      return ray;
    }

    template<int K>
    __forceinline RayK<K> getRayByOffset(size_t offset)
    {
      RayK<K> ray;
      ray.org.x  = vfloat<K>::loadu(org_x(offset));
      ray.org.y  = vfloat<K>::loadu(org_y(offset));
      ray.org.z  = vfloat<K>::loadu(org_z(offset));
      ray.tnear  = vfloat<K>::loadu(tnear(offset));
      ray.dir.x  = vfloat<K>::loadu(dir_x(offset));
      ray.dir.y  = vfloat<K>::loadu(dir_y(offset));
      ray.dir.z  = vfloat<K>::loadu(dir_z(offset));
      ray.tfar   = vfloat<K>::loadu(tfar(offset));
      ray.time   = vfloat<K>::loadu(time(offset));
      ray.mask   = vint<K>::loadu(mask(offset));
      ray.id     = vint<K>::loadu(id(offset));
      ray.flags  = vint<K>::loadu(flags(offset));
      ray.instID = vint<K>::loadu(instID(offset));
      ray.geomID = RTC_INVALID_GEOMETRY_ID;
      return ray;
    }

    template<int K>
    __forceinline RayK<K> getRayByOffset(const vbool<K>& valid, size_t offset)
    {
      RayK<K> ray;
      ray.org.x   = vfloat<K>::loadu(valid, org_x(offset));
      ray.org.y   = vfloat<K>::loadu(valid, org_y(offset));
      ray.org.z   = vfloat<K>::loadu(valid, org_z(offset));
      ray.tnear() = vfloat<K>::loadu(valid, tnear(offset));
      ray.dir.x   = vfloat<K>::loadu(valid, dir_x(offset));
      ray.dir.y   = vfloat<K>::loadu(valid, dir_y(offset));
      ray.dir.z   = vfloat<K>::loadu(valid, dir_z(offset));
      ray.tfar()  = vfloat<K>::loadu(valid, tfar(offset));
      ray.time    = vfloat<K>::loadu(valid, time(offset));
      ray.mask    = vint<K>::loadu(valid, mask(offset));
      ray.id      = vint<K>::loadu(valid, id(offset));
      ray.flags   = vint<K>::loadu(valid, flags(offset));
#if !defined(__AVX__)
      /* SSE: some ray members must be loaded with scalar instructions to ensure that we don't cause memory faults,
         because the SSE masked loads always access the entire vector */
      if (unlikely(!all(valid)))
      {
        ray.instID = zero;
        for (size_t k = 0; k < K; k++)
        {
          if (likely(valid[k]))
            ray.instID[k] = instID(offset)[k];
        }
      }
      else
#endif
      {
        ray.instID = vint<K>::loadu(valid, instID(offset));
      }

      ray.geomID = RTC_INVALID_GEOMETRY_ID;
      return ray;
    }

    template<int K>
    __forceinline void getRayByIndex(size_t index, RayK<K>& ray, size_t index_dest)
    {
      const size_t offset = index * sizeof(float);
      ray.org.x[index_dest]  = org_x(offset)[0];
      ray.org.y[index_dest]  = org_y(offset)[0];
      ray.org.z[index_dest]  = org_z(offset)[0];
      ray.tnear[index_dest]  = tnear(offset)[0];
      ray.dir.x[index_dest]  = dir_x(offset)[0];
      ray.dir.y[index_dest]  = dir_y(offset)[0];
      ray.dir.z[index_dest]  = dir_z(offset)[0];
      ray.tfar[index_dest]   = tfar(offset)[0];
      ray.time[index_dest]   = time(offset)[0];
      ray.mask[index_dest]   = mask(offset)[0];
      ray.id[index_dest]     = id(offset)[0];
      ray.flags[index_dest]  = flags(offset)[0];
      ray.instID[index_dest] = instID(offset)[0];
      ray.geomID[index_dest] = RTC_INVALID_GEOMETRY_ID;
    }

    template<int K>
    __forceinline void setHitByIndex(size_t index, const RayK<K>& ray, size_t index_source, bool intersect = true)
    {
      const size_t offset = index * sizeof(float);

      if (intersect)
      {
        if (ray.geomID[index_source] != RTC_INVALID_GEOMETRY_ID)
        {
          tfar(offset)[0]   = ray.tfar[index_source];
          geomID(offset)[0] = ray.geomID[index_source];
          u(offset)[0]      = ray.u[index_source];
          v(offset)[0]      = ray.v[index_source];
          primID(offset)[0] = ray.primID[index_source];
          Ng_x(offset)[0]   = ray.Ng.x[index_source];
          Ng_y(offset)[0]   = ray.Ng.y[index_source];
          Ng_z(offset)[0]   = ray.Ng.z[index_source];
          instID(offset)[0] = ray.instID[index_source];
        }
      }
      else
      {
        tfar(offset)[0] = ray.tfar[index_source];
      }
    }

    __forceinline void setHitByOffset(size_t offset, const Ray& ray, bool intersect = true)
    {
      if (intersect)
      {
        if (ray.geomID != RTC_INVALID_GEOMETRY_ID)
        {
          geomID(offset)[0] = ray.geomID;
          tfar(offset)[0]   = ray.tfar();
          u(offset)[0]      = ray.u;
          v(offset)[0]      = ray.v;
          primID(offset)[0] = ray.primID;
          Ng_x(offset)[0]   = ray.Ng.x;
          Ng_y(offset)[0]   = ray.Ng.y;
          Ng_z(offset)[0]   = ray.Ng.z;
          instID(offset)[0] = ray.instID;
        }
      }
      else
      {
        tfar(offset)[0] = ray.tfar();
      }
    }

    template<int K>
    __forceinline void setHitByOffset(const vbool<K>& valid_i, size_t offset, const RayK<K>& ray, bool intersect = true)
    {
      vbool<K> valid = valid_i;
      valid &= intersect ? (ray.geomID != RTC_INVALID_GEOMETRY_ID) : (ray.tfar() < 0.0f);

      if (likely(any(valid)))
      {
        vfloat<K>::storeu(valid, tfar(offset), ray.tfar());

        if (intersect)
        {
#if !defined(__AVX__)
          /* SSE: some ray members must be stored with scalar instructions to ensure that we don't cause memory faults,
             because the SSE masked stores always access the entire vector */
          if (unlikely(!all(valid_i)))
          {
            for (size_t k = 0; k < K; k++)
            {
              if (likely(valid[k]))
              {
                geomID(offset)[k] = ray.geomID[k];
                primID(offset)[k] = ray.primID[k];
                instID(offset)[k] = ray.instID[k];
              }
            }
          }
          else
#endif
          {
            vint<K>::storeu(valid, geomID(offset), ray.geomID);
            vint<K>::storeu(valid, primID(offset), ray.primID);
            vint<K>::storeu(valid, instID(offset), ray.instID);
          }

          vfloat<K>::storeu(valid, Ng_x(offset), ray.Ng.x);
          vfloat<K>::storeu(valid, Ng_y(offset), ray.Ng.y);
          vfloat<K>::storeu(valid, Ng_z(offset), ray.Ng.z);
          vfloat<K>::storeu(valid, u(offset), ray.u);
          vfloat<K>::storeu(valid, v(offset), ray.v);
        }
      }
    }

    __forceinline size_t getOctantByOffset(size_t offset)
    {
      const float dx = dir_x(offset)[0];
      const float dy = dir_y(offset)[0];
      const float dz = dir_z(offset)[0];
      const size_t octantID = (dx < 0.0f ? 1 : 0) + (dy < 0.0f ? 2 : 0) + (dz < 0.0f ? 4 : 0);
      return octantID;
    }

    __forceinline bool isValidByOffset(size_t offset)
    {
      const float nnear = tnear(offset)[0];
      const float ffar  = tfar(offset)[0];
      return nnear <= ffar;
    }

    template<int K>
    __forceinline RayK<K> getRayByOffset(const vbool<K>& valid, const vint<K>& offset)
    {
      RayK<K> ray;

#if defined(__AVX2__)
      ray.org.x   = vfloat<K>::template gather<1>(valid, org_x(), offset);
      ray.org.y   = vfloat<K>::template gather<1>(valid, org_y(), offset);
      ray.org.z   = vfloat<K>::template gather<1>(valid, org_z(), offset);
      ray.tnear() = vfloat<K>::template gather<1>(valid, tnear(), offset);
      ray.dir.x   = vfloat<K>::template gather<1>(valid, dir_x(), offset);
      ray.dir.y   = vfloat<K>::template gather<1>(valid, dir_y(), offset);
      ray.dir.z   = vfloat<K>::template gather<1>(valid, dir_z(), offset);
      ray.tfar()  = vfloat<K>::template gather<1>(valid, tfar(), offset);
      ray.time    = vfloat<K>::template gather<1>(valid, time(), offset);
      ray.mask    = vint<K>::template gather<1>(valid, mask(), offset);
      ray.id      = vint<K>::template gather<1>(valid, id(), offset);
      ray.flags   = vint<K>::template gather<1>(valid, flags(), offset);

      ray.instID  = vint<K>::template gather<1>(valid, instID(), offset);
#else
      ray.org     = zero;
      ray.dir     = zero;
      ray.tnear() = zero;
      ray.tfar()  = zero;
      ray.time    = zero;
      ray.mask    = zero;
      ray.id      = zero;
      ray.flags   = zero;

      ray.instID  = zero;

      for (size_t k = 0; k < K; k++)
      {
        if (likely(valid[k]))
        {
          const size_t ofs = offset[k];

          ray.org.x[k]   = *org_x(ofs);
          ray.org.y[k]   = *org_y(ofs);
          ray.org.z[k]   = *org_z(ofs);
          ray.dir.x[k]   = *dir_x(ofs);
          ray.dir.y[k]   = *dir_y(ofs);
          ray.dir.z[k]   = *dir_z(ofs);
          ray.tnear()[k] = *tnear(ofs);
          ray.tfar()[k]  = *tfar(ofs);
          ray.time[k]    = *time(ofs);
          ray.mask[k]    = *mask(ofs);
          ray.id[k]      = *id(ofs);
          ray.flags[k]   = *flags(ofs);
          ray.instID[k]  = *instID(ofs);
        }
      }
#endif

      ray.geomID = RTC_INVALID_GEOMETRY_ID;

      return ray;
    }

    template<int K>
    __forceinline void setHitByOffset(const vbool<K>& valid_i, const vint<K>& offset, const RayK<K>& ray, bool intersect = true)
    {
      vbool<K> valid = valid_i;
      valid &= intersect ? (ray.geomID != RTC_INVALID_GEOMETRY_ID) : (ray.tfar() < 0.0f);
      if (likely(any(valid)))
      {
#if defined(__AVX512F__)
        vfloat<K>::template scatter<1>(valid, tfar(), offset, ray.tfar());
        if (intersect)
        {
          vint<K>::template scatter<1>(valid, geomID(), offset, ray.geomID);
          vfloat<K>::template scatter<1>(valid, u(), offset, ray.u);
          vfloat<K>::template scatter<1>(valid, v(), offset, ray.v);
          vint<K>::template scatter<1>(valid, primID(), offset, ray.primID);
          vfloat<K>::template scatter<1>(valid, Ng_x(), offset, ray.Ng.x);
          vfloat<K>::template scatter<1>(valid, Ng_y(), offset, ray.Ng.y);
          vfloat<K>::template scatter<1>(valid, Ng_z(), offset, ray.Ng.z);
          vint<K>::template scatter<1>(valid, instID(), offset, ray.instID);
        }
#else
        size_t valid_bits = movemask(valid);
        while (valid_bits != 0)
        {
          const size_t k = __bscf(valid_bits);
          const size_t ofs = offset[k];

          *tfar(ofs) = ray.tfar()[k];
          if (intersect)
          {
            *geomID(ofs) = ray.geomID[k];
            *u(ofs)      = ray.u[k];
            *v(ofs)      = ray.v[k];
            *primID(ofs) = ray.primID[k];
            *Ng_x(ofs)   = ray.Ng.x[k];
            *Ng_y(ofs)   = ray.Ng.y[k];
            *Ng_z(ofs)   = ray.Ng.z[k];
            *instID(ofs) = ray.instID[k];
          }
        }
#endif
      }
    }

    char* __restrict__ ptr;
    size_t N;
  };

  template<size_t MAX_K>
  struct StackRayStreamSOA : public RayStreamSOA
  {
    __forceinline StackRayStreamSOA(size_t K)
      : RayStreamSOA(data, K) { assert(K <= MAX_K); }

    char data[MAX_K / 4 * sizeof(Ray4)];
  };


  struct RayStreamSOP
  {
    template<class T>
    __forceinline void init(T& t)
    {
      org_x  = (float*)&t.org.x;
      org_y  = (float*)&t.org.y;
      org_z  = (float*)&t.org.z;
      tnear  = (float*)&t.tnear;
      dir_x  = (float*)&t.dir.x;
      dir_y  = (float*)&t.dir.y;
      dir_z  = (float*)&t.dir.z;
      tfar   = (float*)&t.tfar;
      time   = (float*)&t.time;
      mask   = (unsigned int*)&t.mask;
      id     = (unsigned int*)&t.id;
      flags  = (unsigned int*)&t.flags;

      Ng_x   = (float*)&t.Ng.x;
      Ng_y   = (float*)&t.Ng.y;
      Ng_z   = (float*)&t.Ng.z;
      u      = (float*)&t.u;
      v      = (float*)&t.v;
      primID = (unsigned int*)&t.primID;
      geomID = (unsigned int*)&t.geomID;
      instID = (unsigned int*)&t.instID;
    }

    __forceinline Ray getRayByOffset(size_t offset)
    {
      Ray ray;
      ray.org.x   = *(float* __restrict__)((char*)org_x + offset);
      ray.org.y   = *(float* __restrict__)((char*)org_y + offset);
      ray.org.z   = *(float* __restrict__)((char*)org_z + offset);
      ray.dir.x   = *(float* __restrict__)((char*)dir_x + offset);
      ray.dir.y   = *(float* __restrict__)((char*)dir_y + offset);
      ray.dir.z   = *(float* __restrict__)((char*)dir_z + offset);
      ray.tfar()  = *(float* __restrict__)((char*)tfar + offset);
      ray.tnear() = tnear ? *(float* __restrict__)((char*)tnear + offset) : 0.0f;
      ray.time    = time ? *(float* __restrict__)((char*)time + offset) : 0.0f;
      ray.mask    = mask ? *(unsigned int* __restrict__)((char*)mask + offset) : -1;
      ray.id      = id ? *(unsigned int* __restrict__)((char*)id + offset) : -1;
      ray.flags   = flags ? *(unsigned int* __restrict__)((char*)flags + offset) : -1;

      ray.instID  = instID ? *(unsigned int* __restrict__)((char*)instID + offset) : -1;
      ray.geomID  = RTC_INVALID_GEOMETRY_ID;
      return ray;
    }

    template<int K>
    __forceinline RayK<K> getRayByOffset(const vbool<K>& valid, size_t offset)
    {
      RayK<K> ray;
      ray.org.x   = vfloat<K>::loadu(valid, (float* __restrict__)((char*)org_x + offset));
      ray.org.y   = vfloat<K>::loadu(valid, (float* __restrict__)((char*)org_y + offset));
      ray.org.z   = vfloat<K>::loadu(valid, (float* __restrict__)((char*)org_z + offset));
      ray.dir.x   = vfloat<K>::loadu(valid, (float* __restrict__)((char*)dir_x + offset));
      ray.dir.y   = vfloat<K>::loadu(valid, (float* __restrict__)((char*)dir_y + offset));
      ray.dir.z   = vfloat<K>::loadu(valid, (float* __restrict__)((char*)dir_z + offset));
      ray.tfar()  = vfloat<K>::loadu(valid, (float* __restrict__)((char*)tfar + offset));
      ray.tnear() = tnear ? vfloat<K>::loadu(valid, (float* __restrict__)((char*)tnear + offset)) : 0.0f;
      ray.time    = time ? vfloat<K>::loadu(valid, (float* __restrict__)((char*)time + offset)) : 0.0f;
      ray.mask    = mask ? vint<K>::loadu(valid, (const void* __restrict__)((char*)mask + offset)) : -1;
      ray.id      = id ? vint<K>::loadu(valid, (const void* __restrict__)((char*)id + offset)) : -1;
      ray.flags   = flags ? vint<K>::loadu(valid, (const void* __restrict__)((char*)flags + offset)) : -1;

      ray.instID  = instID ? vint<K>::loadu(valid, (const void* __restrict__)((char*)instID + offset)) : -1;
      ray.geomID  = RTC_INVALID_GEOMETRY_ID;
      return ray;
    }

    template<int K>
    __forceinline Vec3vf<K> getDirByOffset(const vbool<K>& valid, size_t offset)
    {
      Vec3vf<K> dir;
      dir.x = vfloat<K>::loadu(valid, (float* __restrict__)((char*)dir_x + offset));
      dir.y = vfloat<K>::loadu(valid, (float* __restrict__)((char*)dir_y + offset));
      dir.z = vfloat<K>::loadu(valid, (float* __restrict__)((char*)dir_z + offset));
      return dir;
    }

    __forceinline void setHitByOffset(size_t offset, const Ray& ray, bool intersect = true)
    {
      if (intersect)
      {
        if (ray.geomID != RTC_INVALID_GEOMETRY_ID)
        {
          *(unsigned int* __restrict__)((char*)geomID + offset) = ray.geomID;
          *(float* __restrict__)((char*)tfar + offset) = ray.tfar();
          *(float* __restrict__)((char*)u + offset) = ray.u;
          *(float* __restrict__)((char*)v + offset) = ray.v;
          *(unsigned int* __restrict__)((char*)primID + offset) = ray.primID;
          if (likely(Ng_x)) *(float* __restrict__)((char*)Ng_x + offset) = ray.Ng.x;
          if (likely(Ng_y)) *(float* __restrict__)((char*)Ng_y + offset) = ray.Ng.y;
          if (likely(Ng_z)) *(float* __restrict__)((char*)Ng_z + offset) = ray.Ng.z;
          if (likely(instID)) *(unsigned int* __restrict__)((char*)instID + offset) = ray.instID;
        }
      }
      else
      {
        *(float* __restrict__)((char*)tfar + offset) = ray.tfar();
      }
    }

    template<int K>
    __forceinline void setHitByOffset(const vbool<K>& valid_i, size_t offset, const RayK<K>& ray, bool intersect = true)
    {
      vbool<K> valid = valid_i;
      valid &= intersect ? (ray.geomID != RTC_INVALID_GEOMETRY_ID) : (ray.tfar() < 0.0f);

      if (likely(any(valid)))
      {
        vfloat<K>::storeu(valid, (float* __restrict__)((char*)tfar + offset), ray.tfar());
        if (intersect)
        {
          vint<K>::storeu(valid, (int* __restrict__)((char*)geomID + offset), ray.geomID);
          vfloat<K>::storeu(valid, (float* __restrict__)((char*)u + offset), ray.u);
          vfloat<K>::storeu(valid, (float* __restrict__)((char*)v + offset), ray.v);
          vint<K>::storeu(valid, (int* __restrict__)((char*)primID + offset), ray.primID);
          if (likely(Ng_x)) vfloat<K>::storeu(valid, (float* __restrict__)((char*)Ng_x + offset), ray.Ng.x);
          if (likely(Ng_y)) vfloat<K>::storeu(valid, (float* __restrict__)((char*)Ng_y + offset), ray.Ng.y);
          if (likely(Ng_z)) vfloat<K>::storeu(valid, (float* __restrict__)((char*)Ng_z + offset), ray.Ng.z);
          if (likely(instID)) vint<K>::storeu(valid, (int* __restrict__)((char*)instID + offset), ray.instID);
        }
      }
    }

    __forceinline size_t getOctantByOffset(size_t offset)
    {
      const float dx = *(float* __restrict__)((char*)dir_x + offset);
      const float dy = *(float* __restrict__)((char*)dir_y + offset);
      const float dz = *(float* __restrict__)((char*)dir_z + offset);
      const size_t octantID = (dx < 0.0f ? 1 : 0) + (dy < 0.0f ? 2 : 0) + (dz < 0.0f ? 4 : 0);
      return octantID;
    }

    __forceinline bool isValidByOffset(size_t offset)
    {
      const float nnear = tnear ? *(float* __restrict__)((char*)tnear + offset) : 0.0f;
      const float ffar  = *(float* __restrict__)((char*)tfar + offset);
      return nnear <= ffar;
    }

    template<int K>
    __forceinline vbool<K> isValidByOffset(const vbool<K>& valid, size_t offset)
    {
      const vfloat<K> nnear = tnear ? vfloat<K>::loadu(valid, (float* __restrict__)((char*)tnear + offset)) : 0.0f;
      const vfloat<K> ffar  = vfloat<K>::loadu(valid, (float* __restrict__)((char*)tfar + offset));
      return nnear <= ffar;
    }

    template<int K>
    __forceinline RayK<K> getRayByOffset(const vbool<K>& valid, const vint<K>& offset)
    {
      RayK<K> ray;

#if defined(__AVX2__)
      ray.org.x   = vfloat<K>::template gather<1>(valid, org_x, offset);
      ray.org.y   = vfloat<K>::template gather<1>(valid, org_y, offset);
      ray.org.z   = vfloat<K>::template gather<1>(valid, org_z, offset);
      ray.dir.x   = vfloat<K>::template gather<1>(valid, dir_x, offset);
      ray.dir.y   = vfloat<K>::template gather<1>(valid, dir_y, offset);
      ray.dir.z   = vfloat<K>::template gather<1>(valid, dir_z, offset);
      ray.tfar()  = vfloat<K>::template gather<1>(valid, tfar, offset);
      ray.tnear() = tnear ? vfloat<K>::template gather<1>(valid, tnear, offset) : vfloat<K>(zero);
      ray.time    = time ? vfloat<K>::template gather<1>(valid, time, offset) : vfloat<K>(zero);
      ray.mask    = mask ? vint<K>::template gather<1>(valid, (int*)mask, offset) : vint<K>(-1);
      ray.id      = id ? vint<K>::template gather<1>(valid, (int*)id, offset) : vint<K>(-1);
      ray.flags   = flags ? vint<K>::template gather<1>(valid, (int*)flags, offset) : vint<K>(-1);

      ray.instID  = instID ? vint<K>::template gather<1>(valid, (int*)instID, offset) : vint<K>(-1);
#else
      ray.org     = zero;
      ray.dir     = zero;
      ray.tnear() = zero;
      ray.tfar()  = zero;
      ray.time    = zero;
      ray.mask    = zero;
      ray.id      = zero;
      ray.flags   = zero;

      ray.instID  = zero;

      for (size_t k = 0; k < K; k++)
      {
        if (likely(valid[k]))
        {
          const size_t ofs = offset[k];

          ray.org.x[k]   = *(float* __restrict__)((char*)org_x + ofs);
          ray.org.y[k]   = *(float* __restrict__)((char*)org_y + ofs);
          ray.org.z[k]   = *(float* __restrict__)((char*)org_z + ofs);
          ray.dir.x[k]   = *(float* __restrict__)((char*)dir_x + ofs);
          ray.dir.y[k]   = *(float* __restrict__)((char*)dir_y + ofs);
          ray.dir.z[k]   = *(float* __restrict__)((char*)dir_z + ofs);
          ray.tfar()[k]  = *(float* __restrict__)((char*)tfar + ofs);
          ray.tnear()[k] = tnear ? *(float* __restrict__)((char*)tnear + ofs) : 0.0f;
          ray.time[k]    = time ? *(float* __restrict__)((char*)time + ofs) : 0.0f;
          ray.mask[k]    = mask ? *(unsigned int* __restrict__)((char*)mask + ofs) : -1;

          ray.id[k]      = id ? *(unsigned int* __restrict__)((char*)id + ofs) : -1;
          ray.flags[k]   = flags ? *(unsigned int* __restrict__)((char*)flags + ofs) : -1;

          ray.instID[k]  = instID ? *(unsigned int* __restrict__)((char*)instID + ofs) : -1;
        }
      }
#endif

      ray.geomID = RTC_INVALID_GEOMETRY_ID;
      return ray;
    }

    template<int K>
    __forceinline void setHitByOffset(const vbool<K>& valid_i, const vint<K>& offset, const RayK<K>& ray, bool intersect = true)
    {
      vbool<K> valid = valid_i;
      valid &= intersect ? (ray.geomID != RTC_INVALID_GEOMETRY_ID) : (ray.tfar() < 0.0f);

      if (likely(any(valid)))
      {
#if defined(__AVX512F__)
        vfloat<K>::template scatter<1>(valid, tfar, offset, ray.tfar());
        if (intersect)
        {
          vint<K>::template scatter<1>(valid, (int*)geomID, offset, ray.geomID);
          vfloat<K>::template scatter<1>(valid, u, offset, ray.u);
          vfloat<K>::template scatter<1>(valid, v, offset, ray.v);
          vint<K>::template scatter<1>(valid, (int*)primID, offset, ray.primID);
          if (likely(Ng_x)) vfloat<K>::template scatter<1>(valid, Ng_x, offset, ray.Ng.x);
          if (likely(Ng_y)) vfloat<K>::template scatter<1>(valid, Ng_y, offset, ray.Ng.y);
          if (likely(Ng_z)) vfloat<K>::template scatter<1>(valid, Ng_z, offset, ray.Ng.z);
          if (likely(instID)) vint<K>::template scatter<1>(valid, (int*)instID, offset, ray.instID);
        }
#else
        size_t valid_bits = movemask(valid);
        while (valid_bits != 0)
        {
          const size_t k = __bscf(valid_bits);
          const size_t ofs = offset[k];
          *(float* __restrict__)((char*)tfar + ofs) = ray.tfar()[k];
          if (intersect)
          {
            *(unsigned int* __restrict__)((char*)geomID + ofs) = ray.geomID[k];
            *(float* __restrict__)((char*)u + ofs) = ray.u[k];
            *(float* __restrict__)((char*)v + ofs) = ray.v[k];
            *(unsigned int* __restrict__)((char*)primID + ofs) = ray.primID[k];
            if (likely(Ng_x)) *(float* __restrict__)((char*)Ng_x + ofs) = ray.Ng.x[k];
            if (likely(Ng_y)) *(float* __restrict__)((char*)Ng_y + ofs) = ray.Ng.y[k];
            if (likely(Ng_z)) *(float* __restrict__)((char*)Ng_z + ofs) = ray.Ng.z[k];
            if (likely(instID)) *(unsigned int* __restrict__)((char*)instID + ofs) = ray.instID[k];
          }
        }
#endif
      }
    }

    /* ray data */
    float* __restrict__ org_x; //!< x coordinate of ray origin
    float* __restrict__ org_y; //!< y coordinate of ray origin
    float* __restrict__ org_z; //!< z coordinate of ray origin
    float* __restrict__ tnear; //!< start of ray segment (optional)

    float* __restrict__ dir_x; //!< x coordinate of ray direction
    float* __restrict__ dir_y; //!< y coordinate of ray direction
    float* __restrict__ dir_z; //!< z coordinate of ray direction
    float* __restrict__ tfar;  //!< end of ray segment (set to hit distance)

    float* __restrict__ time;         //!< time of this ray for motion blur (optional)
    unsigned int* __restrict__ mask;  //!< used to mask out objects during traversal (optional)

    unsigned int* __restrict__ id;    //!< ray ID
    unsigned int* __restrict__ flags; //!< ray flags

    /* hit data */
    float* __restrict__ Ng_x; //!< x coordinate of geometry normal (optional)
    float* __restrict__ Ng_y; //!< y coordinate of geometry normal (optional)
    float* __restrict__ Ng_z; //!< z coordinate of geometry normal (optional)

    float* __restrict__ u;    //!< barycentric u coordinate of hit
    float* __restrict__ v;    //!< barycentric v coordinate of hit

    unsigned int* __restrict__ primID; //!< primitive ID
    unsigned int* __restrict__ geomID; //!< geometry ID
    unsigned int* __restrict__ instID; //!< instance ID (optional)
  };


  struct RayStreamAOS
  {
    __forceinline RayStreamAOS(void* rays)
      : ptr((Ray*)rays) {}

    __forceinline Ray& getRayByOffset(size_t offset)
    {
      return *(Ray*)((char*)ptr + offset);
    }

    template<int K>
    __forceinline RayK<K> getRayByOffset(const vint<K>& offset);

    template<int K>
    __forceinline RayK<K> getRayByOffset(const vbool<K>& valid, const vint<K>& offset)
    {
      const vint<K> valid_offset = select(valid, offset, vintx(zero));
      return getRayByOffset(valid_offset);
    }

    template<int K>
    __forceinline void setHitByOffset(const vbool<K>& valid_i, const vint<K>& offset, const RayK<K>& ray, bool intersect = true)
    {
      vbool<K> valid = valid_i;
      valid &= intersect ? (ray.geomID != RTC_INVALID_GEOMETRY_ID) : (ray.tfar() < 0.0f);

      if (likely(any(valid)))
      {
#if defined(__AVX512F__)
        vfloat<K>::template scatter<1>(valid, &ptr->tfar(), offset, ray.tfar());
        if (intersect)
        {
          vint<K>::template scatter<1>(valid, (int*)&ptr->geomID, offset, ray.geomID);
          vfloat<K>::template scatter<1>(valid, &ptr->Ng.x, offset, ray.Ng.x);
          vfloat<K>::template scatter<1>(valid, &ptr->Ng.y, offset, ray.Ng.y);
          vfloat<K>::template scatter<1>(valid, &ptr->Ng.z, offset, ray.Ng.z);
          vfloat<K>::template scatter<1>(valid, &ptr->u, offset, ray.u);
          vfloat<K>::template scatter<1>(valid, &ptr->v, offset, ray.v);
          vint<K>::template scatter<1>(valid, (int*)&ptr->primID, offset, ray.primID);
          vint<K>::template scatter<1>(valid, (int*)&ptr->instID, offset, ray.instID);
        }
#else
        size_t valid_bits = movemask(valid);
        while (valid_bits != 0)
        {
          const size_t k = __bscf(valid_bits);
          Ray* __restrict__ ray_k = (Ray*)((char*)ptr + offset[k]);

          ray_k->tfar() = ray.tfar()[k];
          if (intersect)
          {
            ray_k->geomID = ray.geomID[k];
            ray_k->Ng.x   = ray.Ng.x[k];
            ray_k->Ng.y   = ray.Ng.y[k];
            ray_k->Ng.z   = ray.Ng.z[k];
            ray_k->u      = ray.u[k];
            ray_k->v      = ray.v[k];
            ray_k->primID = ray.primID[k];
            ray_k->instID = ray.instID[k];
          }
        }
#endif
      }
    }

    Ray* __restrict__ ptr;
  };

  template<>
  __forceinline Ray4 RayStreamAOS::getRayByOffset(const vint4& offset)
  {
    Ray4 ray;
    /* gather: instID */
    ray.instID = vint4::gather<1>((int*)&ptr->instID, offset);

    /* load and transpose: org.x, org.y, org.z */
    const vfloat4 a0 = vfloat4::loadu(&((Ray*)((char*)ptr + offset[0]))->org);
    const vfloat4 a1 = vfloat4::loadu(&((Ray*)((char*)ptr + offset[1]))->org);
    const vfloat4 a2 = vfloat4::loadu(&((Ray*)((char*)ptr + offset[2]))->org);
    const vfloat4 a3 = vfloat4::loadu(&((Ray*)((char*)ptr + offset[3]))->org);

    transpose(a0,a1,a2,a3, ray.org.x, ray.org.y, ray.org.z, ray.tnear());

    /* load and transpose: dir.x, dir.y, dir.z */
    const vfloat4 b0 = vfloat4::loadu(&((Ray*)((char*)ptr + offset[0]))->dir);
    const vfloat4 b1 = vfloat4::loadu(&((Ray*)((char*)ptr + offset[1]))->dir);
    const vfloat4 b2 = vfloat4::loadu(&((Ray*)((char*)ptr + offset[2]))->dir);
    const vfloat4 b3 = vfloat4::loadu(&((Ray*)((char*)ptr + offset[3]))->dir);

    transpose(b0,b1,b2,b3, ray.dir.x, ray.dir.y, ray.dir.z, ray.tfar());

    /* load and transpose: tnear, tfar, time, mask */
    const vfloat4 c0 = vfloat4::loadu(&((Ray*)((char*)ptr + offset[0]))->time);
    const vfloat4 c1 = vfloat4::loadu(&((Ray*)((char*)ptr + offset[1]))->time);
    const vfloat4 c2 = vfloat4::loadu(&((Ray*)((char*)ptr + offset[2]))->time);
    const vfloat4 c3 = vfloat4::loadu(&((Ray*)((char*)ptr + offset[3]))->time);

    vfloat4 maskf, idf, flagsf;
    transpose(c0,c1,c2,c3, ray.time, maskf, idf, flagsf);
    ray.mask  = asInt(maskf);
    ray.id    = asInt(idf);
    ray.flags = asInt(flagsf);

    ray.geomID = RTC_INVALID_GEOMETRY_ID;
    return ray;
  }

#if defined(__AVX__)
  template<>
  __forceinline Ray8 RayStreamAOS::getRayByOffset(const vint8& offset)
  {
    Ray8 ray;

    /* gather: instID */
    ray.instID = vint8::gather<1>((int*)&ptr->instID, offset);

    /* load and transpose: org.x, org.y, org.z, tnear, dir.x, dir.y, dir.z, tfar */
    const vfloat8 ab0 = vfloat8::loadu(&((Ray*)((char*)ptr + offset[0]))->org);
    const vfloat8 ab1 = vfloat8::loadu(&((Ray*)((char*)ptr + offset[1]))->org);
    const vfloat8 ab2 = vfloat8::loadu(&((Ray*)((char*)ptr + offset[2]))->org);
    const vfloat8 ab3 = vfloat8::loadu(&((Ray*)((char*)ptr + offset[3]))->org);
    const vfloat8 ab4 = vfloat8::loadu(&((Ray*)((char*)ptr + offset[4]))->org);
    const vfloat8 ab5 = vfloat8::loadu(&((Ray*)((char*)ptr + offset[5]))->org);
    const vfloat8 ab6 = vfloat8::loadu(&((Ray*)((char*)ptr + offset[6]))->org);
    const vfloat8 ab7 = vfloat8::loadu(&((Ray*)((char*)ptr + offset[7]))->org);

    transpose(ab0,ab1,ab2,ab3,ab4,ab5,ab6,ab7, ray.org.x, ray.org.y, ray.org.z, ray.tnear(), ray.dir.x, ray.dir.y, ray.dir.z, ray.tfar());

    /* load and transpose: time, mask, id, flags */
    const vfloat4 c0 = vfloat4::loadu(&((Ray*)((char*)ptr + offset[0]))->time);
    const vfloat4 c1 = vfloat4::loadu(&((Ray*)((char*)ptr + offset[1]))->time);
    const vfloat4 c2 = vfloat4::loadu(&((Ray*)((char*)ptr + offset[2]))->time);
    const vfloat4 c3 = vfloat4::loadu(&((Ray*)((char*)ptr + offset[3]))->time);
    const vfloat4 c4 = vfloat4::loadu(&((Ray*)((char*)ptr + offset[4]))->time);
    const vfloat4 c5 = vfloat4::loadu(&((Ray*)((char*)ptr + offset[5]))->time);
    const vfloat4 c6 = vfloat4::loadu(&((Ray*)((char*)ptr + offset[6]))->time);
    const vfloat4 c7 = vfloat4::loadu(&((Ray*)((char*)ptr + offset[7]))->time);

    vfloat8 maskf, idf, flagsf;
    transpose(c0,c1,c2,c3,c4,c5,c6,c7, ray.time, maskf, idf, flagsf);
    ray.mask  = asInt(maskf);
    ray.id    = asInt(idf);
    ray.flags = asInt(flagsf);

    ray.geomID = RTC_INVALID_GEOMETRY_ID;
    return ray;
  }
#endif

#if defined(__AVX512F__)
  template<>
  __forceinline Ray16 RayStreamAOS::getRayByOffset(const vint16& offset)
  {
    Ray16 ray;

    /* gather: instID */
    ray.instID = vint16::gather<1>((int*)&ptr->instID, offset);

    /* load and transpose: org.x, org.y, org.z, tnear, dir.x, dir.y, dir.z, tfar */
    const vfloat8 ab0  = vfloat8::loadu(&((Ray*)((char*)ptr + offset[ 0]))->org);
    const vfloat8 ab1  = vfloat8::loadu(&((Ray*)((char*)ptr + offset[ 1]))->org);
    const vfloat8 ab2  = vfloat8::loadu(&((Ray*)((char*)ptr + offset[ 2]))->org);
    const vfloat8 ab3  = vfloat8::loadu(&((Ray*)((char*)ptr + offset[ 3]))->org);
    const vfloat8 ab4  = vfloat8::loadu(&((Ray*)((char*)ptr + offset[ 4]))->org);
    const vfloat8 ab5  = vfloat8::loadu(&((Ray*)((char*)ptr + offset[ 5]))->org);
    const vfloat8 ab6  = vfloat8::loadu(&((Ray*)((char*)ptr + offset[ 6]))->org);
    const vfloat8 ab7  = vfloat8::loadu(&((Ray*)((char*)ptr + offset[ 7]))->org);
    const vfloat8 ab8  = vfloat8::loadu(&((Ray*)((char*)ptr + offset[ 8]))->org);
    const vfloat8 ab9  = vfloat8::loadu(&((Ray*)((char*)ptr + offset[ 9]))->org);
    const vfloat8 ab10 = vfloat8::loadu(&((Ray*)((char*)ptr + offset[10]))->org);
    const vfloat8 ab11 = vfloat8::loadu(&((Ray*)((char*)ptr + offset[11]))->org);
    const vfloat8 ab12 = vfloat8::loadu(&((Ray*)((char*)ptr + offset[12]))->org);
    const vfloat8 ab13 = vfloat8::loadu(&((Ray*)((char*)ptr + offset[13]))->org);
    const vfloat8 ab14 = vfloat8::loadu(&((Ray*)((char*)ptr + offset[14]))->org);
    const vfloat8 ab15 = vfloat8::loadu(&((Ray*)((char*)ptr + offset[15]))->org);

    transpose(ab0,ab1,ab2,ab3,ab4,ab5,ab6,ab7,ab8,ab9,ab10,ab11,ab12,ab13,ab14,ab15,
              ray.org.x, ray.org.y, ray.org.z, ray.tnear(), ray.dir.x, ray.dir.y, ray.dir.z, ray.tfar());

    /* load and transpose: time, mask, id, flags */
    const vfloat4 c0  = vfloat4::loadu(&((Ray*)((char*)ptr + offset[ 0]))->time);
    const vfloat4 c1  = vfloat4::loadu(&((Ray*)((char*)ptr + offset[ 1]))->time);
    const vfloat4 c2  = vfloat4::loadu(&((Ray*)((char*)ptr + offset[ 2]))->time);
    const vfloat4 c3  = vfloat4::loadu(&((Ray*)((char*)ptr + offset[ 3]))->time);
    const vfloat4 c4  = vfloat4::loadu(&((Ray*)((char*)ptr + offset[ 4]))->time);
    const vfloat4 c5  = vfloat4::loadu(&((Ray*)((char*)ptr + offset[ 5]))->time);
    const vfloat4 c6  = vfloat4::loadu(&((Ray*)((char*)ptr + offset[ 6]))->time);
    const vfloat4 c7  = vfloat4::loadu(&((Ray*)((char*)ptr + offset[ 7]))->time);
    const vfloat4 c8  = vfloat4::loadu(&((Ray*)((char*)ptr + offset[ 8]))->time);
    const vfloat4 c9  = vfloat4::loadu(&((Ray*)((char*)ptr + offset[ 9]))->time);
    const vfloat4 c10 = vfloat4::loadu(&((Ray*)((char*)ptr + offset[10]))->time);
    const vfloat4 c11 = vfloat4::loadu(&((Ray*)((char*)ptr + offset[11]))->time);
    const vfloat4 c12 = vfloat4::loadu(&((Ray*)((char*)ptr + offset[12]))->time);
    const vfloat4 c13 = vfloat4::loadu(&((Ray*)((char*)ptr + offset[13]))->time);
    const vfloat4 c14 = vfloat4::loadu(&((Ray*)((char*)ptr + offset[14]))->time);
    const vfloat4 c15 = vfloat4::loadu(&((Ray*)((char*)ptr + offset[15]))->time);

    vfloat16 maskf, idf, flagsf;
    transpose(c0,c1,c2,c3,c4,c5,c6,c7,c8,c9,c10,c11,c12,c13,c14,c15,
              ray.time, maskf, idf, flagsf);
    ray.mask  = asInt(maskf);
    ray.id    = asInt(idf);
    ray.flags = asInt(flagsf);

    ray.geomID = RTC_INVALID_GEOMETRY_ID;
    return ray;
  }
#endif


  struct RayStreamAOP
  {
    __forceinline RayStreamAOP(void* rays)
      : ptr((Ray**)rays) {}

    __forceinline Ray& getRayByIndex(size_t index)
    {
      return *ptr[index];
    }

    template<int K>
    __forceinline RayK<K> getRayByIndex(const vint<K>& index);

    template<int K>
    __forceinline RayK<K> getRayByIndex(const vbool<K>& valid, const vint<K>& index)
    {
      const vint<K> valid_index = select(valid, index, vintx(zero));
      return getRayByIndex(valid_index);
    }

    template<int K>
    __forceinline void setHitByIndex(const vbool<K>& valid_i, const vint<K>& index, const RayK<K>& ray, bool intersect = true)
    {
      vbool<K> valid = valid_i;
      valid &= intersect ? (ray.geomID != RTC_INVALID_GEOMETRY_ID) : (ray.tfar() < 0.0f);

      if (likely(any(valid)))
      {
        size_t valid_bits = movemask(valid);
        while (valid_bits != 0)
        {
          const size_t k = __bscf(valid_bits);
          Ray* __restrict__ ray_k = ptr[index[k]];

          ray_k->tfar() = ray.tfar()[k];
          if (intersect)
          {
            ray_k->geomID = ray.geomID[k];
            ray_k->Ng.x   = ray.Ng.x[k];
            ray_k->Ng.y   = ray.Ng.y[k];
            ray_k->Ng.z   = ray.Ng.z[k];
            ray_k->u      = ray.u[k];
            ray_k->v      = ray.v[k];
            ray_k->primID = ray.primID[k];
            ray_k->instID = ray.instID[k];
          }
        }
      }
    }

    Ray** __restrict__ ptr;
  };

  template<>
  __forceinline Ray4 RayStreamAOP::getRayByIndex(const vint4& index)
  {
    Ray4 ray;

    /* gather: instID */
    for (size_t k = 0; k < 4; k++)
      ray.instID[k] = ptr[index[k]]->instID;

    /* load and transpose: org.x, org.y, org.z */
    const vfloat4 a0 = vfloat4::loadu(&ptr[index[0]]->org);
    const vfloat4 a1 = vfloat4::loadu(&ptr[index[1]]->org);
    const vfloat4 a2 = vfloat4::loadu(&ptr[index[2]]->org);
    const vfloat4 a3 = vfloat4::loadu(&ptr[index[3]]->org);

    transpose(a0,a1,a2,a3, ray.org.x, ray.org.y, ray.org.z, ray.tnear());

    /* load and transpose: dir.x, dir.y, dir.z */
    const vfloat4 b0 = vfloat4::loadu(&ptr[index[0]]->dir);
    const vfloat4 b1 = vfloat4::loadu(&ptr[index[1]]->dir);
    const vfloat4 b2 = vfloat4::loadu(&ptr[index[2]]->dir);
    const vfloat4 b3 = vfloat4::loadu(&ptr[index[3]]->dir);

    transpose(b0,b1,b2,b3, ray.dir.x, ray.dir.y, ray.dir.z, ray.tfar());

    /* load and transpose: tnear, tfar, time, mask */
    const vfloat4 c0 = vfloat4::loadu(&ptr[index[0]]->time);
    const vfloat4 c1 = vfloat4::loadu(&ptr[index[1]]->time);
    const vfloat4 c2 = vfloat4::loadu(&ptr[index[2]]->time);
    const vfloat4 c3 = vfloat4::loadu(&ptr[index[3]]->time);

    vfloat4 maskf, idf, flagsf;
    transpose(c0,c1,c2,c3, ray.time, maskf, idf, flagsf);
    ray.mask  = asInt(maskf);
    ray.id    = asInt(idf);
    ray.flags = asInt(flagsf);

    ray.geomID = RTC_INVALID_GEOMETRY_ID;
    return ray;
  }

#if defined(__AVX__)
  template<>
  __forceinline Ray8 RayStreamAOP::getRayByIndex(const vint8& index)
  {
    Ray8 ray;

    /* gather: instID */
    for (size_t k = 0; k < 8; k++)
      ray.instID[k] = ptr[index[k]]->instID;

    /* load and transpose: org.x, org.y, org.z, tnear, dir.x, dir.y, dir.z, tfar */
    const vfloat8 ab0 = vfloat8::loadu(&ptr[index[0]]->org);
    const vfloat8 ab1 = vfloat8::loadu(&ptr[index[1]]->org);
    const vfloat8 ab2 = vfloat8::loadu(&ptr[index[2]]->org);
    const vfloat8 ab3 = vfloat8::loadu(&ptr[index[3]]->org);
    const vfloat8 ab4 = vfloat8::loadu(&ptr[index[4]]->org);
    const vfloat8 ab5 = vfloat8::loadu(&ptr[index[5]]->org);
    const vfloat8 ab6 = vfloat8::loadu(&ptr[index[6]]->org);
    const vfloat8 ab7 = vfloat8::loadu(&ptr[index[7]]->org);

    transpose(ab0,ab1,ab2,ab3,ab4,ab5,ab6,ab7, ray.org.x, ray.org.y, ray.org.z, ray.tnear(), ray.dir.x, ray.dir.y, ray.dir.z, ray.tfar());

    /* load and transpose: time, mask, id, flags */
    const vfloat4 c0 = vfloat4::loadu(&ptr[index[0]]->time);
    const vfloat4 c1 = vfloat4::loadu(&ptr[index[1]]->time);
    const vfloat4 c2 = vfloat4::loadu(&ptr[index[2]]->time);
    const vfloat4 c3 = vfloat4::loadu(&ptr[index[3]]->time);
    const vfloat4 c4 = vfloat4::loadu(&ptr[index[4]]->time);
    const vfloat4 c5 = vfloat4::loadu(&ptr[index[5]]->time);
    const vfloat4 c6 = vfloat4::loadu(&ptr[index[6]]->time);
    const vfloat4 c7 = vfloat4::loadu(&ptr[index[7]]->time);

    vfloat8 maskf, idf, flagsf;
    transpose(c0,c1,c2,c3,c4,c5,c6,c7, ray.time, maskf, idf, flagsf);
    ray.mask  = asInt(maskf);
    ray.id    = asInt(idf);
    ray.flags = asInt(flagsf);

    ray.geomID = RTC_INVALID_GEOMETRY_ID;
    return ray;
  }
#endif

#if defined(__AVX512F__)
  template<>
  __forceinline Ray16 RayStreamAOP::getRayByIndex(const vint16& index)
  {
    Ray16 ray;

    /* gather: instID */
    for (size_t k = 0; k < 16; k++)
      ray.instID[k] = ptr[index[k]]->instID;

    /* load and transpose: org.x, org.y, org.z, tnear, dir.x, dir.y, dir.z, tfar */
    const vfloat8 ab0  = vfloat8::loadu(&ptr[index[0]]->org);
    const vfloat8 ab1  = vfloat8::loadu(&ptr[index[1]]->org);
    const vfloat8 ab2  = vfloat8::loadu(&ptr[index[2]]->org);
    const vfloat8 ab3  = vfloat8::loadu(&ptr[index[3]]->org);
    const vfloat8 ab4  = vfloat8::loadu(&ptr[index[4]]->org);
    const vfloat8 ab5  = vfloat8::loadu(&ptr[index[5]]->org);
    const vfloat8 ab6  = vfloat8::loadu(&ptr[index[6]]->org);
    const vfloat8 ab7  = vfloat8::loadu(&ptr[index[7]]->org);
    const vfloat8 ab8  = vfloat8::loadu(&ptr[index[8]]->org);
    const vfloat8 ab9  = vfloat8::loadu(&ptr[index[9]]->org);
    const vfloat8 ab10 = vfloat8::loadu(&ptr[index[10]]->org);
    const vfloat8 ab11 = vfloat8::loadu(&ptr[index[11]]->org);
    const vfloat8 ab12 = vfloat8::loadu(&ptr[index[12]]->org);
    const vfloat8 ab13 = vfloat8::loadu(&ptr[index[13]]->org);
    const vfloat8 ab14 = vfloat8::loadu(&ptr[index[14]]->org);
    const vfloat8 ab15 = vfloat8::loadu(&ptr[index[15]]->org);

    transpose(ab0,ab1,ab2,ab3,ab4,ab5,ab6,ab7,ab8,ab9,ab10,ab11,ab12,ab13,ab14,ab15,
              ray.org.x, ray.org.y, ray.org.z, ray.tnear(), ray.dir.x, ray.dir.y, ray.dir.z, ray.tfar());

    /* load and transpose: time, mask, id, flags */
    const vfloat4 c0  = vfloat4::loadu(&ptr[index[0]]->time);
    const vfloat4 c1  = vfloat4::loadu(&ptr[index[1]]->time);
    const vfloat4 c2  = vfloat4::loadu(&ptr[index[2]]->time);
    const vfloat4 c3  = vfloat4::loadu(&ptr[index[3]]->time);
    const vfloat4 c4  = vfloat4::loadu(&ptr[index[4]]->time);
    const vfloat4 c5  = vfloat4::loadu(&ptr[index[5]]->time);
    const vfloat4 c6  = vfloat4::loadu(&ptr[index[6]]->time);
    const vfloat4 c7  = vfloat4::loadu(&ptr[index[7]]->time);
    const vfloat4 c8  = vfloat4::loadu(&ptr[index[8]]->time);
    const vfloat4 c9  = vfloat4::loadu(&ptr[index[9]]->time);
    const vfloat4 c10 = vfloat4::loadu(&ptr[index[10]]->time);
    const vfloat4 c11 = vfloat4::loadu(&ptr[index[11]]->time);
    const vfloat4 c12 = vfloat4::loadu(&ptr[index[12]]->time);
    const vfloat4 c13 = vfloat4::loadu(&ptr[index[13]]->time);
    const vfloat4 c14 = vfloat4::loadu(&ptr[index[14]]->time);
    const vfloat4 c15 = vfloat4::loadu(&ptr[index[15]]->time);

    vfloat16 maskf, idf, flagsf;
    transpose(c0,c1,c2,c3,c4,c5,c6,c7,c8,c9,c10,c11,c12,c13,c14,c15,
              ray.time, maskf, idf, flagsf);

    ray.mask  = asInt(maskf);
    ray.id    = asInt(idf);
    ray.flags = asInt(flagsf);

    ray.geomID = RTC_INVALID_GEOMETRY_ID;
    return ray;
  }
#endif
}
