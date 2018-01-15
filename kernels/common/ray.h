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

#pragma once

#include "default.h"

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
                       const vfloat<K>& time = zero, const vint<K>& mask = -1)
      : org(org), dir(dir), tnear(tnear), tfar(tfar), time(time), mask(mask), geomID(-1), primID(-1), instID(-1) {}

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
      const vbool<K> vn = abs(tnear) <= vfloat<K>(inf);
      const vbool<K> vf = abs(tfar) <= vfloat<K>(inf);
      return vx & vy & vz & vn & vf;
    }

    /* Calculates if the hit is valid */
    __forceinline void verifyHit(const vbool<K>& valid0) const
    {
      vbool<K> valid = valid0 & geomID != vint<K>(RTC_INVALID_GEOMETRY_ID);
      const vbool<K> vt = (abs(tfar) <= vfloat<K>(FLT_LARGE));
      const vbool<K> vu = (abs(u) <= vfloat<K>(FLT_LARGE));
      const vbool<K> vv = (abs(u) <= vfloat<K>(FLT_LARGE));
      const vbool<K> vnx = abs(Ng.x) <= vfloat<K>(FLT_LARGE);
      const vbool<K> vny = abs(Ng.y) <= vfloat<K>(FLT_LARGE);
      const vbool<K> vnz = abs(Ng.z) <= vfloat<K>(FLT_LARGE);
      if (any(valid & !vt)) throw_RTCError(RTC_UNKNOWN_ERROR,"invalid t");
      if (any(valid & !vu)) throw_RTCError(RTC_UNKNOWN_ERROR,"invalid u");
      if (any(valid & !vv)) throw_RTCError(RTC_UNKNOWN_ERROR,"invalid v");
      if (any(valid & !vnx)) throw_RTCError(RTC_UNKNOWN_ERROR,"invalid Ng.x");
      if (any(valid & !vny)) throw_RTCError(RTC_UNKNOWN_ERROR,"invalid Ng.y");
      if (any(valid & !vnz)) throw_RTCError(RTC_UNKNOWN_ERROR,"invalid Ng.z");
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
      tfar[rayIndex] = new_t[i];
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
    Vec3vf<K> org;   // ray origin
    Vec3vf<K> dir;   // ray direction
    vfloat<K> tnear; // start of ray segment
    vfloat<K> tfar;  // end of ray segment
    vfloat<K> time;  // time of this ray for motion blur.
    vint<K> mask;    // used to mask out objects during traversal

    /* Hit data */
    Vec3vf<K> Ng;    // geometry normal
    vfloat<K> u;     // barycentric u coordinate of hit
    vfloat<K> v;     // barycentric v coordinate of hit
    vint<K> geomID;  // geometry ID
    vint<K> primID;  // primitive ID
    vint<K> instID;  // instance ID
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
    vfloat16::storeu_compact_single(m_mask, &tfar[rayIndex], new_t);
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
    __forceinline RayK(const Vec3fa& org, const Vec3fa& dir, float tnear = zero, float tfar = inf, float time = zero, int mask = -1)
      : org(org), dir(dir), tnear(tnear), tfar(tfar), time(time), mask(mask), geomID(-1), primID(-1), instID(-1) {}

    /* Tests if we hit something */
    __forceinline operator bool() const { return geomID != RTC_INVALID_GEOMETRY_ID; }

    /* Calculates if this is a valid ray that does not cause issues during traversal */
    __forceinline bool valid() const {
      return all(le_mask(abs(org), Vec3fa(FLT_LARGE)) & le_mask(abs(dir), Vec3fa(FLT_LARGE))) && fabs(tnear) <= float(inf) && fabs(tfar) <= float(inf);
    }

    /* Calculates if the hit is valid */
    __forceinline void verifyHit() const
    {
      if (geomID == RTC_INVALID_GEOMETRY_ID) return;
      const bool vt = (abs(tfar) <= FLT_LARGE);
      const bool vu = (abs(u) <= FLT_LARGE);
      const bool vv = (abs(u) <= FLT_LARGE);
      const bool vnx = abs(Ng.x) <= FLT_LARGE;
      const bool vny = abs(Ng.y) <= FLT_LARGE;
      const bool vnz = abs(Ng.z) <= FLT_LARGE;
      if (!vt) throw_RTCError(RTC_UNKNOWN_ERROR, "invalid t");
      if (!vu) throw_RTCError(RTC_UNKNOWN_ERROR, "invalid u");
      if (!vv) throw_RTCError(RTC_UNKNOWN_ERROR, "invalid v");
      if (!vnx) throw_RTCError(RTC_UNKNOWN_ERROR, "invalid Ng.x");
      if (!vny) throw_RTCError(RTC_UNKNOWN_ERROR, "invalid Ng.y");
      if (!vnz) throw_RTCError(RTC_UNKNOWN_ERROR, "invalid Ng.z");
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
    unsigned geomID;  // geometry ID
    unsigned primID;  // primitive ID
    unsigned instID;  // instance ID

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

      vfloat16::storeu_compact_single(m_mask, &tfar, new_t);
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
      vint16::storeu_compact_single(m_mask, &geomID, new_geomID);
      vint16::storeu_compact_single(m_mask, &primID, new_primID);
      vfloat16::storeu_compact_single(m_mask, &tfar, new_t);
      vfloat16::storeu_compact_single(m_mask, &u, new_u);
      vfloat16::storeu_compact_single(m_mask, &v, new_v);
      vfloat16::storeu_compact_single(m_mask, &Ng.x, new_gnormalx);
      vfloat16::storeu_compact_single(m_mask, &Ng.y, new_gnormaly);
      vfloat16::storeu_compact_single(m_mask, &Ng.z, new_gnormalz);
    }

#endif
  };

  /* Converts ray packet to single rays */
  template<int K>
  __forceinline void RayK<K>::get(RayK<1>* ray) const
  {
    for (size_t i = 0; i < K; i++) // FIXME: use SIMD transpose
    {
      ray[i].org.x = org.x[i]; ray[i].org.y = org.y[i]; ray[i].org.z = org.z[i];
      ray[i].dir.x = dir.x[i]; ray[i].dir.y = dir.y[i]; ray[i].dir.z = dir.z[i];
      ray[i].tnear = tnear[i]; ray[i].tfar  = tfar [i]; ray[i].time  = time[i]; ray[i].mask = mask[i];
      ray[i].Ng.x = Ng.x[i]; ray[i].Ng.y = Ng.y[i]; ray[i].Ng.z = Ng.z[i];
      ray[i].u = u[i]; ray[i].v = v[i];
      ray[i].geomID = geomID[i]; ray[i].primID = primID[i]; ray[i].instID = instID[i];
    }
  }

  /* Extracts a single ray out of a ray packet*/
  template<int K>
  __forceinline void RayK<K>::get(size_t i, RayK<1>& ray) const
  {
    ray.org.x = org.x[i]; ray.org.y = org.y[i]; ray.org.z = org.z[i];
    ray.dir.x = dir.x[i]; ray.dir.y = dir.y[i]; ray.dir.z = dir.z[i];
    ray.tnear = tnear[i]; ray.tfar  = tfar [i]; ray.time  = time[i]; ray.mask = mask[i];
    ray.Ng.x = Ng.x[i]; ray.Ng.y = Ng.y[i]; ray.Ng.z = Ng.z[i];
    ray.u = u[i]; ray.v = v[i];
    ray.geomID = geomID[i]; ray.primID = primID[i]; ray.instID = instID[i];
  }

  /* Converts single rays to ray packet */
  template<int K>
  __forceinline void RayK<K>::set(const RayK<1>* ray)
  {
    for (size_t i = 0; i < K; i++)
    {
      org.x[i] = ray[i].org.x; org.y[i] = ray[i].org.y; org.z[i] = ray[i].org.z;
      dir.x[i] = ray[i].dir.x; dir.y[i] = ray[i].dir.y; dir.z[i] = ray[i].dir.z;
      tnear[i] = ray[i].tnear; tfar [i] = ray[i].tfar;  time[i] = ray[i].time; mask[i] = ray[i].mask;
      Ng.x[i] = ray[i].Ng.x; Ng.y[i] = ray[i].Ng.y; Ng.z[i] = ray[i].Ng.z;
      u[i] = ray[i].u; v[i] = ray[i].v;
      geomID[i] = ray[i].geomID; primID[i] = ray[i].primID; instID[i] = ray[i].instID;
    }
  }

  /* inserts a single ray into a ray packet element */
  template<int K>
  __forceinline void RayK<K>::set(size_t i, const RayK<1>& ray)
  {
    org.x[i] = ray.org.x; org.y[i] = ray.org.y; org.z[i] = ray.org.z;
    dir.x[i] = ray.dir.x; dir.y[i] = ray.dir.y; dir.z[i] = ray.dir.z;
    tnear[i] = ray.tnear; tfar [i] = ray.tfar;  time[i] = ray.time; mask[i] = ray.mask;
    Ng.x[i] = ray.Ng.x; Ng.y[i] = ray.Ng.y; Ng.z[i] = ray.Ng.z;
    u[i] = ray.u; v[i] = ray.v;
    geomID[i] = ray.geomID; primID[i] = ray.primID; instID[i] = ray.instID;
  }

  /* copies a ray packet element into another element*/
  template<int K>
  __forceinline void RayK<K>::copy(size_t dest, size_t source)
  {
    org.x[dest] = org.x[source]; org.y[dest] = org.y[source]; org.z[dest] = org.z[source];
    dir.x[dest] = dir.x[source]; dir.y[dest] = dir.y[source]; dir.z[dest] = dir.z[source];
    tnear[dest] = tnear[source]; tfar [dest] = tfar[source];  time[dest] = time[source]; mask[dest] = mask[source];
    Ng.x[dest] = Ng.x[source]; Ng.y[dest] = Ng.y[source]; Ng.z[dest] = Ng.z[source];
    u[dest] = u[source]; v[dest] = v[source];
    geomID[dest] = geomID[source]; primID[dest] = primID[source]; instID[dest] = instID[source];
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
                << "  near = " << ray.tnear << std::endl
                << "  far = " << ray.tfar << std::endl
                << "  time = " << ray.time << std::endl
                << "  mask = " << ray.mask << std::endl
                << "  instID = " << ray.instID << std::endl
                << "  geomID = " << ray.geomID << std::endl
                << "  primID = " << ray.primID <<  std::endl
                << "  u = " << ray.u <<  std::endl
                << "  v = " << ray.v << std::endl
                << "  Ng = " << ray.Ng
                << "}";
  }


  struct RayStreamSOA
  {
    __forceinline RayStreamSOA(void* rays, size_t N)
      : ptr((char*)rays), N(N) {}

    /* ray data access functions */
    __forceinline float* orgx(size_t offset = 0) { return (float*)&ptr[0*4*N+offset]; }  //!< x coordinate of ray origin
    __forceinline float* orgy(size_t offset = 0) { return (float*)&ptr[1*4*N+offset]; }  //!< y coordinate of ray origin
    __forceinline float* orgz(size_t offset = 0) { return (float*)&ptr[2*4*N+offset]; };  //!< z coordinate of ray origin

    __forceinline float* dirx(size_t offset = 0) { return (float*)&ptr[3*4*N+offset]; };  //!< x coordinate of ray direction
    __forceinline float* diry(size_t offset = 0) { return (float*)&ptr[4*4*N+offset]; };  //!< y coordinate of ray direction
    __forceinline float* dirz(size_t offset = 0) { return (float*)&ptr[5*4*N+offset]; };  //!< z coordinate of ray direction

    __forceinline float* tnear(size_t offset = 0) { return (float*)&ptr[6*4*N+offset]; }; //!< Start of ray segment
    __forceinline float* tfar (size_t offset = 0) { return (float*)&ptr[7*4*N+offset]; }; //!< End of ray segment (set to hit distance)

    __forceinline float* time(size_t offset = 0) { return (float*)&ptr[8*4*N+offset]; };  //!< Time of this ray for motion blur
    __forceinline int*   mask(size_t offset = 0) { return (int*)  &ptr[9*4*N+offset]; };  //!< Used to mask out objects during traversal (optional)

    /* hit data access functions */
    __forceinline float* Ngx(size_t offset = 0) { return (float*)&ptr[10*4*N+offset]; };   //!< x coordinate of geometry normal
    __forceinline float* Ngy(size_t offset = 0) { return (float*)&ptr[11*4*N+offset]; };   //!< y coordinate of geometry normal
    __forceinline float* Ngz(size_t offset = 0) { return (float*)&ptr[12*4*N+offset]; };   //!< z coordinate of geometry normal

    __forceinline float* u(size_t offset = 0) { return (float*)&ptr[13*4*N+offset]; };     //!< Barycentric u coordinate of hit
    __forceinline float* v(size_t offset = 0) { return (float*)&ptr[14*4*N+offset]; };     //!< Barycentric v coordinate of hit

    __forceinline int* geomID(size_t offset = 0) { return (int*)&ptr[15*4*N+offset]; };  //!< geometry ID
    __forceinline int* primID(size_t offset = 0) { return (int*)&ptr[16*4*N+offset]; };  //!< primitive ID
    __forceinline int* instID(size_t offset = 0) { return (int*)&ptr[17*4*N+offset]; };  //!< instance ID


    __forceinline void setRayByIndex(size_t index, const Ray& ray, int* valid)
    {
      const size_t offset = 4*index;
      valid[index] = -1;
      orgx(offset)[0] = ray.org.x;
      orgy(offset)[0] = ray.org.y;
      orgz(offset)[0] = ray.org.z;
      dirx(offset)[0] = ray.dir.x;
      diry(offset)[0] = ray.dir.y;
      dirz(offset)[0] = ray.dir.z;
      tnear(offset)[0] = ray.tnear;
      tfar(offset)[0] = ray.tfar;
      time(offset)[0] = ray.time;
      mask(offset)[0] = ray.mask;
      instID(offset)[0] = ray.instID;
      geomID(offset)[0] = RTC_INVALID_GEOMETRY_ID;
    }

    __forceinline void getHitByIndex(size_t index, Ray& ray)
    {
      const size_t offset = 4*index;
      const unsigned int geometryID = geomID(offset)[0];
      if (geometryID != RTC_INVALID_GEOMETRY_ID)
      {
        ray.tfar = tfar(offset)[0];
        ray.u = u(offset)[0];
        ray.v = v(offset)[0];
        ray.Ng.x = Ngx(offset)[0];
        ray.Ng.y = Ngy(offset)[0];
        ray.Ng.z = Ngz(offset)[0];
        ray.instID = instID(offset)[0];
        ray.geomID = geometryID;
        ray.primID = primID(offset)[0];
      }
    }

    __forceinline void getOcclusionByIndex(size_t index, Ray& ray)
    {
      ray.geomID = geomID(4*index)[0];
    }

    __forceinline Ray getRayByOffset(size_t offset)
    {
      Ray ray;
      ray.org.x = orgx(offset)[0];
      ray.org.y = orgy(offset)[0];
      ray.org.z = orgz(offset)[0];
      ray.dir.x = dirx(offset)[0];
      ray.dir.y = diry(offset)[0];
      ray.dir.z = dirz(offset)[0];
      ray.tnear = tnear(offset)[0];
      ray.tfar  = tfar(offset)[0];
      ray.time  = time(offset)[0];
      ray.mask  = mask(offset)[0];
      ray.instID = instID(offset)[0];
      ray.geomID = RTC_INVALID_GEOMETRY_ID;
      return ray;
    }

    template<int K>
    __forceinline RayK<K> getRayByOffset(size_t offset)
    {
      RayK<K> ray;
      ray.org.x = vfloat<K>::loadu(orgx(offset));
      ray.org.y = vfloat<K>::loadu(orgy(offset));
      ray.org.z = vfloat<K>::loadu(orgz(offset));
      ray.dir.x = vfloat<K>::loadu(dirx(offset));
      ray.dir.y = vfloat<K>::loadu(diry(offset));
      ray.dir.z = vfloat<K>::loadu(dirz(offset));
      ray.tnear = vfloat<K>::loadu(tnear(offset));
      ray.tfar  = vfloat<K>::loadu(tfar(offset));
      ray.time  = vfloat<K>::loadu(time(offset));
      ray.mask  = vint<K>::loadu(mask(offset));
      ray.instID = vint<K>::loadu(instID(offset));
      ray.geomID = RTC_INVALID_GEOMETRY_ID;
      return ray;
    }

    template<int K>
    __forceinline RayK<K> getRayByOffset(const vbool<K>& valid, size_t offset)
    {
      RayK<K> ray;
      ray.org.x = vfloat<K>::loadu(valid, orgx(offset));
      ray.org.y = vfloat<K>::loadu(valid, orgy(offset));
      ray.org.z = vfloat<K>::loadu(valid, orgz(offset));
      ray.dir.x = vfloat<K>::loadu(valid, dirx(offset));
      ray.dir.y = vfloat<K>::loadu(valid, diry(offset));
      ray.dir.z = vfloat<K>::loadu(valid, dirz(offset));
      ray.tnear = vfloat<K>::loadu(valid, tnear(offset));
      ray.tfar  = vfloat<K>::loadu(valid, tfar(offset));
      ray.time  = vfloat<K>::loadu(valid, time(offset));
      ray.mask  = vint<K>::loadu(valid, mask(offset));

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
      ray.org.x[index_dest] = orgx(offset)[0];
      ray.org.y[index_dest] = orgy(offset)[0];
      ray.org.z[index_dest] = orgz(offset)[0];
      ray.dir.x[index_dest] = dirx(offset)[0];
      ray.dir.y[index_dest] = diry(offset)[0];
      ray.dir.z[index_dest] = dirz(offset)[0];
      ray.tnear[index_dest] = tnear(offset)[0];
      ray.tfar[index_dest]  = tfar(offset)[0];
      ray.time[index_dest]  = time(offset)[0];
      ray.mask[index_dest]  = mask(offset)[0];
      ray.instID[index_dest] = instID(offset)[0];
      ray.geomID[index_dest] = RTC_INVALID_GEOMETRY_ID;
    }

    template<int K>
    __forceinline void setHitByIndex(size_t index, const RayK<K>& ray, size_t index_source, bool intersect = true)
    {
      if (ray.geomID[index_source] != RTC_INVALID_GEOMETRY_ID)
      {
        const size_t offset = index * sizeof(float);
        geomID(offset)[0] = ray.geomID[index_source];
        if (intersect)
        {
          tfar(offset)[0] = ray.tfar[index_source];
          u(offset)[0] = ray.u[index_source];
          v(offset)[0] = ray.v[index_source];
          primID(offset)[0] = ray.primID[index_source];
          Ngx(offset)[0] = ray.Ng.x[index_source];
          Ngy(offset)[0] = ray.Ng.y[index_source];
          Ngz(offset)[0] = ray.Ng.z[index_source];
          instID(offset)[0] = ray.instID[index_source];
        }
      }
    }

    __forceinline void setHitByOffset(size_t offset, const Ray& ray, bool intersect = true)
    {
      if (ray.geomID != RTC_INVALID_GEOMETRY_ID)
      {
        geomID(offset)[0] = ray.geomID;
        if (intersect)
        {
          tfar(offset)[0] = ray.tfar;
          u(offset)[0] = ray.u;
          v(offset)[0] = ray.v;
          primID(offset)[0] = ray.primID;
          Ngx(offset)[0] = ray.Ng.x;
          Ngy(offset)[0] = ray.Ng.y;
          Ngz(offset)[0] = ray.Ng.z;
          instID(offset)[0] = ray.instID;
        }
      }
    }

    template<int K>
    __forceinline void setHitByOffset(const vbool<K>& valid_i, size_t offset, const RayK<K>& ray, bool intersect = true)
    {
      vbool<K> valid = valid_i;
      valid &= ray.geomID != RTC_INVALID_GEOMETRY_ID;

      if (likely(any(valid)))
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
              if (intersect)
              {
                primID(offset)[k] = ray.primID[k];
                instID(offset)[k] = ray.instID[k];
              }
            }
          }
        }
        else
#endif
        {
          vint<K>::storeu(valid, geomID(offset), ray.geomID);
          if (intersect)
          {
            vint<K>::storeu(valid, primID(offset), ray.primID);
            vint<K>::storeu(valid, instID(offset), ray.instID);
          }
        }
        
        if (intersect)
        {
          vfloat<K>::storeu(valid, tfar(offset), ray.tfar);
          vfloat<K>::storeu(valid, Ngx(offset), ray.Ng.x);
          vfloat<K>::storeu(valid, Ngy(offset), ray.Ng.y);
          vfloat<K>::storeu(valid, Ngz(offset), ray.Ng.z);
          vfloat<K>::storeu(valid, u(offset), ray.u);
          vfloat<K>::storeu(valid, v(offset), ray.v);
        }
      }
    }

    __forceinline size_t getOctantByOffset(size_t offset)
    {
      const float dx = dirx(offset)[0];
      const float dy = diry(offset)[0];
      const float dz = dirz(offset)[0];
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
      ray.org.x  = vfloat<K>::template gather<1>(valid, orgx(), offset);
      ray.org.y  = vfloat<K>::template gather<1>(valid, orgy(), offset);
      ray.org.z  = vfloat<K>::template gather<1>(valid, orgz(), offset);
      ray.dir.x  = vfloat<K>::template gather<1>(valid, dirx(), offset);
      ray.dir.y  = vfloat<K>::template gather<1>(valid, diry(), offset);
      ray.dir.z  = vfloat<K>::template gather<1>(valid, dirz(), offset);
      ray.tnear  = vfloat<K>::template gather<1>(valid, tnear(), offset);
      ray.tfar   = vfloat<K>::template gather<1>(valid, tfar(), offset);
      ray.time   = vfloat<K>::template gather<1>(valid, time(), offset);
      ray.mask   = vint<K>::template gather<1>(valid, mask(), offset);
      ray.instID = vint<K>::template gather<1>(valid, instID(), offset);
#else
      ray.org = zero;
      ray.dir = zero;
      ray.tnear = zero;
      ray.tfar = zero;
      ray.time = zero;
      ray.mask = zero;
      ray.instID = zero;

      for (size_t k = 0; k < K; k++)
      {
        if (likely(valid[k]))
        {
          const size_t ofs = offset[k];

          ray.org.x[k]  = *orgx(ofs);
          ray.org.y[k]  = *orgy(ofs);
          ray.org.z[k]  = *orgz(ofs);
          ray.dir.x[k]  = *dirx(ofs);
          ray.dir.y[k]  = *diry(ofs);
          ray.dir.z[k]  = *dirz(ofs);
          ray.tnear[k]  = *tnear(ofs);
          ray.tfar[k]   = *tfar(ofs);
          ray.time[k]   = *time(ofs);
          ray.mask[k]   = *mask(ofs);
          ray.instID[k] = *instID(ofs);
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
      valid &= ray.geomID != RTC_INVALID_GEOMETRY_ID;

      if (likely(any(valid)))
      {
#if defined(__AVX512F__)
        vint<K>::template scatter<1>(valid, geomID(), offset, ray.geomID);
        if (intersect)
        {
          vfloat<K>::template scatter<1>(valid, tfar(), offset, ray.tfar);
          vfloat<K>::template scatter<1>(valid, u(), offset, ray.u);
          vfloat<K>::template scatter<1>(valid, v(), offset, ray.v);
          vint<K>::template scatter<1>(valid, primID(), offset, ray.primID);
          vfloat<K>::template scatter<1>(valid, Ngx(), offset, ray.Ng.x);
          vfloat<K>::template scatter<1>(valid, Ngy(), offset, ray.Ng.y);
          vfloat<K>::template scatter<1>(valid, Ngz(), offset, ray.Ng.z);
          vint<K>::template scatter<1>(valid, instID(), offset, ray.instID);
        }
#else
        size_t valid_bits = movemask(valid);
        while (valid_bits != 0)
        {
          const size_t k = __bscf(valid_bits);
          const size_t ofs = offset[k];

          *geomID(ofs) = ray.geomID[k];
          if (intersect)
          {
            *tfar(ofs) = ray.tfar[k];
            *u(ofs) = ray.u[k];
            *v(ofs) = ray.v[k];
            *primID(ofs) = ray.primID[k];
            *Ngx(ofs) = ray.Ng.x[k];
            *Ngy(ofs) = ray.Ng.y[k];
            *Ngz(ofs) = ray.Ng.z[k];
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
      orgx   = (float*)&t.org.x;
      orgy   = (float*)&t.org.y;
      orgz   = (float*)&t.org.z;
      dirx   = (float*)&t.dir.x;
      diry   = (float*)&t.dir.y;
      dirz   = (float*)&t.dir.z;
      tnear  = (float*)&t.tnear;
      tfar   = (float*)&t.tfar;
      time   = (float*)&t.time;
      mask   = (unsigned*)&t.mask;
      Ngx    = (float*)&t.Ng.x;
      Ngy    = (float*)&t.Ng.y;
      Ngz    = (float*)&t.Ng.z;
      u      = (float*)&t.u;
      v      = (float*)&t.v;
      geomID = (unsigned*)&t.geomID;
      primID = (unsigned*)&t.primID;
      instID = (unsigned*)&t.instID;
    }

    __forceinline Ray getRayByOffset(size_t offset)
    {
      Ray ray;
      ray.org.x = *(float* __restrict__)((char*)orgx + offset);
      ray.org.y = *(float* __restrict__)((char*)orgy + offset);
      ray.org.z = *(float* __restrict__)((char*)orgz + offset);
      ray.dir.x = *(float* __restrict__)((char*)dirx + offset);
      ray.dir.y = *(float* __restrict__)((char*)diry + offset);
      ray.dir.z = *(float* __restrict__)((char*)dirz + offset);
      ray.tfar  = *(float* __restrict__)((char*)tfar + offset);
      ray.tnear = tnear ? *(float* __restrict__)((char*)tnear + offset) : 0.0f;
      ray.time  = time ? *(float* __restrict__)((char*)time + offset) : 0.0f;
      ray.mask  = mask ? *(unsigned* __restrict__)((char*)mask + offset) : -1;
      ray.instID = instID ? *(unsigned* __restrict__)((char*)instID + offset) : -1;
      ray.geomID = RTC_INVALID_GEOMETRY_ID;
      return ray;
    }

    template<int K>
    __forceinline RayK<K> getRayByOffset(const vbool<K>& valid, size_t offset)
    {
      RayK<K> ray;
      ray.org.x = vfloat<K>::loadu(valid, (float* __restrict__)((char*)orgx + offset));
      ray.org.y = vfloat<K>::loadu(valid, (float* __restrict__)((char*)orgy + offset));
      ray.org.z = vfloat<K>::loadu(valid, (float* __restrict__)((char*)orgz + offset));
      ray.dir.x = vfloat<K>::loadu(valid, (float* __restrict__)((char*)dirx + offset));
      ray.dir.y = vfloat<K>::loadu(valid, (float* __restrict__)((char*)diry + offset));
      ray.dir.z = vfloat<K>::loadu(valid, (float* __restrict__)((char*)dirz + offset));
      ray.tfar  = vfloat<K>::loadu(valid, (float* __restrict__)((char*)tfar + offset));
      ray.tnear = tnear ? vfloat<K>::loadu(valid, (float* __restrict__)((char*)tnear + offset)) : 0.0f;
      ray.time  = time ? vfloat<K>::loadu(valid, (float* __restrict__)((char*)time + offset)) : 0.0f;
      ray.mask  = mask ? vint<K>::loadu(valid, (const void * __restrict__)((char*)mask + offset)) : -1;
      ray.instID = instID ? vint<K>::loadu(valid, (const void * __restrict__)((char*)instID + offset)) : -1;
      ray.geomID = RTC_INVALID_GEOMETRY_ID;
      return ray;
    }

    template<int K>
    __forceinline Vec3vf<K> getDirByOffset(const vbool<K>& valid, size_t offset)
    {
      Vec3vf<K> dir;
      dir.x = vfloat<K>::loadu(valid, (float* __restrict__)((char*)dirx + offset));
      dir.y = vfloat<K>::loadu(valid, (float* __restrict__)((char*)diry + offset));
      dir.z = vfloat<K>::loadu(valid, (float* __restrict__)((char*)dirz + offset));
      return dir;
    }

    __forceinline void setHitByOffset(size_t offset, const Ray& ray, bool intersect = true)
    {
      if (ray.geomID != RTC_INVALID_GEOMETRY_ID)
      {
        *(unsigned* __restrict__)((char*)geomID + offset) = ray.geomID;
        if (intersect)
        {
          *(float* __restrict__)((char*)tfar + offset) = ray.tfar;
          *(float* __restrict__)((char*)u + offset) = ray.u;
          *(float* __restrict__)((char*)v + offset) = ray.v;
          *(unsigned* __restrict__)((char*)primID + offset) = ray.primID;
          if (likely(Ngx)) *(float* __restrict__)((char*)Ngx + offset) = ray.Ng.x;
          if (likely(Ngy)) *(float* __restrict__)((char*)Ngy + offset) = ray.Ng.y;
          if (likely(Ngz)) *(float* __restrict__)((char*)Ngz + offset) = ray.Ng.z;
          if (likely(instID)) *(unsigned* __restrict__)((char*)instID + offset) = ray.instID;
        }
      }
    }

    template<int K>
    __forceinline void setHitByOffset(const vbool<K>& valid_i, size_t offset, const RayK<K>& ray, bool intersect = true)
    {
      vbool<K> valid = valid_i;
      valid &= ray.geomID != RTC_INVALID_GEOMETRY_ID;

      if (likely(any(valid)))
      {
        vint<K>::storeu(valid, (int* __restrict__)((char*)geomID + offset), ray.geomID);
        if (intersect)
        {
          vfloat<K>::storeu(valid, (float* __restrict__)((char*)tfar + offset), ray.tfar);
          vfloat<K>::storeu(valid, (float* __restrict__)((char*)u + offset), ray.u);
          vfloat<K>::storeu(valid, (float* __restrict__)((char*)v + offset), ray.v);
          vint<K>::storeu(valid, (int* __restrict__)((char*)primID + offset), ray.primID);
          if (likely(Ngx)) vfloat<K>::storeu(valid, (float* __restrict__)((char*)Ngx + offset), ray.Ng.x);
          if (likely(Ngy)) vfloat<K>::storeu(valid, (float* __restrict__)((char*)Ngy + offset), ray.Ng.y);
          if (likely(Ngz)) vfloat<K>::storeu(valid, (float* __restrict__)((char*)Ngz + offset), ray.Ng.z);
          if (likely(instID)) vint<K>::storeu(valid, (int* __restrict__)((char*)instID + offset), ray.instID);
        }
      }
    }

    __forceinline size_t getOctantByOffset(size_t offset)
    {
      const float dx = *(float* __restrict__)((char*)dirx + offset);
      const float dy = *(float* __restrict__)((char*)diry + offset);
      const float dz = *(float* __restrict__)((char*)dirz + offset);
      const size_t octantID = (dx < 0.0f ? 1 : 0) + (dy < 0.0f ? 2 : 0) + (dz < 0.0f ? 4 : 0);
      return octantID;
    }

    __forceinline bool isValidByOffset(size_t offset)
    {
      const float nnear = tnear ? *(float* __restrict__)((char*)tnear + offset) : 0.0f;
      const float ffar  = *(float* __restrict__)((char*)tfar  + offset);
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
      ray.org.x  = vfloat<K>::template gather<1>(valid, orgx, offset);
      ray.org.y  = vfloat<K>::template gather<1>(valid, orgy, offset);
      ray.org.z  = vfloat<K>::template gather<1>(valid, orgz, offset);
      ray.dir.x  = vfloat<K>::template gather<1>(valid, dirx, offset);
      ray.dir.y  = vfloat<K>::template gather<1>(valid, diry, offset);
      ray.dir.z  = vfloat<K>::template gather<1>(valid, dirz, offset);
      ray.tfar   = vfloat<K>::template gather<1>(valid, tfar, offset);
      ray.tnear  = tnear ? vfloat<K>::template gather<1>(valid, tnear, offset) : vfloat<K>(zero);
      ray.time   = time ? vfloat<K>::template gather<1>(valid, time, offset) : vfloat<K>(zero);
      ray.mask   = mask ? vint<K>::template gather<1>(valid, (int*)mask, offset) : vint<K>(-1);
      ray.instID = instID ? vint<K>::template gather<1>(valid, (int*)instID, offset) : vint<K>(-1);
#else
      ray.org = zero;
      ray.dir = zero;
      ray.tnear = zero;
      ray.tfar = zero;
      ray.time = zero;
      ray.mask = zero;
      ray.instID = zero;

      for (size_t k = 0; k < K; k++)
      {
        if (likely(valid[k]))
        {
          const size_t ofs = offset[k];

          ray.org.x[k]  = *(float* __restrict__)((char*)orgx + ofs);
          ray.org.y[k]  = *(float* __restrict__)((char*)orgy + ofs);
          ray.org.z[k]  = *(float* __restrict__)((char*)orgz + ofs);
          ray.dir.x[k]  = *(float* __restrict__)((char*)dirx + ofs);
          ray.dir.y[k]  = *(float* __restrict__)((char*)diry + ofs);
          ray.dir.z[k]  = *(float* __restrict__)((char*)dirz + ofs);
          ray.tfar[k]   = *(float* __restrict__)((char*)tfar + ofs);
          ray.tnear[k]  = tnear ? *(float* __restrict__)((char*)tnear + ofs) : 0.0f;
          ray.time[k]   = time ? *(float* __restrict__)((char*)time + ofs) : 0.0f;
          ray.mask[k]   = mask ? *(unsigned* __restrict__)((char*)mask + ofs) : -1;
          ray.instID[k] = instID ? *(unsigned* __restrict__)((char*)instID + ofs) : -1;
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
      valid &= ray.geomID != RTC_INVALID_GEOMETRY_ID;

      if (likely(any(valid)))
      {
#if defined(__AVX512F__)
        vint<K>::template scatter<1>(valid, (int*)geomID, offset, ray.geomID);
        if (intersect)
        {
          vfloat<K>::template scatter<1>(valid, tfar, offset, ray.tfar);
          vfloat<K>::template scatter<1>(valid, u, offset, ray.u);
          vfloat<K>::template scatter<1>(valid, v, offset, ray.v);
          vint<K>::template scatter<1>(valid, (int*)primID, offset, ray.primID);
          if (likely(Ngx)) vfloat<K>::template scatter<1>(valid, Ngx, offset, ray.Ng.x);
          if (likely(Ngy)) vfloat<K>::template scatter<1>(valid, Ngy, offset, ray.Ng.y);
          if (likely(Ngz)) vfloat<K>::template scatter<1>(valid, Ngz, offset, ray.Ng.z);
          if (likely(instID)) vint<K>::template scatter<1>(valid, (int*)instID, offset, ray.instID);
        }
#else
        size_t valid_bits = movemask(valid);
        while (valid_bits != 0)
        {
          const size_t k = __bscf(valid_bits);
          const size_t ofs = offset[k];

          *(unsigned* __restrict__)((char*)geomID + ofs) = ray.geomID[k];
          if (intersect)
          {
            *(float* __restrict__)((char*)tfar + ofs) = ray.tfar[k];
            *(float* __restrict__)((char*)u + ofs) = ray.u[k];
            *(float* __restrict__)((char*)v + ofs) = ray.v[k];
            *(unsigned* __restrict__)((char*)primID + ofs) = ray.primID[k];
            if (likely(Ngx)) *(float* __restrict__)((char*)Ngx + ofs) = ray.Ng.x[k];
            if (likely(Ngy)) *(float* __restrict__)((char*)Ngy + ofs) = ray.Ng.y[k];
            if (likely(Ngz)) *(float* __restrict__)((char*)Ngz + ofs) = ray.Ng.z[k];
            if (likely(instID)) *(unsigned* __restrict__)((char*)instID + ofs) = ray.instID[k];
          }
        }
#endif
      }
    }

    /* ray data */
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
      valid &= ray.geomID != RTC_INVALID_GEOMETRY_ID;

      if (likely(any(valid)))
      {
#if defined(__AVX512F__)
        vint<K>::template scatter<1>(valid, (int*)&ptr->geomID, offset, ray.geomID);
        if (intersect)
        {
          vfloat<K>::template scatter<1>(valid, &ptr->tfar, offset, ray.tfar);
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

          ray_k->geomID = ray.geomID[k];
          if (intersect)
          {
            ray_k->tfar   = ray.tfar[k];
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

    transpose(a0,a1,a2,a3, ray.org.x, ray.org.y, ray.org.z);

    /* load and transpose: dir.x, dir.y, dir.z */
    const vfloat4 b0 = vfloat4::loadu(&((Ray*)((char*)ptr + offset[0]))->dir);
    const vfloat4 b1 = vfloat4::loadu(&((Ray*)((char*)ptr + offset[1]))->dir);
    const vfloat4 b2 = vfloat4::loadu(&((Ray*)((char*)ptr + offset[2]))->dir);
    const vfloat4 b3 = vfloat4::loadu(&((Ray*)((char*)ptr + offset[3]))->dir);

    transpose(b0,b1,b2,b3, ray.dir.x, ray.dir.y, ray.dir.z);

    /* load and transpose: tnear, tfar, time, mask */
    const vfloat4 c0 = vfloat4::loadu(&((Ray*)((char*)ptr + offset[0]))->tnear);
    const vfloat4 c1 = vfloat4::loadu(&((Ray*)((char*)ptr + offset[1]))->tnear);
    const vfloat4 c2 = vfloat4::loadu(&((Ray*)((char*)ptr + offset[2]))->tnear);
    const vfloat4 c3 = vfloat4::loadu(&((Ray*)((char*)ptr + offset[3]))->tnear);

    vfloat4 maskf;
    transpose(c0,c1,c2,c3, ray.tnear, ray.tfar, ray.time, maskf);
    ray.mask = asInt(maskf);

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

    /* load and transpose: org.x, org.y, org.z, align0, dir.x, dir.y, dir.z, align1 */
    const vfloat8 ab0 = vfloat8::loadu(&((Ray*)((char*)ptr + offset[0]))->org);
    const vfloat8 ab1 = vfloat8::loadu(&((Ray*)((char*)ptr + offset[1]))->org);
    const vfloat8 ab2 = vfloat8::loadu(&((Ray*)((char*)ptr + offset[2]))->org);
    const vfloat8 ab3 = vfloat8::loadu(&((Ray*)((char*)ptr + offset[3]))->org);
    const vfloat8 ab4 = vfloat8::loadu(&((Ray*)((char*)ptr + offset[4]))->org);
    const vfloat8 ab5 = vfloat8::loadu(&((Ray*)((char*)ptr + offset[5]))->org);
    const vfloat8 ab6 = vfloat8::loadu(&((Ray*)((char*)ptr + offset[6]))->org);
    const vfloat8 ab7 = vfloat8::loadu(&((Ray*)((char*)ptr + offset[7]))->org);

    vfloat8 unused0, unused1;
    transpose(ab0,ab1,ab2,ab3,ab4,ab5,ab6,ab7, ray.org.x, ray.org.y, ray.org.z, unused0, ray.dir.x, ray.dir.y, ray.dir.z, unused1);

    /* load and transpose: tnear, tfar, time, mask */
    const vfloat4 c0 = vfloat4::loadu(&((Ray*)((char*)ptr + offset[0]))->tnear);
    const vfloat4 c1 = vfloat4::loadu(&((Ray*)((char*)ptr + offset[1]))->tnear);
    const vfloat4 c2 = vfloat4::loadu(&((Ray*)((char*)ptr + offset[2]))->tnear);
    const vfloat4 c3 = vfloat4::loadu(&((Ray*)((char*)ptr + offset[3]))->tnear);
    const vfloat4 c4 = vfloat4::loadu(&((Ray*)((char*)ptr + offset[4]))->tnear);
    const vfloat4 c5 = vfloat4::loadu(&((Ray*)((char*)ptr + offset[5]))->tnear);
    const vfloat4 c6 = vfloat4::loadu(&((Ray*)((char*)ptr + offset[6]))->tnear);
    const vfloat4 c7 = vfloat4::loadu(&((Ray*)((char*)ptr + offset[7]))->tnear);

    vfloat8 maskf;
    transpose(c0,c1,c2,c3,c4,c5,c6,c7, ray.tnear, ray.tfar, ray.time, maskf);
    ray.mask = asInt(maskf);

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

    /* load and transpose: org.x, org.y, org.z, align0, dir.x, dir.y, dir.z, align1 */
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

    vfloat16 unused0, unused1;
    transpose(ab0,ab1,ab2,ab3,ab4,ab5,ab6,ab7,ab8,ab9,ab10,ab11,ab12,ab13,ab14,ab15,
              ray.org.x, ray.org.y, ray.org.z, unused0, ray.dir.x, ray.dir.y, ray.dir.z, unused1);

    /* load and transpose: tnear, tfar, time, mask */
    const vfloat4 c0  = vfloat4::loadu(&((Ray*)((char*)ptr + offset[ 0]))->tnear);
    const vfloat4 c1  = vfloat4::loadu(&((Ray*)((char*)ptr + offset[ 1]))->tnear);
    const vfloat4 c2  = vfloat4::loadu(&((Ray*)((char*)ptr + offset[ 2]))->tnear);
    const vfloat4 c3  = vfloat4::loadu(&((Ray*)((char*)ptr + offset[ 3]))->tnear);
    const vfloat4 c4  = vfloat4::loadu(&((Ray*)((char*)ptr + offset[ 4]))->tnear);
    const vfloat4 c5  = vfloat4::loadu(&((Ray*)((char*)ptr + offset[ 5]))->tnear);
    const vfloat4 c6  = vfloat4::loadu(&((Ray*)((char*)ptr + offset[ 6]))->tnear);
    const vfloat4 c7  = vfloat4::loadu(&((Ray*)((char*)ptr + offset[ 7]))->tnear);
    const vfloat4 c8  = vfloat4::loadu(&((Ray*)((char*)ptr + offset[ 8]))->tnear);
    const vfloat4 c9  = vfloat4::loadu(&((Ray*)((char*)ptr + offset[ 9]))->tnear);
    const vfloat4 c10 = vfloat4::loadu(&((Ray*)((char*)ptr + offset[10]))->tnear);
    const vfloat4 c11 = vfloat4::loadu(&((Ray*)((char*)ptr + offset[11]))->tnear);
    const vfloat4 c12 = vfloat4::loadu(&((Ray*)((char*)ptr + offset[12]))->tnear);
    const vfloat4 c13 = vfloat4::loadu(&((Ray*)((char*)ptr + offset[13]))->tnear);
    const vfloat4 c14 = vfloat4::loadu(&((Ray*)((char*)ptr + offset[14]))->tnear);
    const vfloat4 c15 = vfloat4::loadu(&((Ray*)((char*)ptr + offset[15]))->tnear);

    vfloat16 maskf;
    transpose(c0,c1,c2,c3,c4,c5,c6,c7,c8,c9,c10,c11,c12,c13,c14,c15,
              ray.tnear, ray.tfar, ray.time, maskf);
    ray.mask = asInt(maskf);

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
      valid &= ray.geomID != RTC_INVALID_GEOMETRY_ID;

      if (likely(any(valid)))
      {
        size_t valid_bits = movemask(valid);
        while (valid_bits != 0)
        {
          const size_t k = __bscf(valid_bits);
          Ray* __restrict__ ray_k = ptr[index[k]];

          ray_k->geomID = ray.geomID[k];
          if (intersect)
          {
            ray_k->tfar   = ray.tfar[k];
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

    transpose(a0,a1,a2,a3, ray.org.x, ray.org.y, ray.org.z);

    /* load and transpose: dir.x, dir.y, dir.z */
    const vfloat4 b0 = vfloat4::loadu(&ptr[index[0]]->dir);
    const vfloat4 b1 = vfloat4::loadu(&ptr[index[1]]->dir);
    const vfloat4 b2 = vfloat4::loadu(&ptr[index[2]]->dir);
    const vfloat4 b3 = vfloat4::loadu(&ptr[index[3]]->dir);

    transpose(b0,b1,b2,b3, ray.dir.x, ray.dir.y, ray.dir.z);

    /* load and transpose: tnear, tfar, time, mask */
    const vfloat4 c0 = vfloat4::loadu(&ptr[index[0]]->tnear);
    const vfloat4 c1 = vfloat4::loadu(&ptr[index[1]]->tnear);
    const vfloat4 c2 = vfloat4::loadu(&ptr[index[2]]->tnear);
    const vfloat4 c3 = vfloat4::loadu(&ptr[index[3]]->tnear);

    vfloat4 maskf;
    transpose(c0,c1,c2,c3, ray.tnear, ray.tfar, ray.time, maskf);
    ray.mask = asInt(maskf);

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

    /* load and transpose: org.x, org.y, org.z, align0, dir.x, dir.y, dir.z, align1 */
    const vfloat8 ab0 = vfloat8::loadu(&ptr[index[0]]->org);
    const vfloat8 ab1 = vfloat8::loadu(&ptr[index[1]]->org);
    const vfloat8 ab2 = vfloat8::loadu(&ptr[index[2]]->org);
    const vfloat8 ab3 = vfloat8::loadu(&ptr[index[3]]->org);
    const vfloat8 ab4 = vfloat8::loadu(&ptr[index[4]]->org);
    const vfloat8 ab5 = vfloat8::loadu(&ptr[index[5]]->org);
    const vfloat8 ab6 = vfloat8::loadu(&ptr[index[6]]->org);
    const vfloat8 ab7 = vfloat8::loadu(&ptr[index[7]]->org);

    vfloat8 unused0, unused1;
    transpose(ab0,ab1,ab2,ab3,ab4,ab5,ab6,ab7, ray.org.x, ray.org.y, ray.org.z, unused0, ray.dir.x, ray.dir.y, ray.dir.z, unused1);

    /* load and transpose: tnear, tfar, time, mask */
    const vfloat4 c0 = vfloat4::loadu(&ptr[index[0]]->tnear);
    const vfloat4 c1 = vfloat4::loadu(&ptr[index[1]]->tnear);
    const vfloat4 c2 = vfloat4::loadu(&ptr[index[2]]->tnear);
    const vfloat4 c3 = vfloat4::loadu(&ptr[index[3]]->tnear);
    const vfloat4 c4 = vfloat4::loadu(&ptr[index[4]]->tnear);
    const vfloat4 c5 = vfloat4::loadu(&ptr[index[5]]->tnear);
    const vfloat4 c6 = vfloat4::loadu(&ptr[index[6]]->tnear);
    const vfloat4 c7 = vfloat4::loadu(&ptr[index[7]]->tnear);

    vfloat8 maskf;
    transpose(c0,c1,c2,c3,c4,c5,c6,c7, ray.tnear, ray.tfar, ray.time, maskf);
    ray.mask = asInt(maskf);

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

    /* load and transpose: org.x, org.y, org.z, align0, dir.x, dir.y, dir.z, align1 */
    const vfloat8 ab0  = vfloat8::loadu(&ptr[index[ 0]]->org);
    const vfloat8 ab1  = vfloat8::loadu(&ptr[index[ 1]]->org);
    const vfloat8 ab2  = vfloat8::loadu(&ptr[index[ 2]]->org);
    const vfloat8 ab3  = vfloat8::loadu(&ptr[index[ 3]]->org);
    const vfloat8 ab4  = vfloat8::loadu(&ptr[index[ 4]]->org);
    const vfloat8 ab5  = vfloat8::loadu(&ptr[index[ 5]]->org);
    const vfloat8 ab6  = vfloat8::loadu(&ptr[index[ 6]]->org);
    const vfloat8 ab7  = vfloat8::loadu(&ptr[index[ 7]]->org);
    const vfloat8 ab8  = vfloat8::loadu(&ptr[index[ 8]]->org);
    const vfloat8 ab9  = vfloat8::loadu(&ptr[index[ 9]]->org);
    const vfloat8 ab10 = vfloat8::loadu(&ptr[index[10]]->org);
    const vfloat8 ab11 = vfloat8::loadu(&ptr[index[11]]->org);
    const vfloat8 ab12 = vfloat8::loadu(&ptr[index[12]]->org);
    const vfloat8 ab13 = vfloat8::loadu(&ptr[index[13]]->org);
    const vfloat8 ab14 = vfloat8::loadu(&ptr[index[14]]->org);
    const vfloat8 ab15 = vfloat8::loadu(&ptr[index[15]]->org);

    vfloat16 unused0, unused1;
    transpose(ab0,ab1,ab2,ab3,ab4,ab5,ab6,ab7,ab8,ab9,ab10,ab11,ab12,ab13,ab14,ab15,
              ray.org.x, ray.org.y, ray.org.z, unused0, ray.dir.x, ray.dir.y, ray.dir.z, unused1);

    /* load and transpose: tnear, tfar, time, mask */
    const vfloat4 c0  = vfloat4::loadu(&ptr[index[ 0]]->tnear);
    const vfloat4 c1  = vfloat4::loadu(&ptr[index[ 1]]->tnear);
    const vfloat4 c2  = vfloat4::loadu(&ptr[index[ 2]]->tnear);
    const vfloat4 c3  = vfloat4::loadu(&ptr[index[ 3]]->tnear);
    const vfloat4 c4  = vfloat4::loadu(&ptr[index[ 4]]->tnear);
    const vfloat4 c5  = vfloat4::loadu(&ptr[index[ 5]]->tnear);
    const vfloat4 c6  = vfloat4::loadu(&ptr[index[ 6]]->tnear);
    const vfloat4 c7  = vfloat4::loadu(&ptr[index[ 7]]->tnear);
    const vfloat4 c8  = vfloat4::loadu(&ptr[index[ 8]]->tnear);
    const vfloat4 c9  = vfloat4::loadu(&ptr[index[ 9]]->tnear);
    const vfloat4 c10 = vfloat4::loadu(&ptr[index[10]]->tnear);
    const vfloat4 c11 = vfloat4::loadu(&ptr[index[11]]->tnear);
    const vfloat4 c12 = vfloat4::loadu(&ptr[index[12]]->tnear);
    const vfloat4 c13 = vfloat4::loadu(&ptr[index[13]]->tnear);
    const vfloat4 c14 = vfloat4::loadu(&ptr[index[14]]->tnear);
    const vfloat4 c15 = vfloat4::loadu(&ptr[index[15]]->tnear);

    vfloat16 maskf;
    transpose(c0,c1,c2,c3,c4,c5,c6,c7,c8,c9,c10,c11,c12,c13,c14,c15,
              ray.tnear, ray.tfar, ray.time, maskf);
    ray.mask = asInt(maskf);

    ray.geomID = RTC_INVALID_GEOMETRY_ID;

    return ray;
  }
#endif
}
