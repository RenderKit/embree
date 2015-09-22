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

#include "ray.h"

namespace embree
{
  /*! Ray structure. Contains all information of 16 rays including
   *  precomputed reciprocal direction. */
  struct Ray16
  {
    typedef vbool16 simdb;
    typedef vfloat16 simdf;
    typedef vint16 simdi;

    /*! Default construction does nothing. */
    __forceinline Ray16() {}

    /*! Constructs a ray from origin, direction, and ray segment. Near
     *  has to be smaller than far. */
    __forceinline Ray16(const Vec3vf16& org, const Vec3vf16& dir, 
                        const vfloat16& tnear = zero, const vfloat16& tfar = inf, 
                        const vfloat16& time = zero, const vint16& mask = -1)
      : org(org), dir(dir), tnear(tnear), tfar(tfar), geomID(-1), primID(-1), mask(mask), time(time) {}

    /*! returns the size of the ray */
    static __forceinline size_t size() { return 16; }

    /*! Tests if we hit something. */
    __forceinline operator vbool16() const { return geomID != vint16(-1); }

    /*! calculates if this is a valid ray that does not cause issues during traversal */
    __forceinline vbool16 valid() const {
      const vbool16 vx = abs(org.x) <= vfloat16(FLT_LARGE) & abs(dir.x) <= vfloat16(FLT_LARGE);
      const vbool16 vy = abs(org.y) <= vfloat16(FLT_LARGE) & abs(dir.y) <= vfloat16(FLT_LARGE);
      const vbool16 vz = abs(org.z) <= vfloat16(FLT_LARGE) & abs(dir.z) <= vfloat16(FLT_LARGE);
      const vbool16 vn = abs(tnear) <= vfloat16(inf);
      const vbool16 vf = abs(tfar) <= vfloat16(inf);
      return vx & vy & vz & vn & vf;
    }

  public:
    Vec3vf16 org;      //!< Ray origin
    Vec3vf16 dir;      //!< Ray direction
    vfloat16 tnear;    //!< Start of ray segment 
    vfloat16 tfar;     //!< End of ray segment   
    vfloat16 time;     //!< Time of this ray for motion blur.
    vint16 mask;     //!< used to mask out objects during traversal

    Vec3vf16 Ng;       //!< Geometry normal
    vfloat16 u;        //!< Barycentric u coordinate of hit
    vfloat16 v;        //!< Barycentric v coordinate of hit
    vint16 geomID;   //!< geometry ID
    vint16 primID;   //!< primitive ID
    vint16 instID;   //!< instance ID

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

    __forceinline void update(const vbool16 &m_mask,
			      const size_t rayIndex,
			      const vfloat16 &new_t,
			      const vfloat16 &new_u,
			      const vfloat16 &new_v,
			      const vfloat16 &new_gnormalx,
			      const vfloat16 &new_gnormaly,
			      const vfloat16 &new_gnormalz,
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

    __forceinline void update(const vbool16 &m_mask,
			      const vfloat16 &new_t,
			      const vfloat16 &new_u,
			      const vfloat16 &new_v,
			      const vfloat16 &new_gnormalx,
			      const vfloat16 &new_gnormaly,
			      const vfloat16 &new_gnormalz,
			      const vint16 &new_geomID,
			      const vint16 &new_primID)
    {
      store16f(m_mask,(float*)&tfar,new_t);
      store16f(m_mask,(float*)&u,new_u);
      store16f(m_mask,(float*)&v,new_v);
      store16f(m_mask,(float*)&Ng.x,new_gnormalx);
      store16f(m_mask,(float*)&Ng.y,new_gnormaly);
      store16f(m_mask,(float*)&Ng.z,new_gnormalz);
      store16i(m_mask,(int*)&geomID,new_geomID);
      store16i(m_mask,(int*)&primID,new_primID);     
    }

    /* converts ray packet to single rays */
    __forceinline void get(Ray ray[16]) const
    {
      for (size_t i=0; i<16; i++) // FIXME: use SSE and AVX transpose
      {
	ray[i].org.x = org.x[i]; ray[i].org.y = org.y[i]; ray[i].org.z = org.z[i]; 
	ray[i].dir.x = dir.x[i]; ray[i].dir.y = dir.y[i]; ray[i].dir.z = dir.z[i];
	ray[i].tnear = tnear[i]; ray[i].tfar  = tfar [i]; ray[i].time  = time[i]; ray[i].mask = mask[i];
	ray[i].Ng.x = Ng.x[i]; ray[i].Ng.y = Ng.y[i]; ray[i].Ng.z = Ng.z[i];
	ray[i].u = u[i]; ray[i].v = v[i];
	ray[i].geomID = geomID[i]; ray[i].primID = primID[i]; ray[i].instID = instID[i];
      }
    }

    /* converts single rays to ray packet */
    __forceinline void set(const Ray ray[16])
    {
      for (size_t i=0; i<16; i++)
      {
	org.x[i] = ray[i].org.x; org.y[i] = ray[i].org.y; org.z[i] = ray[i].org.z;
	dir.x[i] = ray[i].dir.x; dir.y[i] = ray[i].dir.y; dir.z[i] = ray[i].dir.z;
	tnear[i] = ray[i].tnear; tfar [i] = ray[i].tfar;  time[i] = ray[i].time; mask[i] = ray[i].mask;
	Ng.x[i] = ray[i].Ng.x; Ng.y[i] = ray[i].Ng.y; Ng.z[i] = ray[i].Ng.z;
	u[i] = ray[i].u; v[i] = ray[i].v;
	geomID[i] = ray[i].geomID; primID[i] = ray[i].primID; instID[i] = ray[i].instID;
      }
    }

  };

  /*! Outputs ray to stream. */
  inline std::ostream& operator<<(std::ostream& cout, const Ray16& ray) {
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
