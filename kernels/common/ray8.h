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
  /*! Ray structure. Contains all information of 8 rays including
   *  precomputed reciprocal direction. */
  struct Ray8
  {
    typedef vbool8 simdb;
    typedef vfloat8 simdf;
    typedef vint8 simdi;

    /*! Default construction does nothing. */
    __forceinline Ray8() {}

    /*! Constructs a ray from origin, direction, and ray segment. Near
     *  has to be smaller than far. */
    __forceinline Ray8(const Vec3vf8& org, const Vec3vf8& dir, 
                       const vfloat8& tnear = zero, const vfloat8& tfar = inf, 
                       const vfloat8& time = zero, const vint8& mask = -1)
      : org(org), dir(dir), tnear(tnear), tfar(tfar), geomID(-1), primID(-1), instID(-1), mask(mask), time(time)  {}

    /*! returns the size of the ray */
    static __forceinline size_t size() { return 8; }
    
    /*! Tests if we hit something. */
    __forceinline operator vbool8() const { return geomID != vint8(-1); }

    /*! calculates if this is a valid ray that does not cause issues during traversal */
    __forceinline vbool8 valid() const {
      const vbool8 vx = abs(org.x) <= vfloat8(FLT_LARGE) & abs(dir.x) <= vfloat8(FLT_LARGE);
      const vbool8 vy = abs(org.y) <= vfloat8(FLT_LARGE) & abs(dir.y) <= vfloat8(FLT_LARGE);
      const vbool8 vz = abs(org.z) <= vfloat8(FLT_LARGE) & abs(dir.z) <= vfloat8(FLT_LARGE);
      const vbool8 vn = abs(tnear) <= vfloat8(inf);
      const vbool8 vf = abs(tfar) <= vfloat8(inf);
      return vx & vy & vz & vn & vf;
    }

    /* converts ray packet to single rays */
    __forceinline void get(Ray ray[8]) const
    {
      for (size_t i=0; i<8; i++) // FIXME: use SSE and AVX transpose
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
    __forceinline void set(const Ray ray[8])
    {
      for (size_t i=0; i<8; i++)
      {
	org.x[i] = ray[i].org.x; org.y[i] = ray[i].org.y; org.z[i] = ray[i].org.z;
	dir.x[i] = ray[i].dir.x; dir.y[i] = ray[i].dir.y; dir.z[i] = ray[i].dir.z;
	tnear[i] = ray[i].tnear; tfar [i] = ray[i].tfar;  time[i] = ray[i].time; mask[i] = ray[i].mask;
	Ng.x[i] = ray[i].Ng.x; Ng.y[i] = ray[i].Ng.y; Ng.z[i] = ray[i].Ng.z;
	u[i] = ray[i].u; v[i] = ray[i].v;
	geomID[i] = ray[i].geomID; primID[i] = ray[i].primID; instID[i] = ray[i].instID;
      }
    }

  public:
    Vec3vf8 org;      //!< Ray origin
    Vec3vf8 dir;      //!< Ray direction
    vfloat8 tnear;     //!< Start of ray segment 
    vfloat8 tfar;      //!< End of ray segment   
    vfloat8 time;      //!< Time of this ray for motion blur.
    vint8 mask;      //!< used to mask out objects during traversal

  public:
    Vec3vf8 Ng;       //!< Geometry normal
    vfloat8 u;         //!< Barycentric u coordinate of hit
    vfloat8 v;         //!< Barycentric v coordinate of hit
    vint8 geomID;    //!< geometry ID
    vint8 primID;    //!< primitive ID
    vint8 instID;    //!< instance ID
  };

  /*! Outputs ray to stream. */
  inline std::ostream& operator<<(std::ostream& cout, const Ray8& ray) {
    return cout << "{ " 
                << "org = " << ray.org << std::endl
                << ", dir = " << ray.dir << std::endl
                << ", near = " << ray.tnear << std::endl
                << ", far = " << ray.tfar << std::endl
                << ", time = " << ray.time << std::endl
                << ", instID = " << ray.instID << std::endl
                << ", geomID = " << ray.geomID << std::endl
                << ", primID = " << ray.primID << std::endl
                <<  ", u = " << ray.u << std::endl
                <<  ", v = " << ray.v << std::endl
                << ", Ng = " << ray.Ng << std::endl
                << " }";
  }
}
