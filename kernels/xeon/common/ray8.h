// ======================================================================== //
// Copyright 2009-2013 Intel Corporation                                    //
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

#ifndef __EMBREE_RAY8_H__
#define __EMBREE_RAY8_H__

#include "ray.h"

namespace embree
{
  /*! Ray structure. Contains all information of 8 rays including
   *  precomputed reciprocal direction. */
  struct Ray8
  {
    /*! Default construction does nothing. */
    __forceinline Ray8() {}

    /*! Constructs a ray from origin, direction, and ray segment. Near
     *  has to be smaller than far. */
    __forceinline Ray8(const avx3f& org, const avx3f& dir, const avxf& tnear = zero, const avxf& tfar = inf, const avxf& time = zero, const avxi& mask = -1)
      : org(org), dir(dir), tnear(tnear), tfar(tfar), geomID(-1), primID(-1), instID(-1), mask(mask), time(time)  {}

    /*! Tests if we hit something. */
    __forceinline operator avxb() const { return geomID != avxi(-1); }

  public:
    avx3f org;      //!< Ray origin
    avx3f dir;      //!< Ray direction
    avxf tnear;     //!< Start of ray segment 
    avxf tfar;      //!< End of ray segment   
    avxf time;      //!< Time of this ray for motion blur.
    avxi mask;      //!< used to mask out objects during traversal

  public:
    avx3f Ng;       //!< Geometry normal
    avxf u;         //!< Barycentric u coordinate of hit
    avxf v;         //!< Barycentric v coordinate of hit
    avxi geomID;    //!< geometry ID
    avxi primID;    //!< primitive ID
    avxi instID;    //!< instance ID
  };

  /*! Outputs ray to stream. */
  inline std::ostream& operator<<(std::ostream& cout, const Ray8& ray) {
    return cout << "{ " << 
      "org = " << ray.org << ", dir = " << ray.dir << ", near = " << ray.tnear << ", far = " << ray.tfar << ", time = " << ray.time << ", " <<
      "instID = " << ray.instID << ", geomID = " << ray.geomID << ", primID = " << ray.primID <<  ", " << "u = " << ray.u <<  ", v = " << ray.v << ", Ng = " << ray.Ng << " }";
  }

  /*! Ray structure for 8xN rays. */
  template <int N>
  struct Ray8x
  {
    /*! Default construction does nothing. */
    __forceinline Ray8x() {}

    __forceinline void get(size_t i, Ray8& dst) 
    {
      assert(i < N);
      dst.org.x = orgx[i];
      dst.org.y = orgy[i];
      dst.org.z = orgz[i];
      dst.dir.x = dirx[i];
      dst.dir.y = diry[i];
      dst.dir.z = dirz[i];
      dst.tnear = tnear[i];
      dst.tfar  = tfar[i];
      dst.time  = time[i];
      dst.mask  = mask[i];

      dst.Ng.x = Ngx[i];
      dst.Ng.y = Ngy[i];
      dst.Ng.z = Ngz[i];
      dst.u = u[i];
      dst.v = v[i];
      dst.geomID = geomID[i];
      dst.primID = primID[i];
      dst.instID = instID[i];
    }

    __forceinline void set(size_t i, Ray8& src) 
    {
      assert(i < N);
      orgx[i] = src.org.x;
      orgy[i] = src.org.y;
      orgz[i] = src.org.z;
      dirx[i] = src.dir.x;
      diry[i] = src.dir.y;
      dirz[i] = src.dir.z;
      tnear[i] = src.tnear;
      tfar[i] = src.tfar;
      time[i] = src.time;
      mask[i] = src.mask;

      Ngx[i] = src.Ng.x;
      Ngy[i] = src.Ng.y;
      Ngz[i] = src.Ng.z;
      u[i] = src.u;
      v[i] = src.v;
      geomID[i] = src.geomID;
      primID[i] = src.primID;
      instID[i] = src.instID;
    }

  public:
    avxf orgx[N];   //!< Ray origin
    avxf orgy[N];
    avxf orgz[N];
    avxf dirx[N];   //!< Ray direction
    avxf diry[N];
    avxf dirz[N];
    avxf tnear[N];  //!< Start of ray segment 
    avxf tfar[N];   //!< End of ray segment   
    avxf time[N];   //!< Time of this ray for motion blur.
    avxi mask[N];   //!< used to mask out objects during traversal

  public:
    avxf Ngx[N];    //!< Geometry normal
    avxf Ngy[N];
    avxf Ngz[N];
    avxf u[N];      //!< Barycentric u coordinate of hit
    avxf v[N];      //!< Barycentric v coordinate of hit
    avxi geomID[N]; //!< geometry ID
    avxi primID[N]; //!< primitive ID
    avxi instID[N]; //!< instance ID
  };

  /*! Converts a 8-wide AVX intersector to an 16-wide AVX intersector */
  template<typename BVH, typename Intersector8>
  struct Intersector8To16
  {
    static Accel::Intersector16 create () { 
      return Accel::Intersector16((RTCIntersectFunc16)intersect16, (RTCOccludedFunc16)occluded16); 
    }

    static void intersect16(avxb* valid, BVH* bvh, Ray8x<2>& ray16)
    {
      Ray8 ray8;
      for (size_t i=0; i<2; i++) {
        ray16.get(i,ray8);
        Intersector8::intersect(valid+i,bvh,ray8);
        ray16.set(i,ray8);
      }
    }
    
    static void occluded16(avxb* valid, BVH* bvh, Ray8x<2>& ray16) 
    {
      Ray8 ray8;
      for (size_t i=0; i<2; i++) {
        ray16.get(i,ray8);
        Intersector8::occluded(valid+i,bvh,ray8);
        ray16.set(i,ray8);
      }
    }
  };

}

#endif
