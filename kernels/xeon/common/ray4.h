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

#ifndef __EMBREE_RAY4_H__
#define __EMBREE_RAY4_H__

#include "ray.h"

namespace embree
{
  /*! Ray structure for 4 rays. */
  struct Ray4
  {
    /*! Default construction does nothing. */
    __forceinline Ray4() {}

    /*! Constructs a ray from origin, direction, and ray segment. Near
     *  has to be smaller than far. */
    __forceinline Ray4(const sse3f& org, const sse3f& dir, const ssef& tnear = zero, const ssef& tfar = inf, const ssef& time = zero, const ssei& mask = -1)
      : org(org), dir(dir), tnear(tnear), tfar(tfar), geomID(-1), primID(-1), instID(-1), mask(mask), time(time) {}

    /*! Tests if we hit something. */
    __forceinline operator sseb() const { return geomID != ssei(-1); }

  public:
    sse3f org;      //!< Ray origin
    sse3f dir;      //!< Ray direction
    ssef tnear;     //!< Start of ray segment 
    ssef tfar;      //!< End of ray segment   
    ssef time;      //!< Time of this ray for motion blur.
    ssei mask;      //!< used to mask out objects during traversal

  public:
    sse3f Ng;       //!< Geometry normal
    ssef u;         //!< Barycentric u coordinate of hit
    ssef v;         //!< Barycentric v coordinate of hit
    ssei geomID;    //!< geometry ID
    ssei primID;    //!< primitive ID
    ssei instID;    //!< instance ID
  };

  /*! Outputs ray to stream. */
  inline std::ostream& operator<<(std::ostream& cout, const Ray4& ray) {
    return cout << "{ " << 
      "org = " << ray.org << ", dir = " << ray.dir << ", near = " << ray.tnear << ", far = " << ray.tfar << ", time = " << ray.time << ", " <<
      "instID = " << ray.instID << ", geomID = " << ray.geomID << ", primID = " << ray.primID <<  ", " << "u = " << ray.u <<  ", v = " << ray.v << ", Ng = " << ray.Ng << " }";
  }

  /*! Ray structure for 4xN rays. */
  template <int N>
  struct Ray4x
  {
    /*! Default construction does nothing. */
    __forceinline Ray4x() {}

    __forceinline void get(size_t i, Ray4& dst) 
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

    __forceinline void set(size_t i, Ray4& src) 
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
    ssef orgx[N];   //!< Ray origin
    ssef orgy[N];
    ssef orgz[N];
    ssef dirx[N];   //!< Ray direction
    ssef diry[N];
    ssef dirz[N];
    ssef tnear[N];  //!< Start of ray segment 
    ssef tfar[N];   //!< End of ray segment   
    ssef time[N];   //!< Time of this ray for motion blur.
    ssei mask[N];   //!< used to mask out objects during traversal

  public:
    ssef Ngx[N];    //!< Geometry normal
    ssef Ngy[N];
    ssef Ngz[N];
    ssef u[N];      //!< Barycentric u coordinate of hit
    ssef v[N];      //!< Barycentric v coordinate of hit
    ssei geomID[N]; //!< geometry ID
    ssei primID[N]; //!< primitive ID
    ssei instID[N]; //!< instance ID
  };

  /*! Converts a 4-wide SSE intersector to an 8-wide SSE intersector */
  template<typename BVH, typename Intersector4>
  struct Intersector4To8
  {
    static Accel::Intersector8 create () { 
      return Accel::Intersector8((RTCIntersectFunc8)intersect8, (RTCOccludedFunc8)occluded8); 
    }

    static void intersect8(sseb* valid, BVH* bvh, Ray4x<2>& ray8)
    {
      Ray4 ray4;
      for (size_t i=0; i<2; i++) {
        ray8.get(i,ray4);
        Intersector4::intersect(valid+i,bvh,ray4);
        ray8.set(i,ray4);
      }
    }
    
    static void occluded8(sseb* valid, BVH* bvh, Ray4x<2>& ray8) 
    {
      Ray4 ray4;
      for (size_t i=0; i<2; i++) {
        ray8.get(i,ray4);
        Intersector4::occluded(valid+i,bvh,ray4);
        ray8.set(i,ray4);
      }
    }
  };

  /*! Converts a 4-wide SSE intersector to an 16-wide SSE intersector */
  template<typename BVH, typename Intersector4>
    struct Intersector4To16
  {
    static Accel::Intersector16 create () { 
      return Accel::Intersector16((RTCIntersectFunc16)intersect16, (RTCOccludedFunc16)occluded16); 
    }

    static void intersect16(sseb* valid, BVH* bvh, Ray4x<4>& ray16)
    {
      Ray4 ray4;
      for (size_t i=0; i<4; i++) {
        ray16.get(i,ray4);
        Intersector4::intersect(valid+i,bvh,ray4);
        ray16.set(i,ray4);
      }
    }
    
    static void occluded16(sseb* valid, BVH* bvh, Ray4x<4>& ray16) 
    {
      Ray4 ray4;
      for (size_t i=0; i<4; i++) {
        ray16.get(i,ray4);
        Intersector4::occluded(valid+i,bvh,ray4);
        ray16.set(i,ray4);
      }
    }
  };
}

#endif
