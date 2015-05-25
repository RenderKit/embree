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

#include "../../common/default.h"
#include "primitive.h"

#define EVAL_BOUNDS 1

namespace embree
{
  extern Vec4f16 coeff0;
  extern Vec4f16 coeff1;
  extern Vec4f16 coeff01;

  extern Vec4f16 coeff_P0;
  extern Vec4f16 coeff_P1;
  extern Vec4f16 coeff_P2;
  extern Vec4f16 coeff_P3;

  __forceinline Vec3f16 convert(const LinearSpace3fa &mat)
  {
    return Vec3f16(broadcast4to16f((float*)&mat.vx),
		 broadcast4to16f((float*)&mat.vy),
		 broadcast4to16f((float*)&mat.vz));
  }

  __forceinline float16 xfmPoint4f(const Vec3fa &p, 
				 const float16 &c0,
				 const float16 &c1,
				 const float16 &c2)
  {
    const float16 xyz  = c0 * float16(p.x) + c1 * float16(p.y) + c2 * float16(p.z);
    const float16 xyzw = select(0x7777,xyz,float16(p.w));
    return xyzw;
  }


    static __forceinline Vec4f16 eval16(const float16 &p0,
				      const float16 &p1,
				      const float16 &p2,
				      const float16 &p3)
    {
      const float16 c0 = coeff01.x;
      const float16 c1 = coeff01.y;
      const float16 c2 = coeff01.z;
      const float16 c3 = coeff01.w;

      const float16 x = c0 * swAAAA(p0) + c1 * swAAAA(p1) + c2 * swAAAA(p2) + c3 * swAAAA(p3);
      const float16 y = c0 * swBBBB(p0) + c1 * swBBBB(p1) + c2 * swBBBB(p2) + c3 * swBBBB(p3);
      const float16 z = c0 * swCCCC(p0) + c1 * swCCCC(p1) + c2 * swCCCC(p2) + c3 * swCCCC(p3);
      const float16 w = c0 * swDDDD(p0) + c1 * swDDDD(p1) + c2 * swDDDD(p2) + c3 * swDDDD(p3);
      return Vec4f16(x,y,z,w);
    }


  struct __aligned(16) Bezier1i
  {
  public:

    /*! Default constructor. */
    __forceinline Bezier1i () {}

    /*! Construction from vertices and IDs. */
    __forceinline Bezier1i (const Vec3fa* p, const unsigned int geomID, const unsigned int primID)
      : p(p), geomID(geomID), primID(primID) {}

    /*! calculate the bounds of the triangle */
    __forceinline BBox3fa bounds() const {
      const BBox3fa b = merge(BBox3fa(p[0]),BBox3fa(p[1]),BBox3fa(p[2]),BBox3fa(p[3]));
      return enlarge(b,Vec3fa(b.upper.w));
    }

    __forceinline Vec2f16 getBounds() const 
    {
      const float16 v0 = broadcast4to16f((float*)&p[0]);
      const float16 v1 = broadcast4to16f((float*)&p[1]);
      const float16 v2 = broadcast4to16f((float*)&p[2]);
      const float16 v3 = broadcast4to16f((float*)&p[3]);
      
#if EVAL_BOUNDS==1
      const Vec4f16 v = eval16(v0,v1,v2,v3);
      const float16 min_x = min(vreduce_min(v.x),v3[0]);
      const float16 max_x = max(vreduce_max(v.x),v3[0]);
      const float16 min_y = min(vreduce_min(v.y),v3[1]);
      const float16 max_y = max(vreduce_max(v.y),v3[1]);
      const float16 min_z = min(vreduce_min(v.z),v3[2]);
      const float16 max_z = max(vreduce_max(v.z),v3[2]);
      const float16 b_min = select(0x4444,min_z,select(0x2222,min_y,min_x));
      const float16 b_max = select(0x4444,max_z,select(0x2222,max_y,max_x));

      const float16 r_max = max(max(v0,v1),max(v2,v3));
      const float16 b_min_r = b_min - swDDDD(r_max);
      const float16 b_max_r = b_max + swDDDD(r_max);

#else
      const float16 b_min = min(min(v0,v1),min(v2,v3));
      const float16 b_max = max(max(v0,v1),max(v2,v3));
      
      const float16 b_min_r = b_min - swDDDD(b_max);
      const float16 b_max_r = b_max + swDDDD(b_max);
#endif      
      return Vec2f16(b_min_r,b_max_r);
    }


    __forceinline Vec2f16 getBounds(LinearSpace3fa &xfm) const 
    {
      const Vec3fa q0 = xfmPoint(xfm,p[0]);
      const Vec3fa q1 = xfmPoint(xfm,p[1]);
      const Vec3fa q2 = xfmPoint(xfm,p[2]);
      const Vec3fa q3 = xfmPoint(xfm,p[3]);
      
      const Vec3fa b_min = min(min(q0,q1),min(q2,q3));
      const Vec3fa b_max = max(max(q0,q1),max(q2,q3));

      const Vec3fa max_radius = max(max(p[0].w,p[1].w),max(p[2].w,p[3].w));
      
      const Vec3fa b_min_r = b_min - max_radius;
      const Vec3fa b_max_r = b_max + max_radius;

      const float16 b_lower = broadcast4to16f((float*)&b_min_r);
      const float16 b_upper = broadcast4to16f((float*)&b_max_r);
      
      return Vec2f16(b_lower,b_upper);
    }

    __forceinline Vec2f16 getBounds(const float16 &c0,const float16 &c1,const float16 &c2) const 
    {
      const float16 v0 = xfmPoint4f(p[0],c0,c1,c2);
      const float16 v1 = xfmPoint4f(p[1],c0,c1,c2);
      const float16 v2 = xfmPoint4f(p[2],c0,c1,c2);
      const float16 v3 = xfmPoint4f(p[3],c0,c1,c2);

#if EVAL_BOUNDS==1
      const Vec4f16 v = eval16(v0,v1,v2,v3);
      const float16 min_x = min(vreduce_min(v.x),v3[0]);
      const float16 max_x = max(vreduce_max(v.x),v3[0]);
      const float16 min_y = min(vreduce_min(v.y),v3[1]);
      const float16 max_y = max(vreduce_max(v.y),v3[1]);
      const float16 min_z = min(vreduce_min(v.z),v3[2]);
      const float16 max_z = max(vreduce_max(v.z),v3[2]);
      const float16 b_min = select(0x4444,min_z,select(0x2222,min_y,min_x));
      const float16 b_max = select(0x4444,max_z,select(0x2222,max_y,max_x));

      const float16 r_max = max(max(v0,v1),max(v2,v3));
      const float16 b_min_r = b_min - swDDDD(r_max);
      const float16 b_max_r = b_max + swDDDD(r_max);

#else
      const float16 b_min = min(min(v0,v1),min(v2,v3));
      const float16 b_max = max(max(v0,v1),max(v2,v3));
      const float16 b_min_r = b_min - swDDDD(b_max);
      const float16 b_max_r = b_max + swDDDD(b_max);
#endif      
      
      return Vec2f16(b_min_r,b_max_r);
    }       

    template<int HINT>
    __forceinline void prefetchControlPoints() const {
      prefetch<HINT>(p + 0);
      prefetch<HINT>(p + 3);
    }

    const Vec3fa* p;      //!< pointer to first control point (x,y,z,r)
    unsigned int geomID;  //!< geometry ID
    unsigned int primID;  //!< primitive ID
  };

};
