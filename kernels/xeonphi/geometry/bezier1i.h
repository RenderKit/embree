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

#pragma once

#include "common/default.h"
#include "primitive.h"

namespace embree
{
  extern mic_f coeff0[4];
  extern mic_f coeff1[4];
  extern mic_f coeff01[4];

  extern mic_f coeff_P0[4];
  extern mic_f coeff_P1[4];
  extern mic_f coeff_P2[4];
  extern mic_f coeff_P3[4];

  __forceinline mic3f convert(const LinearSpace3fa &mat)
  {
    return mic3f(broadcast4to16f((float*)&mat.vx),
		 broadcast4to16f((float*)&mat.vy),
		 broadcast4to16f((float*)&mat.vz));
  }

  __forceinline mic_f xfmPoint(const Vec3fa  * __restrict__ const p, 
			       const mic_f &c0,
			       const mic_f &c1,
			       const mic_f &c2)
  {
    const mic_f p0123 = uload16f((float*)p);
    const mic_f p0123_1 = select(0x7777,p0123,mic_f::one());
    const mic_f x = ldot3_xyz(p0123_1,c0);
    const mic_f y = ldot3_xyz(p0123_1,c1);
    const mic_f z = ldot3_xyz(p0123_1,c2);
    const mic_f xyzw = select(0x7777,select(0x4444,z,select(0x2222,y,x)),p0123);
    return xyzw;
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

    __forceinline mic2f getBounds() const 
    {
      const mic_f v0 = broadcast4to16f((float*)&p[0]);
      const mic_f v1 = broadcast4to16f((float*)&p[1]);
      const mic_f v2 = broadcast4to16f((float*)&p[2]);
      const mic_f v3 = broadcast4to16f((float*)&p[3]);
      
      const mic_f b_min = min(min(v0,v1),min(v2,v3));
      const mic_f b_max = max(max(v0,v1),max(v2,v3));
      
      const mic_f b_min_r = b_min - swDDDD(b_max);
      const mic_f b_max_r = b_max + swDDDD(b_max);
      
      return mic2f(b_min_r,b_max_r);
    }

    __forceinline mic2f getBounds(const mic_f &c0,const mic_f &c1,const mic_f &c2) const 
    {
      const mic_f v0123 = xfmPoint(p,c0,c1,c2);
      const mic_f v0 = permute<0>(v0123);
      const mic_f v1 = permute<1>(v0123);
      const mic_f v2 = permute<2>(v0123);
      const mic_f v3 = permute<3>(v0123);	

      const mic_f b_min = min(min(v0,v1),min(v2,v3));
      const mic_f b_max = max(max(v0,v1),max(v2,v3));
      
      const mic_f b_min_r = b_min - swDDDD(b_max);
      const mic_f b_max_r = b_max + swDDDD(b_max);
      
      return mic2f(b_min_r,b_max_r);
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
