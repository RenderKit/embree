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

#ifndef __EMBREE_ACCEL_TRIANGLE1V_INTERSECTOR8_PLUECKER_H__
#define __EMBREE_ACCEL_TRIANGLE1V_INTERSECTOR8_PLUECKER_H__

#include "triangle1v.h"
#include "../common/ray8.h"

namespace embree
{
  /*! Modified Pluecker ray/triangle intersector. The test first shifts the ray
   *  origin into the origin of the coordinate system and then uses
   *  Pluecker coordinates for the intersection. Due to the shift, the
   *  Pluecker coordinate calculation simplifies. The edge equations
   *  are watertight along the edge for neighboring triangles. */
  struct Triangle1vIntersector8Pluecker
  {
    typedef Triangle1v Primitive;
    
    static __forceinline void intersect(const avxb& valid_i, Ray8& ray, const Triangle1v* __restrict__ tris, size_t num, const void* geom)
    {
      for (size_t i=0; i<num; i++) 
      {
        STAT3(normal.trav_prims,1,popcnt(valid_i),8);

        /* calculate vertices relative to ray origin */
        avxb valid = valid_i;
        const Triangle1v& tri = tris[i];
        const avx3f O = ray.org;
        const avx3f D = ray.dir;
        const avx3f v0 = avx3f(tri.v0)-O;
        const avx3f v1 = avx3f(tri.v1)-O;
        const avx3f v2 = avx3f(tri.v2)-O;

        /* calculate triangle edges */
        const avx3f e0 = v2-v0;
        const avx3f e1 = v0-v1;
        const avx3f e2 = v1-v2;

        /* calculate geometry normal and denominator */
        const avx3f Ng = cross(e1,e0);
        const avx3f Ng2 = Ng+Ng;
        const avxf den = dot(Ng2,D);
        const avxf absDen = abs(den);
        const avxf sgnDen = signmsk(den);

        /* perform edge tests */
        const avxf U = dot(avx3f(cross(v2+v0,e0)),D) ^ sgnDen;
        valid &= U >= 0.0f;
        if (likely(none(valid))) continue;
        const avxf V = dot(avx3f(cross(v0+v1,e1)),D) ^ sgnDen;
        valid &= V >= 0.0f;
        if (likely(none(valid))) continue;
        const avxf W = dot(avx3f(cross(v1+v2,e2)),D) ^ sgnDen;
        valid &= W >= 0.0f;
        if (likely(none(valid))) continue;
      
        /* perform depth test */
        const avxf T = dot(v0,Ng2) ^ sgnDen;
        valid &= (T >= absDen*ray.tnear) & (absDen*ray.tfar >= T);
        if (unlikely(none(valid))) continue;

        /* perform backface culling */
#if defined(__BACKFACE_CULLING__)
        valid &= den > avxf(zero);
        if (unlikely(none(valid))) continue;
#else
        valid &= den != avxf(zero);
        if (unlikely(none(valid))) continue;
#endif
        
        /* ray masking test */
#if USE_RAY_MASK
        valid &= (tri.mask() & ray.mask) != 0;
        if (unlikely(none(valid))) continue;
#endif

        /* update hit information */
        const avxf rcpAbsDen = rcp(absDen);
        store8f(valid,&ray.u,U*rcpAbsDen);
        store8f(valid,&ray.v,V*rcpAbsDen);
        store8f(valid,&ray.tfar,T*rcpAbsDen);
        store8i(valid,&ray.geomID,tri.geomID());
        store8i(valid,&ray.primID,tri.primID());
        store8f(valid,&ray.Ng.x,Ng.x);
        store8f(valid,&ray.Ng.y,Ng.y);
        store8f(valid,&ray.Ng.z,Ng.z);
      }
    }

    static __forceinline avxb occluded(const avxb& valid_i, Ray8& ray, const Triangle1v* __restrict__ tris, size_t num, const void* geom)
    {
      avxb valid0 = valid_i;

      for (size_t i=0; i<num; i++) 
      {
        STAT3(shadow.trav_prims,1,popcnt(valid0),8);

        /* calculate vertices relative to ray origin */
        avxb valid = valid0;
        const Triangle1v& tri = tris[i];
        const avx3f O = ray.org;
        const avx3f D = ray.dir;
        const avx3f v0 = avx3f(tri.v0)-O;
        const avx3f v1 = avx3f(tri.v1)-O;
        const avx3f v2 = avx3f(tri.v2)-O;

        /* calculate triangle edges */
        const avx3f e0 = v2-v0;
        const avx3f e1 = v0-v1;
        const avx3f e2 = v1-v2;

        /* calculate geometry normal and denominator */
        const avx3f Ng = cross(e1,e0);
        const avx3f Ng2 = Ng+Ng;
        const avxf den = dot(Ng2,D);
        const avxf absDen = abs(den);
        const avxf sgnDen = signmsk(den);

        /* perform edge tests */
        const avxf U = dot(avx3f(cross(v2+v0,e0)),D) ^ sgnDen;
        valid &= U >= 0.0f;
        if (likely(none(valid))) continue;
        const avxf V = dot(avx3f(cross(v0+v1,e1)),D) ^ sgnDen;
        valid &= V >= 0.0f;
        if (likely(none(valid))) continue;
        const avxf W = dot(avx3f(cross(v1+v2,e2)),D) ^ sgnDen;
        valid &= W >= 0.0f;
        if (likely(none(valid))) continue;
      
        /* perform depth test */
        const avxf T = dot(v0,Ng2) ^ sgnDen;
        valid &= (T >= absDen*ray.tnear) & (absDen*ray.tfar >= T);
        if (unlikely(none(valid))) continue;

        /* perform backface culling */
#if defined(__BACKFACE_CULLING__)
        valid &= den > avxf(zero);
        if (unlikely(none(valid))) continue;
#else
        valid &= den != avxf(zero);
        if (unlikely(none(valid))) continue;
#endif
        
        /* ray masking test */
#if USE_RAY_MASK
        valid &= (tri.mask() & ray.mask) != 0;
        if (unlikely(none(valid))) continue;
#endif
        
        /* update occlusion */
        valid0 &= !valid;
        if (none(valid0)) break;
      }
      return !valid0;
    }
  };
}

#endif


