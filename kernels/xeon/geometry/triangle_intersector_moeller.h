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

#include "common/ray.h"
#include "geometry/filter.h"

namespace embree
{
  namespace isa
  {
     /*! Intersector for a single ray with N triangles. This intersector
     *  implements a modified version of the Moeller Trumbore
     *  intersector from the paper "Fast, Minimum Storage Ray-Triangle
     *  Intersection". In contrast to the paper we precalculate some
     *  factors and factor the calculations differently to allow
     *  precalculating the cross product e1 x e2. The resulting
     *  algorithm is similar to the fastest one of the paper "Optimizing
     *  Ray-Triangle Intersection via Automated Search". */

     template<typename TriangleN>
       struct TriangleNIntersector1Moeller
       {
         /* type shortcuts */
         typedef typename TriangleN::simdb simdb;
         typedef typename TriangleN::simdf simdf;
         typedef Vec3<simdf> simd3f;

         struct Precalculations {
          __forceinline Precalculations (const Ray& ray, const void* ptr) {}
         };
         
         /*! Intersect a ray with the 4 triangles and updates the hit. */
         static __forceinline void intersect(const Precalculations& pre, Ray& ray, const TriangleN& tri, Scene* scene)
         {
           /* calculate denominator */
           STAT3(normal.trav_prims,1,1,1);
           const simd3f O = simd3f(ray.org);
           const simd3f D = simd3f(ray.dir);
           const simd3f C = simd3f(tri.v0) - O;
           const simd3f R = cross(D,C);
           const simdf den = dot(simd3f(tri.Ng),D);
           const simdf absDen = abs(den);
           const simdf sgnDen = signmsk(den);
           
           /* perform edge tests */
           const simdf U = dot(R,simd3f(tri.e2)) ^ sgnDen;
           const simdf V = dot(R,simd3f(tri.e1)) ^ sgnDen;
           
           /* perform backface culling */
#if defined(RTCORE_BACKFACE_CULLING)
           simdb valid = (den > simdf(zero)) & (U >= 0.0f) & (V >= 0.0f) & (U+V<=absDen);
#else
           simdb valid = (den != simdf(zero)) & (U >= 0.0f) & (V >= 0.0f) & (U+V<=absDen);
#endif
           if (likely(none(valid))) return;
           
           /* perform depth test */
           const simdf T = dot(simd3f(tri.Ng),C) ^ sgnDen;
           valid &= (T > absDen*simdf(ray.tnear)) & (T < absDen*simdf(ray.tfar));
           if (likely(none(valid))) return;
           
           /* ray masking test */
#if defined(RTCORE_RAY_MASK)
           valid &= (tri.mask & ray.mask) != 0;
           if (unlikely(none(valid))) return;
#endif
           
           /* calculate hit information */
           const simdf rcpAbsDen = rcp(absDen);
           const simdf u = U * rcpAbsDen;
           const simdf v = V * rcpAbsDen;
           const simdf t = T * rcpAbsDen;
           size_t i = select_min(valid,t);
           int geomID = tri.geomID(i);
           
      /* intersection filter test */
#if defined(RTCORE_INTERSECTION_FILTER)
           while (true) 
           {
             Geometry* geometry = scene->get(geomID);
             if (likely(!geometry->hasIntersectionFilter1())) 
             {
#endif
               /* update hit information */
               ray.u = u[i];
               ray.v = v[i];
               ray.tfar = t[i];
               ray.Ng.x = tri.Ng.x[i];
               ray.Ng.y = tri.Ng.y[i];
               ray.Ng.z = tri.Ng.z[i];
               ray.geomID = geomID;
               ray.primID = tri.primID(i);
               //ray.primID = (unsigned) (size_t) &tri.Ng.x[i]; 
               
#if defined(RTCORE_INTERSECTION_FILTER)
               return;
             }
             
             Vec3fa Ng = Vec3fa(tri.Ng.x[i],tri.Ng.y[i],tri.Ng.z[i]);
             if (runIntersectionFilter1(geometry,ray,u[i],v[i],t[i],Ng,geomID,tri.primID(i))) return;
             valid[i] = 0;
             if (none(valid)) return;
             i = select_min(valid,t);
             geomID = tri.geomID(i);
           }
#endif
         }

         /*! Test if the ray is occluded by one of the triangles. */
         static __forceinline bool occluded(const Precalculations& pre, Ray& ray, const TriangleN& tri, Scene* scene)
         {
           /* calculate denominator */
           STAT3(shadow.trav_prims,1,1,1);
           const simd3f O = simd3f(ray.org);
           const simd3f D = simd3f(ray.dir);
           const simd3f C = simd3f(tri.v0) - O;
           const simd3f R = cross(D,C);
           const simdf den = dot(simd3f(tri.Ng),D);
           const simdf absDen = abs(den);
           const simdf sgnDen = signmsk(den);
           
           /* perform edge tests */
           const simdf U = dot(R,simd3f(tri.e2)) ^ sgnDen;
           const simdf V = dot(R,simd3f(tri.e1)) ^ sgnDen;
           const simdf W = absDen-U-V;
           simdb valid = (U >= 0.0f) & (V >= 0.0f) & (W >= 0.0f);
           if (unlikely(none(valid))) return false;
           
           /* perform depth test */
           const simdf T = dot(simd3f(tri.Ng),C) ^ sgnDen;
           valid &= (den != simdf(zero)) & (T >= absDen*simdf(ray.tnear)) & (absDen*simdf(ray.tfar) >= T);
           if (unlikely(none(valid))) return false;
           
           /* perform backface culling */
#if defined(RTCORE_BACKFACE_CULLING)
           valid &= den > simdf(zero);
           if (unlikely(none(valid))) return false;
#endif
           
           /* ray masking test */
#if defined(RTCORE_RAY_MASK)
           valid &= (tri.mask & ray.mask) != 0;
           if (unlikely(none(valid))) return false;
#endif
           
           /* intersection filter test */
#if defined(RTCORE_INTERSECTION_FILTER)
           size_t m=movemask(valid), i=__bsf(m);
           while (true)
           {  
             const int geomID = tri.geomID(i);
             Geometry* geometry = scene->get(geomID);
             
             /* if we have no filter then the test pasimds */
             if (likely(!geometry->hasOcclusionFilter1()))
               break;
             
             /* calculate hit information */
             const simdf rcpAbsDen = rcp(absDen);
             const simdf u = U * rcpAbsDen;
             const simdf v = V * rcpAbsDen;
             const simdf t = T * rcpAbsDen;
             const Vec3fa Ng = Vec3fa(tri.Ng.x[i],tri.Ng.y[i],tri.Ng.z[i]);
             if (runOcclusionFilter1(geometry,ray,u[i],v[i],t[i],Ng,geomID,tri.primID(i))) 
               break;
             
             /* test if one more triangle hit */
             m=__btc(m,i); i=__bsf(m);
             if (m == 0) return false;
           }
#endif
           
           return true;
         }
       };
  }
}
