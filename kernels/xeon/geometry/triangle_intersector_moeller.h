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
#include "common/ray4.h"
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

     /*! Intersector for 4 triangles with 4 rays. This intersector
     *  implements a modified version of the Moeller Trumbore
     *  intersector from the paper "Fast, Minimum Storage Ray-Triangle
     *  Intersection". In contrast to the paper we precalculate some
     *  factors and factor the calculations differently to allow
     *  precalculating the cross product e1 x e2. */
     template<typename TriangleN, bool enableIntersectionFilter>
       struct TriangleNIntersector4MoellerTrumbore
       {
         /* type shortcuts */
         typedef typename TriangleN::simdb simdb;
         typedef typename TriangleN::simdf simdf;
         typedef Vec3<simdf> simd3f;

         struct Precalculations {
           __forceinline Precalculations (const sseb& valid, const Ray4& ray) {}
         };
         
         /*! Intersects a 4 rays with 4 triangles. */
         static __forceinline void intersect(const sseb& valid_i, Precalculations& pre, Ray4& ray, const TriangleN& tri, Scene* scene)
         {
           for (size_t i=0; i<4; i++)
           {
             if (!tri.valid(i)) break;
             STAT3(normal.trav_prims,1,popcnt(valid_i),4);
             
             /* load edges and geometry normal */
             sseb valid = valid_i;
             const sse3f p0 = broadcast4f(tri.v0,i);
             const sse3f e1 = broadcast4f(tri.e1,i);
             const sse3f e2 = broadcast4f(tri.e2,i);
             const sse3f Ng = broadcast4f(tri.Ng,i);
             
             /* calculate denominator */
             const sse3f C = p0 - ray.org;
             const sse3f R = cross(ray.dir,C);
             const ssef den = dot(Ng,ray.dir);
             const ssef absDen = abs(den);
             const ssef sgnDen = signmsk(den);
             
             /* test against edge p2 p0 */
             const ssef U = dot(R,e2) ^ sgnDen;
             valid &= U >= 0.0f;
             if (likely(none(valid))) continue;
             
             /* test against edge p0 p1 */
             const ssef V = dot(R,e1) ^ sgnDen;
             valid &= V >= 0.0f;
             if (likely(none(valid))) continue;
             
             /* test against edge p1 p2 */
             const ssef W = absDen-U-V;
             valid &= W >= 0.0f;
             if (likely(none(valid))) continue;
             
             /* perform depth test */
             const ssef T = dot(Ng,C) ^ sgnDen;
             valid &= (T >= absDen*ray.tnear) & (absDen*ray.tfar >= T);
             if (unlikely(none(valid))) continue;
             
             /* perform backface culling */
#if defined(RTCORE_BACKFACE_CULLING)
             valid &= den > ssef(zero);
             if (unlikely(none(valid))) continue;
#else
             valid &= den != ssef(zero);
             if (unlikely(none(valid))) continue;
#endif
             
             /* ray masking test */
#if defined(RTCORE_RAY_MASK)
             valid &= (tri.mask[i] & ray.mask) != 0;
             if (unlikely(none(valid))) continue;
#endif
             
             /* calculate hit information */
             const ssef rcpAbsDen = rcp(absDen);
             const ssef u = U*rcpAbsDen;
             const ssef v = V*rcpAbsDen;
             const ssef t = T*rcpAbsDen;
             const int geomID = tri.geomID(i);
             const int primID = tri.primID(i);
             
             /* intersection filter test */
#if defined(RTCORE_INTERSECTION_FILTER)
             if (enableIntersectionFilter) {
               Geometry* geometry = scene->get(geomID);
               if (unlikely(geometry->hasIntersectionFilter4())) {
                 runIntersectionFilter4(valid,geometry,ray,u,v,t,Ng,geomID,primID);
                 continue;
               }
             }
#endif
             
             /* update hit information */
             ssef::store(valid,&ray.u,u);
             ssef::store(valid,&ray.v,v);
             ssef::store(valid,&ray.tfar,t);
             ssei::store(valid,&ray.geomID,geomID);
             ssei::store(valid,&ray.primID,primID);
             ssef::store(valid,&ray.Ng.x,Ng.x);
             ssef::store(valid,&ray.Ng.y,Ng.y);
             ssef::store(valid,&ray.Ng.z,Ng.z);
           }
         }
         
         /*! Test for 4 rays if they are occluded by any of the 4 triangle. */
         static __forceinline sseb occluded(const sseb& valid_i, Precalculations& pre, Ray4& ray, const TriangleN& tri, Scene* scene)
         {
           sseb valid0 = valid_i;
           
           for (size_t i=0; i<4; i++)
           {
             if (!tri.valid(i)) break;
             STAT3(shadow.trav_prims,1,popcnt(valid0),4);
             
             /* load edges and geometry normal */
             sseb valid = valid0;
             const sse3f p0 = broadcast4f(tri.v0,i);
             const sse3f e1 = broadcast4f(tri.e1,i);
             const sse3f e2 = broadcast4f(tri.e2,i);
             const sse3f Ng = broadcast4f(tri.Ng,i);
             
             /* calculate denominator */
             const sse3f C = p0 - ray.org;
             const sse3f R = cross(ray.dir,C);
             const ssef den = dot(Ng,ray.dir);
             const ssef absDen = abs(den);
             const ssef sgnDen = signmsk(den);
             
             /* test against edge p2 p0 */
             const ssef U = dot(R,e2) ^ sgnDen;
             valid &= U >= 0.0f;
             if (likely(none(valid))) continue;
             
             /* test against edge p0 p1 */
             const ssef V = dot(R,e1) ^ sgnDen;
             valid &= V >= 0.0f;
             if (likely(none(valid))) continue;
             
             /* test against edge p1 p2 */
             const ssef W = absDen-U-V;
             valid &= W >= 0.0f;
             if (likely(none(valid))) continue;
             
             /* perform depth test */
             const ssef T = dot(Ng,C) ^ sgnDen;
             valid &= (T >= absDen*ray.tnear) & (absDen*ray.tfar >= T);
             if (unlikely(none(valid))) continue;
             
             /* perform backface culling */
#if defined(RTCORE_BACKFACE_CULLING)
             valid &= den > ssef(zero);
             if (unlikely(none(valid))) continue;
#else
             valid &= den != ssef(zero);
             if (unlikely(none(valid))) continue;
#endif
             
             /* ray masking test */
#if defined(RTCORE_RAY_MASK)
             valid &= (tri.mask[i] & ray.mask) != 0;
             if (unlikely(none(valid))) continue;
#endif
             
             /* intersection filter test */
#if defined(RTCORE_INTERSECTION_FILTER)
             if (enableIntersectionFilter) 
             {
               const int geomID = tri.geomID(i);
               Geometry* geometry = scene->get(geomID);
               if (unlikely(geometry->hasOcclusionFilter4()))
               {
                 /* calculate hit information */
                 const ssef rcpAbsDen = rcp(absDen);
                 const ssef u = U*rcpAbsDen;
                 const ssef v = V*rcpAbsDen;
                 const ssef t = T*rcpAbsDen;
                 const int primID = tri.primID(i);
                 valid = runOcclusionFilter4(valid,geometry,ray,u,v,t,Ng,geomID,primID);
               }
             }
#endif
             
             /* update occlusion */
             valid0 &= !valid;
             if (none(valid0)) break;
           }
           return !valid0;
         }
         
         /*! Intersect a ray with the 4 triangles and updates the hit. */
         static __forceinline void intersect(Precalculations& pre, Ray4& ray, size_t k, const TriangleN& tri, Scene* scene)
         {
           /* calculate denominator */
           STAT3(normal.trav_prims,1,1,1);
           const sse3f O = broadcast4f(ray.org,k);
           const sse3f D = broadcast4f(ray.dir,k);
           const sse3f C = sse3f(tri.v0) - O;
           const sse3f R = cross(D,C);
           const ssef den = dot(sse3f(tri.Ng),D);
           const ssef absDen = abs(den);
           const ssef sgnDen = signmsk(den);
           
           /* perform edge tests */
           const ssef U = dot(R,sse3f(tri.e2)) ^ sgnDen;
           const ssef V = dot(R,sse3f(tri.e1)) ^ sgnDen;
           
           /* perform backface culling */
#if defined(RTCORE_BACKFACE_CULLING)
           sseb valid = (den > ssef(zero)) & (U >= 0.0f) & (V >= 0.0f) & (U+V<=absDen);
#else
           sseb valid = (den != ssef(zero)) & (U >= 0.0f) & (V >= 0.0f) & (U+V<=absDen);
#endif
           if (likely(none(valid))) return;
           
           /* perform depth test */
           const ssef T = dot(sse3f(tri.Ng),C) ^ sgnDen;
           valid &= (T > absDen*ssef(ray.tnear[k])) & (T < absDen*ssef(ray.tfar[k]));
           if (likely(none(valid))) return;
           
           /* ray masking test */
#if defined(RTCORE_RAY_MASK)
           valid &= (tri.mask & ray.mask[k]) != 0;
           if (unlikely(none(valid))) return;
#endif
           
           /* calculate hit information */
           const ssef rcpAbsDen = rcp(absDen);
           const ssef u = U * rcpAbsDen;
           const ssef v = V * rcpAbsDen;
           const ssef t = T * rcpAbsDen;
           size_t i = select_min(valid,t);
           int geomID = tri.geomID(i);
           
           /* intersection filter test */
#if defined(RTCORE_INTERSECTION_FILTER)
           while (true) 
           {
             Geometry* geometry = scene->get(geomID);
             if (likely(!enableIntersectionFilter || !geometry->hasIntersectionFilter4())) 
             {
#endif
               /* update hit information */
               ray.u[k] = u[i];
               ray.v[k] = v[i];
               ray.tfar[k] = t[i];
               ray.Ng.x[k] = tri.Ng.x[i];
               ray.Ng.y[k] = tri.Ng.y[i];
               ray.Ng.z[k] = tri.Ng.z[i];
               ray.geomID[k] = geomID;
               ray.primID[k] = tri.primID(i);
               
#if defined(RTCORE_INTERSECTION_FILTER)
               return;
             }
             
             const Vec3fa Ng(tri.Ng.x[i],tri.Ng.y[i],tri.Ng.z[i]);
             if (runIntersectionFilter4(geometry,ray,k,u[i],v[i],t[i],Ng,geomID,tri.primID(i))) return;
             valid[i] = 0;
             if (unlikely(none(valid))) return;
             i = select_min(valid,t);
             geomID = tri.geomID(i);
           }
#endif
         }
         
         /*! Test if the ray is occluded by one of the triangles. */
         static __forceinline bool occluded(Precalculations& pre, Ray4& ray, size_t k, const TriangleN& tri, Scene* scene)
         {
           /* calculate denominator */
           STAT3(shadow.trav_prims,1,1,1);
           const sse3f O = broadcast4f(ray.org,k);
           const sse3f D = broadcast4f(ray.dir,k);
           const sse3f C = sse3f(tri.v0) - O;
           const sse3f R = cross(D,C);
           const ssef den = dot(sse3f(tri.Ng),D);
           const ssef absDen = abs(den);
           const ssef sgnDen = signmsk(den);
           
           /* perform edge tests */
           const ssef U = dot(R,sse3f(tri.e2)) ^ sgnDen;
           const ssef V = dot(R,sse3f(tri.e1)) ^ sgnDen;
           const ssef W = absDen-U-V;
           sseb valid = (U >= 0.0f) & (V >= 0.0f) & (W >= 0.0f);
           if (unlikely(none(valid))) return false;
           
           /* perform depth test */
           const ssef T = dot(sse3f(tri.Ng),C) ^ sgnDen;
           valid &= (T >= absDen*ssef(ray.tnear[k])) & (absDen*ssef(ray.tfar[k]) >= T);
           if (unlikely(none(valid))) return false;
           
           /* perform backface culling */
#if defined(RTCORE_BACKFACE_CULLING)
           valid &= den > ssef(zero);
           if (unlikely(none(valid))) return false;
#else
           valid &= den != ssef(zero);
           if (unlikely(none(valid))) return false;
#endif
           
           /* ray masking test */
#if defined(RTCORE_RAY_MASK)
           valid &= (tri.mask & ray.mask[k]) != 0;
           if (unlikely(none(valid))) return false;
#endif
           
           /* intersection filter test */
#if defined(RTCORE_INTERSECTION_FILTER)
           
           size_t i = select_min(valid,T);
           int geomID = tri.geomID(i);
           
           while (true) 
           {
             Geometry* geometry = scene->get(geomID);
             if (likely(!enableIntersectionFilter || !geometry->hasOcclusionFilter4())) break;
             
             /* calculate hit information */
             const ssef rcpAbsDen = rcp(absDen);
             const ssef u = U * rcpAbsDen;
             const ssef v = V * rcpAbsDen;
             const ssef t = T * rcpAbsDen;
             const Vec3fa Ng(tri.Ng.x[i],tri.Ng.y[i],tri.Ng.z[i]);
             if (runOcclusionFilter4(geometry,ray,k,u[i],v[i],t[i],Ng,geomID,tri.primID(i))) break;
             valid[i] = 0;
             if (unlikely(none(valid))) return false;
             i = select_min(valid,T);
             geomID = tri.geomID(i);
           }
#endif
           
           return true;
         }
       };
  }
}
