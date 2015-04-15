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
         typedef TriangleN Primitive;

         /* type shortcuts */
         typedef typename TriangleN::simdb tsimdb;
         typedef typename TriangleN::simdf tsimdf;
         typedef Vec3<tsimdf> tsimd3f;

         struct Precalculations {
          __forceinline Precalculations (const Ray& ray, const void* ptr) {}
         };
         
         /*! Intersect a ray with the 4 triangles and updates the hit. */
         static __forceinline void intersect(const Precalculations& pre, Ray& ray, const TriangleN& tri, Scene* scene)
         {
           /* calculate denominator */
           STAT3(normal.trav_prims,1,1,1);
           const tsimd3f O = tsimd3f(ray.org);
           const tsimd3f D = tsimd3f(ray.dir);
           const tsimd3f C = tsimd3f(tri.v0) - O;
           const tsimd3f R = cross(D,C);
           const tsimdf den = dot(tsimd3f(tri.Ng),D);
           const tsimdf absDen = abs(den);
           const tsimdf sgnDen = signmsk(den);
           
           /* perform edge tests */
           const tsimdf U = dot(R,tsimd3f(tri.e2)) ^ sgnDen;
           const tsimdf V = dot(R,tsimd3f(tri.e1)) ^ sgnDen;
           
           /* perform backface culling */
#if defined(RTCORE_BACKFACE_CULLING)
           tsimdb valid = (den > tsimdf(zero)) & (U >= 0.0f) & (V >= 0.0f) & (U+V<=absDen);
#else
           tsimdb valid = (den != tsimdf(zero)) & (U >= 0.0f) & (V >= 0.0f) & (U+V<=absDen);
#endif
           if (likely(none(valid))) return;
           
           /* perform depth test */
           const tsimdf T = dot(tsimd3f(tri.Ng),C) ^ sgnDen;
           valid &= (T > absDen*tsimdf(ray.tnear)) & (T < absDen*tsimdf(ray.tfar));
           if (likely(none(valid))) return;
           
           /* ray masking test */
#if defined(RTCORE_RAY_MASK)
           valid &= (tri.mask & ray.mask) != 0;
           if (unlikely(none(valid))) return;
#endif
           
           /* calculate hit information */
           const tsimdf rcpAbsDen = rcp(absDen);
           const tsimdf u = U * rcpAbsDen;
           const tsimdf v = V * rcpAbsDen;
           const tsimdf t = T * rcpAbsDen;
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
           const tsimd3f O = tsimd3f(ray.org);
           const tsimd3f D = tsimd3f(ray.dir);
           const tsimd3f C = tsimd3f(tri.v0) - O;
           const tsimd3f R = cross(D,C);
           const tsimdf den = dot(tsimd3f(tri.Ng),D);
           const tsimdf absDen = abs(den);
           const tsimdf sgnDen = signmsk(den);
           
           /* perform edge tests */
           const tsimdf U = dot(R,tsimd3f(tri.e2)) ^ sgnDen;
           const tsimdf V = dot(R,tsimd3f(tri.e1)) ^ sgnDen;
           const tsimdf W = absDen-U-V;
           tsimdb valid = (U >= 0.0f) & (V >= 0.0f) & (W >= 0.0f);
           if (unlikely(none(valid))) return false;
           
           /* perform depth test */
           const tsimdf T = dot(tsimd3f(tri.Ng),C) ^ sgnDen;
           valid &= (den != tsimdf(zero)) & (T >= absDen*tsimdf(ray.tnear)) & (absDen*tsimdf(ray.tfar) >= T);
           if (unlikely(none(valid))) return false;
           
           /* perform backface culling */
#if defined(RTCORE_BACKFACE_CULLING)
           valid &= den > tsimdf(zero);
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
             
             /* if we have no filter then the test patsimds */
             if (likely(!geometry->hasOcclusionFilter1()))
               break;
             
             /* calculate hit information */
             const tsimdf rcpAbsDen = rcp(absDen);
             const tsimdf u = U * rcpAbsDen;
             const tsimdf v = V * rcpAbsDen;
             const tsimdf t = T * rcpAbsDen;
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

     /*! Intersector for N triangles with M rays. This intersector
     *  implements a modified version of the Moeller Trumbore
     *  intersector from the paper "Fast, Minimum Storage Ray-Triangle
     *  Intersection". In contrast to the paper we precalculate some
     *  factors and factor the calculations differently to allow
     *  precalculating the cross product e1 x e2. */
     template<typename RayM, typename TriangleN, bool enableIntersectionFilter>
       struct TriangleNIntersectorMMoellerTrumbore
       {
         typedef TriangleN Primitive;

         /* triangle SIMD type shortcuts */
         typedef typename TriangleN::simdb tsimdb;
         typedef typename TriangleN::simdf tsimdf;
         typedef Vec3<tsimdf> tsimd3f;

         /* ray SIMD type shortcuts */
         typedef typename RayM::simdb rsimdb;
         typedef typename RayM::simdf rsimdf;
         typedef typename RayM::simdi rsimdi;
         typedef Vec3<rsimdf> rsimd3f;

         struct Precalculations {
           __forceinline Precalculations (const rsimdb& valid, const RayM& ray) {}
         };
         
         /*! Intersects a 4 rays with 4 triangles. */
         static __forceinline void intersect(const rsimdb& valid_i, Precalculations& pre, RayM& ray, const TriangleN& tri, Scene* scene)
         {
           for (size_t i=0; i<TriangleN::max_size(); i++)
           {
             if (!tri.valid(i)) break;
             STAT3(normal.trav_prims,1,popcnt(valid_i),RayM::size());
             
             /* load edges and geometry normal */
             rsimdb valid = valid_i;
             const rsimd3f p0 = broadcast<rsimdf>(tri.v0,i);
             const rsimd3f e1 = broadcast<rsimdf>(tri.e1,i);
             const rsimd3f e2 = broadcast<rsimdf>(tri.e2,i);
             const rsimd3f Ng = broadcast<rsimdf>(tri.Ng,i);
             
             /* calculate denominator */
             const rsimd3f C = p0 - ray.org;
             const rsimd3f R = cross(ray.dir,C);
             const rsimdf den = dot(Ng,ray.dir);
             const rsimdf absDen = abs(den);
             const rsimdf sgnDen = signmsk(den);
             
             /* test against edge p2 p0 */
             const rsimdf U = dot(R,e2) ^ sgnDen;
             valid &= U >= 0.0f;
             if (likely(none(valid))) continue;
             
             /* test against edge p0 p1 */
             const rsimdf V = dot(R,e1) ^ sgnDen;
             valid &= V >= 0.0f;
             if (likely(none(valid))) continue;
             
             /* test against edge p1 p2 */
             const rsimdf W = absDen-U-V;
             valid &= W >= 0.0f;
             if (likely(none(valid))) continue;
             
             /* perform depth test */
             const rsimdf T = dot(Ng,C) ^ sgnDen;
             valid &= (T >= absDen*ray.tnear) & (absDen*ray.tfar >= T);
             if (unlikely(none(valid))) continue;
             
             /* perform backface culling */
#if defined(RTCORE_BACKFACE_CULLING)
             valid &= den > rsimdf(zero);
             if (unlikely(none(valid))) continue;
#else
             valid &= den != rsimdf(zero);
             if (unlikely(none(valid))) continue;
#endif
             
             /* ray masking test */
#if defined(RTCORE_RAY_MASK)
             valid &= (tri.mask[i] & ray.mask) != 0;
             if (unlikely(none(valid))) continue;
#endif
             
             /* calculate hit information */
             const rsimdf rcpAbsDen = rcp(absDen);
             const rsimdf u = U*rcpAbsDen;
             const rsimdf v = V*rcpAbsDen;
             const rsimdf t = T*rcpAbsDen;
             const int geomID = tri.geomID(i);
             const int primID = tri.primID(i);
             
             /* intersection filter test */
#if defined(RTCORE_INTERSECTION_FILTER)
             if (enableIntersectionFilter) {
               Geometry* geometry = scene->get(geomID);
               if (unlikely(geometry->hasIntersectionFilter<rsimdf>())) {
                 runIntersectionFilter(valid,geometry,ray,u,v,t,Ng,geomID,primID);
                 continue;
               }
             }
#endif
             
             /* update hit information */
             rsimdf::store(valid,&ray.u,u);
             rsimdf::store(valid,&ray.v,v);
             rsimdf::store(valid,&ray.tfar,t);
             rsimdi::store(valid,&ray.geomID,geomID);
             rsimdi::store(valid,&ray.primID,primID);
             rsimdf::store(valid,&ray.Ng.x,Ng.x);
             rsimdf::store(valid,&ray.Ng.y,Ng.y);
             rsimdf::store(valid,&ray.Ng.z,Ng.z);
           }
         }
         
         /*! Test for 4 rays if they are occluded by any of the 4 triangle. */
         static __forceinline rsimdb occluded(const rsimdb& valid_i, Precalculations& pre, RayM& ray, const TriangleN& tri, Scene* scene)
         {
           rsimdb valid0 = valid_i;
           
           for (size_t i=0; i<TriangleN::max_size(); i++)
           {
             if (!tri.valid(i)) break;
             STAT3(shadow.trav_prims,1,popcnt(valid0),RayM::size());
             
             /* load edges and geometry normal */
             rsimdb valid = valid0;
             const rsimd3f p0 = broadcast<rsimdf>(tri.v0,i);
             const rsimd3f e1 = broadcast<rsimdf>(tri.e1,i);
             const rsimd3f e2 = broadcast<rsimdf>(tri.e2,i);
             const rsimd3f Ng = broadcast<rsimdf>(tri.Ng,i);
             
             /* calculate denominator */
             const rsimd3f C = p0 - ray.org;
             const rsimd3f R = cross(ray.dir,C);
             const rsimdf den = dot(Ng,ray.dir);
             const rsimdf absDen = abs(den);
             const rsimdf sgnDen = signmsk(den);
             
             /* test against edge p2 p0 */
             const rsimdf U = dot(R,e2) ^ sgnDen;
             valid &= U >= 0.0f;
             if (likely(none(valid))) continue;
             
             /* test against edge p0 p1 */
             const rsimdf V = dot(R,e1) ^ sgnDen;
             valid &= V >= 0.0f;
             if (likely(none(valid))) continue;
             
             /* test against edge p1 p2 */
             const rsimdf W = absDen-U-V;
             valid &= W >= 0.0f;
             if (likely(none(valid))) continue;
             
             /* perform depth test */
             const rsimdf T = dot(Ng,C) ^ sgnDen;
             valid &= (T >= absDen*ray.tnear) & (absDen*ray.tfar >= T);
             if (unlikely(none(valid))) continue;
             
             /* perform backface culling */
#if defined(RTCORE_BACKFACE_CULLING)
             valid &= den > rsimdf(zero);
             if (unlikely(none(valid))) continue;
#else
             valid &= den != rsimdf(zero);
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
               if (unlikely(geometry->hasOcclusionFilter<rsimdf>()))
               {
                 /* calculate hit information */
                 const rsimdf rcpAbsDen = rcp(absDen);
                 const rsimdf u = U*rcpAbsDen;
                 const rsimdf v = V*rcpAbsDen;
                 const rsimdf t = T*rcpAbsDen;
                 const int primID = tri.primID(i);
                 valid = runOcclusionFilter(valid,geometry,ray,u,v,t,Ng,geomID,primID);
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
         static __forceinline void intersect(Precalculations& pre, RayM& ray, size_t k, const TriangleN& tri, Scene* scene)
         {
           /* calculate denominator */
           STAT3(normal.trav_prims,1,1,1);
           const tsimd3f O = broadcast<tsimdf>(ray.org,k);
           const tsimd3f D = broadcast<tsimdf>(ray.dir,k);
           const tsimd3f C = tsimd3f(tri.v0) - O;
           const tsimd3f R = cross(D,C);
           const tsimdf den = dot(tsimd3f(tri.Ng),D);
           const tsimdf absDen = abs(den);
           const tsimdf sgnDen = signmsk(den);
           
           /* perform edge tests */
           const tsimdf U = dot(R,tsimd3f(tri.e2)) ^ sgnDen;
           const tsimdf V = dot(R,tsimd3f(tri.e1)) ^ sgnDen;
           
           /* perform backface culling */
#if defined(RTCORE_BACKFACE_CULLING)
           tsimdb valid = (den > tsimdf(zero)) & (U >= 0.0f) & (V >= 0.0f) & (U+V<=absDen);
#else
           tsimdb valid = (den != tsimdf(zero)) & (U >= 0.0f) & (V >= 0.0f) & (U+V<=absDen);
#endif
           if (likely(none(valid))) return;
           
           /* perform depth test */
           const tsimdf T = dot(tsimd3f(tri.Ng),C) ^ sgnDen;
           valid &= (T > absDen*tsimdf(ray.tnear[k])) & (T < absDen*tsimdf(ray.tfar[k]));
           if (likely(none(valid))) return;
           
           /* ray masking test */
#if defined(RTCORE_RAY_MASK)
           valid &= (tri.mask & ray.mask[k]) != 0;
           if (unlikely(none(valid))) return;
#endif
           
           /* calculate hit information */
           const tsimdf rcpAbsDen = rcp(absDen);
           const tsimdf u = U * rcpAbsDen;
           const tsimdf v = V * rcpAbsDen;
           const tsimdf t = T * rcpAbsDen;
           size_t i = select_min(valid,t);
           int geomID = tri.geomID(i);
           
           /* intersection filter test */
#if defined(RTCORE_INTERSECTION_FILTER)
           while (true) 
           {
             Geometry* geometry = scene->get(geomID);
             if (likely(!enableIntersectionFilter || !geometry->hasIntersectionFilter<rsimdf>())) 
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
             if (runIntersectionFilter(geometry,ray,k,u[i],v[i],t[i],Ng,geomID,tri.primID(i))) return;
             valid[i] = 0;
             if (unlikely(none(valid))) return;
             i = select_min(valid,t);
             geomID = tri.geomID(i);
           }
#endif
         }
         
         /*! Test if the ray is occluded by one of the triangles. */
         static __forceinline bool occluded(Precalculations& pre, RayM& ray, size_t k, const TriangleN& tri, Scene* scene)
         {
           /* calculate denominator */
           STAT3(shadow.trav_prims,1,1,1);
           const tsimd3f O = broadcast<tsimdf>(ray.org,k);
           const tsimd3f D = broadcast<tsimdf>(ray.dir,k);
           const tsimd3f C = tsimd3f(tri.v0) - O;
           const tsimd3f R = cross(D,C);
           const tsimdf den = dot(tsimd3f(tri.Ng),D);
           const tsimdf absDen = abs(den);
           const tsimdf sgnDen = signmsk(den);
           
           /* perform edge tests */
           const tsimdf U = dot(R,tsimd3f(tri.e2)) ^ sgnDen;
           const tsimdf V = dot(R,tsimd3f(tri.e1)) ^ sgnDen;
           const tsimdf W = absDen-U-V;
           tsimdb valid = (U >= 0.0f) & (V >= 0.0f) & (W >= 0.0f);
           if (unlikely(none(valid))) return false;
           
           /* perform depth test */
           const tsimdf T = dot(tsimd3f(tri.Ng),C) ^ sgnDen;
           valid &= (T >= absDen*tsimdf(ray.tnear[k])) & (absDen*tsimdf(ray.tfar[k]) >= T);
           if (unlikely(none(valid))) return false;
           
           /* perform backface culling */
#if defined(RTCORE_BACKFACE_CULLING)
           valid &= den > tsimdf(zero);
           if (unlikely(none(valid))) return false;
#else
           valid &= den != tsimdf(zero);
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
             if (likely(!enableIntersectionFilter || !geometry->hasOcclusionFilter<rsimdf>())) break;
             
             /* calculate hit information */
             const tsimdf rcpAbsDen = rcp(absDen);
             const tsimdf u = U * rcpAbsDen;
             const tsimdf v = V * rcpAbsDen;
             const tsimdf t = T * rcpAbsDen;
             const Vec3fa Ng(tri.Ng.x[i],tri.Ng.y[i],tri.Ng.z[i]);
             if (runOcclusionFilter(geometry,ray,k,u[i],v[i],t[i],Ng,geomID,tri.primID(i))) break;
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
