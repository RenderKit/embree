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

#include "../../common/ray.h"
#include "filter.h"

/*! This intersector implements a modified version of the Moeller
 *  Trumbore intersector from the paper "Fast, Minimum Storage
 *  Ray-Triangle Intersection". In contrast to the paper we
 *  precalculate some factors and factor the calculations differently
 *  to allow precalculating the cross product e1 x e2. The resulting
 *  algorithm is similar to the fastest one of the paper "Optimizing
 *  Ray-Triangle Intersection via Automated Search". */

namespace embree
{
  namespace isa
  {
    /*! Intersect a ray with the N triangles and updates the hit. */
    template<bool enableIntersectionFilter, typename tsimdb, typename tsimdf, typename tsimdi>
      __forceinline void triangle_intersect_moeller_trumbore(Ray& ray, 
                                                             const Vec3<tsimdf>& tri_v0, 
							     const Vec3<tsimdf>& tri_e1, 
							     const Vec3<tsimdf>& tri_e2, 
							     const Vec3<tsimdf>& tri_Ng, 
                                                             const tsimdi& tri_geomIDs, 
							     const tsimdi& tri_primIDs, 
							     Scene* scene)
    {
      /* calculate denominator */
      typedef Vec3<tsimdf> tsimd3f;
      const tsimd3f O = tsimd3f(ray.org);
      const tsimd3f D = tsimd3f(ray.dir);
      const tsimd3f C = tsimd3f(tri_v0) - O;
      const tsimd3f R = cross(D,C);
      const tsimdf den = dot(tsimd3f(tri_Ng),D);
      const tsimdf absDen = abs(den);
      const tsimdf sgnDen = signmsk(den);
      
      /* perform edge tests */
      const tsimdf U = dot(R,tsimd3f(tri_e2)) ^ sgnDen;
      const tsimdf V = dot(R,tsimd3f(tri_e1)) ^ sgnDen;
      
      /* perform backface culling */
#if defined(RTCORE_BACKFACE_CULLING)
      tsimdb valid = (den > tsimdf(zero)) & (U >= 0.0f) & (V >= 0.0f) & (U+V<=absDen);
#else
      tsimdb valid = (den != tsimdf(zero)) & (U >= 0.0f) & (V >= 0.0f) & (U+V<=absDen);
#endif
      if (likely(none(valid))) return;
      
      /* perform depth test */
      const tsimdf T = dot(tsimd3f(tri_Ng),C) ^ sgnDen;
      valid &= (T > absDen*tsimdf(ray.tnear)) & (T < absDen*tsimdf(ray.tfar));
      if (likely(none(valid))) return;
      
      /* calculate hit information */
      const tsimdf rcpAbsDen = rcp(absDen);
      const tsimdf u = U * rcpAbsDen;
      const tsimdf v = V * rcpAbsDen;
      const tsimdf t = T * rcpAbsDen;
      size_t i = select_min(valid,t);
      int geomID = tri_geomIDs[i];
      
      /* intersection filter test */
#if defined(RTCORE_INTERSECTION_FILTER) || defined(RTCORE_RAY_MASK)
      goto entry;
      while (true) 
      {
        if (unlikely(none(valid))) return;
        i = select_min(valid,t);
        geomID = tri_geomIDs[i];
      entry:
        Geometry* geometry = scene->get(geomID);
        
#if defined(RTCORE_RAY_MASK)
        /* goto next hit if mask test fails */
        if ((geometry->mask & ray.mask) == 0) {
          valid[i] = 0;
          continue;
        }
#endif
        
#if defined(RTCORE_INTERSECTION_FILTER) 
        /* call intersection filter function */
        if (enableIntersectionFilter) {
          if (unlikely(geometry->hasIntersectionFilter1())) {
            Vec3fa Ng = Vec3fa(tri_Ng.x[i],tri_Ng.y[i],tri_Ng.z[i]);
            if (runIntersectionFilter1(geometry,ray,u[i],v[i],t[i],Ng,geomID,tri_primIDs[i])) return;
            valid[i] = 0;
            continue;
          }
        }
#endif
        break;
      }
#endif
      
      /* update hit information */
      ray.u = u[i];
      ray.v = v[i];
      ray.tfar = t[i];
      ray.Ng.x = tri_Ng.x[i];
      ray.Ng.y = tri_Ng.y[i];
      ray.Ng.z = tri_Ng.z[i];
      ray.geomID = geomID;
      ray.primID = tri_primIDs[i];
    }
    
    template<bool enableIntersectionFilter, typename tsimdb, typename tsimdf, typename tsimdi>
      __forceinline void triangle_intersect_moeller_trumbore(Ray& ray, 
                                                             const Vec3<tsimdf>& v0, 
							     const Vec3<tsimdf>& v1, 
							     const Vec3<tsimdf>& v2,
                                                             const tsimdi& tri_geomIDs, 
							     const tsimdi& tri_primIDs, 
							     Scene* scene)
    {
      typedef Vec3<tsimdf> tsimd3f;
      const tsimd3f e1 = v0-v1;
      const tsimd3f e2 = v2-v0;
      const tsimd3f Ng = cross(e1,e2);
      triangle_intersect_moeller_trumbore<enableIntersectionFilter,tsimdb,tsimdf,tsimdi>(ray,v0,e1,e2,Ng,tri_geomIDs,tri_primIDs,scene);
    }
    
    /*! Test if the ray is occluded by one of N triangles. */
    template<bool enableIntersectionFilter, typename tsimdb, typename tsimdf, typename tsimdi>
      __forceinline bool triangle_occluded_moeller_trumbore(Ray& ray, 
                                                            const Vec3<tsimdf>& tri_v0, 
							    const Vec3<tsimdf>& tri_e1, 
							    const Vec3<tsimdf>& tri_e2, 
							    const Vec3<tsimdf>& tri_Ng, 
                                                            const tsimdi& tri_geomIDs, const tsimdi& tri_primIDs, Scene* scene)
    {
      /* calculate denominator */
      typedef Vec3<tsimdf> tsimd3f;
      const tsimd3f O = tsimd3f(ray.org);
      const tsimd3f D = tsimd3f(ray.dir);
      const tsimd3f C = tsimd3f(tri_v0) - O;
      const tsimd3f R = cross(D,C);
      const tsimdf den = dot(tsimd3f(tri_Ng),D);
      const tsimdf absDen = abs(den);
      const tsimdf sgnDen = signmsk(den);
      
      /* perform edge tests */
      const tsimdf U = dot(R,tsimd3f(tri_e2)) ^ sgnDen;
      const tsimdf V = dot(R,tsimd3f(tri_e1)) ^ sgnDen;
      const tsimdf W = absDen-U-V;
      tsimdb valid = (U >= 0.0f) & (V >= 0.0f) & (W >= 0.0f);
      if (unlikely(none(valid))) return false;
      
      /* perform depth test */
      const tsimdf T = dot(tsimd3f(tri_Ng),C) ^ sgnDen;
      valid &= (den != tsimdf(zero)) & (T >= absDen*tsimdf(ray.tnear)) & (absDen*tsimdf(ray.tfar) >= T);
      if (unlikely(none(valid))) return false;
      
      /* perform backface culling */
#if defined(RTCORE_BACKFACE_CULLING)
      valid &= den > tsimdf(zero);
      if (unlikely(none(valid))) return false;
#endif
      
      /* intersection filter test */
#if defined(RTCORE_INTERSECTION_FILTER) || defined(RTCORE_RAY_MASK)
      size_t m=movemask(valid);
      goto entry;
      while (true)
      {  
        if (unlikely(m == 0)) return false;
      entry:
        size_t i=__bsf(m);
        const int geomID = tri_geomIDs[i];
        Geometry* geometry = scene->get(geomID);
        
#if defined(RTCORE_RAY_MASK)
        /* goto next hit if mask test fails */
        if ((geometry->mask & ray.mask) == 0) {
          m=__btc(m,i);
          continue;
        }
#endif
        
#if defined(RTCORE_INTERSECTION_FILTER)
        /* if we have no filter then the test passed */
        if (enableIntersectionFilter) {
          if (unlikely(geometry->hasOcclusionFilter1())) 
          {
            /* calculate hit information */
            const tsimdf rcpAbsDen = rcp(absDen);
            const tsimdf u = U * rcpAbsDen;
            const tsimdf v = V * rcpAbsDen;
            const tsimdf t = T * rcpAbsDen;
            const Vec3fa Ng = Vec3fa(tri_Ng.x[i],tri_Ng.y[i],tri_Ng.z[i]);
            if (runOcclusionFilter1(geometry,ray,u[i],v[i],t[i],Ng,geomID,tri_primIDs[i])) return true;
            m=__btc(m,i);
            continue;
          }
        }
#endif
        break;
      }
#endif
      
      return true;
    }
    
    template<bool enableIntersectionFilter, typename tsimdb, typename tsimdf, typename tsimdi>
      __forceinline bool triangle_occluded_moeller_trumbore(Ray& ray, 
                                                            const Vec3<tsimdf>& v0, 
							    const Vec3<tsimdf>& v1, 
							    const Vec3<tsimdf>& v2,
                                                            const tsimdi& tri_geomIDs, 
							    const tsimdi& tri_primIDs, 
							    Scene* scene)
    {
      typedef Vec3<tsimdf> tsimd3f;
      const tsimd3f e1 = v0-v1;
      const tsimd3f e2 = v2-v0;
      const tsimd3f Ng = cross(e1,e2);
      return triangle_occluded_moeller_trumbore<enableIntersectionFilter,tsimdb,tsimdf,tsimdi>(ray,v0,e1,e2,Ng,tri_geomIDs,tri_primIDs,scene);
    }
    
    /*! Intersects a M rays with M triangles. */
    template<bool enableIntersectionFilter, typename tsimdf, typename tsimdi, typename RayM>
      __forceinline void triangle_intersect_moeller_trumbore(const typename RayM::simdb& valid0, 
							     RayM& ray, 
                                                             const Vec3<tsimdf>& tri_v0, 
							     const Vec3<tsimdf>& tri_e1, 
							     const Vec3<tsimdf>& tri_e2, 
							     const Vec3<tsimdf>& tri_Ng, 
                                                             const tsimdi& tri_geomIDs, 
							     const tsimdi& tri_primIDs, 
							     const size_t i, Scene* scene)
    {
      /* ray SIMD type shortcuts */
      typedef typename RayM::simdb rsimdb;
      typedef typename RayM::simdf rsimdf;
      typedef typename RayM::simdi rsimdi;
      typedef Vec3<rsimdf> rsimd3f;
      
      /* calculate denominator */
      rsimdb valid = valid0;
      const rsimd3f C = tri_v0 - ray.org;
      const rsimd3f R = cross(ray.dir,C);
      const rsimdf den = dot(tri_Ng,ray.dir);
      const rsimdf absDen = abs(den);
      const rsimdf sgnDen = signmsk(den);
      
      /* test against edge p2 p0 */
      const rsimdf U = dot(R,tri_e2) ^ sgnDen;
      valid &= U >= 0.0f;
      if (likely(none(valid))) return;
      
      /* test against edge p0 p1 */
      const rsimdf V = dot(R,tri_e1) ^ sgnDen;
      valid &= V >= 0.0f;
      if (likely(none(valid))) return;
      
      /* test against edge p1 p2 */
      const rsimdf W = absDen-U-V;
      valid &= W >= 0.0f;
      if (likely(none(valid))) return;
      
      /* perform depth test */
      const rsimdf T = dot(tri_Ng,C) ^ sgnDen;
      valid &= (T >= absDen*ray.tnear) & (absDen*ray.tfar >= T);
      if (unlikely(none(valid))) return;
      
      /* perform backface culling */
#if defined(RTCORE_BACKFACE_CULLING)
      valid &= den > rsimdf(zero);
      if (unlikely(none(valid))) return;
#else
      valid &= den != rsimdf(zero);
      if (unlikely(none(valid))) return;
#endif
      
      /* calculate hit information */
      const rsimdf rcpAbsDen = rcp(absDen);
      const rsimdf u = U*rcpAbsDen;
      const rsimdf v = V*rcpAbsDen;
      const rsimdf t = T*rcpAbsDen;
      const int geomID = tri_geomIDs[i];
      const int primID = tri_primIDs[i];
      Geometry* geometry = scene->get(geomID);
      
      /* ray masking test */
#if defined(RTCORE_RAY_MASK)
      valid &= (geometry->mask & ray.mask) != 0;
      if (unlikely(none(valid))) return;
#endif
      
      /* occlusion filter test */
#if defined(RTCORE_INTERSECTION_FILTER)
      if (enableIntersectionFilter) {
        if (unlikely(geometry->hasIntersectionFilter<rsimdf>())) {
          runIntersectionFilter(valid,geometry,ray,u,v,t,tri_Ng,geomID,primID);
          return;
        }
      }
#endif
      
      /* update hit information */
      rsimdf::store(valid,&ray.u,u);
      rsimdf::store(valid,&ray.v,v);
      rsimdf::store(valid,&ray.tfar,t);
      rsimdi::store(valid,&ray.geomID,geomID);
      rsimdi::store(valid,&ray.primID,primID);
      rsimdf::store(valid,&ray.Ng.x,tri_Ng.x);
      rsimdf::store(valid,&ray.Ng.y,tri_Ng.y);
      rsimdf::store(valid,&ray.Ng.z,tri_Ng.z);
    }
    
    template<bool enableIntersectionFilter, typename tsimdf, typename tsimdi, typename RayM>
      __forceinline void triangle_intersect_moeller_trumbore(const typename RayM::simdb& valid0, 
							     RayM& ray, 
                                                             const Vec3<tsimdf>& v0, 
							     const Vec3<tsimdf>& v1, 
							     const Vec3<tsimdf>& v2,
                                                             const tsimdi& tri_geomIDs, 
							     const tsimdi& tri_primIDs, 
							     const size_t i, Scene* scene)
    {
      typedef Vec3<tsimdf> tsimd3f;
      const tsimd3f e1 = v0-v1;
      const tsimd3f e2 = v2-v0;
      const tsimd3f Ng = cross(e1,e2);
      triangle_intersect_moeller_trumbore<enableIntersectionFilter>(valid0,ray,v0,e1,e2,Ng,tri_geomIDs,tri_primIDs,i,scene);
    }
    
    /*! Test for M rays if they are occluded by any of the N triangle. */
    template<bool enableIntersectionFilter, typename tsimdf, typename tsimdi, typename RayM>
      __forceinline void triangle_occluded_moeller_trumbore(typename RayM::simdb& valid0, RayM& ray, 
                                                            const Vec3<tsimdf>& tri_v0, const Vec3<tsimdf>& tri_e1, const Vec3<tsimdf>& tri_e2, const Vec3<tsimdf>& tri_Ng, 
                                                            const tsimdi& tri_geomIDs, const tsimdi& tri_primIDs, const size_t i, Scene* scene)
    {
      /* ray SIMD type shortcuts */
      typedef typename RayM::simdb rsimdb;
      typedef typename RayM::simdf rsimdf;
      typedef typename RayM::simdi rsimdi;
      typedef Vec3<rsimdf> rsimd3f;
      
      /* calculate denominator */
      rsimdb valid = valid0;
      const rsimd3f C = tri_v0 - ray.org;
      const rsimd3f R = cross(ray.dir,C);
      const rsimdf den = dot(tri_Ng,ray.dir);
      const rsimdf absDen = abs(den);
      const rsimdf sgnDen = signmsk(den);
      
      /* test against edge p2 p0 */
      const rsimdf U = dot(R,tri_e2) ^ sgnDen;
      valid &= U >= 0.0f;
      if (likely(none(valid))) return;
      
      /* test against edge p0 p1 */
      const rsimdf V = dot(R,tri_e1) ^ sgnDen;
      valid &= V >= 0.0f;
      if (likely(none(valid))) return;
      
      /* test against edge p1 p2 */
      const rsimdf W = absDen-U-V;
      valid &= W >= 0.0f;
      if (likely(none(valid))) return;
      
      /* perform depth test */
      const rsimdf T = dot(tri_Ng,C) ^ sgnDen;
      valid &= (T >= absDen*ray.tnear) & (absDen*ray.tfar >= T);
      if (unlikely(none(valid))) return;
      
      /* perform backface culling */
#if defined(RTCORE_BACKFACE_CULLING)
      valid &= den > rsimdf(zero);
      if (unlikely(none(valid))) return;
#else
      valid &= den != rsimdf(zero);
      if (unlikely(none(valid))) return;
#endif
      
      /* ray masking test */
      const int geomID = tri_geomIDs[i];
      Geometry* geometry = scene->get(geomID);
#if defined(RTCORE_RAY_MASK)
      valid &= (geometry->mask & ray.mask) != 0;
      if (unlikely(none(valid))) return;
#endif
      
      /* intersection filter test */
#if defined(RTCORE_INTERSECTION_FILTER)
      if (enableIntersectionFilter) 
      {
        if (unlikely(geometry->hasOcclusionFilter<rsimdf>()))
        {
          const rsimdf rcpAbsDen = rcp(absDen);
          const rsimdf u = U*rcpAbsDen;
          const rsimdf v = V*rcpAbsDen;
          const rsimdf t = T*rcpAbsDen;
          const int primID = tri_primIDs[i];
          valid = runOcclusionFilter(valid,geometry,ray,u,v,t,tri_Ng,geomID,primID);
        }
      }
#endif
      
      /* update occlusion */
      valid0 &= !valid;
    }
    
    template<bool enableIntersectionFilter, typename tsimdf, typename tsimdi, typename RayM>
      __forceinline void triangle_occluded_moeller_trumbore(typename RayM::simdb& valid0, 
							    RayM& ray, 
                                                            const Vec3<tsimdf>& v0, 
							    const Vec3<tsimdf>& v1, 
							    const Vec3<tsimdf>& v2,
                                                            const tsimdi& tri_geomIDs, 
							    const tsimdi& tri_primIDs, 
							    const size_t i, Scene* scene)
    {
      typedef Vec3<tsimdf> tsimd3f;
      const tsimd3f e1 = v0-v1;
      const tsimd3f e2 = v2-v0;
      const tsimd3f Ng = cross(e1,e2);
      triangle_occluded_moeller_trumbore<enableIntersectionFilter>(valid0,ray,v0,e1,e2,Ng,tri_geomIDs,tri_primIDs,i,scene);
    }
    
    /*! Intersect a ray with the 4 triangles and updates the hit. */
    template<bool enableIntersectionFilter, typename tsimdf, typename tsimdi, typename RayM>
      __forceinline void triangle_intersect_moeller_trumbore(RayM& ray, size_t k,
                                                             const Vec3<tsimdf>& tri_v0, 
							     const Vec3<tsimdf>& tri_e1, 
							     const Vec3<tsimdf>& tri_e2, 
							     const Vec3<tsimdf>& tri_Ng, 
                                                             const tsimdi& tri_geomIDs, 
							     const tsimdi& tri_primIDs, 
							     Scene* scene)
    {
      /* type shortcuts */
      typedef typename RayM::simdf rsimdf;
      typedef typename tsimdf::Mask tsimdb;
      typedef Vec3<tsimdf> tsimd3f;
      
      /* calculate denominator */
      const tsimd3f O = broadcast<tsimdf>(ray.org,k);
      const tsimd3f D = broadcast<tsimdf>(ray.dir,k);
      const tsimd3f C = tsimd3f(tri_v0) - O;
      const tsimd3f R = cross(D,C);
      const tsimdf den = dot(tsimd3f(tri_Ng),D);
      const tsimdf absDen = abs(den);
      const tsimdf sgnDen = signmsk(den);
      
      /* perform edge tests */
      const tsimdf U = dot(R,tsimd3f(tri_e2)) ^ sgnDen;
      const tsimdf V = dot(R,tsimd3f(tri_e1)) ^ sgnDen;
      
      /* perform backface culling */
#if defined(RTCORE_BACKFACE_CULLING)
      tsimdb valid = (den > tsimdf(zero)) & (U >= 0.0f) & (V >= 0.0f) & (U+V<=absDen);
#else
      tsimdb valid = (den != tsimdf(zero)) & (U >= 0.0f) & (V >= 0.0f) & (U+V<=absDen);
#endif
      if (likely(none(valid))) return;
      
      /* perform depth test */
      const tsimdf T = dot(tsimd3f(tri_Ng),C) ^ sgnDen;
      valid &= (T > absDen*tsimdf(ray.tnear[k])) & (T < absDen*tsimdf(ray.tfar[k]));
      if (likely(none(valid))) return;
      
      /* calculate hit information */
      const tsimdf rcpAbsDen = rcp(absDen);
      const tsimdf u = U * rcpAbsDen;
      const tsimdf v = V * rcpAbsDen;
      const tsimdf t = T * rcpAbsDen;
      size_t i = select_min(valid,t);
      int geomID = tri_geomIDs[i];
      
      /* intersection filter test */
#if defined(RTCORE_INTERSECTION_FILTER) || defined(RTCORE_RAY_MASK)
      goto entry;
      while (true) 
      {
        if (unlikely(none(valid))) return;
        i = select_min(valid,t);
        geomID = tri_geomIDs[i];
      entry:
        Geometry* geometry = scene->get(geomID);
        
#if defined(RTCORE_RAY_MASK)
        /* goto next hit if mask test fails */
        if ((geometry->mask & ray.mask[k]) == 0) {
          valid[i] = 0;
          continue;
        }
#endif
        
#if defined(RTCORE_INTERSECTION_FILTER) 
        /* call intersection filter function */
        if (enableIntersectionFilter) {
          if (unlikely(geometry->hasIntersectionFilter<rsimdf>())) {
            Vec3fa Ng = Vec3fa(tri_Ng.x[i],tri_Ng.y[i],tri_Ng.z[i]);
            if (runIntersectionFilter(geometry,ray,k,u[i],v[i],t[i],Ng,geomID,tri_primIDs[i])) return;
            valid[i] = 0;
            continue;
          }
        }
#endif
        break;
      }
#endif
      
      /* update hit information */
      ray.u[k] = u[i];
      ray.v[k] = v[i];
      ray.tfar[k] = t[i];
      ray.Ng.x[k] = tri_Ng.x[i];
      ray.Ng.y[k] = tri_Ng.y[i];
      ray.Ng.z[k] = tri_Ng.z[i];
      ray.geomID[k] = geomID;
      ray.primID[k] = tri_primIDs[i];
    }
    
    template<bool enableIntersectionFilter, typename tsimdf, typename tsimdi, typename RayM>
      __forceinline void triangle_intersect_moeller_trumbore(RayM& ray, size_t k,
                                                             const Vec3<tsimdf>& v0, const Vec3<tsimdf>& v1, const Vec3<tsimdf>& v2,
                                                             const tsimdi& tri_geomIDs, const tsimdi& tri_primIDs, const size_t i, Scene* scene)
    {
      typedef Vec3<tsimdf> tsimd3f;
      const tsimd3f e1 = v0-v1;
      const tsimd3f e2 = v2-v0;
      const tsimd3f Ng = cross(e1,e2);
      triangle_intersect_moeller_trumbore<enableIntersectionFilter>(ray,k,v0,e1,e2,Ng,tri_geomIDs,tri_primIDs,i,scene);
    }
    
    
    /*! Test if the ray is occluded by one of the triangles. */
    template<bool enableIntersectionFilter, typename tsimdf, typename tsimdi, typename RayM>
      __forceinline bool triangle_occluded_moeller_trumbore(RayM& ray, size_t k, 
                                                            const Vec3<tsimdf>& tri_v0, const Vec3<tsimdf>& tri_e1, const Vec3<tsimdf>& tri_e2, const Vec3<tsimdf>& tri_Ng, 
                                                            const tsimdi& tri_geomIDs, const tsimdi& tri_primIDs, Scene* scene)
    {
      /* type shortcuts */
      typedef typename RayM::simdf rsimdf;
      typedef typename tsimdf::Mask tsimdb;
      typedef Vec3<tsimdf> tsimd3f;
      
      /* calculate denominator */
      const tsimd3f O = broadcast<tsimdf>(ray.org,k);
      const tsimd3f D = broadcast<tsimdf>(ray.dir,k);
      const tsimd3f C = tsimd3f(tri_v0) - O;
      const tsimd3f R = cross(D,C);
      const tsimdf den = dot(tsimd3f(tri_Ng),D);
      const tsimdf absDen = abs(den);
      const tsimdf sgnDen = signmsk(den);
      
      /* perform edge tests */
      const tsimdf U = dot(R,tsimd3f(tri_e2)) ^ sgnDen;
      const tsimdf V = dot(R,tsimd3f(tri_e1)) ^ sgnDen;
      const tsimdf W = absDen-U-V;
      tsimdb valid = (U >= 0.0f) & (V >= 0.0f) & (W >= 0.0f);
      if (unlikely(none(valid))) return false;
      
      /* perform depth test */
      const tsimdf T = dot(tsimd3f(tri_Ng),C) ^ sgnDen;
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
      
      /* intersection filter test */
#if defined(RTCORE_INTERSECTION_FILTER) || defined(RTCORE_RAY_MASK)
      size_t m=movemask(valid);
      goto entry;
      while (true)
      {  
        if (unlikely(m == 0)) return false;
      entry:
        size_t i=__bsf(m);
        const int geomID = tri_geomIDs[i];
        Geometry* geometry = scene->get(geomID);
        
#if defined(RTCORE_RAY_MASK)
        /* goto next hit if mask test fails */
        if ((geometry->mask & ray.mask[k]) == 0) {
          m=__btc(m,i);
          continue;
        }
#endif
        
#if defined(RTCORE_INTERSECTION_FILTER)
        /* execute occlusion filer */
        if (enableIntersectionFilter) {
          if (unlikely(geometry->hasOcclusionFilter<rsimdf>())) 
          {
            const tsimdf rcpAbsDen = rcp(absDen);
            const tsimdf u = U * rcpAbsDen;
            const tsimdf v = V * rcpAbsDen;
            const tsimdf t = T * rcpAbsDen;
            const Vec3fa Ng = Vec3fa(tri_Ng.x[i],tri_Ng.y[i],tri_Ng.z[i]);
            if (runOcclusionFilter(geometry,ray,k,u[i],v[i],t[i],Ng,geomID,tri_primIDs[i])) return true;
            m=__btc(m,i);
            continue;
          }
        }
#endif
        break;
      }
#endif
      
      return true;
    }
    
    template<bool enableIntersectionFilter, typename tsimdf, typename tsimdi, typename RayM>
      __forceinline bool triangle_occluded_moeller_trumbore(RayM& ray, size_t k,
                                                            const Vec3<tsimdf>& v0, const Vec3<tsimdf>& v1, const Vec3<tsimdf>& v2,
                                                            const tsimdi& tri_geomIDs, const tsimdi& tri_primIDs, const size_t i, Scene* scene)
    {
      typedef Vec3<tsimdf> tsimd3f;
      const tsimd3f e1 = v0-v1;
      const tsimd3f e2 = v2-v0;
      const tsimd3f Ng = cross(e1,e2);
      return triangle_occluded_moeller_trumbore<enableIntersectionFilter>(ray,k,v0,e1,e2,Ng,tri_geomIDs,tri_primIDs,i,scene);
    }
    
    /*! Intersects N triangles with 1 ray */
    template<typename TriangleN, bool enableIntersectionFilter>
      struct TriangleNIntersector1MoellerTrumbore
      {
        typedef TriangleN Primitive;
        
        /* type shortcuts */
        typedef typename TriangleN::simdb tsimdb;
        typedef typename TriangleN::simdf tsimdf;
        typedef typename TriangleN::simdi tsimdi;
        typedef Vec3<tsimdf> tsimd3f;
        
        struct Precalculations {
          __forceinline Precalculations (const Ray& ray, const void* ptr) {}
        };
        
        /*! Intersect a ray with the N triangles and updates the hit. */
        static __forceinline void intersect(const Precalculations& pre, Ray& ray, const TriangleN& tri, Scene* scene)
        {
          STAT3(normal.trav_prims,1,1,1);
          triangle_intersect_moeller_trumbore<enableIntersectionFilter,tsimdb,tsimdf,tsimdi>(ray,tri.v0,tri.e1,tri.e2,tri.Ng,tri.geomIDs,tri.primIDs,scene);
        }
        
        /*! Test if the ray is occluded by one of N triangles. */
        static __forceinline bool occluded(const Precalculations& pre, Ray& ray, const TriangleN& tri, Scene* scene)
        {
          STAT3(shadow.trav_prims,1,1,1);
          return triangle_occluded_moeller_trumbore<enableIntersectionFilter,tsimdb,tsimdf,tsimdi>(ray,tri.v0,tri.e1,tri.e2,tri.Ng,tri.geomIDs,tri.primIDs,scene);
        }
      };



    /*! Intersects N/2 triangle pairs with 1 ray */
    template<typename TrianglePairsN, bool enableIntersectionFilter>
      struct TrianglePairsNIntersector1MoellerTrumbore
      {
        typedef TrianglePairsN Primitive;
        
        /* type shortcuts */
        typedef typename TrianglePairsN::simdb tsimdb;
        typedef typename TrianglePairsN::simdf tsimdf;
        typedef typename TrianglePairsN::simdi tsimdi;
        typedef Vec3<tsimdf> tsimd3f;
        
        struct Precalculations {
          __forceinline Precalculations (const Ray& ray, const void* ptr) {}
        };
        
        /*! Intersect a ray with the N/2 triangle pairs and updates the hit. */
        static __forceinline void intersect(const Precalculations& pre, Ray& ray, const TrianglePairsN& tri, Scene* scene)
        {
          STAT3(normal.trav_prims,1,1,1);
          triangle_intersect_moeller_trumbore<enableIntersectionFilter,tsimdb,tsimdf,tsimdi>(ray,tri.v0,tri.e1,tri.e2,tri.geomIDs,tri.primIDs,scene);
        }
        
        /*! Test if the ray is occluded by one of N/2 triangle pairs. */
        static __forceinline bool occluded(const Precalculations& pre, Ray& ray, const TrianglePairsN& tri, Scene* scene)
        {
          STAT3(shadow.trav_prims,1,1,1);
          return triangle_occluded_moeller_trumbore<enableIntersectionFilter,tsimdb,tsimdf,tsimdi>(ray,tri.v0,tri.e1,tri.e2,tri.geomIDs,tri.primIDs,scene);
        }
      };

    
    /*! Intersector for N triangles with M rays. */
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
        
        /*! Intersects a M rays with N triangles. */
        static __forceinline void intersect(const rsimdb& valid_i, Precalculations& pre, RayM& ray, const TriangleN& tri, Scene* scene)
        {
          for (size_t i=0; i<TriangleN::max_size(); i++)
          {
            if (!tri.valid(i)) break;
            STAT3(normal.trav_prims,1,popcnt(valid_i),RayM::size());
            const rsimd3f p0 = broadcast<rsimdf>(tri.v0,i);
            const rsimd3f e1 = broadcast<rsimdf>(tri.e1,i);
            const rsimd3f e2 = broadcast<rsimdf>(tri.e2,i);
            const rsimd3f Ng = broadcast<rsimdf>(tri.Ng,i);
            triangle_intersect_moeller_trumbore<enableIntersectionFilter>(valid_i,ray,p0,e1,e2,Ng,tri.geomIDs,tri.primIDs,i,scene);
          }
        }
        
        /*! Test for M rays if they are occluded by any of the N triangle. */
        static __forceinline rsimdb occluded(const rsimdb& valid_i, Precalculations& pre, RayM& ray, const TriangleN& tri, Scene* scene)
        {
          rsimdb valid0 = valid_i;
          
          for (size_t i=0; i<TriangleN::max_size(); i++)
          {
            if (!tri.valid(i)) break;
            STAT3(shadow.trav_prims,1,popcnt(valid0),RayM::size());
            const rsimd3f p0 = broadcast<rsimdf>(tri.v0,i);
            const rsimd3f e1 = broadcast<rsimdf>(tri.e1,i);
            const rsimd3f e2 = broadcast<rsimdf>(tri.e2,i);
            const rsimd3f Ng = broadcast<rsimdf>(tri.Ng,i);
            triangle_occluded_moeller_trumbore<enableIntersectionFilter>(valid0,ray,p0,e1,e2,Ng,tri.geomIDs,tri.primIDs,i,scene);
            if (none(valid0)) break;
          }
          return !valid0;
        }
        
        /*! Intersect a ray with the 4 triangles and updates the hit. */
        static __forceinline void intersect(Precalculations& pre, RayM& ray, size_t k, const TriangleN& tri, Scene* scene)
        {
          STAT3(normal.trav_prims,1,1,1);
          triangle_intersect_moeller_trumbore<enableIntersectionFilter>(ray,k,tri.v0,tri.e1,tri.e2,tri.Ng,tri.geomIDs,tri.primIDs,scene);
        }
        
        /*! Test if the ray is occluded by one of the triangles. */
        static __forceinline bool occluded(Precalculations& pre, RayM& ray, size_t k, const TriangleN& tri, Scene* scene)
        {
          STAT3(shadow.trav_prims,1,1,1);
          return triangle_occluded_moeller_trumbore<enableIntersectionFilter>(ray,k,tri.v0,tri.e1,tri.e2,tri.Ng,tri.geomIDs,tri.primIDs,scene);
        }
      };
    
    /*! Intersects N triangles with 1 ray */
    template<typename TriangleNMblur, bool enableIntersectionFilter>
      struct TriangleNMblurIntersector1MoellerTrumbore
      {
        typedef TriangleNMblur Primitive;
        
        /* type shortcuts */
        typedef typename TriangleNMblur::simdb tsimdb;
        typedef typename TriangleNMblur::simdf tsimdf;
        typedef typename TriangleNMblur::simdi tsimdi;
        typedef Vec3<tsimdf> tsimd3f;
        
        struct Precalculations {
          __forceinline Precalculations (const Ray& ray, const void* ptr) {}
        };
        
        /*! Intersect a ray with the N triangles and updates the hit. */
        static __forceinline void intersect(const Precalculations& pre, Ray& ray, const TriangleNMblur& tri, Scene* scene)
        {
          STAT3(normal.trav_prims,1,1,1);
          const tsimdf time = ray.time;
          const tsimd3f v0 = tri.v0 + time*tri.dv0;
          const tsimd3f v1 = tri.v1 + time*tri.dv1;
          const tsimd3f v2 = tri.v2 + time*tri.dv2;
          triangle_intersect_moeller_trumbore<enableIntersectionFilter,tsimdb,tsimdf,tsimdi>(ray,v0,v1,v2,tri.geomIDs,tri.primIDs,scene);
        }
        
        /*! Test if the ray is occluded by one of N triangles. */
        static __forceinline bool occluded(const Precalculations& pre, Ray& ray, const TriangleNMblur& tri, Scene* scene)
        {
          STAT3(shadow.trav_prims,1,1,1);
          const tsimdf time = ray.time;
          const tsimd3f v0 = tri.v0 + time*tri.dv0;
          const tsimd3f v1 = tri.v1 + time*tri.dv1;
          const tsimd3f v2 = tri.v2 + time*tri.dv2;
          return triangle_occluded_moeller_trumbore<enableIntersectionFilter,tsimdb,tsimdf,tsimdi>(ray,v0,v1,v2,tri.geomIDs,tri.primIDs,scene);
        }
      };
    
    /*! Intersector for N triangles with M rays. */
    template<typename RayM, typename TriangleNMblur, bool enableIntersectionFilter>
      struct TriangleNMblurIntersectorMMoellerTrumbore
      {
        typedef TriangleNMblur Primitive;
        
        /* triangle SIMD type shortcuts */
        typedef typename TriangleNMblur::simdb tsimdb;
        typedef typename TriangleNMblur::simdf tsimdf;
        typedef Vec3<tsimdf> tsimd3f;
        
        /* ray SIMD type shortcuts */
        typedef typename RayM::simdb rsimdb;
        typedef typename RayM::simdf rsimdf;
        typedef typename RayM::simdi rsimdi;
        typedef Vec3<rsimdf> rsimd3f;
        
        struct Precalculations {
          __forceinline Precalculations (const rsimdb& valid, const RayM& ray) {}
        };
        
        /*! Intersects a M rays with N triangles. */
        static __forceinline void intersect(const rsimdb& valid_i, Precalculations& pre, RayM& ray, const TriangleNMblur& tri, Scene* scene)
        {
          for (size_t i=0; i<TriangleNMblur::max_size(); i++)
          {
            if (!tri.valid(i)) break;
            STAT3(normal.trav_prims,1,popcnt(valid_i),RayM::size());
            const rsimdf time = ray.time;
            const rsimd3f v0 = broadcast<rsimdf>(tri.v0,i) + time*broadcast<rsimdf>(tri.dv0,i);
            const rsimd3f v1 = broadcast<rsimdf>(tri.v1,i) + time*broadcast<rsimdf>(tri.dv1,i);
            const rsimd3f v2 = broadcast<rsimdf>(tri.v2,i) + time*broadcast<rsimdf>(tri.dv2,i);
            triangle_intersect_moeller_trumbore<enableIntersectionFilter>(valid_i,ray,v0,v1,v2,tri.geomIDs,tri.primIDs,i,scene);
          }
        }
        
        /*! Test for M rays if they are occluded by any of the N triangle. */
        static __forceinline rsimdb occluded(const rsimdb& valid_i, Precalculations& pre, RayM& ray, const TriangleNMblur& tri, Scene* scene)
        {
          rsimdb valid0 = valid_i;
          
          for (size_t i=0; i<TriangleNMblur::max_size(); i++)
          {
            if (!tri.valid(i)) break;
            STAT3(shadow.trav_prims,1,popcnt(valid0),RayM::size());
            const rsimdf time = ray.time;
            const rsimd3f v0 = broadcast<rsimdf>(tri.v0,i) + time*broadcast<rsimdf>(tri.dv0,i);
            const rsimd3f v1 = broadcast<rsimdf>(tri.v1,i) + time*broadcast<rsimdf>(tri.dv1,i);
            const rsimd3f v2 = broadcast<rsimdf>(tri.v2,i) + time*broadcast<rsimdf>(tri.dv2,i);
            triangle_occluded_moeller_trumbore<enableIntersectionFilter>(valid0,ray,v0,v1,v2,tri.geomIDs,tri.primIDs,i,scene);
            if (none(valid0)) break;
          }
          return !valid0;
        }
        
        /*! Intersect a ray with the N triangles and updates the hit. */
        static __forceinline void intersect(Precalculations& pre, RayM& ray, size_t k, const TriangleNMblur& tri, Scene* scene)
        {
          STAT3(normal.trav_prims,1,1,1);
          const tsimdf time = broadcast<tsimdf>(ray.time,k);
          const tsimd3f v0 = tri.v0 + time*tri.dv0;
          const tsimd3f v1 = tri.v1 + time*tri.dv1;
          const tsimd3f v2 = tri.v2 + time*tri.dv2;
          triangle_intersect_moeller_trumbore<enableIntersectionFilter>(ray,k,v0,v1,v2,tri.geomIDs,tri.primIDs,scene);
        }
        
        /*! Test if the ray is occluded by one of the N triangles. */
        static __forceinline bool occluded(Precalculations& pre, RayM& ray, size_t k, const TriangleNMblur& tri, Scene* scene)
        {
          STAT3(shadow.trav_prims,1,1,1);
          const tsimdf time = broadcast<tsimdf>(ray.time,k);
          const tsimd3f v0 = tri.v0 + time*tri.dv0;
          const tsimd3f v1 = tri.v1 + time*tri.dv1;
          const tsimd3f v2 = tri.v2 + time*tri.dv2;
          return triangle_occluded_moeller_trumbore<enableIntersectionFilter>(ray,k,v0,v1,v2,tri.geomIDs,tri.primIDs,scene);
        }
      };
  }
}
