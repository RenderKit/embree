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

/*! Modified Pluecker ray/triangle intersector. The edge equations
 *  are watertight along the edge for neighboring triangles. */

namespace embree
{
  namespace isa
  {
    /////////////////////////////////////////////////////////////////////////////////////////////////////////
    /////////////////////////////////////////////////////////////////////////////////////////////////////////
    /////////////////////////////////////////////////////////////////////////////////////////////////////////

    /*! Intersect a ray with the multiple triangles and updates the hit. */
    template<bool enableIntersectionFilter, typename tsimdb, typename tsimdf, typename tsimdi>
      __forceinline void triangle_intersect_pluecker2(const Vec3<tsimdf>& ray_rdir, 
						      const Vec3<tsimdf>& ray_org_rdir, 
						      const Vec3<tsimdf>& ray_dir_scale, 
						      Ray& ray, 
						      const Vec3<tsimdf>& tri_v0, 
						      const Vec3<tsimdf>& tri_v1, 
						      const Vec3<tsimdf>& tri_v2,  
						      const tsimdi& tri_geomIDs, 
						      const tsimdi& tri_primIDs, Scene* scene)
    {
      /* calculate vertices relative to ray origin */
      typedef Vec3<tsimdf> tsimd3f;
      const tsimd3f v0 = msub(tri_v0,ray_rdir,ray_org_rdir);
      const tsimd3f v1 = msub(tri_v1,ray_rdir,ray_org_rdir);
      const tsimd3f v2 = msub(tri_v2,ray_rdir,ray_org_rdir);
      
      /* calculate triangle edges */
      const tsimd3f e0 = v2-v0;
      const tsimd3f e1 = v0-v1;
      const tsimd3f e2 = v1-v2;
      
      /* perform edge tests */
      const tsimdf U = reduce_add(cross(v2+v0,e0));
      const tsimdf V = reduce_add(cross(v0+v1,e1));
      const tsimdf W = reduce_add(cross(v1+v2,e2));
      const tsimdf minUVW = min(U,V,W);
      const tsimdf maxUVW = max(U,V,W);
      tsimdb valid = minUVW >= 0.0f;
#if !defined(RTCORE_BACKFACE_CULLING)
      valid |= maxUVW <= 0.0f;
#endif
      if (unlikely(none(valid))) return;

      /* calculate geometry normal and denominator */
      //const tsimd3f Ng1 = cross(e1,e0);
      const tsimd3f Ng1 = stable_triangle_normal(e2,e1,e0);
      tsimd3f tri_Ng = Ng1+Ng1;
      const tsimdf den     = reduce_add(tri_Ng);
      const tsimdf absDen  = abs(den);
      const tsimdf sgnDen  = signmsk(den);

      /* perform depth test */
      const tsimdf T = dot(v0,tri_Ng);
      valid &= ((T^sgnDen) >= absDen*tsimdf(ray.tnear)) & (absDen*tsimdf(ray.tfar) >= (T^sgnDen));
      if (unlikely(none(valid))) return;
      
      /* perform backface culling */
      valid &= den != tsimdf(zero);
      if (unlikely(none(valid))) return;

      const tsimdf rcpDen = rcp(den);
      const tsimdf t = T * rcpDen;
      const tsimdf u = U * rcpDen;
      const tsimdf v = V * rcpDen;

      tri_Ng = tri_Ng * ray_dir_scale;

      size_t i = select_min(valid,t);

      int geomID = tri_geomIDs[i];
      
      /* intersection filter test */
#if defined(RTCORE_INTERSECTION_FILTER) || defined(RTCORE_RAY_MASK)
      goto entry;
      while (true) 
      {
        if (none(valid)) return;
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
    
    /*! Test if the ray is occluded by one of the triangles. */
    template<bool enableIntersectionFilter, typename tsimdb, typename tsimdf, typename tsimdi>
      __forceinline bool triangle_occluded_pluecker2(const Vec3<tsimdf>& ray_rdir, 
						     const Vec3<tsimdf>& ray_org_rdir, 
						     const Vec3<tsimdf>& ray_dir_scale,
						     Ray& ray, 
						     const Vec3<tsimdf>& tri_v0, 
						     const Vec3<tsimdf>& tri_v1, 
						     const Vec3<tsimdf>& tri_v2, 
						     const tsimdi& tri_geomIDs, 
						     const tsimdi& tri_primIDs,  
						     Scene* scene)
    {
      /* calculate vertices relative to ray origin */
      typedef Vec3<tsimdf> tsimd3f;
      const tsimd3f v0 = msub(tri_v0,ray_rdir,ray_org_rdir);
      const tsimd3f v1 = msub(tri_v1,ray_rdir,ray_org_rdir);
      const tsimd3f v2 = msub(tri_v2,ray_rdir,ray_org_rdir);
      
      /* calculate triangle edges */
      const tsimd3f e0 = v2-v0;
      const tsimd3f e1 = v0-v1;
      const tsimd3f e2 = v1-v2;

      /* perform edge tests */
      const tsimdf U = reduce_add(cross(v2+v0,e0));
      const tsimdf V = reduce_add(cross(v0+v1,e1));
      const tsimdf W = reduce_add(cross(v1+v2,e2));
      const tsimdf minUVW = min(U,V,W);
      const tsimdf maxUVW = max(U,V,W);
      tsimdb valid = minUVW >= 0.0f;
#if !defined(RTCORE_BACKFACE_CULLING)
      valid |= maxUVW <= 0.0f;
#endif
      if (unlikely(none(valid))) return false;

      /* calculate geometry normal and denominator */
      //const tsimd3f Ng1 = cross(e1,e0);
      const tsimd3f Ng1 = stable_triangle_normal(e2,e1,e0);
      tsimd3f tri_Ng = Ng1+Ng1;
      const tsimdf den     = reduce_add(tri_Ng);
      const tsimdf absDen  = abs(den);
      const tsimdf sgnDen  = signmsk(den);

      /* perform depth test */
      const tsimdf T = dot(v0,tri_Ng);
      valid &= ((T^sgnDen) >= absDen*tsimdf(ray.tnear)) & (absDen*tsimdf(ray.tfar) >= (T^sgnDen));
      if (unlikely(none(valid))) return false;
      
      /* perform backface culling */
      valid &= den != tsimdf(zero);
      if (unlikely(none(valid))) return false;

      tri_Ng = tri_Ng * ray_dir_scale;
      
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
            const tsimdf rcpDen = rcp(den);
            const tsimdf u = U * rcpDen;
            const tsimdf v = V * rcpDen;
            const tsimdf t = T * rcpDen;
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
    

    /////////////////////////////////////////////////////////////////////////////////////////////////////////
    /////////////////////////////////////////////////////////////////////////////////////////////////////////
    /////////////////////////////////////////////////////////////////////////////////////////////////////////
    
    /*! Intersects M rays with N triangles. */

    // 
    template<bool enableIntersectionFilter, typename rsimdb, typename rsimdf, typename tsimdi, typename RayM>
      __forceinline void triangle_intersect_pluecker2(const rsimdb& valid0,
						      const Vec3<rsimdf>& ray_rdir, 
						      const Vec3<rsimdf>& ray_org_rdir, 
						      const Vec3<rsimdf>& ray_dir_scale,
						      RayM& ray, 
						      const Vec3<rsimdf>& tri_v0, 
						      const Vec3<rsimdf>& tri_v1, 
						      const Vec3<rsimdf>& tri_v2, 
						      const tsimdi& tri_geomIDs, 
						      const tsimdi& tri_primIDs, 
						      const size_t i, 
						      Scene* scene)						      
    {
      /* ray SIMD type shortcuts */
      //typedef typename RayM::simdb rsimdb;
      //typedef typename RayM::simdf rsimdf;
      typedef typename RayM::simdi rsimdi;
      typedef Vec3<rsimdf> rsimd3f;
      
      /* calculate vertices relative to ray origin */
      rsimdb valid = valid0;

      const rsimd3f v0 = msub(tri_v0,ray_rdir,ray_org_rdir);
      const rsimd3f v1 = msub(tri_v1,ray_rdir,ray_org_rdir);
      const rsimd3f v2 = msub(tri_v2,ray_rdir,ray_org_rdir);
      
      /* calculate triangle edges */
      const rsimd3f e0 = v2-v0;
      const rsimd3f e1 = v0-v1;
      const rsimd3f e2 = v1-v2;
      
      /* calculate geometry normal and denominator */
      //const rsimd3f Ng1 = cross(e1,e0);
      const rsimd3f Ng1 = stable_triangle_normal(e2,e1,e0);
            rsimd3f Ng = Ng1+Ng1;
      const rsimdf den = reduce_add(rsimd3f(Ng));
      const rsimdf absDen = abs(den);
      const rsimdf sgnDen = signmsk(den);
      
      /* perform edge tests */
      const rsimdf U = reduce_add(rsimd3f(cross(v2+v0,e0))) ^ sgnDen;
      valid &= U >= 0.0f;
      if (likely(none(valid))) return;
      const rsimdf V = reduce_add(rsimd3f(cross(v0+v1,e1))) ^ sgnDen;
      valid &= V >= 0.0f;
      if (likely(none(valid))) return;
      const rsimdf W = reduce_add(rsimd3f(cross(v1+v2,e2))) ^ sgnDen;
      valid &= W >= 0.0f;
      if (likely(none(valid))) return;
      
      /* perform depth test */
      const rsimdf T = dot(v0,rsimd3f(Ng)) ^ sgnDen;
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
      const rsimdf u = U * rcpAbsDen;
      const rsimdf v = V * rcpAbsDen;
      const rsimdf t = T * rcpAbsDen;
      const int geomID = tri_geomIDs[i];
      const int primID = tri_primIDs[i];


      Ng = Ng * ray_dir_scale;

      Geometry* geometry = scene->get(geomID);
      
      /* ray masking test */
#if defined(RTCORE_RAY_MASK)
      valid &= (geometry->mask & ray.mask) != 0;
      if (unlikely(none(valid))) return;
#endif
      
      /* intersection filter test */
#if defined(RTCORE_INTERSECTION_FILTER)
      if (enableIntersectionFilter) {
        if (unlikely(geometry->hasIntersectionFilter<rsimdf>())) {
          runIntersectionFilter(valid,geometry,ray,u,v,t,Ng,geomID,primID);
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
      rsimdf::store(valid,&ray.Ng.x,Ng.x);
      rsimdf::store(valid,&ray.Ng.y,Ng.y);
      rsimdf::store(valid,&ray.Ng.z,Ng.z);
    }
    
    
    /*! Test for M rays if they are occluded by any of the N triangle. */
    template<bool enableIntersectionFilter, typename rsimdf, typename tsimdi, typename RayM>
      __forceinline void triangle_occluded_pluecker2(typename RayM::simdb& valid0,
						     const Vec3<rsimdf>& ray_rdir, 
						     const Vec3<rsimdf>& ray_org_rdir, 
						     const Vec3<rsimdf>& ray_dir_scale,
						     RayM& ray, 
						     const Vec3<rsimdf>& tri_v0, 
						     const Vec3<rsimdf>& tri_v1, 
						     const Vec3<rsimdf>& tri_v2, 
						     const tsimdi& tri_geomIDs, 
						     const tsimdi& tri_primIDs, 
						     const size_t i, Scene* scene)
    {
      /* ray SIMD type shortcuts */
      typedef typename RayM::simdb rsimdb;
      //typedef typename RayM::simdf rsimdf;
      //typedef typename RayM::simdi rsimdi;
      typedef Vec3<rsimdf> rsimd3f;
      
      /* calculate vertices relative to ray origin */
      rsimdb valid = valid0;
      const rsimd3f v0 = msub(tri_v0,ray_rdir,ray_org_rdir);
      const rsimd3f v1 = msub(tri_v1,ray_rdir,ray_org_rdir);
      const rsimd3f v2 = msub(tri_v2,ray_rdir,ray_org_rdir);
      
      /* calculate triangle edges */
      const rsimd3f e0 = v2-v0;
      const rsimd3f e1 = v0-v1;
      const rsimd3f e2 = v1-v2;
      
      /* calculate geometry normal and denominator */
      //const rsimd3f Ng1 = cross(e1,e0);
      const rsimd3f Ng1 = stable_triangle_normal(e2,e1,e0);
            rsimd3f Ng = Ng1+Ng1;
      const rsimdf den = reduce_add(rsimd3f(Ng));
      const rsimdf absDen = abs(den);
      const rsimdf sgnDen = signmsk(den);
      
      /* perform edge tests */
      const rsimdf U = reduce_add(rsimd3f(cross(v2+v0,e0))) ^ sgnDen;
      valid &= U >= 0.0f;
      if (likely(none(valid))) return;
      const rsimdf V = reduce_add(rsimd3f(cross(v0+v1,e1))) ^ sgnDen;
      valid &= V >= 0.0f;
      if (likely(none(valid))) return;
      const rsimdf W = reduce_add(rsimd3f(cross(v1+v2,e2))) ^ sgnDen;
      valid &= W >= 0.0f;
      if (likely(none(valid))) return;
      
      /* perform depth test */
      const rsimdf T = dot(v0,rsimd3f(Ng)) ^ sgnDen;
      valid &= (T >= absDen*ray.tnear) & (absDen*ray.tfar >= T);
      
      /* perform backface culling */
#if defined(RTCORE_BACKFACE_CULLING)
      valid &= den > rsimdf(zero);
      if (unlikely(none(valid))) return;
#else
      valid &= den != rsimdf(zero);
      if (unlikely(none(valid))) return;
#endif
      

      Ng = Ng * ray_dir_scale;

      /* ray masking test */
      const int geomID = tri_geomIDs[i];
      Geometry* geometry = scene->get(geomID);
#if defined(RTCORE_RAY_MASK)
      valid &= (geometry->mask & ray.mask) != 0;
      if (unlikely(none(valid))) return;
#endif
      
      /* occlusion filter test */
#if defined(RTCORE_INTERSECTION_FILTER)
      if (enableIntersectionFilter) {
        if (unlikely(geometry->hasOcclusionFilter<rsimdf>()))
        {
          const rsimdf rcpAbsDen = rcp(absDen);
          const rsimdf u = U * rcpAbsDen;
          const rsimdf v = V * rcpAbsDen;
          const rsimdf t = T * rcpAbsDen;
          const int primID = tri_primIDs[i];
          valid = runOcclusionFilter(valid,geometry,ray,u,v,t,Ng,geomID,primID);
        }
      }
#endif
      
      /* update occlusion */
      valid0 &= !valid;
    }
    
    /*! Intersect a ray with the N triangles and updates the hit. */
    template<bool enableIntersectionFilter, typename tsimdf, typename tsimdi, typename RayM>
      __forceinline void triangle_intersect_pluecker2(const Vec3<tsimdf>& ray_rdir, 
						      const Vec3<tsimdf>& ray_org_rdir, 
						      const Vec3<tsimdf>& ray_dir_scale,
						      RayM& ray, 
						      size_t k, 
						      const Vec3<tsimdf>& tri_v0, 
						      const Vec3<tsimdf>& tri_v1, 
						      const Vec3<tsimdf>& tri_v2,
						      const tsimdi& tri_geomIDs, 
						      const tsimdi& tri_primIDs, 
						      Scene* scene)
    {
      /* type shortcuts */
      typedef typename RayM::simdf rsimdf;
      typedef typename tsimdf::Mask tsimdb;
      typedef Vec3<tsimdf> tsimd3f;
      
      /* calculate vertices relative to ray origin */
      const tsimd3f v0 = msub(tri_v0,ray_rdir,ray_org_rdir);
      const tsimd3f v1 = msub(tri_v1,ray_rdir,ray_org_rdir);
      const tsimd3f v2 = msub(tri_v2,ray_rdir,ray_org_rdir);

      /* calculate triangle edges */
      const tsimd3f e0 = v2-v0;
      const tsimd3f e1 = v0-v1;
      const tsimd3f e2 = v1-v2;
      
      /* calculate geometry normal and denominator */
      //const tsimd3f Ng1 = cross(e1,e0);
      const tsimd3f Ng1 = stable_triangle_normal(e2,e1,e0);
            tsimd3f tri_Ng = Ng1+Ng1;
      const tsimdf den = reduce_add(tri_Ng);
      const tsimdf absDen = abs(den);
      const tsimdf sgnDen = signmsk(den);
      
      /* perform edge tests */
      const tsimdf U = reduce_add(cross(v2+v0,e0)) ^ sgnDen;
      const tsimdf V = reduce_add(cross(v0+v1,e1)) ^ sgnDen;
      const tsimdf W = reduce_add(cross(v1+v2,e2)) ^ sgnDen;
      tsimdb valid = (U >= 0.0f) & (V >= 0.0f) & (W >= 0.0f);
      if (unlikely(none(valid))) return;
      
      /* perform depth test */
      const tsimdf T = dot(v0,tri_Ng) ^ sgnDen;
      valid &= (T >= absDen*tsimdf(ray.tnear[k])) & (absDen*tsimdf(ray.tfar[k]) >= T);
      if (unlikely(none(valid))) return;
      
      /* perform backface culling */
#if defined(RTCORE_BACKFACE_CULLING)
      valid &= den > tsimdf(zero);
      if (unlikely(none(valid))) return;
#else
      valid &= den != tsimdf(zero);
      if (unlikely(none(valid))) return;
#endif
      
      /* calculate hit information */
      const tsimdf rcpAbsDen = rcp(absDen);
      const tsimdf t = T * rcpAbsDen;
      const tsimdf u = U * rcpAbsDen;
      const tsimdf v = V * rcpAbsDen;

      size_t i = select_min(valid,t);
      int geomID = tri_geomIDs[i];
      
      tri_Ng = tri_Ng * ray_dir_scale;

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
    
    /*! Test if the ray is occluded by one of the triangles. */
    template<bool enableIntersectionFilter,typename tsimdf, typename tsimdi, typename RayM>
      __forceinline bool triangle_occluded_pluecker2(const Vec3<tsimdf>& ray_rdir, 
						     const Vec3<tsimdf>& ray_org_rdir, 
						     const Vec3<tsimdf>& ray_dir_scale,
						     RayM& ray, 
						     size_t k, 
						     const Vec3<tsimdf>& tri_v0, 
						     const Vec3<tsimdf>& tri_v1, 
						     const Vec3<tsimdf>& tri_v2, 
						     const tsimdi& tri_geomIDs, 
						     const tsimdi& tri_primIDs, 
						     Scene* scene)
    {
      /* type shortcuts */
      typedef typename RayM::simdf rsimdf;
      typedef typename tsimdf::Mask tsimdb;
      typedef Vec3<tsimdf> tsimd3f;
      
      /* calculate vertices relative to ray origin */
      const tsimd3f v0 = msub(tri_v0,ray_rdir,ray_org_rdir);
      const tsimd3f v1 = msub(tri_v1,ray_rdir,ray_org_rdir);
      const tsimd3f v2 = msub(tri_v2,ray_rdir,ray_org_rdir);

      /* calculate triangle edges */
      const tsimd3f e0 = v2-v0;
      const tsimd3f e1 = v0-v1;
      const tsimd3f e2 = v1-v2;
      
      /* calculate geometry normal and denominator */
      //const tsimd3f Ng1 = cross(e1,e0);
      const tsimd3f Ng1 = stable_triangle_normal(e2,e1,e0);
            tsimd3f tri_Ng = Ng1+Ng1;
      const tsimdf den = reduce_add(tri_Ng);
      const tsimdf absDen = abs(den);
      const tsimdf sgnDen = signmsk(den);
      
      /* perform edge tests */
      const tsimdf U = reduce_add(cross(v2+v0,e0)) ^ sgnDen;
      const tsimdf V = reduce_add(cross(v0+v1,e1)) ^ sgnDen;
      const tsimdf W = reduce_add(cross(v1+v2,e2)) ^ sgnDen;
      tsimdb valid = (U >= 0.0f) & (V >= 0.0f) & (W >= 0.0f);
      if (unlikely(none(valid))) return false;
      
      /* perform depth test */
      const tsimdf T = dot(v0,tri_Ng) ^ sgnDen;
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
      
      tri_Ng = tri_Ng * ray_dir_scale;

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
	    const tsimdf t = T * rcpAbsDen;
	    const tsimdf u = U * rcpAbsDen;
	    const tsimdf v = V * rcpAbsDen;
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
    
    
    /*! Intersects N triangles with 1 ray */
    template<typename TriangleNv, bool enableIntersectionFilter>
      struct TriangleNvIntersector1Pluecker2
      {
        typedef TriangleNv Primitive;
        
        /* type shortcuts */
        typedef typename TriangleNv::simdb tsimdb;
        typedef typename TriangleNv::simdf tsimdf;
        typedef typename TriangleNv::simdi tsimdi;
        typedef Vec3<tsimdf> tsimd3f;
        
        struct Precalculations {
	  tsimd3f ray_rdir;
	  tsimd3f ray_org_rdir;
	  tsimd3f ray_dir_scale;

          __forceinline Precalculations (const Ray& ray, const void* ptr) 
	  {
	    const Vec3fa rdir     = rcp_safe(ray.dir);
	    const Vec3fa org_rdir = ray.org*rdir;
	    ray_rdir              = tsimd3f(rdir.x,rdir.y,rdir.z);
	    ray_org_rdir          = tsimd3f(org_rdir.x,org_rdir.y,org_rdir.z);
	    ray_dir_scale         = tsimd3f(ray.dir.y*ray.dir.z,ray.dir.z*ray.dir.x,ray.dir.x*ray.dir.y);
	  }
        };
        
        /*! Intersect a ray with the 4 triangles and updates the hit. */
        static __forceinline void intersect(Precalculations& pre, Ray& ray, const Primitive& tri, Scene* scene)
        {
          STAT3(normal.trav_prims,1,1,1);
          triangle_intersect_pluecker2<enableIntersectionFilter,tsimdb,tsimdf,tsimdi>(pre.ray_rdir,
										     pre.ray_org_rdir,
										     pre.ray_dir_scale,
										     ray,
										     tri.v0,
										     tri.v1,
										     tri.v2,
										     tri.geomIDs,
										     tri.primIDs,
										     scene);
        }
        
        /*! Test if the ray is occluded by one of the triangles. */
        static __forceinline bool occluded(const Precalculations& pre, Ray& ray, const Primitive& tri, Scene* scene)
        {
          STAT3(shadow.trav_prims,1,1,1);
          return triangle_occluded_pluecker2<enableIntersectionFilter,tsimdb,tsimdf,tsimdi>(pre.ray_rdir,
											   pre.ray_org_rdir,
											   pre.ray_dir_scale,
											   ray,
											   tri.v0,
											   tri.v1,
											   tri.v2,
											   tri.geomIDs,
											   tri.primIDs,
											   scene);
        }
      };
    
    template<typename RayM, typename TriangleNv, bool enableIntersectionFilter>
      struct TriangleNvIntersectorMPluecker2
      {
        typedef TriangleNv Primitive;
        
        /* triangle SIMD type shortcuts */
        typedef typename TriangleNv::simdb tsimdb;
        typedef typename TriangleNv::simdf tsimdf;
        typedef typename TriangleNv::simdf tsimdi;
        typedef Vec3<tsimdf> tsimd3f;
        
        /* ray SIMD type shortcuts */
        typedef typename RayM::simdb rsimdb;
        typedef typename RayM::simdf rsimdf;
        typedef typename RayM::simdi rsimdi;
        typedef Vec3<rsimdf> rsimd3f;
        
        struct Precalculations {

	  rsimd3f ray_rdir;
	  rsimd3f ray_org_rdir;
	  rsimd3f ray_dir_scale;

          __forceinline Precalculations (const rsimdb& valid, const RayM& ray) 
	  {
	    ray_rdir              = rcp_safe(ray.dir);
	    ray_org_rdir          = ray.org*ray_rdir;
	    ray_dir_scale         = rsimd3f(ray.dir.y*ray.dir.z,ray.dir.z*ray.dir.x,ray.dir.x*ray.dir.y);
	  }
        };
        
        /*! Intersects a M rays with N triangles. */
        static __forceinline void intersect(const rsimdb& valid_i, Precalculations& pre, RayM& ray, const Primitive& tri, Scene* scene)
        {
          for (size_t i=0; i<TriangleNv::max_size(); i++)
          {
            if (!tri.valid(i)) break;
            STAT3(normal.trav_prims,1,popcnt(valid_i),RayM::size());
            const rsimd3f v0 = broadcast<rsimdf>(tri.v0,i);
            const rsimd3f v1 = broadcast<rsimdf>(tri.v1,i);
            const rsimd3f v2 = broadcast<rsimdf>(tri.v2,i);

            triangle_intersect_pluecker2<enableIntersectionFilter>(valid_i,
								   pre.ray_rdir,
								   pre.ray_org_rdir,
								   pre.ray_dir_scale,
								   ray,
								   v0,
								   v1,
								   v2,
								   tri.geomIDs,
								   tri.primIDs,
								   i,
								   scene
								   );

          }
        }
        
        /*! Test for M rays if they are occluded by any of the N triangle. */
        static __forceinline rsimdb occluded(const rsimdb& valid_i, Precalculations& pre, RayM& ray, const Primitive& tri, Scene* scene)
        {
          rsimdb valid0 = valid_i;
          for (size_t i=0; i<TriangleNv::max_size(); i++)
          {
            if (!tri.valid(i)) break;
            STAT3(shadow.trav_prims,1,popcnt(valid_i),RayM::size());
            const rsimd3f v0 = broadcast<rsimdf>(tri.v0,i);
            const rsimd3f v1 = broadcast<rsimdf>(tri.v1,i);
            const rsimd3f v2 = broadcast<rsimdf>(tri.v2,i);
            triangle_occluded_pluecker2<enableIntersectionFilter>(valid0,
								 pre.ray_rdir,
								 pre.ray_org_rdir,
								 pre.ray_dir_scale,
								 ray,
								 v0,
								 v1,
								 v2,
								 tri.geomIDs,
								 tri.primIDs,
								 i,
								 scene);
            if (none(valid0)) break;
          }
          return !valid0;
        }
        
        /*! Intersect a ray with the N triangles and updates the hit. */
        static __forceinline void intersect(Precalculations& pre, RayM& ray, size_t k, const Primitive& tri, Scene* scene)
        {
          STAT3(normal.trav_prims,1,1,1);
	  const tsimd3f ray_rdir      = broadcast<tsimdf>(pre.ray_rdir,k);
	  const tsimd3f ray_org_rdir  = broadcast<tsimdf>(pre.ray_org_rdir,k);
	  const tsimd3f ray_dir_scale = broadcast<tsimdf>(pre.ray_dir_scale,k);
          triangle_intersect_pluecker2<enableIntersectionFilter>(ray_rdir,
								 ray_org_rdir,
								 ray_dir_scale,
								 ray,
								 k,
								 tri.v0,
								 tri.v1,
								 tri.v2,
								 tri.geomIDs,
								 tri.primIDs,
								 scene);
        }
        
        /*! Test if the ray is occluded by one of the triangles. */
        static __forceinline bool occluded(Precalculations& pre, RayM& ray, size_t k, const Primitive& tri, Scene* scene)
        {
          STAT3(shadow.trav_prims,1,1,1);
	  const tsimd3f ray_rdir      = broadcast<tsimdf>(pre.ray_rdir,k);
	  const tsimd3f ray_org_rdir  = broadcast<tsimdf>(pre.ray_org_rdir,k);
	  const tsimd3f ray_dir_scale = broadcast<tsimdf>(pre.ray_dir_scale,k);
          return triangle_occluded_pluecker2<enableIntersectionFilter>(ray_rdir,
								       ray_org_rdir,
								       ray_dir_scale,
								       ray,
								       k,
								       tri.v0,
								       tri.v1,
								       tri.v2,
								       tri.geomIDs,
								       tri.primIDs,
								       scene);
        }
      };
  }
}
