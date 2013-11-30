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

#ifndef __EMBREE_ACCEL_TRIANGLE4_INTERSECTOR4_MOELLER_H__
#define __EMBREE_ACCEL_TRIANGLE4_INTERSECTOR4_MOELLER_H__

#include "triangle4.h"
#include "triangle4_intersector1_moeller.h"

#include "../common/ray4.h"

namespace embree
{
  /*! Intersector for 4 triangles with 4 rays. This intersector
   *  implements a modified version of the Moeller Trumbore
   *  intersector from the paper "Fast, Minimum Storage Ray-Triangle
   *  Intersection". In contrast to the paper we precalculate some
   *  factors and factor the calculations differently to allow
   *  precalculating the cross product e1 x e2. */
  struct Triangle4Intersector4MoellerTrumbore
  {
    typedef Triangle4 Primitive;

    /*! Intersects a 4 rays with 4 triangles. */
    static __forceinline void intersect(const sseb& valid_i, Ray4& ray, const Triangle4& tri, void* geom)
    {
      for (size_t i=0; i<tri.size(); i++)
      {
        STAT3(normal.trav_prims,1,popcnt(valid_i),4);

        /* load edges and geometry normal */
        sseb valid = valid_i;
        const sse3f p0(tri.v0.x[i],tri.v0.y[i],tri.v0.z[i]);
        const sse3f e1(tri.e1.x[i],tri.e1.y[i],tri.e1.z[i]);
        const sse3f e2(tri.e2.x[i],tri.e2.y[i],tri.e2.z[i]);
        const sse3f Ng(tri.Ng.x[i],tri.Ng.y[i],tri.Ng.z[i]);
        
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
#if defined(__BACKFACE_CULLING__)
        valid &= den > ssef(zero);
        if (unlikely(none(valid))) continue;
#else
        valid &= den != ssef(zero);
        if (unlikely(none(valid))) continue;
#endif

        /* ray masking test */
#if USE_RAY_MASK
        valid &= (tri.mask[i] & ray.mask) != 0;
        if (unlikely(none(valid))) continue;
#endif
        
        /* update hit information for all rays that hit the triangle */
        const ssef rcpAbsDen = rcp(absDen);
        ray.u    = select(valid,U * rcpAbsDen,ray.u );
        ray.v    = select(valid,V * rcpAbsDen,ray.v );
        ray.tfar  = select(valid,T * rcpAbsDen,ray.tfar );
        ray.geomID  = select(valid,tri.geomID[i]  ,ray.geomID);
        ray.primID  = select(valid,tri.primID[i]  ,ray.primID);
        ray.Ng.x = select(valid,Ng.x,ray.Ng.x);
        ray.Ng.y = select(valid,Ng.y,ray.Ng.y);
        ray.Ng.z = select(valid,Ng.z,ray.Ng.z);
      }
    }

    static __forceinline void intersect(const sseb& valid, Ray4& ray, const Triangle4* tri, size_t num, void* geom)
    {
      for (size_t i=0; i<num; i++) {
        intersect(valid,ray,tri[i],geom);
      }
    }

    /*! Test for 4 rays if they are occluded by any of the 4 triangle. */
    static __forceinline sseb occluded(const sseb& valid_i, const Ray4& ray, const Triangle4& tri, void* geom)
    {
      sseb valid0 = valid_i;

      for (size_t i=0; i<tri.size(); i++)
      {
        STAT3(shadow.trav_prims,1,popcnt(valid0),4);

        /* load edges and geometry normal */
        sseb valid = valid0;
        const sse3f p0(tri.v0.x[i],tri.v0.y[i],tri.v0.z[i]);
        const sse3f e1(tri.e1.x[i],tri.e1.y[i],tri.e1.z[i]);
        const sse3f e2(tri.e2.x[i],tri.e2.y[i],tri.e2.z[i]);
        const sse3f Ng(tri.Ng.x[i],tri.Ng.y[i],tri.Ng.z[i]);
        
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
#if defined(__BACKFACE_CULLING__)
        valid &= den > ssef(zero);
        if (unlikely(none(valid))) continue;
#else
        valid &= den != ssef(zero);
        if (unlikely(none(valid))) continue;
#endif

        /* ray masking test */
#if USE_RAY_MASK
        valid &= (tri.mask[i] & ray.mask) != 0;
        if (unlikely(none(valid))) continue;
#endif

        /* update occlusion */
        valid0 &= !valid;
        if (none(valid0)) break;
      }
      return !valid0;
    }

    static __forceinline sseb occluded(const sseb& valid, const Ray4& ray, const Triangle4* tri, size_t num, void* geom)
    {
      sseb valid0 = valid;
      for (size_t i=0; i<num; i++) {
        valid0 &= !occluded(valid0,ray,tri[i],geom);
        if (none(valid0)) break;
      }
      return !valid0;
    }

    /*! Intersect a ray with the 4 triangles and updates the hit. */
    static __forceinline void intersect(Ray4& ray, size_t k, const Triangle4& tri, void* geom)
    {
      /* calculate denominator */
      STAT3(normal.trav_prims,1,1,1);
      const sse3f O = sse3f(ray.org.x[k],ray.org.y[k],ray.org.z[k]); // FIXME: optimize this with shuffles
      const sse3f D = sse3f(ray.dir.x[k],ray.dir.y[k],ray.dir.z[k]); // FIXME: optimize this with shuffles
      const sse3f C = sse3f(tri.v0) - O;
      const sse3f R = cross(D,C);
      const ssef den = dot(sse3f(tri.Ng),D);
      const ssef absDen = abs(den);
      const ssef sgnDen = signmsk(den);

      /* perform edge tests */
      const ssef U = dot(R,sse3f(tri.e2)) ^ sgnDen;
      const ssef V = dot(R,sse3f(tri.e1)) ^ sgnDen;

      /* perform backface culling */
#if defined(__BACKFACE_CULLING__)
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
#if USE_RAY_MASK
      valid &= (tri.mask & ray.mask[k]) != 0;
      if (unlikely(none(valid))) return;
#endif

      /* update hit information */
      const ssef rcpAbsDen = rcp(absDen);
      const ssef u = U * rcpAbsDen;
      const ssef v = V * rcpAbsDen;
      const ssef t = T * rcpAbsDen;
      const size_t i = select_min(valid,t);
      ray.u[k]   = u[i];
      ray.v[k]   = v[i];
      ray.tfar[k] = t[i];
      ray.Ng.x[k] = tri.Ng.x[i];
      ray.Ng.y[k] = tri.Ng.y[i];
      ray.Ng.z[k] = tri.Ng.z[i];
      ray.geomID[k] = tri.geomID[i];
      ray.primID[k] = tri.primID[i];
    }

    static __forceinline void intersect(Ray4& ray, size_t k, const Triangle4* tri, size_t num, void* geom)
    {
      for (size_t i=0; i<num; i++)
        intersect(ray,k,tri[i],geom);
    }

    /*! Test if the ray is occluded by one of the triangles. */
    static __forceinline bool occluded(Ray4& ray, size_t k, const Triangle4& tri, void* geom)
    {
      /* calculate denominator */
      STAT3(shadow.trav_prims,1,1,1);
      const sse3f O = sse3f(ray.org.x[k],ray.org.y[k],ray.org.z[k]); // FIXME: optimize this with shuffles
      const sse3f D = sse3f(ray.dir.x[k],ray.dir.y[k],ray.dir.z[k]); // FIXME: optimize this with shuffles
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
#if defined(__BACKFACE_CULLING__)
      valid &= den > ssef(zero);
      if (unlikely(none(valid))) return false;
#else
      valid &= den != ssef(zero);
      if (unlikely(none(valid))) return false;
#endif

      /* ray masking test */
#if USE_RAY_MASK
      valid &= (tri.mask & ray.mask[k]) != 0;
      if (unlikely(none(valid))) return false;
#endif
      return true;
    }

    static __forceinline bool occluded(Ray4& ray, size_t k, const Triangle4* tri, size_t num, void* geom) 
    {
      for (size_t i=0; i<num; i++) 
        if (occluded(ray,k,tri[i],geom))
          return true;

      return false;
    }
  };
}

#endif
