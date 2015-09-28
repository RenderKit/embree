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

#include "triangle.h"
#include "trianglev.h"
#include "trianglev_mb.h"
#include "intersector_epilog.h"

/*! Modified Pluecker ray/triangle intersector. The test first shifts
 *  the ray origin into the origin of the coordinate system and then
 *  uses Pluecker coordinates for the intersection. Due to the shift,
 *  the Pluecker coordinate calculation simplifies and the tests get
 *  numerically stable. The edge equations are watertight along the
 *  edge for neighboring triangles. */

namespace embree
{
  namespace isa
  {
    template<int M>
      struct PlueckerIntersector1
      {
        __forceinline PlueckerIntersector1 (const Ray& ray, const void* ptr) {
        }
        
        template<typename UVMapper, typename Epilog>
          __forceinline bool intersect(Ray& ray, 
                                       const Vec3<vfloat<M>>& tri_v0, 
                                       const Vec3<vfloat<M>>& tri_v1, 
                                       const Vec3<vfloat<M>>& tri_v2,  
                                       const UVMapper& mapUV,
                                       const Epilog& epilog) const
        {
          /* calculate vertices relative to ray origin */
          typedef Vec3<vfloat<M>> tsimd3f;
          const tsimd3f O = tsimd3f(ray.org);
          const tsimd3f D = tsimd3f(ray.dir);
          const tsimd3f v0 = tri_v0-O;
          const tsimd3f v1 = tri_v1-O;
          const tsimd3f v2 = tri_v2-O;
          
          /* calculate triangle edges */
          const tsimd3f e0 = v2-v0;
          const tsimd3f e1 = v0-v1;
          const tsimd3f e2 = v1-v2;
          
          /* perform edge tests */
          const vfloat<M> U = dot(cross(v2+v0,e0),D);
          const vfloat<M> V = dot(cross(v0+v1,e1),D);
          const vfloat<M> W = dot(cross(v1+v2,e2),D);
          const vfloat<M> minUVW = min(U,V,W);
          const vfloat<M> maxUVW = max(U,V,W);
          vbool<M> valid = minUVW >= 0.0f;
#if !defined(RTCORE_BACKFACE_CULLING)
          valid |= maxUVW <= 0.0f;
#endif
          if (unlikely(none(valid))) return false;
          
          /* calculate geometry normal and denominator */
          //const tsimd3f Ng1 = cross(e1,e0);
          const tsimd3f Ng1 = stable_triangle_normal(e2,e1,e0);
          const tsimd3f Ng = Ng1+Ng1;
          const vfloat<M> den = dot(Ng,D);
          const vfloat<M> absDen = abs(den);
          const vfloat<M> sgnDen = signmsk(den);
          
          /* perform depth test */
          const vfloat<M> T = dot(v0,Ng);
          valid &= ((T^sgnDen) >= absDen*vfloat<M>(ray.tnear));
          valid &=(absDen*vfloat<M>(ray.tfar) >= (T^sgnDen));
          if (unlikely(none(valid))) return false;
          
          /* avoid division by 0 */
          valid &= den != vfloat<M>(zero);
          if (unlikely(none(valid))) return false;
          
          /* update hit information */
          return epilog(valid,[&] () {
              const vfloat<M> rcpDen = rcp(den);
              const vfloat<M> t = T * rcpDen;
              vfloat<M> u = U * rcpDen;
              vfloat<M> v = V * rcpDen;
              mapUV(u,v);
              return std::make_tuple(u,v,t,Ng);
            });
        }
      };
    
    template<int M, int K>
      struct PlueckerIntersectorK
      {
        __forceinline PlueckerIntersectorK(const vbool<K>& valid, const RayK<K>& ray) {}
        
        /*! Intersects K rays with one of M triangles. */
        template<typename UVMapper, typename Epilog>
          __forceinline vbool<K> intersectK(const vbool<K>& valid0, 
                                            RayK<K>& ray, 
                                            const Vec3<vfloat<K>>& tri_v0, 
                                            const Vec3<vfloat<K>>& tri_v1, 
                                            const Vec3<vfloat<K>>& tri_v2, 
                                            const UVMapper& mapUV,
                                            const Epilog& epilog) const
        {
          /* calculate vertices relative to ray origin */
          typedef Vec3<vfloat<K>> rsimd3f;
          vbool<K> valid = valid0;
          const rsimd3f O = ray.org;
          const rsimd3f D = ray.dir;
          const rsimd3f v0 = tri_v0-O;
          const rsimd3f v1 = tri_v1-O;
          const rsimd3f v2 = tri_v2-O;
          
          /* calculate triangle edges */
          const rsimd3f e0 = v2-v0;
          const rsimd3f e1 = v0-v1;
          const rsimd3f e2 = v1-v2;
           
          /* perform edge tests */
          const vfloat<K> U = dot(rsimd3f(cross(v2+v0,e0)),D);
          const vfloat<K> V = dot(rsimd3f(cross(v0+v1,e1)),D);
          const vfloat<K> W = dot(rsimd3f(cross(v1+v2,e2)),D);
          const vfloat<K> minUVW = min(U,V,W);
          const vfloat<K> maxUVW = max(U,V,W);
#if defined(RTCORE_BACKFACE_CULLING)
          valid &= minUVW >= 0.0f;
#else
          valid &= (minUVW >= 0.0f) | (maxUVW <= 0.0f);
#endif
          if (unlikely(none(valid))) return false;
          
           /* calculate geometry normal and denominator */
          //const rsimd3f Ng1 = cross(e1,e0);
          const rsimd3f Ng1 = stable_triangle_normal(e2,e1,e0);
          const rsimd3f Ng = Ng1+Ng1;
          const vfloat<K> den = dot(rsimd3f(Ng),D);
          const vfloat<K> absDen = abs(den);
          const vfloat<K> sgnDen = signmsk(den);

          /* perform depth test */
          const vfloat<K> T = dot(v0,rsimd3f(Ng));
          valid &= ((T^sgnDen) >= absDen*ray.tnear);
          valid &= (absDen*ray.tfar >= (T^sgnDen));
          if (unlikely(none(valid))) return false;
          
          /* avoid division by 0 */
          valid &= den != vfloat<K>(zero);
          if (unlikely(none(valid))) return false;
          
          /* calculate hit information */
          return epilog(valid,[&] () {
              const vfloat<K> rcpDen = rcp(den);
              const vfloat<K> t = T * rcpDen;
              vfloat<K> u = U * rcpDen;
              vfloat<K> v = V * rcpDen;
              mapUV(u,v);
              return std::make_tuple(u,v,t,Ng);
            });
        }
        
        /*! Intersect k'th ray from ray packet of size K with M triangles. */
        template<typename UVMapper, typename Epilog>
          __forceinline bool intersect(RayK<K>& ray, size_t k,
                                       const Vec3<vfloat<M>>& tri_v0, 
                                       const Vec3<vfloat<M>>& tri_v1, 
                                       const Vec3<vfloat<M>>& tri_v2, 
                                       const UVMapper& mapUV,
                                       const Epilog& epilog) const
        {
          /* calculate vertices relative to ray origin */
          typedef Vec3<vfloat<M>> tsimd3f;
          const tsimd3f O = broadcast<vfloat<M>>(ray.org,k);
          const tsimd3f D = broadcast<vfloat<M>>(ray.dir,k);
          const tsimd3f v0 = tri_v0-O;
          const tsimd3f v1 = tri_v1-O;
          const tsimd3f v2 = tri_v2-O;
          
          /* calculate triangle edges */
          const tsimd3f e0 = v2-v0;
          const tsimd3f e1 = v0-v1;
          const tsimd3f e2 = v1-v2;
          
          /* perform edge tests */
          const vfloat<M> U = dot(cross(v2+v0,e0),D);
          const vfloat<M> V = dot(cross(v0+v1,e1),D);
          const vfloat<M> W = dot(cross(v1+v2,e2),D);
          const vfloat<M> minUVW = min(U,V,W);
          const vfloat<M> maxUVW = max(U,V,W);
#if defined(RTCORE_BACKFACE_CULLING)
          vbool<M> valid = minUVW >= 0.0f;
#else
          vbool<M> valid = (minUVW >= 0.0f) | (maxUVW <= 0.0f);
#endif
          if (unlikely(none(valid))) return false;
          
          /* calculate geometry normal and denominator */
          //const tsimd3f Ng1 = cross(e1,e0);
          const tsimd3f Ng1 = stable_triangle_normal(e2,e1,e0);
          const tsimd3f Ng = Ng1+Ng1;
          const vfloat<M> den = dot(Ng,D);
          const vfloat<M> absDen = abs(den);
          const vfloat<M> sgnDen = signmsk(den);

          /* perform depth test */
          const vfloat<M> T = dot(v0,Ng);
          valid &= ((T^sgnDen) >= absDen*vfloat<M>(ray.tnear[k]));
          valid &= (absDen*vfloat<M>(ray.tfar[k]) >= (T^sgnDen));
          if (unlikely(none(valid))) return false;
          
          /* avoid division by 0 */
          valid &= den != vfloat<M>(zero);
          if (unlikely(none(valid))) return false;
          
          /* calculate hit information */
          return epilog(valid,[&] () {
              const vfloat<M> rcpDen = rcp(den);
              const vfloat<M> t = T * rcpDen;
              vfloat<M> u = U * rcpDen;
              vfloat<M> v = V * rcpDen;
              mapUV(u,v);
              return std::make_tuple(u,v,t,Ng);
            });
        }
      };
    
    /*! Intersects M triangles with 1 ray */
    template<int M, bool filter>
      struct TriangleMvIntersector1Pluecker
      {
        typedef TriangleMv<M> Primitive;
        typedef PlueckerIntersector1<M> Precalculations;
        
        /*! Intersect a ray with the 4 triangles and updates the hit. */
        static __forceinline void intersect(Precalculations& pre, Ray& ray, const Primitive& tri, Scene* scene, const unsigned* geomID_to_instID)
        {
          STAT3(normal.trav_prims,1,1,1);
          pre.intersect(ray,tri.v0,tri.v1,tri.v2,UVIdentity<M>(),Intersect1Epilog<M,filter>(ray,tri.geomIDs,tri.primIDs,scene,geomID_to_instID));
        }
        
        /*! Test if the ray is occluded by one of the triangles. */
        static __forceinline bool occluded(const Precalculations& pre, Ray& ray, const Primitive& tri, Scene* scene, const unsigned* geomID_to_instID)
        {
          STAT3(shadow.trav_prims,1,1,1);
          return pre.intersect(ray,tri.v0,tri.v1,tri.v2,UVIdentity<M>(),Occluded1Epilog<M,filter>(ray,tri.geomIDs,tri.primIDs,scene,geomID_to_instID));
        }
      };
    
    /*! Intersects M triangles with K rays */
    template<int M, int K, bool filter>
      struct TriangleMvIntersectorKPluecker
      {
        typedef TriangleMv<M> Primitive;
        typedef PlueckerIntersectorK<M,K> Precalculations;
        
        /*! Intersects a M rays with N triangles. */
        static __forceinline void intersect(const vbool<K>& valid_i, Precalculations& pre, RayK<K>& ray, const Primitive& tri, Scene* scene)
        {
          for (size_t i=0; i<M; i++)
          {
            if (!tri.valid(i)) break;
            STAT3(normal.trav_prims,1,popcnt(valid_i),RayK<K>::size());
            const Vec3<vfloat<K>> v0 = broadcast<vfloat<K>>(tri.v0,i);
            const Vec3<vfloat<K>> v1 = broadcast<vfloat<K>>(tri.v1,i);
            const Vec3<vfloat<K>> v2 = broadcast<vfloat<K>>(tri.v2,i);
            pre.intersectK(valid_i,ray,v0,v1,v2,UVIdentity<K>(),IntersectKEpilog<M,K,filter>(ray,tri.geomIDs,tri.primIDs,i,scene));
          }
        }
        
        /*! Test for M rays if they are occluded by any of the N triangle. */
        static __forceinline vbool<K> occluded(const vbool<K>& valid_i, Precalculations& pre, RayK<K>& ray, const Primitive& tri, Scene* scene)
        {
          vbool<K> valid0 = valid_i;
          
          for (size_t i=0; i<M; i++)
          {
            if (!tri.valid(i)) break;
            STAT3(shadow.trav_prims,1,popcnt(valid_i),RayK<K>::size());
            const Vec3<vfloat<K>> v0 = broadcast<vfloat<K>>(tri.v0,i);
            const Vec3<vfloat<K>> v1 = broadcast<vfloat<K>>(tri.v1,i);
            const Vec3<vfloat<K>> v2 = broadcast<vfloat<K>>(tri.v2,i);
            pre.intersectK(valid0,ray,v0,v1,v2,UVIdentity<K>(),OccludedKEpilog<M,K,filter>(valid0,ray,tri.geomIDs,tri.primIDs,i,scene));
            if (none(valid0)) break;
          }
          return !valid0;
        }
        
        /*! Intersect a ray with the N triangles and updates the hit. */
        static __forceinline void intersect(Precalculations& pre, RayK<K>& ray, size_t k, const Primitive& tri, Scene* scene)
        {
          STAT3(normal.trav_prims,1,1,1);
          pre.intersect(ray,k,tri.v0,tri.v1,tri.v2,UVIdentity<M>(),Intersect1KEpilog<M,K,filter>(ray,k,tri.geomIDs,tri.primIDs,scene));
        }
        
        /*! Test if the ray is occluded by one of the triangles. */
        static __forceinline bool occluded(Precalculations& pre, RayK<K>& ray, size_t k, const Primitive& tri, Scene* scene)
        {
          STAT3(shadow.trav_prims,1,1,1);
          return pre.intersect(ray,k,tri.v0,tri.v1,tri.v2,UVIdentity<M>(),Occluded1KEpilog<M,K,filter>(ray,k,tri.geomIDs,tri.primIDs,scene));
        }
      };
  }
}
