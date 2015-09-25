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

#include "intersector_epilog.h"

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
    template<int M>
      struct MoellerTrumboreIntersector1
    {
      struct Precalculations {
        __forceinline Precalculations (const Ray& ray, const void* ptr) {}
      };

      template<typename Epilog>
      __forceinline static bool intersect(Ray& ray, 
                                          const Vec3<vfloat<M>>& tri_v0, 
                                          const Vec3<vfloat<M>>& tri_e1, 
                                          const Vec3<vfloat<M>>& tri_e2, 
                                          const Vec3<vfloat<M>>& tri_Ng,
                                          const Epilog& epilog)
      {
        /* calculate denominator */
        typedef Vec3<vfloat<M>> Vec3vfM;
        const Vec3vfM O = Vec3vfM(ray.org);
        const Vec3vfM D = Vec3vfM(ray.dir);
        const Vec3vfM C = Vec3vfM(tri_v0) - O;
        const Vec3vfM R = cross(D,C);
        const vfloat<M> den = dot(Vec3vfM(tri_Ng),D);
        const vfloat<M> absDen = abs(den);
        const vfloat<M> sgnDen = signmsk(den);
        
        /* perform edge tests */
        const vfloat<M> U = dot(R,Vec3vfM(tri_e2)) ^ sgnDen;
        const vfloat<M> V = dot(R,Vec3vfM(tri_e1)) ^ sgnDen;
        
        /* perform backface culling */
#if defined(RTCORE_BACKFACE_CULLING)
        vbool<M> valid = (den > vfloat<M>(zero)) & (U >= 0.0f) & (V >= 0.0f) & (U+V<=absDen);
#else
        vbool<M> valid = (den != vfloat<M>(zero)) & (U >= 0.0f) & (V >= 0.0f) & (U+V<=absDen);
#endif
        if (likely(none(valid))) return false;
        
        /* perform depth test */
        const vfloat<M> T = dot(Vec3vfM(tri_Ng),C) ^ sgnDen;
        valid &= (T > absDen*vfloat<M>(ray.tnear)) & (T < absDen*vfloat<M>(ray.tfar));
        if (likely(none(valid))) return false;
        
        /* update hit information */
        return epilog(valid,[&] () {
            const vfloat<M> rcpAbsDen = rcp(absDen);
            const vfloat<M> u = U * rcpAbsDen;
            const vfloat<M> v = V * rcpAbsDen;
            const vfloat<M> t = T * rcpAbsDen;
            return std::make_tuple(u,v,t,tri_Ng);
          });
      }
      
      template<typename Epilog>
        __forceinline static bool intersect(Ray& ray, 
                                            const Vec3<vfloat<M>>& v0, 
                                            const Vec3<vfloat<M>>& v1, 
                                            const Vec3<vfloat<M>>& v2, 
                                            const Epilog& epilog)
      {
        const Vec3<vfloat<M>> e1 = v0-v1;
        const Vec3<vfloat<M>> e2 = v2-v0;
        const Vec3<vfloat<M>> Ng = cross(e1,e2);
        return intersect(ray,v0,e1,e2,Ng,epilog);
      }
    };
    
    template<int K, int M>
    struct MoellerTrumboreIntersectorK
    {
      struct Precalculations {
        __forceinline Precalculations (const vbool<K>& valid, const RayK<K>& ray) {}
      };
      
      /*! Intersects K rays with one of M triangles. */
      template<typename Epilog>
        __forceinline static vbool<K> intersectK(const vbool<K>& valid0, 
                                                 RayK<K>& ray, 
                                                 const Vec3<vfloat<K>>& tri_v0, 
                                                 const Vec3<vfloat<K>>& tri_e1, 
                                                 const Vec3<vfloat<K>>& tri_e2, 
                                                 const Vec3<vfloat<K>>& tri_Ng, 
                                                 const Epilog& epilog)
      {
        /* ray SIMD type shortcuts */
        typedef Vec3<vfloat<K>> rsimd3f;
        
        /* calculate denominator */
        vbool<K> valid = valid0;
        const rsimd3f C = tri_v0 - ray.org;
        const rsimd3f R = cross(ray.dir,C);
        const vfloat<K> den = dot(tri_Ng,ray.dir);
        const vfloat<K> absDen = abs(den);
        const vfloat<K> sgnDen = signmsk(den);
        
        /* test against edge p2 p0 */
        const vfloat<K> U = dot(R,tri_e2) ^ sgnDen;
        valid &= U >= 0.0f;
        if (likely(none(valid))) return false;
        
        /* test against edge p0 p1 */
        const vfloat<K> V = dot(R,tri_e1) ^ sgnDen;
        valid &= V >= 0.0f;
        if (likely(none(valid))) return false;
        
        /* test against edge p1 p2 */
        const vfloat<K> W = absDen-U-V;
        valid &= W >= 0.0f;
        if (likely(none(valid))) return false;
        
        /* perform depth test */
        const vfloat<K> T = dot(tri_Ng,C) ^ sgnDen;
        valid &= (T >= absDen*ray.tnear) & (absDen*ray.tfar >= T);
        if (unlikely(none(valid))) return false;
        
        /* perform backface culling */
#if defined(RTCORE_BACKFACE_CULLING)
        valid &= den > vfloat<K>(zero);
        if (unlikely(none(valid))) return false;
#else
        valid &= den != vfloat<K>(zero);
        if (unlikely(none(valid))) return false;
#endif
        
        /* calculate hit information */
        return epilog(valid,[&] () {
            const vfloat<K> rcpAbsDen = rcp(absDen);
            const vfloat<K> u = U*rcpAbsDen;
            const vfloat<K> v = V*rcpAbsDen;
            const vfloat<K> t = T*rcpAbsDen;
            return std::make_tuple(u,v,t,tri_Ng);
          });
      }
      
      /*! Intersects K rays with one of M triangles. */
      template<typename Epilog>
      __forceinline static vbool<K> intersectK(const vbool<K>& valid0, 
                                               RayK<K>& ray, 
                                               const Vec3<vfloat<K>>& tri_v0, 
                                               const Vec3<vfloat<K>>& tri_v1, 
                                               const Vec3<vfloat<K>>& tri_v2, 
                                               const Epilog& epilog)
      {
        typedef Vec3<vfloat<K>> tsimd3f;
        const tsimd3f e1 = tri_v0-tri_v1;
        const tsimd3f e2 = tri_v2-tri_v0;
        const tsimd3f Ng = cross(e1,e2);
        return intersectK(valid0,ray,tri_v0,e1,e2,Ng,epilog);
      }
      
      /*! Intersect k'th ray from ray packet of size K with M triangles. */
      template<typename Epilog>
        __forceinline static bool intersect(RayK<K>& ray, size_t k,
                                            const Vec3<vfloat<M>>& tri_v0, 
                                            const Vec3<vfloat<M>>& tri_e1, 
                                            const Vec3<vfloat<M>>& tri_e2, 
                                            const Vec3<vfloat<M>>& tri_Ng,
                                            const Epilog& epilog)
      {
        /* calculate denominator */
        typedef Vec3<vfloat<M>> tsimd3f;
        const tsimd3f O = broadcast<vfloat<M>>(ray.org,k);
        const tsimd3f D = broadcast<vfloat<M>>(ray.dir,k);
        const tsimd3f C = tsimd3f(tri_v0) - O;
        const tsimd3f R = cross(D,C);
        const vfloat<M> den = dot(tsimd3f(tri_Ng),D);
        const vfloat<M> absDen = abs(den);
        const vfloat<M> sgnDen = signmsk(den);
        
        /* perform edge tests */
        const vfloat<M> U = dot(R,tsimd3f(tri_e2)) ^ sgnDen;
        const vfloat<M> V = dot(R,tsimd3f(tri_e1)) ^ sgnDen;
        
        /* perform backface culling */
#if defined(RTCORE_BACKFACE_CULLING)
        vbool<M> valid = (den > vfloat<M>(zero)) & (U >= 0.0f) & (V >= 0.0f) & (U+V<=absDen);
#else
        vbool<M> valid = (den != vfloat<M>(zero)) & (U >= 0.0f) & (V >= 0.0f) & (U+V<=absDen);
#endif
        if (likely(none(valid))) return false;
        
        /* perform depth test */
        const vfloat<M> T = dot(tsimd3f(tri_Ng),C) ^ sgnDen;
        valid &= (T > absDen*vfloat<M>(ray.tnear[k])) & (T < absDen*vfloat<M>(ray.tfar[k]));
        if (likely(none(valid))) return false;
        
        /* calculate hit information */
        return epilog(valid,[&] () {
            const vfloat<M> rcpAbsDen = rcp(absDen);
            const vfloat<M> u = U * rcpAbsDen;
            const vfloat<M> v = V * rcpAbsDen;
            const vfloat<M> t = T * rcpAbsDen;
            return std::make_tuple(u,v,t,tri_Ng);
          });
      }
      
      template<typename Epilog>
      __forceinline static bool intersect1(RayK<K>& ray, size_t k,
                                           const Vec3<vfloat<M>>& v0, 
                                           const Vec3<vfloat<M>>& v1, 
                                           const Vec3<vfloat<M>>& v2, 
                                           const Epilog& epilog)
      {
        const Vec3<vfloat<M>> e1 = v0-v1;
        const Vec3<vfloat<M>> e2 = v2-v0;
        const Vec3<vfloat<M>> Ng = cross(e1,e2);
        return intersect(ray,k,v0,e1,e2,Ng,epilog);
      }
    };

    /*! Intersects N triangles with 1 ray */
    template<typename TriangleN, bool enableIntersectionFilter>
      struct TriangleNIntersector1MoellerTrumbore
      {
        enum { M = TriangleN::M };
        typedef TriangleN Primitive;
        typedef typename MoellerTrumboreIntersector1<M>::Precalculations Precalculations;
        
        /*! Intersect a ray with the N triangles and updates the hit. */
        static __forceinline void intersect(const Precalculations& pre, Ray& ray, const TriangleN& tri, Scene* scene, const unsigned* geomID_to_instID)
        {
          STAT3(normal.trav_prims,1,1,1);
          MoellerTrumboreIntersector1<M>::intersect(ray,tri.v0,tri.e1,tri.e2,tri.Ng,
                                                    Intersect1Epilog<M,enableIntersectionFilter>(ray,tri.geomIDs,tri.primIDs,scene,geomID_to_instID));
        }
        
        /*! Test if the ray is occluded by one of N triangles. */
        static __forceinline bool occluded(const Precalculations& pre, Ray& ray, const TriangleN& tri, Scene* scene, const unsigned* geomID_to_instID)
        {
          STAT3(shadow.trav_prims,1,1,1);
          return MoellerTrumboreIntersector1<M>::intersect(ray,tri.v0,tri.e1,tri.e2,tri.Ng,
                                                            Occluded1Epilog<M,enableIntersectionFilter>(ray,tri.geomIDs,tri.primIDs,scene,geomID_to_instID));
        }
      };

    /*! Intersector for M triangles with K rays. */
    template<typename RayK, typename TriangleM, bool enableIntersectionFilter>
      struct TriangleNIntersectorMMoellerTrumbore
      {
        enum { K = RayK::K };
        enum { M = TriangleM::M };
        typedef TriangleM Primitive;
        typedef typename MoellerTrumboreIntersectorK<K,M>::Precalculations Precalculations;
        
        /*! Intersects a M rays with N triangles. */
        static __forceinline void intersect(const vbool<K>& valid_i, Precalculations& pre, RayK& ray, const TriangleM& tri, Scene* scene)
        {
          for (size_t i=0; i<TriangleM::max_size(); i++)
          {
            if (!tri.valid(i)) break;
            STAT3(normal.trav_prims,1,popcnt(valid_i),RayK::size());
            const Vec3<vfloat<K>> p0 = broadcast<vfloat<K>>(tri.v0,i);
            const Vec3<vfloat<K>> e1 = broadcast<vfloat<K>>(tri.e1,i);
            const Vec3<vfloat<K>> e2 = broadcast<vfloat<K>>(tri.e2,i);
            const Vec3<vfloat<K>> Ng = broadcast<vfloat<K>>(tri.Ng,i);
            MoellerTrumboreIntersectorK<K,M>::intersectK(valid_i,ray,p0,e1,e2,Ng,
                                                         IntersectKEpilog<K,M,enableIntersectionFilter>(ray,tri.geomIDs,tri.primIDs,i,scene));
          }
        }
        
        /*! Test for M rays if they are occluded by any of the N triangle. */
        static __forceinline vbool<K> occluded(const vbool<K>& valid_i, Precalculations& pre, RayK& ray, const TriangleM& tri, Scene* scene)
        {
          vbool<K> valid0 = valid_i;
          
          for (size_t i=0; i<TriangleM::max_size(); i++)
          {
            if (!tri.valid(i)) break;
            STAT3(shadow.trav_prims,1,popcnt(valid0),RayK::size());
            const Vec3<vfloat<K>> p0 = broadcast<vfloat<K>>(tri.v0,i);
            const Vec3<vfloat<K>> e1 = broadcast<vfloat<K>>(tri.e1,i);
            const Vec3<vfloat<K>> e2 = broadcast<vfloat<K>>(tri.e2,i);
            const Vec3<vfloat<K>> Ng = broadcast<vfloat<K>>(tri.Ng,i);
            MoellerTrumboreIntersectorK<K,M>::intersectK(valid0,ray,p0,e1,e2,Ng,
                                                         OccludedKEpilog<K,M,enableIntersectionFilter>(valid0,ray,tri.geomIDs,tri.primIDs,i,scene));
            if (none(valid0)) break;
          }
          return !valid0;
        }
        
        /*! Intersect a ray with the 4 triangles and updates the hit. */
        static __forceinline void intersect(Precalculations& pre, RayK& ray, size_t k, const TriangleM& tri, Scene* scene)
        {
          STAT3(normal.trav_prims,1,1,1);
          MoellerTrumboreIntersectorK<K,M>::intersect(ray,k,tri.v0,tri.e1,tri.e2,tri.Ng,
                                                     Intersect1KEpilog<K,M,enableIntersectionFilter>(ray,k,tri.geomIDs,tri.primIDs,scene));
        }
        
        /*! Test if the ray is occluded by one of the triangles. */
        static __forceinline bool occluded(Precalculations& pre, RayK& ray, size_t k, const TriangleM& tri, Scene* scene)
        {
          STAT3(shadow.trav_prims,1,1,1);
          return MoellerTrumboreIntersectorK<K,M>::intersect(ray,k,tri.v0,tri.e1,tri.e2,tri.Ng,
                                                             Occluded1KEpilog<K,M,enableIntersectionFilter>(ray,k,tri.geomIDs,tri.primIDs,scene));
        }
      };
    
    /*! Intersects N triangles with 1 ray */
    template<typename TriangleNMblur, bool enableIntersectionFilter>
      struct TriangleNMblurIntersector1MoellerTrumbore
      {
        enum { M = TriangleNMblur::M };
        typedef TriangleNMblur Primitive;
        typedef typename MoellerTrumboreIntersector1<M>::Precalculations Precalculations;
        
        /*! Intersect a ray with the N triangles and updates the hit. */
        static __forceinline void intersect(const Precalculations& pre, Ray& ray, const TriangleNMblur& tri, Scene* scene, const unsigned* geomID_to_instID)
        {
          STAT3(normal.trav_prims,1,1,1);
          const vfloat<M> time = ray.time;
          const Vec3<vfloat<M>> v0 = tri.v0 + time*tri.dv0;
          const Vec3<vfloat<M>> v1 = tri.v1 + time*tri.dv1;
          const Vec3<vfloat<M>> v2 = tri.v2 + time*tri.dv2;
          MoellerTrumboreIntersector1<M>::intersect(ray,v0,v1,v2,
                                                    Intersect1Epilog<M,enableIntersectionFilter>(ray,tri.geomIDs,tri.primIDs,scene,geomID_to_instID));
        }
        
        /*! Test if the ray is occluded by one of N triangles. */
        static __forceinline bool occluded(const Precalculations& pre, Ray& ray, const TriangleNMblur& tri, Scene* scene, const unsigned* geomID_to_instID)
        {
          STAT3(shadow.trav_prims,1,1,1);
          const vfloat<M> time = ray.time;
          const Vec3<vfloat<M>> v0 = tri.v0 + time*tri.dv0;
          const Vec3<vfloat<M>> v1 = tri.v1 + time*tri.dv1;
          const Vec3<vfloat<M>> v2 = tri.v2 + time*tri.dv2;
          return MoellerTrumboreIntersector1<M>::intersect(ray,v0,v1,v2,
                                                           Occluded1Epilog<M,enableIntersectionFilter>(ray,tri.geomIDs,tri.primIDs,scene,geomID_to_instID));
        }
      };
    
    /*! Intersector for M triangles with K rays. */
    template<typename RayK, typename TriangleMMblur, bool enableIntersectionFilter>
      struct TriangleNMblurIntersectorMMoellerTrumbore
      {
        enum { K = RayK::K };
        enum { M = TriangleMMblur::M };
        typedef TriangleMMblur Primitive;
        typedef typename MoellerTrumboreIntersectorK<K,M>::Precalculations Precalculations;
        
        /*! Intersects a M rays with N triangles. */
        static __forceinline void intersect(const vbool<K>& valid_i, Precalculations& pre, RayK& ray, const TriangleMMblur& tri, Scene* scene)
        {
          for (size_t i=0; i<TriangleMMblur::max_size(); i++)
          {
            if (!tri.valid(i)) break;
            STAT3(normal.trav_prims,1,popcnt(valid_i),RayK::size());
            const vfloat<K> time = ray.time;
            const Vec3<vfloat<K>> v0 = broadcast<vfloat<K>>(tri.v0,i) + time*broadcast<vfloat<K>>(tri.dv0,i);
            const Vec3<vfloat<K>> v1 = broadcast<vfloat<K>>(tri.v1,i) + time*broadcast<vfloat<K>>(tri.dv1,i);
            const Vec3<vfloat<K>> v2 = broadcast<vfloat<K>>(tri.v2,i) + time*broadcast<vfloat<K>>(tri.dv2,i);
            MoellerTrumboreIntersectorK<K,M>::intersectK(valid_i,ray,v0,v1,v2,
                                                         IntersectKEpilog<K,M,enableIntersectionFilter>(ray,tri.geomIDs,tri.primIDs,i,scene));
          }
        }
        
        /*! Test for M rays if they are occluded by any of the N triangle. */
        static __forceinline vbool<K> occluded(const vbool<K>& valid_i, Precalculations& pre, RayK& ray, const TriangleMMblur& tri, Scene* scene)
        {
          vbool<K> valid0 = valid_i;
          
          for (size_t i=0; i<TriangleMMblur::max_size(); i++)
          {
            if (!tri.valid(i)) break;
            STAT3(shadow.trav_prims,1,popcnt(valid0),RayK::size());
            const vfloat<K> time = ray.time;
            const Vec3<vfloat<K>> v0 = broadcast<vfloat<K>>(tri.v0,i) + time*broadcast<vfloat<K>>(tri.dv0,i);
            const Vec3<vfloat<K>> v1 = broadcast<vfloat<K>>(tri.v1,i) + time*broadcast<vfloat<K>>(tri.dv1,i);
            const Vec3<vfloat<K>> v2 = broadcast<vfloat<K>>(tri.v2,i) + time*broadcast<vfloat<K>>(tri.dv2,i);
            MoellerTrumboreIntersectorK<K,M>::intersectK(valid0,ray,v0,v1,v2,
                                                         OccludedKEpilog<K,M,enableIntersectionFilter>(valid0,ray,tri.geomIDs,tri.primIDs,i,scene));
            if (none(valid0)) break;
          }
          return !valid0;
        }
        
        /*! Intersect a ray with the N triangles and updates the hit. */
        static __forceinline void intersect(Precalculations& pre, RayK& ray, size_t k, const TriangleMMblur& tri, Scene* scene)
        {
          STAT3(normal.trav_prims,1,1,1);
          const vfloat<M> time = broadcast<vfloat<M>>(ray.time,k);
          const Vec3<vfloat<M>> v0 = tri.v0 + time*tri.dv0;
          const Vec3<vfloat<M>> v1 = tri.v1 + time*tri.dv1;
          const Vec3<vfloat<M>> v2 = tri.v2 + time*tri.dv2;
          MoellerTrumboreIntersectorK<K,M>::intersect(ray,k,v0,v1,v2,
                                                      Intersect1KEpilog<K,M,enableIntersectionFilter>(ray,k,tri.geomIDs,tri.primIDs,scene));
        }
        
        /*! Test if the ray is occluded by one of the N triangles. */
        static __forceinline bool occluded(Precalculations& pre, RayK& ray, size_t k, const TriangleMMblur& tri, Scene* scene)
        {
          STAT3(shadow.trav_prims,1,1,1);
          const vfloat<M> time = broadcast<vfloat<M>>(ray.time,k);
          const Vec3<vfloat<M>> v0 = tri.v0 + time*tri.dv0;
          const Vec3<vfloat<M>> v1 = tri.v1 + time*tri.dv1;
          const Vec3<vfloat<M>> v2 = tri.v2 + time*tri.dv2;
          return MoellerTrumboreIntersectorK<K,M>::intersect(ray,k,v0,v1,v2,
                                                             Occluded1KEpilog<K,M,enableIntersectionFilter>(ray,k,tri.geomIDs,tri.primIDs,scene));
        }
      };
  }
}
