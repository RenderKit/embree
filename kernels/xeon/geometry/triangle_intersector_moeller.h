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
#include "trianglev_mb.h"
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
      struct MoellerTrumboreHitM
    {
      __forceinline MoellerTrumboreHitM() {}

      __forceinline MoellerTrumboreHitM(const vbool<M>& valid, const vfloat<M>& U, const vfloat<M>& V, const vfloat<M>& T, const vfloat<M>& absDen, const Vec3<vfloat<M>>& Ng)
        : valid(valid), U(U), V(V), T(T), absDen(absDen), ng(Ng) {}
      
      __forceinline void finalize() 
      {
        const vfloat<M> rcpAbsDen = rcp(absDen);
        vt = T * rcpAbsDen;
        vu = U * rcpAbsDen;
        vv = V * rcpAbsDen;
        vNg = ng;
      }
      
      __forceinline Vec2f uv (const size_t i) const { return Vec2f(vu[i],vv[i]); }
      __forceinline float t  (const size_t i) const { return vt[i]; }
      __forceinline Vec3fa Ng(const size_t i) const { return Vec3fa(vNg.x[i],vNg.y[i],vNg.z[i]); }
      
    public:
      vfloat<M> U;
      vfloat<M> V;
      vfloat<M> T;
      vfloat<M> absDen;
      Vec3<vfloat<M>> ng;
      
    public:
      vbool<M> valid;
      vfloat<M> vu;
      vfloat<M> vv;
      vfloat<M> vt;
      Vec3<vfloat<M>> vNg;
    };
    
    template<int M>
      struct MoellerTrumboreIntersector1
    {
      __forceinline MoellerTrumboreIntersector1(const Ray& ray, const void* ptr) {}

      __forceinline bool intersect(Ray& ray, 
                                   const Vec3<vfloat<M>>& tri_v0, 
                                   const Vec3<vfloat<M>>& tri_e1, 
                                   const Vec3<vfloat<M>>& tri_e2, 
                                   const Vec3<vfloat<M>>& tri_Ng,
                                   MoellerTrumboreHitM<M>& hit) const
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
        new (&hit) MoellerTrumboreHitM<M>(valid,U,V,T,absDen,tri_Ng);
        return true;
      }
      
      __forceinline bool intersect(Ray& ray, 
                                   const Vec3<vfloat<M>>& v0, 
                                   const Vec3<vfloat<M>>& v1, 
                                   const Vec3<vfloat<M>>& v2, 
                                   MoellerTrumboreHitM<M>& hit) const
      {
        const Vec3<vfloat<M>> e1 = v0-v1;
        const Vec3<vfloat<M>> e2 = v2-v0;
        const Vec3<vfloat<M>> Ng = cross(e1,e2);
        return intersect(ray,v0,e1,e2,Ng,hit);
      }

      template<typename Epilog>
        __forceinline bool intersect(Ray& ray, 
                                     const Vec3<vfloat<M>>& v0, 
                                     const Vec3<vfloat<M>>& e1, 
                                     const Vec3<vfloat<M>>& e2, 
                                     const Vec3<vfloat<M>>& Ng, 
                                     const Epilog& epilog) const
      {
        MoellerTrumboreHitM<M> hit;
        if (likely(intersect(ray,v0,e1,e2,Ng,hit))) return epilog(hit.valid,hit);
        return false;
      }

      template<typename Epilog>
        __forceinline bool intersect(Ray& ray, 
                                     const Vec3<vfloat<M>>& v0, 
                                     const Vec3<vfloat<M>>& v1, 
                                     const Vec3<vfloat<M>>& v2, 
                                     const Epilog& epilog) const
      {
        MoellerTrumboreHitM<M> hit;
        if (likely(intersect(ray,v0,v1,v2,hit))) return epilog(hit.valid,hit);
        return false;
      }
    };

     template<int K>
      struct MoellerTrumboreHitK
    {
      __forceinline MoellerTrumboreHitK(const vfloat<K>& U, const vfloat<K>& V, const vfloat<K>& T, const vfloat<K>& absDen, const Vec3<vfloat<K>>& Ng)
        : U(U), V(V), T(T), absDen(absDen), Ng(Ng) {}
      
      __forceinline std::tuple<vfloat<K>,vfloat<K>,vfloat<K>,Vec3<vfloat<K>>> operator() () const
      {
        const vfloat<K> rcpAbsDen = rcp(absDen);
        const vfloat<K> t = T * rcpAbsDen;
        const vfloat<K> u = U * rcpAbsDen;
        const vfloat<K> v = V * rcpAbsDen;
        return std::make_tuple(u,v,t,Ng);
      }
      
    private:
      const vfloat<K> U;
      const vfloat<K> V;
      const vfloat<K> T;
      const vfloat<K> absDen;
      const Vec3<vfloat<K>> Ng;
    };
    
     template<int M, int K>
    struct MoellerTrumboreIntersectorK
    {
      __forceinline MoellerTrumboreIntersectorK(const vbool<K>& valid, const RayK<K>& ray) {}
      
      /*! Intersects K rays with one of M triangles. */
      template<typename Epilog>
        __forceinline vbool<K> intersectK(const vbool<K>& valid0, 
                                          RayK<K>& ray, 
                                          const Vec3<vfloat<K>>& tri_v0, 
                                          const Vec3<vfloat<K>>& tri_e1, 
                                          const Vec3<vfloat<K>>& tri_e2, 
                                          const Vec3<vfloat<K>>& tri_Ng, 
                                          const Epilog& epilog) const
      {
        /* ray SIMD type shortcuts */
        typedef Vec3<vfloat<K>> Vec3vfK;
        
        /* calculate denominator */
        vbool<K> valid = valid0;
        const Vec3vfK C = tri_v0 - ray.org;
        const Vec3vfK R = cross(ray.dir,C);
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
        MoellerTrumboreHitK<K> hit(U,V,T,absDen,tri_Ng);
        return epilog(valid,hit);
      }
      
      /*! Intersects K rays with one of M triangles. */
      template<typename Epilog>
      __forceinline vbool<K> intersectK(const vbool<K>& valid0, 
                                        RayK<K>& ray, 
                                        const Vec3<vfloat<K>>& tri_v0, 
                                        const Vec3<vfloat<K>>& tri_v1, 
                                        const Vec3<vfloat<K>>& tri_v2, 
                                        const Epilog& epilog) const
      {
        typedef Vec3<vfloat<K>> Vec3vfK;
        const Vec3vfK e1 = tri_v0-tri_v1;
        const Vec3vfK e2 = tri_v2-tri_v0;
        const Vec3vfK Ng = cross(e1,e2);
        return intersectK(valid0,ray,tri_v0,e1,e2,Ng,epilog);
      }
      
      /*! Intersect k'th ray from ray packet of size K with M triangles. */
      template<typename Epilog>
        __forceinline bool intersect(RayK<K>& ray, 
                                     size_t k,
                                     const Vec3<vfloat<M>>& tri_v0, 
                                     const Vec3<vfloat<M>>& tri_e1, 
                                     const Vec3<vfloat<M>>& tri_e2, 
                                     const Vec3<vfloat<M>>& tri_Ng,
                                     const Epilog& epilog) const
      {
        /* calculate denominator */
        typedef Vec3<vfloat<M>> Vec3vfM;
        const Vec3vfM O = broadcast<vfloat<M>>(ray.org,k);
        const Vec3vfM D = broadcast<vfloat<M>>(ray.dir,k);
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
        valid &= (T > absDen*vfloat<M>(ray.tnear[k])) & (T < absDen*vfloat<M>(ray.tfar[k]));
        if (likely(none(valid))) return false;
        
        /* calculate hit information */
        MoellerTrumboreHitM<M> hit(valid,U,V,T,absDen,tri_Ng);
        return epilog(valid,hit);
      }
      
      template<typename Epilog>
      __forceinline bool intersect1(RayK<K>& ray, 
                                    size_t k,
                                    const Vec3<vfloat<M>>& v0, 
                                    const Vec3<vfloat<M>>& v1, 
                                    const Vec3<vfloat<M>>& v2, 
                                    const Epilog& epilog) const
      {
        const Vec3<vfloat<M>> e1 = v0-v1;
        const Vec3<vfloat<M>> e2 = v2-v0;
        const Vec3<vfloat<M>> Ng = cross(e1,e2);
        return intersect(ray,k,v0,e1,e2,Ng,epilog);
      }
    };

    /*! Intersects M triangles with 1 ray */
    template<int M, int Mx, bool filter>
      struct TriangleMIntersector1MoellerTrumbore
      {
        typedef TriangleM<M> Primitive;
        typedef MoellerTrumboreIntersector1<Mx> Precalculations;
        
        /*! Intersect a ray with the M triangles and updates the hit. */
        static __forceinline void intersect(const Precalculations& pre, Ray& ray, const TriangleM<M>& tri, Scene* scene, const unsigned* geomID_to_instID)
        {
          STAT3(normal.trav_prims,1,1,1);
          pre.intersect(ray,tri.v0,tri.e1,tri.e2,tri.Ng,Intersect1Epilog<M,Mx,filter>(ray,tri.geomIDs,tri.primIDs,scene,geomID_to_instID));
        }
        
        /*! Test if the ray is occluded by one of M triangles. */
        static __forceinline bool occluded(const Precalculations& pre, Ray& ray, const TriangleM<M>& tri, Scene* scene, const unsigned* geomID_to_instID)
        {
          STAT3(shadow.trav_prims,1,1,1);
          return pre.intersect(ray,tri.v0,tri.e1,tri.e2,tri.Ng,Occluded1Epilog<M,Mx,filter>(ray,tri.geomIDs,tri.primIDs,scene,geomID_to_instID));
        }
      };

    /*! Intersects M triangles with K rays. */
    template<int M, int Mx, int K, bool filter>
      struct TriangleMIntersectorKMoellerTrumbore
      {
        typedef TriangleM<M> Primitive;
        typedef MoellerTrumboreIntersectorK<Mx,K> Precalculations;
        
        /*! Intersects K rays with M triangles. */
        static __forceinline void intersect(const vbool<K>& valid_i, Precalculations& pre, RayK<K>& ray, const TriangleM<M>& tri, Scene* scene)
        {
          for (size_t i=0; i<TriangleM<M>::max_size(); i++)
          {
            if (!tri.valid(i)) break;
            STAT3(normal.trav_prims,1,popcnt(valid_i),K);
            const Vec3<vfloat<K>> p0 = broadcast<vfloat<K>>(tri.v0,i);
            const Vec3<vfloat<K>> e1 = broadcast<vfloat<K>>(tri.e1,i);
            const Vec3<vfloat<K>> e2 = broadcast<vfloat<K>>(tri.e2,i);
            const Vec3<vfloat<K>> Ng = broadcast<vfloat<K>>(tri.Ng,i);
            pre.intersectK(valid_i,ray,p0,e1,e2,Ng,IntersectKEpilog<M,K,filter>(ray,tri.geomIDs,tri.primIDs,i,scene));
          }
        }
        
        /*! Test for K rays if they are occluded by any of the M triangles. */
        static __forceinline vbool<K> occluded(const vbool<K>& valid_i, Precalculations& pre, RayK<K>& ray, const TriangleM<M>& tri, Scene* scene)
        {
          vbool<K> valid0 = valid_i;
          
          for (size_t i=0; i<TriangleM<M>::max_size(); i++)
          {
            if (!tri.valid(i)) break;
            STAT3(shadow.trav_prims,1,popcnt(valid0),K);
            const Vec3<vfloat<K>> p0 = broadcast<vfloat<K>>(tri.v0,i);
            const Vec3<vfloat<K>> e1 = broadcast<vfloat<K>>(tri.e1,i);
            const Vec3<vfloat<K>> e2 = broadcast<vfloat<K>>(tri.e2,i);
            const Vec3<vfloat<K>> Ng = broadcast<vfloat<K>>(tri.Ng,i);
            pre.intersectK(valid0,ray,p0,e1,e2,Ng,OccludedKEpilog<M,K,filter>(valid0,ray,tri.geomIDs,tri.primIDs,i,scene));
            if (none(valid0)) break;
          }
          return !valid0;
        }
        
        /*! Intersect a ray with M triangles and updates the hit. */
        static __forceinline void intersect(Precalculations& pre, RayK<K>& ray, size_t k, const TriangleM<M>& tri, Scene* scene)
        {
          STAT3(normal.trav_prims,1,1,1);
          pre.intersect(ray,k,tri.v0,tri.e1,tri.e2,tri.Ng,Intersect1KEpilog<M,Mx,K,filter>(ray,k,tri.geomIDs,tri.primIDs,scene));
        }
        
        /*! Test if the ray is occluded by one of the M triangles. */
        static __forceinline bool occluded(Precalculations& pre, RayK<K>& ray, size_t k, const TriangleM<M>& tri, Scene* scene)
        {
          STAT3(shadow.trav_prims,1,1,1);
          return pre.intersect(ray,k,tri.v0,tri.e1,tri.e2,tri.Ng,Occluded1KEpilog<M,Mx,K,filter>(ray,k,tri.geomIDs,tri.primIDs,scene));
        }
      };

    
    /*! Intersects M motion blur triangles with 1 ray */
    template<int M, int Mx, bool filter>
      struct TriangleMvMBIntersector1MoellerTrumbore
      {
        typedef TriangleMvMB<M> Primitive;
        typedef MoellerTrumboreIntersector1<Mx> Precalculations;
        
        /*! Intersect a ray with the M triangles and updates the hit. */
        static __forceinline void intersect(const Precalculations& pre, Ray& ray, const TriangleMvMB<M>& tri, Scene* scene, const unsigned* geomID_to_instID)
        {
          STAT3(normal.trav_prims,1,1,1);
          const Vec3<vfloat<Mx>> time(ray.time);
          const Vec3<vfloat<Mx>> v0 = madd(time,Vec3<vfloat<Mx>>(tri.dv0),Vec3<vfloat<Mx>>(tri.v0));
          const Vec3<vfloat<Mx>> v1 = madd(time,Vec3<vfloat<Mx>>(tri.dv1),Vec3<vfloat<Mx>>(tri.v1));
          const Vec3<vfloat<Mx>> v2 = madd(time,Vec3<vfloat<Mx>>(tri.dv2),Vec3<vfloat<Mx>>(tri.v2));
          pre.intersect(ray,v0,v1,v2,Intersect1Epilog<M,Mx,filter>(ray,tri.geomIDs,tri.primIDs,scene,geomID_to_instID)); 
        }
        
        /*! Test if the ray is occluded by one of M triangles. */
        static __forceinline bool occluded(const Precalculations& pre, Ray& ray, const TriangleMvMB<M>& tri, Scene* scene, const unsigned* geomID_to_instID)
        {
          STAT3(shadow.trav_prims,1,1,1);
          const Vec3<vfloat<Mx>> time(ray.time);
          const Vec3<vfloat<Mx>> v0 = madd(time,Vec3<vfloat<Mx>>(tri.dv0),Vec3<vfloat<Mx>>(tri.v0));
          const Vec3<vfloat<Mx>> v1 = madd(time,Vec3<vfloat<Mx>>(tri.dv1),Vec3<vfloat<Mx>>(tri.v1));
          const Vec3<vfloat<Mx>> v2 = madd(time,Vec3<vfloat<Mx>>(tri.dv2),Vec3<vfloat<Mx>>(tri.v2));
          return pre.intersect(ray,v0,v1,v2,Occluded1Epilog<M,Mx,filter>(ray,tri.geomIDs,tri.primIDs,scene,geomID_to_instID)); 
        }
      };
    
    /*! Intersects M motion blur triangles with K rays. */
    template<int M, int Mx, int K, bool filter>
      struct TriangleMvMBIntersectorKMoellerTrumbore
      {
        typedef TriangleMvMB<M> Primitive;
        typedef MoellerTrumboreIntersectorK<Mx,K> Precalculations;
        
        /*! Intersects K rays with M triangles. */
        static __forceinline void intersect(const vbool<K>& valid_i, Precalculations& pre, RayK<K>& ray, const TriangleMvMB<M>& tri, Scene* scene)
        {
          for (size_t i=0; i<TriangleMvMB<M>::max_size(); i++)
          {
            if (!tri.valid(i)) break;
            STAT3(normal.trav_prims,1,popcnt(valid_i),K);
            const Vec3<vfloat<K>> time(ray.time);
            const Vec3<vfloat<K>> v0 = madd(time,broadcast<vfloat<K>>(tri.dv0,i),broadcast<vfloat<K>>(tri.v0,i));
            const Vec3<vfloat<K>> v1 = madd(time,broadcast<vfloat<K>>(tri.dv1,i),broadcast<vfloat<K>>(tri.v1,i));
            const Vec3<vfloat<K>> v2 = madd(time,broadcast<vfloat<K>>(tri.dv2,i),broadcast<vfloat<K>>(tri.v2,i));
            pre.intersectK(valid_i,ray,v0,v1,v2,IntersectKEpilog<M,K,filter>(ray,tri.geomIDs,tri.primIDs,i,scene));
          }
        }
        
        /*! Test for K rays if they are occluded by any of the M triangles. */
        static __forceinline vbool<K> occluded(const vbool<K>& valid_i, Precalculations& pre, RayK<K>& ray, const TriangleMvMB<M>& tri, Scene* scene)
        {
          vbool<K> valid0 = valid_i;
          
          for (size_t i=0; i<TriangleMvMB<M>::max_size(); i++)
          {
            if (!tri.valid(i)) break;
            STAT3(shadow.trav_prims,1,popcnt(valid0),K);
            const Vec3<vfloat<K>> time(ray.time);
            const Vec3<vfloat<K>> v0 = madd(time,broadcast<vfloat<K>>(tri.dv0,i),broadcast<vfloat<K>>(tri.v0,i));
            const Vec3<vfloat<K>> v1 = madd(time,broadcast<vfloat<K>>(tri.dv1,i),broadcast<vfloat<K>>(tri.v1,i));
            const Vec3<vfloat<K>> v2 = madd(time,broadcast<vfloat<K>>(tri.dv2,i),broadcast<vfloat<K>>(tri.v2,i));
            pre.intersectK(valid0,ray,v0,v1,v2,OccludedKEpilog<M,K,filter>(valid0,ray,tri.geomIDs,tri.primIDs,i,scene));
            if (none(valid0)) break;
          }
          return !valid0;
        }
        
        /*! Intersect a ray with M triangles and updates the hit. */
        static __forceinline void intersect(Precalculations& pre, RayK<K>& ray, size_t k, const TriangleMvMB<M>& tri, Scene* scene)
        {
          STAT3(normal.trav_prims,1,1,1);
          const Vec3<vfloat<Mx>> time(ray.time[k]);
          const Vec3<vfloat<Mx>> v0 = madd(time,Vec3<vfloat<Mx>>(tri.dv0),Vec3<vfloat<Mx>>(tri.v0));
          const Vec3<vfloat<Mx>> v1 = madd(time,Vec3<vfloat<Mx>>(tri.dv1),Vec3<vfloat<Mx>>(tri.v1));
          const Vec3<vfloat<Mx>> v2 = madd(time,Vec3<vfloat<Mx>>(tri.dv2),Vec3<vfloat<Mx>>(tri.v2));
          pre.intersect1(ray,k,v0,v1,v2,Intersect1KEpilog<M,Mx,K,filter>(ray,k,tri.geomIDs,tri.primIDs,scene)); 
        }
        
        /*! Test if the ray is occluded by one of the M triangles. */
        static __forceinline bool occluded(Precalculations& pre, RayK<K>& ray, size_t k, const TriangleMvMB<M>& tri, Scene* scene)
        {
          STAT3(shadow.trav_prims,1,1,1);
          const Vec3<vfloat<Mx>> time(ray.time[k]);
          const Vec3<vfloat<Mx>> v0 = madd(time,Vec3<vfloat<Mx>>(tri.dv0),Vec3<vfloat<Mx>>(tri.v0));
          const Vec3<vfloat<Mx>> v1 = madd(time,Vec3<vfloat<Mx>>(tri.dv1),Vec3<vfloat<Mx>>(tri.v1));
          const Vec3<vfloat<Mx>> v2 = madd(time,Vec3<vfloat<Mx>>(tri.dv2),Vec3<vfloat<Mx>>(tri.v2));
          return pre.intersect1(ray,k,v0,v1,v2,Occluded1KEpilog<M,Mx,K,filter>(ray,k,tri.geomIDs,tri.primIDs,scene)); 
        }
      };
  }
}
