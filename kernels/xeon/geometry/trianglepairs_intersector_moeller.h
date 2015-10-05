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
#include "trianglepairsv.h"
#include "trianglev_mb.h"
#include "intersector_epilog.h"
#include "triangle_intersector_moeller.h"

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
      struct MoellerTrumboreIntersectorPairs1
    {
      __forceinline MoellerTrumboreIntersectorPairs1(const Ray& ray, const void* ptr) {}

      template<typename Epilog>
      __forceinline bool intersect(Ray& ray, 
                                   const Vec3<vfloat<M>>& tri_v0, 
                                   const Vec3<vfloat<M>>& tri_e1, 
                                   const Vec3<vfloat<M>>& tri_e2, 
                                   const Vec3<vfloat<M>>& tri_Ng,
                                   const Epilog& epilog) const
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
        return epilog(valid,[&] (const vbool<M> &valid, const vint<M>& flags) {
            const vfloat<M> rcpAbsDen = rcp(absDen);
            const vfloat<M> t = T * rcpAbsDen;
            const size_t i = select_min(valid,t);
            const vfloat<M> u = U * rcpAbsDen;
            const vfloat<M> v = V * rcpAbsDen;
            const vfloat<M> w = max(1.0f - u - v,vfloat<M>(zero));
            const vfloat<M> uvw[3] = { u,v,w };
            const unsigned int indexU = (((unsigned int)flags[i]) >>  0) & 0xff;
            const unsigned int indexV = (((unsigned int)flags[i]) >> 16) & 0xff;
          
          /* update hit information */
            const vfloat<M> uu = uvw[indexU];
            const vfloat<M> vv = uvw[indexV];
            return std::make_tuple(uu,vv,t,tri_Ng,i);
          });
      }
      
      template<typename Epilog>
        __forceinline bool intersect(Ray& ray, 
                                     const Vec3<vfloat<M>>& v0, 
                                     const Vec3<vfloat<M>>& v1, 
                                     const Vec3<vfloat<M>>& v2, 
                                     const Epilog& epilog) const
      {
        const Vec3<vfloat<M>> e1 = v0-v1;
        const Vec3<vfloat<M>> e2 = v2-v0;
        const Vec3<vfloat<M>> Ng = cross(e1,e2);
        return intersect(ray,v0,e1,e2,Ng,epilog);
      }
    };

    template<int M, bool filter>
      struct IntersectPairs1Epilog
      {
        Ray& ray;
        const vint<M>& geomIDs;
        const vint<M>& primIDs;
        const vint<M>& flags;
        Scene* scene;
        const unsigned* geomID_to_instID;
        
        __forceinline IntersectPairs1Epilog(Ray& ray,
                                            const vint<M>& geomIDs, 
                                            const vint<M>& primIDs, 
                                            const vint<M>& flags, 
                                            Scene* scene,
                                            const unsigned* geomID_to_instID)
          : ray(ray), geomIDs(geomIDs), primIDs(primIDs), flags(flags), scene(scene), geomID_to_instID(geomID_to_instID) {}
        
        template<typename Hit>
        __forceinline bool operator() (const vbool<M>& valid_i, const Hit& hit) const
        {
          vfloat<M> u,v,t; 
          Vec3<vfloat<M>> Ng;
          vbool<M> valid = valid_i;
          size_t i;
          std::tie(u,v,t,Ng,i) = hit(valid, flags);
          
          int geomID = geomIDs[i];
          int instID = geomID_to_instID ? geomID_to_instID[0] : geomID;

          ray.u = u[i];
          ray.v = v[i];
          ray.tfar = t[i];
          ray.Ng.x = Ng.x[i];
          ray.Ng.y = Ng.y[i];
          ray.Ng.z = Ng.z[i];
          ray.geomID = instID;
          ray.primID = primIDs[i];
          return true;
        }
      };


    template<int M, bool filter>
      struct OccludedPairs1Epilog
      {
        Ray& ray;
        const vint<M>& geomIDs;
        const vint<M>& primIDs;
        const vint<M>& flags;
        Scene* scene;
        const unsigned* geomID_to_instID;
        
        __forceinline OccludedPairs1Epilog(Ray& ray,
                                           const vint<M>& geomIDs, 
                                           const vint<M>& primIDs, 
                                           const vint<M>& flags, 
                                           Scene* scene,
                                           const unsigned* geomID_to_instID)
          : ray(ray), geomIDs(geomIDs), primIDs(primIDs), flags(flags), scene(scene), geomID_to_instID(geomID_to_instID) {}
        
        template<typename Hit>
        __forceinline bool operator() (const vbool<M>& valid, const Hit& hit) const
        {
          return true;
        }
      };


    /*! Intersects M triangle pairs with 1 ray */
    template<int M, bool filter>
      struct TrianglePairsMIntersector1MoellerTrumbore
      {
        typedef TrianglePairsMv<M> Primitive;
#if defined(__AVX__)
        typedef MoellerTrumboreIntersectorPairs1<2*M> Precalculations;
#else
        typedef MoellerTrumboreIntersectorPairs1<M> Precalculations;
#endif
        
        /*! Intersect a ray with the M triangles and updates the hit. */
        static __forceinline void intersect(const Precalculations& pre, Ray& ray, const Primitive& tri, Scene* scene, const unsigned* geomID_to_instID)
        {
          STAT3(normal.trav_prims,1,1,1);
#if defined(__AVX__)
        Vec3vf8 vtx0(vfloat8(tri.v1.x,tri.v3.x),
                     vfloat8(tri.v1.y,tri.v3.y),
                     vfloat8(tri.v1.z,tri.v3.z));
        Vec3vf8 vtx1(vfloat8(tri.v0.x),
                     vfloat8(tri.v0.y),
                     vfloat8(tri.v0.z));
        Vec3vf8 vtx2(vfloat8(tri.v2.x),
                     vfloat8(tri.v2.y),
                     vfloat8(tri.v2.z));
        vint8   geomIDs(tri.geomIDs); 
        vint8   primIDs(tri.primIDs,tri.primIDs+1);
        vint8   flags(tri.flags);
        pre.intersect(ray,vtx0,vtx1,vtx2,IntersectPairs1Epilog<8,filter>(ray,geomIDs,primIDs,flags,scene,geomID_to_instID));          
#else
        vint<M> geomIDs(tri.geomIDs);
        pre.intersect(ray,tri.v1,tri.v0,tri.v2,IntersectPairs1Epilog<M,filter>(ray,geomIDs,tri.primIDs+0,scene,geomID_to_instID));
        pre.intersect(ray,tri.v3,tri.v0,tri.v2,IntersectPairs1Epilog<M,filter>(ray,geomIDs,tri.primIDs+1,scene,geomID_to_instID));
#endif
        }
        
        /*! Test if the ray is occluded by one of M triangles. */
        static __forceinline bool occluded(const Precalculations& pre, Ray& ray, const Primitive& tri, Scene* scene, const unsigned* geomID_to_instID)
        {
          STAT3(shadow.trav_prims,1,1,1);
#if defined(__AVX__)
          Vec3vf8 vtx0(vfloat8(tri.v1.x,tri.v3.x),
                       vfloat8(tri.v1.y,tri.v3.y),
                       vfloat8(tri.v1.z,tri.v3.z));
          Vec3vf8 vtx1(vfloat8(tri.v0.x),
                       vfloat8(tri.v0.y),
                       vfloat8(tri.v0.z));
          Vec3vf8 vtx2(vfloat8(tri.v2.x),
                       vfloat8(tri.v2.y),
                       vfloat8(tri.v2.z));
          vint8   geomIDs(tri.geomIDs); 
          vint8   primIDs(tri.primIDs,tri.primIDs+1);
          vint8   flags(tri.flags);
          return pre.intersect(ray,vtx0,vtx1,vtx2,OccludedPairs1Epilog<8,filter>(ray,geomIDs,primIDs,flags,scene,geomID_to_instID));
#else
          vint<M> geomIDs(tri.geomIDs);
          if (pre.intersect(ray,tri.v0,tri.v1,tri.v2,OccludedPairs1Epilog<M,filter>(ray,geomIDs,tri.primIDs+0,scene,geomID_to_instID))) return true;
          if (pre.intersect(ray,tri.v0,tri.v2,tri.v3,OccludedPairs1Epilog<M,filter>(ray,geomIDs,tri.primIDs+1,scene,geomID_to_instID))) return true;
          
#endif
          return false;
        }
      };


    /*! Intersects M triangles with K rays. */
    template<int M, int K, bool filter>
      struct TrianglePairsMIntersectorKMoellerTrumbore
      {
        typedef TrianglePairsMv<M> Primitive;
        typedef MoellerTrumboreIntersectorK<M,K> Precalculations;
        
        /*! Intersects K rays with M triangles. */
        static __forceinline void intersect(const vbool<K>& valid_i, Precalculations& pre, RayK<K>& ray, const TrianglePairsMv<M>& tri, Scene* scene)
        {
          for (size_t i=0; i<TrianglePairsMv<M>::max_size(); i++)
          {
            if (!tri.valid(i)) break;
            STAT3(normal.trav_prims,1,popcnt(valid_i),K);
            const Vec3<vfloat<K>> p0 = broadcast<vfloat<K>>(tri.v0,i);
            const Vec3<vfloat<K>> p1 = broadcast<vfloat<K>>(tri.v1,i);
            const Vec3<vfloat<K>> p2 = broadcast<vfloat<K>>(tri.v2,i);
            const Vec3<vfloat<K>> p3 = broadcast<vfloat<K>>(tri.v3,i);
            pre.intersectK(valid_i,ray,p0,p1,p2,IntersectKEpilog<M,K,filter>(ray,tri.geomIDs,tri.primIDs,i,scene));
            pre.intersectK(valid_i,ray,p0,p2,p3,IntersectKEpilog<M,K,filter>(ray,tri.geomIDs,tri.primIDs,i,scene));
          }
        }
        
        /*! Test for K rays if they are occluded by any of the M triangles. */
        static __forceinline vbool<K> occluded(const vbool<K>& valid_i, Precalculations& pre, RayK<K>& ray, const TrianglePairsMv<M>& tri, Scene* scene)
        {
          vbool<K> valid0 = valid_i;
          
          for (size_t i=0; i<TrianglePairsMv<M>::max_size(); i++)
          {
            if (!tri.valid(i)) break;
            STAT3(shadow.trav_prims,1,popcnt(valid0),K);
            const Vec3<vfloat<K>> p0 = broadcast<vfloat<K>>(tri.v0,i);
            const Vec3<vfloat<K>> p1 = broadcast<vfloat<K>>(tri.v1,i);
            const Vec3<vfloat<K>> p2 = broadcast<vfloat<K>>(tri.v2,i);
            const Vec3<vfloat<K>> p3 = broadcast<vfloat<K>>(tri.v3,i);
            pre.intersectK(valid0,ray,p0,p1,p2,OccludedKEpilog<M,K,filter>(valid0,ray,tri.geomIDs,tri.primIDs,i,scene));
            if (none(valid0)) break;
            pre.intersectK(valid0,ray,p0,p2,p3,OccludedKEpilog<M,K,filter>(valid0,ray,tri.geomIDs,tri.primIDs,i,scene));
            if (none(valid0)) break;
          }
          return !valid0;
        }
        
        /*! Intersect a ray with M triangles and updates the hit. */
        static __forceinline void intersect(Precalculations& pre, RayK<K>& ray, size_t k, const TrianglePairsMv<M>& tri, Scene* scene)
        {
          STAT3(normal.trav_prims,1,1,1);
          pre.intersect(ray,k,tri.v0,tri.v1,tri.v2,Intersect1KEpilog<M,K,filter>(ray,k,tri.geomIDs,tri.primIDs,scene));
          pre.intersect(ray,k,tri.v0,tri.v2,tri.v3,Intersect1KEpilog<M,K,filter>(ray,k,tri.geomIDs,tri.primIDs,scene));
        }
        
        /*! Test if the ray is occluded by one of the M triangles. */
        static __forceinline bool occluded(Precalculations& pre, RayK<K>& ray, size_t k, const TrianglePairsMv<M>& tri, Scene* scene)
        {
          STAT3(shadow.trav_prims,1,1,1);
          if (pre.intersect(ray,k,tri.v0,tri.v1,tri.v2,Occluded1KEpilog<M,K,filter>(ray,k,tri.geomIDs,tri.primIDs,scene))) return true;
          if (pre.intersect(ray,k,tri.v0,tri.v2,tri.v3,Occluded1KEpilog<M,K,filter>(ray,k,tri.geomIDs,tri.primIDs,scene))) return true;
          return false;
        }
      };


  }
}
