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

    /*! Intersects M triangle pairs with 1 ray */
    template<int M, bool filter>
      struct TrianglePairsMIntersector1MoellerTrumbore
      {
        typedef TrianglePairsMv<M> Primitive;
        typedef MoellerTrumboreIntersector1<M> Precalculations;
        
        /*! Intersect a ray with the M triangles and updates the hit. */
        static __forceinline void intersect(const Precalculations& pre, Ray& ray, const Primitive& tri, Scene* scene, const unsigned* geomID_to_instID)
        {
          STAT3(normal.trav_prims,1,1,1);
#if 0 //defined(__AVX__)
          /* Vec3vf8 vtx0(vfloat8(tri.v1.x,tri.v3.x), */
          /*              vfloat8(tri.v1.y,tri.v3.y), */
          /*              vfloat8(tri.v1.z,tri.v3.z)); */
          /* Vec3vf8 vtx1(vfloat8(tri.v0.x), */
          /*              vfloat8(tri.v0.y), */
          /*              vfloat8(tri.v0.z)); */
          /* Vec3vf8 vtx2(vfloat8(tri.v2.x), */
          /*              vfloat8(tri.v2.y), */
          /*              vfloat8(tri.v2.z)); */

          Vec3vf8 vtx0,vtx1,vtx2;
          vint8   geomIDs(tri.geomIDs);
          vint8   primIDs(tri.primIDs);
          pre.intersect(ray,vtx0,vtx1,vtx2,Intersect1Epilog<8,filter>(ray,geomIDs,primIDs,scene,geomID_to_instID));
          
#else
          pre.intersect(ray,tri.v1,tri.v0,tri.v2,Intersect1Epilog<M,filter>(ray,tri.geomIDs,tri.primIDs,scene,geomID_to_instID));
          pre.intersect(ray,tri.v3,tri.v0,tri.v2,Intersect1Epilog<M,filter>(ray,tri.geomIDs,tri.primIDs,scene,geomID_to_instID));
#endif
        }
        
        /*! Test if the ray is occluded by one of M triangles. */
        static __forceinline bool occluded(const Precalculations& pre, Ray& ray, const Primitive& tri, Scene* scene, const unsigned* geomID_to_instID)
        {
          STAT3(shadow.trav_prims,1,1,1);
          if (pre.intersect(ray,tri.v0,tri.v1,tri.v2,Occluded1Epilog<M,filter>(ray,tri.geomIDs,tri.primIDs,scene,geomID_to_instID))) return true;
          if (pre.intersect(ray,tri.v0,tri.v2,tri.v3,Occluded1Epilog<M,filter>(ray,tri.geomIDs,tri.primIDs,scene,geomID_to_instID))) return true;
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
