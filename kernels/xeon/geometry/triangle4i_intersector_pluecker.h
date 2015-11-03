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

#include "trianglei.h"
#include "../../common/ray.h"

#include "triangle_intersector_pluecker.h"
#include "../../common/scene_triangle_mesh.h"

namespace embree
{
  namespace isa
  {
    /*! Intersector1 for Triangle4i */
    template<int M, int Mx, bool filter>
    struct Triangle4iIntersector1Pluecker
      {
        typedef Triangle4i Primitive;
        typedef PlueckerIntersector1<Mx> Precalculations;
        
        static __forceinline void intersect(const Precalculations& pre, Ray& ray, const Primitive& tri, Scene* scene, const unsigned* geomID_to_instID)
        {
          STAT3(normal.trav_prims,1,1,1);
          Vec3vf4 v0, v1, v2; tri.gather(v0,v1,v2);
          pre.intersect(ray,v0,v1,v2,UVIdentity<M>(),Intersect1Epilog<M,Mx,filter>(ray,tri.geomIDs,tri.primIDs,scene,geomID_to_instID)); 
        }
        
        static __forceinline bool occluded(const Precalculations& pre, Ray& ray, const Primitive& tri, Scene* scene, const unsigned* geomID_to_instID)
        {
          STAT3(shadow.trav_prims,1,1,1);
          Vec3vf4 v0, v1, v2; tri.gather(v0,v1,v2);
          return pre.intersect(ray,v0,v1,v2,UVIdentity<M>(),Occluded1Epilog<M,Mx,filter>(ray,tri.geomIDs,tri.primIDs,scene,geomID_to_instID)); 
        }
      };

    /*! Triangle4i intersector for K rays */
    template<int M, int Mx, int K, bool filter>
      struct Triangle4iIntersectorKPluecker
      {
        typedef Triangle4i Primitive;
        typedef PlueckerIntersectorK<Mx,K> Precalculations;
 
        static __forceinline void intersect(const vbool<K>& valid_i, Precalculations& pre, RayK<K>& ray, const Primitive& tri, Scene* scene)
        {
          for (size_t i=0; i<Triangle4i::max_size(); i++)
          {
            if (!tri.valid(i)) break;
            STAT3(normal.trav_prims,1,popcnt(valid_i),RayK<K>::size());
            const Vec3f& p0 = *tri.v0[i];
            const Vec3f& p1 = *(Vec3f*)((int*)&p0 + tri.v1[i]);
            const Vec3f& p2 = *(Vec3f*)((int*)&p0 + tri.v2[i]);
            const Vec3<vfloat<K>> v0 = Vec3<vfloat<K>>(p0);
            const Vec3<vfloat<K>> v1 = Vec3<vfloat<K>>(p1);
            const Vec3<vfloat<K>> v2 = Vec3<vfloat<K>>(p2);
            pre.intersectK(valid_i,ray,v0,v1,v2,UVIdentity<K>(),IntersectKEpilog<M,K,filter>(ray,tri.geomIDs,tri.primIDs,i,scene));
          }
        }
        
        static __forceinline vbool<K> occluded(const vbool<K>& valid_i, Precalculations& pre, RayK<K>& ray, const Primitive& tri, Scene* scene)
        {
          vbool<K> valid0 = valid_i;
          
          for (size_t i=0; i<Triangle4i::max_size(); i++)
          {
            if (!tri.valid(i)) break;
            STAT3(shadow.trav_prims,1,popcnt(valid_i),RayK<K>::size());
            const Vec3f& p0 = *tri.v0[i];
            const Vec3f& p1 = *(Vec3f*)((int*)&p0 + tri.v1[i]);
            const Vec3f& p2 = *(Vec3f*)((int*)&p0 + tri.v2[i]);
            const Vec3<vfloat<K>> v0 = Vec3<vfloat<K>>(p0);
            const Vec3<vfloat<K>> v1 = Vec3<vfloat<K>>(p1);
            const Vec3<vfloat<K>> v2 = Vec3<vfloat<K>>(p2);
            pre.intersectK(valid0,ray,v0,v1,v2,UVIdentity<K>(),OccludedKEpilog<M,K,filter>(valid0,ray,tri.geomIDs,tri.primIDs,i,scene));
            if (none(valid0)) break;
          }
          return !valid0;
        }

        static __forceinline void intersect(Precalculations& pre, RayK<K>& ray, size_t k, const Primitive& tri, Scene* scene)
        {
          STAT3(normal.trav_prims,1,1,1);
          Vec3vf4 v0, v1, v2; tri.gather(v0,v1,v2);
          pre.intersect(ray,k,v0,v1,v2,UVIdentity<Mx>(),Intersect1KEpilog<M,Mx,K,filter>(ray,k,tri.geomIDs,tri.primIDs,scene)); 
        }
        
        static __forceinline bool occluded(Precalculations& pre, RayK<K>& ray, size_t k, const Primitive& tri, Scene* scene)
        {
          STAT3(shadow.trav_prims,1,1,1);
          Vec3vf4 v0, v1, v2; tri.gather(v0,v1,v2);
          return pre.intersect(ray,k,v0,v1,v2,UVIdentity<Mx>(),Occluded1KEpilog<M,Mx,K,filter>(ray,k,tri.geomIDs,tri.primIDs,scene)); 
        }
      };
  }
}
