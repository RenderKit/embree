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
    template<bool filter>
    struct Triangle4iIntersector1Pluecker
      {
        enum { M = 4 };
        typedef Triangle4i Primitive;
        typedef PlueckerIntersector1<M> Precalculations;
        
        static __forceinline void intersect(const Precalculations& pre, Ray& ray, const Primitive& tri, Scene* scene, const unsigned* geomID_to_instID)
        {
          /* gather vertices */
          STAT3(normal.trav_prims,1,1,1);
          const int* base0 = (const int*) tri.v0[0];
          const int* base1 = (const int*) tri.v0[1];
          const int* base2 = (const int*) tri.v0[2];
          const int* base3 = (const int*) tri.v0[3];
          const vfloat4 a0 = vfloat4::loadu(base0          ), a1 = vfloat4::loadu(base1          ), a2 = vfloat4::loadu(base2          ), a3 = vfloat4::loadu(base3          );
          const vfloat4 b0 = vfloat4::loadu(base0+tri.v1[0]), b1 = vfloat4::loadu(base1+tri.v1[1]), b2 = vfloat4::loadu(base2+tri.v1[2]), b3 = vfloat4::loadu(base3+tri.v1[3]);
          const vfloat4 c0 = vfloat4::loadu(base0+tri.v2[0]), c1 = vfloat4::loadu(base1+tri.v2[1]), c2 = vfloat4::loadu(base2+tri.v2[2]), c3 = vfloat4::loadu(base3+tri.v2[3]);
          Vec3vf4 p0; transpose(a0,a1,a2,a3,p0.x,p0.y,p0.z);
          Vec3vf4 p1; transpose(b0,b1,b2,b3,p1.x,p1.y,p1.z);
          Vec3vf4 p2; transpose(c0,c1,c2,c3,p2.x,p2.y,p2.z);
          pre.intersect(ray,p0,p1,p2,UVIdentity<M>(),Intersect1Epilog<M,filter>(ray,tri.geomIDs,tri.primIDs,scene,geomID_to_instID));
        }
        
        static __forceinline bool occluded(const Precalculations& pre, Ray& ray, const Primitive& tri, Scene* scene, const unsigned* geomID_to_instID)
        {
          /* gather vertices */
          STAT3(shadow.trav_prims,1,1,1);
          const int* base0 = (const int*) tri.v0[0];
          const int* base1 = (const int*) tri.v0[1];
          const int* base2 = (const int*) tri.v0[2];
          const int* base3 = (const int*) tri.v0[3];
          const vfloat4 a0 = vfloat4::loadu(base0          ), a1 = vfloat4::loadu(base1          ), a2 = vfloat4::loadu(base2          ), a3 = vfloat4::loadu(base3          );
          const vfloat4 b0 = vfloat4::loadu(base0+tri.v1[0]), b1 = vfloat4::loadu(base1+tri.v1[1]), b2 = vfloat4::loadu(base2+tri.v1[2]), b3 = vfloat4::loadu(base3+tri.v1[3]);
          const vfloat4 c0 = vfloat4::loadu(base0+tri.v2[0]), c1 = vfloat4::loadu(base1+tri.v2[1]), c2 = vfloat4::loadu(base2+tri.v2[2]), c3 = vfloat4::loadu(base3+tri.v2[3]);
          Vec3vf4 p0; transpose(a0,a1,a2,a3,p0.x,p0.y,p0.z);
          Vec3vf4 p1; transpose(b0,b1,b2,b3,p1.x,p1.y,p1.z);
          Vec3vf4 p2; transpose(c0,c1,c2,c3,p2.x,p2.y,p2.z);
          return pre.intersect(ray,p0,p1,p2,UVIdentity<M>(),Occluded1Epilog<M,filter>(ray,tri.geomIDs,tri.primIDs,scene,geomID_to_instID));
        }
      };

    /*! Triangle4i intersector for K rays */
    template<int K, bool filter>
      struct Triangle4iIntersectorKPluecker
      {
        typedef Triangle4i Primitive;
        enum { M = 4 };
        typedef PlueckerIntersectorK<M,K> Precalculations;
 
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
      };
  }
}
