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

#include "triangle4i.h"
#include "../../common/ray.h"

#include "triangle_intersector_pluecker.h"
#include "../../common/scene_triangle_mesh.h"

namespace embree
{
  namespace isa
  {
    /*! Intersector1 for triangle4i */
    template<bool enableIntersectionFilter>
    struct Triangle4iIntersector1Pluecker
      {
        typedef Triangle4i Primitive;
        
        struct Precalculations {
          __forceinline Precalculations (const Ray& ray, const void* ptr) {}
        };
        
        static __forceinline void intersect(const Precalculations& pre, Ray& ray, const Primitive& tri, Scene* scene)
        {
          /* gather vertices */
          STAT3(normal.trav_prims,1,1,1);
          const int* base0 = (const int*) tri.v0[0];
          const int* base1 = (const int*) tri.v0[1];
          const int* base2 = (const int*) tri.v0[2];
          const int* base3 = (const int*) tri.v0[3];
          const float4 a0 = loadu4f(base0          ), a1 = loadu4f(base1          ), a2 = loadu4f(base2          ), a3 = loadu4f(base3          );
          const float4 b0 = loadu4f(base0+tri.v1[0]), b1 = loadu4f(base1+tri.v1[1]), b2 = loadu4f(base2+tri.v1[2]), b3 = loadu4f(base3+tri.v1[3]);
          const float4 c0 = loadu4f(base0+tri.v2[0]), c1 = loadu4f(base1+tri.v2[1]), c2 = loadu4f(base2+tri.v2[2]), c3 = loadu4f(base3+tri.v2[3]);
          Vec3f4 p0; transpose(a0,a1,a2,a3,p0.x,p0.y,p0.z);
          Vec3f4 p1; transpose(b0,b1,b2,b3,p1.x,p1.y,p1.z);
          Vec3f4 p2; transpose(c0,c1,c2,c3,p2.x,p2.y,p2.z);
          triangle_intersect_pluecker<enableIntersectionFilter,bool4,float4,int4>(ray,p0,p1,p2,tri.geomIDs,tri.primIDs,scene);
        }
        
        static __forceinline bool occluded(const Precalculations& pre, Ray& ray, const Primitive& tri, Scene* scene)
        {
          /* gather vertices */
          STAT3(shadow.trav_prims,1,1,1);
          const int* base0 = (const int*) tri.v0[0];
          const int* base1 = (const int*) tri.v0[1];
          const int* base2 = (const int*) tri.v0[2];
          const int* base3 = (const int*) tri.v0[3];
          const float4 a0 = loadu4f(base0          ), a1 = loadu4f(base1          ), a2 = loadu4f(base2          ), a3 = loadu4f(base3          );
          const float4 b0 = loadu4f(base0+tri.v1[0]), b1 = loadu4f(base1+tri.v1[1]), b2 = loadu4f(base2+tri.v1[2]), b3 = loadu4f(base3+tri.v1[3]);
          const float4 c0 = loadu4f(base0+tri.v2[0]), c1 = loadu4f(base1+tri.v2[1]), c2 = loadu4f(base2+tri.v2[2]), c3 = loadu4f(base3+tri.v2[3]);
          Vec3f4 p0; transpose(a0,a1,a2,a3,p0.x,p0.y,p0.z);
          Vec3f4 p1; transpose(b0,b1,b2,b3,p1.x,p1.y,p1.z);
          Vec3f4 p2; transpose(c0,c1,c2,c3,p2.x,p2.y,p2.z);
          return triangle_occluded_pluecker<enableIntersectionFilter,bool4,float4,int4>(ray,p0,p1,p2,tri.geomIDs,tri.primIDs,scene);
        }
      };

    /*! Intersector4 for triangle4i */
    template<typename RayM, bool enableIntersectionFilter>
      struct Triangle4iIntersectorMPluecker
      {
        typedef Triangle4i Primitive;
        
        /* ray SIMD type shortcuts */
        typedef typename RayM::simdb rsimdb;
        typedef typename RayM::simdf rsimdf;
        typedef typename RayM::simdi rsimdi;
        typedef Vec3<rsimdf> rsimd3f;
        
        struct Precalculations {
          __forceinline Precalculations (const rsimdb& valid, const RayM& ray) {}
        };
        
        static __forceinline void intersect(const rsimdb& valid_i, Precalculations& pre, RayM& ray, const Primitive& tri, Scene* scene)
        {
          for (size_t i=0; i<Triangle4i::max_size(); i++)
          {
            if (!tri.valid(i)) break;
            STAT3(normal.trav_prims,1,popcnt(valid_i),RayM::size());
            const Vec3f& p0 = *tri.v0[i];
            const Vec3f& p1 = *(Vec3f*)((int*)&p0 + tri.v1[i]);
            const Vec3f& p2 = *(Vec3f*)((int*)&p0 + tri.v2[i]);
            const rsimd3f v0 = rsimd3f(p0);
            const rsimd3f v1 = rsimd3f(p1);
            const rsimd3f v2 = rsimd3f(p2);
            triangle_intersect_pluecker<enableIntersectionFilter>(valid_i,ray,v0,v1,v2,tri.geomIDs,tri.primIDs,i,scene);
          }
        }
        
        static __forceinline rsimdb occluded(const rsimdb& valid_i, Precalculations& pre, RayM& ray, const Primitive& tri, Scene* scene)
        {
          rsimdb valid0 = valid_i;
          
          for (size_t i=0; i<Triangle4i::max_size(); i++)
          {
            if (!tri.valid(i)) break;
            STAT3(shadow.trav_prims,1,popcnt(valid_i),RayM::size());
            const Vec3f& p0 = *tri.v0[i];
            const Vec3f& p1 = *(Vec3f*)((int*)&p0 + tri.v1[i]);
            const Vec3f& p2 = *(Vec3f*)((int*)&p0 + tri.v2[i]);
            const rsimd3f v0 = rsimd3f(p0);
            const rsimd3f v1 = rsimd3f(p1);
            const rsimd3f v2 = rsimd3f(p2);
            triangle_occluded_pluecker<enableIntersectionFilter>(valid0,ray,v0,v1,v2,tri.geomIDs,tri.primIDs,i,scene);
            if (none(valid0)) break;
          }
          return !valid0;
        }
      };
  }
}
