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
#include "common/ray4.h"

namespace embree
{
  namespace isa
  {
    /*! Intersector8 for triangle4i */
    template<bool list>
      struct Triangle4iIntersector8Pluecker
      {
        typedef Triangle4i Primitive;
        
        struct Precalculations {
          __forceinline Precalculations (const avxb& valid, const Ray8& ray) {}
        };
        
        static __forceinline void intersect(const avxb& valid_i, Precalculations& pre, Ray8& ray, const Primitive& tri, Scene* scene)
        {
          for (size_t i=0; i<4; i++)
          {
            if (!tri.valid(i)) break;
            STAT3(normal.trav_prims,1,popcnt(valid_i),8);

            /* load vertices */
            const Vec3f& p0 = *tri.v0[i];
            const Vec3f& p1 = *(Vec3f*)((int*)&p0 + tri.v1[i]);
            const Vec3f& p2 = *(Vec3f*)((int*)&p0 + tri.v2[i]);
            const avx3f v0 = avx3f(p0);
            const avx3f v1 = avx3f(p1);
            const avx3f v2 = avx3f(p2);
            embree::isa::intersect(valid_i,ray,v0,v1,v2,tri.geomIDs,tri.primIDs,i,scene);
          }
        }
        
        static __forceinline avxb occluded(const avxb& valid_i, Precalculations& pre, Ray8& ray, const Primitive& tri, Scene* scene)
        {
          avxb valid0 = valid_i;
          
          for (size_t i=0; i<4; i++)
          {
            if (!tri.valid(i)) break;
            STAT3(shadow.trav_prims,1,popcnt(valid_i),8);

            /* load vertices */
            const Vec3f& p0 = *tri.v0[i];
            const Vec3f& p1 = *(Vec3f*)((int*)&p0 + tri.v1[i]);
            const Vec3f& p2 = *(Vec3f*)((int*)&p0 + tri.v2[i]);
            const avx3f v0 = avx3f(p0);
            const avx3f v1 = avx3f(p1);
            const avx3f v2 = avx3f(p2);
            embree::isa::occluded(valid0,ray,v0,v1,v2,tri.geomIDs,tri.primIDs,i,scene);
            if (none(valid0)) break;
          }
          return !valid0;
        }
      };
  }
}
