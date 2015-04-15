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

#include "triangle4.h"
#include "triangle_intersector_moeller.h"

namespace embree
{
  namespace isa
  {
    template<bool list, bool enableIntersectionFilter>
      struct Triangle4Intersector4MoellerTrumbore
      {
        typedef Triangle4 Primitive;
        typedef TriangleNIntersector4MoellerTrumbore<Triangle4,enableIntersectionFilter> Intersector;
        typedef typename Intersector::Precalculations Precalculations;

        /*! Intersects a 4 rays with 4 triangles. */
        static __forceinline void intersect(const sseb& valid_i, Precalculations& pre, Ray4& ray, const Primitive& tri, Scene* scene) {
          Intersector::intersect(valid_i,pre,ray,tri,scene);
        }
        
        /*! Test for 4 rays if they are occluded by any of the 4 triangle. */
        static __forceinline sseb occluded(const sseb& valid_i, Precalculations& pre, Ray4& ray, const Primitive& tri, Scene* scene) {
          return Intersector::occluded(valid_i,pre,ray,tri,scene);
        }
        
        /*! Intersect a ray with the 4 triangles and updates the hit. */
        static __forceinline void intersect(Precalculations& pre, Ray4& ray, size_t k, const Primitive& tri, Scene* scene) {
          Intersector::intersect(pre,ray,k,tri,scene);
        }
        
        /*! Test if the ray is occluded by one of the triangles. */
        static __forceinline bool occluded(Precalculations& pre, Ray4& ray, size_t k, const Primitive& tri, Scene* scene) {
          return Intersector::occluded(pre,ray,k,tri,scene);
        }
      };
  }
}
