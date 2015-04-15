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

#include "triangle8.h"
#include "triangle_intersector_moeller.h"

namespace embree
{
  namespace isa
  {
    template<bool list>
      struct Triangle8Intersector1MoellerTrumbore
      {
        typedef Triangle8 Primitive;
        typedef typename TriangleNIntersector1Moeller<Triangle8>::Precalculations Precalculations;
        
        static __forceinline void intersect(const Precalculations& pre, Ray& ray, const Primitive& tri, Scene* scene) {
          TriangleNIntersector1Moeller<Triangle8>::intersect(pre,ray,tri,scene);
        }
        
        static __forceinline bool occluded(const Precalculations& pre, Ray& ray, const Primitive& tri, Scene* scene) {
          return TriangleNIntersector1Moeller<Triangle8>::occluded(pre,ray,tri,scene);
        }
      };
  }
}
