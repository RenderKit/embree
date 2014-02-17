// ======================================================================== //
// Copyright 2009-2013 Intel Corporation                                    //
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

#include "bvh4hair.h"
#include "common/ray.h"
#include "common/stack_item.h"

namespace embree
{
  namespace isa
  {
    /*! BVH4Hair single ray traversal implementation. */
    class BVH4HairIntersector1 
    {
      /* shortcuts for frequently used types */
      typedef BVH4Hair::NodeRef NodeRef;
      typedef BVH4Hair::AlignedNode AlignedNode;
      typedef BVH4Hair::UnalignedNode UnalignedNode;
      typedef BVH4Hair::Bezier1 Bezier1;
      typedef BVH4Hair::NAABBox3fa NAABBox3fa;
      static const size_t stackSize = 1+3*BVH4Hair::maxDepth;

      struct StackItem {
        NodeRef ref;
        float tNear,tFar;
      };

    private:
      static void intersectBezier(const LinearSpace3fa& ray_space, Ray& ray, const Bezier1& bezier);
      static bool occludedBezier(const LinearSpace3fa& ray_space, Ray& ray, const Bezier1& bezier);
      
    public:
      static void intersect(const BVH4Hair* This, Ray& ray);
      static void occluded (const BVH4Hair* This, Ray& ray);
    };
  }
}
