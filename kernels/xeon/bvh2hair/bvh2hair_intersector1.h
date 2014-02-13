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

#include "bvh2hair.h"
#include "common/ray.h"
#include "common/stack_item.h"

namespace embree
{
  namespace isa
  {
    /*! BVH2Hair single ray traversal implementation. */
    class BVH2HairIntersector1 
    {
      /* shortcuts for frequently used types */
      typedef BVH2Hair::NodeRef NodeRef;
      typedef BVH2Hair::AlignedNode AlignedNode;
      typedef BVH2Hair::UnalignedNode UnalignedNode;
      typedef BVH2Hair::Bezier1 Bezier1;
      typedef BVH2Hair::NAABBox3fa NAABBox3fa;
      static const size_t stackSize = 1+BVH2Hair::maxDepth;

      struct StackItem {
        NodeRef ref;
        float tNear,tFar;
      };

    private:
      static bool intersectBox(const BBox3fa& aabb, const Ray& ray, const Vec3fa& rdir, float& tNear, float& tFar);
      static bool intersectBox(const AffineSpace3fa& naabb, const Ray& ray, float& tNear, float& tFar);
      static void intersectBezier(Ray& ray, const Bezier1& bezier);
      
    public:
      static void intersect(const BVH2Hair* This, Ray& ray);
      static void occluded (const BVH2Hair* This, Ray& ray);
    };
  }
}
