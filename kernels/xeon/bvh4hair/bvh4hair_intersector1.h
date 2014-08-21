// ======================================================================== //
// Copyright 2009-2014 Intel Corporation                                    //
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
    template<typename PrimitiveIntersector, int flags>
      class BVH4HairIntersector1 
    {
      /* shortcuts for frequently used types */
      typedef typename PrimitiveIntersector::Primitive Primitive;
      typedef BVH4Hair::NodeRef NodeRef;
      typedef BVH4Hair::Node Node;
      typedef BVH4Hair::AlignedNode AlignedNode;
      typedef BVH4Hair::UnalignedNode UnalignedNode;

      static const size_t stackSize = 1+3*BVH4Hair::maxDepth;

    private:
      static size_t intersectBox(const BVH4Hair::AlignedNode* node, 
				 const sse3f& org, const sse3f& rdir, const sse3f& org_rdir, 
				 const size_t nearX, const size_t nearY, const size_t nearZ,
				 ssef& tNear, ssef& tFar);

      static size_t intersectBox(const BVH4Hair::AlignedNodeMB* node, 
                                 const sse3f& org, const sse3f& rdir, const sse3f& org_rdir, const float time, 
                                 const size_t nearX, const size_t nearY, const size_t nearZ,
                                 ssef& tNear, ssef& tFar);

      static size_t intersectBox(const BVH4Hair::UnalignedNode* node, Ray& ray, 
                                 const sse3f& org, const sse3f& dir, 
                                 ssef& tNear, ssef& tFar);

      static size_t intersectBox(const BVH4Hair::UnalignedNodeMB* node, Ray& ray,
                                 const sse3f& ray_org, const sse3f& ray_dir, 
                                 ssef& tNear, ssef& tFar);

    public:
      static void intersect(const BVH4Hair* This, Ray& ray);
      static void occluded (const BVH4Hair* This, Ray& ray);
    };
  }
}
