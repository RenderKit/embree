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
#include "common/ray4.h"
#include "common/stack_item.h"

namespace embree
{
  namespace isa
  {
    /*! BVH4Hair single ray traversal implementation. */
    template<typename PrimitiveIntersector>
      class BVH4HairIntersector4 
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

      static size_t intersectBox(const BVH4Hair::CompressedUnalignedNode* node, const sse3f& org, const sse3f& dir, ssef& tNear, ssef& tFar);
      static size_t intersectBox(const BVH4Hair::UncompressedUnalignedNode* node, const sse3f& org, const sse3f& dir, ssef& tNear, ssef& tFar);

      static void intersect_k(const BVH4Hair* bvh, Ray4& ray, const size_t k);
      static void occluded_k (const BVH4Hair* bvh, Ray4& ray, const size_t k);

    public:
      static void intersect(sseb* valid, const BVH4Hair* bvh, Ray4& ray);
      static void occluded (sseb* valid, const BVH4Hair* bvh, Ray4& ray);
    };
  }
}
