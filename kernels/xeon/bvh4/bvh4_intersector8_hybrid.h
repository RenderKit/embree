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

#ifndef __EMBREE_BVH4_INTERSECTOR4_HYBRID_H__
#define __EMBREE_BVH4_INTERSECTOR4_HYBRID_H__

#include "bvh4.h"
#include "common/ray8.h"
#include "common/stack_item.h"

namespace embree
{
  namespace isa 
  {
    /*! BVH4 Traverser. Hybrid Packet traversal implementation for a Quad BVH. */
    template<typename PrimitiveIntersector8>
      class BVH4Intersector8Hybrid 
    {
      /* shortcuts for frequently used types */
      typedef typename PrimitiveIntersector8::Primitive Primitive;
      typedef typename BVH4::NodeRef NodeRef;
      typedef typename BVH4::Node Node;
      typedef StackItemT<NodeRef> StackItem;
      static const size_t stackSizeSingle = 1+3*BVH4::maxDepth;
      static const size_t stackSizeChunk = 4*BVH4::maxDepth+1;

    public:
      static void intersect1(const BVH4* bvh, NodeRef root, size_t k, Ray8& ray, avx3f ray_org, avx3f ray_dir, avx3f ray_rdir, avxf ray_tnear, avxf ray_tfar);
      static bool occluded1 (const BVH4* bvh, NodeRef root, size_t k, Ray8& ray, avx3f ray_org, avx3f ray_dir, avx3f ray_rdir, avxf ray_tnear, avxf ray_tfar);

      static void intersect(avxb* valid, BVH4* bvh, Ray8& ray);
      static void occluded (avxb* valid, BVH4* bvh, Ray8& ray);
    };
  }
}

#endif
  
