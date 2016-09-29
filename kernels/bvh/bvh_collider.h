// ======================================================================== //
// Copyright 2009-2016 Intel Corporation                                    //
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

#include "bvh.h"
#include "../geometry/trianglev.h"

namespace embree
{
  namespace isa
  {
    template<int N>
      class BVHNCollider
    {
      /* shortcuts for frequently used types */
      typedef BVHN<N> BVH;
      typedef typename BVH::NodeRef NodeRef;
      typedef typename BVH::Node Node;
      static const size_t stackSize = 2*(1+(N-1)*BVH::maxDepth);

    public:
      static void processLeaf(const Triangle4v& tris0, const Triangle4v& tris1, RTCCollideFunc callback, void* userPtr);
      static void processLeaf(NodeRef leaf0, NodeRef leaf1, RTCCollideFunc callback, void* userPtr);
      static void collide_recurse(NodeRef node0, const BBox3fa& bounds0, NodeRef node1, const BBox3fa& bounds1, RTCCollideFunc callback, void* userPtr);
      static void collide(BVH* __restrict__ bvh0, BVH* __restrict__ bvh1, RTCCollideFunc callback, void* userPtr);
    };
  }
}
