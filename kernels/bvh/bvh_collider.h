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
#include "../geometry/object.h"

namespace embree
{
  namespace isa
  {
    template<int N>
      class BVHNCollider
    {
      typedef BVHN<N> BVH;
      typedef typename BVH::NodeRef NodeRef;
      typedef typename BVH::Node Node;

    public:
      __forceinline BVHNCollider (Scene* scene0, Scene* scene1, RTCCollideFunc callback, void* userPtr)
        : scene0(scene0), scene1(scene1), callback(callback), userPtr(userPtr) {}

    public:
      virtual void processLeaf(NodeRef leaf0, NodeRef leaf1) = 0;
      void collide_recurse(NodeRef node0, const BBox3fa& bounds0, NodeRef node1, const BBox3fa& bounds1, size_t depth);
      void collide_recurse_entry(NodeRef node0, const BBox3fa& bounds0, NodeRef node1, const BBox3fa& bounds1, size_t depth);
    
    protected:
      Scene* scene0;
      Scene* scene1;
      RTCCollideFunc callback;
      void* userPtr;
    };

    template<int N>
      class BVHNColliderTriangle4v : public BVHNCollider<N>
    {
      typedef BVHN<N> BVH;
      typedef typename BVH::NodeRef NodeRef;
      typedef typename BVH::Node Node;

      __forceinline BVHNColliderTriangle4v (Scene* scene0, Scene* scene1, RTCCollideFunc callback, void* userPtr)
        : BVHNCollider<N>(scene0,scene1,callback,userPtr) {}

      void processLeaf(const Triangle4v& tris0, const Triangle4v& tris1);
      virtual void processLeaf(NodeRef leaf0, NodeRef leaf1);
    public:
      static void collide(BVH* __restrict__ bvh0, BVH* __restrict__ bvh1, RTCCollideFunc callback, void* userPtr);
    };

    template<int N>
      class BVHNColliderUserGeom : public BVHNCollider<N>
    {
      typedef BVHN<N> BVH;
      typedef typename BVH::NodeRef NodeRef;
      typedef typename BVH::Node Node;

      __forceinline BVHNColliderUserGeom (Scene* scene0, Scene* scene1, RTCCollideFunc callback, void* userPtr)
        : BVHNCollider<N>(scene0,scene1,callback,userPtr) {}

      virtual void processLeaf(NodeRef leaf0, NodeRef leaf1);
    public:
      static void collide(BVH* __restrict__ bvh0, BVH* __restrict__ bvh1, RTCCollideFunc callback, void* userPtr);
    };
  }
}
