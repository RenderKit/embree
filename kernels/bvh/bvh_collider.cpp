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

#include "bvh_collider.h"
#include "../geometry/triangle.h"

namespace embree
{ 
  namespace isa
  {
    template<int N>
    __forceinline size_t overlap(const BBox3fa& box0, const typename BVHN<N>::Node& node1)
    {
      vfloat<N> lower_x = max(vfloat<N>(box0.lower.x),node1.lower_x);
      vfloat<N> lower_y = max(vfloat<N>(box0.lower.y),node1.lower_y);
      vfloat<N> lower_z = max(vfloat<N>(box0.lower.z),node1.lower_z);
      vfloat<N> upper_x = min(vfloat<N>(box0.upper.x),node1.upper_x);
      vfloat<N> upper_y = min(vfloat<N>(box0.upper.y),node1.upper_y);
      vfloat<N> upper_z = min(vfloat<N>(box0.upper.z),node1.upper_z);
      return movemask((lower_x <= upper_x) & (lower_y <= upper_y) & (lower_z <= upper_z));
    }

    /*template<int N>
    __forceinline size_t overlap(const typename BVHN<N>::Node& node0, size_t i0, const typename BVHN<N>::Node& node1)
    {
      vfloat<N> lower_x = max(vfloat<N>(node0.lower_x[i0]),node1.lower_x);
      vfloat<N> lower_y = max(vfloat<N>(node0.lower_y[i0]),node1.lower_y);
      vfloat<N> lower_z = max(vfloat<N>(node0.lower_z[i0]),node1.lower_z);
      vfloat<N> upper_x = min(vfloat<N>(node0.upper_x[i0]),node1.upper_x);
      vfloat<N> upper_y = min(vfloat<N>(node0.upper_y[i0]),node1.upper_y);
      vfloat<N> upper_z = min(vfloat<N>(node0.upper_z[i0]),node1.upper_z);
      return movemask((lower_x > upper_x) | (lower_y > upper_y) | (lower_z > upper_z));
      }*/

    template<int N>
    void BVHNCollider<N>::collide(BVH* __restrict__ bvh0, BVH* __restrict__ bvh1, RTCCollideFunc callback, void* userPtr)
    {
      size_t steps = 0;

      /*! stack state */
      struct StackItem 
      {
      public:
        __forceinline StackItem () {}

        __forceinline StackItem (NodeRef node0, const BBox3fa& bounds0,
                                 NodeRef node1, const BBox3fa& bounds1)
          : node0(node0), node1(node1), bounds0(bounds0), bounds1(bounds1) {}
        
      public:
        NodeRef node0;
        NodeRef node1;
        BBox3fa bounds0;
        BBox3fa bounds1;
      };
      StackItem stack[stackSize];           //!< stack of nodes 
      StackItem* stackPtr = stack+1;        //!< current stack pointer
      stack[0] = StackItem(bvh0->root,bvh0->bounds,bvh1->root,bvh1->bounds);

      /* pop loop */
      while (true)
      {
        /*! pop next node */
        if (unlikely(stackPtr == stack)) break;
        stackPtr--;
        StackItem cur = *stackPtr;

        steps++;
        if (cur.node0.isLeaf()) {
          if (cur.node1.isLeaf()) {
            callback(userPtr,(size_t)cur.node0,0,(size_t)cur.node1,0);
          } else {
            Node* node1 = cur.node1.node();
            size_t mask = overlap<N>(cur.bounds0,*node1);
            for (size_t m=mask, i=__bsf(m); m!=0; m=__btc(m,i), i=__bsf(m)) {
              node1->child(i).prefetch(BVH_FLAG_ALIGNED_NODE);
              *stackPtr++ = StackItem(cur.node0,cur.bounds0,node1->child(i),node1->bounds(i));
              
            }
          }
        } else {
          if (cur.node1.isLeaf()) {
            Node* node0 = cur.node0.node();
            size_t mask = overlap<N>(cur.bounds1,*node0);
            for (size_t m=mask, i=__bsf(m); m!=0; m=__btc(m,i), i=__bsf(m)) {
              node0->child(i).prefetch(BVH_FLAG_ALIGNED_NODE);
              *stackPtr++ = StackItem(node0->child(i),node0->bounds(i),cur.node1,cur.bounds1);                
            }
          } else {
            if (area(cur.bounds0) > area(cur.bounds1)) {
              Node* node0 = cur.node0.node();
              size_t mask = overlap<N>(cur.bounds1,*node0);
              for (size_t m=mask, i=__bsf(m); m!=0; m=__btc(m,i), i=__bsf(m)) {
                node0->child(i).prefetch(BVH_FLAG_ALIGNED_NODE);
                *stackPtr++ = StackItem(node0->child(i),node0->bounds(i),cur.node1,cur.bounds1);
              }
            } else {
              Node* node1 = cur.node1.node();
              size_t mask = overlap<N>(cur.bounds0,*node1);
              for (size_t m=mask, i=__bsf(m); m!=0; m=__btc(m,i), i=__bsf(m)) {
                node1->child(i).prefetch(BVH_FLAG_ALIGNED_NODE);
                *stackPtr++ = StackItem(cur.node0,cur.bounds0,node1->child(i),node1->bounds(i));
              }
            }
          }
        }
      }
      PRINT(steps);
      AVX_ZERO_UPPER();
    }

    ////////////////////////////////////////////////////////////////////////////////
    /// Collider Definitions
    ////////////////////////////////////////////////////////////////////////////////

    DEFINE_COLLIDER(BVH4Collider,BVHNCollider<4>);

#if defined(__AVX__)
    DEFINE_COLLIDER(BVH8Collider,BVHNCollider<8>);
#endif
  }
}
