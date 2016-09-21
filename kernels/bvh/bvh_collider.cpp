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

namespace embree
{ 
  namespace isa
  {
#define CSTAT(x)

    size_t bvh_collide_traversal_steps = 0;
    size_t bvh_collide_leaf_pairs = 0;
    size_t bvh_collide_leaf_iterations = 0;
    size_t bvh_collide_prim_pairs = 0;

    template<int N>
    __forceinline size_t overlap(const BBox3fa& box0, const typename BVHN<N>::Node& node1)
    {
      const vfloat<N> lower_x = max(vfloat<N>(box0.lower.x),node1.lower_x);
      const vfloat<N> lower_y = max(vfloat<N>(box0.lower.y),node1.lower_y);
      const vfloat<N> lower_z = max(vfloat<N>(box0.lower.z),node1.lower_z);
      const vfloat<N> upper_x = min(vfloat<N>(box0.upper.x),node1.upper_x);
      const vfloat<N> upper_y = min(vfloat<N>(box0.upper.y),node1.upper_y);
      const vfloat<N> upper_z = min(vfloat<N>(box0.upper.z),node1.upper_z);
      return movemask((lower_x <= upper_x) & (lower_y <= upper_y) & (lower_z <= upper_z));
    }

    template<int N>
    __forceinline size_t overlap(const BBox3fa& box0, const BBox<Vec3<vfloat<N>>>& box1)
    {
      const vfloat<N> lower_x = max(vfloat<N>(box0.lower.x),box1.lower.x);
      const vfloat<N> lower_y = max(vfloat<N>(box0.lower.y),box1.lower.y);
      const vfloat<N> lower_z = max(vfloat<N>(box0.lower.z),box1.lower.z);
      const vfloat<N> upper_x = min(vfloat<N>(box0.upper.x),box1.upper.x);
      const vfloat<N> upper_y = min(vfloat<N>(box0.upper.y),box1.upper.y);
      const vfloat<N> upper_z = min(vfloat<N>(box0.upper.z),box1.upper.z);
      return movemask((lower_x <= upper_x) & (lower_y <= upper_y) & (lower_z <= upper_z));
    }

    template<int N>
    __forceinline void BVHNCollider<N>::processLeaf(const Triangle4v& __restrict__ tris0, const Triangle4v& __restrict__ tris1, RTCCollideFunc callback, void* userPtr)
    {
      size_t size0 = tris0.size();
      size_t size1 = tris1.size();
      BBox<Vec3vf4> bounds0(min(tris0.v0,tris0.v1,tris0.v2),max(tris0.v0,tris0.v1,tris0.v2));
      BBox<Vec3vf4> bounds1(min(tris1.v0,tris1.v1,tris1.v2),max(tris1.v0,tris1.v1,tris1.v2));
     
      if (size0 < size1)
      {
        for (size_t i=0; i<size0; i++) 
        {
          CSTAT(bvh_collide_leaf_iterations++);
          const Vec3fa lower(bounds0.lower.x[i],bounds0.lower.y[i],bounds0.lower.z[i]);
          const Vec3fa upper(bounds0.upper.x[i],bounds0.upper.y[i],bounds0.upper.z[i]);
          const BBox3fa bounds(lower,upper);
          size_t mask = movemask(tris1.valid()) & overlap(bounds,bounds1);
          for (size_t m=mask, j=__bsf(m); m!=0; m=__btc(m,j), j=__bsf(m)) {
            CSTAT(bvh_collide_prim_pairs++);
            callback(userPtr,tris0.geomID(i),tris0.primID(i),tris1.geomID(j),tris1.primID(j));
          }
        }
      } 
      else 
      {
        for (size_t j=0; j<size1; j++) 
        {
          CSTAT(bvh_collide_leaf_iterations++);
          const Vec3fa lower(bounds1.lower.x[j],bounds1.lower.y[j],bounds1.lower.z[j]);
          const Vec3fa upper(bounds1.upper.x[j],bounds1.upper.y[j],bounds1.upper.z[j]);
          const BBox3fa bounds(lower,upper);
          size_t mask = movemask(tris0.valid()) & overlap(bounds,bounds0);
          for (size_t m=mask, i=__bsf(m); m!=0; m=__btc(m,i), i=__bsf(m)) {
            CSTAT(bvh_collide_prim_pairs++);
            callback(userPtr,tris0.geomID(i),tris0.primID(i),tris1.geomID(j),tris1.primID(j));
          }
        }
      }
    }

    template<int N>
    __forceinline void BVHNCollider<N>::processLeaf(NodeRef node0, NodeRef node1, RTCCollideFunc callback, void* userPtr)
    {
      size_t N0; Triangle4v* leaf0 = (Triangle4v*) node0.leaf(N0);
      size_t N1; Triangle4v* leaf1 = (Triangle4v*) node1.leaf(N1);
      for (size_t i=0; i<N0; i++)
        for (size_t j=0; j<N1; j++)
          processLeaf(leaf0[i],leaf1[j],callback,userPtr);
    }

    template<int N>
    void BVHNCollider<N>::collide(BVH* __restrict__ bvh0, BVH* __restrict__ bvh1, RTCCollideFunc callback, void* userPtr)
    {
      

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

        CSTAT(bvh_collide_traversal_steps++);
        if (cur.node0.isLeaf()) {
          if (cur.node1.isLeaf()) {
            CSTAT(bvh_collide_leaf_pairs++);
            processLeaf(cur.node0,cur.node1,callback,userPtr);
            continue;
          } else goto recurse_node1;

        } else {
          if (cur.node1.isLeaf()) {
            goto recurse_node0;
          } else {
            if (area(cur.bounds0) > area(cur.bounds1)) {
              goto recurse_node0;
            } else {
              goto recurse_node1;
            }
          }
        }

        {
        recurse_node0:
          Node* node0 = cur.node0.node();
          size_t mask = overlap<N>(cur.bounds1,*node0);
          for (size_t m=mask, i=__bsf(m); m!=0; m=__btc(m,i), i=__bsf(m)) {
            node0->child(i).prefetch(BVH_FLAG_ALIGNED_NODE);
            *stackPtr++ = StackItem(node0->child(i),node0->bounds(i),cur.node1,cur.bounds1);
          }
          continue;
        }

        {
        recurse_node1:
          Node* node1 = cur.node1.node();
          size_t mask = overlap<N>(cur.bounds0,*node1);
          for (size_t m=mask, i=__bsf(m); m!=0; m=__btc(m,i), i=__bsf(m)) {
            node1->child(i).prefetch(BVH_FLAG_ALIGNED_NODE);
            *stackPtr++ = StackItem(cur.node0,cur.bounds0,node1->child(i),node1->bounds(i));          
          }
          continue;
        }
      }
      
      CSTAT(PRINT(bvh_collide_traversal_steps));
      CSTAT(PRINT(bvh_collide_leaf_pairs));
      CSTAT(PRINT(bvh_collide_leaf_iterations));
      CSTAT(PRINT(bvh_collide_prim_pairs));
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
