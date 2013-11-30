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

#include "bvh4_rotate.h"

namespace embree
{
  // FIXME: fast version 19.279 ms versus slow version 21.43 ms, not much difference!

  /*! Computes half surface area of box. */
  __forceinline float halfArea3f(const BBox<ssef>& box) {
    const ssef d = box.size();
    const ssef a = d*shuffle<1,2,0,3>(d);
    return a[0]+a[1]+a[2];
  }

  size_t BVH4Rotate::rotate(BVH4* bvh, NodeRef parentRef, size_t depth)
  {
#if 1
    /*! nothing to rotate if we reached a leaf node. */
    if (parentRef.isBarrier()) return 0;
    if (parentRef.isLeaf()) return 0;
    Node* parent = parentRef.node();

    /*! rotate all children first */
    ssei cdepth;
    for (size_t c=0; c<4; c++)
      cdepth[c] = (int)rotate(bvh,parent->child(c),depth+1);

    /* compute current area of all children */
    ssef sizeX = parent->upper_x-parent->lower_x;
    ssef sizeY = parent->upper_y-parent->lower_y;
    ssef sizeZ = parent->upper_z-parent->lower_z;
    ssef childArea = sizeX*(sizeY + sizeZ) + sizeY*sizeZ;

    /*! get node bounds */
    BBox<ssef> child1_0,child1_1,child1_2,child1_3;
    parent->bounds(child1_0,child1_1,child1_2,child1_3);

    /*! Find best rotation. We pick a first child (child1) and a sub-child 
      (child2child) of a different second child (child2), and swap child1 
      and child2child. We perform the best such swap. */
    float bestArea = 0;
    int bestChild1 = -1, bestChild2 = -1, bestChild2Child = -1;
    for (size_t c2=0; c2<4; c2++)
    {
      /*! ignore leaf nodes as we cannot descent into them */
      if (parent->child(c2).isBarrier()) continue;
      if (parent->child(c2).isLeaf()) continue;
      Node* child2 = parent->child(c2).node();

      /*! transpose child bounds */
      BBox<ssef> child2c0,child2c1,child2c2,child2c3;
      child2->bounds(child2c0,child2c1,child2c2,child2c3);

      /*! put child1_0 at each child2 position */
      float cost00 = halfArea3f(merge(child1_0,child2c1,child2c2,child2c3));
      float cost01 = halfArea3f(merge(child2c0,child1_0,child2c2,child2c3));
      float cost02 = halfArea3f(merge(child2c0,child2c1,child1_0,child2c3));
      float cost03 = halfArea3f(merge(child2c0,child2c1,child2c2,child1_0));
      ssef cost0 = ssef(cost00,cost01,cost02,cost03);
      ssef min0 = vreduce_min(cost0);
      int pos0 = (int)__bsf(movemask(min0 == cost0));

      /*! put child1_1 at each child2 position */
      float cost10 = halfArea3f(merge(child1_1,child2c1,child2c2,child2c3));
      float cost11 = halfArea3f(merge(child2c0,child1_1,child2c2,child2c3));
      float cost12 = halfArea3f(merge(child2c0,child2c1,child1_1,child2c3));
      float cost13 = halfArea3f(merge(child2c0,child2c1,child2c2,child1_1));
      ssef cost1 = ssef(cost10,cost11,cost12,cost13);
      ssef min1 = vreduce_min(cost1);
      int pos1 = (int)__bsf(movemask(min1 == cost1));

      /*! put child1_2 at each child2 position */
      float cost20 = halfArea3f(merge(child1_2,child2c1,child2c2,child2c3));
      float cost21 = halfArea3f(merge(child2c0,child1_2,child2c2,child2c3));
      float cost22 = halfArea3f(merge(child2c0,child2c1,child1_2,child2c3));
      float cost23 = halfArea3f(merge(child2c0,child2c1,child2c2,child1_2));
      ssef cost2 = ssef(cost20,cost21,cost22,cost23);
      ssef min2 = vreduce_min(cost2);
      int pos2 = (int)__bsf(movemask(min2 == cost2));

      /*! put child1_3 at each child2 position */
      float cost30 = halfArea3f(merge(child1_3,child2c1,child2c2,child2c3));
      float cost31 = halfArea3f(merge(child2c0,child1_3,child2c2,child2c3));
      float cost32 = halfArea3f(merge(child2c0,child2c1,child1_3,child2c3));
      float cost33 = halfArea3f(merge(child2c0,child2c1,child2c2,child1_3));
      ssef cost3 = ssef(cost30,cost31,cost32,cost33);
      ssef min3 = vreduce_min(cost3);
      int pos3 = (int)__bsf(movemask(min3 == cost3));

      /*! find best other child */
      ssef area0123 = ssef(extract<0>(min0),extract<0>(min1),extract<0>(min2),extract<0>(min3)) - ssef(childArea[c2]);
      int pos[4] = { pos0,pos1,pos2,pos3 };
      sseb valid = ssei(int(depth+1))+cdepth <= ssei(BVH4::maxBuildDepth); // only select swaps that fulfill depth constraints
      //valid[c2] = false;
      valid &= sseb(c2 != 0, c2 != 1, c2 != 2, c2 != 3); // FIXME: optimize
      if (none(valid)) continue;
      size_t c1 = select_min(valid,area0123);
      float area = area0123[c1]; 
      assert(c1 != c2);
      
      /*! accept a swap when it reduces cost and is not swapping a node with itself */
      if (area < bestArea) {
        bestArea = area;
        bestChild1 = c1;
        bestChild2 = c2;
        bestChild2Child = pos[c1];
      }
    }

    /*! if we did not find a swap that improves the SAH then do nothing */
    if (bestChild1 == -1) return 1+reduce_max(cdepth);
      
    /*! perform the best found tree rotation */
    Node* child2 = parent->child(bestChild2).node();
    BVH4::swap(parent,bestChild1,child2,bestChild2Child);
    parent->set(bestChild2,child2->bounds());
    BVH4::compact(parent);
    BVH4::compact(child2);
    
    /*! This returned depth is conservative as the child that was
     *  pulled up in the tree could have been on the critical path. */
    cdepth[bestChild1]++; // bestChild1 was pushed down one level
    return 1+reduce_max(cdepth); 

#else

    /*! nothing to rotate if we reached a leaf node. */
    if (parentRef.isBarrier()) return 0;
    if (parentRef.isLeaf()) return 0;
    Node* parent = parentRef.node();
    
    /*! rotate all children first */
    int max_cdepth = 0;
    int cdepth[4];
    for (size_t c=0; c<4; c++) {
      int d = (int)rotate(bvh,parent->child(c),depth+1);
      max_cdepth = max(max_cdepth,d);
      cdepth[c] = d;
    }
    
    /* compute bounds */
    float parentAreas[4]; BBox3f parentBounds[4];
    for (size_t i=0; i<4; i++) {
      const BBox3f bound = parent->bounds(i);
      parentBounds[i] = bound;
      parentAreas [i] = halfArea(bound);
    }
    
    /*! Find best rotation. We pick a first child a second child and a 
      subchild of it. We perform the best swap of the subchild and the 
      first child. */
    float bestArea = 0.0f;
    int bestChild1 = -1, bestChild2 = -1, bestChild2Child = -1;
    
    /* iterate over all first children */
    for (size_t c1=0; c1<4; c1++)
    {
      if ((depth+1)+cdepth[c1] > BVH4::maxBuildDepth) continue;
      const BBox3f child1Bound = parentBounds[c1];
      
      /* iterate over all second children */
      for (size_t c2=0; c2<4; c2++)
      {
        if (c1 == c2) continue;
        
        /*! ignore leaf nodes as we cannot descent into them */
        NodeRef child2Ref = parent->child(c2);
        if (child2Ref.isBarrier()) continue;
        if (child2Ref.isLeaf()) continue;
        Node* child2 = child2Ref.node();
        
        /* compute child bounds */
        BBox3f child2Bounds[4];
        for (size_t i=0; i<4; i++)
          child2Bounds[i] = child2->bounds(i);
        
        /* iterate over all subchildren of the second child */
        for (size_t c2c=0; c2c<4; c2c++)
        {
          /* compute heuristic */
          BBox3f bounds = child1Bound;
          for (size_t i=0; i<4; i++) {
            if (i == c2c) continue;
            bounds = merge(bounds,child2Bounds[i]);
          }
          float area = halfArea(bounds)-parentAreas[c2];
          
          /* select best swap */
          if (area < bestArea) {
            bestArea = area;
            bestChild1 = (int)c1;
            bestChild2 = (int)c2;
            bestChild2Child = (int)c2c;
          }
        }
      }
    }

    /*! if we did not find a swap that improves the SAH then do nothing */
    if (bestChild1 == -1) return 1+max_cdepth;
      
    /*! perform the best found tree rotation */
    Node* child2 = parent->child(bestChild2).node();
    BVH4::swap(parent,bestChild1,child2,bestChild2Child);
    parent->set(bestChild2,child2->bounds());
    
      /*! This returned depth is conservative as the child that was
       *  pulled up in the tree could have been on the critical path. */
    cdepth[bestChild1]++; // bestChild1 was pushed down one level
    return 1+max_cdepth; 
#endif
  }
}
