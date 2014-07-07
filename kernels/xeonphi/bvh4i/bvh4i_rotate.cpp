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

#include "bvh4i_rotate.h"

#define DBG(x) 

namespace embree
{

  size_t BVH4iRotate::rotate(BVH4i* bvh, NodeRef parentRef, size_t depth)
  {
    DBG( DBG_PRINT(parentRef) );
    DBG( DBG_PRINT(depth) );

    assert(depth < BVH4i::maxBuildDepth);

    /*! nothing to rotate if we reached a leaf node. */
    if (parentRef.isLeaf()) return 0;
    assert(parentRef != BVH4i::invalidNode);

    Node* parent = parentRef.node(bvh->nodePtr());

    /*! rotate all children first */
    size_t cdepth[4];
    for (size_t c=0; c<4; c++)
      cdepth[c] = (int)rotate(bvh,parent->child(c),depth+1);

    /* compute current area of all children */
    float childArea[4];
    for (size_t i=0;i<4;i++)
      {
	childArea[i] = parent->areaBounds( i );
	DBG( DBG_PRINT(i) );
	DBG( DBG_PRINT(childArea[i]) );

      }

    /*! get node bounds */
    BBox3fa child1_0,child1_1,child1_2,child1_3;
    child1_0 = parent->bounds( 0 );
    child1_1 = parent->bounds( 1 );
    child1_2 = parent->bounds( 2 );
    child1_3 = parent->bounds( 3 );


    /*! Find best rotation. We pick a first child (child1) and a sub-child 
      (child2child) of a different second child (child2), and swap child1 
      and child2child. We perform the best such swap. */
    float bestArea = 0;
    int bestChild1 = -1, bestChild2 = -1, bestChild2Child = -1;
    for (size_t c2=0; c2<4; c2++)
    {
      DBG( DBG_PRINT(c2) );

      /*! ignore leaf nodes as we cannot descent into them */
      if (parent->child(c2).isLeaf()) continue;
      Node* child2 = parent->child(c2).node(bvh->nodePtr());


      /*! transpose child bounds */
      BBox3fa child2c0,child2c1,child2c2,child2c3;
      //child2->bounds(child2c0,child2c1,child2c2,child2c3);
      child2c0 = child2->bounds( 0 );
      child2c1 = child2->bounds( 1 );
      child2c2 = child2->bounds( 2 );
      child2c3 = child2->bounds( 3 );


      /*! put child1_0 at each child2 position */
      __aligned(16) float cost0[4];
      cost0[0] = area(merge(child1_0,child2c1,child2c2,child2c3));
      cost0[1] = area(merge(child2c0,child1_0,child2c2,child2c3));
      cost0[2] = area(merge(child2c0,child2c1,child1_0,child2c3));
      cost0[3] = area(merge(child2c0,child2c1,child2c2,child1_0));
      const float min0 = min(cost0[0],cost0[1],cost0[2],cost0[3]);

      int pos0 = (int)__bsf(mic_f(min0) == broadcast4to16f(cost0));
      assert(0 <= pos0 && pos0 < 4);
      /*! put child1_1 at each child2 position */
      __aligned(16) float cost1[4];
      cost1[0] = area(merge(child1_1,child2c1,child2c2,child2c3));
      cost1[1] = area(merge(child2c0,child1_1,child2c2,child2c3));
      cost1[2] = area(merge(child2c0,child2c1,child1_1,child2c3));
      cost1[3] = area(merge(child2c0,child2c1,child2c2,child1_1));

      const float min1 = min(cost1[0],cost1[1],cost1[2],cost1[3]);
      int pos1 = (int)__bsf(mic_f(min1) == broadcast4to16f(cost1));
      assert(0 <= pos1 && pos1 < 4);

      /*! put child1_2 at each child2 position */
      __aligned(16) float cost2[4];
      cost2[0] = area(merge(child1_2,child2c1,child2c2,child2c3));
      cost2[1] = area(merge(child2c0,child1_2,child2c2,child2c3));
      cost2[2] = area(merge(child2c0,child2c1,child1_2,child2c3));
      cost2[3] = area(merge(child2c0,child2c1,child2c2,child1_2));
      const float min2 = min(cost2[0],cost2[1],cost2[2],cost2[3]);
      int pos2 = (int)__bsf(mic_f(min2) == broadcast4to16f(cost2));
      assert(0 <= pos2 && pos2 < 4);

      /*! put child1_3 at each child2 position */
      __aligned(16) float cost3[4];
      cost3[0] = area(merge(child1_3,child2c1,child2c2,child2c3));
      cost3[1] = area(merge(child2c0,child1_3,child2c2,child2c3));
      cost3[2] = area(merge(child2c0,child2c1,child1_3,child2c3));
      cost3[3] = area(merge(child2c0,child2c1,child2c2,child1_3));
      const float min3 = min(cost3[0],cost3[1],cost3[2],cost3[3]);
      int pos3 = (int)__bsf(mic_f(min3) == broadcast4to16f(cost3));
      assert(0 <= pos3 && pos3 < 4);

      /*! find best other child */
      float area0123[4] = { min0,min1,min2,min3 };
      int pos[4] = { pos0,pos1,pos2,pos3 };

      for (size_t i=0;i<4;i++)
	{	  
	  const float area_i = area0123[i] - childArea[c2];

	  DBG( DBG_PRINT(i) );
	  DBG( DBG_PRINT(area_i) );
	  DBG( DBG_PRINT(bestArea) );

	  if ((depth+1)+cdepth[i] > BVH4i::maxBuildDepth) continue;
	  if (i == c2) continue;

	  /*! accept a swap when it reduces cost and is not swapping a node with itself */
	  if (area_i < bestArea) {
	    bestArea = area_i;
	    bestChild1 = i;
	    bestChild2 = c2;
	    bestChild2Child = pos[i];

	    DBG( DBG_PRINT(bestChild1) );
	    DBG( DBG_PRINT(bestChild2) );
	  }
	  
	}
    }

    /*! if we did not find a swap that improves the SAH then do nothing */
    if (bestChild1 == -1) return 1+max(cdepth[0],cdepth[1],cdepth[2],cdepth[3]);

    /*! perform the best found tree rotation */
    Node* child2 = parent->child(bestChild2).node(bvh->nodePtr());

    BVH4i::swap(parent,bestChild1,child2,bestChild2Child);

    parent->setBounds(bestChild2,child2->bounds());

    BVH4i::compact(parent);
    BVH4i::compact(child2);
    
    /*! This returned depth is conservative as the child that was
     *  pulled up in the tree could have been on the critical path. */
    cdepth[bestChild1]++; // bestChild1 was pushed down one level
    return 1+max(cdepth[0],cdepth[1],cdepth[2],cdepth[3]); 
    return 0;
  }
}
