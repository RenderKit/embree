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

#include "bvh16i/bvh16i_builder.h"

namespace embree
{
#define DBG(x) x

#define L1_PREFETCH_ITEMS 2
#define L2_PREFETCH_ITEMS 16

  Builder* BVH16iBuilder::create (void* accel, BuildSource* source, void* geometry, size_t mode ) 
  { 
    Builder* builder = new BVH16iBuilder((BVH16i*)accel,source,geometry);
    return builder;
  }

  void BVH16iBuilder::printBuilderName()
  {
    std::cout << "building BVH16i with binned SAH builder (MIC) ... " << std::endl;    
  }

  __forceinline void convertToBVH4Layout(BVHNode *__restrict__ const bptr)
  {
    const mic_i box01 = load16i((int*)(bptr + 0));
    const mic_i box23 = load16i((int*)(bptr + 2));

    const mic_i box_min01 = permute<2,0,2,0>(box01);
    const mic_i box_max01 = permute<3,1,3,1>(box01);

    const mic_i box_min23 = permute<2,0,2,0>(box23);
    const mic_i box_max23 = permute<3,1,3,1>(box23);
    const mic_i box_min0123 = select(0x00ff,box_min01,box_min23);
    const mic_i box_max0123 = select(0x00ff,box_max01,box_max23);

    const mic_m min_d_mask = bvhLeaf(box_min0123) != mic_i::zero();
    const mic_i childID    = bvhChildID(box_min0123)>>2;
    const mic_i min_d_node = qbvhCreateNode(childID,mic_i::zero());
    const mic_i min_d_leaf = ((box_min0123 ^ BVH_LEAF_MASK)<<1) | QBVH_LEAF_MASK; // * 2 as accel size is 128 bytes now
    const mic_i min_d      = select(min_d_mask,min_d_leaf,min_d_node);
    const mic_i bvh4_min   = select(0x7777,box_min0123,min_d);
    const mic_i bvh4_max   = box_max0123;
    store16i_nt((int*)(bptr + 0),bvh4_min);
    store16i_nt((int*)(bptr + 2),bvh4_max);
  }

  void BVH16iBuilder::convertQBVHLayout(const size_t threadIndex, const size_t threadCount)
  {
    DBG(PING);
    LockStepTaskScheduler::dispatchTask( task_convertToSOALayout, this, threadIndex, threadCount );    
  }

  void BVH16iBuilder::countLeaves(const size_t index)
  {
    BVHNode &entry = node[index];

    if (entry.isLeaf())
      {
	entry.upper.a = 1;
      }
    else
      {
	const unsigned int childID = entry.firstChildID();
	const unsigned int children = entry.items();

	unsigned int leaves = 0;
	for (unsigned int i=0;i<children;i++) 
	  {
	    countLeaves(childID+i);
	    leaves += node[childID+i].upper.a;
	  }      
	entry.upper.a = leaves;
      }
  }

}
