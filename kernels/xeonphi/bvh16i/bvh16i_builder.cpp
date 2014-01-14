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

  static mic_i bvh16_node_dist = 0;

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
    //LockStepTaskScheduler::dispatchTask( task_convertToSOALayout, this, threadIndex, threadCount );    
    
    countLeaves(0);

    BVH16i::Node *bvh16 = (BVH16i::Node*)this->prims;
    bvh16[0].reset();
    size_t index16 = 1;
    bvh16_node_dist = 0;
    convertBVH4iToBVH16i(node,
			 node[0].lower.a,
			 node[0].upper.a,
			 bvh16,
			 index16,
			 (unsigned int&)bvh16[0].child[0]);
        
    DBG_PRINT(numPrimitives * sizeof(BVHNode) / sizeof(BVH16i::Node));

    DBG_PRINT(index16);
    DBG_PRINT(index16*sizeof(BVH16i::Node));
 
    unsigned int total = 0;
    float util = 0.0f;
    for (size_t i=0;i<16;i++) {
      util += (float)(i+1) * bvh16_node_dist[i];
      total += bvh16_node_dist[i];
    }
    DBG_PRINT(total);
    std::cout << "node util dist: ";
    for (size_t i=0;i<16;i++) 
      {
	std::cout << i+1 << "[" << (float)bvh16_node_dist[i] * 100.0f / total << "] ";
      }
    std::cout << std::endl;
    DBG_PRINT(100.0f * util / (16.0f * total));
    std::cout << std::endl;
    sleep(1);
    exit(0);
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



  void BVH16iBuilder::convertBVH4iToBVH16i(const BVHNode *const bvh4,
					   const unsigned int bvh4_ext_min, 
					   const unsigned int bvh4_ext_max, 
					   BVH16i::Node *const bvh16,
					   size_t &index16,
					   unsigned int &parent_offset)
    {
      size_t bvh16_used_slots = 0;
      const size_t bvh16_node_index = index16++;
      
      DBG(DBG_PRINT(bvh16_node_index));
      
      bvh16[bvh16_node_index].reset();
      
      {
        const size_t childID = bvhChildID(bvh4_ext_min);
        const size_t children = bvhItems(bvh4_ext_min);
        DBG(
          DBG_PRINT(childID);
          DBG_PRINT(children);
          );
        
        for (size_t i=0;i<children;i++) 
	{
	  DBG(std::cout << "Putting " << bvh4[childID+i] << " in slot " << bvh16_used_slots << std::endl);
	  bvh16[bvh16_node_index].set(bvh16_used_slots++,bvh4[childID+i]);
	}
      }
      
      DBG(DBG_PRINT(bvh16[bvh16_node_index]));
      
      while(bvh16_used_slots < 16)
      {
	DBG(
          std::cout << std::endl << std::flush;
          DBG_PRINT(bvh16_used_slots);
          DBG_PRINT(bvh16[bvh16_node_index]);
          );
	mic_f node_area = bvh16[bvh16_node_index].area();
	DBG(DBG_PRINT(node_area));
        
        
	ssize_t max_index = -1;
	float max_area = 0.0f;
	const unsigned int free_slots = 16 - bvh16_used_slots;
        
	ssize_t max_index_small = -1;
	ssize_t min_children_small = 16;
	float max_area_small = 0.0f;
        
	for (size_t i=0;i<bvh16_used_slots;i++)
        {
          if (bvhLeaf(bvh16[bvh16_node_index].child[i])) continue;
          if ((bvhItems(bvh16[bvh16_node_index].child[i]) + bvh16_used_slots - 1) <= 16 && 
              node_area[i] > max_area)
          {      
            //if (bvh16_used_slots >=8)
            if (bvh16[bvh16_node_index].data[i] >= 8 && bvh16[bvh16_node_index].data[i] <= 16) continue;	      
            
            max_index = i;
            max_area = node_area[i];
            
            DBG(
              DBG_PRINT(i);
              DBG_PRINT(max_index);
              DBG_PRINT(max_area);
              );
            
          }
	  
          if (bvh16[bvh16_node_index].data[i] <= free_slots && 
              bvh16[bvh16_node_index].data[i] < min_children_small)// &&
            //node_area[i] > max_area_small)
          {
            min_children_small = bvh16[bvh16_node_index].data[i];
            max_index_small = i;
            max_area_small = node_area[i];
            
            DBG(
              DBG_PRINT(i);
              DBG_PRINT(max_index_small);
              DBG_PRINT(max_area_small);
              );
            
          }	  
          
        }
        
	if (max_index == -1) 
        {
          break;
        }
        
	if (max_index_small != -1)
	  max_index = max_index_small;
        
        
	DBG(std::cout << "1" << std::endl << std::flush);
	const unsigned int parent_index = bvh16[bvh16_node_index].child[max_index];
	DBG(DBG_PRINT(parent_index));
        
	
	bvh16[bvh16_node_index].shift(max_index);
	bvh16_used_slots--;
        
	assert(!bvhLeaf(parent_index));
	assert( bvhItems(parent_index) + bvh16_used_slots <= 16);
        
	DBG(std::cout << "2" << std::endl << std::flush);
        
	const size_t childID = bvhChildID(parent_index);
	const size_t children = bvhItems(parent_index);
	for (size_t i=0;i<children;i++) 
        {
          DBG(std::cout << "Putting node " << childID+i << " -> " << bvh4[childID+i] << " in slot " << bvh16_used_slots << std::endl);
          bvh16[bvh16_node_index].set(bvh16_used_slots++,bvh4[childID+i]);
          
        }
	if (bvh16_used_slots > 16) FATAL("HERE");
        
	DBG(std::cout << "3" << std::endl << std::flush);
	assert(bvh16_used_slots <= 16);
      }
      
      
      DBG(
        std::cout << "FINAL: " << std::endl << std::flush;
        DBG_PRINT(bvh16_used_slots);
        DBG_PRINT(bvh16[bvh16_node_index]);
        );
      
      DBG(DBG_PRINT(bvh16[bvh16_node_index].child));
      
      
      DBG(DBG_PRINT(bvh16_used_slots));
      
      parent_offset = (unsigned int)(sizeof(BVH16i::Node) * bvh16_node_index);
      
      bvh16_node_dist[bvh16_used_slots-1]++;
      
      BVH16i::Node &b16 = bvh16[bvh16_node_index];
      
      DBG(DBG_PRINT(b16));
      for (size_t i=0;i<bvh16_used_slots;i++)
        if (!bvhLeaf(b16.child[i]))
	{
	  DBG(std::cout << "RECURSE FOR " << b16.child[i] << " " << b16.data[i] << std::endl << std::flush);
	  convertBVH4iToBVH16i(bvh4,
			       b16.child[i],
			       b16.data[i],
			       bvh16,			      
			       index16,
			       (unsigned int&)b16.child[i]);
	}
        else
	{
	  b16.child[i] = (b16.child[i] ^ BVH_LEAF_MASK) | QBVH_LEAF_MASK;
	}
      
      DBG(std::cout << "DONE" << std::endl << std::flush);
      
    }


}
