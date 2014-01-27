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

#include "bvh4i/bvh4i.h"
#include "bvh4i/bvh4i_builder.h"

#include "bvh8i_builder.h"

#include "limits.h"
#include "sys/sync/barrier.h"

#include "bvh8i/bvh8i.h"

using namespace embree;

#define __ALIGN(x) __declspec(align(x))


#define CONVERT_TO_BVH8

namespace embree
{
  namespace isa
  {
    // =======================================================================================================
    // =======================================================================================================
    // =======================================================================================================
    
    BVH8iBuilderTriangle8::BVH8iBuilderTriangle8 (BVH4i* bvh, BuildSource* source, void* geometry, const size_t minLeafSize, const size_t maxLeafSize) 

    {
      bvh4i_builder8 = (BVH4iBuilder8 *)BVH4iBuilderObjectSplit8(bvh,source,geometry,minLeafSize,maxLeafSize);
    } 
    
    static unsigned int countLeavesButtomUp(BVH4i::Node *base, BVH4i::NodeRef& n)
    {
      if (n.isLeaf()) 
	return 1;
      else
	{
	  BVH4i::Node *node = n.node(base);
	  for (size_t i=0;i<4;i++)
	    node->data[i] = 0;
	  unsigned int leaves = 0;
	  for (size_t i=0;i<node->numValidChildren();i++)
	    {
	      node->data[i] = countLeavesButtomUp(base,node->child(i));
	      leaves += node->data[i];
	    }
	  return leaves;
	}
    }


    static void convertBVH4itoBVH8i(BVH4i::Node *bvh4i,
				    BVH4i::NodeRef &ref, 
				    unsigned int numLeavesInSubTree, 
				    BVH8i::BVH8iNode *const bvh8i,
				    size_t &index8,
				    unsigned int &parent_offset,
				    avxi &bvh8i_node_dist)
    {
      size_t bvh8i_used_slots = 0;
      const size_t bvh8i_node_index = index8++;
      
      
      bvh8i[bvh8i_node_index].reset();

      {
	BVH4i::Node *node4 = ref.node(bvh4i);
	unsigned int children = node4->numValidChildren();
        
	for (size_t i=0;i<children;i++) 
	  bvh8i[bvh8i_node_index].set(bvh8i_used_slots++,*node4,i);      
      }

      while(bvh8i_used_slots < 8)
	{
	  const avxf node_area = bvh8i[bvh8i_node_index].area();
                
	  ssize_t max_index = -1;
	  float max_area = 0.0f;
	  const unsigned int free_slots = 8 - bvh8i_used_slots;
        
	  ssize_t max_index_small = -1;
	  ssize_t min_children_small = 8;
	  float max_area_small = 0.0f;
        
	  for (size_t i=0;i<bvh8i_used_slots;i++)
	    {
	      if (bvhLeaf(bvh8i[bvh8i_node_index].min_d[i])) continue;
	      if ((bvhItems(bvh8i[bvh8i_node_index].min_d[i]) + bvh8i_used_slots - 1) <= 8 && 
		  node_area[i] > max_area)
		{      
		  //if (bvh8_used_slots >=8)
		  if (bvh8i[bvh8i_node_index].max_d[i] >= 4 && bvh8i[bvh8i_node_index].max_d[i] <= 8) continue;	      
            
		  max_index = i;
		  max_area = node_area[i];
                        
		}
	  
	      if (bvh8i[bvh8i_node_index].max_d[i] <= free_slots && 
		  bvh8i[bvh8i_node_index].max_d[i] < min_children_small)// &&
		//node_area[i] > max_area_small)
		{
		  min_children_small = bvh8i[bvh8i_node_index].max_d[i];
		  max_index_small = i;
		  max_area_small = node_area[i];
                        
		}	  
          
	    }

	  if (max_index == -1) break;
        
	  if (max_index_small != -1)
	    max_index = max_index_small;
        
        
	  const BVH4i::NodeRef parent_ref = bvh8i[bvh8i_node_index].min_d[max_index];
        
	
	  bvh8i[bvh8i_node_index].shift(max_index);
	  bvh8i_used_slots--;
        
        
	  BVH4i::Node *node4 = parent_ref.node(bvh4i);
	  unsigned int children = node4->numValidChildren();
        
	  //const size_t childID = bvhChildID(parent_index);
	  //const size_t children = bvhItems(parent_index);

	  for (size_t i=0;i<children;i++) 
	    bvh8i[bvh8i_node_index].set(bvh8i_used_slots++,*node4,i);

	  if (bvh8i_used_slots > 8) FATAL("HERE");
        
	  assert(bvh8i_used_slots <= 8);

	}
      

#if 0            
      parent_offset = (unsigned int)(sizeof(BVH8i::BVH8iNode) * bvh8_node_index);
      
      bvh8_node_dist[bvh8_used_slots-1]++;
      
      BVH8i::BVH8iNode &b8 = bvh8[bvh8_node_index];
      
      for (size_t i=0;i<bvh8_used_slots;i++)
        if (!bvhLeaf(b8.min_d[i]))
	{
	  convertBVH4toBVH8(bvh4,
			    b8.min_d[i],
			    b8.max_d[i],
			    bvh8,			      
			    index8,
			    (unsigned int&)b8.min_d[i],
			    bvh8_node_dist);
	}
        else
	{
	  b8.min_d[i] = (b8.min_d[i] ^ BVH_LEAF_MASK) | QBVH_LEAF_MASK;
	}
#endif
      
    }

    void BVH8iBuilderTriangle8::build(size_t threadIndex, size_t threadCount) 
    {
      bvh4i_builder8->build(threadIndex,threadCount);
      std::cout << "DONE" << std::endl << std::flush;
      unsigned int totalLeaves = countLeavesButtomUp((BVH4i::Node*)bvh4i_builder8->bvh->nodePtr(),bvh4i_builder8->bvh->root);
      DBG_PRINT(totalLeaves);
      exit(0);

      // bvh->init();
      // allocateData();
      // TaskScheduler::executeTask(threadIndex,threadCount,_build_parallel,this,TaskScheduler::getNumThreads(),"build_parallel");
    }
    
    // =======================================================================================================
    // =======================================================================================================
    // =======================================================================================================
    
    
    // =======================================================================================================
    // =======================================================================================================
    // =======================================================================================================
#define DBG2(x) x
#define DBG3(x) 
    
    
    static void convertBVH4toBVH8(const BVHNode *const bvh4,
                                  const unsigned int bvh4_ext_min, 
                                  const unsigned int bvh4_ext_max, 
                                  BVH8i::BVH8iNode *const bvh8,
                                  size_t &index8,
                                  unsigned int &parent_offset,
                                  avxi &bvh8_node_dist)
    {
      size_t bvh8_used_slots = 0;
      const size_t bvh8_node_index = index8++;
      
      DBG3(DBG_PRINT(bvh8_node_index));
      
      bvh8[bvh8_node_index].reset();
      
      {
        const size_t childID = bvhChildID(bvh4_ext_min);
        const size_t children = bvhItems(bvh4_ext_min);
        DBG3(
          DBG_PRINT(childID);
          DBG_PRINT(children);
          );
        
        for (size_t i=0;i<children;i++) 
	{
	  DBG3(std::cout << "Putting " << bvh4[childID+i] << " in slot " << bvh8_used_slots << std::endl);
	  bvh8[bvh8_node_index].set(bvh8_used_slots++,bvh4[childID+i]);
	}
      }
      
      DBG3(DBG_PRINT(bvh8[bvh8_node_index]));
      
      while(bvh8_used_slots < 8)
      {
	DBG3(
          std::cout << std::endl << std::flush;
          DBG_PRINT(bvh8_used_slots);
          DBG_PRINT(bvh8[bvh8_node_index]);
          );
	avxf node_area = bvh8[bvh8_node_index].area();
	DBG3(DBG_PRINT(node_area));
        
        
	ssize_t max_index = -1;
	float max_area = 0.0f;
	const unsigned int free_slots = 8 - bvh8_used_slots;
        
	ssize_t max_index_small = -1;
	ssize_t min_children_small = 8;
	float max_area_small = 0.0f;
        
	for (size_t i=0;i<bvh8_used_slots;i++)
        {
          if (bvhLeaf(bvh8[bvh8_node_index].min_d[i])) continue;
          if ((bvhItems(bvh8[bvh8_node_index].min_d[i]) + bvh8_used_slots - 1) <= 8 && 
              node_area[i] > max_area)
          {      
            //if (bvh8_used_slots >=8)
            if (bvh8[bvh8_node_index].max_d[i] >= 8 && bvh8[bvh8_node_index].max_d[i] <= 8) continue;	      
            
            max_index = i;
            max_area = node_area[i];
            
            DBG3(
              DBG_PRINT(i);
              DBG_PRINT(max_index);
              DBG_PRINT(max_area);
              );
            
          }
	  
          if (bvh8[bvh8_node_index].max_d[i] <= free_slots && 
              bvh8[bvh8_node_index].max_d[i] < min_children_small)// &&
            //node_area[i] > max_area_small)
          {
            min_children_small = bvh8[bvh8_node_index].max_d[i];
            max_index_small = i;
            max_area_small = node_area[i];
            
            DBG3(
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
        
        
	DBG3(std::cout << "1" << std::endl << std::flush);
	const unsigned int parent_index = bvh8[bvh8_node_index].min_d[max_index];
	DBG3(DBG_PRINT(parent_index));
        
	
	bvh8[bvh8_node_index].shift(max_index);
	bvh8_used_slots--;
        
	assert(!bvhLeaf(parent_index));
	assert( bvhItems(parent_index) + bvh8_used_slots <= 8);
        
	DBG3(std::cout << "2" << std::endl << std::flush);
        
	const size_t childID = bvhChildID(parent_index);
	const size_t children = bvhItems(parent_index);
	for (size_t i=0;i<children;i++) 
        {
          DBG3(std::cout << "Putting node " << childID+i << " -> " << bvh4[childID+i] << " in slot " << bvh8_used_slots << std::endl);
          bvh8[bvh8_node_index].set(bvh8_used_slots++,bvh4[childID+i]);
          
        }
	if (bvh8_used_slots > 8) FATAL("HERE");
        
	DBG3(std::cout << "3" << std::endl << std::flush);
	assert(bvh8_used_slots <= 8);
      }
      
      
      DBG3(
        std::cout << "FINAL: " << std::endl << std::flush;
        DBG_PRINT(bvh8_used_slots);
        DBG_PRINT(bvh8[bvh8_node_index]);
        );
      
      DBG3(DBG_PRINT(bvh8[bvh8_node_index].min_d));
      
      
      DBG3(DBG_PRINT(bvh8_used_slots));
      
      parent_offset = (unsigned int)(sizeof(BVH8i::BVH8iNode) * bvh8_node_index);
      
      bvh8_node_dist[bvh8_used_slots-1]++;
      
      BVH8i::BVH8iNode &b8 = bvh8[bvh8_node_index];
      
      DBG3(DBG_PRINT(b8));
      for (size_t i=0;i<bvh8_used_slots;i++)
        if (!bvhLeaf(b8.min_d[i]))
	{
	  DBG3(std::cout << "RECURSE FOR " << b8.min_d[i] << " " << b8.max_d[i] << std::endl << std::flush);
	  convertBVH4toBVH8(bvh4,
			    b8.min_d[i],
			    b8.max_d[i],
			    bvh8,			      
			    index8,
			    (unsigned int&)b8.min_d[i],
			    bvh8_node_dist);
	}
        else
	{
	  b8.min_d[i] = (b8.min_d[i] ^ BVH_LEAF_MASK) | QBVH_LEAF_MASK;
	}
      
      DBG3(std::cout << "DONE" << std::endl << std::flush);
      
    }
    // =======================================================================================================
    // =======================================================================================================
    // =======================================================================================================
    

     Builder* BVH8iTriangle8BuilderObjectSplit (void* bvh, BuildSource* source, Scene* scene, const size_t minLeafSize, const size_t maxLeafSize) {
       return new BVH8iBuilderTriangle8((BVH4i*)bvh,source,scene,minLeafSize,maxLeafSize);
     }
  }
};
  

