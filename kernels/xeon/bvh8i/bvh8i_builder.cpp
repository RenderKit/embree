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
#include "geometry/triangle8.h"

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

    // =======================================================================================================
    // =======================================================================================================
    // =======================================================================================================

    static unsigned int countBVH4iNodes(BVH4i::Node *base, BVH4i::NodeRef& n)
    {
      if (n.isLeaf()) 
	return 0;
      else
	{
	  unsigned int nodes = 1;
	  BVH4i::Node *node = n.node(base);
	  for (size_t i=0;i<node->numValidChildren();i++)
	    nodes += countBVH4iNodes(base,node->child(i));
	  return nodes;
	}
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

    // =======================================================================================================
    // =======================================================================================================
    // =======================================================================================================

    static void convertBVH4itoBVH8i(BVH4i::Node *bvh4i,
				    BVH4i::NodeRef &ref, 
				    unsigned int numLeavesInSubTree, 
				    BVH8i::Node *bvh8i,
				    size_t &index8,
				    BVH4i::NodeRef &parent_offset,
				    avxi &bvh8i_node_dist)
    {
      size_t bvh8i_used_slots = 0;
      const size_t bvh8i_node_index = index8++;
      
      
      bvh8i[bvh8i_node_index].reset();

      {
	BVH4i::Node *node4 = ref.node(bvh4i);
	DBG_PRINT(*node4);
	unsigned int children = node4->numValidChildren();
        DBG_PRINT(children);
	for (size_t i=0;i<children;i++) 
	  bvh8i[bvh8i_node_index].set(bvh8i_used_slots++,*node4,i);      
	DBG_PRINT( bvh8i[bvh8i_node_index] );
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
	      if (bvh8i[bvh8i_node_index].children[i].isLeaf()) continue;

	      BVH4i::Node *node4 = bvh8i[bvh8i_node_index].children[i].node(bvh4i);
	      unsigned int children = node4->numValidChildren();

	      if ((children + bvh8i_used_slots - 1) <= 8 && 
		  node_area[i] > max_area)
		{      
		  //if (bvh8_used_slots >=8)
		  //if (bvh8i[bvh8i_node_index].data[i] >= 6 && bvh8i[bvh8i_node_index].data[i] <= 8) continue;	      
            
		  max_index = i;
		  max_area = node_area[i];
                        
		}
	  
#if 1
	      if (bvh8i[bvh8i_node_index].data[i] <= free_slots && 
		  bvh8i[bvh8i_node_index].data[i] < min_children_small)// &&
		//node_area[i] > max_area_small)
		{
		  min_children_small = bvh8i[bvh8i_node_index].data[i];
		  max_index_small = i;
		  max_area_small = node_area[i];
                        
		}	  
#endif     
	    }

	  if (max_index == -1) break;
        
	  if (max_index_small != -1)
	    max_index = max_index_small;
        
        
	  const BVH4i::NodeRef parent_ref = bvh8i[bvh8i_node_index].children[max_index];
        
	
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
      

      parent_offset = (unsigned int)(sizeof(BVH8i::Node) * bvh8i_node_index);
      
      bvh8i_node_dist[bvh8i_used_slots-1]++;
      
      BVH8i::Node &b8 = bvh8i[bvh8i_node_index];
      
      for (size_t i=0;i<bvh8i_used_slots;i++)
	{
	  if (b8.children[i].isNode())
	    {
	      convertBVH4itoBVH8i(bvh4i,
				  b8.children[i],
				  b8.data[i],
				  bvh8i,			      
				  index8,
				  b8.children[i],
				  bvh8i_node_dist);
	    }      
	}
    }

    // =======================================================================================================
    // =======================================================================================================
    // =======================================================================================================

    void BVH8iBuilderTriangle8::build(size_t threadIndex, size_t threadCount) 
    {
      PING;
      bvh4i_builder8->build(threadIndex,threadCount);
      DBG_PRINT(*(Triangle8*)bvh4i_builder8->bvh->accel);
      std::cout << "BVH4i BUILD DONE" << std::endl << std::flush;
      unsigned int numBVH4iNodes = countBVH4iNodes((BVH4i::Node*)bvh4i_builder8->bvh->nodePtr(),bvh4i_builder8->bvh->root);
      DBG_PRINT(numBVH4iNodes);
      unsigned int totalLeaves = countLeavesButtomUp((BVH4i::Node*)bvh4i_builder8->bvh->nodePtr(),bvh4i_builder8->bvh->root);
      DBG_PRINT(totalLeaves);
      avxi bvh8i_node_dist = 0;
      BVH8i::Node *bvh8i_base = (BVH8i::Node *)os_malloc(sizeof(BVH8i::Node) * numBVH4iNodes);
      BVH4i::NodeRef bvh8i_root;
      size_t index8 = 0;
      convertBVH4itoBVH8i((BVH4i::Node*)bvh4i_builder8->bvh->nodePtr(),
			  bvh4i_builder8->bvh->root,
			  totalLeaves,
			  bvh8i_base,
			  index8,
			  bvh8i_root,
			  bvh8i_node_dist);

      std::cout << "BVH4i TO BVH8I CONVERSION DONE" << std::endl << std::flush;
      DBG_PRINT(index8);
      DBG_PRINT(bvh8i_root);
      DBG_PRINT(bvh8i_base[0]);
 
      /* bvh8i node util */
      {
	unsigned int total = 0;
	float util = 0.0f;
	for (size_t i=0;i<8;i++) {
	  util += (float)(i+1) * bvh8i_node_dist[i];
	  total += bvh8i_node_dist[i];
	}
	DBG_PRINT(total);
	std::cout << "bvh8i node util dist: ";
	DBG_PRINT(bvh8i_node_dist);
	float sum = 0;
	for (size_t i=0;i<8;i++) 
	  {
	    sum += (float)bvh8i_node_dist[i] * 100.0f / total;
	    std::cout << i+1 << "[" << (float)bvh8i_node_dist[i] * 100.0f / total << "%, sum " << sum << "%] ";
	  }
	std::cout << std::endl;
	DBG_PRINT(100.0f * util / (8.0f * total));
	std::cout << std::endl;
      }

      bvh4i_builder8->bvh->root = bvh8i_root;
      bvh4i_builder8->bvh->qbvh = bvh8i_base; 

      DBG_PRINT(bvh8i_root);
      DBG_PRINT(bvh8i_base);

      // bvh->init();
      // allocateData();
      // TaskScheduler::executeTask(threadIndex,threadCount,_build_parallel,this,TaskScheduler::getNumThreads(),"build_parallel");
    }
    
    
    
    // =======================================================================================================
    // =======================================================================================================
    // =======================================================================================================
    

     Builder* BVH8iTriangle8BuilderObjectSplit (void* bvh, BuildSource* source, Scene* scene, const size_t minLeafSize, const size_t maxLeafSize) {
       return new BVH8iBuilderTriangle8((BVH4i*)bvh,source,scene,minLeafSize,maxLeafSize);
     }
  }
};
  

