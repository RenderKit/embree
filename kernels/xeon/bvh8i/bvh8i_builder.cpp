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

#include "bvh4i/bvh4i.h"
#include "bvh4i/bvh4i_builder.h"

#include "bvh8i_builder.h"

#include "limits.h"
#include "sys/sync/barrier.h"

#include "bvh8i/bvh8i.h"
#include "geometry/triangle8.h"

using namespace embree;

#define __ALIGN(x) __declspec(align(x))

#define DBG(x) x

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
				    BVH4i::NodeRef &parent_offset)
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

      DBG(std::cout << std::endl);
      DBG(DBG_PRINT(bvh8i_node_index));
      DBG(DBG_PRINT(numLeavesInSubTree));

      while(bvh8i_used_slots < 8)
	{
	  const avxf node_area = bvh8i[bvh8i_node_index].area();
                
	  ssize_t max_index = -1;
	  float max_area = 0.0f;
	  const unsigned int free_slots = 8 - bvh8i_used_slots;
        
	  ssize_t max_index_small = -1;
	  ssize_t min_children_small = 8;
	  float max_area_small = 0.0f;

	  DBG(DBG_PRINT(bvh8i_used_slots));
	  DBG(DBG_PRINT(*(avxi*)bvh8i[bvh8i_node_index].data));
        
	  for (size_t i=0;i<bvh8i_used_slots;i++)
	    {
	      if (bvh8i[bvh8i_node_index].children[i].isLeaf()) continue;

	      BVH4i::Node *node4 = bvh8i[bvh8i_node_index].children[i].node(bvh4i);
	      unsigned int children = node4->numValidChildren();

#if 0
	      if (bvh8i[bvh8i_node_index].data[i] < 8)
		if (children != bvh8i[bvh8i_node_index].data[i]) 
		  {
		    DBG_PRINT(children);
		    DBG_PRINT(bvh8i[bvh8i_node_index].data[i]);		    
		  }
#endif
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

	  DBG(DBG_PRINT(max_index));
	  DBG(DBG_PRINT(max_area));

	  DBG(DBG_PRINT(max_index_small));
	  DBG(DBG_PRINT(min_children_small));

	  if (max_index == -1) break;
        
	  if (max_index_small != -1)
	    max_index = max_index_small;
        
        
	  const BVH4i::NodeRef parent_ref = bvh8i[bvh8i_node_index].children[max_index];
        
	
	  bvh8i[bvh8i_node_index].shift(max_index);
	  bvh8i_used_slots--;
        
        
	  BVH4i::Node *node4 = parent_ref.node(bvh4i);
	  unsigned int children = node4->numValidChildren();
        
	  for (size_t i=0;i<children;i++) 
	    bvh8i[bvh8i_node_index].set(bvh8i_used_slots++,*node4,i);

	  if (bvh8i_used_slots > 8) FATAL("HERE");
        
	  assert(bvh8i_used_slots <= 8);

	}

      DBG(DBG_PRINT(bvh8i_used_slots));

      BVH8i::Node &b8 = bvh8i[bvh8i_node_index];

      // try to pull up a child to fill nodes



#if 1
      while (bvh8i_used_slots < 8)
	{	 
	  DBG(std::cout << "PULL UP CHILD" << std::endl);
	  ssize_t max_index = -1;
	  ssize_t max_leaves = 4;

	  for (size_t i=0;i<bvh8i_used_slots;i++)
	    if (b8.children[i].isNode())
	      if (b8.data[i] > max_leaves)
		{
		  max_leaves = b8.data[i];
		  max_index = i;
		}

          if (max_index == -1) break;

	  // for (size_t i=0;i<bvh8i_used_slots;i++)
	  //   if (b8.children[i].isNode())
	  //     if (b8.data[i] == 2)
          //       max_index = i;

	  DBG(DBG_PRINT(*(avxi*)b8.data));
	  DBG(DBG_PRINT(max_index));

	  if (max_index != -1) 
	    {
	      BVH4i::NodeRef child = b8.children[max_index];
	      BVH4i::Node *node4 = child.node(bvh4i);
	      unsigned int children4 = node4->numValidChildren();
	      DBG(DBG_PRINT(node4->numValidChildren()));
              int child4_index = -1;
              int child4_leaves = 0;
              for (size_t i=0;i<children4;i++)
                if (node4->data[i] > child4_leaves)
                  {
                    child4_leaves = node4->data[i];
                    child4_index = i;
                  }
              if (child4_index == -1) break;
              assert(child4_index != -1);
              DBG_PRINT(child4_index);
              DBG_PRINT(children4);

              BVH4i::swap(node4,child4_index,node4,children4-1);

	      b8.set(bvh8i_used_slots++,*node4,children4-1);
	      node4->children[children4-1] = BVH4i::emptyNode;

	      DBG(DBG_PRINT(node4->data[children4-1]));

	      b8.data[max_index] -= node4->data[children4-1];

	      DBG(DBG_PRINT(node4->numValidChildren()));

	    }

	  DBG(DBG_PRINT(*(avxi*)b8.data));
	  DBG(DBG_PRINT(bvh8i_used_slots));
	  
	  for (size_t i=0;i<bvh8i_used_slots;i++)
	    if (b8.children[i].isNode())
	      {
		BVH4i::NodeRef child = b8.children[i];
		BVH4i::Node *node4 = child.node(bvh4i);
		unsigned int children4 = node4->numValidChildren();
		if (children4==1)
		  b8.set(i,*node4,children4-1);
		//if (BVH4i::NodeRef(b8.children[i]).isNode()) FATAL("isNode");
	      }
	}
#endif

      for (size_t i=0;i<bvh8i_used_slots;i++)
        {
          DBG_PRINT(i);
          if (b8.children[i].isNode())
            DBG_PRINT(b8.data[i]);
        }

      //assert(bvh8i_used_slots == 8);

      parent_offset = (unsigned int)(sizeof(BVH8i::Node) * bvh8i_node_index);
            

      DBG(
	  {
	    bool recurse = false;
	    for (size_t i=0;i<bvh8i_used_slots;i++)
	      if (b8.children[i].isNode())
		recurse = true;
	    DBG(DBG_PRINT(recurse));
	  }
	  );

      
      for (size_t i=0;i<bvh8i_used_slots;i++)
	  if (b8.children[i].isNode())
	      convertBVH4itoBVH8i(bvh4i,
				  b8.children[i],
				  b8.data[i],
				  bvh8i,			      
				  index8,
				  b8.children[i]);
    }

    __forceinline size_t numValidChildren(BVH8i::Node* base, BVH4i::NodeRef& node) 
    {
      if (node.isLeaf()) return 0;
      BVH8i::Node* n = (BVH8i::Node*)node.node(base);
      return n->numValidChildren();
    }

    static void refitBVH8i(BVH8i::Node *bvh8i,
                           BVH4i::NodeRef &node,
                           BVH8i::Node &root,
                           const unsigned int root_slot)
    {
      if (node.isNode()) 
        {
          BVH8i::Node* n = (BVH8i::Node*)node.node(bvh8i);
          
          BBox3fa bounds = empty;
          size_t children = n->numValidChildren();
          for (size_t c=0; c<children; c++) 
            {
              refitBVH8i(bvh8i,n->child(c),*n,c);
              bounds.extend( n->extract(c) );
            }
          root.setBounds(root_slot, bounds);
          if (children == 1) 
            root.set(root_slot,*n,0);
        }
    }

    static size_t compactBVH8i(BVH8i::Node *bvh8i,
                               BVH4i::NodeRef &node)
    {
      if (node.isNode()) 
        {
          BVH8i::Node* n = (BVH8i::Node*)node.node(bvh8i);

          size_t children = n->numValidChildren();
          for (size_t c=0; c<children; c++) 
            n->data[c] = compactBVH8i(bvh8i,n->child(c));

          while(1)
            {
              ssize_t min_index = -1;
              ssize_t min_slots =  8;
              for (size_t c=0; c<children; c++) 
                {
                  BVH4i::NodeRef child = n->child(c);
                  if (child.isLeaf()) continue;
                  
                  size_t child_children = numValidChildren(bvh8i,child);
                  
                  if (child_children < min_slots)
                    {
                      min_index = c;
                      min_slots = child_children;
                    }              
                }

              if (min_index == -1) break;

              BBox3fa min_bounds = n->extract(min_index);

              ssize_t pair_index = -1;
              ssize_t pair_slots =  8;
              float pair_sah = pos_inf;

              for (size_t c=0; c<children; c++) 
                {
                  if (c == min_index) continue;
                  BVH4i::NodeRef child = n->child(c);
                  if (child.isLeaf()) continue;
                  
                  size_t child_children = numValidChildren(bvh8i,child);

                  BBox3fa b = min_bounds;
                  b.extend( n->area(c) );
                  float new_sah = area( b );
                  
                  if (child_children <= pair_slots && min_slots + child_children <= 8 && new_sah < pair_sah)
                    {
                      pair_index = c;
                      pair_slots = child_children;
                      pair_sah = new_sah;
                    }              
                }
              if (pair_index == -1) break;

              BVH4i::NodeRef child0 = n->child( min_index);
              BVH4i::NodeRef child1 = n->child(pair_index);

              BVH8i::Node* p_child0 = (BVH8i::Node*)child0.node(bvh8i);
              BVH8i::Node* p_child1 = (BVH8i::Node*)child1.node(bvh8i);

              p_child0->merge(*p_child1);

              n->shift(pair_index);
              n->setInvalid(children-1);

              
            }

          return n->numValidChildren();
        }
      else 
        {
          return 0;
        }
      
    }

    // =======================================================================================================
    // =======================================================================================================
    // =======================================================================================================

    void BVH8iBuilderTriangle8::build(size_t threadIndex, size_t threadCount) 
    {
      bvh4i_builder8->build(threadIndex,threadCount);
      unsigned int numBVH4iNodes = countBVH4iNodes((BVH4i::Node*)bvh4i_builder8->bvh->nodePtr(),bvh4i_builder8->bvh->root);
      unsigned int totalLeaves = countLeavesButtomUp((BVH4i::Node*)bvh4i_builder8->bvh->nodePtr(),bvh4i_builder8->bvh->root);
      avxi bvh8i_node_dist = 0;
      BVH8i::Node *bvh8i_base = (BVH8i::Node *)os_malloc(sizeof(BVH8i::Node) * numBVH4iNodes);
      BVH4i::NodeRef bvh8i_root;
      size_t index8 = 0;
      convertBVH4itoBVH8i((BVH4i::Node*)bvh4i_builder8->bvh->nodePtr(),
			  bvh4i_builder8->bvh->root,
			  totalLeaves,
			  bvh8i_base,
			  index8,
			  bvh8i_root);


      if (g_verbose >= 2)
	{

	}

      std::cout << "BVH4i TO BVH8I CONVERSION DONE" << std::endl << std::flush;
      DBG_PRINT(numBVH4iNodes);
      DBG_PRINT(numBVH4iNodes*sizeof(BVH4i::Node));
      DBG_PRINT(totalLeaves);
      DBG_PRINT(index8);
      DBG_PRINT(index8*sizeof(BVH8i::Node));


#if 0
      unsigned int root_data = 0;
      root_data = compactBVH8i(bvh8i_base,bvh8i_root);      
      DBG_PRINT(root_data);
#endif

#if 1
      BVH8i::Node tmp_root;
      tmp_root.reset();
      refitBVH8i(bvh8i_base,bvh8i_root,tmp_root,0);      
      DBG_PRINT(tmp_root);
#endif

      bvh8i_node_dist = 0;
      bvh4i_builder8->bvh->root = bvh8i_root;
#if !defined(USE_QUANTIZED_NODES)
      bvh4i_builder8->bvh->qbvh = bvh8i_base; 


      std::cout << "SAH = " << BVH8i::sah8( bvh8i_base, bvh8i_root, bvh8i_node_dist ) << std::endl;

#else
      DBG_PRINT(sizeof(BVH8i::Quantized8BitNode));
      DBG_PRINT(sizeof(BVH8i::Node));
      DBG_PRINT(index8);

#if 1
      BVH8i::Quantized8BitNode *bvh8i_quantized = (BVH8i::Quantized8BitNode *)os_malloc(sizeof(BVH8i::Quantized8BitNode) * index8);
      for (size_t i=0;i<index8;i++) bvh8i_quantized[i].init( bvh8i_base[i] );
      
      std::cout << "8BIT QUANTIZATION DONE" << std::endl << std::flush;
      std::cout << "SAH = " << BVH8i::sah8_quantized( bvh8i_quantized, bvh8i_root,bvh8i_node_dist ) << std::endl;

      bvh4i_builder8->bvh->qbvh = bvh8i_quantized; 

#else
      BBox3fa root_bounds = bvh4i_builder8->bvh->bounds;
      DBG_PRINT( root_bounds );
      BVH8i::NodeHF16 *bvh8i_hf = (BVH8i::NodeHF16 *)os_malloc(sizeof(BVH8i::NodeHF16) * index8);
      for (size_t i=0;i<index8;i++) {
        bvh8i_hf[i].init( root_bounds , bvh8i_base[i] );
      }
      std::cout << "HF CONVERSION DONE" << std::endl << std::flush;
      bvh4i_builder8->bvh->qbvh = bvh8i_hf; 
#endif

#endif

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

    }
    
    
    
    // =======================================================================================================
    // =======================================================================================================
    // =======================================================================================================
    

     Builder* BVH8iTriangle8BuilderObjectSplit (void* bvh, BuildSource* source, Scene* scene, const size_t minLeafSize, const size_t maxLeafSize) {
       return new BVH8iBuilderTriangle8((BVH4i*)bvh,source,scene,minLeafSize,maxLeafSize);
     }
  }
};
  

