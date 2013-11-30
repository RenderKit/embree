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

#include "bvh4i_builder_util.h"
#include "bvh4i.h"

using namespace embree;

#define DBG(x)

#define TREE_BRANCHING_FACTOR 4

#define QBVH_MAX_STACK_DEPTH 64

namespace embree
{
  __forceinline static BBox3f getItemBoundsFromAccelerationData(const Triangle1 *__restrict__ const accel)
  {

#if !defined(__MIC__)
    const Vec3fa vtxA = (__m128)select(0x7,load4f_nt((float*)&accel->v0),zero);
    const Vec3fa vtxB = (__m128)select(0x7,load4f_nt((float*)&accel->v1),zero);
    const Vec3fa vtxC = (__m128)select(0x7,load4f_nt((float*)&accel->v2),zero);
#else
    const Vec3fa vtxA = *(Vec3fa*)&accel->v0;
    const Vec3fa vtxB = *(Vec3fa*)&accel->v1;
    const Vec3fa vtxC = *(Vec3fa*)&accel->v2;
#endif

    Vec3fa b_min = vtxA;
    Vec3fa b_max = vtxA;

    b_min = min(b_min,vtxB);
    b_min = min(b_min,vtxC);
    
    b_max = max(b_max,vtxB);
    b_max = max(b_max,vtxC);

    BBox3f aabb;
    aabb.lower = b_min;
    aabb.upper = b_max;
    return aabb;
  }


  size_t handleNonSplitableLeaves(BuildRecord &current, // FIXME: fails for very large leaves
				  BuildRecord *record)
  {
    size_t numSplits = 1;
    const size_t items  = current.items();

    for (size_t i=0;i<TREE_BRANCHING_FACTOR;i++)
      {
	record[i] = current;
	record[i].createLeaf();
      }

    record[0].end   = record[0].begin+4;
    record[1].begin = record[0].end;

    if (current.items() <= 8)
      {
	record[1].end   = record[1].begin + items - 4;
	numSplits = 2;
      }
    else if (current.items() <= 12)
      {
	record[1].end   = record[1].begin+4;
	record[2].begin = record[1].end;
	record[2].end   = record[2].begin + items - 8;
	numSplits = 3;
      }
    else if (current.items() <= 16)
      {
	record[1].end   = record[1].begin+4;
	record[2].begin = record[1].end;
	record[2].end   = record[2].begin+4;
	record[3].begin = record[2].end;
	record[3].end   = record[3].begin + items - 12;
	numSplits = 4;
      }
    else // current.items() > 16, should be very rare
      {
	record[1].end   = record[1].begin+4;
	record[2].begin = record[1].end;
	record[2].end   = record[2].begin+4;
	numSplits = 4;
	record[3].createNode();
	record[3].begin = record[2].begin+4;
      }

    return numSplits;  
  }

  //#include <pthread.h> 

#define DBG_THREADS(x) 



  // ================================================================================
  // ================================================================================
  // ================================================================================

  void checkBVH4iTree(const BVHNode *__restrict__ const node, 
		      const Triangle1 *__restrict__ const accel,
		      const size_t primitives,
		      const bool checkPrimBounds)
  {  
    struct {
      unsigned int node;
    } stackNode[QBVH_MAX_STACK_DEPTH*2];

    unsigned int nodes = 0;
    unsigned int nodeDistribution[16];
    for (unsigned int i=0;i<16;i++)
      nodeDistribution[i] = 0;

    unsigned int binaryNodes = 0;
    unsigned int triNodes = 0;
    unsigned int quadNodes = 0;
    unsigned int leaves = 0;
    unsigned int items = 0;
    unsigned int qnodes = 0;
    float trav = 0.0f;
    float isec = 0.0f;
  
    unsigned int equal_parent_child = 0;

    unsigned int sindex = 0;

    for (int i=0;i<1;i++)
      {
	stackNode[i].node = i;
	sindex++;
      }

    unsigned int itemDistribution[32];
    for (unsigned int i=0;i<32;i++)
      itemDistribution[i] = 0;

    DBG(DBG_PRINT(node[0]));
    //DBG_PRINT(primitives);
    unsigned int *primID = new unsigned int[primitives];
    for (unsigned int i=0;i<primitives;i++) primID[i] = 0;

    nodes += 4;

    while (1) {
      if (unlikely(sindex == 0)) break;
    
      sindex--;
      unsigned int index = stackNode[sindex].node;

      const BVHNode &entry = node[index];

      if (entry.isLeaf())
	{
	  trav += area(entry) * entry.items();

	  //DBG(std::cout << "LEAF: offset " << entry.itemListOfs() << " items:" << entry.items() << std::endl);
	  items += entry.items();
	  if(entry.items()<32)
	    itemDistribution[entry.items()]++;
	  else
	    std::cout << "LEAF " << index << " items(" << entry.items() << ") >= 32" << std::endl;

	  isec += area(entry) * entry.items();
	  leaves++;
	  const unsigned int offset = entry.itemListOfs();

	  if(entry.items() > 4) std::cout << "WARNING: " << entry.items() << " items in leaf" << std::endl;

	  BBox3f primitiveBounds;
	  primitiveBounds = empty;
	

	  for (unsigned int i=0;i<entry.items();i++)
	    {	    
	      //if (checkPrimBounds) primitiveBounds.extend(aabb[offset+i]);
	      if (checkPrimBounds) primitiveBounds.extend(getItemBoundsFromAccelerationData(&accel[offset+i]));
	      primID[offset+i]++;
	    }
#if 1
	  if (checkPrimBounds)
	    if(!subset(primitiveBounds,entry))
	      {	    
		DBG_PRINT(entry);
		DBG_PRINT(primitiveBounds);
		std::cout << "offset " << offset << " items " << entry.items() << std::endl;
		exit(0);
	      }
#endif
	}
      else
	{
	  trav += area(entry);

	  DBG(std::cout << "NODE: offset " << entry.firstChildID() << std::endl);

	  assert(sindex + entry.items() < QBVH_MAX_STACK_DEPTH);
	  const unsigned int children = entry.firstChildID();
	  unsigned int n = 0;

	  //DBG(DBG_PRINT(entry.items()));
	  nodes += entry.items(); 
	  qnodes++;

	  nodeDistribution[entry.items()]++;

	  if (entry.items() == 2) 
	    binaryNodes++;
	  else if (entry.items() == 3) 
	    triNodes++;
	  else if (entry.items() == 4) 
	    quadNodes++;
	  else if (entry.items() > TREE_BRANCHING_FACTOR)
	    {
	      DBG_PRINT(entry.items());
	      std::cout << index << " number of nodes " << entry.items() << std::endl; 
	    } 

	  unsigned int child_node_leaves = 0;
	  for (unsigned int i=0;i<entry.items();i++) 
	    {
	      n++;
	      const unsigned int childIndex = children + i;
	    
	      if (node[childIndex].isLeaf())
		child_node_leaves++;

	      if(!subset(node[childIndex],entry))
		{
		  std::cout << "Enclosing error" << std::endl;
		  std::cout << "parent " << index << " -> " << entry << std::endl;
		  std::cout << "child  " << childIndex << " -> " << node[childIndex] << std::endl;
		  exit(0);
		}

	      //if (equal(entry,node[childIndex])) equal_parent_child++;

	      stackNode[sindex+i].node = childIndex;
	    }      

	  sindex+=n;
	  //DBG(DBG_PRINT(sindex));
	}
    
    }

    for (unsigned int i=0;i<primitives;i++) 
      if (primID[i] == 0 || primID[i] > 1)
	{
	  std::cout << "primitive error at " << i << " " << primID[i] << std::endl;
	  exit(0);
	}

    int sum = 0;  
    for (unsigned int i=0;i<32;i++)
      sum += itemDistribution[i];

    std::cout << "item distribution: " << std::endl;

    int part = 0;
    for (unsigned int i=0;i<8;i++)
      {
	part += itemDistribution[i];
	std::cout << i << " [" << 100.0f * itemDistribution[i] / sum << ", " << 100.0f * part / sum << " p-sum " << part << "] ";
      }
    std::cout << std::endl;

    part = 0;
    for (unsigned int i=0;i<=4;i++)
      {
	part += i*itemDistribution[i];
      }

    std::cout << "leaf/item util        " << 100.0f * (float)part / (sum*4) << std::endl;
    std::cout << "equal_parent_child    " << equal_parent_child << std::endl;


    std::cout << "node distribution: " << std::endl;
    sum = 0;
    part = 0;

    for (unsigned int i=0;i<16;i++)
      sum += nodeDistribution[i];

    for (unsigned int i=0;i<=8;i++)
      {
	part += nodeDistribution[i];
	std::cout << i << " [" << 100.0f * nodeDistribution[i] / sum << ", " << 100.0f * part / sum << " p-sum " << part << "] ";
      }
    std::cout << std::endl;

    
    DBG_PRINT(area(node[0]));
    DBG_PRINT(isec);
    DBG_PRINT(trav);
    DBG_PRINT(isec / area(node[0]));
    DBG_PRINT(trav / area(node[0]));
    
    DBG_PRINT(nodes);
    DBG_PRINT(qnodes);

    std::cout << "BinaryNodes " << binaryNodes << " [" << binaryNodes*2 << "] " << 100.0f * binaryNodes / qnodes << " %" << std::endl;
    std::cout << "TriNodes    " << triNodes << " [" << triNodes*4 << "] " << 100.0f * triNodes / qnodes << " %" << std::endl;
    std::cout << "QuadNodes   " << quadNodes << " [" << quadNodes*4 << "] " << 100.0f * quadNodes / qnodes << " %" << std::endl;
    //std::cout << "BVH size " << (float)sizeof(BVHNode)*nodes / 1024.0f << " KB" << std::endl;
    std::cout << "QBVH size " << (float)sizeof(BVHNode)*4*qnodes / 1024.0f << " KB" << std::endl;

    const float node_util = (float)(2*binaryNodes+3*triNodes+4*quadNodes) / (binaryNodes+triNodes+quadNodes);
    std::cout << "Quad node utilization " << node_util <<  " " << 100.0f * node_util / 4 << std::endl;
  
    DBG_PRINT(leaves);
    DBG_PRINT(items);      
    DBG_PRINT(trav);
    DBG_PRINT(isec);

    delete [] primID;
  }

  // ================================================================================
  // ================================================================================
  // ================================================================================


};

