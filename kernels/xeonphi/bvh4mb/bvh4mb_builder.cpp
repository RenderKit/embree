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

#include "bvh4mb/bvh4mb_builder.h"

namespace embree
{
#define DBG(x) 
#define TIMER(x) 

#define L1_PREFETCH_ITEMS 2
#define L2_PREFETCH_ITEMS 16

#define GENERATE_SUBTREES_MAX_TREE_DEPTH 5
#define SERIAL_REFIT_THRESHOLD 1024

  Builder* BVH4mbBuilder::create (void* accel, BuildSource* source, void* geometry, size_t mode ) 
  { 
    Builder* builder = new BVH4mbBuilder((BVH4mb*)accel,source,geometry);
    return builder;
  }

  void BVH4mbBuilder::printBuilderName()
  {
    std::cout << "building BVH4mb with binned SAH builder (MIC) ... " << std::endl;    
  }

  void BVH4mbBuilder::allocateData(const size_t threadCount, const size_t totalNumPrimitives)
  {
    DBG(PING);
    size_t numPrimitivesOld = numPrimitives;
    numPrimitives = totalNumPrimitives;
    DBG(DBG_PRINT(numPrimitives));


    if (numPrimitivesOld != numPrimitives || numPrimitives == 0)
      {
	const size_t numPrims = numPrimitives+4;
	const size_t minAllocNodes = numPrims ? threadCount * ALLOCATOR_NODE_BLOCK_SIZE * 4: 16;
	const size_t numNodes = max((size_t)(numPrims * BVH_NODE_PREALLOC_FACTOR),minAllocNodes);
	allocateMemoryPools(numPrims,numNodes,sizeof(BVH4i::Node),sizeof(BVH4mb::Triangle01));
      }
  }

  __forceinline void computeAccelerationDataMB(const unsigned int &geomID,
					     const unsigned int &primID,     
					     const Scene *__restrict__ const scene,
					     BVH4mb::Triangle01 * __restrict__ const acc)
  {
    const TriangleMeshScene::TriangleMesh* __restrict__ const mesh = scene->getTriangleMesh(geomID);
    const TriangleMeshScene::TriangleMesh::Triangle & tri = mesh->triangle(primID);

    const mic_i pID(primID);
    const mic_i gID(geomID);

    const float *__restrict__ const vptr0_t0 = (float*)&mesh->vertex(tri.v[0]);
    const float *__restrict__ const vptr1_t0 = (float*)&mesh->vertex(tri.v[1]);
    const float *__restrict__ const vptr2_t0 = (float*)&mesh->vertex(tri.v[2]);

    prefetch<PFHINT_L1>(vptr1_t0);
    prefetch<PFHINT_L1>(vptr2_t0);

    const mic_f v0_t0 = broadcast4to16f(vptr0_t0); 
    const mic_f v1_t0 = broadcast4to16f(vptr1_t0);
    const mic_f v2_t0 = broadcast4to16f(vptr2_t0);

    const mic_f tri_accel_t0 = initTriangle1(v0_t0,v1_t0,v2_t0,gID,pID,mic_i(mesh->mask));

    store16f_ngo(&acc->t0,tri_accel_t0);

    if ((int)mesh->numTimeSteps == 1)
      {
	store16f_ngo(&acc->t1,tri_accel_t0);
      }
    else
      {
	assert( (int)mesh->numTimeSteps == 2 );
	const float *__restrict__ const vptr0_t1 = (float*)&mesh->vertex(tri.v[0],1);
	const float *__restrict__ const vptr1_t1 = (float*)&mesh->vertex(tri.v[1],1);
	const float *__restrict__ const vptr2_t1 = (float*)&mesh->vertex(tri.v[2],1);
	
	const mic_f v0_t1 = broadcast4to16f(vptr0_t1); 
	const mic_f v1_t1 = broadcast4to16f(vptr1_t1);
	const mic_f v2_t1 = broadcast4to16f(vptr2_t1);

	const mic_f tri_accel_t1 = initTriangle1(v0_t1,v1_t1,v2_t1,gID,pID,mic_i(mesh->mask));

	store16f_ngo(&acc->t1,tri_accel_t1);
      }
  }

  void BVH4mbBuilder::createTriangle01AccelMB(const size_t threadID, const size_t numThreads)
  {
    const size_t startID = (threadID+0)*numPrimitives/numThreads;
    const size_t endID   = (threadID+1)*numPrimitives/numThreads;

    BVH4mb::Triangle01 * __restrict__  acc  = (BVH4mb::Triangle01 *)accel + startID;
    const PrimRef* __restrict__  bptr = prims + startID;

    for (size_t j=startID; j<endID; j++, bptr++, acc++)
      {
	prefetch<PFHINT_NT>(bptr + L1_PREFETCH_ITEMS);
	prefetch<PFHINT_L2>(bptr + L2_PREFETCH_ITEMS);
	assert(bptr->geomID() < scene->size() );
	assert(bptr->primID() < scene->get( bptr->geomID() )->numPrimitives );

	computeAccelerationDataMB(bptr->geomID(),bptr->primID(),scene,acc);
      }
  }

  void BVH4mbBuilder::createAccel(const size_t threadIndex, const size_t threadCount)
  {
    DBG(PING);
    LockStepTaskScheduler::dispatchTask( task_createTriangle01AccelMB, this, threadIndex, threadCount );   
  }

  void BVH4mbBuilder::refit(const size_t index)
  {   
    BVHNode& entry = node[index];
    if (unlikely(entry.isLeaf()))
      {
	unsigned int accel_entries = entry.items();
	unsigned int accel_offset  = entry.itemListOfs();
	BBox3f leaf_bounds = empty;
	BVH4mb::Triangle01* accelMB = (BVH4mb::Triangle01*)accel + accel_offset;
	for (size_t i=0;i<accel_entries;i++)
	  {
	    leaf_bounds.extend( accelMB[i].t1.bounds() );
	  }

	*(BBox3f*)&node[index+4] = leaf_bounds;
	return;
      }

    const size_t childrenID = entry.firstChildID();
    const size_t items    = entry.items();
    BBox3f* next = (BBox3f*)&node[childrenID+4];
    
    /* init second node */
    const mic_f init_node = load16f((float*)BVH4i::initQBVHNode);
    store16f_ngo(next + 0,init_node);
    store16f_ngo(next + 2,init_node);

    BBox3f parentBounds = empty;
    for (size_t i=0; i<items; i++) 
    {
      const size_t childIndex = childrenID + i;	    	    
      refit(childIndex);
    }      


    for (size_t i=0; i<items; i++) 
      parentBounds.extend( next[i] );

    *(BBox3f*)&node[index+4] = parentBounds;    

  }    

  void BVH4mbBuilder::check_tree(const unsigned index)
  {
    BVHNode& entry = node[index];
    if (unlikely(entry.isLeaf()))
      {
	unsigned int accel_entries = entry.items();
	unsigned int accel_offset  = entry.itemListOfs();
	BBox3f leaf_bounds = empty;
	BVH4mb::Triangle01* accelMB = (BVH4mb::Triangle01*)accel + accel_offset;
	for (size_t i=0;i<accel_entries;i++)
	  leaf_bounds.extend( accelMB[i].t1.bounds() );
	if (leaf_bounds != *(BBox3f*)&node[index+4])
	  {
	    DBG_PRINT(leaf_bounds);
	    DBG_PRINT(*(BBox3f*)&node[index+4]);
	    DBG_PRINT(index);
	    FATAL("LEAF");
	  }
      }
    else
      {
	const size_t childrenID = entry.firstChildID();
	const size_t items    = entry.items();
	BBox3f* next = (BBox3f*)&node[childrenID+4];

	BBox3f bounds = empty;
	for (size_t i=0; i<items; i++) 
	  bounds.extend ( next[i]  );

	if (index != 0)
	  if ( bounds != *(BBox3f*)&node[index+4])
	    {
	      DBG_PRINT(bounds);
	      DBG_PRINT(*(BBox3f*)&node[index+4]);
	      DBG_PRINT(index);
	    FATAL("NODE");
	    }

	for (size_t i=0; i<items; i++) 
	  {
	    const size_t childIndex = childrenID + i;	    	    
	    check_tree(childIndex);
	  }
      }
    
  }

  BBox3f BVH4mbBuilder::refit_subtree(const size_t index)
  {
    BVHNode& entry = node[index];
    if (unlikely(entry.isLeaf()))
      {
	unsigned int accel_entries = entry.items();
	unsigned int accel_offset  = entry.itemListOfs();
	BBox3f leaf_bounds = empty;
	BVH4mb::Triangle01* accelMB = (BVH4mb::Triangle01*)accel + accel_offset;
	for (size_t i=0;i<accel_entries;i++)
	  leaf_bounds.extend( accelMB[i].t1.bounds() );
	return leaf_bounds;
      }

    const size_t childrenID = entry.firstChildID();
    const size_t items    = entry.items();
    BBox3f* next = (BBox3f*)&node[childrenID+4];

    BBox3f local[4];

    /* init second node */
    const mic_f init_node = load16f((float*)BVH4i::initQBVHNode);
    store16f(local + 0,init_node);
    store16f(local + 2,init_node);


    BBox3f bounds = empty;
    for (size_t i=0; i<items; i++) 
    {
      const size_t childIndex = childrenID + i;	    	    
      BBox3f childBounds = refit_subtree(childIndex);
      bounds.extend ( childBounds );
      local[i] = childBounds;
    }      

    store16f_ngo(next + 0,load16f(&local[0]));
    store16f_ngo(next + 2,load16f(&local[2]));
    
    return bounds;
    
  }




  void BVH4mbBuilder::generate_subtrees(const size_t index,const size_t depth, size_t &subtrees)
  {
    BVHNode& entry = node[index];

    if (depth == GENERATE_SUBTREES_MAX_TREE_DEPTH || entry.isLeaf())
      {
	unsigned int *subtrees_array = (unsigned int*)prims;
	subtrees_array[subtrees++] = index;
	return;
      }

    const size_t childrenID = entry.firstChildID();
    const size_t items      = entry.items();

    for (size_t i=0; i<items; i++) 
      {
	const size_t childIndex = childrenID + i;	    	    
	generate_subtrees(childIndex,depth+1,subtrees);
      }      
  }


  BBox3f BVH4mbBuilder::refit_toplevel(const size_t index,const size_t depth)
  {
    BVHNode& entry = node[index];

    if (depth == GENERATE_SUBTREES_MAX_TREE_DEPTH || entry.isLeaf())
      {
	return *(BBox3f*)&node[index+4];
      }

    const size_t childrenID = entry.firstChildID();
    const size_t items    = entry.items();
    BBox3f* next = (BBox3f*)&node[childrenID+4];

    __align(64) BBox3f local[4];

    /* init second node */
    const mic_f init_node = load16f((float*)BVH4i::initQBVHNode);
    store16f(local + 0,init_node);
    store16f(local + 2,init_node);


    BBox3f bounds = empty;
    for (size_t i=0; i<items; i++) 
    {
      const size_t childIndex = childrenID + i;	    	    
      BBox3f childBounds = refit_toplevel(childIndex,depth+1);
      local[i] = childBounds;
      bounds.extend ( childBounds );
    }      

    store16f_ngo(next + 0,load16f(&local[0]));
    store16f_ngo(next + 2,load16f(&local[2]));
    
    return bounds;
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

  void BVH4mbBuilder::convertToSOALayoutMB(const size_t threadID, const size_t numThreads)
  {
    const size_t startID = (threadID+0)*numNodes/numThreads;
    const size_t endID   = (threadID+1)*numNodes/numThreads;

    BVHNode  * __restrict__  bptr = ( BVHNode*)node + startID*4;

    BVH4i::Node * __restrict__  qptr = (BVH4i::Node*)node + startID;

    for (unsigned int n=startID;n<endID;n++,qptr++,bptr+=4) 
      {
	prefetch<PFHINT_L1EX>(bptr+4);
	prefetch<PFHINT_L2EX>(bptr+4*4);
	convertToBVH4Layout(bptr);
	evictL1(bptr);
      }
  }


  void BVH4mbBuilder::refitBVH4MB(const size_t threadID, const size_t numThreads)
  {
    const size_t startID = (threadID+0)*subtrees/numThreads;
    const size_t endID   = (threadID+1)*subtrees/numThreads;

    unsigned int *subtrees_array = (unsigned int*)prims;

    for (size_t i=startID;i<endID;i++)
      {
	const unsigned int index = subtrees_array[i];
	BBox3f bounds = refit_subtree(index);
	*(BBox3f*)&node[index+4] = bounds;		
      }
  }

  void BVH4mbBuilder::convertQBVHLayout(const size_t threadIndex, const size_t threadCount)
  {
    if (numPrimitives < SERIAL_REFIT_THRESHOLD)
      {
	refit(0);
      }
    else
      {
	subtrees = 0;
	generate_subtrees(0,0,subtrees);

	DBG(DBG_PRINT(subtrees));

	LockStepTaskScheduler::dispatchTask( task_refitBVH4MB, this, threadIndex, threadCount );    
    
	refit_toplevel(0,0);
      }

#if defined(DEBUG)
    check_tree(0);
#endif

    LockStepTaskScheduler::dispatchTask( task_convertToSOALayoutMB, this, threadIndex, threadCount );    
  }

}
