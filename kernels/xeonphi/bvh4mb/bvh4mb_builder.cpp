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

#include "bvh4mb/bvh4mb_builder.h"

namespace embree
{
#define DBG(x) 
#define TIMER(x) x

#define L1_PREFETCH_ITEMS 2
#define L2_PREFETCH_ITEMS 16

#define GENERATE_SUBTREES_MAX_TREE_DEPTH 6
#define SERIAL_REFIT_THRESHOLD 1024

  Builder* BVH4mbBuilder::create (void* accel, void* geometry, size_t mode ) 
  { 
    Builder* builder = new BVH4mbBuilder((BVH4mb*)accel,geometry);
    return builder;
  }

  void BVH4mbBuilder::printBuilderName()
  {
    std::cout << "building BVH4mb with binned SAH builder (MIC) ... " << std::endl;    
  }


  size_t BVH4mbBuilder::getNumPrimitives()
  {
    if (scene->numTriangleMeshes2 == 0) return 0;

    /* count total number of triangles */
    size_t primitives = 0;       
    for (size_t i=0;i<scene->size();i++)
      {
	if (unlikely(scene->get(i) == NULL)) continue;
	if (unlikely((scene->get(i)->type != TRIANGLE_MESH))) continue;
	if (unlikely(!scene->get(i)->isEnabled())) continue;

	const TriangleMesh* __restrict__ const mesh = scene->getTriangleMesh(i);
	if (unlikely(mesh->numTimeSteps == 1)) continue;

	primitives += mesh->numTriangles;
      }
    return primitives;	  
  }

  void BVH4mbBuilder::computePrimRefs(const size_t threadIndex, const size_t threadCount)
  {
    LockStepTaskScheduler::dispatchTask( task_computePrimRefsTrianglesMB, this, threadIndex, threadCount );
  }


  void BVH4mbBuilder::computePrimRefsTrianglesMB(const size_t threadID, const size_t numThreads) 
  {
    DBG(PING);
    const size_t numGroups = scene->size();
    const size_t startID = (threadID+0)*numPrimitives/numThreads;
    const size_t endID   = (threadID+1)*numPrimitives/numThreads;
    
    PrimRef *__restrict__ const prims     = this->prims;

    // === find first group containing startID ===
    unsigned int g=0, numSkipped = 0;
    for (; g<numGroups; g++) {       
      if (unlikely(scene->get(g) == NULL)) continue;
      if (unlikely(scene->get(g)->type != TRIANGLE_MESH)) continue;
      const TriangleMesh* __restrict__ const mesh = scene->getTriangleMesh(g);
      if (unlikely(!mesh->isEnabled())) continue;
      if (unlikely(mesh->numTimeSteps == 1)) continue;

      const size_t numTriangles = mesh->numTriangles;
      if (numSkipped + numTriangles > startID) break;
      numSkipped += numTriangles;
    }

    // === start with first group containing startID ===
    mic_f bounds_scene_min((float)pos_inf);
    mic_f bounds_scene_max((float)neg_inf);
    mic_f bounds_centroid_min((float)pos_inf);
    mic_f bounds_centroid_max((float)neg_inf);

    unsigned int num = 0;
    unsigned int currentID = startID;
    unsigned int offset = startID - numSkipped;

    __aligned(64) PrimRef local_prims[2];
    size_t numLocalPrims = 0;
    PrimRef *__restrict__ dest = &prims[currentID];

    for (; g<numGroups; g++) 
    {
      if (unlikely(scene->get(g) == NULL)) continue;
      if (unlikely(scene->get(g)->type != TRIANGLE_MESH)) continue;
      const TriangleMesh* __restrict__ const mesh = scene->getTriangleMesh(g);
      if (unlikely(!mesh->isEnabled())) continue;
      if (unlikely(mesh->numTimeSteps == 1)) continue;

      for (unsigned int i=offset; i<mesh->numTriangles && currentID < endID; i++, currentID++)	 
      { 			    
	//DBG_PRINT(currentID);
	const TriangleMesh::Triangle& tri = mesh->triangle(i);
	prefetch<PFHINT_L2>(&tri + L2_PREFETCH_ITEMS);
	prefetch<PFHINT_L1>(&tri + L1_PREFETCH_ITEMS);

	const mic3f v = mesh->getTriangleVertices<PFHINT_L2>(tri);

	mic_f bmin = min(min(v[0],v[1]),v[2]);
	mic_f bmax = max(max(v[0],v[1]),v[2]);

	bounds_scene_min = min(bounds_scene_min,bmin);
	bounds_scene_max = max(bounds_scene_max,bmax);
	const mic_f centroid2 = bmin+bmax;
	bounds_centroid_min = min(bounds_centroid_min,centroid2);
	bounds_centroid_max = max(bounds_centroid_max,centroid2);

	store4f(&local_prims[numLocalPrims].lower,bmin);
	store4f(&local_prims[numLocalPrims].upper,bmax);	
	local_prims[numLocalPrims].lower.a = g;
	local_prims[numLocalPrims].upper.a = i;

	//DBG_PRINT( local_prims[numLocalPrims] );

	numLocalPrims++;
	if (unlikely(((size_t)dest % 64) != 0) && numLocalPrims == 1)
	  {
	    *dest = local_prims[0];
	    dest++;
	    numLocalPrims--;
	  }
	else
	  {
	    const mic_f twoAABBs = load16f(local_prims);
	    if (numLocalPrims == 2)
	      {
		numLocalPrims = 0;
		store16f_ngo(dest,twoAABBs);
		dest+=2;
	      }
	  }	
      }
      if (currentID == endID) break;
      offset = 0;
    }

    /* is there anything left in the local queue? */
    if (numLocalPrims % 2 != 0)
      *dest = local_prims[0];

    /* update global bounds */
    Centroid_Scene_AABB bounds;
    
    store4f(&bounds.centroid2.lower,bounds_centroid_min);
    store4f(&bounds.centroid2.upper,bounds_centroid_max);
    store4f(&bounds.geometry.lower,bounds_scene_min);
    store4f(&bounds.geometry.upper,bounds_scene_max);

    global_bounds.extend_atomic(bounds);    
  }


  void BVH4mbBuilder::allocateData(const size_t threadCount, const size_t totalNumPrimitives)
  {
    DBG(PING);
    size_t numPrimitivesOld = numPrimitives;
    numPrimitives = totalNumPrimitives;
    DBG(DBG_PRINT(numPrimitives));


    if (numPrimitivesOld != numPrimitives)
      {
	const size_t numPrims = numPrimitives+4;
	const size_t minAllocNodes = numPrims ? threadCount * ALLOCATOR_NODE_BLOCK_SIZE * 4: 16;
	const size_t numNodes = max((size_t)(numPrims * BVH_NODE_PREALLOC_FACTOR),minAllocNodes);
	allocateMemoryPools(numPrims,numNodes,sizeof(BVH4mb::Node),sizeof(BVH4mb::Triangle01));
      }
  }

  __forceinline void computeAccelerationDataMB(const unsigned int &geomID,
					     const unsigned int &primID,     
					     const Scene *__restrict__ const scene,
					     BVH4mb::Triangle01 * __restrict__ const acc)
  {
    const TriangleMesh* __restrict__ const mesh = scene->getTriangleMesh(geomID);
    const TriangleMesh::Triangle & tri = mesh->triangle(primID);

    const mic_i pID(primID);
    const mic_i gID(geomID);

    const mic3f v_t0 = mesh->getTriangleVertices<PFHINT_L1>(tri);

    const mic_f tri_accel_t0 = initTriangle1(v_t0[0],v_t0[1],v_t0[2],gID,pID,mic_i(mesh->mask));

    store16f_ngo(&acc->t0,tri_accel_t0);

    if ((int)mesh->numTimeSteps == 1)
      {
	store16f_ngo(&acc->t1,tri_accel_t0);
      }
    else
      {
	assert( (int)mesh->numTimeSteps == 2 );

	const mic3f v_t1 = mesh->getTriangleVertices<PFHINT_L1>(tri,1);
	const mic_f tri_accel_t1 = initTriangle1(v_t1[0],v_t1[1],v_t1[2],gID,pID,mic_i(mesh->mask));

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

  
  BBox3fa BVH4mbBuilder::refit(const BVH4i::NodeRef &ref)
  {   
    if (unlikely(ref.isLeaf()))
      {
	unsigned int items = ref.items();
	BBox3fa leaf_bounds = empty;
	BVH4mb::Triangle01* accelMB = (BVH4mb::Triangle01*)ref.leaf<8>(accel);

	for (size_t i=0;i<items;i++)
	  prefetch<PFHINT_L1>(&accelMB[i].t1);

	for (size_t i=0;i<items;i++)
	  leaf_bounds.extend( accelMB[i].t1.bounds() );

	return leaf_bounds;
      }

    mic_f node_lower_t1 = broadcast4to16f(&BVH4i::initQBVHNode[0]);
    mic_f node_upper_t1 = broadcast4to16f(&BVH4i::initQBVHNode[1]);
    mic_m m_lane = 0xf;

    BVH4mb::Node *n = (BVH4mb::Node*)ref.node(node);
    BBox3fa parentBounds = empty;
    for (size_t i=0;i<4;i++)
      {
	if (n->child(i) == BVH4i::invalidNode) break;
	BBox3fa bounds = refit( n->child(i) );
	const mic_f b_lower = broadcast4to16f(&bounds.lower);
	const mic_f b_upper = broadcast4to16f(&bounds.upper);
	node_lower_t1 = select(m_lane,b_lower,node_lower_t1);
	node_upper_t1 = select(m_lane,b_upper,node_upper_t1);
	m_lane = (unsigned int)m_lane << 4;
	parentBounds.extend( bounds );
      }
    store16f_ngo((mic_f*)n->lower_t1,node_lower_t1);
    store16f_ngo((mic_f*)n->upper_t1,node_upper_t1);            

    return parentBounds;
  }    

  BBox3fa BVH4mbBuilder::check_tree(const BVH4i::NodeRef &ref)
  {

    if (unlikely(ref.isLeaf()))
      {
	unsigned int items = ref.items();
	BBox3fa leaf_bounds = empty;
	BVH4mb::Triangle01* accelMB = (BVH4mb::Triangle01*)ref.leaf<8>(accel);
	for (size_t i=0;i<items;i++)
	  {
	    leaf_bounds.extend( accelMB[i].t1.bounds() );
	  }
	return leaf_bounds;
      }

    BVH4mb::Node *n = (BVH4mb::Node*)ref.node(node);
    BBox3fa parentBounds = empty;
    for (size_t i=0;i<4;i++)
      {
	if (n->child(i) == BVH4i::invalidNode) break;
	BBox3fa bounds = check_tree( n->child(i) );
	
	if (bounds != n->bounds_t1(i))
	  {
	    DBG_PRINT(bounds);
	    DBG_PRINT(n->bounds_t1(i));
	    DBG_PRINT(ref);
	    FATAL("bounds don't match");
	  }
	parentBounds.extend( n->bounds_t1(i) );
      }
    return parentBounds;
  }


  void BVH4mbBuilder::generate_subtrees(const BVH4i::NodeRef &ref,const size_t depth, size_t &subtrees)
  {
    if (depth == GENERATE_SUBTREES_MAX_TREE_DEPTH || ref.isLeaf())
      {
	BVH4i::NodeRef *subtrees_array = (BVH4i::NodeRef *)prims;
	subtrees_array[subtrees++] = ref;
	return;
      }

    BVH4mb::Node *n = (BVH4mb::Node*)ref.node(node);

    for (size_t i=0;i<4;i++)
      {
	if (n->child(i) == BVH4i::invalidNode) break;
	generate_subtrees(n->child(i),depth+1,subtrees);
      }
  }


  BBox3fa BVH4mbBuilder::refit_toplevel(const BVH4i::NodeRef &ref,const size_t depth)
  {
    if (unlikely(ref.isLeaf()))
      {
	unsigned int items = ref.items();
	BBox3fa leaf_bounds = empty;
	BVH4mb::Triangle01* accelMB = (BVH4mb::Triangle01*)ref.leaf<8>(accel);

	for (size_t i=0;i<items;i++)
	  prefetch<PFHINT_L1>(&accelMB[i].t1);

	for (size_t i=0;i<items;i++)
	  leaf_bounds.extend( accelMB[i].t1.bounds() );

	return leaf_bounds;
      }

    if (depth == GENERATE_SUBTREES_MAX_TREE_DEPTH)
      {
	BVH4mb::Node *n = (BVH4mb::Node*)ref.node(node);
	BBox3fa parentBounds = empty;
	for (size_t i=0;i<4;i++)
	  {
	    if (n->child(i) == BVH4i::invalidNode) break;
	    parentBounds.extend( n->bounds_t1(i) );
	  }
	return parentBounds;
      }

    mic_f node_lower_t1 = broadcast4to16f(&BVH4i::initQBVHNode[0]);
    mic_f node_upper_t1 = broadcast4to16f(&BVH4i::initQBVHNode[1]);
    mic_m m_lane = 0xf;

    BVH4mb::Node *n = (BVH4mb::Node*)ref.node(node);
    BBox3fa parentBounds = empty;
    for (size_t i=0;i<4;i++)
      {
	if (n->child(i) == BVH4i::invalidNode) break;
	BBox3fa bounds = refit_toplevel( n->child(i), depth + 1 );
	const mic_f b_lower = broadcast4to16f(&bounds.lower);
	const mic_f b_upper = broadcast4to16f(&bounds.upper);
	node_lower_t1 = select(m_lane,b_lower,node_lower_t1);
	node_upper_t1 = select(m_lane,b_upper,node_upper_t1);
	m_lane = (unsigned int)m_lane << 4;
	parentBounds.extend( bounds );
      }
    store16f_ngo((mic_f*)n->lower_t1,node_lower_t1);
    store16f_ngo((mic_f*)n->upper_t1,node_upper_t1);            

    return parentBounds;

  }

  // __forceinline void convertToBVH4MBLayout(BVHNode *__restrict__ const bptr)
  // {
  //   const mic_i box01 = load16i((int*)(bptr + 0));
  //   const mic_i box23 = load16i((int*)(bptr + 2));

  //   const mic_i box_min01 = permute<2,0,2,0>(box01);
  //   const mic_i box_max01 = permute<3,1,3,1>(box01);

  //   const mic_i box_min23 = permute<2,0,2,0>(box23);
  //   const mic_i box_max23 = permute<3,1,3,1>(box23);
  //   const mic_i box_min0123 = select(0x00ff,box_min01,box_min23);
  //   const mic_i box_max0123 = select(0x00ff,box_max01,box_max23);

  //   const mic_m min_d_mask = bvhLeaf(box_min0123) != mic_i::zero();
  //   const mic_i childID    = bvhChildID(box_min0123);
  //   const mic_i min_d_node = qbvhCreateNode(childID,mic_i::zero());
  //   const mic_i min_d_leaf = ((box_min0123 ^ BVH_LEAF_MASK)<<0) | QBVH_LEAF_MASK; // * 2 as accel size is 128 bytes now
  //   const mic_i min_d      = select(min_d_mask,min_d_leaf,min_d_node);
  //   const mic_i bvh4_min   = select(0x7777,box_min0123,min_d);
  //   const mic_i bvh4_max   = box_max0123;
  //   store16i_nt((int*)(bptr + 0),bvh4_min);
  //   store16i_nt((int*)(bptr + 2),bvh4_max);
  // }

  // void BVH4mbBuilder::convertToSOALayoutMB(const size_t threadID, const size_t numThreads)
  // {
  //   const size_t startID = (threadID+0)*numNodes/numThreads;
  //   const size_t endID   = (threadID+1)*numNodes/numThreads;

  //   BVHNode  * __restrict__  bptr = ( BVHNode*)node + startID*4;

  //   BVH4i::Node * __restrict__  qptr = (BVH4i::Node*)node + startID;

  //   for (unsigned int n=startID;n<endID;n++,qptr++,bptr+=4) 
  //     {
  // 	prefetch<PFHINT_L1EX>(bptr+4);
  // 	prefetch<PFHINT_L2EX>(bptr+4*4);
  // 	convertToBVH4MBLayout(bptr);
  // 	evictL1(bptr);
  //     }
  // }


  void BVH4mbBuilder::refitBVH4MB(const size_t threadID, const size_t numThreads)
  {
    const size_t startID = (threadID+0)*subtrees/numThreads;
    const size_t endID   = (threadID+1)*subtrees/numThreads;

    BVH4i::NodeRef *subtrees_array = (BVH4i::NodeRef *)prims;

    while(1)
      {
	unsigned int ID = atomicID.inc();
	if (ID >= subtrees) break;
	const BVH4i::NodeRef &ref = subtrees_array[ID];
	BBox3fa bounds = refit(ref);
      }
  }

  void BVH4mbBuilder::convertQBVHLayout(const size_t threadIndex, const size_t threadCount)
  {
    TIMER(double msec = 0.0);


    if (numPrimitives < SERIAL_REFIT_THRESHOLD)
      {
	std::cout << "WORKAROUND" << std::endl;
	refit(0);
      }
    else
      {
	TIMER(msec = getSeconds());
	// ------------------------
	atomicID.reset(0);
	subtrees = 0;
	generate_subtrees(0,0,subtrees);
	// ------------------------
	TIMER(msec = getSeconds()-msec);    
	TIMER(std::cout << "generate subtrees " << 1000. * msec << " ms" << std::endl << std::flush);

	DBG(DBG_PRINT(subtrees));

	TIMER(msec = getSeconds());
	// ------------------------
	LockStepTaskScheduler::dispatchTask( task_refitBVH4MB, this, threadIndex, threadCount );    
	// ------------------------
	TIMER(msec = getSeconds()-msec);    
	TIMER(std::cout << "refit subtrees " << 1000. * msec << " ms" << std::endl << std::flush);

	TIMER(msec = getSeconds());
	// ------------------------    
	refit_toplevel(0,0);
	// ------------------------
	TIMER(msec = getSeconds()-msec);    
	TIMER(std::cout << "refit toplevel " << 1000. * msec << " ms" << std::endl << std::flush);

      }

#if 1 // defined(DEBUG)
    std::cout << "checking tree..." << std::flush;
    check_tree(bvh->root);
    std::cout << "done" << std::endl << std::flush;
#endif


    TIMER(msec = getSeconds());

    //LockStepTaskScheduler::dispatchTask( task_convertToSOALayoutMB, this, threadIndex, threadCount );    

    TIMER(msec = getSeconds()-msec);    
    TIMER(std::cout << "convert " << 1000. * msec << " ms" << std::endl << std::flush);

  }

}
