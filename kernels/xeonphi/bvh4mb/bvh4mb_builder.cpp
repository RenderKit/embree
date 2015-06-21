// ======================================================================== //
// Copyright 2009-2015 Intel Corporation                                    //
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

#define GENERATE_SUBTREES_MAX_TREE_DEPTH 6
#define SERIAL_REFIT_THRESHOLD 1024

  Builder* BVH4mbBuilder::create (void* accel, void* geometry, size_t mode ) 
  { 
    Builder* builder = new BVH4mbBuilder((BVH4mb*)accel,geometry);
    return builder;
  }

  void BVH4mbBuilder::printBuilderName() {
    std::cout << "building BVH4mb with binned SAH builder (MIC) ... " << std::endl;    
  }

  size_t BVH4mbBuilder::getNumPrimitives() {
    return scene->numTriangles2;
  }

  void BVH4mbBuilder::computePrimRefs(const size_t threadIndex, const size_t threadCount) {
    scene->lockstep_scheduler.dispatchTask( task_computePrimRefsTrianglesMB, this, threadIndex, threadCount );
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
      if (unlikely(scene->get(g) == nullptr)) continue;
      if (unlikely(scene->get(g)->type != Geometry::TRIANGLE_MESH)) continue;
      const TriangleMesh* __restrict__ const mesh = scene->getTriangleMesh(g);
      if (unlikely(!mesh->isEnabled())) continue;
      if (unlikely(mesh->numTimeSteps == 1)) continue;

      const size_t numTriangles = mesh->size();
      if (numSkipped + numTriangles > startID) break;
      numSkipped += numTriangles;
    }

    // === start with first group containing startID ===
    float16 bounds_scene_min((float)pos_inf);
    float16 bounds_scene_max((float)neg_inf);
    float16 bounds_centroid_min((float)pos_inf);
    float16 bounds_centroid_max((float)neg_inf);

    unsigned int num = 0;
    unsigned int currentID = startID;
    unsigned int offset = startID - numSkipped;

    __aligned(64) PrimRef local_prims[2];
    size_t numLocalPrims = 0;
    PrimRef *__restrict__ dest = &prims[currentID];

    for (; g<numGroups; g++) 
    {
      if (unlikely(scene->get(g) == nullptr)) continue;
      if (unlikely(scene->get(g)->type != Geometry::TRIANGLE_MESH)) continue;
      const TriangleMesh* __restrict__ const mesh = scene->getTriangleMesh(g);
      if (unlikely(!mesh->isEnabled())) continue;
      if (unlikely(mesh->numTimeSteps == 1)) continue;

      for (unsigned int i=offset; i<mesh->size() && currentID < endID; i++, currentID++)	 
      { 			    
	const TriangleMesh::Triangle& tri = mesh->triangle(i);
	prefetch<PFHINT_L2>(&tri + L2_PREFETCH_ITEMS);
	prefetch<PFHINT_L1>(&tri + L1_PREFETCH_ITEMS);

	const Vec3f16 v = mesh->getTriangleVertices<PFHINT_L2>(tri);

	float16 bmin = min(min(v[0],v[1]),v[2]);
	float16 bmax = max(max(v[0],v[1]),v[2]);

	bounds_scene_min = min(bounds_scene_min,bmin);
	bounds_scene_max = max(bounds_scene_max,bmax);
	const float16 centroid2 = bmin+bmax;
	bounds_centroid_min = min(bounds_centroid_min,centroid2);
	bounds_centroid_max = max(bounds_centroid_max,centroid2);

	store4f(&local_prims[numLocalPrims].lower,bmin);
	store4f(&local_prims[numLocalPrims].upper,bmax);	
	local_prims[numLocalPrims].lower.a = g;
	local_prims[numLocalPrims].upper.a = i;

	numLocalPrims++;
	if (unlikely(((size_t)dest % 64) != 0) && numLocalPrims == 1)
	  {
	    *dest = local_prims[0];
	    dest++;
	    numLocalPrims--;
	  }
	else
	  {
	    const float16 twoAABBs = load16f(local_prims);
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
    DBG(PRINT(numPrimitives));


    if (numPrimitivesOld != numPrimitives)
      {
	const size_t numPrims = numPrimitives+4;
	const size_t minAllocNodes = (threadCount+1) * 2 * ALLOCATOR_NODE_BLOCK_SIZE;
	const size_t numNodes = max((size_t)((numPrims+3)/4),minAllocNodes);
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

    const int16 pID(primID);
    const int16 gID(geomID);

    const Vec3f16 v_t0 = mesh->getTriangleVertices<PFHINT_L1>(tri);

    const float16 tri_accel_t0 = initTriangle1(v_t0[0],v_t0[1],v_t0[2],gID,pID,int16(mesh->mask));

    store16f_ngo(&acc->t0,tri_accel_t0);

    if ((int)mesh->numTimeSteps == 1)
      {
	store16f_ngo(&acc->t1,tri_accel_t0);
      }
    else
      {
	assert( (int)mesh->numTimeSteps == 2 );

	const Vec3f16 v_t1 = mesh->getTriangleVertices<PFHINT_L1>(tri,1);
	const float16 tri_accel_t1 = initTriangle1(v_t1[0],v_t1[1],v_t1[2],gID,pID,int16(mesh->mask));

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
    scene->lockstep_scheduler.dispatchTask( task_createTriangle01AccelMB, this, threadIndex, threadCount );   
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

    float16 node_lower_t1 = broadcast4to16f(&BVH4i::Node::initQBVHNode[0]);
    float16 node_upper_t1 = broadcast4to16f(&BVH4i::Node::initQBVHNode[1]);
    bool16 m_lane = 0xf;

    BVH4mb::Node *n = (BVH4mb::Node*)ref.node(node);
    BBox3fa parentBounds = empty;
    for (size_t i=0;i<4;i++)
      {
	if (n->child(i) == BVH4i::invalidNode) break;
	BBox3fa bounds = refit( n->child(i) );
	const float16 b_lower = broadcast4to16f(&bounds.lower);
	const float16 b_upper = broadcast4to16f(&bounds.upper);
	node_lower_t1 = select(m_lane,b_lower,node_lower_t1);
	node_upper_t1 = select(m_lane,b_upper,node_upper_t1);
	m_lane = (unsigned int)m_lane << 4;
	parentBounds.extend( bounds );
      }
    store16f_ngo((float16*)n->lower_t1,node_lower_t1);
    store16f_ngo((float16*)n->upper_t1,node_upper_t1);            

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
	    PRINT(bounds);
	    PRINT(n->bounds_t1(i));
	    PRINT(ref);
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

    float16 node_lower_t1 = broadcast4to16f(&BVH4i::Node::initQBVHNode[0]);
    float16 node_upper_t1 = broadcast4to16f(&BVH4i::Node::initQBVHNode[1]);
    bool16 m_lane = 0xf;

    BVH4mb::Node *n = (BVH4mb::Node*)ref.node(node);
    BBox3fa parentBounds = empty;
    for (size_t i=0;i<4;i++)
      {
	if (n->child(i) == BVH4i::invalidNode) break;
	BBox3fa bounds = refit_toplevel( n->child(i), depth + 1 );
	const float16 b_lower = broadcast4to16f(&bounds.lower);
	const float16 b_upper = broadcast4to16f(&bounds.upper);
	node_lower_t1 = select(m_lane,b_lower,node_lower_t1);
	node_upper_t1 = select(m_lane,b_upper,node_upper_t1);
	m_lane = (unsigned int)m_lane << 4;
	parentBounds.extend( bounds );
      }
    store16f_ngo((float16*)n->lower_t1,node_lower_t1);
    store16f_ngo((float16*)n->upper_t1,node_upper_t1);            

    return parentBounds;

  }

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

  std::string BVH4mbBuilder::getStatistics()
  {
    return BVH4iStatistics<BVH4mb::Node>(bvh).str();
  }

  void BVH4mbBuilder::finalize(const size_t threadIndex, const size_t threadCount)
  {
    TIMER(double msec = 0.0);


    if (numPrimitives < SERIAL_REFIT_THRESHOLD)
      {
	refit(bvh->root);
      }
    else
      {
	TIMER(msec = getSeconds());
	// ------------------------
	atomicID.reset(0);
	subtrees = 0;
	generate_subtrees(bvh->root,0,subtrees);
	// ------------------------
	TIMER(msec = getSeconds()-msec);    
	TIMER(std::cout << "generate subtrees " << 1000. * msec << " ms" << std::endl << std::flush);

	DBG(PRINT(subtrees));

	TIMER(msec = getSeconds());
	// ------------------------
	scene->lockstep_scheduler.dispatchTask( task_refitBVH4MB, this, threadIndex, threadCount );    
	// ------------------------
	TIMER(msec = getSeconds()-msec);    
	TIMER(std::cout << "refit subtrees " << 1000. * msec << " ms" << std::endl << std::flush);

	TIMER(msec = getSeconds());
	// ------------------------    
	refit_toplevel(bvh->root,0);
	// ------------------------
	TIMER(msec = getSeconds()-msec);    
	TIMER(std::cout << "refit toplevel " << 1000. * msec << " ms" << std::endl << std::flush);

      }

    DBG(
	std::cout << "checking tree..." << std::flush;
	check_tree(bvh->root);
	std::cout << "done" << std::endl << std::flush;
	);


    TIMER(msec = getSeconds());

    //scene->lockstep_scheduler.dispatchTask( task_convertToSOALayoutMB, this, threadIndex, threadCount );    

    TIMER(msec = getSeconds()-msec);    
    TIMER(std::cout << "convert " << 1000. * msec << " ms" << std::endl << std::flush);

  }

}
