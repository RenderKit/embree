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

#include "bvh4hair/bvh4hair_builder.h"
#include "../geometry/bezier1i.h"
#include "../bvh4hair/bvh4hair_statistics.h"
#include "../../algorithms/parallel_partition.h"

namespace embree
{
#define DBG(x) 

#define L1_PREFETCH_ITEMS 4
#define L2_PREFETCH_ITEMS 16
#define SINGLE_THREADED_BUILD_THRESHOLD        512
#define THRESHOLD_FOR_SUBTREE_RECURSION         64
#define BUILD_RECORD_PARALLEL_SPLIT_THRESHOLD 1024

#define ENABLE_OBB_BVH4 1
#define ENABLE_AABB_NODES 1

#define BVH4HAIR_NODE_PREALLOC_FACTOR  0.8f

#define AABB_OBB_SWITCH_THRESHOLD      1.1f
#define THRESHOLD_SWITCH

#define TIMER(x)  

  static double dt = 0.0f;

  // ==========================================================================================
  // ==========================================================================================
  // ==========================================================================================

  enum {
    BVH4HAIR_AABB_NODE = 1,
    BVH4HAIR_OBB_NODE  = 2
  };

  void BVH4HairBuilder::printBuilderName()
  {
    std::cout << "building BVH4Hair using binned SAH builder (MIC) ... " << std::endl;    
    
  }

  size_t BVH4HairBuilder::getNumPrimitives()
  {
    DBG(PING);

    /* count total number of virtual objects */
    size_t numCurves = 0;       
    for (size_t i=0;i<scene->size();i++)
      {
	if (unlikely(scene->get(i) == nullptr)) continue;
	if (unlikely((scene->get(i)->type != Geometry::BEZIER_CURVES))) continue;
	if (unlikely(!scene->get(i)->isEnabled())) continue;
        BezierCurves* geom = (BezierCurves*) scene->getBezierCurves(i);
	numCurves += geom->size();
      }
    return numCurves;	
  }

  void BVH4HairBuilder::allocateData(const size_t threadCount, const size_t totalNumPrimitives)
  {
    DBG(PING);
    size_t numPrimitivesOld = numPrimitives;
    numPrimitives = totalNumPrimitives;

    if (numPrimitivesOld != numPrimitives)
      {
	const size_t numPrims = numPrimitives;
	const size_t minAllocNodes = (threadCount+1) * ALLOCATOR_NODE_BLOCK_SIZE;
	size_t numNodes = (size_t)((numPrims+1)/2 * BVH4HAIR_NODE_PREALLOC_FACTOR) + minAllocNodes;
	if (numPrimitives == 0) numNodes = 0;
	allocateMemoryPools(numPrims,numNodes);
      }
  }

  void BVH4HairBuilder::computePrimRefs(const size_t threadIndex, const size_t threadCount)
  {
    DBG(PING);
    scene->lockstep_scheduler.dispatchTask( task_computePrimRefsBezierCurves, this, threadIndex, threadCount );	
  }

  void BVH4HairBuilder::computePrimRefsBezierCurves(const size_t threadID, const size_t numThreads) 
  {
    DBG(PING);

    const size_t numTotalGroups = scene->size();

    /* count total number of virtual objects */
    const size_t numBezierCurves = numPrimitives;
    const size_t startID   = (threadID+0)*numBezierCurves/numThreads;
    const size_t endID     = (threadID+1)*numBezierCurves/numThreads; 
        
    Bezier1i *__restrict__ const bptr     = (Bezier1i*)this->prims;

    // === find first group containing startID ===
    unsigned int g=0, numSkipped = 0;
    for (; g<numTotalGroups; g++) {       
      if (unlikely(scene->get(g) == nullptr)) continue;
      if (unlikely((scene->get(g)->type != Geometry::BEZIER_CURVES))) continue;
      if (unlikely(!scene->get(g)->isEnabled())) continue;
      BezierCurves* geom = (BezierCurves*) scene->getBezierCurves(g);
      const size_t numPrims = geom->size();
      if (numSkipped + numPrims > startID) break;
      numSkipped += numPrims;
    }

    CentroidGeometryAABB bounds_cs;
    bounds_cs.reset();

    /* start with first group containing startID */

    unsigned int num = 0;
    unsigned int currentID = startID;
    unsigned int offset = startID - numSkipped;

    for (; g<numTotalGroups; g++) 
      {
	if (unlikely(scene->get(g) == nullptr)) continue;
	if (unlikely((scene->get(g)->type != Geometry::BEZIER_CURVES))) continue;
	if (unlikely(!scene->get(g)->isEnabled())) continue;

	BezierCurves* geom = (BezierCurves*) scene->getBezierCurves(g);

        size_t N = geom->size();
        for (unsigned int i=offset; i<N && currentID < endID; i++, currentID++)	 
        { 			    
	  const Vec2f16 b2 = geom->bounds_Vec2f16(i);
	  const float16 bmin = b2.x;
	  const float16 bmax = b2.y;
	  bounds_cs.extend(bmin,bmax);	
          
	  bptr[currentID].p = geom->fristVertexPtr(i); // FIXME: this does not support strides!!
          bptr[currentID].geomID = g;
          bptr[currentID].primID = i;
        }
        if (currentID == endID) break;
        offset = 0;
      }

    /* update global bounds */
    Centroid_Scene_AABB bounds (bounds_cs);
    
    global_bounds.extend_atomic(bounds);    
  }


  void BVH4HairBuilder::allocateMemoryPools(const size_t numPrims,
					    const size_t numNodes)
  {
    const size_t additional_size = 16 * CACHELINE_SIZE;
    const size_t sizeNodeInBytes    = sizeof(BVH4Hair::UnalignedNode);
    const size_t sizePrimRefInBytes = sizeof(Bezier1i);
    const size_t sizeAccelInBytes   = sizeof(Bezier1i);

    /* free previously allocated memory */

    if (prims)  {
      assert(size_prims > 0);
      os_free(prims,size_prims);
    }
    if (node  ) {
      assert(bvh4hair->size_node > 0);
      os_free(node ,bvh4hair->size_node);
    }
    if (accel ) {
      assert(bvh4hair->size_accel > 0);
      os_free(accel,bvh4hair->size_accel);
    }
      
    // === allocated memory for primrefs,nodes, and accel ===
    const size_t size_primrefs = numPrims * sizePrimRefInBytes + additional_size;
    const size_t size_node     = (double)(numNodes * sizeNodeInBytes + additional_size) * scene->device->memory_preallocation_factor;
    const size_t size_accel    = numPrims * sizeAccelInBytes   + additional_size;

    numAllocated64BytesBlocks = size_node / sizeof(BVH4Hair::UnalignedNode); // FIXME: do memory handling in 64 byte blocks

    DBG(
	PRINT(numAllocated64BytesBlocks);
	PRINT(sizeNodeInBytes);
	PRINT(sizePrimRefInBytes);
	PRINT(sizeAccelInBytes);
	
	PRINT(size_primrefs);
	PRINT(size_node);
	PRINT(size_accel);
	);

    prims = (Bezier1i                 *) os_malloc(size_primrefs); 
    node  = (BVH4Hair::UnalignedNode  *) os_malloc(size_node);
    accel = (Bezier1i                 *) os_malloc(size_accel);

    assert(prims  != 0);
    assert(node   != 0);
    assert(accel  != 0);

    memset((char*)accel + numPrims * sizeAccelInBytes,0,additional_size); // clear out as a 4-wide access is possible


    bvh4hair->accel = accel;
    bvh4hair->size_node  = size_node;
    bvh4hair->size_accel = size_accel;

    size_prims = size_primrefs;    
  }


  void BVH4HairBuilder::parallelBinningGlobal(const size_t threadID, const size_t numThreads)
  {
    BuildRecord &current = global_sharedData.rec;

    const unsigned int items = current.items();
    const unsigned int startID = current.begin + ((threadID+0)*items/numThreads);
    const unsigned int endID   = current.begin + ((threadID+1)*items/numThreads);

    const BinMapping mapping(current.bounds);  

    Bezier1i  *__restrict__ const tmp_prims = (Bezier1i*)accel;

    fastbin<Bezier1i>(prims,startID,endID,mapping,global_bin16[threadID]);    

    scene->lockstep_scheduler.syncThreadsWithReduction( threadID, numThreads, reduceBinsParallel, global_bin16 );
    
    if (threadID == 0)
      {
	global_sharedData.split = global_bin16[0].bestSplit(current,mapping.getValidDimMask());
      }
  }


  void BVH4HairBuilder::parallelBinningLocal(const size_t localThreadID,const size_t globalThreadID)
  {
    const size_t globalCoreID = globalThreadID/4;
    BuildRecord &current = local_sharedData[globalCoreID].rec;

    const unsigned int items   = current.items();
    const unsigned int startID = current.begin + ((localThreadID+0)*items/4);
    const unsigned int endID   = current.begin + ((localThreadID+1)*items/4);
    
    const BinMapping mapping(current.bounds);

    fastbin<Bezier1i>(prims,startID,endID,mapping,global_bin16[globalThreadID]);    

    localTaskScheduler[globalCoreID].syncThreads(localThreadID);

    if (localThreadID == 0)
      {
	Bin16 &bin16 = global_bin16[globalThreadID];

	for (size_t i=1;i<4;i++)
	  bin16.merge(global_bin16[globalThreadID+i]);

	local_sharedData[globalCoreID].split = bin16.bestSplit(current,mapping.getValidDimMask());
      }

  }


  void BVH4HairBuilder::build(const size_t threadIndex, const size_t threadCount) 
  {
    DBG(PING);
    if (threadIndex != 0) {
      FATAL("threadIndex != 0");
    }

    const size_t totalNumPrimitives = getNumPrimitives();


    DBG(PRINT(totalNumPrimitives));

    /* print builder name */
    if (unlikely(scene->device->verbosity(2))) 
      printBuilderName();

    if (likely(totalNumPrimitives == 0))
      {
	DBG(std::cout << "EMPTY SCENE BUILD" << std::endl);
	bvh4hair->root = BVH4Hair::invalidNode;
	bvh4hair->bounds = empty;
	bvh4hair->accel = nullptr;
	return;
      }


    /* allocate BVH data */
    allocateData(threadCount,totalNumPrimitives);

    if (likely(numPrimitives > SINGLE_THREADED_BUILD_THRESHOLD && threadCount > 1) )
      {
	DBG(std::cout << "PARALLEL BUILD" << std::endl);
	build_main(threadIndex,threadCount);

      }
    else
      {
	/* number of primitives is small, just use single threaded mode */
	assert( numPrimitives > 0 );
	DBG(std::cout << "SERIAL BUILD" << std::endl);
	build_main(0,1);
      }

    if (scene->device->verbosity(2)) {
      double perf = totalNumPrimitives/dt*1E-6;
      std::cout << "[DONE] " << 1000.0f*dt << "ms (" << perf << " Mtris/s), primitives " << numPrimitives << std::endl;
      std::cout << BVH4HairStatistics<BVH4Hair::UnalignedNode>(bvh4hair).str();
    }

  }




  void BVH4HairBuilder::build_main(size_t threadIndex, size_t threadCount) 
  {
    DBG(PING);

    TIMER(double msec = 0.0);

    /* start measurement */
    double t0 = 0.0f;
#if !defined(PROFILE)
    if (scene->device->verbosity(2)) 
#endif
      t0 = getSeconds();

    TIMER(msec = getSeconds());
    
    /* calculate list of primrefs */
    global_bounds.reset();

    computePrimRefs(threadIndex,threadCount);

    TIMER(msec = getSeconds()-msec);    
    TIMER(std::cout << "task_computePrimRefs " << 1000. * msec << " ms" << std::endl << std::flush);
    TIMER(msec = getSeconds());

    /* allocate and initialize root node */
    atomicID.reset(1);
    node[0].setInvalid();
    node[0].setMatrix(global_bounds.geometry,0);

    /* create initial build record */
    BuildRecord br;
    br.init(global_bounds,0,numPrimitives);
    br.depth       = 1;
    //br.parentID    = 0;
    br.parentPtr   = &node[0].child(0);

    /* node allocator */
    NodeAllocator alloc(atomicID,numAllocated64BytesBlocks);
        
    /* push initial build record to global work stack */
    global_workStack.reset();
    global_workStack.push_nolock(br);    

    /* work in multithreaded toplevel mode until sufficient subtasks got generated */    
    const size_t coreCount = (threadCount+3)/4;
    while (global_workStack.size() < coreCount &&
	   global_workStack.size()+BVH4Hair::N <= SIZE_GLOBAL_WORK_STACK) 
    {
      BuildRecord br;
      if (!global_workStack.pop_nolock_largest(br)) break;
      recurseSAH(br,alloc,BUILD_TOP_LEVEL,threadIndex,threadCount);      
    }

    TIMER(msec = getSeconds()-msec);    
    TIMER(std::cout << "build_top_level " << 1000. * msec << " ms" << std::endl << std::flush);

    /* fill per core work queues */    
    TIMER(msec = getSeconds());    
    scene->lockstep_scheduler.dispatchTask(task_fillLocalWorkQueues, this, threadIndex, threadCount );
    TIMER(msec = getSeconds()-msec);    
    TIMER(std::cout << "task_fillLocalWorkQueues " << 1000. * msec << " ms" << std::endl << std::flush);

    /* now process all created subtasks on multiple threads */    
    TIMER(msec = getSeconds());    
    scene->lockstep_scheduler.dispatchTask(task_buildSubTrees, this, threadIndex, threadCount );
    DBG(PRINT(atomicID));
    TIMER(msec = getSeconds()-msec);    
    TIMER(std::cout << "task_buildSubTrees " << 1000. * msec << " ms" << std::endl << std::flush);

    numNodes = atomicID; 
    
    /* update BVH4 */
    
    bvh4hair->root             = node[0].child(0);
    bvh4hair->bounds           = global_bounds.geometry;
    bvh4hair->unaligned_nodes  = (BVH4Hair::UnalignedNode*)node;
    bvh4hair->accel            = prims;

    /* stop measurement */
    if (scene->device->verbosity(2)) 
      dt = getSeconds()-t0;
  }

  bool split_fallback(Bezier1i * __restrict__ const primref, BuildRecord& current, BuildRecord& leftChild, BuildRecord& rightChild)
  {
    // unsigned int blocks4 = (current.items()+3)/4;
    // unsigned int center = current.begin + (blocks4/2)*4; 

    unsigned int center = (current.begin + current.end)/2;
    assert(center != current.begin);
    assert(center != current.end);

    Centroid_Scene_AABB left; left.reset();
    for (size_t i=current.begin; i<center; i++)
      left.extend(primref[i].bounds());
    leftChild.init(left,current.begin,center);
    
    Centroid_Scene_AABB right; right.reset();
    for (size_t i=center; i<current.end; i++)
      right.extend(primref[i].bounds());	
    rightChild.init(right,center,current.end);
    
    return true;
  }

  __forceinline bool BVH4HairBuilder::split(BuildRecord& current, BuildRecord& left, BuildRecord& right, const size_t mode, const size_t threadID, const size_t numThreads)
  {
    DBG(PING);

   if (unlikely(mode == BUILD_TOP_LEVEL))
      {
	DBG(PRINT("TOP_LEVEL"));

	if (current.items() >= BUILD_RECORD_PARALLEL_SPLIT_THRESHOLD && numThreads > 1)
	  return splitParallelGlobal(current,left,right,threadID,numThreads);
	else
	  {
	    DBG(std::cout << "WARNING in top-level build: too few items for parallel split " << current.items() << std::endl << std::flush);
	    return splitSequential(current,left,right);
	  }
      }
    else if (unlikely(mode == FILL_LOCAL_QUEUES))
       {
     	if (current.items() >= THRESHOLD_FOR_SUBTREE_RECURSION)
     	  return splitParallelLocal(current,left,right,threadID);
    	else
     	  {
     	    DBG(std::cout << "WARNING in fill_local_queues build: too few items for parallel split " << current.items() << std::endl << std::flush);
     	    return splitSequential(current,left,right);
     	  }
	
       }
    else
      return splitSequential(current,left,right);    
  }


  __forceinline bool BVH4HairBuilder::splitSequential(BuildRecord& current, BuildRecord& leftChild, BuildRecord& rightChild)
  {
    /* mark as leaf if leaf threshold reached */
    if (current.items() <= MAX_ITEMS_PER_LEAF) {
      current.createLeaf();
      return false;
    }

    const BinMapping mapping(current.bounds);

    float16 leftArea[3];
    float16 rightArea[3];
    int16 leftNum[3];

    fastbin<Bezier1i>(prims,current.begin,current.end,mapping,leftArea,rightArea,leftNum);

    const Split split = getBestSplit(current,leftArea,rightArea,leftNum,mapping.getValidDimMask());

    if (unlikely(split.invalid())) 
      split_fallback(prims,current,leftChild,rightChild);
   // /* partitioning of items */
    else 
      {
	leftChild.bounds.reset();
	rightChild.bounds.reset();

	const BinPartitionMapping mapping(split,current.bounds);
	const unsigned int mid = partitionPrimitives<Bezier1i,false>(&prims[current.begin] ,current.size(), mapping, leftChild.bounds, rightChild.bounds);

	assert(area(leftChild.bounds.geometry) >= 0.0f);
	//assert(current.begin + mid == current.begin + split.numLeft) // can happen

	if (unlikely(current.begin + mid == current.begin || current.begin + mid == current.end)) 
	  {
	    std::cout << "WARNING: mid == current.begin || mid == current.end " << std::endl;
	    PRINT(split);
	    PRINT(current);
	    PRINT(mid);
	    split_fallback(prims,current,leftChild,rightChild);	    
	  }
	else
	  {
	    const unsigned int current_mid = current.begin + split.numLeft;
	    leftChild.init(current.begin,current_mid);
	    rightChild.init(current_mid,current.end);
	  }

      }



    if (leftChild.items()  <= MAX_ITEMS_PER_LEAF) leftChild.createLeaf();
    if (rightChild.items() <= MAX_ITEMS_PER_LEAF) rightChild.createLeaf();	
    return true;
  }
  

  bool BVH4HairBuilder::splitParallelGlobal( BuildRecord &current,
					  BuildRecord &leftChild,
					  BuildRecord &rightChild,
					  const size_t threadID,
					  const size_t numThreads)
  {
    DBG(PING);

    const unsigned int items = current.end - current.begin;
    assert(items >= BUILD_RECORD_PARALLEL_SPLIT_THRESHOLD);
  
    /* mark as leaf if leaf threshold reached */
    if (items <= MAX_ITEMS_PER_LEAF) {
      current.createLeaf();
      return false;
    }

     global_sharedData.rec = current;
     global_sharedData.split.reset();
     global_sharedData.left.reset();
     global_sharedData.right.reset();
     
     scene->lockstep_scheduler.dispatchTask( task_parallelBinningGlobal, this, threadID, numThreads );

     if (unlikely(global_sharedData.split.invalid())) 
       split_fallback(prims,current,leftChild,rightChild);
     else
       {
	 global_sharedData.left.reset();
	 global_sharedData.right.reset();

	 const BinPartitionMapping mapping(global_sharedData.split,global_sharedData.rec.bounds);

	 auto part = [&] (Bezier1i* const t_array,
			  const size_t size) 
	   {             
	    return partitionPrimitives<Bezier1i,true>(t_array,size,mapping,global_sharedData.left,global_sharedData.right);
	   };

	 size_t mid_parallel = parallel_in_place_partitioning_static<Bezier1i>(&prims[current.begin],
									       current.size(),
									       part,
									       scene->lockstep_scheduler);

	const unsigned int mid = current.begin + mid_parallel;

	if (unlikely(current.begin == mid || mid == current.end)) 
	  {
	    std::cout << "WARNING: mid == current.begin || mid == current.end " << std::endl;
	    PRINT(global_sharedData.split);
	    PRINT(current);
	    PRINT(mid);
	    split_fallback(prims,current,leftChild,rightChild);	    
	  }
	else
	  {
	    leftChild.init(global_sharedData.left,current.begin,mid);
	    rightChild.init(global_sharedData.right,mid,current.end);
	  }	 
       }
     
     if (leftChild.items()  <= MAX_ITEMS_PER_LEAF) leftChild.createLeaf();
     if (rightChild.items() <= MAX_ITEMS_PER_LEAF) rightChild.createLeaf();
     return true;
  }


  bool BVH4HairBuilder::splitParallelLocal(BuildRecord &current,
					   BuildRecord &leftChild,
					   BuildRecord &rightChild,
					   const size_t threadID)
  {
    const unsigned int items    = current.end - current.begin;
    const size_t globalCoreID   = threadID / 4;
    const size_t localThreadID  = threadID % 4;
    const size_t globalThreadID = threadID;
    
    assert(items >= THRESHOLD_FOR_SUBTREE_RECURSION);
  
    /* mark as leaf if leaf threshold reached */
    if (items <= MAX_ITEMS_PER_LEAF) {
      current.createLeaf();
      return false;
    }

    SharedBinningPartitionData &sd = local_sharedData[globalCoreID]; 

    sd.rec = current;
    sd.split.reset();
    sd.left.reset();
    sd.right.reset();

    localTaskScheduler[globalCoreID].dispatchTask( task_parallelBinningLocal, this, localThreadID, globalThreadID );

    if (unlikely(sd.split.invalid())) 
      split_fallback(prims,current,leftChild,rightChild);
    else
      {

	 sd.left.reset();
	 sd.right.reset();

	 const BinPartitionMapping mapping(sd.split,sd.rec.bounds);

	 auto part = [&] (Bezier1i* const t_array,
			  const size_t size) 
	   {               
	    return partitionPrimitives<Bezier1i,true>(t_array,size,mapping,sd.left,sd.right);
	   };

	 size_t mid_parallel = parallel_in_place_partitioning_static<Bezier1i>(&prims[sd.rec.begin],
									       current.size(),
									       part,
									       localTaskScheduler[globalCoreID]);
	 const unsigned int mid = current.begin + mid_parallel;


	 if (unlikely(mid == current.begin || mid == current.end)) 
	   {
	     std::cout << "WARNING: mid == current.begin || mid == current.end " << std::endl;
	     PRINT(sd.split);
	     PRINT(current);
	     PRINT(mid);
	     split_fallback(prims,current,leftChild,rightChild);	    
	   }
	 else
	   {
	     leftChild.init(sd.left,current.begin,mid);
	     rightChild.init(sd.right,mid,current.end);
	   }
	 
       }
     
     if (leftChild.items()  <= MAX_ITEMS_PER_LEAF) leftChild.createLeaf();
     if (rightChild.items() <= MAX_ITEMS_PER_LEAF) rightChild.createLeaf();
     return true;
  }


  __forceinline void BVH4HairBuilder::createLeaf(BuildRecord& current, NodeAllocator& alloc,const size_t threadIndex, const size_t threadCount)
  {
#if defined(DEBUG)
    if (current.depth > BVH4Hair::maxBuildDepthLeaf) 
      THROW_RUNTIME_ERROR("ERROR: depth limit reached");
#endif
    
    /* create leaf */
    if (current.items() <= MAX_ITEMS_PER_LEAF) {
      createBVH4HairLeaf(current.parentPtr,current.begin,current.items());
      return;
    }

    /* first split level */
    BuildRecord record0, record1;
    split_fallback(prims,current,record0,record1);

    /* second split level */
    BuildRecord children[4];
    split_fallback(prims,record0,children[0],children[1]);
    split_fallback(prims,record1,children[2],children[3]);

    /* allocate next four nodes */
    size_t numChildren = 1;
    const size_t currentIndex = alloc.get(1);

    createBVH4HairNode(current.parentPtr,currentIndex);
    
    node[currentIndex].prefetchNode<PFHINT_L2EX>();

    /* recurse into each child */
    for (size_t i=0; i<numChildren; i++) 
    {
      node[currentIndex].setMatrix(children[i].bounds.geometry, i);
      children[i].parentPtr = &node[currentIndex].child(i);
      children[i].depth     = current.depth+1;
      createLeaf(children[i],alloc,threadIndex,threadCount);
    }
  }  


  void BVH4HairBuilder::recurseSAH(BuildRecord& current, 
				   NodeAllocator& alloc,
				   const size_t mode, 
				   const size_t threadID, 
				   const size_t numThreads)
  {
    __aligned(64) BuildRecord children[BVH4Hair::N];

    /* create leaf node */
    if (current.depth >= BVH4Hair::maxBuildDepth || current.isLeaf()) {
      createLeaf(current,alloc,threadID,numThreads);
      return;
    }

    /* fill all 4 children by always splitting the one with the largest surface area */
    unsigned int numChildren = 1;
    children[0] = current;

    do {

      /* find best child with largest bounding box area */
      int bestChild = -1;
      float bestArea = neg_inf;
      for (unsigned int i=0; i<numChildren; i++)
      {
        /* ignore leaves as they cannot get split */
        if (children[i].isLeaf())
          continue;
        
        /* remember child with largest area */
        if (children[i].sceneArea() > bestArea) { 
          bestArea = children[i].sceneArea();
          bestChild = i;
        }
      }
      if (bestChild == -1) break;

      /*! split best child into left and right child */
      __aligned(64) BuildRecord left, right;
      if (!split(children[bestChild],left,right,mode,threadID,numThreads)) 
        continue;
      
      /* add new children left and right */
      left.depth = right.depth = current.depth+1;
      children[bestChild] = children[numChildren-1];
      children[numChildren-1] = left;
      children[numChildren+0] = right;
      numChildren++;
      
    } while (numChildren < BVH4Hair::N);

    /* create leaf node if no split is possible */
    if (numChildren == 1) {
      createLeaf(current,alloc,threadID,numThreads);
      return;
    }

    /* allocate next four nodes */
    const size_t currentIndex = alloc.get(1);
    
    BVH4Hair::UnalignedNode *current_node = (BVH4Hair::UnalignedNode *)&node[currentIndex];

#if ENABLE_AABB_NODES == 1
    createBVH4HairNode(current.parentPtr,currentIndex,BVH4Hair::alignednode_mask);
#else
    createBVH4HairNode(current.parentPtr,currentIndex);
#endif


    /* init used/unused nodes */
    current_node->prefetchNode<PFHINT_L2EX>();
    current_node->setInvalid();

    /* recurse into each child */
    for (unsigned int i=0; i<numChildren; i++) 
    {
      current_node->setMatrix(children[i].bounds.geometry,i);
      //children[i].parentID    = currentIndex;
      children[i].parentPtr   = &current_node->child(i);      
      recurse(children[i],alloc,mode,threadID,numThreads);
    }    

    DBG(PRINT(*current_node));

  }


  __forceinline void BVH4HairBuilder::recurse(BuildRecord& current, NodeAllocator& alloc,const size_t mode, const size_t threadID, const size_t numThreads)
  {
    if (mode == BUILD_TOP_LEVEL) {
      global_workStack.push_nolock(current);
    }
    else if (mode == FILL_LOCAL_QUEUES) {
      const size_t coreID = threadID/4;
      if (!local_workStack[coreID].push(current))
        recurseSAH(current,alloc,RECURSE,threadID,numThreads);
    }
    else
      {
	BuildRecordOBB current_obb;
	current_obb = current;

	computeUnalignedSpace(current_obb);
	computeUnalignedSpaceBounds(current_obb);
   
#if defined(THRESHOLD_SWITCH)
	if (area( current_obb.bounds.geometry ) < area( current.bounds.geometry ) * AABB_OBB_SWITCH_THRESHOLD)
	  recurseOBB(current_obb,alloc,/*mode*/ RECURSE,threadID,numThreads);
	else
	  recurseSAH(current,alloc,RECURSE,threadID,numThreads);
#else
	recurseOBB(current_obb,alloc,/*mode*/ RECURSE,threadID,numThreads);
#endif


      }
  }

  // ===============================================================================================================================================
  // ===============================================================================================================================================
  // ===============================================================================================================================================


  void BVH4HairBuilder::recurseOBB(BuildRecordOBB& current, NodeAllocator& alloc, const size_t mode, const size_t threadID, const size_t numThreads)
  {
#if ENABLE_OBB_BVH4 == 0
    FATAL("recurseOBB disabled");
#endif

    __aligned(64) BuildRecordOBB children[BVH4Hair::N];

    /* create leaf node */
    if (current.depth >= BVH4Hair::maxBuildDepth || current.isLeaf()) {
      createLeaf(current,alloc,threadID,numThreads);
      return;
    }

    /* fill all 4 children by always splitting the one with the largest surface area */
    unsigned int numChildren = 1;
    children[0] = current;

    do {

      /* find best child with largest bounding box area */
      int bestChild = -1;
      float bestArea = neg_inf;
      for (unsigned int i=0; i<numChildren; i++)
      {
        /* ignore leaves as they cannot get split */
        if (children[i].isLeaf())
          continue;
        
        /* remember child with largest area */
        if (children[i].sceneArea() > bestArea) { 
          bestArea = children[i].sceneArea();
          bestChild = i;
        }
      }
      if (bestChild == -1) break;

      /*! split best child into left and right child */
      __aligned(64) BuildRecordOBB left, right;
      if (!splitSequentialOBB(children[bestChild],left,right)) 
        continue;
      
      /* add new children left and right */
      left.depth = right.depth = current.depth+1;
      children[bestChild] = children[numChildren-1];
      children[numChildren-1] = left;
      children[numChildren+0] = right;
      numChildren++;
      
    } while (numChildren < BVH4Hair::N);

    /* create leaf node if no split is possible */
    if (numChildren == 1) {
      createLeaf(current,alloc,threadID,numThreads);
      return;
    }

    /* allocate next four nodes */
    const size_t currentIndex = alloc.get(1);
    /* recurseOBB */

    node[currentIndex].prefetchNode<PFHINT_L2EX>();

    /* init used/unused nodes */
    node[currentIndex].setInvalid();

    // === default OBB node ===
    createBVH4HairNode(current.parentPtr,currentIndex);

    for (unsigned int i=0; i<numChildren; i++) 
      node[currentIndex].setMatrix(children[i].xfm,children[i].bounds.geometry,i);

    /* recurse into each child */
    for (unsigned int i=0; i<numChildren; i++) 
      {
	children[i].parentPtr = &node[currentIndex].child(i);
	recurseOBB(children[i],alloc,mode,threadID,numThreads);
      }  

  }

  void BVH4HairBuilder::buildSubTree(BuildRecord& current, 
				     NodeAllocator& alloc, 
				     const size_t mode,
				     const size_t threadID, 
				     const size_t numThreads)
  {
    DBG(PING);
#if ENABLE_OBB_BVH4 == 1

    if (mode == FILL_LOCAL_QUEUES)
      {
	recurseSAH(current,alloc,mode,threadID,numThreads);
	return;
      }
    else
      {

	BuildRecordOBB current_obb;
	current_obb = current;

	computeUnalignedSpace(current_obb);
	computeUnalignedSpaceBounds(current_obb);

#if defined( THRESHOLD_SWITCH ) 
	if (area( current_obb.bounds.geometry ) < area( current.bounds.geometry ) * AABB_OBB_SWITCH_THRESHOLD)
	  recurseOBB(current_obb,alloc,/*mode*/ RECURSE,threadID,numThreads);
	else
	  recurseSAH(current,alloc,/*mode*/ RECURSE,threadID,numThreads);
#else
	recurseOBB(current_obb,alloc,/*mode*/ RECURSE,threadID,numThreads);

#endif
      }

#else
    recurseSAH(current,alloc,/*mode*/ RECURSE,threadID,numThreads);
#endif
  }


  __forceinline bool BVH4HairBuilder::splitSequentialOBB(BuildRecordOBB& current, BuildRecordOBB& leftChild, BuildRecordOBB& rightChild)
  {
    DBG(PING);
    DBG(PRINT(current));

    //computeUnalignedSpace(current);
    //computeUnalignedSpaceBounds(current);

    /* mark as leaf if leaf threshold reached */
    if (current.items() <= MAX_ITEMS_PER_LEAF) {
      current.createLeaf();
      return false;
    }


    const unsigned int items = current.items();
    const float voxelArea = area(current.bounds.geometry);     
    const BinMapping mapping(current.bounds);

    float16 leftArea[3];
    float16 rightArea[3];
    int16 leftNum[3];


    const Vec3f16 cmat = convert(current.xfm);
    
    fastbin_xfm<Bezier1i>(prims,cmat,current.begin,current.end,mapping,leftArea,rightArea,leftNum);


    Split split = getBestSplit(current,leftArea,rightArea,leftNum,mapping.getValidDimMask());

    if (unlikely(split.invalid())) 
      {
	split_fallback(prims,current,leftChild,rightChild);
      }
    else 
      {
	/* partitioning of items */
	leftChild.bounds.reset();
	rightChild.bounds.reset();

	const BinPartitionMapping mapping(split,current.bounds);
	const unsigned int mid = partitionPrimitives_xfm<L2_PREFETCH_ITEMS>(prims,cmat,current.begin, current.end, mapping, leftChild.bounds, rightChild.bounds);

	assert(area(leftChild.bounds.geometry) >= 0.0f);
	assert(current.begin + mid == current.begin + split.numLeft);

	if (unlikely(current.begin + mid == current.begin || current.begin + mid == current.end)) 
	  {
	    std::cout << "WARNING: mid == current.begin || mid == current.end " << std::endl;
	    PRINT(split);
	    PRINT(current);
	    PRINT(mid);
	    split_fallback(prims,current,leftChild,rightChild);	    
	  }
	else
	  {
	    const unsigned int current_mid = current.begin + split.numLeft;
	    leftChild.init(current.begin,current_mid);
	    rightChild.init(current_mid,current.end);
	  }

      }

    computeUnalignedSpace(leftChild);
    computeUnalignedSpaceBounds(leftChild);

    computeUnalignedSpace(rightChild);
    computeUnalignedSpaceBounds(rightChild);

    //leftChild.xfm  = current.xfm;
    //rightChild.xfm = current.xfm;

    //if (leftChild.items()  <= MAX_ITEMS_PER_LEAF) leftChild.createLeaf();
    //if (rightChild.items() <= MAX_ITEMS_PER_LEAF) rightChild.createLeaf();

    return true;
  }


  __forceinline void BVH4HairBuilder::computeUnalignedSpace( BuildRecordOBB& current )
  {
    Vec3fa axis(0,0,1);
#if 1
    for (size_t i=current.begin;i<current.end;i++)
      {
	const Bezier1i &b = prims[i];
	const Vec3fa &p0 = b.p[0];
	const Vec3fa &p3 = b.p[3];
	const Vec3fa axis1 = normalize(p3 - p0);
	if (length(p3 - p0) > 1E-9f) {
	  axis = axis1;
	  break;
	}	
      }
#endif
    current.xfm = frame(axis).transposed();    

    current.PreQuantizeMatrix();

    DBG(PRINT(current.xfm));
  }

  __forceinline void BVH4HairBuilder::computeUnalignedSpaceBounds( BuildRecordOBB& current )
  {
    const float16 c0 = broadcast4to16f((float*)&current.xfm.vx);
    const float16 c1 = broadcast4to16f((float*)&current.xfm.vy);
    const float16 c2 = broadcast4to16f((float*)&current.xfm.vz);

    const float16 p_inf( pos_inf );
    const float16 n_inf( neg_inf );

    Vec2f16 centroid2(p_inf, n_inf);
    Vec2f16 geometry(p_inf, n_inf);

    for (size_t i=current.begin;i<current.end;i++)
      {
	prefetch<PFHINT_NT>(&prims[i+4]);
	prefetch<PFHINT_L2>(&prims[i+16]);
	const Vec2f16 b = prims[i].getBounds(c0,c1,c2);
	const float16 b_min = b.x;
	const float16 b_max = b.y;
	const float16 c2    = b_min + b_max;
	centroid2.x = min(centroid2.x, c2);
	centroid2.y = max(centroid2.y, c2);
	geometry.x  = min(geometry.x, b_min); 
	geometry.y  = max(geometry.y, b_max); 	
      }    
    
    store4f(&current.bounds.centroid2.lower,centroid2.x);
    store4f(&current.bounds.centroid2.upper,centroid2.y);

    store4f(&current.bounds.geometry.lower,geometry.x);
    store4f(&current.bounds.geometry.upper,geometry.y);

    DBG(
	for (size_t i=current.begin;i<current.end;i++)
	  {
	    for (size_t j=0;j<4;j++)
	      {
		const Vec3fa v = prims[i].p[j];
		const Vec3fa xfm_v = xfmPoint(current.xfm,v);
		if (disjoint(current.bounds.geometry,xfm_v))
		  {
		    PRINT(v);
		    PRINT(xfm_v);
		    PRINT(current.xfm);
		    PRINT(current.bounds.geometry);
		    exit(0);
		  }
	      }
	  }
	
	);
    current.sArea = area(current.bounds.geometry);

  }


  void BVH4HairBuilder::createAccel(const size_t threadIndex, const size_t threadCount)
  {
    DBG(PING);
  }

  std::string BVH4HairBuilder::getStatistics()
  {
    return std::string("not implemented");
  }


};
