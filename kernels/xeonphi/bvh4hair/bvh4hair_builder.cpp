#include "bvh4hair/bvh4hair_builder.h"
#include "geometry/bezier1i.h"

namespace embree
{
#define DBG(x) 

#define L1_PREFETCH_ITEMS 2
#define L2_PREFETCH_ITEMS 16
#define SINGLE_THREADED_BUILD_THRESHOLD        512
#define TIMER(x)  

  static double dt = 0.0f;

  // ==========================================================================================
  // ==========================================================================================
  // ==========================================================================================


  void BVH4HairBuilder::printBuilderName()
  {
    std::cout << "building BVH4Hair binned SAH builder (MIC) ... " << std::endl;    
    
  }


  size_t BVH4HairBuilder::getNumPrimitives()
  {
    DBG(PING);

    /* count total number of virtual objects */
    size_t numCurves = 0;       
    for (size_t i=0;i<scene->size();i++)
      {
	if (unlikely(scene->get(i) == NULL)) continue;
	if (unlikely((scene->get(i)->type != BEZIER_CURVES))) continue;
	if (unlikely(!scene->get(i)->isEnabled())) continue;
        BezierCurves* geom = (BezierCurves*) scene->getBezierCurves(i);
	numCurves += geom->numCurves;
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
	const size_t numPrims = numPrimitives+4;
	const size_t minAllocNodes = numPrims ? threadCount * ALLOCATOR_NODE_BLOCK_SIZE * 4: 16;
	const size_t numNodes = max((size_t)(numPrims * BVH_NODE_PREALLOC_FACTOR),minAllocNodes);
	allocateMemoryPools(numPrims,numNodes);
      }
  }

  void BVH4HairBuilder::computePrimRefs(const size_t threadIndex, const size_t threadCount)
  {
    DBG(PING);
    LockStepTaskScheduler::dispatchTask( task_computePrimRefsBezierCurves, this, threadIndex, threadCount );	
  }

  void BVH4HairBuilder::computePrimRefsBezierCurves(const size_t threadID, const size_t numThreads) 
  {
    DBG(PING);

    const size_t numTotalGroups = scene->size();

    /* count total number of virtual objects */
    const size_t numBezierCurves = numPrimitives;
    const size_t startID   = (threadID+0)*numBezierCurves/numThreads;
    const size_t endID     = (threadID+1)*numBezierCurves/numThreads; 

    DBG(
	DBG_PRINT(numTotalGroups);
	DBG_PRINT(numBezierCurves);
	DBG_PRINT(startID);
	DBG_PRINT(endID);
	);
    
    Bezier1i *__restrict__ const bptr     = (Bezier1i*)this->prims;

    // === find first group containing startID ===
    unsigned int g=0, numSkipped = 0;
    for (; g<numTotalGroups; g++) {       
      if (unlikely(scene->get(g) == NULL)) continue;
      if (unlikely((scene->get(g)->type != BEZIER_CURVES))) continue;
      if (unlikely(!scene->get(g)->isEnabled())) continue;
      BezierCurves* geom = (BezierCurves*) scene->getBezierCurves(g);
      const size_t numPrims = geom->numCurves;
      if (numSkipped + numPrims > startID) break;
      numSkipped += numPrims;
    }

    /* start with first group containing startID */
    mic_f bounds_scene_min((float)pos_inf);
    mic_f bounds_scene_max((float)neg_inf);
    mic_f bounds_centroid_min((float)pos_inf);
    mic_f bounds_centroid_max((float)neg_inf);

    unsigned int num = 0;
    unsigned int currentID = startID;
    unsigned int offset = startID - numSkipped;

    for (; g<numTotalGroups; g++) 
      {
	if (unlikely(scene->get(g) == NULL)) continue;
	if (unlikely((scene->get(g)->type != BEZIER_CURVES))) continue;
	if (unlikely(!scene->get(g)->isEnabled())) continue;

	BezierCurves* geom = (BezierCurves*) scene->getBezierCurves(g);

        size_t N = geom->numCurves;
        for (unsigned int i=offset; i<N && currentID < endID; i++, currentID++)	 
        { 			    
	  const mic2f b2 = geom->bounds_mic2f(i);
	  const mic_f bmin = b2.x;
	  const mic_f bmax = b2.y;
          
          bounds_scene_min = min(bounds_scene_min,bmin);
          bounds_scene_max = max(bounds_scene_max,bmax);
          const mic_f centroid2 = bmin+bmax;
          bounds_centroid_min = min(bounds_centroid_min,centroid2);
          bounds_centroid_max = max(bounds_centroid_max,centroid2);

	  bptr[currentID].p = geom->fristVertexPtr(i);
          bptr[currentID].geomID = g;
          bptr[currentID].primID = i;
        }
        if (currentID == endID) break;
        offset = 0;
      }

    /* update global bounds */
    Centroid_Scene_AABB bounds;
    
    store4f(&bounds.centroid2.lower,bounds_centroid_min);
    store4f(&bounds.centroid2.upper,bounds_centroid_max);
    store4f(&bounds.geometry.lower,bounds_scene_min);
    store4f(&bounds.geometry.upper,bounds_scene_max);

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
    const size_t size_node     = numNodes * BVH_NODE_PREALLOC_FACTOR * sizeNodeInBytes + additional_size;
    const size_t size_accel    = numPrims * sizeAccelInBytes + additional_size;

    numAllocatedNodes = size_node / sizeNodeInBytes;
      
    DBG(DBG_PRINT(numAllocatedNodes));
    DBG(DBG_PRINT(size_primrefs));
    DBG(DBG_PRINT(size_node));
    DBG(DBG_PRINT(size_accel));

    prims = (Bezier1i                 *) os_malloc(size_primrefs); 
    node  = (BVH4Hair::UnalignedNode  *) os_malloc(size_node);
    accel = (Bezier1i                 *) os_malloc(size_accel);

    assert(prims  != 0);
    assert(node   != 0);
    assert(accel  != 0);

    bvh4hair->accel = accel;
    bvh4hair->qbvh  = (BVH4i::Node*)node;
    bvh4hair->size_node  = size_node;
    bvh4hair->size_accel = size_accel;

    size_prims = size_primrefs;    
    size_t total = size_primrefs+size_node+size_accel;
  }


  void BVH4HairBuilder::build(const size_t threadIndex, const size_t threadCount) 
  {
    DBG(PING);
    const size_t totalNumPrimitives = getNumPrimitives();


    DBG(DBG_PRINT(totalNumPrimitives));

    /* print builder name */
    if (unlikely(g_verbose >= 1)) printBuilderName();

    /* allocate BVH data */
    allocateData(TaskScheduler::getNumThreads(),totalNumPrimitives);

    LockStepTaskScheduler::init(TaskScheduler::getNumThreads()); 

    if (likely(numPrimitives > SINGLE_THREADED_BUILD_THRESHOLD && TaskScheduler::getNumThreads() > 1) )
      {
	DBG(std::cout << "PARALLEL BUILD" << std::endl);
	TaskScheduler::executeTask(threadIndex,threadCount,_build_parallel_hair,this,TaskScheduler::getNumThreads(),"build_parallel");
      }
    else
      {
	/* number of primitives is small, just use single threaded mode */
	if (likely(numPrimitives > 0))
	  {
	    DBG(std::cout << "SERIAL BUILD" << std::endl);
	    build_parallel_hair(0,1,0,0,NULL);
	  }
	else
	  {
	    DBG(std::cout << "EMPTY SCENE BUILD" << std::endl);
	    /* handle empty scene */
	    for (size_t i=0;i<4;i++)
	      bvh4hair->qbvh[0].setInvalid(i);
	    for (size_t i=0;i<4;i++)
	      bvh4hair->qbvh[1].setInvalid(i);
	    bvh4hair->qbvh[0].lower[0].child = BVH4i::NodeRef(128);
	    bvh4hair->root = bvh4hair->qbvh[0].lower[0].child; 
	    bvh4hair->bounds = empty;
	  }
      }

    if (g_verbose >= 2) {
      double perf = totalNumPrimitives/dt*1E-6;
      std::cout << "[DONE] " << 1000.0f*dt << "ms (" << perf << " Mtris/s), primitives " << numPrimitives << std::endl;
      //std::cout << BVH4iStatistics(bvh).str();
    }

  }

  void BVH4HairBuilder::build_parallel_hair(size_t threadIndex, size_t threadCount, size_t taskIndex, size_t taskCount, TaskScheduler::Event* event) 
  {
    PING;

    TIMER(double msec = 0.0);

    /* initialize thread-local work stacks */
    if (threadIndex % 4 == 0)
      local_workStack[threadIndex].reset();

    /* all worker threads enter tasking system */
    if (threadIndex != 0) {
      LockStepTaskScheduler::dispatchTaskMainLoop(threadIndex,threadCount); 
      return;
    }

    /* start measurement */
    double t0 = 0.0f;
#if !defined(PROFILE)
    if (g_verbose >= 2) 
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
    br.depth = 1;
    br.parentID    = 0;
    br.parentBoxID = 0;

    /* node allocator */
    NodeAllocator alloc(atomicID,numAllocatedNodes);
        
#if 1
    recurseSAH(br,alloc,RECURSE,threadIndex,threadCount);
    
#else
    /* push initial build record to global work stack */
    global_workStack.reset();
    global_workStack.push_nolock(br);    

    /* work in multithreaded toplevel mode until sufficient subtasks got generated */    
    const size_t coreCount = (threadCount+3)/4;
    while (global_workStack.size() < coreCount &&
	   global_workStack.size()+BVH4i::N <= SIZE_GLOBAL_WORK_STACK) 
    {
      BuildRecord br;
      if (!global_workStack.pop_nolock_largest(br)) break;
      DBG(DBG_PRINT(br));
      //recurseSAH(br,alloc,BUILD_TOP_LEVEL,threadIndex,threadCount);      
    }

    TIMER(msec = getSeconds()-msec);    
    TIMER(std::cout << "build_top_level " << 1000. * msec << " ms" << std::endl << std::flush);

    /* fill per core work queues */    
    TIMER(msec = getSeconds());    
    LockStepTaskScheduler::dispatchTask(task_fillLocalWorkQueues, this, threadIndex, threadCount );
    TIMER(msec = getSeconds()-msec);    
    TIMER(std::cout << "task_fillLocalWorkQueues " << 1000. * msec << " ms" << std::endl << std::flush);

    /* now process all created subtasks on multiple threads */    
    TIMER(msec = getSeconds());    
    LockStepTaskScheduler::dispatchTask(task_buildSubTrees, this, threadIndex, threadCount );
    numNodes = atomicID; // >> 2;
    DBG(DBG_PRINT(atomicID));
    TIMER(msec = getSeconds()-msec);    
    TIMER(std::cout << "task_buildSubTrees " << 1000. * msec << " ms" << std::endl << std::flush);
#endif
    
    /* update BVH4 */
    //bvh4hair->root   = bvh4hair->qbvh[0].lower[0].child; 
    bvh4hair->bounds = global_bounds.geometry;
    
    /* release all threads again */
    LockStepTaskScheduler::releaseThreads(threadCount);

    /* stop measurement */
    if (g_verbose >= 2) 
      dt = getSeconds()-t0;
  }

  bool split_fallback(Bezier1i * __restrict__ const primref, BuildRecord& current, BuildRecord& leftChild, BuildRecord& rightChild)
  {
    const unsigned int center = (current.begin + current.end)/2;
    
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

  __forceinline bool BVH4HairBuilder::split(BuildRecord& current, BuildRecord& leftChild, BuildRecord& rightChild, const size_t mode, const size_t threadID, const size_t numThreads)
  {
    /* mark as leaf if leaf threshold reached */
    if (current.items() <= BVH4Hair::N) {
      current.createLeaf();
      return false;
    }
    
    const mic_f centroidMin = broadcast4to16f(&current.bounds.centroid2.lower);
    const mic_f centroidMax = broadcast4to16f(&current.bounds.centroid2.upper);

    const mic_f centroidBoundsMin_2 = centroidMin;
    const mic_f centroidDiagonal_2  = centroidMax-centroidMin;
    const mic_f scale = select(centroidDiagonal_2 != 0.0f,rcp(centroidDiagonal_2) * mic_f(16.0f * 0.99f),mic_f::zero());

    mic_f leftArea[3];
    mic_f rightArea[3];
    mic_i leftNum[3];

    fastbin<Bezier1i>(prims,current.begin,current.end,centroidBoundsMin_2,scale,leftArea,rightArea,leftNum);

    const unsigned int items = current.items();
    const float voxelArea = area(current.bounds.geometry);
    Split split;
    split.cost = items * voxelArea;

    for (size_t dim = 0;dim < 3;dim++) 
      {
	if (unlikely(centroidDiagonal_2[dim] == 0.0f)) continue;

	const mic_f rArea   = rightArea[dim]; // bin16.prefix_area_rl(dim);
	const mic_f lArea   = leftArea[dim];  // bin16.prefix_area_lr(dim);      
	const mic_i lnum    = leftNum[dim];   // bin16.prefix_count(dim);

	const mic_i rnum    = mic_i(items) - lnum;
	const mic_i lblocks = (lnum + mic_i(3)) >> 2;
	const mic_i rblocks = (rnum + mic_i(3)) >> 2;
	const mic_m m_lnum  = lnum == 0;
	const mic_m m_rnum  = rnum == 0;
	const mic_f cost    = select(m_lnum|m_rnum,mic_f::inf(),lArea * mic_f(lblocks) + rArea * mic_f(rblocks) + voxelArea );

	if (lt(cost,mic_f(split.cost)))
	  {

	    const mic_f min_cost    = vreduce_min(cost); 
	    const mic_m m_pos       = min_cost == cost;
	    const unsigned long pos = bitscan64(m_pos);	    

	    assert(pos < 15);

	    if (pos < 15)
	      {
		split.cost    = cost[pos];
		split.pos     = pos+1;
		split.dim     = dim;	    
		split.numLeft = lnum[pos];
	      }
	  }
      };

    if (unlikely(split.pos == -1)) 
      split_fallback(prims,current,leftChild,rightChild);
   // /* partitioning of items */
    else 
      {
	leftChild.bounds.reset();
	rightChild.bounds.reset();

	const unsigned int mid = partitionPrimitives<L2_PREFETCH_ITEMS>(prims ,current.begin, current.end-1, split.pos, split.dim, centroidBoundsMin_2, scale, leftChild.bounds, rightChild.bounds);

	assert(area(leftChild.bounds.geometry) >= 0.0f);
	assert(current.begin + mid == current.begin + split.numLeft);

	if (unlikely(current.begin + mid == current.begin || current.begin + mid == current.end)) 
	  {
	    std::cout << "WARNING: mid == current.begin || mid == current.end " << std::endl;
	    DBG_PRINT(split);
	    DBG_PRINT(current);
	    DBG_PRINT(mid);
	    split_fallback(prims,current,leftChild,rightChild);	    
	  }
	else
	  {
	    const unsigned int current_mid = current.begin + split.numLeft;
	    leftChild.init(current.begin,current_mid);
	    rightChild.init(current_mid,current.end);
	  }

      }



    if (leftChild.items()  <= BVH4Hair::N) leftChild.createLeaf();
    if (rightChild.items() <= BVH4Hair::N) rightChild.createLeaf();	
    return true;
  }
  

  __forceinline void BVH4HairBuilder::createLeaf(BuildRecord& current, NodeAllocator& alloc,const size_t threadIndex, const size_t threadCount)
  {
#if defined(DEBUG)
    if (current.depth > BVH4Hair::maxBuildDepthLeaf) 
      throw std::runtime_error("ERROR: depth limit reached");
#endif
    
    /* create leaf */
    if (current.items() <= BVH4Hair::N) {
      node[current.parentID].createLeaf(current.begin,current.items(),current.parentBoxID);
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

    node[current.parentID].createNode(&node[currentIndex],current.parentBoxID);
    
    /* recurse into each child */
    for (size_t i=0; i<numChildren; i++) 
    {
      node[currentIndex].setMatrix(children[i].bounds.geometry, i);
      children[i].parentID    = currentIndex;
      children[i].parentBoxID = i;
      children[i].depth       = current.depth+1;
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
    node[current.parentID].createNode(&node[currentIndex],current.parentBoxID);

    /* init used/unused nodes */
    node[currentIndex].setInvalid();

    /* recurse into each child */
    for (unsigned int i=0; i<numChildren; i++) 
    {
      //node[currentIndex+i].lower = (Vec3fa) children[i].bounds.geometry.lower;
      //node[currentIndex+i].upper = (Vec3fa) children[i].bounds.geometry.upper;
      node[currentIndex].setMatrix(children[i].bounds.geometry,i);
      children[i].parentID    = currentIndex;
      children[i].parentBoxID = i;
      recurseSAH(children[i],alloc,mode,threadID,numThreads);
    }    
  }

  void BVH4HairBuilder::buildSubTree(BuildRecord& current, 
				     NodeAllocator& alloc, 
				     const size_t mode,
				     const size_t threadID, 
				     const size_t numThreads)
  {

  }


  // ==========================================================================================
  // ==========================================================================================
  // ==========================================================================================


  void BVH4HairBuilderConvert::printBuilderName()
  {
    std::cout << "building BVH4Hair using BVH4i conversion (MIC) ... " << std::endl;    
  }

  size_t BVH4HairBuilderConvert::getNumPrimitives()
  {
    DBG(PING);

    /* count total number of virtual objects */
    size_t numCurves = 0;       
    for (size_t i=0;i<scene->size();i++)
      {
	if (unlikely(scene->get(i) == NULL)) continue;
	if (unlikely((scene->get(i)->type != BEZIER_CURVES))) continue;
	if (unlikely(!scene->get(i)->isEnabled())) continue;
        BezierCurves* geom = (BezierCurves*) scene->getBezierCurves(i);
	numCurves += geom->numCurves;
      }
    return numCurves;	
  }

  void BVH4HairBuilderConvert::computePrimRefs(const size_t threadIndex, const size_t threadCount)
  {
    DBG(PING);
    LockStepTaskScheduler::dispatchTask( task_computePrimRefsBezierCurves, this, threadIndex, threadCount );	
  }

  void BVH4HairBuilderConvert::createAccel(const size_t threadIndex, const size_t threadCount)
  {
    DBG(PING);
    LockStepTaskScheduler::dispatchTask( task_createBezierCurvesAccel, this, threadIndex, threadCount );
  }

  void BVH4HairBuilderConvert::computePrimRefsBezierCurves(const size_t threadID, const size_t numThreads) 
  {
    DBG(PING);

    const size_t numTotalGroups = scene->size();

    /* count total number of virtual objects */
    const size_t numBezierCurves = numPrimitives;
    const size_t startID   = (threadID+0)*numBezierCurves/numThreads;
    const size_t endID     = (threadID+1)*numBezierCurves/numThreads; 

    DBG(
	DBG_PRINT(numTotalGroups);
	DBG_PRINT(numBezierCurves);
	DBG_PRINT(startID);
	DBG_PRINT(endID);
	);
    
    PrimRef *__restrict__ const prims     = this->prims;

    // === find first group containing startID ===
    unsigned int g=0, numSkipped = 0;
    for (; g<numTotalGroups; g++) {       
      if (unlikely(scene->get(g) == NULL)) continue;
      if (unlikely((scene->get(g)->type != BEZIER_CURVES))) continue;
      if (unlikely(!scene->get(g)->isEnabled())) continue;
      BezierCurves* geom = (BezierCurves*) scene->getBezierCurves(g);
      const size_t numPrims = geom->numCurves;
      if (numSkipped + numPrims > startID) break;
      numSkipped += numPrims;
    }

    /* start with first group containing startID */
    mic_f bounds_scene_min((float)pos_inf);
    mic_f bounds_scene_max((float)neg_inf);
    mic_f bounds_centroid_min((float)pos_inf);
    mic_f bounds_centroid_max((float)neg_inf);

    unsigned int num = 0;
    unsigned int currentID = startID;
    unsigned int offset = startID - numSkipped;

    for (; g<numTotalGroups; g++) 
      {
	if (unlikely(scene->get(g) == NULL)) continue;
	if (unlikely((scene->get(g)->type != BEZIER_CURVES))) continue;
	if (unlikely(!scene->get(g)->isEnabled())) continue;

	BezierCurves* geom = (BezierCurves*) scene->getBezierCurves(g);

        size_t N = geom->numCurves;
        for (unsigned int i=offset; i<N && currentID < endID; i++, currentID++)	 
        { 			    
          //const BBox3fa bounds = geom->bounds(i);
          //const mic_f bmin = broadcast4to16f(&bounds.lower); 
          //const mic_f bmax = broadcast4to16f(&bounds.upper);
	  const mic2f b2 = geom->bounds_mic2f(i);
	  const mic_f bmin = b2.x;
	  const mic_f bmax = b2.y;

          bounds_scene_min = min(bounds_scene_min,bmin);
          bounds_scene_max = max(bounds_scene_max,bmax);
          const mic_f centroid2 = bmin+bmax;
          bounds_centroid_min = min(bounds_centroid_min,centroid2);
          bounds_centroid_max = max(bounds_centroid_max,centroid2);

          store4f(&prims[currentID].lower,bmin);
          store4f(&prims[currentID].upper,bmax);	
          prims[currentID].lower.a = g;
          prims[currentID].upper.a = i;
        }
        if (currentID == endID) break;
        offset = 0;
      }

    /* update global bounds */
    Centroid_Scene_AABB bounds;
    
    store4f(&bounds.centroid2.lower,bounds_centroid_min);
    store4f(&bounds.centroid2.upper,bounds_centroid_max);
    store4f(&bounds.geometry.lower,bounds_scene_min);
    store4f(&bounds.geometry.upper,bounds_scene_max);

    global_bounds.extend_atomic(bounds);    
  }


  void BVH4HairBuilderConvert::createBezierCurvesAccel(const size_t threadID, const size_t numThreads)
  {
    DBG(PING);

    const size_t startID = (threadID+0)*numPrimitives/numThreads;
    const size_t endID   = (threadID+1)*numPrimitives/numThreads;

    Bezier1i *acc = (Bezier1i*)accel + startID;

    const PrimRef*  bptr = prims + startID;

    for (size_t j=startID; j<endID; j++, bptr++,acc++)
      {
     	prefetch<PFHINT_NT>(bptr + L1_PREFETCH_ITEMS);
     	prefetch<PFHINT_L2>(bptr + L2_PREFETCH_ITEMS);
     	assert(bptr->geomID() < scene->size() );
	
	const unsigned int geomID = bptr->geomID();
	const unsigned int primID = bptr->primID();
	BezierCurves* geom = (BezierCurves*) scene->getBezierCurves(geomID);

     	*acc = Bezier1i( &geom->vertex( geom->curve( primID ) ), geomID, primID);
      }
  }

  
  void BVH4HairBuilderConvert::build(const size_t threadIndex, const size_t threadCount) 
  {
    DBG(PING);
    const size_t totalNumPrimitives = getNumPrimitives();


    DBG(DBG_PRINT(totalNumPrimitives));

    /* print builder name */
    if (unlikely(g_verbose >= 1)) printBuilderName();

    /* allocate BVH data */
    allocateData(TaskScheduler::getNumThreads(),totalNumPrimitives);

    LockStepTaskScheduler::init(TaskScheduler::getNumThreads()); 

    if (likely(numPrimitives > SINGLE_THREADED_BUILD_THRESHOLD && TaskScheduler::getNumThreads() > 1) )
      {
	DBG(std::cout << "PARALLEL BUILD" << std::endl);

	TaskScheduler::executeTask(threadIndex,threadCount,_build_parallel,this,TaskScheduler::getNumThreads(),"build_parallel");
      }
    else
      {
	/* number of primitives is small, just use single threaded mode */
	if (likely(numPrimitives > 0))
	  {
	    DBG(std::cout << "SERIAL BUILD" << std::endl);
	    build_parallel(0,1,0,0,NULL);
	  }
	else
	  {
	    DBG(std::cout << "EMPTY SCENE BUILD" << std::endl);
	    /* handle empty scene */
	    for (size_t i=0;i<4;i++)
	      bvh->qbvh[0].setInvalid(i);
	    for (size_t i=0;i<4;i++)
	      bvh->qbvh[1].setInvalid(i);
	    bvh->qbvh[0].lower[0].child = BVH4i::NodeRef(128);
	    bvh->root = bvh->qbvh[0].lower[0].child; 
	    bvh->bounds = empty;
	  }
      }

    if (numNodes && numPrimitives)
      {
	std::cout << "converting from bvh4i to bvh4hair..." << std::endl;
	DBG_PRINT(bvh4hair);
	DBG_PRINT(numNodes);
	bvh4hair->unaligned_nodes = (BVH4Hair::UnalignedNode*)os_malloc(sizeof(BVH4Hair::UnalignedNode) * numNodes);
	for (size_t i=0;i<numNodes;i++)
	  {
	    bvh4hair->unaligned_nodes[i].convertFromBVH4iNode(bvh4hair->qbvh[i],bvh4hair->unaligned_nodes);
	    //DBG_PRINT(i);
	    //DBG_PRINT(bvh4hair->qbvh[i]);
	    //DBG_PRINT(bvh4hair->unaligned_nodes[i]);
	  }
      }

    if (g_verbose >= 2) {
      double perf = totalNumPrimitives/dt*1E-6;
      std::cout << "[DONE] " << 1000.0f*dt << "ms (" << perf << " Mtris/s), primitives " << numPrimitives << std::endl;
      std::cout << BVH4iStatistics(bvh).str();
    }

  }

};
