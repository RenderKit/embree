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
	allocateMemoryPools(numPrims,numNodes,sizeof(BVH4Hair::UnalignedNode),sizeof(Bezier1i));
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

  void BVH4HairBuilder::build(size_t threadIndex, size_t threadCount) 
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
	      bvh->qbvh[0].setInvalid(i);
	    for (size_t i=0;i<4;i++)
	      bvh->qbvh[1].setInvalid(i);
	    bvh->qbvh[0].lower[0].child = BVH4i::NodeRef(128);
	    bvh->root = bvh->qbvh[0].lower[0].child; 
	    bvh->bounds = empty;
	  }
      }

    if (g_verbose >= 2) {
      double perf = totalNumPrimitives/dt*1E-6;
      std::cout << "[DONE] " << 1000.0f*dt << "ms (" << perf << " Mtris/s), primitives " << numPrimitives << std::endl;
      std::cout << BVH4iStatistics(bvh).str();
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
    atomicID.reset(numNodesToAllocate);
    node[0].lower = global_bounds.geometry.lower;
    node[0].upper = global_bounds.geometry.upper;
    
    /* create initial build record */
    BuildRecord br;
    br.init(global_bounds,0,numPrimitives);
    br.depth = 1;
    br.parentID = 0;
        
    /* push initial build record to global work stack */
    global_workStack.reset();
    global_workStack.push_nolock(br);    

    /* work in multithreaded toplevel mode until sufficient subtasks got generated */    
    NodeAllocator alloc(atomicID,numAllocatedNodes);
    const size_t coreCount = (threadCount+3)/4;
    while (global_workStack.size() < coreCount &&
	   global_workStack.size()+BVH4i::N <= SIZE_GLOBAL_WORK_STACK) 
    {
      BuildRecord br;
      if (!global_workStack.pop_nolock_largest(br)) break;
      DBG(DBG_PRINT(br));
      recurseSAH(br,alloc,BUILD_TOP_LEVEL,threadIndex,threadCount);      
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
    numNodes = atomicID >> 2;
    DBG(DBG_PRINT(atomicID));
    TIMER(msec = getSeconds()-msec);    
    TIMER(std::cout << "task_buildSubTrees " << 1000. * msec << " ms" << std::endl << std::flush);

    
    /* update BVH4 */
    FATAL("FIX");
    bvh->root = bvh->qbvh[0].lower[0].child; 
    bvh->bounds = BBox3fa(*(Vec3fa*)&bvh->qbvh->lower[0],*(Vec3fa*)&bvh->qbvh->upper[0]);
    
    /* release all threads again */
    LockStepTaskScheduler::releaseThreads(threadCount);

    /* stop measurement */
    if (g_verbose >= 2) 
      dt = getSeconds()-t0;
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

  
  void BVH4HairBuilderConvert::build(size_t threadIndex, size_t threadCount) 
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
