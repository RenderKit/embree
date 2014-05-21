#include "bvh4hair/bvh4hair_builder.h"
#include "geometry/bezier1i.h"

namespace embree
{
#define DBG(x) 

#define L1_PREFETCH_ITEMS 2
#define L2_PREFETCH_ITEMS 16
#define SINGLE_THREADED_BUILD_THRESHOLD        512

  static double dt = 0.0f;

  // ==========================================================================================
  // ==========================================================================================
  // ==========================================================================================


  void BVH4HairBuilder::printBuilderName()
  {
    std::cout << "building BVH4Hair with binned SAH builder (MIC) ... " << std::endl;    
    DBG(sleep(1));
  }

  size_t BVH4HairBuilder::getNumPrimitives()
  {
    DBG(PING);
    DBG(sleep(1));

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

  void BVH4HairBuilder::computePrimRefs(const size_t threadIndex, const size_t threadCount)
  {
    DBG(PING);
    DBG(sleep(1));
    LockStepTaskScheduler::dispatchTask( task_computePrimRefsBezierCurves, this, threadIndex, threadCount );	
  }

  void BVH4HairBuilder::createAccel(const size_t threadIndex, const size_t threadCount)
  {
    DBG(PING);
    DBG(sleep(1));
    LockStepTaskScheduler::dispatchTask( task_createBezierCurvesAccel, this, threadIndex, threadCount );
  }

  void BVH4HairBuilder::computePrimRefsBezierCurves(const size_t threadID, const size_t numThreads) 
  {
    DBG(PING);
    DBG(sleep(1));

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
          const BBox3fa bounds = geom->bounds(i);
          const mic_f bmin = broadcast4to16f(&bounds.lower); 
          const mic_f bmax = broadcast4to16f(&bounds.upper);
          
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


  void BVH4HairBuilder::createBezierCurvesAccel(const size_t threadID, const size_t numThreads)
  {
    DBG(PING);

    const size_t startID = (threadID+0)*numPrimitives/numThreads;
    const size_t endID   = (threadID+1)*numPrimitives/numThreads;

    Bezier1i *acc = (Bezier1i*)accel + startID;

    const PrimRef* __restrict__  bptr = prims + startID;

    for (size_t j=startID; j<endID; j++, bptr++, acc++)
      {
     	prefetch<PFHINT_NT>(bptr + L1_PREFETCH_ITEMS);
     	prefetch<PFHINT_L2>(bptr + L2_PREFETCH_ITEMS);
     	assert(bptr->geomID() < scene->size() );
	
	const unsigned int geomID = bptr->geomID();
	const unsigned int primID = bptr->primID();
	BezierCurves* geom = (BezierCurves*) scene->getBezierCurves(geomID);

     	*acc = Bezier1i( &geom->vertex( geom->curve( primID ) ), geomID, primID, geom->mask );
      }
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

	TaskScheduler::executeTask(threadIndex,threadCount,_build_parallel,this,TaskScheduler::getNumThreads(),"build_parallel");

#if 1
	std::cout << "converting from bvh4i to bvh4hair..." << std::endl;
	DBG_PRINT(numNodes);
	bvh4hair->unaligned_nodes = (BVH4Hair::UnalignedNode*)os_malloc(sizeof(BVH4Hair::UnalignedNode) * numNodes);
	for (size_t i=0;i<numNodes;i++)
	  bvh4hair->unaligned_nodes[i].convertFromBVH4iNode(bvh4hair->qbvh[i],bvh4hair->unaligned_nodes);
#endif
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

    if (g_verbose >= 2) {
      double perf = totalNumPrimitives/dt*1E-6;
      std::cout << "[DONE] " << 1000.0f*dt << "ms (" << perf << " Mtris/s), primitives " << numPrimitives << std::endl;
      std::cout << BVH4iStatistics(bvh).str();
    }

  }

};
