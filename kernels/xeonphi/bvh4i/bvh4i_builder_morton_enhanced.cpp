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

#include "kernels/xeonphi/bvh4i/bvh4i.h"
#include "kernels/xeonphi/bvh4i/bvh4i_builder_morton_enhanced.h"
#include "kernels/xeonphi/bvh4i/bvh4i_builder_binner.h"
#include "kernels/xeonphi/bvh4i/bvh4i_statistics.h"
#include "kernels/xeonphi/bvh4i/bvh4i_builder_util_mic.h"

#include "limits.h"


#define TREE_BRANCHING_FACTOR 4
#define QBVH_BUILDER_LEAF_ITEM_THRESHOLD 4
#define SINGLE_THREADED_BUILD_THRESHOLD  (MAX_MIC_THREADS*8)

#define DIAG_FACTOR 0.2f
#define MAX_REBUILD_NODES 1024*2
#define PROFILE_ITERATIONS 20

#define TIMER(x) x

//#define PROFILE
//#define TEST_BUILD_PERFORMANCE

#define DBG(x) 

namespace embree 
{
  __align(64) static double dt = 0.0f;

  BVH4iBuilderMortonEnhanced::BVH4iBuilderMortonEnhanced (BVH4i* _bvh, BuildSource* _source, void* _geometry, const size_t _minLeafSize, const size_t _maxLeafSize)
    : BVH4iBuilderMorton(_bvh,_source,_geometry,_minLeafSize,_maxLeafSize){}

  void BVH4iBuilderMortonEnhanced::build(size_t threadIndex, size_t threadCount) 
  {
    if (g_verbose >= 2) {
      std::cout << "building BVH4i with Enhanced Morton builder (MIC)... " << std::endl << std::flush;
    }

    initEncodingAllocateData(threadCount);
    LockStepTaskScheduler::init(TaskScheduler::getNumThreads()); 

#if defined(PROFILE)
    std::cout << "STARTING PROFILE MODE" << std::endl << std::flush;

    double dt_min = pos_inf;
    double dt_avg = 0.0f;
    double dt_max = neg_inf;
    size_t iterations = PROFILE_ITERATIONS;
    for (size_t i=0; i<iterations; i++) 
      {
	TaskScheduler::executeTask(threadIndex,threadCount,_build_parallel_morton_enhanced,this,TaskScheduler::getNumThreads(),"build_parallel_morton_enhanced");

	dt_min = min(dt_min,dt);
	dt_avg = dt_avg + dt;
	dt_max = max(dt_max,dt);
      }
    dt_avg /= double(iterations);

    std::cout << "[DONE]" << std::endl;
    std::cout << "  min = " << 1000.0f*dt_min << "ms (" << source->size()/dt_min*1E-6 << " Mtris/s)" << std::endl;
    std::cout << "  avg = " << 1000.0f*dt_avg << "ms (" << source->size()/dt_avg*1E-6 << " Mtris/s)" << std::endl;
    std::cout << "  max = " << 1000.0f*dt_max << "ms (" << source->size()/dt_max*1E-6 << " Mtris/s)" << std::endl;
    std::cout << BVH4iStatistics(bvh).str();

#else
    DBG(DBG_PRINT(numPrimitives));


    if (likely(numPrimitives > SINGLE_THREADED_BUILD_THRESHOLD && TaskScheduler::getNumThreads() > 1))
      {
	DBG(std::cout << "PARALLEL BUILD" << std::endl << std::flush);
	TaskScheduler::executeTask(threadIndex,threadCount,_build_parallel_morton_enhanced,this,TaskScheduler::getNumThreads(),"build_parallel");
      }
    else
      {
	/* number of primitives is small, just use single threaded mode */
	if (likely(numPrimitives > 0))
	  {
	    DBG(std::cout << "SERIAL BUILD" << std::endl << std::flush);
	    build_parallel_morton_enhanced(0,1,0,0,NULL);
	  }
	else
	  {
	    DBG(std::cout << "EMPTY SCENE BUILD" << std::endl << std::flush);
	    /* handle empty scene */
	    for (size_t i=0;i<4;i++)
	      bvh->qbvh[0].setInvalid(i);
	    for (size_t i=0;i<4;i++)
	      bvh->qbvh[1].setInvalid(i);
	    bvh->qbvh[0].lower[0].child = BVH4i::NodeRef(128);
	    bvh->root = bvh->qbvh[0].lower[0].child; 
	    bvh->bounds = BBox3f(*(Vec3fa*)&bvh->qbvh->lower[0],*(Vec3fa*)&bvh->qbvh->upper[0]);	    
	  }
      }

    if (g_verbose >= 2) {
      double perf = source->size()/dt*1E-6;
      std::cout << "[DONE] " << 1000.0f*dt << "ms (" << perf << " Mtris/s), primitives " << numPrimitives << std::endl;
      std::cout << BVH4iStatistics(bvh).str();
    }
#endif

    // TaskScheduler::executeTask(threadIndex,threadCount,_build_parallel_morton_enhanced,this,TaskScheduler::getNumThreads(),"build_parallel_morton_enhanced");
  }

  bool splitSAH(PrimRef * __restrict__ const primref, BuildRecord& current, BuildRecord& leftChild, BuildRecord& rightChild)
  {
    /* mark as leaf if leaf threshold reached */
    const unsigned int items = current.end - current.begin;
    if (items <= QBVH_BUILDER_LEAF_ITEM_THRESHOLD) {
      current.createLeaf();
      return false;
    }
    
    /* calculate binning function */
    Mapping mapping(current.bounds);

    /* binning of centroids */
    Binner<16> binner;
    binner.bin(primref,current.begin,current.end,mapping);

    /* find best split */
    Split split; 
    binner.best(split,mapping);

    /* if we cannot find a valid split, enforce an arbitrary split */
    if (unlikely(split.pos == -1)) 
      return split_fallback(primref,current,leftChild,rightChild);

    /* partitioning of items */
    binner.partition(primref, current.begin, current.end, split, mapping, leftChild, rightChild);
    if (leftChild.items()  <= QBVH_BUILDER_LEAF_ITEM_THRESHOLD) leftChild.createLeaf();
    if (rightChild.items() <= QBVH_BUILDER_LEAF_ITEM_THRESHOLD) rightChild.createLeaf();	
    return true;
  }



  void BVH4iBuilderMortonEnhanced::extractTopLevelTree(const size_t index,
						       const Vec3fa &root_diag,
						       BVHNode *__restrict__ const local_node,
						       size_t &nodes)
  {
    BVHNode &entry = this->node[index];

    if (!entry.isLeaf())
      {
	const unsigned int children = entry.firstChildID();

	const Vec3fa diag = entry.upper - entry.lower;
	if (diag.x < DIAG_FACTOR * root_diag.x &&
	    diag.y < DIAG_FACTOR * root_diag.y &&
	    diag.z < DIAG_FACTOR * root_diag.z)
	  {
	    if (unlikely(nodes >= MAX_REBUILD_NODES)) FATAL("too many subtrees");
	    //subTreeIDs[subTrees++] = index;	      	      
	    local_node[nodes++] = entry;
	    assert(nodes < MAX_REBUILD_NODES);
	    return;
	  }
	else
	  {
	    for (unsigned int i=0;i<entry.items();i++) 
	      {
		const unsigned int childIndex = children + i;	    
		extractTopLevelTree(childIndex,root_diag,local_node,nodes);	  
	      }
	  }
      }
    else
      {
	if (unlikely(nodes >= MAX_REBUILD_NODES)) FATAL("too many subtrees");
	//subTreeIDs[subTrees++] = index;	      	      
	local_node[nodes++] = entry;
	assert(nodes < MAX_REBUILD_NODES);
      }     
  }


  void BVH4iBuilderMortonEnhanced::buildTopLevelSAHTree(BVHNode &parent,
							BuildRecord &current,
							BVHNode *__restrict__ local_node)
  {
    DBG(std::cout << std::endl; PING);
    DBG(DBG_PRINT(parent));
    DBG(DBG_PRINT(current));
#ifdef DEBUG
    {
      PrimRef tmp;
      tmp.lower = current.bounds.geometry.lower;//storeSceneAABB((float*)&tmp);
      tmp.upper = current.bounds.geometry.upper;
      for (size_t i=0;i<current.items();i++)
	assert(subset(*(PrimRef*)&local_node[current.begin+i],tmp) == true);
      // for (size_t i=0;i<current.items();i++)
      //   cout << i << " " << local_node[current.begin+i] << endl;
    }
#endif      

    BuildRecord record[TREE_BRANCHING_FACTOR];
    BuildRecord left, right;

    unsigned int numSplits = 1;
    record[0] = current;

    while(1)
      {
	bool couldSplit = false;
	if (numSplits < TREE_BRANCHING_FACTOR)
	  {
	    int index = -1;
	    float maxArea = -1.0f;
	    for (unsigned int i=0;i<numSplits;i++)
	      {
		if (!record[i].isLeaf())
		  {
		    assert(record[i].sceneArea() > 0.0f);

		    if (record[i].sceneArea() >= maxArea)
		      {
			maxArea = record[i].sceneArea();
			index = i;
		      }
		  }
	      }

	    assert(index < TREE_BRANCHING_FACTOR);
	    if (index != -1)
	      {
		bool s = splitSAH((PrimRef*)local_node,record[index],left,right);  

		assert(numSplits > 0);

		if ( s )
		  {
		    record[index] = record[numSplits-1];
		    numSplits--;

		    record[numSplits+0] = left;
		    record[numSplits+1] = right;
		    numSplits+=2;

		    couldSplit = true;
		  }
		else 
		  {// became leaf
		    DBG(std::cout << "became leaf" << std::endl);
		    continue;
		  }
	      }
	  }
	// ==================================================
	if (!couldSplit) break;
      }

    DBG(std::cout << "DETERMINED ALL SPLITS" << std::endl);


    DBG(DBG_PRINT(numSplits));

    // could not generate any split, propagate leaf back to parent
    if ( numSplits == 1 )
      {
	current = record[0];
	DBG(std::cout << "CREATING LOCAL NODE LEAF WITH " << current.items() << " NODES" << std::endl);
	assert(current.isLeaf());
	if(current.items() > QBVH_BUILDER_LEAF_ITEM_THRESHOLD) 
	  {
	    std::cout << "WARNING UNSPLITABLE LEAF " << std::endl;
	    FATAL("SHOULD NOT HAPPEN FOR TOP-LEVEL TREE");
	    DBG_PRINT(current.items());
	    record[1] = record[0];
	    record[0].end   = record[0].begin + 4;
	    record[1].begin = record[0].begin + 4;
	    numSplits = 2;
	  }
	else
	  {
	    if (unlikely(current.items() == 1))
	      {
		parent = local_node[current.begin];
		return;
	      }
	    assert(current.items() > 1);

	    const unsigned int currentIndex = this->atomicID.add(TREE_BRANCHING_FACTOR);
	    if (unlikely(currentIndex >= this->numAllocatedNodes))
	      {
		DBG_PRINT(this->numAllocatedNodes);
		FATAL("not enough nodes allocated");
	      }
      
	    /* init used/unused nodes */
	    const mic_f init_node = load16f((float*)BVH4i::initQBVHNode);
	    store16f((float*)&node[currentIndex+0],init_node);
	    store16f((float*)&node[currentIndex+2],init_node);

	    DBG(DBG_PRINT(currentIndex));

	    parent.createNode(currentIndex,current.items());
	    //current.childrenID = currentIndex;

	    for (size_t i=0;i<current.items();i++)
	      {
		assert(subset(*(PrimRef*)&local_node[current.begin+i],*(PrimRef*)&parent));
		this->node[currentIndex+i] = local_node[current.begin+i];
	      }

	    DBG(DBG_PRINT(parent));
	    return;
	  }
      }

    assert(numSplits >= 2 && numSplits <= TREE_BRANCHING_FACTOR);

    // ==== aquire next four nodes ====
    const unsigned int currentIndex = this->atomicID.add(TREE_BRANCHING_FACTOR);
    if (unlikely(currentIndex >= this->numAllocatedNodes))
      {
	DBG_PRINT(this->numAllocatedNodes);
	FATAL("not enough nodes allocated");
      }

    const mic_f init_node = load16f((float*)BVH4i::initQBVHNode);
    store16f((float*)&node[currentIndex+0],init_node);
    store16f((float*)&node[currentIndex+2],init_node);

    parent.createNode(currentIndex,numSplits);
    //current.childrenID = currentIndex;
    //assert(current.childrenID > 0);


    //node[current.parentID].createNode(current.childrenID,numSplits);


    for (unsigned int i=0;i<numSplits;i++)
      {
	DBG(DBG_PRINT(currentIndex+i));
	//record[i].bounds.storeSceneAABB((float*)&this->node[currentIndex+i]);
        this->node[currentIndex+i].lower = record[i].bounds.geometry.lower;
        this->node[currentIndex+i].upper = record[i].bounds.geometry.upper;
	buildTopLevelSAHTree(this->node[currentIndex+i],record[i],local_node);
      }

  }


  void BVH4iBuilderMortonEnhanced::build_parallel_morton_enhanced(size_t threadIndex, size_t threadCount, size_t taskIndex, size_t taskCount, TaskScheduler::Event* event) 
  {
    /* initialize thread state */
    initThreadState(threadIndex,threadCount);
    
    /* let all thread except for control thread wait for work */
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

    // ===================          
    // === start timer ===
    // ===================
    
    TIMER(double msec);
    TIMER(msec = getSeconds());

    build_main(threadIndex,taskCount);

    bvh->accel = this->accel;
    bvh->qbvh  = (BVH4i::Node*)this->node;

    TIMER(msec = getSeconds()-msec);    
    TIMER(std::cout << "build morton tree " << 1000. * msec << " ms" << std::endl << std::flush);

      // ==============================        
      // === rebuild top level tree ===
      // ==============================

    TIMER(msec = getSeconds());

    const Vec3fa rootDiag = this->node[0].upper - this->node[0].lower;

    __align(64) BVHNode local_node[MAX_REBUILD_NODES];
    size_t nodes = 0;
    extractTopLevelTree(0,rootDiag,local_node,nodes);

    TIMER(msec = getSeconds()-msec);    
    TIMER(std::cout << "extract top-level nodes " << 1000. * msec << " ms" << std::endl << std::flush);


    TIMER(msec = getSeconds());    
    BuildRecord topLevelBuildRecord;
    topLevelBuildRecord.init(this->global_bounds,0,nodes);
    buildTopLevelSAHTree(this->node[0],topLevelBuildRecord,local_node);

    TIMER(msec = getSeconds()-msec);    
    TIMER(std::cout << "rebuild top-level " << 1000. * msec << " ms" << std::endl << std::flush);
	    
    // ===================================        
    // === convert to optimized layout ===
    // ===================================

    this->numNodes = this->atomicID >> 2;

    TIMER(msec = getSeconds());    

    LockStepTaskScheduler::dispatchTask( task_convertToSOALayout, this, threadIndex, threadCount );

    TIMER(msec = getSeconds()-msec);    
    TIMER(std::cout << "task_convertToSOALayout " << 1000. * msec << " ms" << std::endl << std::flush);


    bvh->root = bvh->qbvh[0].lower[0].child; 
    bvh->bounds = BBox3f(*(Vec3fa*)&bvh->qbvh->lower[0],*(Vec3fa*)&bvh->qbvh->upper[0]);
    
    // ==================          
    // === stop timer ===
    // ==================

    if (g_verbose >= 2) {
      double t1 = getSeconds();
      double perf = source->size()/(t1-t0)*1E-6;
      std::cout << "[DONE] " << t1-t0 << "sec (" << perf << " Mtris/s)" << std::endl;
      //
    }

    LockStepTaskScheduler::releaseThreads(threadCount);

    /* stop measurement */
#if !defined(PROFILE)
    if (g_verbose >= 2) 
#endif
      dt = getSeconds()-t0;

  };

};
