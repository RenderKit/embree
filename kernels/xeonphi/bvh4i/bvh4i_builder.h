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

#ifndef __EMBREE_BVHI_BUILDER_MIC_H__
#define __EMBREE_BVHI_BUILDER_MIC_H__


#include "common/registry_builder.h"

#include "kernels/xeonphi/bvh4i/bvh4i.h"
#include "kernels/xeonphi/bvh4i/bvh4i_builder_binner.h"
#include "kernels/xeonphi/bvh4i/bvh4i_builder_util_mic.h"
#include "kernels/xeon/bvh4i/bvh4i_builder_util.h"

namespace embree
{
  class BVH4iBuilder : public Builder
  {
    ALIGNED_CLASS;
    static const size_t ALLOCATOR_NODE_BLOCK_SIZE = 64;
    typedef AtomicIDBlock<ALLOCATOR_NODE_BLOCK_SIZE> NodeAllocator;

  private:
    
    void checkBuildRecord(const BuildRecord &current);

  public:

    /*! Constructor. */
    BVH4iBuilder (BVH4i* bvh, BuildSource* source, void* geometry, const size_t minLeafSize = 1, const size_t maxLeafSize = inf);

    ~BVH4iBuilder();

    /*! creates the builder */
    static Builder* create (void* accel, BuildSource* source, void* geometry, const size_t minLeafSize = inf, const size_t maxLeafSize = inf) { 
      return new BVH4iBuilder((BVH4i*)accel,source,geometry,minLeafSize,maxLeafSize);
    }

    /* build function */
    void build(size_t threadIndex, size_t threadCount);

    void allocateData(size_t threadCount);

  public:
    TASK_FUNCTION(BVH4iBuilder,computePrimRefs);
    TASK_FUNCTION(BVH4iBuilder,fillLocalWorkQueues);
    TASK_FUNCTION(BVH4iBuilder,buildSubTrees);
    TASK_FUNCTION(BVH4iBuilder,createTriangle1);
    TASK_FUNCTION(BVH4iBuilder,convertToSOALayout);
    TASK_FUNCTION(BVH4iBuilder,parallelBinning);
    TASK_FUNCTION(BVH4iBuilder,parallelPartition);
    TASK_RUN_FUNCTION(BVH4iBuilder,build_parallel);

  public:

    /*! build mode */
    enum { RECURSE = 1, FILL_LOCAL_QUEUES = 2, BUILD_TOP_LEVEL = 3 };

    /*! splitting function that selects between sequential and parallel mode */
    bool split(BuildRecord& current, BuildRecord& left, BuildRecord& right, const size_t mode, const size_t threadID, const size_t numThreads);

    /*! perform sequential binning and splitting */
    bool splitSequential(BuildRecord& current, BuildRecord& leftChild, BuildRecord& rightChild);

    /*! perform parallel binning and splitting */
    bool splitParallel(BuildRecord& current, BuildRecord& leftChild, BuildRecord& rightChild, const size_t threadID, const size_t threads);
    
    /*! creates a leaf node */
    void createLeaf(BuildRecord& current, NodeAllocator& alloc, const size_t threadIndex, const size_t threadCount);

    /*! select between recursion and stack operations */
    void recurse(BuildRecord& current, NodeAllocator& alloc, const size_t mode, const size_t threadID, const size_t numThreads);
    
    /*! recursive build function */
    void recurseSAH(BuildRecord& current, NodeAllocator& alloc, const size_t mode, const size_t threadID, const size_t numThreads);

  protected:
    BuildSource* source;          //!< input geometry
    void* geometry;               //!< input geometry
    BVH4i* bvh;                   //!< Output BVH

    /* work record handling */
  protected:
    static const size_t SIZE_GLOBAL_WORK_STACK = 512;
    static const size_t SIZE_LOCAL_WORK_STACK  = 16;

    __align(64) WorkStack<BuildRecord,SIZE_GLOBAL_WORK_STACK> global_workStack;
    __align(64) WorkStack<BuildRecord,SIZE_LOCAL_WORK_STACK> local_workStack[MAX_MIC_CORES];
    TaskScheduler::Task task;

    __align(64) LinearBarrierActive global_barrier;
    ParallelBinner<16> parallelBinner;  

  public:
    struct __align(64) SharedBinningPartitionData
    {
      __align(64) BuildRecord rec;
      __align(64) Centroid_Scene_AABB left;
      __align(64) Centroid_Scene_AABB right;
      __align(64) Split split;
      __align(64) AlignedAtomicCounter32 lCounter;
      __align(64) AlignedAtomicCounter32 rCounter;
      __align(64) Bin16 bin16[MAX_MIC_THREADS];
    };

    __align(64) SharedBinningPartitionData sharedData;

  protected:
    PrimRef* prims;
    BVHNode* node;
    Triangle1* accel;

  protected:
    size_t numPrimitives;
    size_t numNodes;
    size_t numAllocatedNodes;
    size_t size_prims;

    /*! bounds shared among threads */    
    Centroid_Scene_AABB global_bounds;

    /*! node allocator */
    AlignedAtomicCounter32  atomicID;

    __forceinline unsigned int allocNode(int size)
    {
      const unsigned int currentIndex = atomicID.add(size);
      if (unlikely(currentIndex >= numAllocatedNodes)) {
        FATAL("not enough nodes allocated");
      }
      return currentIndex;
    }
  };
}

#endif
