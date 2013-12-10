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
    BVH4iBuilder (BVH4i* bvh, BuildSource* source, void* geometry, bool enablePreSplits = false);

    ~BVH4iBuilder();

    /*! creates the builder */
    static Builder* create (void* accel, BuildSource* source, void* geometry, bool enablePreSplits = false) { 
      return new BVH4iBuilder((BVH4i*)accel,source,geometry,enablePreSplits);
    }

    /* build function */
    void build(size_t threadIndex, size_t threadCount);

    void allocateData(size_t threadCount);

  public:
    TASK_FUNCTION(BVH4iBuilder,computePrimRefs);
    TASK_FUNCTION(BVH4iBuilder,computePrimRefsVirtual);
    TASK_FUNCTION(BVH4iBuilder,computePrimRefsPreSplits);
    TASK_FUNCTION(BVH4iBuilder,fillLocalWorkQueues);
    TASK_FUNCTION(BVH4iBuilder,buildSubTrees);
    TASK_FUNCTION(BVH4iBuilder,createTriangle1);
    TASK_FUNCTION(BVH4iBuilder,convertToSOALayout);
    TASK_FUNCTION(BVH4iBuilder,parallelBinningGlobal);
    TASK_FUNCTION(BVH4iBuilder,parallelPartitioningGlobal);
    TASK_RUN_FUNCTION(BVH4iBuilder,build_parallel);
    LOCAL_TASK_FUNCTION(BVH4iBuilder,parallelBinningLocal);
    LOCAL_TASK_FUNCTION(BVH4iBuilder,parallelPartitioningLocal);

  public:

    /*! build mode */
    enum { RECURSE = 1, FILL_LOCAL_QUEUES = 2, BUILD_TOP_LEVEL = 3 };

    /*! splitting function that selects between sequential and parallel mode */
    bool split(BuildRecord& current, BuildRecord& left, BuildRecord& right, const size_t mode, const size_t threadID, const size_t numThreads);

    /*! perform sequential binning and splitting */
    bool splitSequential(BuildRecord& current, BuildRecord& leftChild, BuildRecord& rightChild);

    /*! perform parallel splitting */
    void parallelPartitioning(BuildRecord& current,
			      PrimRef * __restrict__ l_source,
			      PrimRef * __restrict__ r_source,
			      PrimRef * __restrict__ l_dest,
			      PrimRef * __restrict__ r_dest,
			      const Split &split,
			      Centroid_Scene_AABB &local_left,
			      Centroid_Scene_AABB &local_right);			      
			      
    /*! perform parallel binning and splitting using all threads on all cores*/
    bool splitParallelGlobal(BuildRecord& current, BuildRecord& leftChild, BuildRecord& rightChild, const size_t threadID, const size_t threads);

    /*! perform parallel binning and splitting using only the threads per core*/
    bool splitParallelLocal(BuildRecord& current, BuildRecord& leftChild, BuildRecord& rightChild, const size_t threadID);
    
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
    const bool enablePreSplits;

    /* work record handling */
  protected:
    static const size_t SIZE_GLOBAL_WORK_STACK = 512;
    static const size_t SIZE_LOCAL_WORK_STACK  = 16;

    __align(64) WorkStack<BuildRecord,SIZE_GLOBAL_WORK_STACK> global_workStack;
    __align(64) WorkStack<BuildRecord,SIZE_LOCAL_WORK_STACK> local_workStack[MAX_MIC_CORES];

  public:
    struct __align(64) SharedBinningPartitionData
    {
      __align(64) BuildRecord rec;
      __align(64) Centroid_Scene_AABB left;
      __align(64) Centroid_Scene_AABB right;
      __align(64) Split split;
      __align(64) AlignedAtomicCounter32 lCounter;
      __align(64) AlignedAtomicCounter32 rCounter;
    };

    __align(64) SharedBinningPartitionData global_sharedData;
    __align(64) Bin16 global_bin16[MAX_MIC_THREADS];
    __align(64) SharedBinningPartitionData local_sharedData[MAX_MIC_CORES];

  protected:
    PrimRef*   prims;
    BVHNode*   node;
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


    __align(64) LockStepTaskScheduler4ThreadsLocalCore localTaskScheduler[MAX_MIC_CORES];

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
