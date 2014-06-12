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

#pragma once

#include "builders/parallel_builder.h"
#include "builders/builder_util.h"
#include "builders/binning.h"

#include "bvh4i/bvh4i_builder.h"
#include "bvh4hair.h"
#include "geometry/bezier1i.h"


#define BVH_NODE_PREALLOC_FACTOR                 1.15f

namespace embree
{

  /*! derived binned-SAH builder supporting hair primitives */  
  class BVH4HairBuilder : public ParallelBinnedSAHBuilder
  {
    ALIGNED_CLASS;
    
  protected:
    static const size_t ALLOCATOR_NODE_BLOCK_SIZE = 64;
    typedef AtomicIDBlock<ALLOCATOR_NODE_BLOCK_SIZE> NodeAllocator;    

    static const size_t MAX_ITEMS_PER_LEAF = 2;

  public:

    class __aligned(64) BuildRecordOBB : public BuildRecord
    {
    public:
      BuildRecordOBB() {}

      BuildRecordOBB(const BuildRecord &b) 
	{
	  *(BuildRecord*)this = b;
	  xfm = LinearSpace3fa( zero );
	}
      __aligned(64) LinearSpace3fa xfm;


      __forceinline friend std::ostream &operator<<(std::ostream &o, const BuildRecordOBB &br)
	{
	  o << "centroid2 = " << br.bounds.centroid2 << " ";
	  o << "geometry  = " << br.bounds.geometry << " ";
	  o << "begin       " << br.begin << " ";
	  o << "end         " << br.end << " ";
	  o << "items       " << br.end-br.begin << " ";
	  //o << "parentID    " << br.parentID << " ";
	  o << "parentPtr   " << br.parentPtr << " ";
	  o << "flags       " << br.flags << " ";
	  o << "sArea       " << br.sArea << " ";
	  o << "matrix      " << br.xfm << " ";
	  return o;
	};

    };

    BVH4Hair *bvh4hair;
    Bezier1i *prims;
    Bezier1i *accel;
    BVH4Hair::UnalignedNode*   node;
    //BVH4Hair::AlignedNode*   node;
    
    size_t size_prims;
    size_t size_accel;
    size_t size_nodes;
    
  BVH4HairBuilder(BVH4Hair* bvh, void* geometry) 
    : ParallelBinnedSAHBuilder(geometry),
      bvh4hair(bvh),
      prims(NULL),
      node(NULL),
      accel(NULL),
      size_prims(0),
      size_nodes(0),
      size_accel(0)
      {
      }

    virtual ~BVH4HairBuilder() 
      {	
      }

    virtual size_t getNumPrimitives();
    virtual void   printBuilderName();
    virtual void   allocateData   (const size_t threadCount, const size_t newNumPrimitives);
    virtual void   computePrimRefs(const size_t threadIndex, const size_t threadCount);
    virtual void   build          (const size_t threadIndex, const size_t threadCount);
    virtual void   createAccel    (const size_t threadIndex, const size_t threadCount);

    virtual void buildSubTree(BuildRecord& current, 
			      NodeAllocator& alloc, 
			      const size_t mode,
			      const size_t threadID, 
			      const size_t numThreads);

    void build_parallel(size_t threadIndex, size_t threadCount, size_t taskIndex, size_t taskCount, TaskScheduler::Event* event);

  protected:

    __forceinline void createLeaf(void *parentPtr,
				  unsigned int offset, 
				  unsigned int items)
      {
	assert(items <= BVH4Hair::N);
	const unsigned int v = (offset << BVH4Hair::encodingBits) | BVH4Hair::leaf_mask | items;
	size_t *ptr = (size_t*)parentPtr;
	*ptr = (size_t)v;
      }

    __forceinline void createNode(void *parentPtr,
				  void *childPtr,
				  const size_t flags = 0)
    {
      size_t *ptr = (size_t*)parentPtr;
      *ptr = (size_t)childPtr | flags;      
    }


    /*! splitting function that selects between sequential and parallel mode */
    bool split(BuildRecord& current, BuildRecord& left, BuildRecord& right, const size_t mode, const size_t threadID, const size_t numThreads);

    /*! perform sequential binning and splitting */
    bool splitSequential(BuildRecord& current, BuildRecord& leftChild, BuildRecord& rightChild);

    /*! perform parallel binning and splitting using all threads on all cores*/
    bool splitParallelGlobal(BuildRecord& current, BuildRecord& leftChild, BuildRecord& rightChild, const size_t threadID, const size_t threads);

    /*! perform parallel binning and splitting using only the threads per core*/
    bool splitParallelLocal(BuildRecord& current, BuildRecord& leftChild, BuildRecord& rightChild, const size_t threadID);

    /*! perform parallel splitting */
    void parallelPartitioning(BuildRecord& current,
			      Bezier1i * __restrict__ l_source,
			      Bezier1i * __restrict__ r_source,
			      Bezier1i * __restrict__ l_dest,
			      Bezier1i * __restrict__ r_dest,
			      const Split &split,
			      Centroid_Scene_AABB &local_left,
			      Centroid_Scene_AABB &local_right);			      

    void allocateMemoryPools(const size_t numPrims, const size_t numNodes);

    /*! recursive build functions */
    void recurseSAH(BuildRecord& current, NodeAllocator& alloc, const size_t mode, const size_t threadID, const size_t numThreads);
    void createLeaf(BuildRecord& current, NodeAllocator& alloc,const size_t threadIndex, const size_t threadCount);
    void recurse(BuildRecord& current, NodeAllocator& alloc,const size_t mode, const size_t threadID, const size_t numThreads);

    /*! unaligned splits */
    void recurseOBB(BuildRecordOBB& current, NodeAllocator& alloc, const size_t mode, const size_t threadID, const size_t numThreads);

    /*! perform sequential binning and splitting */
    bool splitSequentialOBB(BuildRecordOBB& current, BuildRecordOBB& leftChild, BuildRecordOBB& rightChild);

    void computeUnalignedSpace( BuildRecordOBB& current );
    void computeUnalignedSpaceBounds( BuildRecordOBB& current );

    TASK_RUN_FUNCTION(BVH4HairBuilder,build_parallel_hair);
    TASK_FUNCTION(BVH4HairBuilder,computePrimRefsBezierCurves);
    TASK_FUNCTION(BVH4HairBuilder,parallelBinningGlobal);
    TASK_FUNCTION(BVH4HairBuilder,parallelPartitioningGlobal);
    LOCAL_TASK_FUNCTION(BVH4HairBuilder,parallelBinningLocal);
    LOCAL_TASK_FUNCTION(BVH4HairBuilder,parallelPartitioningLocal);

  };



}
