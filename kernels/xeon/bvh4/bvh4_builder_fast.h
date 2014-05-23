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

#pragma once

#include "bvh4.h"
#include "../bvh4i/bvh4i_builder_util.h"
#include "../bvh4i/bvh4i_builder_binner.h"
#include "../bvh4hair/heuristic_object_partition.h"
#include "builders/workstack.h"

namespace embree
{
  namespace isa
  {
    class BVH4BuilderFast : public Builder
    {
      ALIGNED_CLASS;

    protected:
      typedef BVH4::Node Node;
      typedef BVH4::NodeRef NodeRef;
      typedef GlobalAllocator::ThreadAllocator Allocator;
      static const size_t SIZE_WORK_STACK = 64;

    public:

      class __aligned(64) BuildRecord : public PrimInfo
      {
      public:
	unsigned int depth;         //!< depth from the root of the tree
	float sArea;
	BVH4::NodeRef* parent; 
	
        BuildRecord() {}

	__forceinline void init(unsigned int depth)
	{
	  this->depth = depth;
	  sArea = area(geomBounds);
	}

	__forceinline void init(const CentGeomBBox3fa& _bounds, const unsigned int _begin, const unsigned int _end)
	{
	  geomBounds = _bounds.geomBounds;
	  centBounds = _bounds.centBounds;
	  begin  = _begin;
	  end    = _end;
	  sArea = area(geomBounds);
	}
	
	__forceinline float sceneArea() {
	  return sArea;
	}
	
	__forceinline bool operator<(const BuildRecord &br) const { return size() < br.size(); } 
	__forceinline bool operator>(const BuildRecord &br) const { return size() > br.size(); } 
	
	struct Greater {
	  bool operator()(const BuildRecord& a, const BuildRecord& b) {
	    return a > b;
	  }
	};
      };
      
      struct GlobalState
      {
        ALIGNED_CLASS;
      public:

        GlobalState (size_t numThreads) 
	  : threadStack(new WorkStack<BuildRecord,SIZE_WORK_STACK>[numThreads]) {}
        
        ~GlobalState () {
          delete[] threadStack;
        }

      public:
	WorkHeap<BuildRecord> heap;
        __aligned(64) WorkStack<BuildRecord,SIZE_WORK_STACK>* threadStack;
	ObjectPartition::ParallelBinner parallelBinner;
      };

      static std::auto_ptr<GlobalState> g_state;

    public:

      /*! Constructor. */
      BVH4BuilderFast (BVH4* bvh, Scene* scene, TriangleMesh* mesh, size_t logBlockSize, bool needVertices, size_t primBytes, const size_t minLeafSize, const size_t maxLeafSize);
      
      /*! Destructor */
      ~BVH4BuilderFast ();

      /* initializes the builder */
      void init(size_t threadIndex, size_t threadCount);
      
      /* build function */
      void build(size_t threadIndex, size_t threadCount);

      /* single threaded build */
      void build_sequential(size_t threadIndex, size_t threadCount);
      
    public:
      TASK_RUN_FUNCTION(BVH4BuilderFast,computePrimRefs);
      TASK_RUN_FUNCTION(BVH4BuilderFast,buildSubTrees);
      TASK_RUN_FUNCTION(BVH4BuilderFast,build_parallel);

    public:
      
      /*! build mode */
      enum { RECURSE_SEQUENTIAL = 1, RECURSE_PARALLEL = 2, BUILD_TOP_LEVEL = 3 };
      
      /*! splitting function that selects between sequential and parallel mode */
      void split(BuildRecord& current, BuildRecord& left, BuildRecord& right, const size_t mode, const size_t threadID, const size_t numThreads);
      
      /*! perform sequential binning and splitting */
      void splitSequential(BuildRecord& current, BuildRecord& leftChild, BuildRecord& rightChild, const size_t threadID, const size_t numThreads);
      
      /*! perform parallel binning and splitting */
      void splitParallel(BuildRecord& current, BuildRecord& leftChild, BuildRecord& rightChild, const size_t threadID, const size_t threads);
      
      /*! creates a small leaf node */
      virtual void createSmallLeaf(const BVH4BuilderFast* This, BuildRecord& current, Allocator& leafAlloc, size_t threadID) = 0;

      /*! creates a large leaf node */
      void createLeaf(BuildRecord& current, Allocator& nodeAlloc, Allocator& leafAlloc, size_t threadIndex, size_t threadCount);
      
      /*! select between recursion and stack operations */
      void recurse_continue(BuildRecord& current, Allocator& nodeAlloc, Allocator& leafAlloc, const size_t mode, const size_t threadID, const size_t numThreads);
      
      /*! recursive build function */
      void recurse(BuildRecord& current, Allocator& nodeAlloc, Allocator& leafAlloc, const size_t mode, const size_t threadID, const size_t numThreads);
      
      static void splitFallback(PrimRef * __restrict__ const primref, BuildRecord& current, BuildRecord& leftChild, BuildRecord& rightChild);
    
    public:
      Scene* scene;                            //!< input scene
      TriangleMesh* mesh;   //!< input mesh
      BVH4* bvh;                               //!< Output BVH
      size_t logBlockSize;
      size_t blocks(size_t N) { return (N+((1<<logBlockSize)-1)) >> logBlockSize; }
      bool needVertices;
      size_t primBytes; 
      size_t minLeafSize;
      size_t maxLeafSize;

    protected:
      TaskScheduler::Task task;
      
    public:
      PrimRef* prims;
      size_t bytesPrims;
      
    protected:
      size_t numPrimitives;
      
    protected:
      __aligned(64) GlobalAllocator nodeAllocator;
      __aligned(64) GlobalAllocator primAllocator;
    };

    class BVH4Triangle1BuilderFast : public BVH4BuilderFast
    {
    public:
      BVH4Triangle1BuilderFast (BVH4* bvh, Scene* scene);
      BVH4Triangle1BuilderFast (BVH4* bvh, TriangleMesh* mesh);
      void createSmallLeaf(const BVH4BuilderFast* This, BuildRecord& current, Allocator& leafAlloc, size_t threadID);
    };

    class BVH4Triangle4BuilderFast : public BVH4BuilderFast
    {
    public:
      BVH4Triangle4BuilderFast (BVH4* bvh, Scene* scene);
      BVH4Triangle4BuilderFast (BVH4* bvh, TriangleMesh* mesh);
      void createSmallLeaf(const BVH4BuilderFast* This, BuildRecord& current, Allocator& leafAlloc, size_t threadID);
    };

    class BVH4Triangle8BuilderFast : public BVH4BuilderFast
    {
    public:
      BVH4Triangle8BuilderFast (BVH4* bvh, Scene* scene);
      BVH4Triangle8BuilderFast (BVH4* bvh, TriangleMesh* mesh);
      void createSmallLeaf(const BVH4BuilderFast* This, BuildRecord& current, Allocator& leafAlloc, size_t threadID);
    };

    class BVH4Triangle1vBuilderFast : public BVH4BuilderFast
    {
    public:
      BVH4Triangle1vBuilderFast (BVH4* bvh, Scene* scene);
      BVH4Triangle1vBuilderFast (BVH4* bvh, TriangleMesh* mesh);
      void createSmallLeaf(const BVH4BuilderFast* This, BuildRecord& current, Allocator& leafAlloc, size_t threadID);
    };

    class BVH4Triangle4vBuilderFast : public BVH4BuilderFast
    {
    public:
      BVH4Triangle4vBuilderFast (BVH4* bvh, Scene* scene);
      BVH4Triangle4vBuilderFast (BVH4* bvh, TriangleMesh* mesh);
      void createSmallLeaf(const BVH4BuilderFast* This, BuildRecord& current, Allocator& leafAlloc, size_t threadID);
    };

    class BVH4Triangle4iBuilderFast : public BVH4BuilderFast
    {
    public:
      BVH4Triangle4iBuilderFast (BVH4* bvh, Scene* scene);
      BVH4Triangle4iBuilderFast (BVH4* bvh, TriangleMesh* mesh);
      void createSmallLeaf(const BVH4BuilderFast* This, BuildRecord& current, Allocator& leafAlloc, size_t threadID);
    };
  }
}
