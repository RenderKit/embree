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

namespace embree
{
  namespace isa
  {
    class BVH4BuilderFast : public Builder
    {
      ALIGNED_CLASS;
      typedef BVH4::Node Node;
      typedef BVH4::NodeRef NodeRef;
      typedef GlobalAllocator::ThreadAllocator Allocator;
      static const size_t SIZE_WORK_STACK = 64;

    public:

      class __aligned(64) BuildRecord : public PrimInfo
      {
      public:
	//Centroid_Scene_AABB bounds; //!< geometry and centroid bounds
	//unsigned int begin;         //!< start of range
	//unsigned int end;           //!< end of range
	unsigned int parentID;      //!< the ID of the node that points to us
	unsigned int depth;         //!< depth from the root of the tree
	
	unsigned int flags;
	float sArea;
	size_t parentNode; 
	
      BuildRecord() : PrimInfo(0)
	{
	  assert(sizeof(BuildRecord) == 128);
	}
	
	__forceinline void init(const unsigned int _begin, const unsigned int _end)
	{
	  begin  = _begin;
	  end    = _end;
	  parentID = (unsigned int)-1;
	  sArea = area(geomBounds);
	  flags = BUILD_RECORD_NODE;
	}
	
	__forceinline void init(const Centroid_Scene_AABB& _bounds, const unsigned int _begin, const unsigned int _end)
	{
	  //bounds = _bounds;
	  geomBounds = _bounds.geometry;
	  centBounds = _bounds.centroid2;
	  init(_begin,_end);
	}
	
	__forceinline unsigned int items() const {
	  return end - begin;
	}
	
	__forceinline float sceneArea() {
	  return sArea;
	}
	
	__forceinline bool operator<(const BuildRecord &br) const { return items() < br.items(); } 
	__forceinline bool operator>(const BuildRecord &br) const { return items() > br.items(); } 
	
	__forceinline friend std::ostream &operator<<(std::ostream &o, const BuildRecord &br)
	{
	  o << "centroid2 = " << br.centBounds << " ";
	  o << "geometry = " << br.geomBounds << " ";
	  o << "begin      " << br.begin << " ";
	  o << "end        " << br.end << " ";
	  o << "items      " << br.end-br.begin << " ";
	  o << "parentID     " << br.parentID << " ";
	  o << "flags      " << br.flags << " ";
	  o << "sArea      " << br.sArea << " ";
	  return o;
	};
	
	/* FIXME: this can be removed */
	enum { BUILD_RECORD_INIT  = 0, BUILD_RECORD_NODE  = 1, BUILD_RECORD_LEAF  = 2 };
	__forceinline void createNode() { flags = BUILD_RECORD_NODE; }
	__forceinline void createLeaf() { flags = BUILD_RECORD_LEAF; }
	__forceinline bool isLeaf() { return flags == BUILD_RECORD_LEAF; }
      };
      
      struct GlobalState
      {
        ALIGNED_CLASS;

      public:

        GlobalState (size_t numThreads) {
          threadStack = new WorkStack<BuildRecord,SIZE_WORK_STACK>[numThreads];
        }
        
        ~GlobalState () {
          delete[] threadStack;
        }

      public:
        __aligned(64) WorkStack<BuildRecord,SIZE_WORK_STACK> workStack;
        __aligned(64) WorkStack<BuildRecord,SIZE_WORK_STACK>* threadStack;
        //ParallelBinner<16> parallelBinner;    
	ObjectPartition::ParallelBinner parallelBinner;
        LinearBarrierActive barrier;
      };

      static std::auto_ptr<GlobalState> g_state;

    public:
      
      /*! Constructor. */
      BVH4BuilderFast (BVH4* bvh, BuildSource* source, Scene* scene, TriangleMesh* mesh, const size_t minLeafSize = 1, const size_t maxLeafSize = inf);
      
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
      bool split(BuildRecord& current, BuildRecord& left, BuildRecord& right, const size_t mode, const size_t threadID, const size_t numThreads);
      
      /*! perform sequential binning and splitting */
      bool splitSequential(BuildRecord& current, BuildRecord& leftChild, BuildRecord& rightChild);
      
      /*! perform parallel binning and splitting */
      bool splitParallel(BuildRecord& current, BuildRecord& leftChild, BuildRecord& rightChild, const size_t threadID, const size_t threads);
      
      /*! creates a small leaf node */
      typedef void (*createLeafFunction)(const BVH4BuilderFast* This, BuildRecord& current, Allocator& leafAlloc, size_t threadID);
      static void createTriangle1Leaf(const BVH4BuilderFast* This, BuildRecord& current, Allocator& leafAlloc, size_t threadID);
      static void createTriangle4Leaf(const BVH4BuilderFast* This, BuildRecord& current, Allocator& leafAlloc, size_t threadID);
      static void createTriangle1vLeaf(const BVH4BuilderFast* This, BuildRecord& current, Allocator& leafAlloc, size_t threadID);
      static void createTriangle4vLeaf(const BVH4BuilderFast* This, BuildRecord& current, Allocator& leafAlloc, size_t threadID);
      createLeafFunction createSmallLeaf;
      
      /*! creates a large leaf node */
      void createLeaf(BuildRecord& current, Allocator& nodeAlloc, Allocator& leafAlloc, size_t threadIndex, size_t threadCount);
      
      /*! select between recursion and stack operations */
      void recurse(BuildRecord& current, Allocator& nodeAlloc, Allocator& leafAlloc, const size_t mode, const size_t threadID, const size_t numThreads);
      
      /*! recursive build function */
      void recurseSAH(BuildRecord& current, Allocator& nodeAlloc, Allocator& leafAlloc, const size_t mode, const size_t threadID, const size_t numThreads);
      
      static bool split_fallback(PrimRef * __restrict__ const primref, BuildRecord& current, BuildRecord& leftChild, BuildRecord& rightChild);
    
    protected:
      BuildSource* source;                     //!< input geometry
      Scene* scene;                            //!< input scene
      TriangleMesh* mesh;   //!< input mesh
      BVH4* bvh;                               //!< Output BVH
      const PrimitiveType& primTy;             //!< triangle type stored in BVH
      
    protected:
      TaskScheduler::Task task;
      
    protected:
      PrimRef* prims;
      size_t bytesPrims;
      
    protected:
      size_t numGroups;
      size_t numPrimitives;
      Centroid_Scene_AABB global_bounds;
      
    protected:
      __aligned(64) GlobalAllocator nodeAllocator;
      __aligned(64) GlobalAllocator primAllocator;
    };
  }
}
