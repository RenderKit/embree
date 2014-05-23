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

#include "builders/primrefalloc.h"
#include "builders/primrefblock.h"
#include "geometry/primitive.h"
#include "bvh4hair/heuristic_split.h"
#include "builders/trirefgen.h"

namespace embree
{
  namespace isa
  {
    /* BVH builder. The builder is multi-threaded and implements 3
     * different build strategies: 1) Small tasks are finished in a
     * single thread (BuildTask) 2) Medium sized tasks are split into
     * two tasks using a single thread (SplitTask) and 3) Large tasks are
     * split using multiple threads on one processor. */
    
    class BVH4Builder2 : public Builder
    {
      ALIGNED_CLASS;
    public:
      
      /*! Type shortcuts */
      typedef typename BVH4::Node    Node;
      typedef typename BVH4::NodeRef NodeRef;
      
      /*! Split type of the split heuristic. */
      //typedef ObjectPartition::Split Split;
      typedef atomic_set<PrimRefBlockT<PrimRef> > TriRefList;

      struct BuildRecord 
      {
      public:
	__forceinline BuildRecord () {}

	__forceinline BuildRecord (size_t depth) : depth(depth), pinfo(empty) {}

	__forceinline BuildRecord (size_t depth, TriRefList& prims, const PrimInfo& pinfo, const Split& split, NodeRef* dst)
	: depth(depth), prims(prims), pinfo(pinfo), split(split), dst(dst) {}
	
      public:
	NodeRef*   dst;      //!< Reference to output the node.
	size_t     depth;    //!< Recursion depth of the root of this subtree.
	TriRefList prims;    //!< The list of primitives.
	PrimInfo   pinfo;    //!< Bounding info of primitives.
	Split      split;    //!< The best split for the primitives.
      };
      
    public:
      
      /*! builder entry point */
      void build(size_t threadIndex, size_t threadCount);
      
      /*! Constructor. */
      BVH4Builder2 (BVH4* bvh, Scene* scene, TriangleMesh* mesh, size_t logBlockSize, bool needVertices, size_t primBytes, const size_t minLeafSize, const size_t maxLeafSize);

      /*! build job */
      TASK_RUN_FUNCTION_(BVH4Builder2,buildFunction);
      void buildFunction(size_t threadIndex, size_t threadCount, size_t taskIndex, size_t taskCount, TaskScheduler::Event* event);
      
      /*! creates a leaf node */
      virtual NodeRef createLeaf(size_t threadIndex, TriRefList& prims, const PrimInfo& pinfo) = 0;
      
      /*! creates a large leaf by adding additional internal nodes */
      NodeRef createLargeLeaf(size_t threadIndex, TriRefList& prims, const PrimInfo& pinfo, size_t depth);
      
      /*! Selects between full build and single-threaded split strategy. */
      template<bool PARALLEL>
      void recurse(size_t threadIndex, size_t threadCount, TaskScheduler::Event* event, 
		   NodeRef& node, size_t depth, TriRefList& prims, const PrimInfo& pinfo, const Split& split);

      NodeRef layout_top_nodes(size_t threadIndex, NodeRef node);

      template<bool PARALLEL>
      const Split find(size_t threadIndex, size_t threadCount, size_t depth, TriRefList& prims, const PrimInfo& pinfo, bool spatial);

      size_t recurse(size_t threadIndex, size_t threadCount, BVH4Builder2* parent, BuildRecord& record, BuildRecord records_o[BVH4::N]);

      /***********************************************************************************************************************
       *                                      Single Threaded Build Task
       **********************************************************************************************************************/
      
      /*! Single-threaded task that builds a complete BVH4. */
      class BuildTask {
	ALIGNED_CLASS
	  public:
	
	/*! Default task construction. */
	BuildTask(size_t threadIndex, size_t threadCount, TaskScheduler::Event* event, 
		  BVH4Builder2* parent, NodeRef& node, size_t depth, 
		  TriRefList& prims, const PrimInfo& pinfo, const Split& split);
	
	/*! Task entry function. */
	TASK_COMPLETE_FUNCTION_(BuildTask,run);
	void run(size_t threadIndex, size_t threadCount, TaskScheduler::Event* event);
	
	/*! Recursively finishes the BVH4 construction. */
	void recurse(BVH4Builder2* parent, BuildRecord& record);
	
      private:
	size_t threadIndex;
	size_t threadCount;
	TaskScheduler::Task task;
	
	BVH4Builder2*                     parent;   //!< Pointer to parent task.
	BuildRecord record;
      };
      
      /***********************************************************************************************************************
       *                                      Single Threaded Split Task
       **********************************************************************************************************************/
      
      /*! Single-threaded task that builds a single node and creates subtasks for the children. */
      class SplitTask {
	ALIGNED_CLASS
	  public:
	
	/*! Default task construction. */
	SplitTask(size_t threadIndex, size_t threadCount, TaskScheduler::Event* event, 
		  BVH4Builder2* parent, NodeRef& node, size_t depth, 
		  TriRefList& prims, const PrimInfo& pinfo, const Split& split);
	
	/*! Default task construction. */
	SplitTask(size_t threadIndex, size_t threadCount, 
		  BVH4Builder2* parent, NodeRef& node, size_t depth, 
		  TriRefList& prims, const PrimInfo& pinfo, const Split& split);
	
	/*! Task entry function. */
	TASK_COMPLETE_FUNCTION_(SplitTask,run);
	void run(size_t threadIndex, size_t threadCount, TaskScheduler::Event* event);
	
	template<bool PARALLEL>
	void recurse(size_t threadIndex, size_t threadCount, TaskScheduler::Event* event);
	
	__forceinline friend bool operator< (const SplitTask& a, const SplitTask& b) {
	  //return halfArea(a.bounds.bounds) < halfArea(b.bounds.bounds);
	  return a.pinfo.size() < b.pinfo.size();
	}
	
      public:
	TaskScheduler::Task task;
	BVH4Builder2*    parent;         //!< Pointer to parent task.
	NodeRef*    dst;            //!< Reference to output the node.
	size_t          depth;          //!< Recursion depth of this node.
	TriRefList prims; //!< The list of primitives.
	PrimInfo        pinfo;          //!< Bounding info of primitives.
	Split           split;          //!< The best split for the primitives.
      };

    protected:
      //BuildSource* source;      //!< build source interface
      Scene* scene;           //!< input geometry
      TriangleMesh* mesh;

    public:
      const PrimitiveType& primTy;          //!< triangle type stored in BVH4
      size_t minLeafSize;                 //!< minimal size of a leaf
      size_t maxLeafSize;                 //!< maximal size of a leaf
      PrimRefBlockAlloc<PrimRef> alloc;                 //!< Allocator for primitive blocks
      //TriRefGen initStage;               //!< job to generate build primitives
      TaskScheduler::QUEUE taskQueue;     //!< Task queue to use
      
      std::vector<SplitTask> tasks;
      atomic_t remainingReplications;
      
      size_t logBlockSize;
      size_t blocks(size_t N) { return (N+((1<<logBlockSize)-1)) >> logBlockSize; }
      bool needVertices;
      size_t primBytes; 

    public:
      BVH4* bvh;                      //!< Output BVH4
    };

    template<typename Triangle>
    class BVH4Builder2T : public BVH4Builder2
    {
    public:
      BVH4Builder2T (BVH4* bvh, Scene* scene);
      BVH4Builder2T (BVH4* bvh, TriangleMesh* mesh);
      NodeRef createLeaf(size_t threadIndex, TriRefList& prims, const PrimInfo& pinfo);
    };
  }
}
