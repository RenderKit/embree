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
      typedef atomic_set<PrimRefBlockT<PrimRef> > TriRefList;

      /*! the build record stores all information to continue building some subtree */
      struct BuildRecord 
      {
      public:
	__forceinline BuildRecord () {}

	__forceinline BuildRecord (size_t depth) : depth(depth), pinfo(empty) {}

	__forceinline BuildRecord (size_t depth, TriRefList& prims, const PrimInfo& pinfo, const Split& split, NodeRef* dst)
	: depth(depth), prims(prims), pinfo(pinfo), split(split), dst(dst) {}
	
	__forceinline friend bool operator< (const BuildRecord& a, const BuildRecord& b) {
	  return a.pinfo.size() < b.pinfo.size();
	}
	
      public:
	NodeRef*   dst;      //!< Reference to output the node.
	size_t     depth;    //!< Recursion depth of the root of this subtree.
	TriRefList prims;    //!< The list of primitives.
	PrimInfo   pinfo;    //!< Bounding info of primitives.
	Split      split;    //!< The best split for the primitives.
      };
      
    public:
      
      /*! Constructor. */
      BVH4Builder2 (BVH4* bvh, Scene* scene, TriangleMesh* mesh, size_t mode, size_t logBlockSize, size_t logSAHBlockSize, float intCost, bool needVertices, size_t primBytes, const size_t minLeafSize, const size_t maxLeafSize);

      /*! builder entry point */
      void build(size_t threadIndex, size_t threadCount);
   
      /*! build job */
      TASK_RUN_FUNCTION_(BVH4Builder2,build_parallel);
      void build_parallel(size_t threadIndex, size_t threadCount, size_t taskIndex, size_t taskCount, TaskScheduler::Event* event);
      
      /*! creates a leaf node */
      virtual NodeRef createLeaf(size_t threadIndex, TriRefList& prims, const PrimInfo& pinfo) = 0;
      
      /*! creates a large leaf by adding additional internal nodes */
      NodeRef createLargeLeaf(size_t threadIndex, TriRefList& prims, const PrimInfo& pinfo, size_t depth);
      
      /*! copies topmost nodes to improve memory layout */
      NodeRef layout_top_nodes(size_t threadIndex, NodeRef node);

      /*! finds best possible split for a list of triangles */
      template<bool PARALLEL>
      const Split find(size_t threadIndex, size_t threadCount, size_t depth, TriRefList& prims, const PrimInfo& pinfo, bool spatial);

      /*! creates a node from some build record */
      template<bool PARALLEL>
      static size_t createNode(size_t threadIndex, size_t threadCount, BVH4Builder2* parent, BuildRecord& record, BuildRecord records_o[BVH4::N]);

      /*! continues build */
      void continue_build(size_t threadIndex, size_t threadCount, BuildRecord& record);

      /*! recursively finishes build */
      void finish_build(size_t threadIndex, size_t threadCount, BuildRecord& record);

    protected:
      Scene* scene;           //!< input geometry
      TriangleMesh* mesh;     //!< input triangle mesh

    public:
      size_t minLeafSize;                 //!< minimal size of a leaf
      size_t maxLeafSize;                 //!< maximal size of a leaf
      PrimRefBlockAlloc<PrimRef> alloc;                 //!< Allocator for primitive blocks
      
      volatile atomic_t activeBuildRecords;
      MutexSys taskMutex;
      std::vector<BuildRecord> tasks;
      atomic_t remainingReplications;

      bool enableSpatialSplits;
      size_t intCost;
      size_t logSAHBlockSize;
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
      BVH4Builder2T (BVH4* bvh, Scene* scene, size_t mode);
      BVH4Builder2T (BVH4* bvh, TriangleMesh* mesh, size_t mode);
      NodeRef createLeaf(size_t threadIndex, TriRefList& prims, const PrimInfo& pinfo);
    };
  }
}
