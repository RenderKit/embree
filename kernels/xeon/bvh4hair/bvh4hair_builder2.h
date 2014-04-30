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

#include "geometry/primitive.h"
#include "geometry/bezier1.h"
#include "builders/primrefalloc.h"
#include "heuristic_split.h"

namespace embree
{
  class BVH4HairBuilder2 : public Builder
  {
    ALIGNED_CLASS;
  public:

    /*! builder entry point */
    void build(size_t threadIndex, size_t threadCount);

    /*! Constructor. */
    BVH4HairBuilder2 (BVH4Hair* bvh, Scene* scene);

  private:

    typedef Bezier1 PrimRef;
    typedef PrimRefBlockT<Bezier1> PrimRefBlock;
    typedef atomic_set<PrimRefBlock> BezierRefList;
    
    /*! stores all info to build a subtree */
    struct BuildTask
    {
      __forceinline BuildTask () {}

      __forceinline BuildTask (BVH4Hair::NodeRef* dst, size_t depth, BezierRefList& prims, const PrimInfo& pinfo, const NAABBox3fa& bounds, const PrimInfo& sinfo, const Split& split)
        : dst(dst), depth(depth), prims(prims), pinfo(pinfo), bounds(bounds), sinfo(sinfo), split(split) {}

    public:
      __forceinline friend bool operator< (const BuildTask& a, const BuildTask& b) {
        //return halfArea(a.bounds.bounds) < halfArea(b.bounds.bounds);
	return a.pinfo.size() < b.pinfo.size();
      }

    public:
      BVH4Hair::NodeRef* dst;
      size_t depth;
      BezierRefList prims;
      PrimInfo pinfo;
      NAABBox3fa bounds;
      PrimInfo sinfo;
      Split split;
    };
        
  private:

    /*! creates a leaf node */
    BVH4Hair::NodeRef leaf(size_t threadIndex, size_t depth, BezierRefList& prims, const PrimInfo& pinfo);

    template<bool Parallel = false>
      Split find_split(size_t threadIndex, size_t threadCount, BezierRefList& prims, const PrimInfo& pinfo, const NAABBox3fa& bounds, const PrimInfo& sinfo);

    /*! execute single task and create subtasks */
    template<bool Parallel = false>
      void processTask(size_t threadIndex, size_t threadCount, BuildTask& task, BuildTask task_o[BVH4Hair::N], size_t& N);
   
    /*! recursive build function for aligned and non-aligned bounds */
    void recurseTask(size_t threadIndex, size_t threadCount, BuildTask& task);

    TASK_RUN_FUNCTION(BVH4HairBuilder2,task_build_parallel);

  public:
    Scene* scene;          //!< source
    size_t minLeafSize;    //!< minimal size of a leaf
    size_t maxLeafSize;    //!< maximal size of a leaf

    BVH4Hair* bvh;         //!< output
    PrimRefBlockAlloc<PrimRef> alloc;                 //!< Allocator for primitive blocks

    MutexSys taskMutex;
    volatile atomic_t numActiveTasks;
    volatile atomic_t numGeneratedPrims;
    volatile atomic_t remainingReplications;
    std::vector<BuildTask> tasks;
  };
}
