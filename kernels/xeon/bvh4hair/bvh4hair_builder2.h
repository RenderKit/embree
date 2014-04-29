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

      __forceinline BuildTask (BVH4Hair::NodeRef* dst, size_t depth, BezierRefList& prims, const PrimInfo& pinfo, const NAABBox3fa& bounds, const Split& split)
        : dst(dst), depth(depth), prims(prims), pinfo(pinfo), bounds(bounds), split(split) {}

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
      Split split;
    };
        
  private:

    const BBox3fa subdivideAndAdd(size_t threadIndex, BezierRefList& prims, const Bezier1& bezier, size_t depth);

    /*! try to find best non-axis aligned space, where the sum of all bounding areas is minimal */
    static const NAABBox3fa computeHairSpaceBounds(BezierRefList& curves);

    /*! creates a leaf node */
    BVH4Hair::NodeRef leaf(size_t threadIndex, size_t depth, BezierRefList& prims, const NAABBox3fa& bounds);

    Split find_split(size_t threadIndex, size_t threadCount, BezierRefList& prims, const PrimInfo& pinfo, const NAABBox3fa& bounds);

    BVH4Hair::NodeRef recurse(size_t threadIndex, size_t threadCount, size_t depth, BezierRefList& prims, const PrimInfo& pinfo, const NAABBox3fa& bounds, const Split& split);

    /*! execute single task and create subtasks */
    void processTask(size_t threadIndex, size_t threadCount, BuildTask& task, BuildTask task_o[BVH4Hair::N], size_t& N);
   
    /*! recursive build function for aligned and non-aligned bounds */
    void recurseTask(size_t threadIndex, size_t threadCount, BuildTask& task);

    TASK_RUN_FUNCTION(BVH4HairBuilder2,task_build_parallel);

  public:
    Scene* scene;          //!< source
    size_t minLeafSize;    //!< minimal size of a leaf
    size_t maxLeafSize;    //!< maximal size of a leaf

    bool enableAlignedObjectSplits;
    bool enableAlignedSpatialSplits;
    bool enableUnalignedObjectSplits;
    bool enableStrandSplits;
    int enablePreSubdivision;

    BVH4Hair* bvh;         //!< output
    PrimRefBlockAlloc<PrimRef> alloc;                 //!< Allocator for primitive blocks

    MutexSys taskMutex;
    volatile atomic_t numActiveTasks;
    volatile atomic_t numGeneratedPrims;
    volatile atomic_t remainingReplications;
    std::vector<BuildTask> tasks;
  };
}
