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
    typedef atomic_set<PrimRefBlock> PrimRefList;
    
    /*! stores all info to build a subtree */
    struct BuildTask
    {
      __forceinline BuildTask () {}

      __forceinline BuildTask (BVH4Hair::NodeRef* dst, size_t depth, size_t size, bool makeleaf, atomic_set<PrimRefBlock>& prims, const NAABBox3fa& bounds)
        : dst(dst), depth(depth), size(size), makeleaf(makeleaf), prims(prims), bounds(bounds) {}

    public:
      __forceinline friend bool operator< (const BuildTask& a, const BuildTask& b) {
        return halfArea(a.bounds.bounds) < halfArea(b.bounds.bounds);
      }

    public:
      BVH4Hair::NodeRef* dst;
      size_t depth;
      size_t size;
      bool makeleaf;
      atomic_set<PrimRefBlock> prims;
      NAABBox3fa bounds;
    };
    
    /*! Tries to split hair into two differently aligned hair strands */
    struct StrandSplit
    {
    public:
      StrandSplit () {}

      StrandSplit (const NAABBox3fa& bounds0, const Vec3fa& axis0, const size_t num0,
                   const NAABBox3fa& bounds1, const Vec3fa& axis1, const size_t num1);

      /*! calculates standard surface area heuristic for the split */
      __forceinline float standardSAH() const {
        return BVH4Hair::intCost*float(num0)*halfArea(bounds0.bounds) + BVH4Hair::intCost*float(num1)*halfArea(bounds1.bounds);
      }

      /*! calculates modified surface area heuristic for the split */
      __forceinline float modifiedSAH() const {
        return 
          BVH4Hair::travCostUnaligned*float(num0)*halfArea(bounds0.bounds) + BVH4Hair::intCost*bounds0.bounds.upper.w + 
          BVH4Hair::travCostUnaligned*float(num1)*halfArea(bounds1.bounds) + BVH4Hair::intCost*bounds1.bounds.upper.w;
      }
      
      /*! finds the two hair strands */
      static const StrandSplit find(size_t threadIndex, BVH4HairBuilder2* parent, atomic_set<PrimRefBlock>& curves);
      
      /*! splits hair list into the two strands */
      void split(size_t threadIndex, BVH4HairBuilder2* parent, 
                 atomic_set<PrimRefBlock>& curves, atomic_set<PrimRefBlock>& lcurves_o, atomic_set<PrimRefBlock>& rcurves_o) const;

    public:
      NAABBox3fa bounds0, bounds1;  //!< bounds of the strands
      Vec3fa axis0, axis1;          //!< axis the strands are aligned into
      size_t num0, num1;            //!< number of hairs in the strands
    };

    /*! Performs standard object binning */
    struct ObjectSplit
    {
      /*! number of bins */
      static const size_t BINS = 16;

    public:

      /*! default constructor */
      __forceinline ObjectSplit ()
        : dim(-1), pos(0), cost(inf), num0(0), num1(0), bounds0(inf), bounds1(inf) {}
      
      /*! calculates standard surface area heuristic for the split */
      __forceinline float standardSAH() const {
        return BVH4Hair::intCost*float(num0)*halfArea(bounds0.bounds) + BVH4Hair::intCost*float(num1)*halfArea(bounds1.bounds);
      }

      /*! calculates modified surface area heuristic for the split */
      __forceinline float modifiedSAH() const {
        return 
          BVH4Hair::travCostUnaligned*float(num0)*halfArea(bounds0.bounds) + BVH4Hair::intCost*bounds0.bounds.upper.w + 
          BVH4Hair::travCostUnaligned*float(num1)*halfArea(bounds1.bounds) + BVH4Hair::intCost*bounds1.bounds.upper.w;
      }

      __forceinline bool operator() (const PrimRef& prim) const
      {
        const Vec3fa center = prim.center(space);
        //const ssei bin = clamp(floori((ssef(center) - ofs)*scale),ssei(0),ssei(BINS-1));
        const ssei bin = floori((ssef(center)-ofs)*scale);
        return bin[dim] < pos;
      }

      /*! performs object binning to the the best partitioning */
      static ObjectSplit find(size_t threadIndex, size_t depth, BVH4HairBuilder2* parent, atomic_set<PrimRefBlock>& curves, const LinearSpace3fa& space);

      /*! splits hairs into two sets */
      void split(size_t threadIndex, PrimRefBlockAlloc<PrimRef>& alloc, atomic_set<PrimRefBlock>& curves, atomic_set<PrimRefBlock>& lprims_o, atomic_set<PrimRefBlock>& rprims_o) const;
      
    public:
      LinearSpace3fa space;
      NAABBox3fa bounds0, bounds1;
      int dim;
      int pos;
      float cost;
      size_t num0,num1;
      ssef ofs,scale;
    };

    /*! Performs spatial split in geometry center */
    struct SpatialSplit
    {
      /*! number of bins */
      static const size_t BINS = 16;

    public:

      __forceinline SpatialSplit () {}

      /*! calculates standard surface area heuristic for the split */
      __forceinline float standardSAH() const {
        return BVH4Hair::intCost*float(num0)*halfArea(bounds0.bounds) + BVH4Hair::intCost*float(num1)*halfArea(bounds1.bounds);
        //return cost;
      }

      /*! calculates modified surface area heuristic for the split */
      __forceinline float modifiedSAH() const {
        return 
          BVH4Hair::travCostUnaligned*float(num0)*halfArea(bounds0.bounds) + BVH4Hair::intCost*bounds0.bounds.upper.w + 
          BVH4Hair::travCostUnaligned*float(num1)*halfArea(bounds1.bounds) + BVH4Hair::intCost*bounds1.bounds.upper.w;
      }
      
      /*! finds the two hair strands */
      static const SpatialSplit find(size_t threadIndex, size_t depth, size_t size, BVH4HairBuilder2* parent, atomic_set<PrimRefBlock>& curves, const LinearSpace3fa& space);
      
      /*! splits hair list into the two strands */
      void split(size_t threadIndex, BVH4HairBuilder2* parent, atomic_set<PrimRefBlock>& curves, atomic_set<PrimRefBlock>& lprims_o, atomic_set<PrimRefBlock>& rprims_o) const;

    public:
      LinearSpace3fa space;
      NAABBox3fa bounds0, bounds1;
      float pos;
      int dim;
      float cost;
      size_t numReplications;
      size_t num0, num1;            //!< number of hairs in the strands
      ssef ofs,scale;
    };

    /*! Performs fallback splits */
    struct FallBackSplit
    {
      __forceinline FallBackSplit (const NAABBox3fa& bounds0, size_t num0, const NAABBox3fa& bounds1, size_t num1)
        : bounds0(bounds0), num0(num0), bounds1(bounds1), num1(num1) {}

      /*! finds some partitioning */
      static FallBackSplit find(size_t threadIndex, BVH4HairBuilder2* parent, 
                                atomic_set<PrimRefBlock>& prims, atomic_set<PrimRefBlock>& lprims_o, atomic_set<PrimRefBlock>& rprims_o);

    public:
      size_t num0, num1;
      NAABBox3fa bounds0, bounds1;
    };

  private:

    const BBox3fa subdivideAndAdd(size_t threadIndex, atomic_set<PrimRefBlock>& prims, const Bezier1& bezier, size_t depth);

    void insert(size_t threadIndex, atomic_set<PrimRefBlock>& prims_i, atomic_set<PrimRefBlock>& prims_o);

    template<typename Left>
      void split(size_t threadIndex, atomic_set<PrimRefBlock>& prims, const Left& left, 
                 atomic_set<PrimRefBlock>& lprims_o, size_t& lnum_o, atomic_set<PrimRefBlock>& rprims_o, size_t& rnum_o);

    /*! calculate bounds for range of primitives */
    static const BBox3fa computeAlignedBounds(atomic_set<PrimRefBlock>& curves);

    /*! calculate bounds for range of primitives */
    static const NAABBox3fa computeAlignedBounds(atomic_set<PrimRefBlock>& curves, const LinearSpace3fa& space);
    
    /*! try to find best non-axis aligned space, where the sum of all bounding areas is minimal */
    static const NAABBox3fa computeUnalignedBounds(atomic_set<PrimRefBlock>& curves);

    /*! creates a leaf node */
    BVH4Hair::NodeRef leaf(size_t threadIndex, size_t depth, atomic_set<PrimRefBlock>& prims, const NAABBox3fa& bounds);

    bool split(size_t threadIndex, size_t depth, 
               atomic_set<PrimRefBlock>& prims, const NAABBox3fa& bounds, size_t size,
               atomic_set<PrimRefBlock>& lprims, size_t& lsize,
               atomic_set<PrimRefBlock>& rprims, size_t& rsize,
               bool& isAligned);

    /*! execute single task and create subtasks */
    void processTask(size_t threadIndex, BuildTask& task, BuildTask task_o[BVH4Hair::N], size_t& N);

    /*! recursive build function for aligned and non-aligned bounds */
    void recurseTask(size_t threadIndex, BuildTask& task);

    TASK_RUN_FUNCTION(BVH4HairBuilder2,task_build_parallel);

  public:
    Scene* scene;          //!< source
    size_t minLeafSize;    //!< minimal size of a leaf
    size_t maxLeafSize;    //!< maximal size of a leaf

    size_t numAlignedObjectSplits;
    size_t numAlignedSpatialSplits;
    size_t numUnalignedObjectSplits;
    size_t numUnalignedSpatialSplits;
    size_t numStrandSplits;
    size_t numFallbackSplits;

    bool enableAlignedObjectSplits;
    bool enableAlignedSpatialSplits;
    bool enableUnalignedObjectSplits;
    bool enableUnalignedSpatialSplits;
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
