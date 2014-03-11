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

    static __forceinline float countfunc(size_t N) {
      //return sqrtf(float(N));
      return float(N);
    }

  private:

    typedef Bezier1 PrimRef;

    /*! Block of build primitives */
    class PrimRefBlock
    {
    public:

      typedef PrimRef T;

      /*! Number of primitive references inside a block */
      static const size_t blockSize = 511;

      /*! default constructor */
      PrimRefBlock () : num(0) {}

      /*! frees the block */
      __forceinline void clear(size_t n = 0) { num = n; }
      
      /*! return base pointer */
      __forceinline PrimRef* base() { return ptr; }

      /*! returns number of elements */
      __forceinline size_t size() const { return num; }

      /*! inserts a primitive reference */
      __forceinline bool insert(const PrimRef& ref) {
        if (unlikely(num >= blockSize)) return false;
        ptr[num++] = ref;
        return true;
      }

      /*! access the i-th primitive reference */
      __forceinline       PrimRef& operator[] (size_t i)       { return ptr[i]; }
      __forceinline const PrimRef& operator[] (size_t i) const { return ptr[i]; }
    
      /*! access the i-th primitive reference */
      __forceinline       PrimRef& at (size_t i)       { return ptr[i]; }
      __forceinline const PrimRef& at (size_t i) const { return ptr[i]; }
    
    private:
      PrimRef ptr[blockSize];   //!< Block with primitive references
      size_t num;               //!< Number of primitive references in block
    };

    class PrimRefBlockAlloc : public AllocatorBase
    {
      ALIGNED_CLASS;
    public:
   
      struct __aligned(4096) ThreadPrimBlockAllocator 
      {
        ALIGNED_CLASS_(4096);
      public:
      
        __forceinline atomic_set<PrimRefBlock>::item* malloc(size_t thread, AllocatorBase* alloc) 
        {
          /* try to take a block from local list */
          atomic_set<PrimRefBlock>::item* ptr = local_free_blocks.take_unsafe();
          if (ptr) return new (ptr) atomic_set<PrimRefBlock>::item();
          
          /* if this failed again we have to allocate more memory */
          ptr = (atomic_set<PrimRefBlock>::item*) alloc->malloc(sizeof(atomic_set<PrimRefBlock>::item));
          
          /* return first block */
          return new (ptr) atomic_set<PrimRefBlock>::item();
        }
      
        __forceinline void free(atomic_set<PrimRefBlock>::item* ptr) {
          local_free_blocks.insert_unsafe(ptr);
        }
        
      public:
        atomic_set<PrimRefBlock> local_free_blocks; //!< only accessed from one thread
      };
      
    public:
      
      /*! Allocator default construction. */
      PrimRefBlockAlloc () {
        threadPrimBlockAllocator = new ThreadPrimBlockAllocator[getNumberOfLogicalThreads()];
      }
      
      /*! Allocator destructor. */
      virtual ~PrimRefBlockAlloc() {
        delete[] threadPrimBlockAllocator; threadPrimBlockAllocator = NULL;
      }
      
      /*! Allocate a primitive block */
      __forceinline atomic_set<PrimRefBlock>::item* malloc(size_t thread) {
        return threadPrimBlockAllocator[thread].malloc(thread,this);
      }
      
      /*! Frees a primitive block */
      __forceinline void free(size_t thread, atomic_set<PrimRefBlock>::item* block) {
        return threadPrimBlockAllocator[thread].free(block);
      }
      
    private:
      ThreadPrimBlockAllocator* threadPrimBlockAllocator;  //!< Thread local allocator
    };

    /*! stores all info to build a subtree */
    struct BuildTask
    {
      __forceinline BuildTask () {}

      __forceinline BuildTask (BVH4Hair::NodeRef* dst, size_t depth, size_t size, bool makeleaf, atomic_set<PrimRefBlock>& prims, const NAABBox3fa& bounds)
        : dst(dst), depth(depth), size(size), makeleaf(makeleaf), prims(prims), bounds(bounds) {}

    public:
      __forceinline friend bool operator< (const BuildTask& a, const BuildTask& b) {
        return area(a.bounds.bounds) < area(b.bounds.bounds);
        //return area(a.bounds.bounds)/double(a.size) < area(b.bounds.bounds)/double(b.size);
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
        return BVH4Hair::intCost*countfunc(num0)*embree::area(bounds0.bounds) + BVH4Hair::intCost*countfunc(num1)*embree::area(bounds1.bounds);
      }

      /*! calculates modified surface area heuristic for the split */
      __forceinline float modifiedSAH() const {
        return 
          BVH4Hair::travCostUnaligned*countfunc(num0)*embree::area(bounds0.bounds) + BVH4Hair::intCost*bounds0.bounds.upper.w + 
          BVH4Hair::travCostUnaligned*countfunc(num1)*embree::area(bounds1.bounds) + BVH4Hair::intCost*bounds1.bounds.upper.w;
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
        return BVH4Hair::intCost*countfunc(num0)*embree::area(bounds0.bounds) + BVH4Hair::intCost*countfunc(num1)*embree::area(bounds1.bounds);
        //return cost;
      }

      /*! calculates modified surface area heuristic for the split */
      __forceinline float modifiedSAH() const {
        return 
          BVH4Hair::travCostUnaligned*countfunc(num0)*embree::area(bounds0.bounds) + BVH4Hair::intCost*bounds0.bounds.upper.w + 
          BVH4Hair::travCostUnaligned*countfunc(num1)*embree::area(bounds1.bounds) + BVH4Hair::intCost*bounds1.bounds.upper.w;
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

      /*! calculates aligned bounds for left and right split */
      const ObjectSplit alignedBounds(size_t threadIndex, size_t depth, BVH4HairBuilder2* parent, atomic_set<PrimRefBlock>& curves);

      /*! calculates the bounds for left and right split */
      const ObjectSplit unalignedBounds(size_t threadIndex, size_t depth, BVH4HairBuilder2* parent, atomic_set<PrimRefBlock>& curves);

      /*! splits hairs into two sets */
      void split(size_t threadIndex, BVH4HairBuilder2* parent, atomic_set<PrimRefBlock>& curves, atomic_set<PrimRefBlock>& lprims_o, atomic_set<PrimRefBlock>& rprims_o) const;
      
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
        return BVH4Hair::intCost*countfunc(num0)*embree::area(bounds0.bounds) + BVH4Hair::intCost*countfunc(num1)*embree::area(bounds1.bounds);
        //return cost;
      }

      /*! calculates modified surface area heuristic for the split */
      __forceinline float modifiedSAH() const {
        return 
          BVH4Hair::travCostUnaligned*countfunc(num0)*embree::area(bounds0.bounds) + BVH4Hair::intCost*bounds0.bounds.upper.w + 
          BVH4Hair::travCostUnaligned*countfunc(num1)*embree::area(bounds1.bounds) + BVH4Hair::intCost*bounds1.bounds.upper.w;
      }
      
      /*! finds the two hair strands */
      static const SpatialSplit find(size_t threadIndex, size_t depth, BVH4HairBuilder2* parent, atomic_set<PrimRefBlock>& curves, const LinearSpace3fa& space);
      
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
    size_t numAlignedSubdivObjectSplits;
    size_t numUnalignedObjectSplits;
    size_t numUnalignedSpatialSplits;
    size_t numUnalignedSubdivObjectSplits;
    size_t numStrandSplits;
    size_t numFallbackSplits;

    bool enableAlignedObjectSplits;
    bool enableAlignedSpatialSplits;
    bool enableAlignedSubdivObjectSplits;
    bool enableUnalignedObjectSplits;
    bool enableUnalignedSpatialSplits;
    bool enableUnalignedSubdivObjectSplits;
    bool enableStrandSplits;
    bool enablePresplit3;

    BVH4Hair* bvh;         //!< output
    PrimRefBlockAlloc alloc;                 //!< Allocator for primitive blocks

    MutexSys taskMutex;
    volatile atomic_t numActiveTasks;
    volatile atomic_t numGeneratedPrims;
    volatile atomic_t remainingReplications;
    std::vector<BuildTask> tasks;
  };
}
