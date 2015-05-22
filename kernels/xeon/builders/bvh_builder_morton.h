// ======================================================================== //
// Copyright 2009-2015 Intel Corporation                                    //
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

#include "../../common/builder.h"
#include "../../algorithms/parallel_reduce.h"

namespace embree
{
  namespace isa
  {
    template<typename NodeRef>
      class MortonBuildRecord 
    {
    public:
      unsigned int begin;
      unsigned int end;
      unsigned int depth;
      NodeRef* parent;

      __forceinline MortonBuildRecord() {}

      __forceinline MortonBuildRecord(const unsigned begin, const unsigned end, NodeRef* parent, unsigned depth)
        : begin(begin), end(end), parent(parent), depth(depth) {}

      __forceinline unsigned int size() const {
        return end - begin;
      }
      
      __forceinline void init(const unsigned int _begin, const unsigned int _end)			 
      {
        begin = _begin;
        end = _end;
        depth = 1;
        parent = nullptr;
      }
    };
    
    struct __aligned(8) MortonID32Bit
    {
    public:
      unsigned int code;
      unsigned int index;
      
    public:   
      __forceinline operator unsigned() const { return code; }
      
      __forceinline unsigned int get(const unsigned int shift, const unsigned and_mask) const {
        return (code >> shift) & and_mask;
      }
      
      __forceinline bool operator<(const MortonID32Bit &m) const { return code < m.code; } 
      
      __forceinline friend std::ostream &operator<<(std::ostream &o, const MortonID32Bit& mc) {
        o << "index " << mc.index << " code = " << mc.code;
        return o;
      }
    };
    
    struct MortonCodeGenerator
    {
      static const size_t LATTICE_BITS_PER_DIM = 10;
      static const size_t LATTICE_SIZE_PER_DIM = size_t(1) << LATTICE_BITS_PER_DIM;
      
      struct MortonCodeMapping
      {
        float4 base;
        float4 scale;
        
        __forceinline MortonCodeMapping(const BBox3fa& bounds)
        {
          base  = (float4)bounds.lower;
          const float4 diag  = (float4)bounds.upper - (float4)bounds.lower;
          scale = select(diag > float4(1E-19f), rcp(diag) * float4(LATTICE_SIZE_PER_DIM * 0.99f),float4(0.0f));
        }
      };
      
      __forceinline MortonCodeGenerator(const BBox3fa& bounds, MortonID32Bit* dest)
        : mapping(bounds), dest(dest), currentID(0), slots(0), ax(0), ay(0), az(0), ai(0) {}
      
      __forceinline MortonCodeGenerator(const MortonCodeMapping& mapping, MortonID32Bit* dest)
        : mapping(mapping), dest(dest), currentID(0), slots(0), ax(0), ay(0), az(0), ai(0) {}
      
      __forceinline ~MortonCodeGenerator()
      {
        if (slots != 0)
        {
          const int4 code = bitInterleave(ax,ay,az);
          for (size_t i=0; i<slots; i++) {
            dest[currentID-slots+i].index = ai[i];
            dest[currentID-slots+i].code = code[i];
          }
        }
      }
      
      __forceinline void operator() (const BBox3fa& b, const size_t index)
      {
        const float4 lower = (float4)b.lower;
        const float4 upper = (float4)b.upper;
        const float4 centroid = lower+upper;
        const int4 binID = int4((centroid-mapping.base)*mapping.scale);
        
        ax[slots] = extract<0>(binID);
        ay[slots] = extract<1>(binID);
        az[slots] = extract<2>(binID);
        ai[slots] = index;
        slots++;
        currentID++;
        
        if (slots == 4)
        {
          const int4 code = bitInterleave(ax,ay,az);
          storeu4i(&dest[currentID-4],unpacklo(code,ai));
          storeu4i(&dest[currentID-2],unpackhi(code,ai));
          slots = 0;
        }
      }
      
    public:
      const MortonCodeMapping mapping;
      MortonID32Bit* dest;
      const float4 base;
      const float4 scale;
      size_t currentID;
      size_t slots;
      int4 ax, ay, az, ai;
    };
    
    template<
      typename NodeRef, 
      typename ReductionTy, 
      typename Allocator, 
      typename CreateAllocator, 
      typename AllocNodeFunc, 
      typename SetNodeBoundsFunc, 
      typename CreateLeafFunc, 
      typename CalculateBounds, 
      typename ProgressMonitor>

      class GeneralBVHBuilderMorton
    {
      ALIGNED_CLASS;
      
    protected:
      static const size_t MAX_BRANCHING_FACTOR = 16;         //!< maximal supported BVH branching factor
      static const size_t MIN_LARGE_LEAF_LEVELS = 8;         //!< create balanced tree of we are that many levels before the maximal tree depth
      static const size_t SINGLE_THREADED_THRESHOLD = 4096;  //!< threshold to switch to single threaded build

    public:
      
      GeneralBVHBuilderMorton (const ReductionTy& identity, 
                        CreateAllocator& createAllocator, 
                        AllocNodeFunc& allocNode, 
                        SetNodeBoundsFunc& setBounds, 
                        CreateLeafFunc& createLeaf, 
                        CalculateBounds& calculateBounds,
                        ProgressMonitor& progressMonitor,
                        const size_t branchingFactor, const size_t maxDepth, 
                        const size_t minLeafSize, const size_t maxLeafSize)
        : identity(identity), 
        createAllocator(createAllocator), 
        allocNode(allocNode), 
        setBounds(setBounds), 
        createLeaf(createLeaf), 
        calculateBounds(calculateBounds),
        progressMonitor(progressMonitor),
        branchingFactor(branchingFactor), maxDepth(maxDepth), 
        minLeafSize(minLeafSize), maxLeafSize(maxLeafSize), 
        morton(nullptr) {}
      
      void splitFallback(MortonBuildRecord<NodeRef>& current, MortonBuildRecord<NodeRef>& leftChild, MortonBuildRecord<NodeRef>& rightChild) const
      {
        const unsigned int center = (current.begin + current.end)/2;
        leftChild.init(current.begin,center);
        rightChild.init(center,current.end);
      }
      
      BBox3fa createLargeLeaf(MortonBuildRecord<NodeRef>& current, Allocator alloc)
      {
        if (current.depth > maxDepth) 
          THROW_RUNTIME_ERROR("depth limit reached");
        
        /* create leaf for few primitives */
        if (current.size() <= maxLeafSize) {
          BBox3fa bounds;
          createLeaf(current,alloc,bounds);
          return bounds;
        }
        
        /* fill all children by always splitting the largest one */
        MortonBuildRecord<NodeRef> children[MAX_BRANCHING_FACTOR];
        size_t numChildren = 1;
        children[0] = current;
        
        do {
          
          /* find best child with largest bounding box area */
          int bestChild = -1;
          int bestSize = 0;
          for (size_t i=0; i<numChildren; i++)
          {
            /* ignore leaves as they cannot get split */
            if (children[i].size() <= maxLeafSize)
              continue;
            
            /* remember child with largest size */
            if (children[i].size() > bestSize) { 
              bestSize = children[i].size();
              bestChild = i;
            }
          }
          if (bestChild == -1) break;
          
          /*! split best child into left and right child */
          __aligned(64) MortonBuildRecord<NodeRef> left, right;
          splitFallback(children[bestChild],left,right);
          
          /* add new children left and right */
          left.depth = current.depth+1; 
          right.depth = current.depth+1;
          children[bestChild] = children[numChildren-1];
          children[numChildren-1] = left;
          children[numChildren+0] = right;
          numChildren++;
          
        } while (numChildren < branchingFactor);
        
        /* create node */
        auto node = allocNode(current,children,numChildren,alloc);
        
        /* recurse into each child */
        BBox3fa bounds[MAX_BRANCHING_FACTOR];
        for (size_t i=0; i<numChildren; i++) {
          bounds[i] = createLargeLeaf(children[i],alloc);
        }
        return setBounds(node,bounds,numChildren);
      }
      
      /*! recreates morton codes when reaching a region where all codes are identical */
      void recreateMortonCodes(MortonBuildRecord<NodeRef>& current) const
      {
        BBox3fa centBounds(empty);
        for (size_t i=current.begin; i<current.end; i++)
          centBounds.extend(center2(calculateBounds(morton[i])));
        
        MortonCodeGenerator::MortonCodeMapping mapping(centBounds);
        for (size_t i=current.begin; i<current.end; i++)
        {
          const BBox3fa b = calculateBounds(morton[i]);
          const float4 lower = (float4)b.lower;
          const float4 upper = (float4)b.upper;
          const float4 centroid = lower+upper;
          const int4 binID = int4((centroid-mapping.base)*mapping.scale);
          const unsigned int bx = extract<0>(binID);
          const unsigned int by = extract<1>(binID);
          const unsigned int bz = extract<2>(binID);
          morton[i].code = bitInterleave(bx,by,bz);
        }
        std::sort(morton+current.begin,morton+current.end); // FIXME: use radix sort
      }
      
      __forceinline void split(MortonBuildRecord<NodeRef>& current,
                               MortonBuildRecord<NodeRef>& left,
                               MortonBuildRecord<NodeRef>& right) const
      {
        const unsigned int code_start = morton[current.begin].code;
        const unsigned int code_end   = morton[current.end-1].code;
        unsigned int bitpos = lzcnt(code_start^code_end);
        
        /* if all items mapped to same morton code, then create new morton codes for the items */
        if (unlikely(bitpos == 32)) // FIXME: maybe go here earlier to build better tree
        {
          recreateMortonCodes(current);
          const unsigned int code_start = morton[current.begin].code;
          const unsigned int code_end   = morton[current.end-1].code;
          bitpos = lzcnt(code_start^code_end);
          
          /* if the morton code is still the same, goto fall back split */
          if (unlikely(bitpos == 32)) 
          {
            size_t center = (current.begin + current.end)/2; 
            left.init(current.begin,center);
            right.init(center,current.end);
            return;
          }
        }
        
        /* split the items at the topmost different morton code bit */
        const unsigned int bitpos_diff = 31-bitpos;
        const unsigned int bitmask = 1 << bitpos_diff;
        
        /* find location where bit differs using binary search */
        size_t begin = current.begin;
        size_t end   = current.end;
        while (begin + 1 != end) {
          const size_t mid = (begin+end)/2;
          const unsigned bit = morton[mid].code & bitmask;
          if (bit == 0) begin = mid; else end = mid;
        }
        size_t center = end;
#if defined(DEBUG)      
        for (unsigned int i=begin;  i<center; i++) assert((morton[i].code & bitmask) == 0);
        for (unsigned int i=center; i<end;    i++) assert((morton[i].code & bitmask) == bitmask);
#endif
        
        left.init(current.begin,center);
        right.init(center,current.end);
      }
      
      BBox3fa recurse(MortonBuildRecord<NodeRef>& current, Allocator alloc, bool toplevel) 
      {
        if (alloc == nullptr) 
          alloc = createAllocator();

        /* call memory monitor function to signal progress */
        if (toplevel && current.size() <= SINGLE_THREADED_THRESHOLD)
          progressMonitor(current.size());
        
        __aligned(64) MortonBuildRecord<NodeRef> children[MAX_BRANCHING_FACTOR];
        
        /* create leaf node */
        if (unlikely(current.depth+MIN_LARGE_LEAF_LEVELS >= maxDepth || current.size() <= minLeafSize)) {
          return createLargeLeaf(current,alloc);
        }
        
        /* fill all 4 children by always splitting the one with the largest surface area */
        size_t numChildren = 1;
        children[0] = current;
        
        do {
          
          /* find best child with largest bounding box area */
          int bestChild = -1;
          unsigned bestItems = 0;
          for (unsigned int i=0; i<numChildren; i++)
          {
            /* ignore leaves as they cannot get split */
            if (children[i].size() <= minLeafSize)
              continue;
            
            /* remember child with largest area */
            if (children[i].size() > bestItems) { 
              bestItems = children[i].size();
              bestChild = i;
            }
          }
          if (bestChild == -1) break;
          
          /*! split best child into left and right child */
          __aligned(64) MortonBuildRecord<NodeRef> left, right;
          split(children[bestChild],left,right);
          
          /* add new children left and right */
          left.depth = right.depth = current.depth+1;
          children[bestChild] = children[numChildren-1];
          children[numChildren-1] = left;
          children[numChildren+0] = right;
          numChildren++;
          
        } while (numChildren < branchingFactor);
        
        /* create leaf node if no split is possible */
        if (unlikely(numChildren == 1)) {
          BBox3fa bounds; createLeaf(current,alloc,bounds); return bounds;
        }
        
        /* allocate node */
        auto node = allocNode(current,children,numChildren,alloc);
        
        /* process top parts of tree parallel */
        BBox3fa bounds[4];
        if (current.size() > SINGLE_THREADED_THRESHOLD)
        {
          SPAWN_BEGIN;
          for (size_t i=0; i<numChildren; i++) {
            SPAWN(([&,i]{ bounds[i] = recurse(children[i],nullptr,true); }));
          }
          SPAWN_END;
        }
        
        /* finish tree sequentially */
        else
        {
          for (size_t i=0; i<numChildren; i++) 
            bounds[i] = recurse(children[i],alloc,false);
        }
        
        return setBounds(node,bounds,numChildren);
      }
      
      /* build function */
      std::pair<NodeRef,BBox3fa> build(MortonID32Bit* src, MortonID32Bit* tmp, size_t numPrimitives) 
      {
        /* sort morton codes */
        morton = tmp;
        radix_sort_copy_u32(src,morton,numPrimitives);
        //memcpy(morton,src,numPrimitives*sizeof(MortonID32Bit));
        //std::sort(&morton[0],&morton[numPrimitives]);

        /* build BVH */
        NodeRef root;
        MortonBuildRecord<NodeRef> br(0,numPrimitives,&root,1);
        
        const BBox3fa bounds = recurse(br, nullptr, true);
        return std::make_pair(root,bounds);
      }
      
    public:
      MortonID32Bit* morton;
      const size_t branchingFactor;
      const size_t maxDepth;
      const size_t minLeafSize;
      const size_t maxLeafSize;
      
    public:
      ReductionTy identity;
      CreateAllocator& createAllocator;
      AllocNodeFunc& allocNode;
      SetNodeBoundsFunc& setBounds;
      CreateLeafFunc& createLeaf;
      CalculateBounds& calculateBounds;
      ProgressMonitor& progressMonitor;
    };

    
    template<
      typename NodeRef, 
      typename CreateAllocFunc, 
      typename ReductionTy, 
      typename AllocNodeFunc, 
      typename SetBoundsFunc, 
      typename CreateLeafFunc, 
      typename CalculateBoundsFunc, 
      typename ProgressMonitor>

      std::pair<NodeRef,BBox3fa> bvh_builder_morton_internal(CreateAllocFunc createAllocator, 
                                                             const ReductionTy& identity, 
                                                             AllocNodeFunc allocNode, 
                                                             SetBoundsFunc setBounds, 
                                                             CreateLeafFunc createLeaf, 
                                                             CalculateBoundsFunc calculateBounds,
                                                             ProgressMonitor progressMonitor,
                                                             MortonID32Bit* src, MortonID32Bit* tmp, size_t numPrimitives,
                                                             const size_t branchingFactor, const size_t maxDepth, 
                                                             const size_t minLeafSize, const size_t maxLeafSize)
    {
      typedef GeneralBVHBuilderMorton<
        NodeRef,
        ReductionTy,
        decltype(createAllocator()),
        CreateAllocFunc,
        AllocNodeFunc,
        SetBoundsFunc,
        CreateLeafFunc,
        CalculateBoundsFunc,
        ProgressMonitor> Builder;

      Builder builder(identity,
                      createAllocator,
                      allocNode,
                      setBounds,
                      createLeaf,
                      calculateBounds,
                      progressMonitor,
                      branchingFactor,
                      maxDepth,
                      minLeafSize,
                      maxLeafSize);

      return builder.build(src,tmp,numPrimitives);
    }

    template<
      typename NodeRef, 
      typename CreateAllocFunc, 
      typename ReductionTy, 
      typename AllocNodeFunc, 
      typename SetBoundsFunc, 
      typename CreateLeafFunc, 
      typename CalculateBoundsFunc,
      typename ProgressMonitor>

      std::pair<NodeRef,BBox3fa> bvh_builder_morton(CreateAllocFunc createAllocator, 
                                                    const ReductionTy& identity, 
                                                    AllocNodeFunc allocNode, 
                                                    SetBoundsFunc setBounds, 
                                                    CreateLeafFunc createLeaf, 
                                                    CalculateBoundsFunc calculateBounds,
                                                    ProgressMonitor progressMonitor,
                                                    MortonID32Bit* src, MortonID32Bit* temp, size_t numPrimitives,
                                                    const size_t branchingFactor, const size_t maxDepth, 
                                                    const size_t minLeafSize, const size_t maxLeafSize)
    {
      std::pair<NodeRef,BBox3fa> ret;

      /* compute scene bounds */
      const BBox3fa centBounds = parallel_reduce ( size_t(0), numPrimitives, BBox3fa(empty), [&](const range<size_t>& r) -> BBox3fa
      {
        BBox3fa bounds(empty);
        for (size_t i=r.begin(); i<r.end(); i++) 
          bounds.extend(center2(calculateBounds(src[i])));
        return bounds;
      }, [] (const BBox3fa& a, const BBox3fa& b) { return merge(a,b); });
      
      /* compute morton codes */
      MortonCodeGenerator::MortonCodeMapping mapping(centBounds);
      parallel_for ( size_t(0), numPrimitives, [&](const range<size_t>& r) 
      {
        MortonCodeGenerator generator(mapping,&temp[r.begin()]);
        for (size_t i=r.begin(); i<r.end(); i++) {
          generator(calculateBounds(src[i]),src[i].index);
        }
      });
      
      return bvh_builder_morton_internal<NodeRef>(
        createAllocator,identity,allocNode,setBounds,createLeaf,calculateBounds,progressMonitor,
        temp,src,numPrimitives,branchingFactor,maxDepth,minLeafSize,maxLeafSize);
    }
  }
}

