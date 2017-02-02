// ======================================================================== //
// Copyright 2009-2017 Intel Corporation                                    //
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

#include "../common/builder.h"
#include "../../common/algorithms/parallel_reduce.h"

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
        : begin(begin), end(end), depth(depth), parent(parent) {}

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

      union {
        struct {
          unsigned int code;
          unsigned int index;
        };
        uint64_t t;
      };
      
    public:   

#if defined(__X86_64__)
      __forceinline MortonID32Bit() {}
      __forceinline MortonID32Bit(const MortonID32Bit& m) { t = m.t; }
#endif

        __forceinline operator unsigned() const { return code; }
      
      __forceinline unsigned int get(const unsigned int shift, const unsigned int and_mask) const {
        return (code >> shift) & and_mask;
      }

      __forceinline unsigned int getByte(const size_t index) const {
        return ((unsigned char*)&code)[index];
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
        vfloat4 base;
        vfloat4 scale;
        
        __forceinline MortonCodeMapping(const BBox3fa& bounds)
        {
          base  = (vfloat4)bounds.lower;
          const vfloat4 diag  = (vfloat4)bounds.upper - (vfloat4)bounds.lower;
          scale = select(diag > vfloat4(1E-19f), rcp(diag) * vfloat4(LATTICE_SIZE_PER_DIM * 0.99f),vfloat4(0.0f));
        }
      };

      __forceinline MortonCodeGenerator(const MortonCodeMapping& mapping)
        : mapping(mapping), dest(nullptr), currentID(0), slots(0), ax(0), ay(0), az(0), ai(0) {}
      
      __forceinline MortonCodeGenerator(const BBox3fa& bounds, MortonID32Bit* dest)
        : mapping(bounds), dest(dest), currentID(0), slots(0), ax(0), ay(0), az(0), ai(0) {}
      
      __forceinline MortonCodeGenerator(const MortonCodeMapping& mapping, MortonID32Bit* dest)
        : mapping(mapping), dest(dest), currentID(0), slots(0), ax(0), ay(0), az(0), ai(0) {}

      __forceinline ~MortonCodeGenerator()
      {
#if !defined(__AVX2__)
        if (slots != 0)
        {
          const vint4 code = bitInterleave(ax,ay,az);
          for (size_t i=0; i<slots; i++) {
            dest[currentID-slots+i].index = ai[i];
            dest[currentID-slots+i].code = code[i];
          }
        }
#endif
      }

      __forceinline void operator() (const BBox3fa& b, const unsigned index)
      {
        const vfloat4 lower = (vfloat4)b.lower;
        const vfloat4 upper = (vfloat4)b.upper;
        const vfloat4 centroid = lower+upper;
        const vint4 binID = vint4((centroid-mapping.base)*mapping.scale); // FIXME: transform into fma

#if defined(__AVX2__)
        const unsigned int x = extract<0>(binID);
        const unsigned int y = extract<1>(binID);
        const unsigned int z = extract<2>(binID);
        const unsigned int xyz = bitInterleave(x,y,z);
        dest[currentID].index = index;
        dest[currentID].code  = xyz;
        currentID++;
#else        
        ax[slots] = extract<0>(binID);
        ay[slots] = extract<1>(binID);
        az[slots] = extract<2>(binID);
        ai[slots] = index;
        slots++;
        currentID++;

        if (slots == 4)
        {
          const vint4 code = bitInterleave(ax,ay,az);
          vint4::storeu(&dest[currentID-4],unpacklo(code,ai));
          vint4::storeu(&dest[currentID-2],unpackhi(code,ai));
          slots = 0;
        }
#endif
      }

      __forceinline unsigned int getCode(const BBox3fa& b)
      {
        const vfloat4 lower = (vfloat4)b.lower;
        const vfloat4 upper = (vfloat4)b.upper;
        const vfloat4 centroid = lower+upper;
        const vint4 binID = vint4((centroid-mapping.base)*mapping.scale); // FIXME: transform into fma
        const unsigned int x = extract<0>(binID);
        const unsigned int y = extract<1>(binID);
        const unsigned int z = extract<2>(binID);
        const unsigned int xyz = bitInterleave(x,y,z);
        return xyz;
      }
    
    public:
      const MortonCodeMapping mapping;
      MortonID32Bit* dest;
      const vfloat4 base;
      const vfloat4 scale;
      size_t currentID;
      size_t slots;
      vint4 ax, ay, az, ai;
    };
            

    inline void InPlace32BitRadixSort(MortonID32Bit* const morton, const size_t num, const unsigned int shift = 3*8)
    {
      static const unsigned int BITS = 8;
      static const unsigned int BUCKETS = (1 << BITS);
      static const unsigned int CMP_SORT_THRESHOLD = 16;

      __aligned(64) unsigned int count[BUCKETS];

      /* clear buckets */
      for (size_t i=0;i<BUCKETS;i++) count[i] = 0;

      /* count buckets */
#if defined(__INTEL_COMPILER)
#pragma nounroll
#endif
      for (size_t i=0;i<num;i++) count[ morton[i].get(shift,BUCKETS-1) ]++;

      /* prefix sums */
      __aligned(64) unsigned int head[BUCKETS];
      __aligned(64) unsigned int tail[BUCKETS];

      head[0] = 0;
      for (size_t i=1; i<BUCKETS; i++)    
        head[i] = head[i-1] + count[i-1];

      for (size_t i=0; i<BUCKETS-1; i++)    
        tail[i] = head[i+1];

      tail[BUCKETS-1] = head[BUCKETS-1] + count[BUCKETS-1];

      assert(tail[BUCKETS-1] == head[BUCKETS-1] + count[BUCKETS-1]);      
      assert(tail[BUCKETS-1] == num);      

      /* in-place swap */      
      for (size_t i=0;i<BUCKETS;i++)
      {
        /* process bucket */
        while(head[i] < tail[i])
        {
          MortonID32Bit v = morton[head[i]];
          while(1)
          {
            const size_t b = v.get(shift,BUCKETS-1);
            if (b == i) break;
            std::swap(v,morton[head[b]++]);
          }
          assert(v.get(shift,BUCKETS-1) == i);
          morton[head[i]++] = v;
        }
      }
      if (shift == 0) return;

      size_t offset = 0;
      for (size_t i=0;i<BUCKETS;i++)
        if (count[i])
        {

          for (size_t j=offset;j<offset+count[i]-1;j++)
            assert(morton[j].get(shift,BUCKETS-1) == i);

          if (unlikely(count[i] < CMP_SORT_THRESHOLD))
            insertionsort_ascending(morton + offset, count[i]);
          else
            InPlace32BitRadixSort(morton + offset, count[i], shift-BITS);

          for (size_t j=offset;j<offset+count[i]-1;j++)
            assert(morton[j] <= morton[j+1]);

          offset += count[i];
        }      
    }

    
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

    public:
      
      GeneralBVHBuilderMorton (const ReductionTy& identity, 
                               CreateAllocator& createAllocator, 
                               AllocNodeFunc& allocNode, 
                               SetNodeBoundsFunc& setBounds, 
                               CreateLeafFunc& createLeaf, 
                               CalculateBounds& calculateBounds,
                               ProgressMonitor& progressMonitor,
                               const size_t branchingFactor, const size_t maxDepth, 
                               const size_t minLeafSize, const size_t maxLeafSize,
                               const size_t singleThreadThreshold)
        : identity(identity), 
        createAllocator(createAllocator), 
        allocNode(allocNode), 
        setBounds(setBounds), 
        createLeaf(createLeaf), 
        calculateBounds(calculateBounds),
        progressMonitor(progressMonitor),
        morton(nullptr), 
        branchingFactor(branchingFactor), 
        maxDepth(maxDepth), 
        minLeafSize(minLeafSize), 
        maxLeafSize(maxLeafSize),
        singleThreadThreshold(singleThreadThreshold) {}
      
      void splitFallback(MortonBuildRecord<NodeRef>& current, MortonBuildRecord<NodeRef>& leftChild, MortonBuildRecord<NodeRef>& rightChild) const
      {
        const unsigned int center = (current.begin + current.end)/2;
        leftChild.init(current.begin,center);
        rightChild.init(center,current.end);
      }
      
      BBox3fa createLargeLeaf(MortonBuildRecord<NodeRef>& current, Allocator alloc)
      {
        /* this should never occur but is a fatal error */
        if (current.depth > maxDepth) 
          throw_RTCError(RTC_UNKNOWN_ERROR,"depth limit reached");
        
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
          size_t bestChild = -1;
          size_t bestSize = 0;
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
          if (bestChild == size_t(-1)) break;
          
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
      __noinline void recreateMortonCodes(MortonBuildRecord<NodeRef>& current) const
      {
        BBox3fa centBounds(empty);
        for (size_t i=current.begin; i<current.end; i++)
          centBounds.extend(center2(calculateBounds(morton[i])));
        
        MortonCodeGenerator::MortonCodeMapping mapping(centBounds);
        for (size_t i=current.begin; i<current.end; i++)
        {
          const BBox3fa b = calculateBounds(morton[i]);
          const vfloat4 lower = (vfloat4)b.lower;
          const vfloat4 upper = (vfloat4)b.upper;
          const vfloat4 centroid = lower+upper;
          const vint4 binID = vint4((centroid-mapping.base)*mapping.scale);
          const unsigned int bx = extract<0>(binID);
          const unsigned int by = extract<1>(binID);
          const unsigned int bz = extract<2>(binID);
          morton[i].code = bitInterleave(bx,by,bz);
        }
#if defined(TASKING_TBB)
        tbb::parallel_sort(morton+current.begin,morton+current.end);
#else
        InPlace32BitRadixSort(morton+current.begin,current.end-current.begin);
#endif
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
            unsigned center = (current.begin + current.end)/2; 
            left.init(current.begin,center);
            right.init(center,current.end);
            return;
          }
        }
        
        /* split the items at the topmost different morton code bit */
        const unsigned int bitpos_diff = 31-bitpos;
        const unsigned int bitmask = 1 << bitpos_diff;
        
        /* find location where bit differs using binary search */
        unsigned begin = current.begin;
        unsigned end   = current.end;
        while (begin + 1 != end) {
          const unsigned mid = (begin+end)/2;
          const unsigned bit = morton[mid].code & bitmask;
          if (bit == 0) begin = mid; else end = mid;
        }
        unsigned center = end;
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
        if (toplevel && current.size() <= singleThreadThreshold)
          progressMonitor(current.size());
        
        __aligned(64) MortonBuildRecord<NodeRef> children[MAX_BRANCHING_FACTOR];
        
        /* create leaf node */
        if (unlikely(current.depth+MIN_LARGE_LEAF_LEVELS >= maxDepth || current.size() <= minLeafSize)) {
          return createLargeLeaf(current,alloc);
        }
        
        /* fill all children by always splitting the one with the largest surface area */
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
        BBox3fa bounds[MAX_BRANCHING_FACTOR];
        if (current.size() > singleThreadThreshold)
        {
          /*! parallel_for is faster than spawing sub-tasks */
          parallel_for(size_t(0), numChildren, [&] (const range<size_t>& r) {
              for (size_t i=r.begin(); i<r.end(); i++) {
                bounds[i] = recurse(children[i],nullptr,true); 
                _mm_mfence(); // to allow non-temporal stores during build
              }                
            });
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
        /* using 4 phases radix sort */
        morton = src;
        radix_sort_u32(src,tmp,numPrimitives,singleThreadThreshold);
        //InPlace32BitRadixSort(morton,numPrimitives);

        /* build BVH */
        NodeRef root;
        MortonBuildRecord<NodeRef> br(0,unsigned(numPrimitives),&root,1);
        
        const BBox3fa bounds = recurse(br, nullptr, true);
        _mm_mfence(); // to allow non-temporal stores during build

        return std::make_pair(root,bounds);
      }
      
    public:
      ReductionTy identity;
      CreateAllocator& createAllocator;
      AllocNodeFunc& allocNode;
      SetNodeBoundsFunc& setBounds;
      CreateLeafFunc& createLeaf;
      CalculateBounds& calculateBounds;
      ProgressMonitor& progressMonitor;

    public:
      MortonID32Bit* morton;
      const size_t branchingFactor;
      const size_t maxDepth;
      const size_t minLeafSize;
      const size_t maxLeafSize;
      const size_t singleThreadThreshold;
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
                                                             MortonID32Bit* src, 
                                                             MortonID32Bit* tmp, 
                                                             size_t numPrimitives,
                                                             const size_t branchingFactor, 
                                                             const size_t maxDepth, 
                                                             const size_t minLeafSize, 
                                                             const size_t maxLeafSize,
                                                             const size_t singleThreadThreshold)
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
                      maxLeafSize,
                      singleThreadThreshold);

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
                                                    MortonID32Bit* src, 
                                                    MortonID32Bit* temp, 
                                                    size_t numPrimitives,
                                                    const size_t branchingFactor, 
                                                    const size_t maxDepth, 
                                                    const size_t minLeafSize, 
                                                    const size_t maxLeafSize,
                                                    const size_t singleThreadThreshold)
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
        //MortonCodeGenerator generator(mapping,&temp[r.begin()]);
        MortonCodeGenerator generator(mapping,&src[r.begin()]);

        for (size_t i=r.begin(); i<r.end(); i++) {
          generator(calculateBounds(src[i]),src[i].index);
        }
      });
      
      return bvh_builder_morton_internal<NodeRef>(
        createAllocator,identity,allocNode,setBounds,createLeaf,calculateBounds,progressMonitor,
        src,temp,numPrimitives,branchingFactor,maxDepth,minLeafSize,maxLeafSize,singleThreadThreshold);

    }
  }
}

