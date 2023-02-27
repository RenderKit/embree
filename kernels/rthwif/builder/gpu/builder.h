// Copyright 2009-2021 Intel Corporation
// SPDX-License-Identifier: Apache-2.0

#pragma once

#if defined(EMBREE_SYCL_GPU_BVH_BUILDER)
#include "common.h"
#include "AABB.h"
#include "AABB3f.h"
//#include "bvh.h"
//#include "../bvh/gbvh_intersector.h"
//#include "../bvh/gbvh_point_query.h"
//#include "geometry.h"


/* ====== BVH Builder and Quantization config ====== */

#define BVH_BINNING_BINS 16
// SYCL_SIMD_WIDTH
#define QUANT_BITS            8
#define QUANT_MIN             0
#define QUANT_MAX             255
#define QUANT_MAX_MANT        (255.0f/256.0f)

#define DBG_BUILD(x) 

namespace embree
{
  namespace gpu
  {
    
    struct Globals
    {
      /* 1. cacheline */
      AABB geometryBounds;
      AABB centroidBounds;

      /* 2. cacheline */
      unsigned int node_mem_allocator_start;
      unsigned int node_mem_allocator_cur;
      unsigned int bvh2_index_allocator;
      unsigned int bvh2_breathfirst_records_start;
      unsigned int bvh2_breathfirst_records_end;
      unsigned int range_allocator;
      unsigned int pad0[10];

      /* 3. cacheline */
      unsigned int leaf_mem_allocator_start;
      unsigned int leaf_mem_allocator_cur;
      unsigned int pad1[14];

      /* 4. cacheline */
      unsigned int numPrimitives;
      unsigned int numOriginalPrimitives;
      unsigned int numSplittedPrimitives;
      unsigned int init_numPrimitives;
      unsigned int leafPrimType;
      unsigned int leafSize;
      unsigned int numBuildRecords;
      unsigned int numBuildRecords_extended;
      unsigned int totalAllocatedMem;
      float presplitPrioritySum;

      unsigned int sync;
      float probThreshold;
      unsigned int numGlobalBuildRecords;
      unsigned int rootIndex;
      void *topLevelRoot;

      /* 5. cacheline */
      unsigned int sched;
      unsigned int numWGs;

      __forceinline void init(char *bvh_mem,
                       unsigned int _numPrimitives,
                       unsigned int _node_data_start,
                       unsigned int _leaf_data_start,
                       unsigned int _totalBytes,
                       unsigned int _leafPrimType,
                       unsigned int _leafSize,
                       void* scenePtr)
      {
        /*
        BVHBase *base       = (BVHBase*)bvh_mem;
        base->nodeDataStart        = _node_data_start/64;
        base->nodeDataCur          = _node_data_start/64;
        base->leafDataStart        = _leaf_data_start/64;
        base->leafDataCur          = _leaf_data_start/64;
        base->rootNodeOffset       = _node_data_start; // FIXME: should be set by builder
        base->scenePtr             = scenePtr;
        */

        geometryBounds.init();
        centroidBounds.init();

        node_mem_allocator_cur     = _node_data_start;
        node_mem_allocator_start   = _node_data_start;

        bvh2_index_allocator           = 0;
        bvh2_breathfirst_records_start = 0;
        bvh2_breathfirst_records_end   = 0;
        
        
        leaf_mem_allocator_cur     = _leaf_data_start;
        leaf_mem_allocator_start   = _leaf_data_start;

        numBuildRecords          = 0;
        numBuildRecords_extended = 0;
        numPrimitives            = _numPrimitives;
        init_numPrimitives       = 0;
        numSplittedPrimitives    = 0;
        totalAllocatedMem        = _totalBytes;
        sync                     = 0;
        probThreshold            = 0.0f;
        leafPrimType             = _leafPrimType;
        leafSize                 = _leafSize;
        numGlobalBuildRecords    = 0;
        rootIndex                = 0;
        topLevelRoot             = nullptr;

        sched                    = 0;
        numWGs                   = 0;
        range_allocator = 0;
      }

            
      __forceinline void resetGlobalCounters()
      {
        node_mem_allocator_cur = node_mem_allocator_start;
        leaf_mem_allocator_cur = leaf_mem_allocator_start;
        numBuildRecords = 0;	
      }
      
      __forceinline ulong alloc_node_mem(const uint size)
      {
        const uint aligned_size = ((size+63)/64)*64; /* allocate in 64 bytes blocks */
        return gpu::atomic_add_global(&node_mem_allocator_cur,aligned_size);
      }

      __forceinline ulong alloc_leaf_mem(const uint size)
      {
        const uint aligned_size = ((size+15)/16)*16; /* allocate in 16 bytes blocks */
        return gpu::atomic_add_global(&leaf_mem_allocator_cur,aligned_size);
      }
      
    };

    /*
    struct Range {
      uint start;
      uint end;
      __forceinline Range()
      {}

      __forceinline Range(const uint start, const uint end) : start(start), end(end)
      {}

      __forceinline uint size() const { return end-start; }

      __forceinline uint center() const { return (end+start)>>1; }
      
    };
    */

#if 0
    // ================================================================================================================================================================================
    // ============================================================================= MORTON CODES =====================================================================================
    // ================================================================================================================================================================================

#if 1 
    struct __aligned(8) MortonCodePrimitive
    {
      static const uint GRID_SHIFT = 10; // * 3 = 30
      
      static const uint CODE_SHIFT = 64-3*GRID_SHIFT;      
      static const uint64_t INDEX_MASK = ((uint64_t)1 << CODE_SHIFT)-1;

      uint64_t index_code; // 64bit code + index combo

      __forceinline MortonCodePrimitive() {}
      __forceinline MortonCodePrimitive(const ulong c) : index_code(c) {}
      __forceinline MortonCodePrimitive(const uint64_t code, const uint index) { index_code = (code << CODE_SHIFT) | (uint64_t)index; }
      __forceinline MortonCodePrimitive(const uint3 &v, const uint index) : MortonCodePrimitive(gpu::bitInterleave3D_64bits(v), index) {}

      //__forceinline uint getIndex() const { return (uint)index_code; }
      //__forceinline uint getCode()  const { return (uint)(index_code>>32); }

      __forceinline uint getIndex() const { return (uint)(index_code & INDEX_MASK); }
      __forceinline uint64_t getCode()  const { return index_code; }

      __forceinline uint64_t getMCode()  const { return index_code & (~INDEX_MASK); }      
      
      __forceinline operator const uint64_t&()  const { return index_code; }

      __forceinline bool operator < (const MortonCodePrimitive& mc) const
      {
        return index_code < mc.index_code;
      }
    

    };
#endif

    struct __aligned(8) MortonCodePrimitive40x24Bits3D
    {
      static const uint GRID_SHIFT = 13; // * 3 = 30
      
      static const uint CODE_SHIFT = 64-(3*GRID_SHIFT+1);      
      static const uint64_t INDEX_MASK = ((uint64_t)1 << CODE_SHIFT)-1;
        
      uint64_t index_code; // 64bit code + index combo

      __forceinline MortonCodePrimitive40x24Bits3D() {}      
      __forceinline MortonCodePrimitive40x24Bits3D(const uint64_t code, const uint index) { index_code = (code << CODE_SHIFT) | (uint64_t)index; }
      __forceinline MortonCodePrimitive40x24Bits3D(const uint3 &v, const uint index)
      {
        const uint64_t code = gpu::bitInterleave3D(v);
        MortonCodePrimitive40x24Bits3D(code,index);
      }

      __forceinline uint getIndex() const { return (uint)(index_code & INDEX_MASK); }
      __forceinline uint64_t getCode()  const { return index_code; }

      __forceinline uint64_t getMCode()  const { return index_code & (~INDEX_MASK); }      
      
      __forceinline operator const uint64_t&()  const { return index_code; }

      __forceinline bool operator < (const MortonCodePrimitive40x24Bits3D& mc) const
      {
        return index_code < mc.index_code;
      }
    
    };

    struct __aligned(8) MortonCodePrimitive40x24Bits4D
    {
      static const uint GRID_SHIFT = 10; // * 4 = 40      
      static const uint CODE_SHIFT = 64-4*GRID_SHIFT;      
      static const uint64_t INDEX_MASK = ((uint64_t)1 << CODE_SHIFT)-1;
        
      uint64_t index_code; // 64bit code + index combo

      __forceinline MortonCodePrimitive40x24Bits4D() {}
      __forceinline MortonCodePrimitive40x24Bits4D(const uint64_t code, const uint index) { index_code = (code << CODE_SHIFT) | (uint64_t)index; }
      __forceinline MortonCodePrimitive40x24Bits4D(const uint4 &v, const uint index)
      {
        const uint64_t code = gpu::bitInterleave4D(v);
        MortonCodePrimitive40x24Bits4D(code,index);
      }
      
      __forceinline uint getIndex() const { return (uint)(index_code & INDEX_MASK); }
      __forceinline uint64_t getCode()  const { return index_code; }
      __forceinline uint64_t getMCode()  const { return index_code & (~INDEX_MASK); }            
      __forceinline operator const uint64_t&()  const { return index_code; }
      __forceinline bool operator < (const MortonCodePrimitive40x24Bits4D& mc) const
      {
        return index_code < mc.index_code;
      }
    
    };
    

    /* used by PLOC in a two phase process */
    struct __aligned(8) MortonCodePrimitive64Bit_2x
    {
      static const uint GRID_SHIFT = 21; // * 3 = 63
      
      static const uint CODE_SHIFT = 32;      
      static const uint64_t INDEX_MASK = ((uint64_t)1 << CODE_SHIFT)-1;
        
      uint64_t index_code; // 64bit code + index combo

      __forceinline MortonCodePrimitive64Bit_2x() {}
      __forceinline MortonCodePrimitive64Bit_2x(const uint64_t code, const uint index) { index_code = (code << CODE_SHIFT) | (uint64_t)index; }

      __forceinline uint getIndex() const { return (uint)(index_code & INDEX_MASK); }
      __forceinline uint64_t getCode()  const { return index_code; }

      __forceinline uint64_t getMCode()  const { return index_code & (~INDEX_MASK); }      
      
      __forceinline operator const uint64_t&()  const { return index_code; }

      __forceinline bool operator < (const MortonCodePrimitive64Bit_2x& mc) const
      {
        return index_code < mc.index_code;
      }
    
    };


    struct MortonCodePrimitive64x32Bits3D
    {
      static const uint GRID_SHIFT = 21; // * 3 = 63
      static const uint KEY_BITS   = 64;
      
      uint64_t code;
      uint index;

      __forceinline MortonCodePrimitive64x32Bits3D() {}      
      __forceinline MortonCodePrimitive64x32Bits3D(const uint64_t code, const uint index) : code(code), index(index) {}
      __forceinline MortonCodePrimitive64x32Bits3D(const uint3 &v, const uint index) : code(gpu::bitInterleave3D_64bits(v)), index(index)
      {
      }
      
      __forceinline bool operator < (const MortonCodePrimitive64x32Bits3D& emc) const
      {
      
        if (code < emc.code)
          return true;
        else if (code == emc.code)
          return index < emc.index;
        else
          return false;
      }

      __forceinline uint64_t getCode() const { return code; }
      __forceinline uint64_t getMCode()  const { return code; }            
      __forceinline uint    getIndex() const { return index; }
      __forceinline operator const uint64_t&()  const { return code; }    
    };

    struct MortonCodePrimitive64x32Bits4D
    {
      static const uint GRID_SHIFT = 16; // * 4 = 64
      static const uint KEY_BITS   = 64;
      
      uint64_t code;
      uint index;

      __forceinline MortonCodePrimitive64x32Bits4D() {}
      __forceinline MortonCodePrimitive64x32Bits4D(const uint64_t code, const uint index) : code(code), index(index) {}
      __forceinline MortonCodePrimitive64x32Bits4D(const uint4 &v, const uint index) : code( gpu::bitInterleave4D_64bits(v)), index(index)
      {
      }
      
      __forceinline bool operator < (const MortonCodePrimitive64x32Bits4D& emc) const
      {
      
        if (code < emc.code)
          return true;
        else if (code == emc.code)
          return index < emc.index;
        else
          return false;
      }

      __forceinline uint64_t getCode() const { return code; }
      __forceinline uint64_t getMCode()  const { return code; }            
      __forceinline uint    getIndex() const { return index; }
      __forceinline operator const uint64_t&()  const { return code; }    
    };
    
    template<typename type>  
      __forceinline unsigned int findSplit64Bit(const gpu::Range &current, const type *const morton)
    {
      const uint64_t code_start = morton[current.start].getCode();
      const uint64_t code_end   = morton[current.end-1].getCode();

      if (code_start != code_end)
      {
        uint64_t bitpos = sycl::clz(code_start^code_end);
        
        /* split the items at the topmost different morton code bit */
        const uint64_t bitpos_diff = 63-bitpos;
        const uint64_t bitmask = (uint64_t)1 << bitpos_diff;
  
        /* find location where bit differs using binary search */
        unsigned int begin = current.start;
        unsigned int end   = current.end;
        while (begin + 1 != end) {
          const unsigned int mid = (begin+end)/2;
          const uint64_t bit = morton[mid].getCode() & bitmask;
          if (bit == 0) begin = mid; else end = mid;
        }
        unsigned int center = end;
        return center;        
      }
      else  /* if all items mapped to same morton code */
      {
        uint bitpos = sycl::clz(morton[current.start].getIndex()^morton[current.end-1].getIndex());
        
        /* split the items at the topmost different morton code bit */
        const uint bitpos_diff = 31-bitpos;
        const uint bitmask = (uint)1 << bitpos_diff;
  
        /* find location where bit differs using binary search */
        unsigned int begin = current.start;
        unsigned int end   = current.end;
        while (begin + 1 != end) {
          const unsigned int mid = (begin+end)/2;
          const uint bit = morton[mid].getIndex() & bitmask;
          if (bit == 0) begin = mid; else end = mid;
        }
        unsigned int center = end;
        return center;
      }
    }

    template<typename type>      
    __forceinline uint delta(const type& m0, const type& m1)
    {
      return m0.getCode() != m1 .getCode() ? (m0.getCode()^m1.getCode()) : m0.getIndex()^m1.getIndex(); //sycl::ext::intel::ctz(m0.index_code^m1.index_code);
    }
    

    template<typename type>  
      __forceinline void splitRange(const gpu::Range &current, const type *const morton, gpu::Range &left, gpu::Range &right)
    {
      const uint split = findSplit64Bit(current,morton);
      left.start  = current.start;
      left.end    = split;
      right.start = split;
      right.end   = current.end;
    }


    template<typename type>          
  __forceinline int sigma(const int i1, const int i2, const type *const morton, const uint numPrimitives) 
  {
    const int left  = min(i1, i2);
    const int right = max(i1, i2);
    if (left < 0 || right >= numPrimitives) return -1;
    const ulong left_code  = morton[left].getCode();
    const ulong right_code = morton[right].getCode();
    if (left_code != right_code)
      return sycl::clz(left_code ^ right_code);
    else
      return 64 + sycl::clz(morton[left].getIndex() ^ morton[right].getIndex());
  }

    template<typename type>          
  __forceinline struct gpu::Range findSegment(const unsigned int idx, const type *const mc, uint const numPrimitives)
  {
    const int delta = sigma(idx, idx+1, mc, numPrimitives) >= sigma(idx, idx-1, mc, numPrimitives) ? 1 : -1;
    const int sigma_min = sigma(idx, idx-delta, mc, numPrimitives);
    int max_range = 2;
    while (sigma(idx,idx + max_range * delta, mc, numPrimitives) > sigma_min)
      max_range <<= 1;
    int step  = 0;
    int r = max_range;
    do
    {
      r >>= 1;
      if(sigma(idx, idx + (step + r)*delta, mc, numPrimitives) > sigma_min)
	step += r;
    }
    while (r > 1);
    struct gpu::Range range;
    range.start = min(idx, idx + step*delta);
    range.end   = max(idx, idx + step*delta) + 1;
    return range;          
  }
    

    struct BuildRecord
    {
      AABB centroidBounds;
      unsigned int start, end;
      GBVHN<BVH_NODE_N>::NodeRef *parent;

      __forceinline BuildRecord() {}
      
      __forceinline void init(unsigned int _start, unsigned int _end, AABB &bounds)
      {
        centroidBounds = bounds;
        start = _start;
        end   = _end;
        parent = NULL;	
      }

      __forceinline void init(unsigned int _start, unsigned int _end, void *_parent=NULL)
      {
        centroidBounds.init();
        start = _start;
        end   = _end;
        parent = (NodeRef*)_parent;	
      }
      

      __forceinline void extend(const AABB &primref)
      {
        centroidBounds.extend(primref.centroid2());
      }

      __forceinline unsigned int size() const { return end - start; }

      __forceinline operator const AABB &() const { return centroidBounds; }

      __forceinline float4 centroid2() const { return centroidBounds.centroid2(); }

      friend __forceinline embree_ostream operator<<(embree_ostream cout, const BuildRecord &br)
      {
        cout << "start " << br.start << " end " << br.end << " parent " << (void*)br.parent << " bounds " << br.centroidBounds << " ";
        return cout;
      }
      

    };


    struct __aligned(64) TopLevelBuildRecord
    {
      AABB centroidBounds;
      AABB geometryBounds;
      unsigned int start, end;
      GBVHN<BVH_NODE_N>::NodeRef *parent;

      __forceinline void init(unsigned int _start, unsigned int _end, void *_parent=NULL)
      {
        centroidBounds.init();
        geometryBounds.init();        
        start = _start;
        end   = _end;
        parent = (NodeRef*)_parent;	
      }
      
      __forceinline unsigned int size() const { return end - start; }

      __forceinline operator const AABB &() const { return geometryBounds; }
      
      __forceinline float4 centroid2() const { return geometryBounds.centroid2(); }

      __forceinline operator BuildRecord() const
      {
        BuildRecord br;
        br.centroidBounds = centroidBounds;
        br.start          = start;
        br.end            = end;        
        br.parent         = parent;
        return br;
      }

      __forceinline uint getPrimitiveCost() const { return 1; /*size();*/ }
      
    };


  #endif

    
    struct BinaryMortonCodeHierarchy
    {
      struct Range range;
      unsigned int leftChild;
      unsigned int rightChild;
      unsigned int flag;
    };

    struct StatStackEntry
    {
      AABB aabb;
      unsigned int node;
      unsigned int type;
      unsigned int depth;
      float area;
    };

    struct BuildRecordMorton {
      unsigned int nodeID;
      unsigned int items;
      unsigned int current_index;
      unsigned int parent_index;
    };

    struct Split
    {
      float sah;
      uint dim;
      uint pos;

      Split() {}

    Split(const float sah, const uint dim, const uint pos) : sah(sah), dim(dim), pos(pos) {}
    };


    struct BinMapping
    {
      float4 ofs, scale;

      __forceinline void init(const AABB &centBounds, const uint bins)
      {
        const float4 eps(1E-34f);
        const float4 diag = max(eps, centBounds.upper - centBounds.lower);
        scale = (float4)(0.99f*(float)bins)/diag;
        scale = cselect(diag > eps, scale, (float4)(0.0f));
        ofs  = centBounds.lower;
      }

    };

    __forceinline AABB3f convert_AABB3f(const AABB &aabb)
    {
      return AABB3f(aabb.lower.x(), aabb.lower.y(), aabb.lower.z(), aabb.upper.x(), aabb.upper.y(), aabb.upper.z());
    }

    __forceinline AABB convert_AABB(const AABB3f &aabb)
    {
      return AABB(float4(aabb.lower_x,aabb.lower_y,aabb.lower_z,0.0f),
                  float4(aabb.upper_x,aabb.upper_y,aabb.upper_z,0.0f));
    }


    EMBREE_SYCL_SIMD(BVH_BINNING_BINS) __forceinline float left_to_right_area16(const AABB3f &low)
    {
      AABB3f low_prefix = low.sub_group_scan_exclusive_min_max();
      return low_prefix.halfArea();
    }

    EMBREE_SYCL_SIMD(BVH_BINNING_BINS) __forceinline uint left_to_right_counts16(uint low)
    {
      return sub_group_exclusive_scan( low, SYCL_ONEAPI::plus<uint>());      
    }


    EMBREE_SYCL_SIMD(BVH_BINNING_BINS) __forceinline float right_to_left_area16(const AABB3f &low)
    {
      const uint subgroupLocalID = get_sub_group_local_id();
      const uint subgroupSize    = BVH_BINNING_BINS;
      const uint ID              = subgroupSize - 1 - subgroupLocalID;  
      AABB3f low_reverse         = low.sub_group_shuffle(ID);
      AABB3f low_prefix          = low_reverse.sub_group_scan_inclusive_min_max();
      const float low_area       = embree::sub_group_shuffle<float>(low_prefix.halfArea(),ID);
      return low_area;
    }

    EMBREE_SYCL_SIMD(BVH_BINNING_BINS) __forceinline uint right_to_left_counts16(uint low)
    {
      const uint subgroupLocalID = get_sub_group_local_id();
      const uint subgroupSize    = BVH_BINNING_BINS;
      const uint ID              = subgroupSize - 1 - subgroupLocalID;  
      const uint low_reverse     = embree::sub_group_shuffle<uint>(low,ID);
      const uint low_prefix      = sub_group_inclusive_scan(low_reverse, std::plus<uint>());
      return sub_group_shuffle<uint>(low_prefix,ID);
    }

    EMBREE_SYCL_SIMD(BVH_BINNING_BINS) __forceinline float2 left_to_right_area32(const AABB3f &low, const AABB3f &high)
    {
      AABB3f low_prefix     = low.sub_group_scan_exclusive_min_max();
      AABB3f low_reduce     = low.sub_group_reduce();	      	      
      AABB3f high_prefix    = high.sub_group_scan_exclusive_min_max();
      high_prefix.extend(low_reduce);
      const float low_area  = low_prefix.halfArea();
      const float high_area = high_prefix.halfArea();
      return float2(low_area,high_area);
    }
    
    EMBREE_SYCL_SIMD(BVH_BINNING_BINS) __forceinline uint2 left_to_right_counts32(uint low, uint high)
    {
      const uint low_prefix  = sub_group_exclusive_scan(low, std::plus<uint>());
      const uint low_reduce  = sub_group_reduce(low, std::plus<uint>());
      const uint high_prefix = sub_group_exclusive_scan(high, std::plus<uint>());
      return uint2(low_prefix,low_reduce+high_prefix);
    }
        
    EMBREE_SYCL_SIMD(BVH_BINNING_BINS) __forceinline float2 right_to_left_area32(const AABB3f &low, const AABB3f &high)
    {
      const uint subgroupLocalID = get_sub_group_local_id();
      const uint subgroupSize    = BVH_BINNING_BINS;
      const uint ID              = subgroupSize - 1 - subgroupLocalID;  	
      AABB3f low_reverse         = high.sub_group_shuffle(ID);
      AABB3f high_reverse        = low.sub_group_shuffle(ID);
      AABB3f low_prefix          = low_reverse.sub_group_scan_inclusive_min_max();
      AABB3f low_reduce          = low_reverse.sub_group_reduce();	      	      
      AABB3f high_prefix         = high_reverse.sub_group_scan_inclusive_min_max();
      high_prefix.extend(low_reduce);
      const float low_area       = sub_group_shuffle<float>(high_prefix.halfArea(),ID);
      const float high_area      = sub_group_shuffle<float>( low_prefix.halfArea(),ID);
      return float2(low_area,high_area);	
    }
    
    EMBREE_SYCL_SIMD(BVH_BINNING_BINS) __forceinline uint2 right_to_left_counts32(uint low, uint high)
    {
      const uint subgroupLocalID = get_sub_group_local_id();
      const uint subgroupSize    = BVH_BINNING_BINS; 
      const uint ID              = subgroupSize - 1 - subgroupLocalID;  	
      const uint low_reverse     = sub_group_shuffle<uint>(high,ID);
      const uint high_reverse    = sub_group_shuffle<uint>(low,ID);
      const uint low_prefix      = sub_group_inclusive_scan(low_reverse, SYCL_ONEAPI::plus<uint>());
      const uint low_reduce      = sub_group_reduce(low_reverse, SYCL_ONEAPI::plus<uint>());
      const uint high_prefix     = sub_group_inclusive_scan(high_reverse, SYCL_ONEAPI::plus<uint>()) + low_reduce;
      return uint2(sub_group_shuffle<uint>(high_prefix,ID),sub_group_shuffle<uint>(low_prefix,ID));	
    }

    EMBREE_SYCL_SIMD(BVH_BINNING_BINS) __forceinline ulong getBestSplit(const float3 sah, uint ID, const float4 scale, const ulong defaultSplit)
    {
      ulong splitX = (((ulong)as_uint((float)sah.x())) << 32) | ((uint)ID << 2) | 0;
      ulong splitY = (((ulong)as_uint((float)sah.y())) << 32) | ((uint)ID << 2) | 1;
      ulong splitZ = (((ulong)as_uint((float)sah.z())) << 32) | ((uint)ID << 2) | 2;
	
	
      /* ignore zero sized dimensions */
      splitX = cselect( (ulong)(scale.x() == 0), defaultSplit, splitX);
      splitY = cselect( (ulong)(scale.y() == 0), defaultSplit, splitY);
      splitZ = cselect( (ulong)(scale.z() == 0), defaultSplit, splitZ);
      ulong bestSplit = min(min(splitX,splitY),splitZ);
      bestSplit = sub_group_reduce(bestSplit, SYCL_ONEAPI::minimum<ulong>());      
      return bestSplit;
    }

    EMBREE_SYCL_SIMD(BVH_BINNING_BINS) __forceinline void updateBallot(const uint ID, const gpu::AABB3f &b, AABB3f bounds[BVH_BINNING_BINS])
    {
      uint mask = sub_group_ballot(true);
      while(mask)
      {
        const uint first = sycl::ctz(mask);
        const uint index = sub_group_broadcast(ID,first);
        const bool cmp = ID == index;
        mask &= ~sub_group_ballot(cmp);
        const gpu::AABB3f reduced_bounds = b.sub_group_masked_reduce(cmp);
        if (get_sub_group_local_id() == first)
          reduced_bounds.atomic_merge_local(bounds[index]);
      }        
    }
    
    
    struct BinInfo { // OLD code
      AABB3f boundsX[BVH_BINNING_BINS];
      AABB3f boundsY[BVH_BINNING_BINS];
      AABB3f boundsZ[BVH_BINNING_BINS];
      uint3 counts[BVH_BINNING_BINS];

      EMBREE_SYCL_SIMD(BVH_BINNING_BINS) __forceinline void init()
      {
        for (uint i=get_sub_group_local_id();i<BVH_BINNING_BINS;i+=get_sub_group_size())
        {
          boundsX[i].init();
          boundsY[i].init();
          boundsZ[i].init();
          counts[i] = (uint3)(0);
        }
      }      

      EMBREE_SYCL_SIMD(BVH_BINNING_BINS) __forceinline void atomicUpdate( const gpu::BinMapping &binMapping, const gpu::AABB &primref, const uint inc=1)
      {
        const float4 p = primref.centroid2();
        const float4 bin4 = (p-binMapping.ofs)*binMapping.scale;
        const sycl::uint4 i = bin4.convert<sycl::uint,sycl::rounding_mode::rtz>();
  
        gpu::AABB3f bounds = convert_AABB3f(primref);

        bounds.atomic_merge_local(boundsX[i.x()]);
        bounds.atomic_merge_local(boundsY[i.y()]);
        bounds.atomic_merge_local(boundsZ[i.z()]);

        sycl::atomic_ref<uint, sycl::memory_order::relaxed, sycl::memory_scope::work_group,sycl::access::address_space::local_space> counter_x(counts[i.x()].x());
        sycl::atomic_ref<uint, sycl::memory_order::relaxed, sycl::memory_scope::work_group,sycl::access::address_space::local_space> counter_y(counts[i.y()].y());
        sycl::atomic_ref<uint, sycl::memory_order::relaxed, sycl::memory_scope::work_group,sycl::access::address_space::local_space> counter_z(counts[i.z()].z());

        counter_x.fetch_add(inc);
        counter_y.fetch_add(inc);
        counter_z.fetch_add(inc);
        
      }
      
      
      EMBREE_SYCL_SIMD(BVH_BINNING_BINS) __forceinline void atomicUpdateBallot( const gpu::BinMapping &binMapping, const gpu::AABB &primref, const uint inc=1)
      {
        const float4 p = primref.centroid2();
        const float4 bin4 = (p-binMapping.ofs)*binMapping.scale;
        const sycl::uint4 i = bin4.convert<sycl::uint,sycl::rounding_mode::rtz>();
  
        gpu::AABB3f b = convert_AABB3f(primref);

        updateBallot(i.x(),b,boundsX);
        updateBallot(i.y(),b,boundsY);
        updateBallot(i.z(),b,boundsZ);        

        sycl::atomic_ref<uint, sycl::memory_order::relaxed, sycl::memory_scope::work_group,sycl::access::address_space::local_space> counter_x(counts[0][i.x()]);
        sycl::atomic_ref<uint, sycl::memory_order::relaxed, sycl::memory_scope::work_group,sycl::access::address_space::local_space> counter_y(counts[1][i.y()]);
        sycl::atomic_ref<uint, sycl::memory_order::relaxed, sycl::memory_scope::work_group,sycl::access::address_space::local_space> counter_z(counts[2][i.z()]);

        counter_x.fetch_add(inc);
        counter_y.fetch_add(inc);
        counter_z.fetch_add(inc);        
      }
      
      
      EMBREE_SYCL_SIMD(BVH_BINNING_BINS) __forceinline gpu::Split reduceBinsAndComputeBestSplit(const float4 scale, const uint startID, const uint endID, const uint SAH_LOG_BLOCK_SHIFT) const
      {
        const uint subgroupLocalID = get_sub_group_local_id();		
        const AABB3f &bX      = boundsX[subgroupLocalID];
        const float lr_areaX  = left_to_right_area16(bX);	
        const float rl_areaX  = right_to_left_area16(bX);        
        const AABB3f &bY      = boundsY[subgroupLocalID];
        const float lr_areaY  = left_to_right_area16(bY);
        const float rl_areaY  = right_to_left_area16(bY);
        const AABB3f &bZ      = boundsZ[subgroupLocalID];
        const float lr_areaZ  = left_to_right_area16(bZ);
        const float rl_areaZ  = right_to_left_area16(bZ);
        const uint3 &c = counts[subgroupLocalID];
        const uint lr_countsX = left_to_right_counts16(c.x());
        const uint lr_countsY = left_to_right_counts16(c.y());
        const uint lr_countsZ = left_to_right_counts16(c.z());

#if 0        
        const uint rl_countsX = right_to_left_counts16(c.x());
        const uint rl_countsY = right_to_left_counts16(c.y());  
        const uint rl_countsZ = right_to_left_counts16(c.z());
#else
        const uint numPrims = endID-startID;        
        const uint rl_countsX = numPrims - lr_countsX;    
        const uint rl_countsY = numPrims - lr_countsY;    
        const uint rl_countsZ = numPrims - lr_countsZ;    
#endif        
	
        const uint blocks_shift = SAH_LOG_BLOCK_SHIFT;       
        const uint blocks_add = ((1 << blocks_shift)-1);	
        const uint3 lr_count( (lr_countsX+blocks_add)>>blocks_shift , (lr_countsY+blocks_add)>>blocks_shift, (lr_countsZ+blocks_add)>>blocks_shift );       
        const float3 lr_area(lr_areaX,lr_areaY,lr_areaZ);
        const float3 rl_area(rl_areaX,rl_areaY,rl_areaZ);	
        const uint3 rl_count( (rl_countsX+blocks_add)>>blocks_shift , (rl_countsY+blocks_add)>>blocks_shift, (rl_countsZ+blocks_add)>>blocks_shift );
        const float3 lr_count_f ( (float)lr_count.x(),(float)lr_count.y(),(float)lr_count.z() );
        const float3 rl_count_f ( (float)rl_count.x(),(float)rl_count.y(),(float)rl_count.z() );	

        /* first bin is invalid */
        const float _pos_inf = (float)INFINITY;
        // workaround for compiler bug
        const int m_subgroup = subgroupLocalID != 0 ? -1 : 0;
        const float3 cost = sycl::fma(lr_area,lr_count_f,rl_area*rl_count_f);
        const float3 sah = cselect( int3(m_subgroup), cost , float3(_pos_inf));

        /* select best split */
        const uint mid = (startID+endID)/2;
        const uint maxSAH = as_uint(_pos_inf);	
        const ulong defaultSplit = (((ulong)maxSAH) << 32) | ((uint)mid << 2) | 0;    
        const ulong bestSplit = getBestSplit(sah, subgroupLocalID, scale, defaultSplit);
        const uint bestSplit32 = (uint)(bestSplit >> 32);
        const float split_sah = as_float(bestSplit32);	
        return gpu::Split(split_sah,(uint)bestSplit & 3,(uint)bestSplit >> 2);
      }

      EMBREE_SYCL_SIMD(BVH_BINNING_BINS) __forceinline uint computeSplitPosition(const uint start, const uint end, const gpu::Split &split)
      {
        const uint subgroupLocalID = get_sub_group_local_id();		        
        if (split.sah != (float)(INFINITY))
        {
          uint c = counts[subgroupLocalID].x();
          if (split.dim == 1)
            c =  counts[subgroupLocalID].y();
          else if (split.dim == 2)
            c =  counts[subgroupLocalID].z();
          const uint lr_counts = left_to_right_counts16(c);
          return start + sub_group_broadcast(lr_counts,split.pos);
        }
        return (start+end)/2;
      }

            
    };


    struct BinInfoRegister { // todo: change layout
      AABB3f bounds[3];
      uint counts[3];

      EMBREE_SYCL_SIMD(BVH_BINNING_BINS) __forceinline void init()
      {
        for (uint i=0;i<3;i++)
        {
          bounds[i].init();
          counts[i] = 0;
        }
      }      

      EMBREE_SYCL_SIMD(BVH_BINNING_BINS) __forceinline void update( const gpu::BinMapping &binMapping, const gpu::AABB &primref, const uint inc=1)
      {
        const float4 p = primref.centroid2();
        const float4 bin4 = (p-binMapping.ofs)*binMapping.scale;
        const sycl::uint4 i = bin4.convert<sycl::uint,sycl::rounding_mode::rtz>();
  
        gpu::AABB3f b = convert_AABB3f(primref);
        const uint subgroupLocalID = get_sub_group_local_id();		

        bounds[0].extend(b,i.x());
        bounds[1].extend(b,i.y());
        bounds[2].extend(b,i.z());

        counts[0] += i.x() == subgroupLocalID ? 1 : 0;
        counts[1] += i.y() == subgroupLocalID ? 1 : 0;
        counts[2] += i.z() == subgroupLocalID ? 1 : 0;                
      }
                 
      
      EMBREE_SYCL_SIMD(BVH_BINNING_BINS) __forceinline gpu::Split reduceBinsAndComputeBestSplit(const float4 scale, const uint startID, const uint endID, const uint SAH_LOG_BLOCK_SHIFT) const
      {
        const uint subgroupLocalID = get_sub_group_local_id();		
        const AABB3f &bX      = bounds[0];
        const float lr_areaX  = left_to_right_area16(bX);	
        const float rl_areaX  = right_to_left_area16(bX);
        const AABB3f &bY      = bounds[1];
        const float lr_areaY  = left_to_right_area16(bY);
        const float rl_areaY  = right_to_left_area16(bY);
        const AABB3f &bZ      = bounds[2];
        const float lr_areaZ  = left_to_right_area16(bZ);
        const float rl_areaZ  = right_to_left_area16(bZ);
        const uint lr_countsX = left_to_right_counts16(counts[0]);
        const uint lr_countsY = left_to_right_counts16(counts[1]);
        const uint lr_countsZ = left_to_right_counts16(counts[2]);
        
#if 0        
        const uint rl_countsX = right_to_left_counts16(counts[0]);
        const uint rl_countsY = right_to_left_counts16(counts[1]);  
        const uint rl_countsZ = right_to_left_counts16(counts[2]);
#else
        const uint numPrims = endID-startID;        
        const uint rl_countsX = numPrims - lr_countsX;    
        const uint rl_countsY = numPrims - lr_countsY;    
        const uint rl_countsZ = numPrims - lr_countsZ;    
#endif        
        
	
        const uint blocks_shift = SAH_LOG_BLOCK_SHIFT;       
        const uint blocks_add = ((1 << blocks_shift)-1);	
        const uint3 lr_count( (lr_countsX+blocks_add)>>blocks_shift , (lr_countsY+blocks_add)>>blocks_shift, (lr_countsZ+blocks_add)>>blocks_shift );       
        const float3 lr_area(lr_areaX,lr_areaY,lr_areaZ);
        const float3 rl_area(rl_areaX,rl_areaY,rl_areaZ);	
        const uint3 rl_count( (rl_countsX+blocks_add)>>blocks_shift , (rl_countsY+blocks_add)>>blocks_shift, (rl_countsZ+blocks_add)>>blocks_shift );
        const float3 lr_count_f ( (float)lr_count.x(),(float)lr_count.y(),(float)lr_count.z() );
        const float3 rl_count_f ( (float)rl_count.x(),(float)rl_count.y(),(float)rl_count.z() );	

        /* first bin is invalid */
        const float _pos_inf = (float)INFINITY;
        // workaround for compiler bug
        const int m_subgroup = subgroupLocalID != 0 ? -1 : 0;
        const float3 cost = sycl::fma(lr_area,lr_count_f,rl_area*rl_count_f);
        const float3 sah = cselect( int3(m_subgroup), cost , float3(_pos_inf));

        /* select best split */
        const uint mid = (startID+endID)/2;
        const uint maxSAH = as_uint(_pos_inf);	
        const ulong defaultSplit = (((ulong)maxSAH) << 32) | ((uint)mid << 2) | 0;    
        const ulong bestSplit = getBestSplit(sah, subgroupLocalID, scale, defaultSplit);
        const uint bestSplit32 = (uint)(bestSplit >> 32);
        const float split_sah = as_float(bestSplit32);	
        return gpu::Split(split_sah,(uint)bestSplit & 3,(uint)bestSplit >> 2);
      }
            
    };
    


    
        // ======       - reduce and compute best split avoid rl computations


    struct BinInfoNew {
      
      AABB3f bounds[3][BVH_BINNING_BINS];
      uint counts[3][BVH_BINNING_BINS];

      EMBREE_SYCL_SIMD(BVH_BINNING_BINS) __forceinline void init()
      {
        for (uint i=get_sub_group_local_id();i<BVH_BINNING_BINS;i+=get_sub_group_size())
        {
          bounds[0][i].init();
          bounds[1][i].init();
          bounds[2][i].init();
          counts[0][i] = 0;
          counts[1][i] = 0;
          counts[2][i] = 0;          
        }
      }      

      EMBREE_SYCL_SIMD(BVH_BINNING_BINS) __forceinline void atomicUpdate( const gpu::BinMapping &binMapping, const gpu::AABB &primref, const uint inc=1)
      {
        const float4 p = primref.centroid2();
        const float4 bin4 = (p-binMapping.ofs)*binMapping.scale;
        const sycl::uint4 i = bin4.convert<sycl::uint,sycl::rounding_mode::rtz>();
  
        gpu::AABB3f b = convert_AABB3f(primref);

        b.atomic_merge_local(bounds[0][i.x()]);
        b.atomic_merge_local(bounds[1][i.y()]);
        b.atomic_merge_local(bounds[2][i.z()]);

        sycl::atomic_ref<uint, sycl::memory_order::relaxed, sycl::memory_scope::work_group,sycl::access::address_space::local_space> counter_x(counts[0][i.x()]);
        sycl::atomic_ref<uint, sycl::memory_order::relaxed, sycl::memory_scope::work_group,sycl::access::address_space::local_space> counter_y(counts[1][i.y()]);
        sycl::atomic_ref<uint, sycl::memory_order::relaxed, sycl::memory_scope::work_group,sycl::access::address_space::local_space> counter_z(counts[2][i.z()]);

        counter_x.fetch_add(inc);
        counter_y.fetch_add(inc);
        counter_z.fetch_add(inc);        
      }

      EMBREE_SYCL_SIMD(BVH_BINNING_BINS) __forceinline void atomicUpdateBallot( const gpu::BinMapping &binMapping, const gpu::AABB &primref, const uint inc=1)
      {
        const float4 p = primref.centroid2();
        const float4 bin4 = (p-binMapping.ofs)*binMapping.scale;
        const sycl::uint4 i = bin4.convert<sycl::uint,sycl::rounding_mode::rtz>();
  
        gpu::AABB3f b = convert_AABB3f(primref);


        updateBallot(i.x(),b,bounds[0]);
        updateBallot(i.y(),b,bounds[1]);
        updateBallot(i.z(),b,bounds[2]);        
        
        sycl::atomic_ref<uint, sycl::memory_order::relaxed, sycl::memory_scope::work_group,sycl::access::address_space::local_space> counter_x(counts[0][i.x()]);
        sycl::atomic_ref<uint, sycl::memory_order::relaxed, sycl::memory_scope::work_group,sycl::access::address_space::local_space> counter_y(counts[1][i.y()]);
        sycl::atomic_ref<uint, sycl::memory_order::relaxed, sycl::memory_scope::work_group,sycl::access::address_space::local_space> counter_z(counts[2][i.z()]);

        counter_x.fetch_add(inc);
        counter_y.fetch_add(inc);
        counter_z.fetch_add(inc);        
      }
      

      EMBREE_SYCL_SIMD(BVH_BINNING_BINS) __forceinline void merge(const BinInfoNew &b)
      {
        for (uint i=0;i<3;i++)
        {
          bounds[i][get_sub_group_local_id()].extend(b.bounds[i][get_sub_group_local_id()]);
          counts[i][get_sub_group_local_id()] += b.counts[i][get_sub_group_local_id()];
        }
      }

      EMBREE_SYCL_SIMD(BVH_BINNING_BINS) __forceinline void atomic_merge_local(BinInfoNew &b)
      {
        for (uint i=0;i<3;i++)
        {
          bounds[i][get_sub_group_local_id()].atomic_merge_local(b.bounds[i][get_sub_group_local_id()]);
          sycl::atomic_ref<uint, sycl::memory_order::relaxed, sycl::memory_scope::work_group,sycl::access::address_space::local_space> counter(b.counts[i][get_sub_group_local_id()]);
          counter += counts[i][get_sub_group_local_id()];
        }
      }
      
      EMBREE_SYCL_SIMD(BVH_BINNING_BINS) __forceinline void atomic_merge_global(BinInfoNew &b)
      {
        for (uint i=0;i<3;i++)
        {
          bounds[i][get_sub_group_local_id()].atomic_merge_global(b.bounds[i][get_sub_group_local_id()]);
          sycl::atomic_ref<uint, sycl::memory_order::relaxed, sycl::memory_scope::device,sycl::access::address_space::global_space> counter(b.counts[i][get_sub_group_local_id()]);
          counter += counts[i][get_sub_group_local_id()];
        }
      }

      
      

      EMBREE_SYCL_SIMD(BVH_BINNING_BINS) __forceinline gpu::Split reduceBinsAndComputeBestSplit(const float4 scale, const uint startID, const uint endID, const uint SAH_LOG_BLOCK_SHIFT) const
      {
        const uint subgroupLocalID = get_sub_group_local_id();		
        const AABB3f &bX      = bounds[0][subgroupLocalID];
        const float lr_areaX  = left_to_right_area16(bX);	
        const float rl_areaX  = right_to_left_area16(bX);
        const AABB3f &bY      = bounds[1][subgroupLocalID];
        const float lr_areaY  = left_to_right_area16(bY);
        const float rl_areaY  = right_to_left_area16(bY);
        const AABB3f &bZ      = bounds[2][subgroupLocalID];
        const float lr_areaZ  = left_to_right_area16(bZ);
        const float rl_areaZ  = right_to_left_area16(bZ);
        const uint lr_countsX = left_to_right_counts16(counts[0][subgroupLocalID]);
        const uint lr_countsY = left_to_right_counts16(counts[1][subgroupLocalID]);
        const uint lr_countsZ = left_to_right_counts16(counts[2][subgroupLocalID]);
        
#if 0
        const uint rl_countsX = right_to_left_counts16(counts[0][subgroupLocalID]);
        const uint rl_countsY = right_to_left_counts16(counts[1][subgroupLocalID]);  
        const uint rl_countsZ = right_to_left_counts16(counts[2][subgroupLocalID]);        
#else
        const uint numPrims = endID-startID;        
        const uint rl_countsX = numPrims - lr_countsX;    
        const uint rl_countsY = numPrims - lr_countsY;    
        const uint rl_countsZ = numPrims - lr_countsZ;    
#endif        
        
        const uint blocks_shift = SAH_LOG_BLOCK_SHIFT;       
        const uint blocks_add = ((1 << blocks_shift)-1);	
        const uint3 lr_count( (lr_countsX+blocks_add)>>blocks_shift , (lr_countsY+blocks_add)>>blocks_shift, (lr_countsZ+blocks_add)>>blocks_shift );       
        const float3 lr_area(lr_areaX,lr_areaY,lr_areaZ);
        const float3 rl_area(rl_areaX,rl_areaY,rl_areaZ);	
        const uint3 rl_count( (rl_countsX+blocks_add)>>blocks_shift , (rl_countsY+blocks_add)>>blocks_shift, (rl_countsZ+blocks_add)>>blocks_shift );
        const float3 lr_count_f ( (float)lr_count.x(),(float)lr_count.y(),(float)lr_count.z() );
        const float3 rl_count_f ( (float)rl_count.x(),(float)rl_count.y(),(float)rl_count.z() );	

        /* first bin is invalid */
        const float _pos_inf = (float)INFINITY;
        // workaround for compiler bug
        const int m_subgroup = subgroupLocalID != 0 ? -1 : 0;
        const float3 cost = sycl::fma(lr_area,lr_count_f,rl_area*rl_count_f);
        const float3 sah = cselect( int3(m_subgroup), cost , float3(_pos_inf));

        /* select best split */
        const uint mid = (startID+endID)/2;
        const uint maxSAH = as_uint(_pos_inf);	
        const ulong defaultSplit = (((ulong)maxSAH) << 32) | ((uint)mid << 2) | 0;    
        const ulong bestSplit = getBestSplit(sah, subgroupLocalID, scale, defaultSplit);
        const uint bestSplit32 = (uint)(bestSplit >> 32);
        const float split_sah = as_float(bestSplit32);	
        return gpu::Split(split_sah,(uint)bestSplit & 3,(uint)bestSplit >> 2);
      }

      EMBREE_SYCL_SIMD(BVH_BINNING_BINS) __forceinline uint computeSplitPosition(const uint start, const uint end, const gpu::Split &split)
      {
        const uint subgroupLocalID = get_sub_group_local_id();		        
        if (split.sah != (float)(INFINITY))
        {
          const uint lr_counts = left_to_right_counts16(counts[split.dim][subgroupLocalID]);
          return start + sub_group_broadcast(lr_counts,split.pos);
        }
        return (start+end)/2;
      }

      EMBREE_SYCL_SIMD(BVH_BINNING_BINS) __forceinline uint computeItemsLeft(const gpu::Split &split)
      {
        const uint subgroupLocalID = get_sub_group_local_id();		        
        const uint lr_counts = left_to_right_counts16(counts[split.dim][subgroupLocalID]);
        return sub_group_broadcast(lr_counts,split.pos);
      }

      EMBREE_SYCL_SIMD(BVH_BINNING_BINS) __forceinline uint computeItemsRight(const gpu::Split &split)
      {
        const uint subgroupLocalID = get_sub_group_local_id();		        
        const uint rl_counts = right_to_left_counts16(counts[split.dim][subgroupLocalID]);
        return sub_group_broadcast(rl_counts,split.pos);
      }      
      
    };


    
    struct BinInfo2 {
      AABB3f boundsX[BVH_BINNING_BINS*2];
      AABB3f boundsY[BVH_BINNING_BINS*2];
      AABB3f boundsZ[BVH_BINNING_BINS*2];
      uint3 counts[BVH_BINNING_BINS*2];

      EMBREE_SYCL_SIMD(BVH_BINNING_BINS) __forceinline void init()
      {
        for (uint i=get_sub_group_local_id();i<BVH_BINNING_BINS*2;i+=get_sub_group_size())          
        {
          boundsX[i].init();
          boundsY[i].init();
          boundsZ[i].init();
          counts[i] = (uint3)(0);
        }
      }

      EMBREE_SYCL_SIMD(BVH_BINNING_BINS) __forceinline void atomicUpdate(const gpu::BinMapping &binMapping, const gpu::AABB &primref)
      {
        const float4 p = primref.centroid2();
        const float4 bin4 = (p-binMapping.ofs)*binMapping.scale;
        const sycl::uint4 i = bin4.convert<sycl::uint,sycl::rounding_mode::rtz>();
  
        gpu::AABB3f bounds = convert_AABB3f(primref);

        bounds.atomic_merge_local(boundsX[i.x()]);
        bounds.atomic_merge_local(boundsY[i.y()]);
        bounds.atomic_merge_local(boundsZ[i.z()]);

        gpu::atomic_add_local<uint>((uint *)&counts[i.x()].x(),1);
        gpu::atomic_add_local<uint>((uint *)&counts[i.y()].y(),1);
        gpu::atomic_add_local<uint>((uint *)&counts[i.z()].z(),1);        
      }
      
      EMBREE_SYCL_SIMD(BVH_BINNING_BINS) __forceinline gpu::Split reduceBinsAndComputeBestSplit(const float4 scale, const uint startID, const uint endID, const uint SAH_LOG_BLOCK_SHIFT) // need __forceinline due to CMPLRXDEPS-749
      {
        const uint subgroupLocalID = get_sub_group_local_id(); 
        const uint subgroup_size   = BVH_BINNING_BINS;  

        AABB3f boundsX_low  = boundsX[subgroupLocalID              ];
        AABB3f boundsX_high = boundsX[subgroupLocalID+subgroup_size];

        const float2 lr_areaX = left_to_right_area32(boundsX_low,boundsX_high);
        const float2 rl_areaX = right_to_left_area32(boundsX_low,boundsX_high);
  
        AABB3f boundsY_low  = boundsY[subgroupLocalID              ];
        AABB3f boundsY_high = boundsY[subgroupLocalID+subgroup_size];

        const float2 lr_areaY = left_to_right_area32(boundsY_low,boundsY_high);
        const float2 rl_areaY = right_to_left_area32(boundsY_low,boundsY_high);
  
        AABB3f boundsZ_low  = boundsZ[subgroupLocalID              ];
        AABB3f boundsZ_high = boundsZ[subgroupLocalID+subgroup_size];

        const float2 lr_areaZ = left_to_right_area32(boundsZ_low,boundsZ_high);
        const float2 rl_areaZ = right_to_left_area32(boundsZ_low,boundsZ_high);
  
        const uint3 counts_low  = counts[subgroupLocalID              ];
        const uint3 counts_high = counts[subgroupLocalID+subgroup_size];
	
        const uint2 lr_countsX = left_to_right_counts32(counts_low.x(),counts_high.x());
        const uint2 rl_countsX = right_to_left_counts32(counts_low.x(),counts_high.x());
        const uint2 lr_countsY = left_to_right_counts32(counts_low.y(),counts_high.y());
        const uint2 rl_countsY = right_to_left_counts32(counts_low.y(),counts_high.y());  
        const uint2 lr_countsZ = left_to_right_counts32(counts_low.z(),counts_high.z());
        const uint2 rl_countsZ = right_to_left_counts32(counts_low.z(),counts_high.z());
  
        const uint blocks_shift = SAH_LOG_BLOCK_SHIFT;  
        uint3 blocks_add = (uint3)((1 << blocks_shift)-1);
        const float _pos_inf = (float)INFINITY;

        /* low part: bins 0..15 */
        const float3 lr_area_low(lr_areaX.x(),lr_areaY.x(),lr_areaZ.x());
        const float3 rl_area_low(rl_areaX.x(),rl_areaY.x(),rl_areaZ.x());
        const uint3 lr_count_low = (uint3(lr_countsX.x(),lr_countsY.x(),lr_countsZ.x())+blocks_add) >> blocks_shift;
        const uint3 rl_count_low = (uint3(rl_countsX.x(),rl_countsY.x(),rl_countsZ.x())+blocks_add) >> blocks_shift;
	
        const float3 lr_count_low_f = float3( (float)lr_count_low.x(),(float)lr_count_low.y(),(float)lr_count_low.z() );
        const float3 rl_count_low_f = float3( (float)rl_count_low.x(),(float)rl_count_low.y(),(float)rl_count_low.z() );

        float3 sah_low     = sycl::fma(lr_area_low,lr_count_low_f,rl_area_low*rl_count_low_f);

        /* first bin is invalid */
        // workaround for compiler bug
        const int m_subgroup = subgroupLocalID != 0 ? -1 : 0;	
        sah_low = cselect( int3(m_subgroup), sah_low, float3(_pos_inf));

        /* high part: bins 16..31 */
        const float3 lr_area_high(lr_areaX.y(),lr_areaY.y(),lr_areaZ.y());
        const float3 rl_area_high(rl_areaX.y(),rl_areaY.y(),rl_areaZ.y());
        const uint3 lr_count_high = (uint3(lr_countsX.y(),lr_countsY.y(),lr_countsZ.y())+blocks_add) >> blocks_shift;
        const uint3 rl_count_high = (uint3(rl_countsX.y(),rl_countsY.y(),rl_countsZ.y())+blocks_add) >> blocks_shift;
        const float3 lr_count_high_f = float3( (float)lr_count_high.x(),(float)lr_count_high.y(),(float)lr_count_high.z() );
        const float3 rl_count_high_f = float3( (float)rl_count_high.x(),(float)rl_count_high.y(),(float)rl_count_high.z() );	
        const float3 sah_high        = fma(lr_area_high,lr_count_high_f,rl_area_high*rl_count_high_f);

        const uint mid = (startID+endID)/2;
        const ulong defaultSplit = (((ulong)as_uint((float)(INFINITY))) << 32) | ((uint)mid << 2) | 0;  
  
        const ulong bestSplit_low  = getBestSplit(sah_low ,subgroupLocalID,scale,defaultSplit);
        const ulong bestSplit_high = getBestSplit(sah_high,subgroupLocalID+subgroup_size,scale,defaultSplit);
        const ulong bestSplit = min(bestSplit_low,bestSplit_high);
        const uint bestSplit32 = (uint)(bestSplit >> 32);
        const float split_sah = as_float(bestSplit32);	
        return gpu::Split(split_sah,(uint)bestSplit & 3,(uint)bestSplit >> 2);
      }
      
    };

#if 0
    __forceinline ulong encodeOffset(char *bvh_mem, GBVHN<BVH_NODE_N>::NodeRef *parent, ulong global_child_offset)
    {
#if 0      
      ulong global_parent_offset = (ulong)parent - (ulong)bvh_mem;
      global_parent_offset = global_parent_offset & (~127); 
      ulong relative_offset = global_child_offset - global_parent_offset;
      return relative_offset;
#else
      return (ulong)global_child_offset + (ulong)bvh_mem;
      //return (ulong)global_child_offset;
#endif      
    }

    __forceinline ulong createLeaf(const Globals &globals,
                            char *bvh_mem,
                            GBVHN<BVH_NODE_N>::NodeRef *parent,
                            const uint start,
                            const uint items)
    {
      const ulong global_child_offset = globals.leaf_mem_allocator_start + start * globals.leafSize;            
#if 0      
      ulong global_parent_offset = (ulong)parent - (ulong)bvh_mem;
      global_parent_offset = global_parent_offset & (~127);      
      const ulong offset = global_child_offset - global_parent_offset;
#else
      const ulong offset = (ulong)bvh_mem + global_child_offset;
      //const ulong offset = global_child_offset;      
#endif      
      const ulong final = offset | (ulong)GBVHN<BVH_NODE_N>::NodeRef::ISLEAF | (((ulong)items-1)<<NodeRef::ITEMS_SHIFT);
      return final;
    }
    
    __forceinline ulong createNode(Globals &globals, const uint ID, AABB *childrenAABB, uint numChildren, char *bvh_mem)
    {
      const uint subgroupLocalID = get_sub_group_local_id();
      ulong node_offset = 0;
      if (subgroupLocalID == 0)
      {
        node_offset = globals.alloc_node_mem(sizeof(gpu::QBVHNodeN));
      }
      node_offset = sub_group_broadcast(node_offset,0); 
      gpu::QBVHNodeN &node = *(gpu::QBVHNodeN*)(bvh_mem + node_offset);
      gpu::QBVHNodeN::init(node,childrenAABB,numChildren,ID);
      return node_offset;      
    }

    __forceinline bool checkPrimRefBounds(const BuildRecord &record, const AABB &geometryBounds, const AABB &primref)
    {
      const float4 centroid2 = primref.lower+primref.upper;

      if (centroid2.x() < record.centroidBounds.lower.x()) return false;
      if (centroid2.y() < record.centroidBounds.lower.y()) return false;
      if (centroid2.z() < record.centroidBounds.lower.z()) return false;

      if (centroid2.x() > record.centroidBounds.upper.x()) return false;
      if (centroid2.y() > record.centroidBounds.upper.y()) return false;
      if (centroid2.z() > record.centroidBounds.upper.z()) return false;

      if (primref.lower.x() < geometryBounds.lower.x()) return false;
      if (primref.lower.y() < geometryBounds.lower.y()) return false;
      if (primref.lower.z() < geometryBounds.lower.z()) return false;

      if (primref.upper.x() > geometryBounds.upper.x()) return false;
      if (primref.upper.y() > geometryBounds.upper.y()) return false;
      if (primref.upper.z() > geometryBounds.upper.z()) return false;
 
      return true;  
    }
    

#endif

    const uint shuffle2mirror[16]  = { 1,0,3,2,5,4,7,6,9,8,11,10,13,12,15,14 };
    const uint shuffle4mirror[16]  = { 3,2,1,0,7,6,5,4,11,10,9,8,15,14,13,12 };
    const uint shuffle4rotate[16]  = { 2,3,0,1,6,7,4,5,10,11,8,9,14,15,12,13 };
    const uint shuffle8mirror[16]  = { 7,6,5,4,3,2,1,0,15,14,13,12,11,10,9,8 };
    const uint shuffle8rotate[16]  = { 4,5,6,7,0,1,2,3,12,13,14,15,8,9,10,11 };
    const uint shuffle16mirror[16] = { 15,14,13,12,11,10,9,8,7,6,5,4,3,2,1,0 };

    const uint selGo2[16]  = { 0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1 };
    const uint selGo4[16]  = { 0,0,1,1,0,0,1,1,0,0,1,1,0,0,1,1 };
    const uint selGo8[16]  = { 0,0,0,0,1,1,1,1,0,0,0,0,1,1,1,1 };
    const uint selGo16[16] = { 0,0,0,0,0,0,0,0,1,1,1,1,1,1,1,1 };
     
    __forceinline uint compare_exchange(const uint a0, const uint shuffleMask, const uint selectMask, const bool ascending)
    {
      const uint a1 = embree::sub_group_shuffle(a0,shuffleMask);
      const uint a_min = min(a0,a1);
      const uint a_max = max(a0,a1);
      return sycl::select(ascending ? a_min : a_max,ascending ? a_max : a_min,selectMask);
    }

    __forceinline uint sort16(const uint aa, const bool ascending)
    { 
      const uint slotID = get_sub_group_local_id();		
      const uint bb = compare_exchange(aa,shuffle2mirror[slotID],selGo2[slotID],ascending);
      const uint cc = compare_exchange(bb,shuffle4mirror[slotID],selGo4[slotID],ascending);
      const uint dd = compare_exchange(cc,shuffle2mirror[slotID],selGo2[slotID],ascending);
      const uint ee = compare_exchange(dd,shuffle8mirror[slotID],selGo8[slotID],ascending);
      const uint ff = compare_exchange(ee,shuffle4rotate[slotID],selGo4[slotID],ascending);
      const uint gg = compare_exchange(ff,shuffle2mirror[slotID],selGo2[slotID],ascending);
      const uint hh = compare_exchange(gg,shuffle16mirror[slotID],selGo16[slotID],ascending);
      const uint ii = compare_exchange(hh,shuffle8rotate[slotID],selGo8[slotID],ascending);
      const uint jj = compare_exchange(ii,shuffle4rotate[slotID],selGo4[slotID],ascending);
      const uint kk = compare_exchange(jj,shuffle2mirror[slotID],selGo2[slotID],ascending);
      return kk;
       
    }

    __forceinline uint sort8(const uint aa, const bool ascending)
    {
      const uint slotID = get_sub_group_local_id();		
      const uint bb = compare_exchange(aa,shuffle2mirror[slotID],selGo2[slotID],ascending);
      const uint cc = compare_exchange(bb,shuffle4mirror[slotID],selGo4[slotID],ascending);
      const uint dd = compare_exchange(cc,shuffle2mirror[slotID],selGo2[slotID],ascending);
      const uint ee = compare_exchange(dd,shuffle8mirror[slotID],selGo8[slotID],ascending);
      const uint ff = compare_exchange(ee,shuffle4rotate[slotID],selGo4[slotID],ascending);
      const uint gg = compare_exchange(ff,shuffle2mirror[slotID],selGo2[slotID],ascending);
      return gg;
    }

    __forceinline uint sort4(const uint aa, const bool ascending)
    {
      const uint slotID = get_sub_group_local_id();		
      const uint bb = compare_exchange(aa,shuffle2mirror[slotID],selGo2[slotID],ascending);
      const uint cc = compare_exchange(bb,shuffle4mirror[slotID],selGo4[slotID],ascending);
      const uint dd = compare_exchange(cc,shuffle2mirror[slotID],selGo2[slotID],ascending);
      return dd;
    }

    __forceinline uint sortBVHChildrenIDs(uint input)
    {
#if BVH_NODE_N == 16
      return sort16(input,false);
#elif BVH_NODE_N == 8
      return sort8(input,false);
#else
      return sort4(input,false);  
#endif
    }

    struct __aligned(64) WorkGroupBuildState {

      gpu::BinInfoNew binInfo;
      gpu::AABB3f leftBounds;
      gpu::AABB3f rightBounds;
      uint atomicCountLeft;
      uint atomicCountRight;    
      uint numAssignedWGs;
      uint numDoneWGs;
    
      __forceinline void init() {
        binInfo.init();
        leftBounds.init();
        rightBounds.init();
        atomicCountLeft = 0;
        atomicCountRight = 0;
      }    
    };

    struct __aligned(16) WorkGroupTaskState {

      uint recordID;
      uint buildStateID;    
      uint start;
      uint end;    
    
      __forceinline void init() {
        recordID = 0;
        buildStateID = 0;
        start = 0;
        end = 0;
      }    
    };

    
  };


  __forceinline bool is_left(const gpu::BinMapping &binMapping, const gpu::Split &split, const gpu::AABB &primref)
  {
    const uint   dim  = split.dim;
    const float lower = primref.lower[dim];    
    const float upper = primref.upper[dim];
    const float c     = lower+upper;
    const uint pos    = (uint)sycl::floor((c-binMapping.ofs[dim])*binMapping.scale[dim]);
    return pos < split.pos;    
  }

  EMBREE_SYCL_SIMD(BVH_BINNING_BINS) __forceinline void bin_partition_register(sycl::nd_item<1> &item, gpu::BVH2BuildRecord &current, uint &atomicCountLeft, uint &atomicCountRight, gpu::AABB3f &leftBounds, gpu::AABB3f &rightBounds, const gpu::AABB *const primref, uint *const primref_index0, uint *const primref_index1, const uint cfg_SAHLogBlockShift, const uint cfg_maxLeafSize)
  {
    const uint subgroupLocalID = get_sub_group_local_id();		 
    const uint startID = current.start;
    const uint endID   = current.end;
                                                           
    gpu::BinMapping binMapping;    
    binMapping.init(gpu::convert_AABB(current.bounds),BVH_BINNING_BINS);    

    /* init bininfo */        
    gpu::BinInfoRegister binInfo;
    binInfo.init();
                                                               
    /* bin primrefs */
    for (uint t=startID;t<endID;t++)
    {
      const uint index = primref_index0[t];
      primref_index1[t] = index;      
      binInfo.update(binMapping,primref[index]);
    }

    /* find best split */    
    const gpu::Split split = binInfo.reduceBinsAndComputeBestSplit(binMapping.scale,startID,endID,cfg_SAHLogBlockShift);
    
    atomicCountLeft  = 0;
    atomicCountRight = 0;
                                                                                                                                                                            
    gpu::AABB3f lbounds,rbounds;
    
    lbounds.init();
    rbounds.init();
    
    if (split.sah != (float)(INFINITY))
    {
      /* sah split */                                                                
      for (uint t=startID + subgroupLocalID;t<endID;t+=get_sub_group_size())
      {
        const uint index = primref_index1[t];
        const gpu::AABB pref = primref[index];
                                                                 
        if (is_left(binMapping, split,pref)) 
        {
          sycl::atomic_ref<uint, sycl::memory_order::relaxed, sycl::memory_scope::sub_group,sycl::access::address_space::local_space> counter(atomicCountLeft);
          const uint left_index = counter.fetch_add(1);
          lbounds.extend(pref.centroid2());
          primref_index0[startID + left_index] = index;
        }
        else
        {
          sycl::atomic_ref<uint, sycl::memory_order::relaxed, sycl::memory_scope::sub_group,sycl::access::address_space::local_space> counter(atomicCountRight);
          const uint right_index = counter.fetch_add(1);                                                                   
          rbounds.extend(pref.centroid2());
          primref_index0[endID - 1 - right_index] = index;
        }
      }
                                                             
    }
    else
    {
      /* fallback split */
                                                             
      const uint mid = (startID+endID)/2;
      atomicCountLeft = mid-startID;
                                                             
      for (uint t=startID + subgroupLocalID;t<mid;t+=get_sub_group_size())
      {
        const uint index = primref_index1[t];
        lbounds.extend(primref[index].centroid2());
      }
      for (uint t=mid + subgroupLocalID;t<endID;t+=get_sub_group_size())
      {
        const uint index = primref_index1[t];
        rbounds.extend(primref[index].centroid2());
      }
    }
    leftBounds = lbounds.sub_group_reduce();
    rightBounds = rbounds.sub_group_reduce();
        
  }

  __forceinline void bin_partition_local(sycl::nd_item<1> &item, gpu::BVH2BuildRecord &current, gpu::BinInfoNew &binInfo, gpu::Split &bestSplit, uint &atomicCountLeft, uint &atomicCountRight, gpu::AABB3f &leftBounds, gpu::AABB3f &rightBounds, const gpu::AABB *const primref, uint *const primref_index0, uint *const primref_index1, const uint cfg_SAHLogBlockShift, const uint cfg_maxLeafSize)
  {
    const uint localID     = item.get_local_id(0);                                                         
    const uint subgroupID  = get_sub_group_id();
    const uint localSize   = item.get_local_range().size();

    const uint startID = current.start;
    const uint endID   = current.end;
                                                           
    gpu::BinMapping binMapping;    
    binMapping.init(gpu::convert_AABB(current.bounds),BVH_BINNING_BINS);    

    
    /* init bininfo */    
    if (subgroupID == 0)
      binInfo.init();
                                                         
    item.barrier(sycl::access::fence_space::local_space);
                                                           
    /* bin primrefs */
    for (uint t=startID + localID;t<endID;t+=localSize)
    {
      const uint index = primref_index0[t];
      primref_index1[t] = index;      
      binInfo.atomicUpdateBallot(binMapping,primref[index]);
    }
                                                           
    item.barrier(sycl::access::fence_space::local_space);

                                                         
    leftBounds.init();
    rightBounds.init();
                                                           
    atomicCountLeft  = 0;
    atomicCountRight = 0;
                                                           

    /* find best split */
    if (subgroupID == 0)
    {
      bestSplit = binInfo.reduceBinsAndComputeBestSplit(binMapping.scale,startID,endID,cfg_SAHLogBlockShift);
    }
                                                           
    item.barrier(sycl::access::fence_space::local_space);
                                                      
    const gpu::Split split = bestSplit;

    gpu::AABB3f lbounds, rbounds;
    lbounds.init();
    rbounds.init();

    if (split.sah != (float)(INFINITY))
    {
      /* sah split */                                                                
      for (uint t=startID + localID;t<endID;t+=localSize)
      {
        const uint index = primref_index1[t];
        const gpu::AABB pref = primref[index];
                                                                 
        if (is_left(binMapping, split,pref)) 
        {
          sycl::atomic_ref<uint, sycl::memory_order::relaxed, sycl::memory_scope::work_group,sycl::access::address_space::local_space> counter(atomicCountLeft);
          const uint left_index = counter.fetch_add(1);
          lbounds.extend(pref.centroid2());
          primref_index0[startID + left_index] = index;
        }
        else
        {
          sycl::atomic_ref<uint, sycl::memory_order::relaxed, sycl::memory_scope::work_group,sycl::access::address_space::local_space> counter(atomicCountRight);
          const uint right_index = counter.fetch_add(1);                                                                   
          rbounds.extend(pref.centroid2());
          primref_index0[endID - 1 - right_index] = index;
        }
      }
                                                             
    }
    else
    {
      /* fallback split */
                                                             
      const uint mid = (startID+endID)/2;
      atomicCountLeft = mid-startID;
                                                             
      for (uint t=startID + localID;t<mid;t+=localSize)
      {
        const uint index = primref_index1[t];
        lbounds.extend(primref[index].centroid2());
      }
      for (uint t=mid + localID;t<endID;t+=localSize)
      {
        const uint index = primref_index1[t];
        rbounds.extend(primref[index].centroid2());
      }
    }

    // const uint subgroupLocalID = get_sub_group_local_id();    
    // lbounds.sub_group_reduce();
    // rbounds.sub_group_reduce();
    
    // if (subgroupLocalID == 0)
    // {
      lbounds.atomic_merge_local(leftBounds);
      rbounds.atomic_merge_local(rightBounds);
    // }
    
    item.barrier(sycl::access::fence_space::local_space);
  }
  


  template<typename T>
  __forceinline std::pair<sycl::event, sycl::event> exclusive_prefix_sum2(sycl::queue &gpu_queue, T *array, const uint numPrims, T *scratch_mem, bool sync = true)
  {
    static const uint PREFIX_SUM_SUB_GROUP_WIDTH = 32;
    static const uint PREFIX_SUM_WG_SIZE         = 1024;
    static const uint PREFIX_SUM_WG_NUM          = 64;

    std::pair<sycl::event, sycl::event> out;

    sycl::event queue_event = gpu_queue.submit([&](sycl::handler &cgh) {
        sycl::local_accessor< T       , 1> counts(sycl::range<1>((PREFIX_SUM_WG_SIZE/PREFIX_SUM_SUB_GROUP_WIDTH)),cgh);
        sycl::local_accessor< T       , 1> counts_prefix_sum(sycl::range<1>((PREFIX_SUM_WG_SIZE/PREFIX_SUM_SUB_GROUP_WIDTH)),cgh);
        
        const sycl::nd_range<1> nd_range(sycl::range<1>(PREFIX_SUM_WG_NUM*PREFIX_SUM_WG_SIZE),sycl::range<1>(PREFIX_SUM_WG_SIZE));		  
        cgh.parallel_for(nd_range,[=](sycl::nd_item<1> item) EMBREE_SYCL_SIMD(PREFIX_SUM_SUB_GROUP_WIDTH) {
            const uint localID        = item.get_local_id(0);
            const uint groupID        = item.get_group(0);                                                             
            const uint subgroupID      = get_sub_group_id();                                                                                                                          
            const uint subgroupLocalID = get_sub_group_local_id();
            const uint subgroupSize    = get_sub_group_size();                                                                                                                          
            
            const uint startID        = (groupID + 0)*numPrims / PREFIX_SUM_WG_NUM;
            const uint endID          = (groupID + 1)*numPrims / PREFIX_SUM_WG_NUM;
            const uint sizeID         = endID-startID;
            const uint aligned_sizeID = gpu::alignTo(sizeID,PREFIX_SUM_WG_SIZE);

            T total_offset = 0;                                                                 
            for (uint t=0;t<aligned_sizeID;t+=PREFIX_SUM_WG_SIZE)
            {
              const uint ID = startID + t + localID;
              T value = ID < endID ? array[ID] : 0;
              
              const uint exclusive_scan = sub_group_exclusive_scan(value, std::plus<T>());
              const uint reduction = sub_group_reduce(value, std::plus<T>());                                                     
              counts[subgroupID] = reduction;
                                                             
              item.barrier(sycl::access::fence_space::local_space);
              
              /* -- prefix sum over reduced sub group counts -- */
              uint total_reduction = 0;        
              for (uint j=subgroupLocalID;j<PREFIX_SUM_WG_SIZE/subgroupSize;j+=subgroupSize)
              {
                const T subgroup_counts = counts[j];
                const T sums_exclusive_scan = sub_group_exclusive_scan(subgroup_counts, std::plus<T>());
                const T reduction = sub_group_broadcast(subgroup_counts,subgroupSize-1) + sub_group_broadcast(sums_exclusive_scan,subgroupSize-1);
                counts_prefix_sum[j] = sums_exclusive_scan + total_reduction;
                total_reduction += reduction;
              }

              item.barrier(sycl::access::fence_space::local_space);

              const T sums_prefix_sum = counts_prefix_sum[subgroupID];                                                                 
              const uint p_sum = /*startID +*/ total_offset + sums_prefix_sum + exclusive_scan;

              /* --- store cluster representative into destination array --- */                                                                 
              if (ID < endID)
                array[ID] = p_sum;
          
              total_offset += total_reduction;                                          
            }

            /* ---------------------------------------------------------------------- */                                                       
            /* --- store number of valid cluster representatives into scratch mem --- */
            /* ---------------------------------------------------------------------- */

            item.barrier(sycl::access::fence_space::local_space);
                                                     
            if (localID == 0) 
              scratch_mem[groupID] = total_offset; 
                                                     
          });		  
      });
    if (sync)
      gpu::waitOnQueueAndCatchException(gpu_queue);
    out.first = queue_event;

    queue_event = gpu_queue.submit([&](sycl::handler &cgh) {
                                     const sycl::nd_range<1> nd_range(PREFIX_SUM_WG_NUM*PREFIX_SUM_WG_SIZE,sycl::range<1>(PREFIX_SUM_WG_SIZE));
                                     /* local variables */
                                     sycl::local_accessor< T   , 1> global_wg_prefix_sum(sycl::range<1>(PREFIX_SUM_WG_NUM),cgh);
                                                         
                                     cgh.parallel_for(nd_range,[=](sycl::nd_item<1> item) EMBREE_SYCL_SIMD(PREFIX_SUM_SUB_GROUP_WIDTH) {
                                         const uint localID         = item.get_local_id(0);
                                         const uint groupID         = item.get_group(0);                                                             
                                         const uint subgroupLocalID = get_sub_group_local_id();
                                         const uint subgroupSize    = get_sub_group_size();                                                                                                                          
                                         const uint startID = (groupID + 0)*numPrims / PREFIX_SUM_WG_NUM;
                                         const uint endID   = (groupID + 1)*numPrims / PREFIX_SUM_WG_NUM;
                                         const uint sizeID         = endID-startID;
                                         const uint aligned_sizeID = gpu::alignTo(sizeID,PREFIX_SUM_WG_SIZE);
                                         
                                         /* ---------------------------------------------------- */                                                       
                                         /* --- prefix sum over per WG counts in scratch mem --- */
                                         /* ---------------------------------------------------- */
                                         
                                         T global_total = 0;
                                         for (uint i=0;i<PREFIX_SUM_WG_NUM;i+=subgroupSize)
                                         {
                                           const T subgroup_counts     = i+subgroupLocalID < PREFIX_SUM_WG_NUM ? scratch_mem[i+subgroupLocalID] : 0;

                                           const T total               = sub_group_reduce(subgroup_counts, std::plus<T>());
                                           const T sums_exclusive_scan = sub_group_exclusive_scan(subgroup_counts, std::plus<T>());

                                           global_wg_prefix_sum[i+subgroupLocalID] = sums_exclusive_scan + global_total;
                                           global_total += total;                                           
                                         }
            
                                         const T active_count = scratch_mem[groupID];
                                         item.barrier(sycl::access::fence_space::local_space);

                                         const T global_offset = global_wg_prefix_sum[groupID];
                                         
                                         for (uint t=0;t<aligned_sizeID;t+=PREFIX_SUM_WG_SIZE)
                                         {
                                           const uint ID = startID + t + localID;
                                           if (ID < endID)
                                             array[ID] += global_offset;
                                         }
                                       });		  
                                   });
    if (sync)
      gpu::waitOnQueueAndCatchException(gpu_queue);              
    out.second = queue_event;
    return out;
  }


  template<typename T>
  __forceinline std::pair<sycl::event, sycl::event> inclusive_prefix_sum(sycl::queue &gpu_queue, T *array, const uint numPrims, T *scratch_mem, bool sync = true)
  {
    static const uint PREFIX_SUM_SUB_GROUP_WIDTH = 32;
    static const uint PREFIX_SUM_WG_SIZE         = 1024;
    static const uint PREFIX_SUM_WG_NUM          = 64;

    std::pair<sycl::event, sycl::event> out;

    sycl::event queue_event = gpu_queue.submit([&](sycl::handler &cgh) {
                                                 sycl::local_accessor< T       , 1> counts(sycl::range<1>((PREFIX_SUM_WG_SIZE/PREFIX_SUM_SUB_GROUP_WIDTH)),cgh);
                                                 sycl::local_accessor< T       , 1> counts_prefix_sum(sycl::range<1>((PREFIX_SUM_WG_SIZE/PREFIX_SUM_SUB_GROUP_WIDTH)),cgh);
        
                                                 const sycl::nd_range<1> nd_range(sycl::range<1>(PREFIX_SUM_WG_NUM*PREFIX_SUM_WG_SIZE),sycl::range<1>(PREFIX_SUM_WG_SIZE));		  
                                                 cgh.parallel_for(nd_range,[=](sycl::nd_item<1> item) EMBREE_SYCL_SIMD(PREFIX_SUM_SUB_GROUP_WIDTH) {
                                                     const uint localID        = item.get_local_id(0);
                                                     const uint groupID        = item.get_group(0);                                                             
                                                     const uint subgroupID      = get_sub_group_id();                                                                                                                          
                                                     const uint subgroupLocalID = get_sub_group_local_id();
                                                     const uint subgroupSize    = get_sub_group_size();                                                                                                                          
            
                                                     const uint startID        = (groupID + 0)*numPrims / PREFIX_SUM_WG_NUM;
                                                     const uint endID          = (groupID + 1)*numPrims / PREFIX_SUM_WG_NUM;
                                                     const uint sizeID         = endID-startID;
                                                     const uint aligned_sizeID = gpu::alignTo(sizeID,PREFIX_SUM_WG_SIZE);

                                                     T total_offset = 0;                                                                 
                                                     for (uint t=0;t<aligned_sizeID;t+=PREFIX_SUM_WG_SIZE)
                                                     {
                                                       const uint ID = startID + t + localID;
                                                       T value = ID < endID ? array[ID] : 0;
              
                                                       const T inclusive_scan = sub_group_inclusive_scan(value, std::plus<T>());
                                                       const T reduction = sub_group_reduce(value, std::plus<T>());                                                     
                                                       counts[subgroupID] = reduction;
                                                             
                                                       item.barrier(sycl::access::fence_space::local_space);
              
                                                       /* -- prefix sum over reduced sub group counts -- */
                                                       T total_reduction = 0;        
                                                       for (uint j=subgroupLocalID;j<PREFIX_SUM_WG_SIZE/subgroupSize;j+=subgroupSize)
                                                       {
                                                         const T subgroup_counts = counts[j];
                                                         const T sums_exclusive_scan = sub_group_exclusive_scan(subgroup_counts, std::plus<T>());
                                                         const T reduction = sub_group_broadcast(subgroup_counts,subgroupSize-1) + sub_group_broadcast(sums_exclusive_scan,subgroupSize-1);
                                                         counts_prefix_sum[j] = sums_exclusive_scan + total_reduction;
                                                         total_reduction += reduction;
                                                       }

                                                       item.barrier(sycl::access::fence_space::local_space);

                                                       const T sums_prefix_sum = counts_prefix_sum[subgroupID];                                                                 
                                                       const T p_sum = /*startID +*/ total_offset + sums_prefix_sum + inclusive_scan;

                                                       /* --- store cluster representative into destination array --- */                                                                 
                                                       if (ID < endID)
                                                         array[ID] = p_sum;
          
                                                       total_offset += total_reduction;                                          
                                                     }

                                                     /* ---------------------------------------------------------------------- */                                                       
                                                     /* --- store number of valid cluster representatives into scratch mem --- */
                                                     /* ---------------------------------------------------------------------- */

                                                     item.barrier(sycl::access::fence_space::local_space);
                                                     
                                                     if (localID == 0) 
                                                       scratch_mem[groupID] = total_offset; 
                                                     
                                                   });		  
                                               });
    if (sync)
      gpu::waitOnQueueAndCatchException(gpu_queue);
    out.first = queue_event;

    queue_event = gpu_queue.submit([&](sycl::handler &cgh) {
                                     const sycl::nd_range<1> nd_range(PREFIX_SUM_WG_NUM*PREFIX_SUM_WG_SIZE,sycl::range<1>(PREFIX_SUM_WG_SIZE));
                                     /* local variables */
                                     sycl::local_accessor< T   , 1> global_wg_prefix_sum(sycl::range<1>(PREFIX_SUM_WG_NUM),cgh);
                                                         
                                     cgh.parallel_for(nd_range,[=](sycl::nd_item<1> item) EMBREE_SYCL_SIMD(PREFIX_SUM_SUB_GROUP_WIDTH) {
                                         const uint localID         = item.get_local_id(0);
                                         const uint groupID         = item.get_group(0);                                                             
                                         const uint subgroupLocalID = get_sub_group_local_id();
                                         const uint subgroupSize    = get_sub_group_size();                                                                                                                          
                                         const uint startID = (groupID + 0)*numPrims / PREFIX_SUM_WG_NUM;
                                         const uint endID   = (groupID + 1)*numPrims / PREFIX_SUM_WG_NUM;
                                         const uint sizeID         = endID-startID;
                                         const uint aligned_sizeID = gpu::alignTo(sizeID,PREFIX_SUM_WG_SIZE);
                                         
                                         /* ---------------------------------------------------- */                                                       
                                         /* --- prefix sum over per WG counts in scratch mem --- */
                                         /* ---------------------------------------------------- */
                                         
                                         T global_total = 0;
                                         for (uint i=0;i<PREFIX_SUM_WG_NUM;i+=subgroupSize)
                                         {
                                           const T subgroup_counts     = i+subgroupLocalID < PREFIX_SUM_WG_NUM ? scratch_mem[i+subgroupLocalID] : 0;

                                           const T total               = sub_group_reduce(subgroup_counts, std::plus<T>());
                                           const T sums_exclusive_scan = sub_group_exclusive_scan(subgroup_counts, std::plus<T>());

                                           global_wg_prefix_sum[i+subgroupLocalID] = sums_exclusive_scan + global_total;
                                           global_total += total;                                           
                                         }
            
                                         const T active_count = scratch_mem[groupID];
                                         item.barrier(sycl::access::fence_space::local_space);

                                         const T global_offset = global_wg_prefix_sum[groupID];
                                         
                                         for (uint t=0;t<aligned_sizeID;t+=PREFIX_SUM_WG_SIZE)
                                         {
                                           const uint ID = startID + t + localID;
                                           if (ID < endID)
                                             array[ID] += global_offset;
                                         }
                                       });		  
                                   });
    if (sync)
      gpu::waitOnQueueAndCatchException(gpu_queue);              
    out.second = queue_event;
    return out;
  }

  
  __forceinline sycl::event computeCentroidGeometryBounds(sycl::queue &gpu_queue, gpu::Globals *globals, const gpu::AABB *const aabb, uint *const primref_index, const uint numPrimitives, const bool enableSyncAndProfiling = false)
  {
    const uint wgSize = 256; //1024;
    const uint numDSS = 16;          
    const sycl::nd_range<1> nd_range1(sycl::range<1>(wgSize*numDSS),sycl::range<1>(wgSize));          
    sycl::event queue_event = gpu_queue.submit([&](sycl::handler &cgh) {
                                                 sycl::local_accessor< gpu::AABB, 0> _local_geometry_aabb(cgh);
                                                 sycl::local_accessor< gpu::AABB, 0> _local_centroid_aabb(cgh);
                                                       
                                                 cgh.parallel_for(nd_range1,[=](sycl::nd_item<1> item) EMBREE_SYCL_SIMD(16)
                                                                  {
                                                                    gpu::AABB *local_geometry_aabb = _local_geometry_aabb.get_pointer();
                                                                    gpu::AABB *local_centroid_aabb = _local_centroid_aabb.get_pointer();
                                                                                    
                                                                    const uint localID         = item.get_local_id(0);
                                                                    // const uint subgroupLocalID = get_sub_group_local_id();
                                                                    // const uint subgroupID      = get_sub_group_id();

                                                                    const uint startID = item.get_global_id(0);
                                                                    const uint step    = item.get_global_range().size();
                                                                                    
                                                                    gpu::AABB geometry_aabb;
                                                                    gpu::AABB centroid_aabb;
                                                                    geometry_aabb.init();
                                                                    centroid_aabb.init();
		  
                                                                    for (uint i=startID;i<numPrimitives;i+=step)
                                                                    {
                                                                      const gpu::AABB aabb_geom = aabb[i];
                                                                      const gpu::AABB aabb_centroid(aabb_geom.centroid2());
                                                                      primref_index[i] = i;
                                                                      geometry_aabb.extend(aabb_geom);
                                                                      centroid_aabb.extend(aabb_centroid);		      
                                                                    }

                                                                    geometry_aabb.sub_group_reduce();
                                                                    centroid_aabb.sub_group_reduce();
                                                                                    
                                                                    if (localID == 0)
                                                                    {
                                                                      local_geometry_aabb->init();
                                                                      local_centroid_aabb->init();                                                                                      
                                                                    }
                                                                    item.barrier(sycl::access::fence_space::local_space);
                                                                                      
                                                                    geometry_aabb.atomic_merge_local(*local_geometry_aabb);
                                                                    centroid_aabb.atomic_merge_local(*local_centroid_aabb);
                                                                                    
                                                                    item.barrier(sycl::access::fence_space::local_space);

                                                                    if (localID == 0)
                                                                    {
                                                                      local_geometry_aabb->atomic_merge_global(globals->geometryBounds);
                                                                      local_centroid_aabb->atomic_merge_global(globals->centroidBounds);
                                                                    }
                                                                  });
		  
                                               });
    if (enableSyncAndProfiling)
    {
      gpu::waitOnQueueAndCatchException(gpu_queue);
      double dt = gpu::getDeviceExecutionTiming(queue_event);      
      PRINT2("computeCentroidGeometryBounds", (float)dt);
    }

    return queue_event;
  }

    template<typename type>
  __forceinline sycl::event computeMortonCodes3D(sycl::queue &gpu_queue, gpu::Globals *const globals, type *const mc0, gpu::AABB *const aabb, const uint numPrimitives, double &iteration_time)    
  {
    const uint wgSize = 16; 
    const sycl::nd_range<1> nd_range1(gpu::alignTo(numPrimitives,wgSize),sycl::range<1>(wgSize));              
    sycl::event queue_event = gpu_queue.submit([&](sycl::handler &cgh) {
        cgh.parallel_for(nd_range1,[=](sycl::nd_item<1> item) EMBREE_SYCL_SIMD(32)
                         {
                           const uint globalID     = item.get_global_id(0);                                                                    
                           if (globalID < numPrimitives)
                           {
                             const gpu::AABB3f centroidBounds = convert_AABB3f(globals->centroidBounds);
                             const uint i = globalID;

                             /* get lower and upper bounds of geometry and length of scene diagonal */
                             const float3 lower = centroidBounds.lower();                                                                      
                             const uint   grid_size   = 1 << type::GRID_SHIFT;
                             const float3 grid_base(lower.x(),lower.y(),lower.z());
                             const float3 grid_extend(centroidBounds.maxDiagDim());                             
                             const float3 grid_scale = cselect( int3(grid_extend != 0.0f), (grid_size * 0.99f)/grid_extend, float3(0.0f)); // FIXME: 0.99f!!!!!

                             /* calculate and store morton code */                             
                             const uint index = i;
                             const gpu::AABB3f bounds = convert_AABB3f(aabb[index]);
                             const float3 centroid = bounds.centroid2();
                             const float3 gridpos_f = (centroid-grid_base)*grid_scale;                                                                      
                             const uint3 gridpos = gridpos_f.convert<sycl::uint,sycl::rounding_mode::rtz>();
                             //const uint64_t code = gpu::bitInterleave3D_64bits(gridpos);                             
                             mc0[i] = type(gridpos,index);                                        
                           }
                         });
		  
      });
    gpu::waitOnQueueAndCatchException(gpu_queue);
    double dt = gpu::getDeviceExecutionTiming(queue_event);      
    PRINT2("compute 3D morton codes ",(float)dt);
    iteration_time += dt;
    return queue_event;
  }
  
};

#endif
