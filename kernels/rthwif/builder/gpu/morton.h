// Copyright 2009-2021 Intel Corporation
// SPDX-License-Identifier: Apache-2.0

#pragma once

#if defined(EMBREE_SYCL_SUPPORT)
#include "../../builder/gpu/common.h"

namespace embree
{
  namespace gpu
  {

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
  }
}

#endif
