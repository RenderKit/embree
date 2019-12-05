// ======================================================================== //
// Copyright 2009-2019 Intel Corporation                                    //
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

#include "../math/math.h"

namespace embree
{
  /* 4-wide SSE integer type */
  template<int N>
  struct vint
  {
    //ALIGNED_STRUCT_(16);
    
    typedef vboolf<N> Bool;
    typedef vint<N>   Int;
    typedef vfloat<N> Float;

    enum  { size = N };             // number of SIMD elements
    //union { __m128i v; int i[N]; }; // data
    int v;

    ////////////////////////////////////////////////////////////////////////////////
    /// Constructors, Assignment & Cast Operators
    ////////////////////////////////////////////////////////////////////////////////
    
    __forceinline vint() {}
    //__forceinline vint(const vint<N>& a) { v = a.v; }
    //__forceinline vint<N>& operator =(const vint<N>& a) { v = a.v; return *this; }

    //__forceinline vint(__m128i a) : v(a) {}
    //__forceinline operator const __m128i&() const { return v; }
    //__forceinline operator       __m128i&()       { return v; }

    __forceinline vint(int a) : v(a) {}
    __forceinline vint(int a, int b, int c, int d)
    {
      const uint lid = __spirv_BuiltInSubgroupLocalInvocationId;
      if (lid == 0) v = a;
      if (lid == 1) v = b;
      if (lid == 2) v = c;
      if (lid == 3) v = d;
    }

    //__forceinline explicit vint(__m128 a) : v(_mm_cvtps_epi32(a)) {}
    //__forceinline explicit vint(const vboolf<N>& a) : v(_mm_castps_si128((__m128)a)) {}

    //__forceinline vint(long long a, long long b) : v(_mm_set_epi64x(b,a)) {}

    vint<N> fix_upper(int c) const {
      return vint<N>(__spirv_BuiltInSubgroupLocalInvocationId < N ? v : c);
    }

    static uint sgid() {
      if (N == 16) return __spirv_BuiltInSubgroupLocalInvocationId;
      return cl::sycl::min(__spirv_BuiltInSubgroupLocalInvocationId,unsigned(N-1));
    }
    
    ////////////////////////////////////////////////////////////////////////////////
    /// Constants
    ////////////////////////////////////////////////////////////////////////////////

    __forceinline vint(ZeroTy)        : v(0) {}
    __forceinline vint(OneTy)         : v(1) {}
    __forceinline vint(PosInfTy)      : v(0x7FFFFFFF) {}
    __forceinline vint(NegInfTy)      : v(0x80000000) {}
    __forceinline vint(StepTy)        : v(__spirv_BuiltInSubgroupLocalInvocationId) {}
    //__forceinline vint(ReverseStepTy) : v(_mm_set_epi32(0, 1, 2, 3)) {}

    //__forceinline vint(TrueTy)   { v = _mm_cmpeq_epi32(v,v); }
    //__forceinline vint(UndefinedTy) : v(_mm_castps_si128(_mm_undefined_ps())) {}


    ////////////////////////////////////////////////////////////////////////////////
    /// Loads and Stores
    ////////////////////////////////////////////////////////////////////////////////

    static __forceinline vint<N> load (const void* a)
    {
      if (N == 16) {
        return vint<N>(__spirv_SubgroupBlockReadINTEL<int>((const __attribute__((ocl_global)) uint32_t*) a));
      } else {
        return ((int*)a)[vint<N>::sgid()];
      }
    }
    
    static __forceinline vint<N> loadu(const void* a) {
      return ((int*)a)[vint<N>::sgid()];
    }

    static __forceinline void store (void* ptr, const vint<N>& v)
    {
      if (N == 16) {
        __spirv_SubgroupBlockWriteINTEL<uint32_t>((__attribute__((ocl_global)) uint32_t*) ptr, *(uint32_t*)&v.v);
      } else {
        const uint lid = __spirv_BuiltInSubgroupLocalInvocationId;
        if (lid < N) ((int*)ptr)[lid] = v.v;
      }
    }
    
    static __forceinline void storeu(void* ptr, const vint<N>& v) {
      const uint lid = __spirv_BuiltInSubgroupLocalInvocationId;
      if (lid < N) ((int*)ptr)[lid] = v.v;
    }

    //static __forceinline vint<N> compact(const vboolf<N>& mask, vint<N> &v) {
    //  return _mm_mask_compress_ps(v, mask, v);
    //}
    //static __forceinline vint<N> compact(const vboolf<N>& mask, vint<N> &a, const vint<N>& b) {
    //  return _mm_mask_compress_ps(a, mask, b);
    //}

    static __forceinline vint<N> load (const vboolf<N>& mask, const void* ptr) {
      if (mask.v) return loadu(ptr); else return vint<N>(0.0f);
    }
    static __forceinline vint<N> loadu(const vboolf<N>& mask, const void* ptr) {
      if (mask.v) return loadu(ptr); else return vint<N>(0.0f);
    }

    static __forceinline void store (const vboolf<N>& mask, void* ptr, const vint<N>& v) {
      if (mask.v) storeu(ptr,v);
    }
    static __forceinline void storeu(const vboolf<N>& mask, void* ptr, const vint<N>& v) {
      if (mask.v) storeu(ptr,v);
    }

    static __forceinline vint<N> broadcast(const void* a) {
      return vint<N>(*(int*)a);
    }

    static __forceinline vint<N> load_nt (const int* ptr) {
      return load(ptr);
    }

    static __forceinline void store_nt(void* ptr, const vint<N>& v) {
      store(ptr,v);
    }

    //static __forceinline vint<N> load(const char* ptr) {
    //  return _mm_cvtepi32_ps(_mm_cvtepi8_epi32(_mm_loadu_si128((__m128i*)ptr)));
    //}

    //static __forceinline vint<N> load(const unsigned char* ptr) {
    //  return _mm_cvtepi32_ps(_mm_cvtepu8_epi32(_mm_loadu_si128((__m128i*)ptr)));
    //}

    //static __forceinline vint<N> load(const short* ptr) {
    //  return _mm_cvtepi32_ps(_mm_cvtepi16_epi32(_mm_loadu_si128((__m128i*)ptr)));
    //}

    //static __forceinline vint<N> load(const unsigned short* ptr) {
    //  return _mm_mul_ps(vint<N>(vint<N>::load(ptr)),vint<N>(1.0f/65535.0f));
    //}

    template<int scale = 4>
    static __forceinline vint<N> gather(const int* ptr, const vint<N>& index) {
      return *(int*)((char*)ptr + scale*index.v);
    }

    template<int scale = 4>
      static __forceinline vint<N> gather(const vboolf<N>& mask, const int* ptr, const vint<N>& index) {
      if (mask.v) return gather<scale>(ptr,index);
      else        return vint<N>(0.0f);
    }

    template<int scale = 4>
      static __forceinline void scatter(void* ptr, const vint<N>& index, const vint<N>& v) {
      *(int*) ((char*)ptr + scale*index.v) = v.v;
    }

    template<int scale = 4>
    static __forceinline void scatter(const vboolf<N>& mask, void* ptr, const vint<N>& index, const vint<N>& v) {
      if (mask.v) scatter<scale>(ptr,index,v);
    }

    static __forceinline void store(const vboolf<N>& mask, char* ptr, const vint<N>& ofs, const vint<N>& v) {
      scatter<1>(mask,ptr,ofs,v);
    }
    static __forceinline void store(const vboolf<N>& mask, int* ptr, const vint<N>& ofs, const vint<N>& v) {
      scatter<4>(mask,ptr,ofs,v);
    }
    
    ////////////////////////////////////////////////////////////////////////////////
    /// Array Access
    ////////////////////////////////////////////////////////////////////////////////

    __forceinline const int operator [](size_t index) const {
      assert(index < 4);
      return __spirv_GroupBroadcast<int>(__spv::Scope::Subgroup, v, index);
    }
    __forceinline int operator [](size_t index) {
      assert(index < 4);
      return __spirv_GroupBroadcast<int>(__spv::Scope::Subgroup, v, index);
    }

    friend __forceinline vint<N> select(const vboolf<N>& m, const vint<N>& t, const vint<N>& f) {
      return m.v ? t : f;
    }
  };

  ////////////////////////////////////////////////////////////////////////////////
  /// Unary Operators
  ////////////////////////////////////////////////////////////////////////////////

  //template<int N> __forceinline vboolf<N> asBool(const vint<N>& a) { return _mm_movepi32_mask(a); }

  template<int N> __forceinline vint<N> operator +(const vint<N>& a) { return vint<N>(+a.v); }
  template<int N> __forceinline vint<N> operator -(const vint<N>& a) { return vint<N>(-a.v); }
  template<int N> __forceinline vint<N> abs       (const vint<N>& a) { return vint<N>(std::abs(a.v)); }

  ////////////////////////////////////////////////////////////////////////////////
  /// Binary Operators
  ////////////////////////////////////////////////////////////////////////////////

  template<int N> __forceinline vint<N> operator +(const vint<N>& a, const vint<N>& b) { return vint<N>(a.v+b.v); }
  template<int N> __forceinline vint<N> operator +(const vint<N>& a, int          b) { return a + vint<N>(b); }
  template<int N> __forceinline vint<N> operator +(int          a, const vint<N>& b) { return vint<N>(a) + b; }

  template<int N> __forceinline vint<N> operator -(const vint<N>& a, const vint<N>& b) { return vint<N>(a.v+b.v); }
  template<int N> __forceinline vint<N> operator -(const vint<N>& a, int          b) { return a - vint<N>(b); }
  template<int N> __forceinline vint<N> operator -(int          a, const vint<N>& b) { return vint<N>(a) - b; }

  template<int N> __forceinline vint<N> operator *(const vint<N>& a, const vint<N>& b) { return vint<N>(a.v*b.v); }
  template<int N> __forceinline vint<N> operator *(const vint<N>& a, int          b) { return a * vint<N>(b); }
  template<int N> __forceinline vint<N> operator *(int          a, const vint<N>& b) { return vint<N>(a) * b; }

  template<int N> __forceinline vint<N> operator &(const vint<N>& a, const vint<N>& b) { return vint<N>(a.v & b.v); }
  template<int N> __forceinline vint<N> operator &(const vint<N>& a, int          b) { return a & vint<N>(b); }
  template<int N> __forceinline vint<N> operator &(int          a, const vint<N>& b) { return vint<N>(a) & b; }

  template<int N> __forceinline vint<N> operator |(const vint<N>& a, const vint<N>& b) { return vint<N>(a.v | b.v); }
  template<int N> __forceinline vint<N> operator |(const vint<N>& a, int          b) { return a | vint<N>(b); }
  template<int N> __forceinline vint<N> operator |(int          a, const vint<N>& b) { return vint<N>(a) | b; }

  template<int N> __forceinline vint<N> operator ^(const vint<N>& a, const vint<N>& b) { return vint<N>(a.v ^ b.v); }
  template<int N> __forceinline vint<N> operator ^(const vint<N>& a, int          b) { return a ^ vint<N>(b); }
  template<int N> __forceinline vint<N> operator ^(int          a, const vint<N>& b) { return vint<N>(a) ^ b; }

  template<int N> __forceinline vint<N> operator <<(const vint<N>& a, int n) { return vint<N>(a.v << n); }
  template<int N> __forceinline vint<N> operator >>(const vint<N>& a, int n) { return vint<N>(a.v >> n); }

  template<int N> __forceinline vint<N> sll (const vint<N>& a, int b) { return vint<N>(a.v << b); }
  template<int N> __forceinline vint<N> sra (const vint<N>& a, int b) { return vint<N>(a.v >> b); }
  template<int N> __forceinline vint<N> srl (const vint<N>& a, int b) { return vint<N>(unsigned(a.v) >> b); }
  
  ////////////////////////////////////////////////////////////////////////////////
  /// Assignment Operators
  ////////////////////////////////////////////////////////////////////////////////

  template<int N> __forceinline vint<N>& operator +=(vint<N>& a, const vint<N>& b) { return a = a + b; }
  template<int N> __forceinline vint<N>& operator +=(vint<N>& a, int          b) { return a = a + b; }
  
  template<int N> __forceinline vint<N>& operator -=(vint<N>& a, const vint<N>& b) { return a = a - b; }
  template<int N> __forceinline vint<N>& operator -=(vint<N>& a, int          b) { return a = a - b; }

  template<int N> __forceinline vint<N>& operator *=(vint<N>& a, const vint<N>& b) { return a = a * b; }
  template<int N> __forceinline vint<N>& operator *=(vint<N>& a, int          b) { return a = a * b; }
  
  template<int N> __forceinline vint<N>& operator &=(vint<N>& a, const vint<N>& b) { return a = a & b; }
  template<int N> __forceinline vint<N>& operator &=(vint<N>& a, int          b) { return a = a & b; }
  
  template<int N> __forceinline vint<N>& operator |=(vint<N>& a, const vint<N>& b) { return a = a | b; }
  template<int N> __forceinline vint<N>& operator |=(vint<N>& a, int          b) { return a = a | b; }
  
  template<int N> __forceinline vint<N>& operator <<=(vint<N>& a, int b) { return a = a << b; }
  template<int N> __forceinline vint<N>& operator >>=(vint<N>& a, int b) { return a = a >> b; }

  ////////////////////////////////////////////////////////////////////////////////
  /// Comparison Operators + Select
  ////////////////////////////////////////////////////////////////////////////////

  template<int N> __forceinline vboolf<N> operator ==(const vint<N>& a, const vint<N>& b) { return vboolf<N>(a.v == b.v); }
  template<int N> __forceinline vboolf<N> operator !=(const vint<N>& a, const vint<N>& b) { return vboolf<N>(a.v != b.v); }
  template<int N> __forceinline vboolf<N> operator < (const vint<N>& a, const vint<N>& b) { return vboolf<N>(a.v <  b.v); }
  template<int N> __forceinline vboolf<N> operator >=(const vint<N>& a, const vint<N>& b) { return vboolf<N>(a.v >= b.v); }
  template<int N> __forceinline vboolf<N> operator > (const vint<N>& a, const vint<N>& b) { return vboolf<N>(a.v >  b.v); }
  template<int N> __forceinline vboolf<N> operator <=(const vint<N>& a, const vint<N>& b) { return vboolf<N>(a.v <= b.v); }

  template<int N> __forceinline vboolf<N> operator ==(const vint<N>& a, int          b) { return a == vint<N>(b); }
  template<int N> __forceinline vboolf<N> operator ==(int          a, const vint<N>& b) { return vint<N>(a) == b; }

  template<int N> __forceinline vboolf<N> operator !=(const vint<N>& a, int          b) { return a != vint<N>(b); }
  template<int N> __forceinline vboolf<N> operator !=(int          a, const vint<N>& b) { return vint<N>(a) != b; }

  template<int N> __forceinline vboolf<N> operator < (const vint<N>& a, int          b) { return a <  vint<N>(b); }
  template<int N> __forceinline vboolf<N> operator < (int          a, const vint<N>& b) { return vint<N>(a) <  b; }

  template<int N> __forceinline vboolf<N> operator >=(const vint<N>& a, int          b) { return a >= vint<N>(b); }
  template<int N> __forceinline vboolf<N> operator >=(int          a, const vint<N>& b) { return vint<N>(a) >= b; }

  template<int N> __forceinline vboolf<N> operator > (const vint<N>& a, int          b) { return a >  vint<N>(b); }
  template<int N> __forceinline vboolf<N> operator > (int          a, const vint<N>& b) { return vint<N>(a) >  b; }

  template<int N> __forceinline vboolf<N> operator <=(const vint<N>& a, int          b) { return a <= vint<N>(b); }
  template<int N> __forceinline vboolf<N> operator <=(int          a, const vint<N>& b) { return vint<N>(a) <= b; }

  template<int N> __forceinline vboolf<N> eq(const vint<N>& a, const vint<N>& b) { return a == b; }
  template<int N> __forceinline vboolf<N> ne(const vint<N>& a, const vint<N>& b) { return a != b; }
  template<int N> __forceinline vboolf<N> lt(const vint<N>& a, const vint<N>& b) { return a <  b; }
  template<int N> __forceinline vboolf<N> ge(const vint<N>& a, const vint<N>& b) { return a >= b; }
  template<int N> __forceinline vboolf<N> gt(const vint<N>& a, const vint<N>& b) { return a >  b; }
  template<int N> __forceinline vboolf<N> le(const vint<N>& a, const vint<N>& b) { return a <= b; }

  //template<int N> __forceinline vboolf<N> eq(const vboolf<N>& mask, const vint<N>& a, const vint<N>& b) { return _mm_mask_cmp_epi32_mask(mask, a, b, _MM_CMPINT_EQ); }
  //template<int N> __forceinline vboolf<N> ne(const vboolf<N>& mask, const vint<N>& a, const vint<N>& b) { return _mm_mask_cmp_epi32_mask(mask, a, b, _MM_CMPINT_NE); }
  //template<int N> __forceinline vboolf<N> lt(const vboolf<N>& mask, const vint<N>& a, const vint<N>& b) { return _mm_mask_cmp_epi32_mask(mask, a, b, _MM_CMPINT_LT); }
  //template<int N> __forceinline vboolf<N> ge(const vboolf<N>& mask, const vint<N>& a, const vint<N>& b) { return _mm_mask_cmp_epi32_mask(mask, a, b, _MM_CMPINT_GE); }
  //template<int N> __forceinline vboolf<N> gt(const vboolf<N>& mask, const vint<N>& a, const vint<N>& b) { return _mm_mask_cmp_epi32_mask(mask, a, b, _MM_CMPINT_GT); }
  //template<int N> __forceinline vboolf<N> le(const vboolf<N>& mask, const vint<N>& a, const vint<N>& b) { return _mm_mask_cmp_epi32_mask(mask, a, b, _MM_CMPINT_LE); }

/*
  template<int mask>
  template<int N> __forceinline vint<N> select(const vint<N>& t, const vint<N>& f) {
#if defined(__SSE4_1__) 
    return _mm_castps_si128(_mm_blend_ps(_mm_castsi128_ps(f), _mm_castsi128_ps(t), mask));
#else
    return select(vboolf<N>(mask), t, f);
#endif    
  }
  */
  
  template<int N> __forceinline vint<N> min(const vint<N>& a, const vint<N>& b) { return vint<N>(cl::sycl::min(a.v,b.v)); }
  template<int N> __forceinline vint<N> max(const vint<N>& a, const vint<N>& b) { return vint<N>(cl::sycl::max(a.v,b.v)); }

  template<int N> __forceinline vint<N> umin(const vint<N>& a, const vint<N>& b) { return vint<N>(cl::sycl::min(unsigned(a.v),unsigned(b.v))); }
  template<int N> __forceinline vint<N> umax(const vint<N>& a, const vint<N>& b) { return vint<N>(cl::sycl::max(unsigned(a.v),unsigned(b.v))); }

  template<int N> __forceinline vint<N> min(const vint<N>& a, int          b) { return min(a,vint<N>(b)); }
  template<int N> __forceinline vint<N> min(int          a, const vint<N>& b) { return min(vint<N>(a),b); }
  template<int N> __forceinline vint<N> max(const vint<N>& a, int          b) { return max(a,vint<N>(b)); }
  template<int N> __forceinline vint<N> max(int          a, const vint<N>& b) { return max(vint<N>(a),b); }

  ////////////////////////////////////////////////////////////////////////////////
  // Movement/Shifting/Shuffling Functions
  ////////////////////////////////////////////////////////////////////////////////
/*
  template<int N> __forceinline vint<N> unpacklo(const vint<N>& a, const vint<N>& b) { return _mm_castps_si128(_mm_unpacklo_ps(_mm_castsi128_ps(a), _mm_castsi128_ps(b))); }
  template<int N> __forceinline vint<N> unpackhi(const vint<N>& a, const vint<N>& b) { return _mm_castps_si128(_mm_unpackhi_ps(_mm_castsi128_ps(a), _mm_castsi128_ps(b))); }
*/

  template<int i0, int i1, int i2, int i3, int N>
    __forceinline vint<N> shuffle(const vint<N>& v) {
    return vint<N>(__spirv_SubgroupShuffleINTEL(v.v, vint<N>(i0,i1,i2,i3).v));
  }

/*
  template<int i0, int i1, int i2, int i3>
  template<int N> __forceinline vint<N> shuffle(const vint<N>& a, const vint<N>& b) {
    return _mm_castps_si128(_mm_shuffle_ps(_mm_castsi128_ps(a), _mm_castsi128_ps(b), _MM_SHUFFLE(i3, i2, i1, i0)));
  }
*/

  template<int i, int N>
    __forceinline vint<N> shuffle(const vint<N>& v) {
    return vint<N>(__spirv_GroupBroadcast(__spv::Scope::Subgroup, v.v, i));
  }

  template<int i, int N> __forceinline int extract(const vint<N>& a) {
    return __spirv_GroupBroadcast(__spv::Scope::Subgroup, a.v, i);
  }
  
  //template<int dst> template<int N> __forceinline vint<N> insert(const vint<N>& a, const int b) { return _mm_insert_epi32(a, b, dst); }

  template<int N> __forceinline int toScalar(const vint<N>& v) { return extract<0>(v); }
  template<int N> __forceinline size_t toSizeT(const vint<N>& v) { return toScalar(v); }

/*
  template<int N> __forceinline vint<N> permute(const vint<N> &a, const vint<N> &index) {
    return  _mm_castps_si128(_mm_permutevar_ps(_mm_castsi128_ps(a),index));
  }
*/
/*  
  template<int i>
  template<int N> __forceinline vint<N> align_shift_right(const vint<N>& a, const vint<N>& b) {
    return _mm_alignr_epi32(a, b, i);    
  }  
*/
  ////////////////////////////////////////////////////////////////////////////////
  /// Reductions
  ////////////////////////////////////////////////////////////////////////////////

  template<int N> __forceinline vint<N> vreduce_min(const vint<N>& v) {
    return cl::sycl::detail::calc<int, __spv::GroupOperation::Reduce>(v.fix_upper(0x7FFFFFFF).v, cl::sycl::intel::minimum<int>());
  }

  template<int N> __forceinline vint<N> vreduce_max(const vint<N>& v) {
    return cl::sycl::detail::calc<int, __spv::GroupOperation::Reduce>(v.fix_upper(0x80000000).v, cl::sycl::intel::maximum<int>());
  }
  
  template<int N> __forceinline vint<N> vreduce_add(const vint<N>& v) {
    return cl::sycl::detail::calc<int, __spv::GroupOperation::Reduce>(v.fix_upper(0).v, cl::sycl::intel::plus<int>());
  }

  template<int N> __forceinline int reduce_min(const vint<N>& v) { return vreduce_min(v).v; }
  template<int N> __forceinline int reduce_max(const vint<N>& v) { return vreduce_max(v).v; }
  template<int N> __forceinline int reduce_add(const vint<N>& v) { return vreduce_add(v).v; }

  //template<int N> __forceinline size_t select_min(const vint<N>& v) { return bsf(movemask(v == vreduce_min(v))); }
  //template<int N> __forceinline size_t select_max(const vint<N>& v) { return bsf(movemask(v == vreduce_max(v))); }

  //template<int N> __forceinline size_t select_min(const vboolf<N>& valid, const vint<N>& v) { const vint<N> a = select(valid,v,vint<N>(pos_inf)); return bsf(movemask(valid & (a == vreduce_min(a)))); }
  //template<int N> __forceinline size_t select_max(const vboolf<N>& valid, const vint<N>& v) { const vint<N> a = select(valid,v,vint<N>(neg_inf)); return bsf(movemask(valid & (a == vreduce_max(a)))); }

  ////////////////////////////////////////////////////////////////////////////////
  /// Sorting networks
  ////////////////////////////////////////////////////////////////////////////////

#if 0
  
  template<int N> __forceinline vint<N> usort_ascending(const vint<N>& v)
  {
    const vint<N> a0 = v;
    const vint<N> b0 = shuffle<1,0,3,2>(a0);
    const vint<N> c0 = umin(a0,b0);
    const vint<N> d0 = umax(a0,b0);
    const vint<N> a1 = select<0x5 /* 0b0101 */>(c0,d0);
    const vint<N> b1 = shuffle<2,3,0,1>(a1);
    const vint<N> c1 = umin(a1,b1);
    const vint<N> d1 = umax(a1,b1);
    const vint<N> a2 = select<0x3 /* 0b0011 */>(c1,d1);
    const vint<N> b2 = shuffle<0,2,1,3>(a2);
    const vint<N> c2 = umin(a2,b2);
    const vint<N> d2 = umax(a2,b2);
    const vint<N> a3 = select<0x2 /* 0b0010 */>(c2,d2);
    return a3;
  }

  template<int N> __forceinline vint<N> usort_descending(const vint<N>& v)
  {
    const vint<N> a0 = v;
    const vint<N> b0 = shuffle<1,0,3,2>(a0);
    const vint<N> c0 = umax(a0,b0);
    const vint<N> d0 = umin(a0,b0);
    const vint<N> a1 = select<0x5 /* 0b0101 */>(c0,d0);
    const vint<N> b1 = shuffle<2,3,0,1>(a1);
    const vint<N> c1 = umax(a1,b1);
    const vint<N> d1 = umin(a1,b1);
    const vint<N> a2 = select<0x3 /* 0b0011 */>(c1,d1);
    const vint<N> b2 = shuffle<0,2,1,3>(a2);
    const vint<N> c2 = umax(a2,b2);
    const vint<N> d2 = umin(a2,b2);
    const vint<N> a3 = select<0x2 /* 0b0010 */>(c2,d2);
    return a3;
  }

#endif
  
  ////////////////////////////////////////////////////////////////////////////////
  /// Output Operators
  ////////////////////////////////////////////////////////////////////////////////

  template<int N> inline const cl::sycl::stream& operator <<(const cl::sycl::stream& cout, const vint<N>& a)
  {
    cout << "<";
    for (int i=0; i<N; i++) {
      if (__spirv_BuiltInSubgroupLocalInvocationId == i) {
        cout << a.v;
        if (i != N-1) cout << ", ";
      }
    }
    return cout << ">";
  }

  template<int N> inline std::ostream& operator <<(std::ostream& cout, const vint<N>& a) {
    return cout;
  }
}

