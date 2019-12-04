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

namespace embree
{
  /* N-wide SYCL bool type */
  template<int N>
  struct vboolf
  {
    //ALIGNED_STRUCT_(16);
    
    typedef vboolf<N> Bool;
    typedef vint<N>   Int;
    typedef vfloat<N> Float;

    enum  { size = N };            // number of SIMD elements
    //union { __m128 v; int i[N]; }; // data
    bool v;

    ////////////////////////////////////////////////////////////////////////////////
    /// Constructors, Assignment & Cast Operators
    ////////////////////////////////////////////////////////////////////////////////
    
    __forceinline vboolf() {}
    //__forceinline vboolf(const vboolf& other) { v = other.v; }
    //__forceinline vboolf& operator =(const vboolf& other) { v = other.v; return *this; }

    //__forceinline vboolf(__m128 input) : v(input) {}
    //__forceinline operator const __m128&() const { return v; }
    //__forceinline operator const __m128i() const { return _mm_castps_si128(v); }
    //__forceinline operator const __m128d() const { return _mm_castps_pd(v); }
    
    __forceinline vboolf(bool a)
      : v(a) {}
    
    __forceinline vboolf(bool a, bool b)
    {
      const uint lid = __spirv_BuiltInSubgroupLocalInvocationId;
      if (lid == 0) v = a;
      if (lid == 1) v = b;
      if (lid == 2) v = a;
      if (lid == 3) v = b;
    }
    
    __forceinline vboolf(bool a, bool b, bool c, bool d)
    {
      const uint lid = __spirv_BuiltInSubgroupLocalInvocationId;
      if (lid == 0) v = a;
      if (lid == 1) v = b;
      if (lid == 2) v = c;
      if (lid == 3) v = d;
    }
    
    //__forceinline vboolf(int mask) { assert(mask >= 0 && mask < 16); v = mm_lookupmask_ps[mask]; }
    //__forceinline vboolf(unsigned int mask) { assert(mask < 16); v = mm_lookupmask_ps[mask]; }

    /* return int32 mask */
    //__forceinline __m128i mask32() const { 
    //  return _mm_castps_si128(v);
    //}

    vboolf fix_upper(bool c) const {
      if (N == 16) return c;
      return vboolf(__spirv_BuiltInSubgroupLocalInvocationId < N ? v : c);
    }

    ////////////////////////////////////////////////////////////////////////////////
    /// Constants
    ////////////////////////////////////////////////////////////////////////////////

    __forceinline vboolf(FalseTy) : v(false) {}
    __forceinline vboolf(TrueTy)  : v(true) {}

    ////////////////////////////////////////////////////////////////////////////////
    /// Array Access
    ////////////////////////////////////////////////////////////////////////////////

    //__forceinline bool operator [](size_t index) const { assert(index < N); return (_mm_movemask_ps(v) >> index) & 1; }
    //__forceinline int& operator [](size_t index)       { assert(index < N); return i[index]; }
  };

  ////////////////////////////////////////////////////////////////////////////////
  /// Unary Operators
  ////////////////////////////////////////////////////////////////////////////////
  
  template<int N> __forceinline vboolf<N> operator !(const vboolf<N>& a) { return vboolf<N>(!a.v); }
  
  ////////////////////////////////////////////////////////////////////////////////
  /// Binary Operators
  ////////////////////////////////////////////////////////////////////////////////
  
  template<int N> __forceinline vboolf<N> operator &(const vboolf<N>& a, const vboolf<N>& b) { return vboolf<N>(a.v && b.v); }
  template<int N> __forceinline vboolf<N> operator |(const vboolf<N>& a, const vboolf<N>& b) { return vboolf<N>(a.v || b.v); }
  template<int N> __forceinline vboolf<N> operator ^(const vboolf<N>& a, const vboolf<N>& b) { return vboolf<N>(a.v != b.v); }

  //template<int N> __forceinline vboolf<N> andn(const vboolf<N>& a, const vboolf<N>& b) { return _mm_andnot_ps(b, a); }
  
  ////////////////////////////////////////////////////////////////////////////////
  /// Assignment Operators
  ////////////////////////////////////////////////////////////////////////////////
  
  template<int N> __forceinline vboolf<N>& operator &=(vboolf<N>& a, const vboolf<N>& b) { return a = a & b; }
  template<int N> __forceinline vboolf<N>& operator |=(vboolf<N>& a, const vboolf<N>& b) { return a = a | b; }
  template<int N> __forceinline vboolf<N>& operator ^=(vboolf<N>& a, const vboolf<N>& b) { return a = a ^ b; }
  
  ////////////////////////////////////////////////////////////////////////////////
  /// Comparison Operators + Select
  ////////////////////////////////////////////////////////////////////////////////
  
  template<int N> __forceinline vboolf<N> operator !=(const vboolf<N>& a, const vboolf<N>& b) { return vboolf<N>(a.v != b.v); }
  template<int N> __forceinline vboolf<N> operator ==(const vboolf<N>& a, const vboolf<N>& b) { return vboolf<N>(a.v == b.v); }
  
  template<int N> __forceinline vboolf<N> select(const vboolf<N>& m, const vboolf<N>& t, const vboolf<N>& f) {
    return vboolf<N>(m.v ? t.v : f.v);
  }

  ////////////////////////////////////////////////////////////////////////////////
  /// Movement/Shifting/Shuffling Functions
  ////////////////////////////////////////////////////////////////////////////////
  
  //template<int N> __forceinline vboolf<N> unpacklo(const vboolf<N>& a, const vboolf<N>& b) { return _mm_unpacklo_ps(a, b); }
  //template<int N> __forceinline vboolf<N> unpackhi(const vboolf<N>& a, const vboolf<N>& b) { return _mm_unpackhi_ps(a, b); }

  //template<int i0, int i1, int i2, int i3>
  //template<int N> __forceinline vboolf<N> shuffle(const vboolf<N>& v) {
  //  return _mm_castsi128_ps(_mm_shuffle_epi32(v, _MM_SHUFFLE(i3, i2, i1, i0)));
  //}

  //template<int i0, int i1, int i2, int i3>
  //template<int N> __forceinline vboolf<N> shuffle(const vboolf<N>& a, const vboolf<N>& b) {
  //  return _mm_shuffle_ps(a, b, _MM_SHUFFLE(i3, i2, i1, i0));
  //}

  //template<int i0>
  //template<int N> __forceinline vboolf<N> shuffle(const vboolf<N>& v) {
  //  return shuffle<i0,i0,i0,i0>(v);
  //}

  //template<int dst, int src, int clr> template<int N> __forceinline vboolf<N> insert(const vboolf<N>& a, const vboolf<N>& b) { return _mm_insert_ps(a, b, (dst << 4) | (src << 6) | clr); }
  //template<int dst, int src> template<int N> __forceinline vboolf<N> insert(const vboolf<N>& a, const vboolf<N>& b) { return insert<dst, src, 0>(a, b); }
  //template<int dst> template<int N> __forceinline vboolf<N> insert(const vboolf<N>& a, const bool b) { return insert<dst, 0>(a, vboolf<N>(b)); }
  
  ////////////////////////////////////////////////////////////////////////////////
  /// Reduction Operations
  ////////////////////////////////////////////////////////////////////////////////
    
  template<int N> __forceinline bool reduce_and(const vboolf<N>& a) {
    return __spirv_GroupAll(__spv::Scope::Subgroup, a.fix_upper(true).v);
  }

  template<int N> __forceinline bool reduce_or(const vboolf<N>& a) {
    return __spirv_GroupAny(__spv::Scope::Subgroup, a.fix_upper(false).v);
  }

  template<int N> __forceinline bool all(const vboolf<N>& a) {
    return __spirv_GroupAll(__spv::Scope::Subgroup, a.fix_upper(true).v);
  }

  template<int N> __forceinline bool any(const vboolf<N>& a) {
    return __spirv_GroupAny(__spv::Scope::Subgroup, a.fix_upper(false).v);
  }

  template<int N> __forceinline bool none(const vboolf<N>& a) {
    return !any(a);
  }

  template<int N> __forceinline bool all (const vboolf<N>& valid, const vboolf<N>& b) { return all((!valid) | b); }
  template<int N> __forceinline bool any (const vboolf<N>& valid, const vboolf<N>& b) { return any(valid & b); }
  template<int N> __forceinline bool none(const vboolf<N>& valid, const vboolf<N>& b) { return none(valid & b); }
  
  template<int N> __forceinline size_t movemask(const vboolf<N>& a) {
    return intel_sub_group_ballot(a.fix_upper(false).v);
  }
  template<int N> __forceinline size_t popcnt(const vboolf<N>& a) {
    return cl::sycl::popcount(movemask(a)); 
  }

  ////////////////////////////////////////////////////////////////////////////////
  /// Get/Set Functions
  ////////////////////////////////////////////////////////////////////////////////

  //template<int N> __forceinline bool get(const vboolf<N>& a, size_t index) { return a[index]; }
  //template<int N> __forceinline void set(vboolf<N>& a, size_t index)       { a[index] = -1; }
  //template<int N> __forceinline void clear(vboolf<N>& a, size_t index)     { a[index] =  0; }

  ////////////////////////////////////////////////////////////////////////////////
  /// Output Operators
  ////////////////////////////////////////////////////////////////////////////////
  
  template<int N> inline const cl::sycl::stream& operator <<(const cl::sycl::stream& cout, const vboolf<N>& a)
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

  template<int N> inline std::ostream& operator <<(std::ostream& cout, const vboolf<N>& a) {
    return cout;
  }
}
