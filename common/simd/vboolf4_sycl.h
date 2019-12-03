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
  /* 4-wide SSE bool type */
  template<>
  struct vboolf<4>
  {
    //ALIGNED_STRUCT_(16);
    
    typedef vboolf4 Bool;
    //typedef vint4   Int;
    typedef vfloat4 Float;

    enum  { size = 4 };            // number of SIMD elements
    //union { __m128 v; int i[4]; }; // data
    bool v;

    ////////////////////////////////////////////////////////////////////////////////
    /// Constructors, Assignment & Cast Operators
    ////////////////////////////////////////////////////////////////////////////////
    
    __forceinline vboolf() {}
    //__forceinline vboolf(const vboolf4& other) { v = other.v; }
    //__forceinline vboolf4& operator =(const vboolf4& other) { v = other.v; return *this; }

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

    ////////////////////////////////////////////////////////////////////////////////
    /// Constants
    ////////////////////////////////////////////////////////////////////////////////

    __forceinline vboolf(FalseTy) : v(false) {}
    __forceinline vboolf(TrueTy)  : v(true) {}

    ////////////////////////////////////////////////////////////////////////////////
    /// Array Access
    ////////////////////////////////////////////////////////////////////////////////

    //__forceinline bool operator [](size_t index) const { assert(index < 4); return (_mm_movemask_ps(v) >> index) & 1; }
    //__forceinline int& operator [](size_t index)       { assert(index < 4); return i[index]; }
  };

  ////////////////////////////////////////////////////////////////////////////////
  /// Unary Operators
  ////////////////////////////////////////////////////////////////////////////////
  
  __forceinline vboolf4 operator !(const vboolf4& a) { return vboolf4(!a.v); }
  
  ////////////////////////////////////////////////////////////////////////////////
  /// Binary Operators
  ////////////////////////////////////////////////////////////////////////////////
  
  __forceinline vboolf4 operator &(const vboolf4& a, const vboolf4& b) { return vboolf4(a.v && b.v); }
  __forceinline vboolf4 operator |(const vboolf4& a, const vboolf4& b) { return vboolf4(a.v || b.v); }
  __forceinline vboolf4 operator ^(const vboolf4& a, const vboolf4& b) { return vboolf4(a.v != b.v); }

  //__forceinline vboolf4 andn(const vboolf4& a, const vboolf4& b) { return _mm_andnot_ps(b, a); }
  
  ////////////////////////////////////////////////////////////////////////////////
  /// Assignment Operators
  ////////////////////////////////////////////////////////////////////////////////
  
  __forceinline vboolf4& operator &=(vboolf4& a, const vboolf4& b) { return a = a & b; }
  __forceinline vboolf4& operator |=(vboolf4& a, const vboolf4& b) { return a = a | b; }
  __forceinline vboolf4& operator ^=(vboolf4& a, const vboolf4& b) { return a = a ^ b; }
  
  ////////////////////////////////////////////////////////////////////////////////
  /// Comparison Operators + Select
  ////////////////////////////////////////////////////////////////////////////////
  
  __forceinline vboolf4 operator !=(const vboolf4& a, const vboolf4& b) { return vboolf4(a.v != b.v); }
  __forceinline vboolf4 operator ==(const vboolf4& a, const vboolf4& b) { return vboolf4(a.v == b.v); }
  
  __forceinline vboolf4 select(const vboolf4& m, const vboolf4& t, const vboolf4& f) {
    return vboolf4(m.v ? t.v : f.v);
  }

  ////////////////////////////////////////////////////////////////////////////////
  /// Movement/Shifting/Shuffling Functions
  ////////////////////////////////////////////////////////////////////////////////
  
  //__forceinline vboolf4 unpacklo(const vboolf4& a, const vboolf4& b) { return _mm_unpacklo_ps(a, b); }
  //__forceinline vboolf4 unpackhi(const vboolf4& a, const vboolf4& b) { return _mm_unpackhi_ps(a, b); }

  //template<int i0, int i1, int i2, int i3>
  //__forceinline vboolf4 shuffle(const vboolf4& v) {
  //  return _mm_castsi128_ps(_mm_shuffle_epi32(v, _MM_SHUFFLE(i3, i2, i1, i0)));
  //}

  //template<int i0, int i1, int i2, int i3>
  //__forceinline vboolf4 shuffle(const vboolf4& a, const vboolf4& b) {
  //  return _mm_shuffle_ps(a, b, _MM_SHUFFLE(i3, i2, i1, i0));
  //}

  //template<int i0>
  //__forceinline vboolf4 shuffle(const vboolf4& v) {
  //  return shuffle<i0,i0,i0,i0>(v);
  //}

  //template<int dst, int src, int clr> __forceinline vboolf4 insert(const vboolf4& a, const vboolf4& b) { return _mm_insert_ps(a, b, (dst << 4) | (src << 6) | clr); }
  //template<int dst, int src> __forceinline vboolf4 insert(const vboolf4& a, const vboolf4& b) { return insert<dst, src, 0>(a, b); }
  //template<int dst> __forceinline vboolf4 insert(const vboolf4& a, const bool b) { return insert<dst, 0>(a, vboolf4(b)); }
  
  ////////////////////////////////////////////////////////////////////////////////
  /// Reduction Operations
  ////////////////////////////////////////////////////////////////////////////////
    
  __forceinline bool reduce_and(const vboolf4& a) {
    const bool x = __spirv_BuiltInSubgroupLocalInvocationId < 4 ? a.v : true;
    return __spirv_GroupAll(__spv::Scope::Subgroup, x);
  }

  __forceinline bool reduce_or(const vboolf4& a) {
    const bool x = __spirv_BuiltInSubgroupLocalInvocationId < 4 ? a.v : false;
    return __spirv_GroupAny(__spv::Scope::Subgroup, x);
  }

  __forceinline bool all(const vboolf4& a) {
    const bool x = __spirv_BuiltInSubgroupLocalInvocationId < 4 ? a.v : true;
    return __spirv_GroupAll(__spv::Scope::Subgroup, x);
  }

  __forceinline bool any(const vboolf4& a) {
    const bool x = __spirv_BuiltInSubgroupLocalInvocationId < 4 ? a.v : false;
    return __spirv_GroupAny(__spv::Scope::Subgroup, x);
  }

  __forceinline bool none(const vboolf4& a) {
    return !any(a);
  }

  __forceinline bool all (const vboolf4& valid, const vboolf4& b) { return all((!valid) | b); }
  __forceinline bool any (const vboolf4& valid, const vboolf4& b) { return any(valid & b); }
  __forceinline bool none(const vboolf4& valid, const vboolf4& b) { return none(valid & b); }
  
  //__forceinline size_t movemask(const vboolf4& a) { return _mm_movemask_ps(a); }
  __forceinline size_t popcnt(const vboolf4& a) {
    const int x = a.v ? 1 : 0;
    const int y = __spirv_BuiltInSubgroupLocalInvocationId < 4 ? x : 0;
    return cl::sycl::detail::calc<int, __spv::GroupOperation::Reduce>(y, cl::sycl::intel::plus<int>());
  }

  ////////////////////////////////////////////////////////////////////////////////
  /// Get/Set Functions
  ////////////////////////////////////////////////////////////////////////////////

  //__forceinline bool get(const vboolf4& a, size_t index) { return a[index]; }
  //__forceinline void set(vboolf4& a, size_t index)       { a[index] = -1; }
  //__forceinline void clear(vboolf4& a, size_t index)     { a[index] =  0; }

  ////////////////////////////////////////////////////////////////////////////////
  /// Output Operators
  ////////////////////////////////////////////////////////////////////////////////
  
  inline const cl::sycl::stream& operator <<(const cl::sycl::stream& cout, const vboolf4& a)
  {
    cout << "<";
    for (int i=0; i<4; i++) {
      if (__spirv_BuiltInSubgroupLocalInvocationId == i) {
        cout << a.v;
        if (i != 3) cout << ", ";
      }
    }
    return cout << ">";
  }
}
