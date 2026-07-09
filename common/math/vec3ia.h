// Copyright 2009-2021 Intel Corporation
// SPDX-License-Identifier: Apache-2.0

#pragma once

#include "../sys/alloc.h"
#include "emath.h"

#if defined(EMBREE_SYCL_SUPPORT) && defined(__SYCL_DEVICE_ONLY__)
#  include "vec3ia_sycl.h"
#else

#include "../simd/sse.h"

namespace embree
{
  ////////////////////////////////////////////////////////////////////////////////
  /// SSE Vec3ia Type
  ////////////////////////////////////////////////////////////////////////////////

  struct __aligned(16) Vec3ia
  {
    ALIGNED_STRUCT_(16);

    union {
      __m128i v;
      struct { int x,y,z; };
    };

    typedef int Scalar;
    enum { N = 3 };

    ////////////////////////////////////////////////////////////////////////////////
    /// Constructors, Assignment & Cast Operators
    ////////////////////////////////////////////////////////////////////////////////

    __forceinline Vec3ia( ) {}
    __forceinline Vec3ia( const __m128i a ) : v(a) {}
    __forceinline Vec3ia( const Vec3ia& other ) : v(other.v) {}
    __forceinline Vec3ia& operator =(const Vec3ia& other) { v = other.v; return *this; }

    __forceinline explicit Vec3ia( const int a ) : v(_mm_set1_epi32(a)) {}
    __forceinline          Vec3ia( const int x, const int y, const int z) : v(_mm_set_epi32(z, z, y, x)) {}
#if !defined(_M_ARM64) || defined(__clang__)
    __forceinline explicit Vec3ia( const __m128 a ) : v(_mm_cvtps_epi32(a)) {}
#endif

    __forceinline const __m128i& m128i() const { return v; }
    __forceinline __m128i& m128i()       { return v; }

    __forceinline __m128 m128() const { return _mm_cvtepi32_ps(v);  }

    __forceinline operator vint4() const { return vint4(m128i()); }

    ////////////////////////////////////////////////////////////////////////////////
    /// Constants
    ////////////////////////////////////////////////////////////////////////////////

    __forceinline Vec3ia( ZeroTy   ) : v(_mm_setzero_si128()) {}
    __forceinline Vec3ia( OneTy    ) : v(_mm_set1_epi32(1)) {}
    __forceinline Vec3ia( PosInfTy ) : v(_mm_set1_epi32(pos_inf)) {}
    __forceinline Vec3ia( NegInfTy ) : v(_mm_set1_epi32(neg_inf)) {}

    ////////////////////////////////////////////////////////////////////////////////
    /// Array Access
    ////////////////////////////////////////////////////////////////////////////////

    __forceinline const int& operator []( const size_t index ) const { assert(index < 3); return (&x)[index]; }
    __forceinline       int& operator []( const size_t index )       { assert(index < 3); return (&x)[index]; }
  };


  ////////////////////////////////////////////////////////////////////////////////
  /// Unary Operators
  ////////////////////////////////////////////////////////////////////////////////

  __forceinline Vec3ia operator +( const Vec3ia& a ) { return a; }
  __forceinline Vec3ia operator -( const Vec3ia& a ) { return _mm_sub_epi32(_mm_setzero_si128(), a.v); }
#if defined(__aarch64__) || defined(_M_ARM64)
  __forceinline Vec3ia abs       ( const Vec3ia& a ) { return vabsq_s32(a.v); }
#elif defined(__SSSE3__)
  __forceinline Vec3ia abs       ( const Vec3ia& a ) { return _mm_abs_epi32(a.v); }
#endif

  ////////////////////////////////////////////////////////////////////////////////
  /// Binary Operators
  ////////////////////////////////////////////////////////////////////////////////

  __forceinline Vec3ia operator +( const Vec3ia& a, const Vec3ia& b ) { return _mm_add_epi32(a.v, b.v); }
  __forceinline Vec3ia operator +( const Vec3ia& a, const int     b ) { return a+Vec3ia(b); }
  __forceinline Vec3ia operator +( const int     a, const Vec3ia& b ) { return Vec3ia(a)+b; }

  __forceinline Vec3ia operator -( const Vec3ia& a, const Vec3ia& b ) { return _mm_sub_epi32(a.v, b.v); }
  __forceinline Vec3ia operator -( const Vec3ia& a, const int     b ) { return a-Vec3ia(b); }
  __forceinline Vec3ia operator -( const int     a, const Vec3ia& b ) { return Vec3ia(a)-b; }

#if defined(__aarch64__) || defined(_M_ARM64) || defined(__SSE4_1__)
  __forceinline Vec3ia operator *( const Vec3ia& a, const Vec3ia& b ) { return _mm_mullo_epi32(a.v, b.v); }
  __forceinline Vec3ia operator *( const Vec3ia& a, const int     b ) { return a * Vec3ia(b); }
  __forceinline Vec3ia operator *( const int     a, const Vec3ia& b ) { return Vec3ia(a) * b; }
#endif

  __forceinline Vec3ia operator &( const Vec3ia& a, const Vec3ia& b ) { return _mm_and_si128(a.v, b.v); }
  __forceinline Vec3ia operator &( const Vec3ia& a, const int     b ) { return a & Vec3ia(b); }
  __forceinline Vec3ia operator &( const int     a, const Vec3ia& b ) { return Vec3ia(a) & b; }

  __forceinline Vec3ia operator |( const Vec3ia& a, const Vec3ia& b ) { return _mm_or_si128(a.v, b.v); }
  __forceinline Vec3ia operator |( const Vec3ia& a, const int     b ) { return a | Vec3ia(b); }
  __forceinline Vec3ia operator |( const int     a, const Vec3ia& b ) { return Vec3ia(a) | b; }

  __forceinline Vec3ia operator ^( const Vec3ia& a, const Vec3ia& b ) { return _mm_xor_si128(a.v, b.v); }
  __forceinline Vec3ia operator ^( const Vec3ia& a, const int     b ) { return a ^ Vec3ia(b); }
  __forceinline Vec3ia operator ^( const int     a, const Vec3ia& b ) { return Vec3ia(a) ^ b; }

  __forceinline Vec3ia operator <<( const Vec3ia& a, const int n ) { return _mm_slli_epi32(a.v, n); }
  __forceinline Vec3ia operator >>( const Vec3ia& a, const int n ) { return _mm_srai_epi32(a.v, n); }

  __forceinline Vec3ia sll ( const Vec3ia& a, const int b ) { return _mm_slli_epi32(a.v, b); }
  __forceinline Vec3ia sra ( const Vec3ia& a, const int b ) { return _mm_srai_epi32(a.v, b); }
  __forceinline Vec3ia srl ( const Vec3ia& a, const int b ) { return _mm_srli_epi32(a.v, b); }

  ////////////////////////////////////////////////////////////////////////////////
  /// Assignment Operators
  ////////////////////////////////////////////////////////////////////////////////

  __forceinline Vec3ia& operator +=( Vec3ia& a, const Vec3ia& b ) { return a = a + b; }
  __forceinline Vec3ia& operator +=( Vec3ia& a, const int&   b ) { return a = a + b; }
  
  __forceinline Vec3ia& operator -=( Vec3ia& a, const Vec3ia& b ) { return a = a - b; }
  __forceinline Vec3ia& operator -=( Vec3ia& a, const int&   b ) { return a = a - b; }
  
#if defined(__aarch64__) || defined(_M_ARM64) || defined(__SSE4_1__)
  __forceinline Vec3ia& operator *=( Vec3ia& a, const Vec3ia& b ) { return a = a * b; }
  __forceinline Vec3ia& operator *=( Vec3ia& a, const int&    b ) { return a = a * b; }
#endif
  
  __forceinline Vec3ia& operator &=( Vec3ia& a, const Vec3ia& b ) { return a = a & b; }
  __forceinline Vec3ia& operator &=( Vec3ia& a, const int&    b ) { return a = a & b; }
  
  __forceinline Vec3ia& operator |=( Vec3ia& a, const Vec3ia& b ) { return a = a | b; }
  __forceinline Vec3ia& operator |=( Vec3ia& a, const int&    b ) { return a = a | b; }
  
#if !defined(__ARM_NEON) && !defined(_M_ARM64)
  __forceinline Vec3ia& operator <<=( Vec3ia& a, const int& b ) { return a = a << b; }
  __forceinline Vec3ia& operator >>=( Vec3ia& a, const int& b ) { return a = a >> b; }
#endif

  ////////////////////////////////////////////////////////////////////////////////
  /// Select
  ////////////////////////////////////////////////////////////////////////////////

  __forceinline Vec3ia select( const Vec3ba& m, const Vec3ia& t, const Vec3ia& f ) {
#if defined(__aarch64__) || defined(_M_ARM64) || defined(__SSE4_1__)
    return _mm_castps_si128(_mm_blendv_ps(_mm_castsi128_ps(f.m128i()), _mm_castsi128_ps(t.m128i()), m.m128()));
#else
    return _mm_or_si128(_mm_and_si128(_mm_castps_si128(m.m128()), t.m128i()), _mm_andnot_si128(_mm_castps_si128(m.m128()), f.m128i())); 
#endif
  }

  ////////////////////////////////////////////////////////////////////////////////
  /// Reductions
  ////////////////////////////////////////////////////////////////////////////////
#if defined(__aarch64__) || defined(_M_ARM64)
  __forceinline int reduce_add(const Vec3ia& v) { return vaddvq_s32(select(Vec3ba(1,1,1),v,Vec3ia(0)).v); }
  __forceinline int reduce_mul(const Vec3ia& v) { return v.x*v.y*v.z; }
  __forceinline int reduce_min(const Vec3ia& v) { return vminvq_s32(select(Vec3ba(1,1,1),v,Vec3ia(0x7FFFFFFF)).v); }
  __forceinline int reduce_max(const Vec3ia& v) { return vmaxvq_s32(select(Vec3ba(1,1,1),v,Vec3ia(0x80000000)).v); }
#else
  __forceinline int reduce_add(const Vec3ia& v) { return v.x+v.y+v.z; }
  __forceinline int reduce_mul(const Vec3ia& v) { return v.x*v.y*v.z; }
  __forceinline int reduce_min(const Vec3ia& v) { return min(v.x,v.y,v.z); }
  __forceinline int reduce_max(const Vec3ia& v) { return max(v.x,v.y,v.z); }
#endif
  
  ////////////////////////////////////////////////////////////////////////////////
  /// Comparison Operators
  ////////////////////////////////////////////////////////////////////////////////

  __forceinline bool operator ==( const Vec3ia& a, const Vec3ia& b ) { return (_mm_movemask_ps(_mm_castsi128_ps(_mm_cmpeq_epi32(a.v, b.v))) & 7) == 7; }
  __forceinline bool operator !=( const Vec3ia& a, const Vec3ia& b ) { return (_mm_movemask_ps(_mm_castsi128_ps(_mm_cmpeq_epi32(a.v, b.v))) & 7) != 7; }
  __forceinline bool operator < ( const Vec3ia& a, const Vec3ia& b ) {
    if (a.x != b.x) return a.x < b.x;
    if (a.y != b.y) return a.y < b.y;
    if (a.z != b.z) return a.z < b.z;
    return false;
  }

  __forceinline Vec3ba eq_mask( const Vec3ia& a, const Vec3ia& b ) { return _mm_castsi128_ps(_mm_cmpeq_epi32 (a.v, b.v)); }
  __forceinline Vec3ba lt_mask( const Vec3ia& a, const Vec3ia& b ) { return _mm_castsi128_ps(_mm_cmplt_epi32 (a.v, b.v)); }
  __forceinline Vec3ba gt_mask( const Vec3ia& a, const Vec3ia& b ) { return _mm_castsi128_ps(_mm_cmpgt_epi32 (a.v, b.v)); }

#if defined(__aarch64__) || defined(_M_ARM64) || defined(__SSE4_1__)
  __forceinline Vec3ia min( const Vec3ia& a, const Vec3ia& b ) { return _mm_min_epi32(a.v,b.v); }
  __forceinline Vec3ia max( const Vec3ia& a, const Vec3ia& b ) { return _mm_max_epi32(a.v,b.v); }
#else
  __forceinline Vec3ia min( const Vec3ia& a, const Vec3ia& b ) { return select(lt_mask(a,b),a,b); }
  __forceinline Vec3ia max( const Vec3ia& a, const Vec3ia& b ) { return select(gt_mask(a,b),a,b); }
#endif

  ////////////////////////////////////////////////////////////////////////////////
  /// Output Operators
  ////////////////////////////////////////////////////////////////////////////////

  __forceinline embree_ostream operator<<(embree_ostream cout, const Vec3ia& a) {
    return cout << "(" << a.x << ", " << a.y << ", " << a.z << ")";
  }
}

#endif
