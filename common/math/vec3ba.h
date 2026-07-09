// Copyright 2009-2021 Intel Corporation
// SPDX-License-Identifier: Apache-2.0

#pragma once

#include "../sys/alloc.h"
#include "emath.h"

#if defined(EMBREE_SYCL_SUPPORT) && defined(__SYCL_DEVICE_ONLY__)
#  include "vec3ba_sycl.h"
#else

#include "../simd/sse.h"

namespace embree
{
  ////////////////////////////////////////////////////////////////////////////////
  /// SSE Vec3ba Type
  ////////////////////////////////////////////////////////////////////////////////

  struct __aligned(16) Vec3ba
  {
    ALIGNED_STRUCT_(16);
    
    union {
      __m128 v;
      struct { int x,y,z; };
    };

    typedef int Scalar;
    enum { N = 3 };

    ////////////////////////////////////////////////////////////////////////////////
    /// Constructors, Assignment & Cast Operators
    ////////////////////////////////////////////////////////////////////////////////

    __forceinline Vec3ba( ) {}
    __forceinline Vec3ba( const __m128  input ) : v(input) {}
    __forceinline Vec3ba( const Vec3ba& other ) : v(other.v) {}
    __forceinline Vec3ba& operator =(const Vec3ba& other) { v = other.v; return *this; }

    __forceinline explicit Vec3ba( bool a )
      : v(mm_lookupmask_ps[(size_t(a) << 3) | (size_t(a) << 2) | (size_t(a) << 1) | size_t(a)]) {}
    __forceinline Vec3ba( bool a, bool b, bool c)
      : v(mm_lookupmask_ps[(size_t(c) << 2) | (size_t(b) << 1) | size_t(a)]) {}

    __forceinline const __m128& m128() const { return v; }
    __forceinline __m128& m128()       { return v; }

    ////////////////////////////////////////////////////////////////////////////////
    /// Constants
    ////////////////////////////////////////////////////////////////////////////////

    __forceinline Vec3ba( FalseTy ) : v(_mm_setzero_ps()) {}
    __forceinline Vec3ba( TrueTy  ) : v(_mm_castsi128_ps(_mm_cmpeq_epi32(_mm_setzero_si128(), _mm_setzero_si128()))) {}

    ////////////////////////////////////////////////////////////////////////////////
    /// Array Access
    ////////////////////////////////////////////////////////////////////////////////

    __forceinline const int& operator []( const size_t index ) const { assert(index < 3); return (&x)[index]; }
    __forceinline       int& operator []( const size_t index )       { assert(index < 3); return (&x)[index]; }
  };


  ////////////////////////////////////////////////////////////////////////////////
  /// Unary Operators
  ////////////////////////////////////////////////////////////////////////////////

  __forceinline Vec3ba operator !( const Vec3ba& a ) { return _mm_xor_ps(a.v, Vec3ba(embree::True).v); }

  ////////////////////////////////////////////////////////////////////////////////
  /// Binary Operators
  ////////////////////////////////////////////////////////////////////////////////

  __forceinline Vec3ba operator &( const Vec3ba& a, const Vec3ba& b ) { return _mm_and_ps(a.v, b.v); }
  __forceinline Vec3ba operator |( const Vec3ba& a, const Vec3ba& b ) { return _mm_or_ps (a.v, b.v); }
  __forceinline Vec3ba operator ^( const Vec3ba& a, const Vec3ba& b ) { return _mm_xor_ps(a.v, b.v); }
  
  ////////////////////////////////////////////////////////////////////////////////
  /// Assignment Operators
  ////////////////////////////////////////////////////////////////////////////////
  
  __forceinline Vec3ba& operator &=( Vec3ba& a, const Vec3ba& b ) { return a = a & b; }
  __forceinline Vec3ba& operator |=( Vec3ba& a, const Vec3ba& b ) { return a = a | b; }
  __forceinline Vec3ba& operator ^=( Vec3ba& a, const Vec3ba& b ) { return a = a ^ b; }
  
  ////////////////////////////////////////////////////////////////////////////////
  /// Comparison Operators + Select
  ////////////////////////////////////////////////////////////////////////////////
  
  __forceinline bool operator ==( const Vec3ba& a, const Vec3ba& b ) { 
    return (_mm_movemask_ps(_mm_castsi128_ps(_mm_cmpeq_epi32(_mm_castps_si128(a.v), _mm_castps_si128(b.v)))) & 7) == 7; 
  }
  __forceinline bool operator !=( const Vec3ba& a, const Vec3ba& b ) { 
    return (_mm_movemask_ps(_mm_castsi128_ps(_mm_cmpeq_epi32(_mm_castps_si128(a.v), _mm_castps_si128(b.v)))) & 7) != 7; 
  }
  __forceinline bool operator < ( const Vec3ba& a, const Vec3ba& b ) {
    if (a.x != b.x) return a.x < b.x;
    if (a.y != b.y) return a.y < b.y;
    if (a.z != b.z) return a.z < b.z;
    return false;
  }

  ////////////////////////////////////////////////////////////////////////////////
  /// Reduction Operations
  ////////////////////////////////////////////////////////////////////////////////
    
  __forceinline bool reduce_and( const Vec3ba& a ) { return (_mm_movemask_ps(a.m128()) & 0x7) == 0x7; }
  __forceinline bool reduce_or ( const Vec3ba& a ) { return (_mm_movemask_ps(a.m128()) & 0x7) != 0x0; }

  __forceinline bool all       ( const Vec3ba& b ) { return (_mm_movemask_ps(b.m128()) & 0x7) == 0x7; }
  __forceinline bool any       ( const Vec3ba& b ) { return (_mm_movemask_ps(b.m128()) & 0x7) != 0x0; }
  __forceinline bool none      ( const Vec3ba& b ) { return (_mm_movemask_ps(b.m128()) & 0x7) == 0x0; }

  __forceinline size_t movemask(const Vec3ba& a) { return _mm_movemask_ps(a.m128()) & 0x7; }

  ////////////////////////////////////////////////////////////////////////////////
  /// Output Operators
  ////////////////////////////////////////////////////////////////////////////////

  __forceinline embree_ostream operator<<(embree_ostream cout, const Vec3ba& a) {
    return cout << "(" << (a.x ? "1" : "0") << ", " << (a.y ? "1" : "0") << ", " << (a.z ? "1" : "0") << ")";
  }
}

#endif
