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

#include "../sys/alloc.h"
#include "math.h"
#include "../simd/sse.h"

#include "../../kernels/gpu/common.h"

namespace embree
{
  ////////////////////////////////////////////////////////////////////////////////
  /// SSE Vec3ba Type
  ////////////////////////////////////////////////////////////////////////////////

  struct __aligned(16) Vec3ba
  {
    //ALIGNED_STRUCT_(16);
    
    struct { bool x,y,z; };

    typedef bool Scalar;
    enum { N = 3 };

    ////////////////////////////////////////////////////////////////////////////////
    /// Constructors, Assignment & Cast Operators
    ////////////////////////////////////////////////////////////////////////////////

    __forceinline Vec3ba( ) {}
    //__forceinline Vec3ba( const __m128  input ) : m128(input) {}
    //__forceinline Vec3ba( const Vec3ba& other ) : m128(other.m128) {}
    //__forceinline Vec3ba& operator =(const Vec3ba& other) { m128 = other.m128; return *this; }

    __forceinline explicit Vec3ba( bool a ) : x(a), y(a), z(a) {}
    __forceinline Vec3ba( bool a, bool b, bool c) : x(a), y(b), z(c) {}

    //__forceinline operator const __m128&() const { return m128; }
    //__forceinline operator       __m128&()       { return m128; }

    ////////////////////////////////////////////////////////////////////////////////
    /// Constants
    ////////////////////////////////////////////////////////////////////////////////

    __forceinline Vec3ba( FalseTy ) : x(false), y(false), z(false) {}
    __forceinline Vec3ba( TrueTy  ) : x(true),  y(true),  z(true)  {}

    ////////////////////////////////////////////////////////////////////////////////
    /// Array Access
    ////////////////////////////////////////////////////////////////////////////////

    //__forceinline const int& operator []( const size_t index ) const { assert(index < 3); return (&x)[index]; }
    //__forceinline       int& operator []( const size_t index )       { assert(index < 3); return (&x)[index]; }
  };


  ////////////////////////////////////////////////////////////////////////////////
  /// Unary Operators
  ////////////////////////////////////////////////////////////////////////////////

  __forceinline Vec3ba operator !( const Vec3ba& a ) { return Vec3ba(!a.x,!a.y,!a.z); }

  ////////////////////////////////////////////////////////////////////////////////
  /// Binary Operators
  ////////////////////////////////////////////////////////////////////////////////

  __forceinline Vec3ba operator &( const Vec3ba& a, const Vec3ba& b ) { return Vec3ba(a.x & b.x, a.y & b.y, a.z & b.z); }
  __forceinline Vec3ba operator |( const Vec3ba& a, const Vec3ba& b ) { return Vec3ba(a.x | b.x, a.y | b.y, a.z | b.z); }
  __forceinline Vec3ba operator ^( const Vec3ba& a, const Vec3ba& b ) { return Vec3ba(a.x != b.x, a.y != b.y, a.z != b.z); }
  
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
    return a.x == b.x && a.y == b.y && a.z == b.z;
  }
  __forceinline bool operator !=( const Vec3ba& a, const Vec3ba& b ) {
    return a.x != b.x || a.y != b.y || a.z != b.z;
  }
/*
  __forceinline bool operator < ( const Vec3ba& a, const Vec3ba& b ) {
    if (a.x != b.x) return a.x < b.x;
    if (a.y != b.y) return a.y < b.y;
    if (a.z != b.z) return a.z < b.z;
    return false;
  }
*/
  ////////////////////////////////////////////////////////////////////////////////
  /// Reduction Operations
  ////////////////////////////////////////////////////////////////////////////////
    
  __forceinline bool reduce_and( const Vec3ba& a ) { return a.x & a.y & a.z; }
  __forceinline bool reduce_or ( const Vec3ba& a ) { return a.x | a.y | a.z; }

  __forceinline bool all       ( const Vec3ba& b ) { return reduce_and(b); }
  __forceinline bool any       ( const Vec3ba& b ) { return reduce_or(b); }
  __forceinline bool none      ( const Vec3ba& b ) { return !reduce_or(b); }

  ////////////////////////////////////////////////////////////////////////////////
  /// Output Operators
  ////////////////////////////////////////////////////////////////////////////////

  inline std::ostream& operator<<(std::ostream& cout, const Vec3ba& a) {
    return cout;
  }
}
