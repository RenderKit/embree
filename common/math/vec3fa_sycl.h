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
  /// SSE Vec3fa Type
  ////////////////////////////////////////////////////////////////////////////////

  struct __aligned(16) Vec3fa
  {
    //ALIGNED_STRUCT_(16);

    typedef float Scalar;
    enum { N = 3 };
    struct { float x,y,z; union { int a; unsigned u; float w; }; };

    ////////////////////////////////////////////////////////////////////////////////
    /// Constructors, Assignment & Cast Operators
    ////////////////////////////////////////////////////////////////////////////////

    __forceinline Vec3fa( ) {}
    //__forceinline Vec3fa( const __m128 a ) : m128(a) {}

    __forceinline Vec3fa            ( const Vec3<float>& other  ) { x = other.x; y = other.y; z = other.z; }
    __forceinline Vec3fa& operator =( const Vec3<float>& other ) { x = other.x; y = other.y; z = other.z; return *this; }

    //__forceinline Vec3fa            ( const Vec3fa& other ) { m128 = other.m128; }

    //__forceinline Vec3fa& operator =( const Vec3fa& other ) { m128 = other.m128; return *this; }

    __forceinline explicit Vec3fa( const float a ) : x(a), y(a), z(a), w(a) {}
    __forceinline          Vec3fa( const float x, const float y, const float z) : x(x), y(y), z(z), w(z) {}

    __forceinline Vec3fa( const Vec3fa& other, const int      a1) : x(other.x), y(other.y), z(other.z), a(a1) {}
    __forceinline Vec3fa( const Vec3fa& other, const unsigned a1) : x(other.x), y(other.y), z(other.z), u(a1) {}
    __forceinline Vec3fa( const Vec3fa& other, const float    w1) : x(other.x), y(other.y), z(other.z), w(w1) {}

    //__forceinline Vec3fa( const float x, const float y, const float z, const int      a) : x(x), y(y), z(z), a(a) {} // not working properly!
    //__forceinline Vec3fa( const float x, const float y, const float z, const unsigned a) : x(x), y(y), z(z), u(a) {} // not working properly!
    __forceinline Vec3fa( const float x, const float y, const float z, const float w) : x(x), y(y), z(z), w(w) {}

    //__forceinline explicit Vec3fa( const __m128i a ) : m128(_mm_cvtepi32_ps(a)) {}

    //__forceinline operator const __m128&() const { return m128; }
    //__forceinline operator       __m128&()       { return m128; }

    friend __forceinline Vec3fa copy_a( const Vec3fa& a, const Vec3fa& b ) { Vec3fa c = a; c.a = b.a; return c; }

    ////////////////////////////////////////////////////////////////////////////////
    /// Loads and Stores
    ////////////////////////////////////////////////////////////////////////////////

    static __forceinline Vec3fa load( const void* const a ) {
      const float* ptr = (const float*)a;
      return Vec3fa(ptr[0],ptr[1],ptr[2],ptr[3]);
    }

    static __forceinline Vec3fa loadu( const void* const a ) {
      const float* ptr = (const float*)a;
      return Vec3fa(ptr[0],ptr[1],ptr[2],ptr[3]);
    }

    static __forceinline void storeu ( void* a, const Vec3fa& v ) {
      float* ptr = (float*)a;
      ptr[0] = v.x; ptr[1] = v.y; ptr[2] = v.z; ptr[3] = v.w;
    }

    ////////////////////////////////////////////////////////////////////////////////
    /// Constants
    ////////////////////////////////////////////////////////////////////////////////

    __forceinline Vec3fa( ZeroTy   ) : x(0.0f), y(0.0f), z(0.0f), w(0.0f) {}
    __forceinline Vec3fa( OneTy    ) : x(1.0f), y(1.0f), z(1.0f), w(1.0f) {}
    __forceinline Vec3fa( PosInfTy ) : x(+INFINITY), y(+INFINITY), z(+INFINITY), w(+INFINITY) {}
    __forceinline Vec3fa( NegInfTy ) : x(-INFINITY), y(-INFINITY), z(-INFINITY), w(-INFINITY) {}

    ////////////////////////////////////////////////////////////////////////////////
    /// Array Access
    ////////////////////////////////////////////////////////////////////////////////

    //__forceinline const float& operator []( const size_t index ) const { assert(index < 3); return (&x)[index]; }
    //__forceinline       float& operator []( const size_t index )       { assert(index < 3); return (&x)[index]; }
  };

  ////////////////////////////////////////////////////////////////////////////////
  /// Unary Operators
  ////////////////////////////////////////////////////////////////////////////////

  __forceinline Vec3fa operator +( const Vec3fa& a ) { return a; }
  __forceinline Vec3fa operator -( const Vec3fa& a ) { return Vec3fa(-a.x,-a.y,-a.z); }
  __forceinline Vec3fa abs  ( const Vec3fa& a ) { return Vec3fa(cl::sycl::fabs(a.x),cl::sycl::fabs(a.y),cl::sycl::fabs(a.z)); }
  __forceinline Vec3fa sign ( const Vec3fa& a ) { return Vec3fa(cl::sycl::sign(a.x),cl::sycl::sign(a.y),cl::sycl::sign(a.z)); }

  //__forceinline Vec3fa rcp  ( const Vec3fa& a ) { return Vec3fa(cl::sycl::recip(a.x),cl::sycl::recip(a.y),cl::sycl::recip(a.z)); }
  __forceinline Vec3fa rcp  ( const Vec3fa& a ) { return Vec3fa(__sycl_std::__invoke_native_recip<float>(a.x),__sycl_std::__invoke_native_recip<float>(a.y),__sycl_std::__invoke_native_recip<float>(a.z)); }
  __forceinline Vec3fa sqrt ( const Vec3fa& a ) { return Vec3fa(cl::sycl::sqrt(a.x),cl::sycl::sqrt(a.y),cl::sycl::sqrt(a.z)); }
  __forceinline Vec3fa sqr  ( const Vec3fa& a ) { return Vec3fa(a.x*a.x,a.y*a.y,a.z*a.z); }

  __forceinline Vec3fa rsqrt( const Vec3fa& a ) { return Vec3fa(cl::sycl::rsqrt(a.x),cl::sycl::rsqrt(a.y),cl::sycl::rsqrt(a.z)); }

  __forceinline Vec3fa zero_fix(const Vec3fa& a) {
    const float x = cl::sycl::fabs(a.x) < min_rcp_input ? min_rcp_input : a.x;
    const float y = cl::sycl::fabs(a.y) < min_rcp_input ? min_rcp_input : a.y;
    const float z = cl::sycl::fabs(a.z) < min_rcp_input ? min_rcp_input : a.z;
    return Vec3fa(x,y,z);
  }
  __forceinline Vec3fa rcp_safe(const Vec3fa& a) {
    return rcp(zero_fix(a));
  }
  //__forceinline Vec3fa log ( const Vec3fa& a ) {
  //  return Vec3fa(logf(a.x),logf(a.y),logf(a.z));
  //}

  //__forceinline Vec3fa exp ( const Vec3fa& a ) {
  //  return Vec3fa(expf(a.x),expf(a.y),expf(a.z));
  //}

  ////////////////////////////////////////////////////////////////////////////////
  /// Binary Operators
  ////////////////////////////////////////////////////////////////////////////////

  __forceinline Vec3fa operator +( const Vec3fa& a, const Vec3fa& b ) { return Vec3fa(a.x+b.x, a.y+b.y, a.z+b.z); }
  __forceinline Vec3fa operator -( const Vec3fa& a, const Vec3fa& b ) { return Vec3fa(a.x-b.x, a.y-b.y, a.z-b.z); }
  __forceinline Vec3fa operator *( const Vec3fa& a, const Vec3fa& b ) { return Vec3fa(a.x*b.x, a.y*b.y, a.z*b.z); }
  __forceinline Vec3fa operator *( const Vec3fa& a, const float b ) { return a * Vec3fa(b); }
  __forceinline Vec3fa operator *( const float a, const Vec3fa& b ) { return Vec3fa(a) * b; }
  __forceinline Vec3fa operator /( const Vec3fa& a, const Vec3fa& b ) { return Vec3fa(a.x/b.x, a.y/b.y, a.z/b.z); }
  __forceinline Vec3fa operator /( const Vec3fa& a, const float b        ) { return Vec3fa(a.x/b, a.y/b, a.z/b); }
  __forceinline Vec3fa operator /( const        float a, const Vec3fa& b ) { return Vec3fa(a/b.x, a/b.y, a/b.z); }

  __forceinline Vec3fa min( const Vec3fa& a, const Vec3fa& b ) {
    return Vec3fa(cl::sycl::fmin(a.x,b.x), cl::sycl::fmin(a.y,b.y), cl::sycl::fmin(a.z,b.z));
  }
  __forceinline Vec3fa max( const Vec3fa& a, const Vec3fa& b ) {
    return Vec3fa(cl::sycl::fmax(a.x,b.x), cl::sycl::fmax(a.y,b.y), cl::sycl::fmax(a.z,b.z));
  }

/*
#if defined(__SSE4_1__)
    __forceinline Vec3fa mini(const Vec3fa& a, const Vec3fa& b) {
      const vint4 ai = _mm_castps_si128(a);
      const vint4 bi = _mm_castps_si128(b);
      const vint4 ci = _mm_min_epi32(ai,bi);
      return _mm_castsi128_ps(ci);
    }
#endif

#if defined(__SSE4_1__)
    __forceinline Vec3fa maxi(const Vec3fa& a, const Vec3fa& b) {
      const vint4 ai = _mm_castps_si128(a);
      const vint4 bi = _mm_castps_si128(b);
      const vint4 ci = _mm_max_epi32(ai,bi);
      return _mm_castsi128_ps(ci);
    }
#endif

    __forceinline Vec3fa pow ( const Vec3fa& a, const float& b ) {
      return Vec3fa(powf(a.x,b),powf(a.y,b),powf(a.z,b));
    }
*/
  
  ////////////////////////////////////////////////////////////////////////////////
  /// Ternary Operators
  ////////////////////////////////////////////////////////////////////////////////

  __forceinline Vec3fa madd  ( const Vec3fa& a, const Vec3fa& b, const Vec3fa& c) { return Vec3fa(madd(a.x,b.x,c.x), madd(a.y,b.y,c.y), madd(a.z,b.z,c.z)); }
  __forceinline Vec3fa msub  ( const Vec3fa& a, const Vec3fa& b, const Vec3fa& c) { return Vec3fa(msub(a.x,b.x,c.x), msub(a.y,b.y,c.y), msub(a.z,b.z,c.z)); }
  __forceinline Vec3fa nmadd ( const Vec3fa& a, const Vec3fa& b, const Vec3fa& c) { return Vec3fa(nmadd(a.x,b.x,c.x), nmadd(a.y,b.y,c.y), nmadd(a.z,b.z,c.z)); }
  __forceinline Vec3fa nmsub ( const Vec3fa& a, const Vec3fa& b, const Vec3fa& c) { return Vec3fa(nmsub(a.x,b.x,c.x), nmsub(a.y,b.y,c.y), nmsub(a.z,b.z,c.z)); }

  __forceinline Vec3fa madd  ( const float a, const Vec3fa& b, const Vec3fa& c) { return madd(Vec3fa(a),b,c); }
  __forceinline Vec3fa msub  ( const float a, const Vec3fa& b, const Vec3fa& c) { return msub(Vec3fa(a),b,c); }
  __forceinline Vec3fa nmadd ( const float a, const Vec3fa& b, const Vec3fa& c) { return nmadd(Vec3fa(a),b,c); }
  __forceinline Vec3fa nmsub ( const float a, const Vec3fa& b, const Vec3fa& c) { return nmsub(Vec3fa(a),b,c); }

  ////////////////////////////////////////////////////////////////////////////////
  /// Assignment Operators
  ////////////////////////////////////////////////////////////////////////////////

  __forceinline Vec3fa& operator +=( Vec3fa& a, const Vec3fa& b ) { return a = a + b; }
  __forceinline Vec3fa& operator -=( Vec3fa& a, const Vec3fa& b ) { return a = a - b; }
  __forceinline Vec3fa& operator *=( Vec3fa& a, const Vec3fa& b ) { return a = a * b; }
  __forceinline Vec3fa& operator *=( Vec3fa& a, const float   b ) { return a = a * b; }
  __forceinline Vec3fa& operator /=( Vec3fa& a, const Vec3fa& b ) { return a = a / b; }
  __forceinline Vec3fa& operator /=( Vec3fa& a, const float   b ) { return a = a / b; }

  ////////////////////////////////////////////////////////////////////////////////
  /// Reductions
  ////////////////////////////////////////////////////////////////////////////////

  __forceinline float reduce_add(const Vec3fa& v) { return v.x+v.y+v.z; }
  __forceinline float reduce_mul(const Vec3fa& v) { return v.x*v.y*v.z; }
  __forceinline float reduce_min(const Vec3fa& v) { return cl::sycl::fmin(cl::sycl::fmin(v.x,v.y),v.z); }
  __forceinline float reduce_max(const Vec3fa& v) { return cl::sycl::fmax(cl::sycl::fmax(v.x,v.y),v.z); }

  ////////////////////////////////////////////////////////////////////////////////
  /// Comparison Operators
  ////////////////////////////////////////////////////////////////////////////////

  __forceinline bool operator ==( const Vec3fa& a, const Vec3fa& b ) { return a.x == b.x && a.y == b.y && a.z == b.z; }
  __forceinline bool operator !=( const Vec3fa& a, const Vec3fa& b ) { return a.x != b.x || a.y != b.y || a.z != b.z; }

  __forceinline Vec3ba eq_mask( const Vec3fa& a, const Vec3fa& b ) { return Vec3ba(a.x == b.x, a.y == b.y, a.z == b.z); }
  __forceinline Vec3ba neq_mask(const Vec3fa& a, const Vec3fa& b ) { return Vec3ba(a.x != b.x, a.y != b.y, a.z != b.z); }
  __forceinline Vec3ba lt_mask( const Vec3fa& a, const Vec3fa& b ) { return Vec3ba(a.x <  b.x, a.y <  b.y, a.z <  b.z); }
  __forceinline Vec3ba le_mask( const Vec3fa& a, const Vec3fa& b ) { return Vec3ba(a.x <= b.x, a.y <= b.y, a.z <= b.z); }
  __forceinline Vec3ba gt_mask( const Vec3fa& a, const Vec3fa& b ) { return Vec3ba(a.x >  b.x, a.y >  b.y, a.z >  b.z); }
  __forceinline Vec3ba ge_mask( const Vec3fa& a, const Vec3fa& b ) { return Vec3ba(a.x >= b.x, a.y >= b.y, a.z >= b.z); }

  __forceinline bool isvalid ( const Vec3fa& v ) {
    return all(gt_mask(v,Vec3fa(-FLT_LARGE)) & lt_mask(v,Vec3fa(+FLT_LARGE)));
  }

  __forceinline bool is_finite ( const Vec3fa& a ) {
    return all(ge_mask(a,Vec3fa(-FLT_MAX)) & le_mask(a,Vec3fa(+FLT_MAX)));
  }

  ////////////////////////////////////////////////////////////////////////////////
  /// Euclidian Space Operators
  ////////////////////////////////////////////////////////////////////////////////

  __forceinline float dot ( const Vec3fa& a, const Vec3fa& b ) {
    return reduce_add(a*b);
  }

  __forceinline Vec3fa cross ( const Vec3fa& a, const Vec3fa& b ) {
    return Vec3fa(msub(a.y,b.z,a.z*b.y), msub(a.z,b.x,a.x*b.z), msub(a.x,b.y,a.y*b.x));
  }
  
  __forceinline float  sqr_length ( const Vec3fa& a )                { return dot(a,a); }
  __forceinline float  rcp_length ( const Vec3fa& a )                { return rsqrt(dot(a,a)); }
  __forceinline float  rcp_length2( const Vec3fa& a )                { return rcp(dot(a,a)); }
  __forceinline float  length   ( const Vec3fa& a )                  { return sqrt(dot(a,a)); }
  __forceinline Vec3fa normalize( const Vec3fa& a )                  { return a*rsqrt(dot(a,a)); }
  __forceinline float  distance ( const Vec3fa& a, const Vec3fa& b ) { return length(a-b); }
  __forceinline float  halfArea ( const Vec3fa& d )                  { return madd(d.x,(d.y+d.z),d.y*d.z); }
  __forceinline float  area     ( const Vec3fa& d )                  { return 2.0f*halfArea(d); }

  __forceinline Vec3fa normalize_safe( const Vec3fa& a ) {
    const float d = dot(a,a); if (unlikely(d == 0.0f)) return a; else return a*rsqrt(d);
  }

  /*! differentiated normalization */
  __forceinline Vec3fa dnormalize(const Vec3fa& p, const Vec3fa& dp)
  {
    const float pp  = dot(p,p);
    const float pdp = dot(p,dp);
    return (pp*dp-pdp*p)*rcp(pp)*rsqrt(pp);
  }

  ////////////////////////////////////////////////////////////////////////////////
  /// Select
  ////////////////////////////////////////////////////////////////////////////////

  __forceinline Vec3fa select( bool s, const Vec3fa& t, const Vec3fa& f ) {
    return Vec3fa(s ? t.x : f.x, s ? t.y : f.y, s ? t.z : f.z);
  }

/*
  __forceinline Vec3fa select( const Vec3ba& s, const Vec3fa& t, const Vec3fa& f ) {
    return blendv_ps(f, t, s);
  }
*/
  
  __forceinline Vec3fa lerp(const Vec3fa& v0, const Vec3fa& v1, const float t) {
    return madd(1.0f-t,v0,t*v1);
  }

  __forceinline int maxDim ( const Vec3fa& a )
  {
    const Vec3fa b = abs(a);
    if (b.x > b.y) {
      if (b.x > b.z) return 0; else return 2;
    } else {
      if (b.y > b.z) return 1; else return 2;
    }
  }

  ////////////////////////////////////////////////////////////////////////////////
  /// Rounding Functions
  ////////////////////////////////////////////////////////////////////////////////

  __forceinline Vec3fa trunc( const Vec3fa& a ) { return Vec3fa(cl::sycl::trunc(a.x),cl::sycl::trunc(a.y),cl::sycl::trunc(a.z)); }
  __forceinline Vec3fa floor( const Vec3fa& a ) { return Vec3fa(cl::sycl::floor(a.x),cl::sycl::floor(a.y),cl::sycl::floor(a.z)); }
  __forceinline Vec3fa ceil ( const Vec3fa& a ) { return Vec3fa(cl::sycl::ceil (a.x),cl::sycl::ceil (a.y),cl::sycl::ceil (a.z)); }

  ////////////////////////////////////////////////////////////////////////////////
  /// Output Operators
  ////////////////////////////////////////////////////////////////////////////////

  inline std::ostream& operator<<(std::ostream& cout, const Vec3fa& a) {
    return cout;
  }

  typedef Vec3fa Vec3fa_t;
}
