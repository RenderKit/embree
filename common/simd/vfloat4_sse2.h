// ======================================================================== //
// Copyright 2009-2015 Intel Corporation                                    //
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

#define EMBREE_FLOAT4

namespace embree
{
  /*! 4-wide SSE float type. */
  struct float4
  {
    typedef bool4 Mask;                    // mask type
    typedef int4 Int;                     // int type
    typedef float4 Float;                   // float type
    
    enum   { size = 4 };  // number of SIMD elements
    union { __m128 m128; float f[4]; int i[4]; }; // data

    ////////////////////////////////////////////////////////////////////////////////
    /// Constructors, Assignment & Cast Operators
    ////////////////////////////////////////////////////////////////////////////////
    
    __forceinline float4           ( ) {}
    __forceinline float4           ( const float4& other ) { m128 = other.m128; }
    __forceinline float4& operator=( const float4& other ) { m128 = other.m128; return *this; }

    __forceinline float4( const __m128 a ) : m128(a) {}
    __forceinline operator const __m128&( void ) const { return m128; }
    __forceinline operator       __m128&( void )       { return m128; }

    __forceinline float4           ( float  a ) : m128(_mm_set1_ps(a)) {}
    __forceinline float4           ( float  a, float  b, float  c, float  d) : m128(_mm_set_ps(d, c, b, a)) {}

    __forceinline explicit float4( const __m128i a ) : m128(_mm_cvtepi32_ps(a)) {}

    ////////////////////////////////////////////////////////////////////////////////
    /// Constants
    ////////////////////////////////////////////////////////////////////////////////

    __forceinline float4( ZeroTy   ) : m128(_mm_setzero_ps()) {}
    __forceinline float4( OneTy    ) : m128(_mm_set1_ps(1.0f)) {}
    __forceinline float4( PosInfTy ) : m128(_mm_set1_ps(pos_inf)) {}
    __forceinline float4( NegInfTy ) : m128(_mm_set1_ps(neg_inf)) {}
    __forceinline float4( StepTy   ) : m128(_mm_set_ps(3.0f, 2.0f, 1.0f, 0.0f)) {}
    __forceinline float4( NaNTy    ) : m128(_mm_set1_ps(nan)) {}

    ////////////////////////////////////////////////////////////////////////////////
    /// Loads and Stores
    ////////////////////////////////////////////////////////////////////////////////

#if defined(__AVX__)
    static __forceinline float4 broadcast( const void* const a ) { return _mm_broadcast_ss((float*)a); }

#else
    static __forceinline float4 broadcast( const void* const a ) { return _mm_set1_ps(*(float*)a); }
#endif

    static __forceinline float4 load( const float* const a ) {
      return _mm_load_ps(a); 
    }

    static __forceinline float4 loadu( const void* const a ) {
      return _mm_loadu_ps((float*)a); 
    }

    static __forceinline float4 load_nt ( const float* ptr ) {
#if defined (__SSE4_1__)
    return _mm_castsi128_ps(_mm_stream_load_si128((__m128i*)ptr));
#else
    return _mm_load_ps(ptr); 
#endif
  }

#if defined(__SSE4_1__)
    static __forceinline float4 load( const unsigned char* const ptr ) { 
      return _mm_cvtepi32_ps(_mm_cvtepu8_epi32(_mm_loadu_si128((__m128i*)ptr)));
    }
#else
    static __forceinline float4 load( const unsigned char* const ptr ) { 
      //return _mm_cvtpu8_ps(*(__m64*)ptr); // don't enable, will use MMX instructions
      return float4(ptr[0],ptr[1],ptr[2],ptr[3]);
    }
#endif

    static __forceinline float4 load(const unsigned short* const ptr) {
      return _mm_mul_ps(float4(int4::load(ptr)),float4(1.0f/65535.0f));
    } 

    static __forceinline void store ( void* ptr, const float4& v ) {  
      _mm_store_ps((float*)ptr,v); 
    }

    static __forceinline void storeu ( void* ptr, const float4& v ) {
      _mm_storeu_ps((float*)ptr,v);
    }

    static __forceinline void storeu ( float* ptr, const float4& v, size_t n) // FIXME: is there a better way of implementing this
    { 
      if (n >= 1) ptr[0] = v[0];
      if (n >= 2) ptr[1] = v[1];
      if (n >= 3) ptr[2] = v[2];
      if (n >= 4) ptr[3] = v[3];
    }

    static __forceinline void store_nt ( float4* ptr, const float4& v) 
    {
#if defined (__SSE4_1__)
      _mm_stream_ps((float*)ptr,v);
#else
      _mm_store_ps((float*)ptr,v);
#endif
    }
    
    static __forceinline void store ( const bool4& mask, void* ptr, const float4& f ) 
    { 
#if defined (__AVX__)
      _mm_maskstore_ps((float*)ptr,(__m128i)mask,f);
#else
      *(float4*)ptr = select(mask,f,*(float4*)ptr);
#endif
    }

    static __forceinline void storeu( const bool4& mask, void* ptr, const float4& f ) { 
      return _mm_storeu_ps((float*)ptr,select(mask,f,_mm_loadu_ps((float*)ptr)));
    }

    static __forceinline void store ( const bool4& mask, void* ptr, const int4& ofs, const float4& v, const int scale = 1 )
    {
      if (likely(mask[0])) *(float*)(((char*)ptr)+scale*ofs[0]) = v[0];
      if (likely(mask[1])) *(float*)(((char*)ptr)+scale*ofs[1]) = v[1];
      if (likely(mask[2])) *(float*)(((char*)ptr)+scale*ofs[2]) = v[2];
      if (likely(mask[3])) *(float*)(((char*)ptr)+scale*ofs[3]) = v[3];
    }
    static __forceinline void store ( const bool4& mask, char* ptr, const int4& ofs, const float4& v ) {
      store(mask,ptr,ofs,v,1);
    }
    static __forceinline void store ( const bool4& mask, float* ptr, const int4& ofs, const float4& v ) {
      store(mask,ptr,ofs,v,4);
    }
    
    ////////////////////////////////////////////////////////////////////////////////
    /// Array Access
    ////////////////////////////////////////////////////////////////////////////////

    __forceinline const float& operator []( const size_t i ) const { assert(i < 4); return f[i]; }
    __forceinline       float& operator []( const size_t i )       { assert(i < 4); return f[i]; }

    friend __forceinline const float4 select( const bool4& m, const float4& t, const float4& f ) { 
#if defined(__SSE4_1__)
      return _mm_blendv_ps(f, t, m); 
#else
      return _mm_or_ps(_mm_and_ps(m, t), _mm_andnot_ps(m, f)); 
#endif
    }
  };


  ////////////////////////////////////////////////////////////////////////////////
  /// Unary Operators
  ////////////////////////////////////////////////////////////////////////////////

  __forceinline const float4 cast      (const __m128i& a) { return _mm_castsi128_ps(a); }
  __forceinline const float4 operator +( const float4& a ) { return a; }
  __forceinline const float4 operator -( const float4& a ) { return _mm_xor_ps(a.m128, _mm_castsi128_ps(_mm_set1_epi32(0x80000000))); }
  __forceinline const float4 abs       ( const float4& a ) { return _mm_and_ps(a.m128, _mm_castsi128_ps(_mm_set1_epi32(0x7fffffff))); }
  __forceinline const float4 sign      ( const float4& a ) { return blendv_ps(float4(one), -float4(one), _mm_cmplt_ps (a,float4(zero))); }
  __forceinline const float4 signmsk   ( const float4& a ) { return _mm_and_ps(a.m128,_mm_castsi128_ps(_mm_set1_epi32(0x80000000))); }
  
  __forceinline const float4 rcp  ( const float4& a ) {
    const float4 r = _mm_rcp_ps(a.m128);
#if defined(__AVX2__)
    return _mm_mul_ps(r,_mm_fnmadd_ps(r, a, float4(2.0f)));     
#else
    return _mm_mul_ps(r,_mm_sub_ps(float4(2.0f), _mm_mul_ps(r, a)));     
#endif
  }
  __forceinline const float4 sqr  ( const float4& a ) { return _mm_mul_ps(a,a); }
  __forceinline const float4 sqrt ( const float4& a ) { return _mm_sqrt_ps(a.m128); }
  __forceinline const float4 rsqrt( const float4& a ) {
    const float4 r = _mm_rsqrt_ps(a.m128);
    return _mm_add_ps(_mm_mul_ps(_mm_set_ps(1.5f, 1.5f, 1.5f, 1.5f), r),
                      _mm_mul_ps(_mm_mul_ps(_mm_mul_ps(a, _mm_set_ps(-0.5f, -0.5f, -0.5f, -0.5f)), r), _mm_mul_ps(r, r)));
  }

  ////////////////////////////////////////////////////////////////////////////////
  /// Binary Operators
  ////////////////////////////////////////////////////////////////////////////////

  __forceinline const float4 operator +( const float4& a, const float4& b ) { return _mm_add_ps(a.m128, b.m128); }
  __forceinline const float4 operator +( const float4& a, const float& b ) { return a + float4(b); }
  __forceinline const float4 operator +( const float& a, const float4& b ) { return float4(a) + b; }

  __forceinline const float4 operator -( const float4& a, const float4& b ) { return _mm_sub_ps(a.m128, b.m128); }
  __forceinline const float4 operator -( const float4& a, const float& b ) { return a - float4(b); }
  __forceinline const float4 operator -( const float& a, const float4& b ) { return float4(a) - b; }

  __forceinline const float4 operator *( const float4& a, const float4& b ) { return _mm_mul_ps(a.m128, b.m128); }
  __forceinline const float4 operator *( const float4& a, const float& b ) { return a * float4(b); }
  __forceinline const float4 operator *( const float& a, const float4& b ) { return float4(a) * b; }

  __forceinline const float4 operator /( const float4& a, const float4& b ) { return _mm_div_ps(a.m128,b.m128); }
  __forceinline const float4 operator /( const float4& a, const float& b ) { return a/float4(b); }
  __forceinline const float4 operator /( const float& a, const float4& b ) { return float4(a)/b; }

  __forceinline const float4 operator^( const float4& a, const float4& b ) { return _mm_xor_ps(a.m128,b.m128); }
  __forceinline const float4 operator^( const float4& a, const int4& b ) { return _mm_xor_ps(a.m128,_mm_castsi128_ps(b.m128)); }

  __forceinline const float4 min( const float4& a, const float4& b ) { return _mm_min_ps(a.m128,b.m128); }
  __forceinline const float4 min( const float4& a, const float& b ) { return _mm_min_ps(a.m128,float4(b)); }
  __forceinline const float4 min( const float& a, const float4& b ) { return _mm_min_ps(float4(a),b.m128); }

  __forceinline const float4 max( const float4& a, const float4& b ) { return _mm_max_ps(a.m128,b.m128); }
  __forceinline const float4 max( const float4& a, const float& b ) { return _mm_max_ps(a.m128,float4(b)); }
  __forceinline const float4 max( const float& a, const float4& b ) { return _mm_max_ps(float4(a),b.m128); }

#if defined(__SSE4_1__)
    __forceinline float4 mini(const float4& a, const float4& b) {
      const int4 ai = _mm_castps_si128(a);
      const int4 bi = _mm_castps_si128(b);
      const int4 ci = _mm_min_epi32(ai,bi);
      return _mm_castsi128_ps(ci);
    }
#endif
    
#if defined(__SSE4_1__)
    __forceinline float4 maxi(const float4& a, const float4& b) {
      const int4 ai = _mm_castps_si128(a);
      const int4 bi = _mm_castps_si128(b);
      const int4 ci = _mm_max_epi32(ai,bi);
      return _mm_castsi128_ps(ci);
    }
#endif

  ////////////////////////////////////////////////////////////////////////////////
  /// Ternary Operators
  ////////////////////////////////////////////////////////////////////////////////

#if defined(__AVX2__)
  __forceinline const float4 madd  ( const float4& a, const float4& b, const float4& c) { return _mm_fmadd_ps(a,b,c); }
  __forceinline const float4 msub  ( const float4& a, const float4& b, const float4& c) { return _mm_fmsub_ps(a,b,c); }
  __forceinline const float4 nmadd ( const float4& a, const float4& b, const float4& c) { return _mm_fnmadd_ps(a,b,c); }
  __forceinline const float4 nmsub ( const float4& a, const float4& b, const float4& c) { return _mm_fnmsub_ps(a,b,c); }
#else
  __forceinline const float4 madd  ( const float4& a, const float4& b, const float4& c) { return a*b+c; }
  __forceinline const float4 msub  ( const float4& a, const float4& b, const float4& c) { return a*b-c; }
  __forceinline const float4 nmadd ( const float4& a, const float4& b, const float4& c) { return -a*b+c;}
  __forceinline const float4 nmsub ( const float4& a, const float4& b, const float4& c) { return -a*b-c; }
#endif

  ////////////////////////////////////////////////////////////////////////////////
  /// Assignment Operators
  ////////////////////////////////////////////////////////////////////////////////

  __forceinline float4& operator +=( float4& a, const float4& b ) { return a = a + b; }
  __forceinline float4& operator +=( float4& a, const float& b ) { return a = a + b; }

  __forceinline float4& operator -=( float4& a, const float4& b ) { return a = a - b; }
  __forceinline float4& operator -=( float4& a, const float& b ) { return a = a - b; }

  __forceinline float4& operator *=( float4& a, const float4& b ) { return a = a * b; }
  __forceinline float4& operator *=( float4& a, const float& b ) { return a = a * b; }

  __forceinline float4& operator /=( float4& a, const float4& b ) { return a = a / b; }
  __forceinline float4& operator /=( float4& a, const float& b ) { return a = a / b; }

  ////////////////////////////////////////////////////////////////////////////////
  /// Comparison Operators + Select
  ////////////////////////////////////////////////////////////////////////////////

  __forceinline const bool4 operator ==( const float4& a, const float4& b ) { return _mm_cmpeq_ps (a.m128, b.m128); }
  __forceinline const bool4 operator ==( const float4& a, const float& b ) { return a == float4(b); }
  __forceinline const bool4 operator ==( const float& a, const float4& b ) { return float4(a) == b; }

  __forceinline const bool4 operator !=( const float4& a, const float4& b ) { return _mm_cmpneq_ps(a.m128, b.m128); }
  __forceinline const bool4 operator !=( const float4& a, const float& b ) { return a != float4(b); }
  __forceinline const bool4 operator !=( const float& a, const float4& b ) { return float4(a) != b; }

  __forceinline const bool4 operator < ( const float4& a, const float4& b ) { return _mm_cmplt_ps (a.m128, b.m128); }
  __forceinline const bool4 operator < ( const float4& a, const float& b ) { return a <  float4(b); }
  __forceinline const bool4 operator < ( const float& a, const float4& b ) { return float4(a) <  b; }

  __forceinline const bool4 operator >=( const float4& a, const float4& b ) { return _mm_cmpnlt_ps(a.m128, b.m128); }
  __forceinline const bool4 operator >=( const float4& a, const float& b ) { return a >= float4(b); }
  __forceinline const bool4 operator >=( const float& a, const float4& b ) { return float4(a) >= b; }

  __forceinline const bool4 operator > ( const float4& a, const float4& b ) { return _mm_cmpnle_ps(a.m128, b.m128); }
  __forceinline const bool4 operator > ( const float4& a, const float& b ) { return a >  float4(b); }
  __forceinline const bool4 operator > ( const float& a, const float4& b ) { return float4(a) >  b; }

  __forceinline const bool4 operator <=( const float4& a, const float4& b ) { return _mm_cmple_ps (a.m128, b.m128); }
  __forceinline const bool4 operator <=( const float4& a, const float& b ) { return a <= float4(b); }
  __forceinline const bool4 operator <=( const float& a, const float4& b ) { return float4(a) <= b; }

  
#if defined(__SSE4_1__) 
#if defined(__clang__) || defined(_MSC_VER) && (!defined(__INTEL_COMPILER) || defined(_DEBUG))
  __forceinline const float4 select(const int mask, const float4& t, const float4& f) {
    return select(bool4(mask), t, f);
  }
#else
  __forceinline const float4 select(const int mask, const float4& t, const float4& f) {
    return _mm_blend_ps(f, t, mask);
  }
#endif
#endif

  __forceinline float4  lerp(const float4& a, const float4& b, const float4& t) {
#if defined(__AVX2__)
    return madd(t, b, madd(-t, a, a));
#else
    return a + t*(b-a);
#endif
  }
  
  __forceinline bool isvalid ( const float4& v ) {
    return all((v > float4(-FLT_LARGE)) & (v < float4(+FLT_LARGE)));
  }

  __forceinline bool is_finite ( const float4& a ) { 
    return all((a >= float4(-FLT_MAX) & (a <= float4(+FLT_MAX))));
  }

  __forceinline bool is_finite ( const bool4& valid, const float4& a ) { 
    return all(valid, a >= float4(-FLT_MAX) & (a <= float4(+FLT_MAX)));
  }
  
  ////////////////////////////////////////////////////////////////////////////////
  /// Rounding Functions
  ////////////////////////////////////////////////////////////////////////////////

#if defined (__SSE4_1__)
  __forceinline const float4 floor     ( const float4& a ) { return _mm_round_ps(a, _MM_FROUND_TO_NEG_INF    ); }
  __forceinline const float4 ceil      ( const float4& a ) { return _mm_round_ps(a, _MM_FROUND_TO_POS_INF    ); }
  __forceinline const float4 trunc     ( const float4& a ) { return _mm_round_ps(a, _MM_FROUND_TO_ZERO       ); }
#else
  __forceinline const float4 floor     ( const float4& a ) { return float4(floorf(a[0]),floorf(a[1]),floorf(a[2]),floorf(a[3]));  }
  __forceinline const float4 ceil      ( const float4& a ) { return float4(ceilf (a[0]),ceilf (a[1]),ceilf (a[2]),ceilf (a[3])); }
  //__forceinline const float4 trunc     ( const float4& a ) { return float4(truncf(a[0]),truncf(a[1]),truncf(a[2]),truncf(a[3])); }
#endif
  __forceinline const float4 frac      ( const float4& a ) { return a-floor(a); }

  __forceinline int4 floori (const float4& a) {
#if defined (__SSE4_1__)
    return int4(floor(a));
#else
    return int4(a-float4(0.5f));
#endif
  }

  ////////////////////////////////////////////////////////////////////////////////
  /// Movement/Shifting/Shuffling Functions
  ////////////////////////////////////////////////////////////////////////////////

  __forceinline float4 unpacklo( const float4& a, const float4& b ) { return _mm_unpacklo_ps(a.m128, b.m128); }
  __forceinline float4 unpackhi( const float4& a, const float4& b ) { return _mm_unpackhi_ps(a.m128, b.m128); }

  template<size_t i0, size_t i1, size_t i2, size_t i3> __forceinline const float4 shuffle( const float4& b ) {
    return _mm_castsi128_ps(_mm_shuffle_epi32(_mm_castps_si128(b), _MM_SHUFFLE(i3, i2, i1, i0)));
  }

  template<size_t i0, size_t i1, size_t i2, size_t i3> __forceinline const float4 shuffle( const float4& a, const float4& b ) {
    return _mm_shuffle_ps(a, b, _MM_SHUFFLE(i3, i2, i1, i0));
  }

#if defined (__SSSE3__)
  __forceinline const float4 shuffle8(const float4& a, const int4& shuf) { 
    return _mm_castsi128_ps(_mm_shuffle_epi8(_mm_castps_si128(a), shuf)); 
  }
#endif

#if defined(__SSE3__)
  template<> __forceinline const float4 shuffle<0, 0, 2, 2>( const float4& b ) { return _mm_moveldup_ps(b); }
  template<> __forceinline const float4 shuffle<1, 1, 3, 3>( const float4& b ) { return _mm_movehdup_ps(b); }
  template<> __forceinline const float4 shuffle<0, 1, 0, 1>( const float4& b ) { return _mm_castpd_ps(_mm_movedup_pd(_mm_castps_pd(b))); }
#endif

  template<size_t i0> __forceinline const float4 shuffle( const float4& b ) {
    return shuffle<i0,i0,i0,i0>(b);
  }

#if defined (__SSE4_1__) && !defined(__GNUC__)
  template<size_t i> __forceinline float extract   ( const float4& a ) { return _mm_cvtss_f32(_mm_extract_ps(a,i)); }
#else
  template<size_t i> __forceinline float extract   ( const float4& a ) { return _mm_cvtss_f32(shuffle<i,i,i,i>(a)); }
#endif
  template<>         __forceinline float extract<0>( const float4& a ) { return _mm_cvtss_f32(a); }

#if defined (__SSE4_1__)
  template<size_t dst, size_t src, size_t clr> __forceinline const float4 insert( const float4& a, const float4& b ) { return _mm_insert_ps(a, b, (dst << 4) | (src << 6) | clr); }
  template<size_t dst, size_t src> __forceinline const float4 insert( const float4& a, const float4& b ) { return insert<dst, src, 0>(a, b); }
  template<size_t dst>             __forceinline const float4 insert( const float4& a, const float b ) { return insert<dst,      0>(a, _mm_set_ss(b)); }
#else
  template<size_t dst, size_t src> __forceinline const float4 insert( const float4& a, const float4& b ) { float4 c = a; c[dst] = b[src]; return c; }
  template<size_t dst>             __forceinline const float4 insert( const float4& a, const float b ) { float4 c = a; c[dst] = b; return c; }
#endif

  __forceinline float4 broadcast4f( const float4& a, const size_t k ) {  
    return float4::broadcast(&a[k]);
  }

#if defined (__AVX2__)
  __forceinline float4 permute(const float4 &a, const __m128i &index) {
    return _mm_permutevar_ps(a,index);
  }

  __forceinline float4 broadcast1f( const void* const a ) { return _mm_broadcast_ss((float*)a); }

#endif


  ////////////////////////////////////////////////////////////////////////////////
  /// Transpose
  ////////////////////////////////////////////////////////////////////////////////

  __forceinline void transpose(const float4& r0, const float4& r1, const float4& r2, const float4& r3, float4& c0, float4& c1, float4& c2, float4& c3) 
  {
    float4 l02 = unpacklo(r0,r2);
    float4 h02 = unpackhi(r0,r2);
    float4 l13 = unpacklo(r1,r3);
    float4 h13 = unpackhi(r1,r3);
    c0 = unpacklo(l02,l13);
    c1 = unpackhi(l02,l13);
    c2 = unpacklo(h02,h13);
    c3 = unpackhi(h02,h13);
  }

  __forceinline void transpose(const float4& r0, const float4& r1, const float4& r2, const float4& r3, float4& c0, float4& c1, float4& c2) 
  {
    float4 l02 = unpacklo(r0,r2);
    float4 h02 = unpackhi(r0,r2);
    float4 l13 = unpacklo(r1,r3);
    float4 h13 = unpackhi(r1,r3);
    c0 = unpacklo(l02,l13);
    c1 = unpackhi(l02,l13);
    c2 = unpacklo(h02,h13);
  }

  ////////////////////////////////////////////////////////////////////////////////
  /// Reductions
  ////////////////////////////////////////////////////////////////////////////////

  __forceinline const float4 vreduce_min(const float4& v) { float4 h = min(shuffle<1,0,3,2>(v),v); return min(shuffle<2,3,0,1>(h),h); }
  __forceinline const float4 vreduce_max(const float4& v) { float4 h = max(shuffle<1,0,3,2>(v),v); return max(shuffle<2,3,0,1>(h),h); }
  __forceinline const float4 vreduce_add(const float4& v) { float4 h = shuffle<1,0,3,2>(v)   + v ; return shuffle<2,3,0,1>(h)   + h ; }

  __forceinline float reduce_min(const float4& v) { return _mm_cvtss_f32(vreduce_min(v)); }
  __forceinline float reduce_max(const float4& v) { return _mm_cvtss_f32(vreduce_max(v)); }
  __forceinline float reduce_add(const float4& v) { return _mm_cvtss_f32(vreduce_add(v)); }

  __forceinline size_t select_min(const float4& v) { return __bsf(movemask(v == vreduce_min(v))); }
  __forceinline size_t select_max(const float4& v) { return __bsf(movemask(v == vreduce_max(v))); }

  __forceinline size_t select_min(const bool4& valid, const float4& v) { const float4 a = select(valid,v,float4(pos_inf)); return __bsf(movemask(valid & (a == vreduce_min(a)))); }
  __forceinline size_t select_max(const bool4& valid, const float4& v) { const float4 a = select(valid,v,float4(neg_inf)); return __bsf(movemask(valid & (a == vreduce_max(a)))); }

  ////////////////////////////////////////////////////////////////////////////////
  /// Memory load and store operations
  ////////////////////////////////////////////////////////////////////////////////

  __forceinline float4 load4f( const void* const a ) {
    return _mm_load_ps((float*)a); 
  }

  __forceinline void store4f ( void* ptr, const float4& v ) {
    _mm_store_ps((float*)ptr,v);
  }

  __forceinline float4 loadu4f( const void* const a ) {
    return _mm_loadu_ps((float*)a); 
  }

  __forceinline void storeu4f ( void* ptr, const float4& v ) {
    _mm_storeu_ps((float*)ptr,v);
  }

  __forceinline void store4f ( const bool4& mask, void* ptr, const float4& f ) { 
#if defined (__AVX__)
    _mm_maskstore_ps((float*)ptr,(__m128i)mask,f);
#else
    *(float4*)ptr = select(mask,f,*(float4*)ptr);
#endif
  }

  __forceinline float4 load4f_nt (void* ptr) {
#if defined (__SSE4_1__)
    return _mm_castsi128_ps(_mm_stream_load_si128((__m128i*)ptr));
#else
    return _mm_load_ps((float*)ptr); 
#endif
  }

  __forceinline void store4f_nt (void* ptr, const float4& v) {
#if defined (__SSE4_1__)
    _mm_stream_ps((float*)ptr,v);
#else
    _mm_store_ps((float*)ptr,v);
#endif
  }
  
  ////////////////////////////////////////////////////////////////////////////////
  /// Euclidian Space Operators
  ////////////////////////////////////////////////////////////////////////////////

  __forceinline float dot ( const float4& a, const float4& b ) {
    return reduce_add(a*b);
  }

  __forceinline float4 cross ( const float4& a, const float4& b ) 
  {
    const float4 a0 = a;
    const float4 b0 = shuffle<1,2,0,3>(b);
    const float4 a1 = shuffle<1,2,0,3>(a);
    const float4 b1 = b;
    return shuffle<1,2,0,3>(msub(a0,b0,a1*b1));
  }

  ////////////////////////////////////////////////////////////////////////////////
  /// Output Operators
  ////////////////////////////////////////////////////////////////////////////////

  inline std::ostream& operator<<(std::ostream& cout, const float4& a) {
    return cout << "<" << a[0] << ", " << a[1] << ", " << a[2] << ", " << a[3] << ">";
  }

}
