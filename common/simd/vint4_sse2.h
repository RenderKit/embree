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

#include "../math/math.h"

namespace embree
{
  /*! 4-wide SSE integer type. */
  struct int4
  {
    typedef bool4 Mask;                   // mask type
    typedef int4 Int;                     // int type
    typedef float4 Float;                 // float type

    enum   { size = 4 };                  // number of SIMD elements
    union  { __m128i m128; int i[4]; }; // data

    ////////////////////////////////////////////////////////////////////////////////
    /// Constructors, Assignment & Cast Operators
    ////////////////////////////////////////////////////////////////////////////////
    
    __forceinline int4           ( ) {}
    __forceinline int4           ( const int4& a ) { m128 = a.m128; }
    __forceinline int4& operator=( const int4& a ) { m128 = a.m128; return *this; }

    __forceinline int4( const __m128i a ) : m128(a) {}
    __forceinline operator const __m128i&( void ) const { return m128; }
    __forceinline operator       __m128i&( void )       { return m128; }

    __forceinline int4           ( const int&  a ) : m128(_mm_shuffle_epi32(_mm_castps_si128(_mm_load_ss((float*)&a)), _MM_SHUFFLE(0, 0, 0, 0))) {}
    __forceinline int4           ( const uint32_t& a ) : m128(_mm_shuffle_epi32(_mm_castps_si128(_mm_load_ss((float*)&a)), _MM_SHUFFLE(0, 0, 0, 0))) {}
#if defined(__X86_64__)
    __forceinline int4           ( const size_t a  ) : m128(_mm_set1_epi32((int)a)) {}
#endif
    __forceinline int4           ( int  a, int  b, int  c, int  d) : m128(_mm_set_epi32(d, c, b, a)) {}

    __forceinline explicit int4( const __m128 a ) : m128(_mm_cvtps_epi32(a)) {}

    ////////////////////////////////////////////////////////////////////////////////
    /// Constants
    ////////////////////////////////////////////////////////////////////////////////

    __forceinline int4( ZeroTy   ) : m128(_mm_setzero_si128()) {}
    __forceinline int4( OneTy    ) : m128(_mm_set_epi32(1, 1, 1, 1)) {}
    __forceinline int4( PosInfTy ) : m128(_mm_set_epi32(pos_inf, pos_inf, pos_inf, pos_inf)) {}
    __forceinline int4( NegInfTy ) : m128(_mm_set_epi32(neg_inf, neg_inf, neg_inf, neg_inf)) {}
    __forceinline int4( StepTy )   : m128(_mm_set_epi32(3, 2, 1, 0)) {}
    __forceinline int4( TrueTy  ) { m128 = _mm_cmpeq_epi32(m128,m128); }


    ////////////////////////////////////////////////////////////////////////////////
    /// Loads and Stores
    ////////////////////////////////////////////////////////////////////////////////

#if defined(__SSE4_1__)
    static __forceinline int4 load( const unsigned char* const ptr ) { 
      return _mm_cvtepu8_epi32(_mm_load_si128((__m128i*)ptr));
    }
#endif

    static __forceinline int4 load(const unsigned short* const ptr)
    {
#if defined (__SSE4_1__)
      return _mm_cvtepu16_epi32(_mm_loadu_si128((__m128i*)ptr));
#else
      return int4(ptr[0],ptr[1],ptr[2],ptr[3]);
#endif
    } 

    static  __forceinline int4 load( const void* const a ) { 
      return _mm_load_si128((__m128i*)a); 
    }
    
    static __forceinline int4 loadu( const void* const a ) { 
      return _mm_loadu_si128((__m128i*)a); 
    }
    
    static __forceinline void store(void* ptr, const int4& v) {
      _mm_store_si128((__m128i*)ptr,v);
    }
    
    static __forceinline void storeu(void* ptr, const int4& v) {
      _mm_storeu_si128((__m128i*)ptr,v);
    }
    
    static __forceinline void store( const bool4& mask, void* ptr, const int4& i ) { 
#if defined (__AVX__)
      _mm_maskstore_ps((float*)ptr,(__m128i)mask,_mm_castsi128_ps(i));
#else
      *(int4*)ptr = select(mask,i,*(int4*)ptr);
#endif
    }
    
    static __forceinline int4 load_nt (void* ptr) { 
#if defined(__SSE4_1__)
      return _mm_stream_load_si128((__m128i*)ptr); 
#else
      return _mm_load_si128((__m128i*)ptr); 
#endif
    }
    
    static __forceinline void store_nt(void* ptr, const int4& v) { 
#if defined(__SSE4_1__)
      _mm_stream_ps((float*)ptr,_mm_castsi128_ps(v)); 
#else
      _mm_store_si128((__m128i*)ptr,v);
#endif
    }



    ////////////////////////////////////////////////////////////////////////////////
    /// Array Access
    ////////////////////////////////////////////////////////////////////////////////

    __forceinline const int& operator []( const size_t index ) const { assert(index < 4); return i[index]; }
    __forceinline       int& operator []( const size_t index )       { assert(index < 4); return i[index]; }

    friend __forceinline const int4 select( const bool4& m, const int4& t, const int4& f ) { 
#if defined(__SSE4_1__)
      return _mm_castps_si128(_mm_blendv_ps(_mm_castsi128_ps(f), _mm_castsi128_ps(t), m)); 
#else
      return _mm_or_si128(_mm_and_si128(m, t), _mm_andnot_si128(m, f)); 
#endif
    }
  };

  ////////////////////////////////////////////////////////////////////////////////
  /// Unary Operators
  ////////////////////////////////////////////////////////////////////////////////

  __forceinline const int4 cast      ( const __m128& a ) { return _mm_castps_si128(a); }
  __forceinline const int4 operator +( const int4& a ) { return a; }
  __forceinline const int4 operator -( const int4& a ) { return _mm_sub_epi32(_mm_setzero_si128(), a.m128); }
#if defined(__SSSE3__)
  __forceinline const int4 abs       ( const int4& a ) { return _mm_abs_epi32(a.m128); }
#endif

  ////////////////////////////////////////////////////////////////////////////////
  /// Binary Operators
  ////////////////////////////////////////////////////////////////////////////////

  __forceinline const int4 operator +( const int4& a, const int4& b ) { return _mm_add_epi32(a.m128, b.m128); }
  __forceinline const int4 operator +( const int4& a, const int&  b ) { return a + int4(b); }
  __forceinline const int4 operator +( const int&  a, const int4& b ) { return int4(a) + b; }

  __forceinline const int4 operator -( const int4& a, const int4& b ) { return _mm_sub_epi32(a.m128, b.m128); }
  __forceinline const int4 operator -( const int4& a, const int&  b ) { return a - int4(b); }
  __forceinline const int4 operator -( const int&  a, const int4& b ) { return int4(a) - b; }

#if defined(__SSE4_1__)
  __forceinline const int4 operator *( const int4& a, const int4& b ) { return _mm_mullo_epi32(a.m128, b.m128); }
#else
  __forceinline const int4 operator *( const int4& a, const int4& b ) { return int4(a[0]*b[0],a[1]*b[1],a[2]*b[2],a[3]*b[3]); }
#endif
  __forceinline const int4 operator *( const int4& a, const int&  b ) { return a * int4(b); }
  __forceinline const int4 operator *( const int&  a, const int4& b ) { return int4(a) * b; }

  __forceinline const int4 operator &( const int4& a, const int4& b ) { return _mm_and_si128(a.m128, b.m128); }
  __forceinline const int4 operator &( const int4& a, const int&  b ) { return a & int4(b); }
  __forceinline const int4 operator &( const int& a, const int4& b ) { return int4(a) & b; }

  __forceinline const int4 operator |( const int4& a, const int4& b ) { return _mm_or_si128(a.m128, b.m128); }
  __forceinline const int4 operator |( const int4& a, const int&  b ) { return a | int4(b); }
  __forceinline const int4 operator |( const int& a, const int4& b ) { return int4(a) | b; }

  __forceinline const int4 operator ^( const int4& a, const int4& b ) { return _mm_xor_si128(a.m128, b.m128); }
  __forceinline const int4 operator ^( const int4& a, const int&  b ) { return a ^ int4(b); }
  __forceinline const int4 operator ^( const int& a, const int4& b ) { return int4(a) ^ b; }

  __forceinline const int4 operator <<( const int4& a, const int& n ) { return _mm_slli_epi32(a.m128, n); }
  __forceinline const int4 operator >>( const int4& a, const int& n ) { return _mm_srai_epi32(a.m128, n); }

  __forceinline const int4 sra ( const int4& a, const int& b ) { return _mm_srai_epi32(a.m128, b); }
  __forceinline const int4 srl ( const int4& a, const int& b ) { return _mm_srli_epi32(a.m128, b); }
  
#if defined(__SSE4_1__)
  __forceinline const int4 min( const int4& a, const int4& b ) { return _mm_min_epi32(a.m128, b.m128); }
  __forceinline const int4 max( const int4& a, const int4& b ) { return _mm_max_epi32(a.m128, b.m128); }

  __forceinline const int4 umin( const int4& a, const int4& b ) { return _mm_min_epu32(a.m128, b.m128); }
  __forceinline const int4 umax( const int4& a, const int4& b ) { return _mm_max_epu32(a.m128, b.m128); }

#else
  __forceinline const int4 min( const int4& a, const int4& b ) { return int4(min(a[0],b[0]),min(a[1],b[1]),min(a[2],b[2]),min(a[3],b[3])); }
  __forceinline const int4 max( const int4& a, const int4& b ) { return int4(max(a[0],b[0]),max(a[1],b[1]),max(a[2],b[2]),max(a[3],b[3])); }
#endif

  __forceinline const int4 min( const int4& a, const int&  b ) { return min(a,int4(b)); }
  __forceinline const int4 min( const int&  a, const int4& b ) { return min(int4(a),b); }
  __forceinline const int4 max( const int4& a, const int&  b ) { return max(a,int4(b)); }
  __forceinline const int4 max( const int&  a, const int4& b ) { return max(int4(a),b); }

  ////////////////////////////////////////////////////////////////////////////////
  /// Assignment Operators
  ////////////////////////////////////////////////////////////////////////////////

  __forceinline int4& operator +=( int4& a, const int4& b ) { return a = a + b; }
  __forceinline int4& operator +=( int4& a, const int&  b ) { return a = a + b; }
  
  __forceinline int4& operator -=( int4& a, const int4& b ) { return a = a - b; }
  __forceinline int4& operator -=( int4& a, const int&  b ) { return a = a - b; }

#if defined(__SSE4_1__)
  __forceinline int4& operator *=( int4& a, const int4& b ) { return a = a * b; }
  __forceinline int4& operator *=( int4& a, const int&  b ) { return a = a * b; }
#endif
  
  __forceinline int4& operator &=( int4& a, const int4& b ) { return a = a & b; }
  __forceinline int4& operator &=( int4& a, const int&  b ) { return a = a & b; }
  
  __forceinline int4& operator |=( int4& a, const int4& b ) { return a = a | b; }
  __forceinline int4& operator |=( int4& a, const int&  b ) { return a = a | b; }
  
  __forceinline int4& operator <<=( int4& a, const int&  b ) { return a = a << b; }
  __forceinline int4& operator >>=( int4& a, const int&  b ) { return a = a >> b; }

  ////////////////////////////////////////////////////////////////////////////////
  /// Comparison Operators + Select
  ////////////////////////////////////////////////////////////////////////////////

  __forceinline const bool4 operator ==( const int4& a, const int4& b ) { return _mm_castsi128_ps(_mm_cmpeq_epi32 (a.m128, b.m128)); }
  __forceinline const bool4 operator ==( const int4& a, const int&  b ) { return a == int4(b); }
  __forceinline const bool4 operator ==( const int&  a, const int4& b ) { return int4(a) == b; }
  
  __forceinline const bool4 operator !=( const int4& a, const int4& b ) { return !(a == b); }
  __forceinline const bool4 operator !=( const int4& a, const int&  b ) { return a != int4(b); }
  __forceinline const bool4 operator !=( const int&  a, const int4& b ) { return int4(a) != b; }
  
  __forceinline const bool4 operator < ( const int4& a, const int4& b ) { return _mm_castsi128_ps(_mm_cmplt_epi32 (a.m128, b.m128)); }
  __forceinline const bool4 operator < ( const int4& a, const int&  b ) { return a <  int4(b); }
  __forceinline const bool4 operator < ( const int&  a, const int4& b ) { return int4(a) <  b; }
  
  __forceinline const bool4 operator >=( const int4& a, const int4& b ) { return !(a <  b); }
  __forceinline const bool4 operator >=( const int4& a, const int&  b ) { return a >= int4(b); }
  __forceinline const bool4 operator >=( const int&  a, const int4& b ) { return int4(a) >= b; }

  __forceinline const bool4 operator > ( const int4& a, const int4& b ) { return _mm_castsi128_ps(_mm_cmpgt_epi32 (a.m128, b.m128)); }
  __forceinline const bool4 operator > ( const int4& a, const int&  b ) { return a >  int4(b); }
  __forceinline const bool4 operator > ( const int&  a, const int4& b ) { return int4(a) >  b; }

  __forceinline const bool4 operator <=( const int4& a, const int4& b ) { return !(a >  b); }
  __forceinline const bool4 operator <=( const int4& a, const int&  b ) { return a <= int4(b); }
  __forceinline const bool4 operator <=( const int&  a, const int4& b ) { return int4(a) <= b; }

 

#if defined(__SSE4_1__) 
#if defined(__clang__) || defined(_MSC_VER) && !defined(__INTEL_COMPILER) // FIXME: can this get removed?
  __forceinline const int4 select(const int mask, const int4& t, const int4& f) {
	  return select(bool4(mask), t, f);
  }
#else
  __forceinline const int4 select(const int m, const int4& t, const int4& f) {
	  return _mm_castps_si128(_mm_blend_ps(_mm_castsi128_ps(f), _mm_castsi128_ps(t), m));
  }
#endif
#endif

  ////////////////////////////////////////////////////////////////////////////////
  // Movement/Shifting/Shuffling Functions
  ////////////////////////////////////////////////////////////////////////////////

  __forceinline int4 unpacklo( const int4& a, const int4& b ) { return _mm_castps_si128(_mm_unpacklo_ps(_mm_castsi128_ps(a.m128), _mm_castsi128_ps(b.m128))); }
  __forceinline int4 unpackhi( const int4& a, const int4& b ) { return _mm_castps_si128(_mm_unpackhi_ps(_mm_castsi128_ps(a.m128), _mm_castsi128_ps(b.m128))); }

  template<size_t i0, size_t i1, size_t i2, size_t i3> __forceinline const int4 shuffle( const int4& a ) {
    return _mm_shuffle_epi32(a, _MM_SHUFFLE(i3, i2, i1, i0));
  }

  template<size_t i0, size_t i1, size_t i2, size_t i3> __forceinline const int4 shuffle( const int4& a, const int4& b ) {
    return _mm_castps_si128(_mm_shuffle_ps(_mm_castsi128_ps(a), _mm_castsi128_ps(b), _MM_SHUFFLE(i3, i2, i1, i0)));
  }

#if defined(__SSE3__)
  template<> __forceinline const int4 shuffle<0, 0, 2, 2>( const int4& a ) { return _mm_castps_si128(_mm_moveldup_ps(_mm_castsi128_ps(a))); }
  template<> __forceinline const int4 shuffle<1, 1, 3, 3>( const int4& a ) { return _mm_castps_si128(_mm_movehdup_ps(_mm_castsi128_ps(a))); }
  template<> __forceinline const int4 shuffle<0, 1, 0, 1>( const int4& a ) { return _mm_castpd_si128(_mm_movedup_pd (_mm_castsi128_pd(a))); }
#endif

  template<size_t i0> __forceinline const int4 shuffle( const int4& b ) {
    return shuffle<i0,i0,i0,i0>(b);
  }

#if defined(__SSE4_1__)
  template<size_t src> __forceinline int extract( const int4& b ) { return _mm_extract_epi32(b, src); }
  template<size_t dst> __forceinline const int4 insert( const int4& a, const int b ) { return _mm_insert_epi32(a, b, dst); }
#else
  template<size_t src> __forceinline int extract( const int4& b ) { return b[src]; }
  template<size_t dst> __forceinline const int4 insert( const int4& a, const int b ) { int4 c = a; c[dst] = b; return c; }
#endif

  ////////////////////////////////////////////////////////////////////////////////
  /// Reductions
  ////////////////////////////////////////////////////////////////////////////////

#if defined(__SSE4_1__)
  __forceinline const int4 vreduce_min(const int4& v) { int4 h = min(shuffle<1,0,3,2>(v),v); return min(shuffle<2,3,0,1>(h),h); }
  __forceinline const int4 vreduce_max(const int4& v) { int4 h = max(shuffle<1,0,3,2>(v),v); return max(shuffle<2,3,0,1>(h),h); }
  __forceinline const int4 vreduce_add(const int4& v) { int4 h = shuffle<1,0,3,2>(v)   + v ; return shuffle<2,3,0,1>(h)   + h ; }

  __forceinline int reduce_min(const int4& v) { return extract<0>(vreduce_min(v)); }
  __forceinline int reduce_max(const int4& v) { return extract<0>(vreduce_max(v)); }
  __forceinline int reduce_add(const int4& v) { return extract<0>(vreduce_add(v)); }

  __forceinline size_t select_min(const int4& v) { return __bsf(movemask(v == vreduce_min(v))); }
  __forceinline size_t select_max(const int4& v) { return __bsf(movemask(v == vreduce_max(v))); }

  __forceinline size_t select_min(const bool4& valid, const int4& v) { const int4 a = select(valid,v,int4(pos_inf)); return __bsf(movemask(valid & (a == vreduce_min(a)))); }
  __forceinline size_t select_max(const bool4& valid, const int4& v) { const int4 a = select(valid,v,int4(neg_inf)); return __bsf(movemask(valid & (a == vreduce_max(a)))); }

#else

  __forceinline int reduce_min(const int4& v) { return min(v[0],v[1],v[2],v[3]); }
  __forceinline int reduce_max(const int4& v) { return max(v[0],v[1],v[2],v[3]); }
  __forceinline int reduce_add(const int4& v) { return v[0]+v[1]+v[2]+v[3]; }

#endif

  ////////////////////////////////////////////////////////////////////////////////
  /// Memory load and store operations
  ////////////////////////////////////////////////////////////////////////////////

  __forceinline int4 load4i( const void* const a ) { 
    return _mm_load_si128((__m128i*)a); 
  }

  __forceinline int4 loadu4i( const void* const a ) { 
    return _mm_loadu_si128((__m128i*)a); 
  }

  __forceinline void store4i(void* ptr, const int4& v) {
    _mm_store_si128((__m128i*)ptr,v);
  }

  __forceinline void storeu4i(void* ptr, const int4& v) {
    _mm_storeu_si128((__m128i*)ptr,v);
  }
  
  __forceinline void store4i( const bool4& mask, void* ptr, const int4& i ) { 
#if defined (__AVX__)
    _mm_maskstore_ps((float*)ptr,(__m128i)mask,_mm_castsi128_ps(i));
#else
    *(int4*)ptr = select(mask,i,*(int4*)ptr);
#endif
  }

  __forceinline int4 load4i_nt (void* ptr) { 
#if defined(__SSE4_1__)
    return _mm_stream_load_si128((__m128i*)ptr); 
#else
    return _mm_load_si128((__m128i*)ptr); 
#endif
  }

  __forceinline void store4i_nt(void* ptr, const int4& v) { 
#if defined(__SSE4_1__)
    _mm_stream_ps((float*)ptr,_mm_castsi128_ps(v)); 
#else
    _mm_store_si128((__m128i*)ptr,v);
#endif
  }

#if 0 // FIXME: not compiling under VS2013
#if defined (__AVX2__)

    __forceinline int4 networkSort(const int4 &v)
    {
      const int4 a0 = v;
      const int4 b0 = shuffle<1,0,3,2>(a0);
      const int4 c0 = umin(a0,b0);
      const int4 d0 = umax(a0,b0);
      const int4 a1 = select(0b0101,c0,d0);
      const int4 b1 = shuffle<2,3,0,1>(a1);
      const int4 c1 = umin(a1,b1);
      //const unsigned int min_dist_index = extract<0>(c1) & 3;
      // 2 case border?
      //assert(min_dist_index < 4);
      const int4 d1 = umax(a1,b1);
      const int4 a2 = select(0b0011,c1,d1);
      const int4 b2 = shuffle<0,2,1,3>(a2);
      const int4 c2 = umin(a2,b2);
      const int4 d2 = umax(a2,b2);
      const int4 a3 = select(0b0010,c2,d2);
      return a3;
    }

#endif
#endif

  ////////////////////////////////////////////////////////////////////////////////
  /// Output Operators
  ////////////////////////////////////////////////////////////////////////////////

  inline std::ostream& operator<<(std::ostream& cout, const int4& a) {
    return cout << "<" << a[0] << ", " << a[1] << ", " << a[2] << ", " << a[3] << ">";
  }
}

