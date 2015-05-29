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

namespace embree
{
  /*! 8-wide AVX integer type. */
  struct int8
  {
    typedef bool8 Mask;                    // mask type for us
    enum   { size = 8 };                  // number of SIMD elements
    union  {                              // data
      __m256i m256; 
      struct { __m128i l,h; }; 
      int v[8]; 
    }; 

    ////////////////////////////////////////////////////////////////////////////////
    /// Constructors, Assignment & Cast Operators
    ////////////////////////////////////////////////////////////////////////////////

    __forceinline int8           ( ) {}
    __forceinline int8           ( const int8& a ) { m256 = a.m256; }
    __forceinline int8& operator=( const int8& a ) { m256 = a.m256; return *this; }

    __forceinline int8( const __m256i a ) : m256(a) {}
    __forceinline operator const __m256i&( void ) const { return m256; }
    __forceinline operator       __m256i&( void )       { return m256; }

    __forceinline explicit int8( const int4& a ) : m256(_mm256_insertf128_si256(_mm256_castsi128_si256(a),a,1)) {}
    __forceinline int8( const int4& a, const int4& b ) : m256(_mm256_insertf128_si256(_mm256_castsi128_si256(a),b,1)) {}
    __forceinline int8( const __m128i& a, const __m128i& b ) : l(a), h(b) {}
 
    __forceinline explicit int8  ( const int* const a ) : m256(_mm256_castps_si256(_mm256_loadu_ps((const float*)a))) {}
    __forceinline int8           ( int  a ) : m256(_mm256_set1_epi32(a)) {}
    __forceinline int8           ( int  a, int  b) : m256(_mm256_set_epi32(b, a, b, a, b, a, b, a)) {}
    __forceinline int8           ( int  a, int  b, int  c, int  d) : m256(_mm256_set_epi32(d, c, b, a, d, c, b, a)) {}
    __forceinline int8           ( int  a, int  b, int  c, int  d, int  e, int  f, int  g, int  h) : m256(_mm256_set_epi32(h, g, f, e, d, c, b, a)) {}

    __forceinline explicit int8( const __m256 a ) : m256(_mm256_cvtps_epi32(a)) {}

    ////////////////////////////////////////////////////////////////////////////////
    /// Constants
    ////////////////////////////////////////////////////////////////////////////////

    __forceinline int8( ZeroTy   ) : m256(_mm256_setzero_si256()) {}
    __forceinline int8( OneTy    ) : m256(_mm256_set_epi32(1,1,1,1,1,1,1,1)) {}
    __forceinline int8( PosInfTy ) : m256(_mm256_set_epi32(pos_inf,pos_inf,pos_inf,pos_inf,pos_inf,pos_inf,pos_inf,pos_inf)) {}
    __forceinline int8( NegInfTy ) : m256(_mm256_set_epi32(neg_inf,neg_inf,neg_inf,neg_inf,neg_inf,neg_inf,neg_inf,neg_inf)) {}
    __forceinline int8( StepTy   ) : m256(_mm256_set_epi32(7, 6, 5, 4, 3, 2, 1, 0)) {}

    ////////////////////////////////////////////////////////////////////////////////
    /// Loads and Stores
    ////////////////////////////////////////////////////////////////////////////////
    
    static __forceinline int8 load( const void* const a) { 
      return _mm256_castps_si256(_mm256_load_ps((float*)a)); 
    }

    static __forceinline int8 loadu( const void* const a) { 
      return _mm256_castps_si256(_mm256_loadu_ps((float*)a)); 
    }
    
    static __forceinline void store(void* ptr, const int8& f ) { 
      _mm256_store_ps((float*)ptr,_mm256_castsi256_ps(f));
    }
    
    static __forceinline void storeu(void* ptr, const int8& f ) { 
      _mm256_storeu_ps((float*)ptr,_mm256_castsi256_ps(f));
    }
    
    static __forceinline void store( const bool8& mask, void* ptr, const int8& f ) { 
      _mm256_maskstore_ps((float*)ptr,(__m256i)mask,_mm256_castsi256_ps(f));
    }
    
    ////////////////////////////////////////////////////////////////////////////////
    /// Array Access
    ////////////////////////////////////////////////////////////////////////////////

    __forceinline const int& operator []( const size_t i ) const { assert(i < 8); return v[i]; }
    __forceinline       int& operator []( const size_t i )       { assert(i < 8); return v[i]; }
  };

  ////////////////////////////////////////////////////////////////////////////////
  /// Unary Operators
  ////////////////////////////////////////////////////////////////////////////////

  __forceinline const int8 operator +( const int8& a ) { return a; }
  __forceinline const int8 operator -( const int8& a ) { return int8(_mm_sub_epi32(_mm_setzero_si128(), a.l), _mm_sub_epi32(_mm_setzero_si128(), a.h)); }
  __forceinline const int8 abs       ( const int8& a ) { return int8(_mm_abs_epi32(a.l), _mm_abs_epi32(a.h)); }

  ////////////////////////////////////////////////////////////////////////////////
  /// Binary Operators
  ////////////////////////////////////////////////////////////////////////////////

  __forceinline const int8 operator +( const int8& a, const int8& b ) { return int8(_mm_add_epi32(a.l, b.l), _mm_add_epi32(a.h, b.h)); }
  __forceinline const int8 operator +( const int8& a, const int   b ) { return a + int8(b); }
  __forceinline const int8 operator +( const int   a, const int8& b ) { return int8(a) + b; }

  __forceinline const int8 operator -( const int8& a, const int8& b ) { return int8(_mm_sub_epi32(a.l, b.l), _mm_sub_epi32(a.h, b.h)); }
  __forceinline const int8 operator -( const int8& a, const int   b ) { return a - int8(b); }
  __forceinline const int8 operator -( const int   a, const int8& b ) { return int8(a) - b; }

  __forceinline const int8 operator *( const int8& a, const int8& b ) { return int8(_mm_mullo_epi32(a.l, b.l), _mm_mullo_epi32(a.h, b.h)); }
  __forceinline const int8 operator *( const int8& a, const int   b ) { return a * int8(b); }
  __forceinline const int8 operator *( const int   a, const int8& b ) { return int8(a) * b; }

  __forceinline const int8 operator &( const int8& a, const int8& b ) { return _mm256_castps_si256(_mm256_and_ps(_mm256_castsi256_ps(a), _mm256_castsi256_ps(b))); }
  __forceinline const int8 operator &( const int8& a, const int   b ) { return a & int8(b); }
  __forceinline const int8 operator &( const int   a, const int8& b ) { return int8(a) & b; }

  __forceinline const int8 operator |( const int8& a, const int8& b ) { return _mm256_castps_si256(_mm256_or_ps (_mm256_castsi256_ps(a), _mm256_castsi256_ps(b))); }
  __forceinline const int8 operator |( const int8& a, const int   b ) { return a | int8(b); }
  __forceinline const int8 operator |( const int   a, const int8& b ) { return int8(a) | b; }

  __forceinline const int8 operator ^( const int8& a, const int8& b ) { return _mm256_castps_si256(_mm256_xor_ps(_mm256_castsi256_ps(a), _mm256_castsi256_ps(b))); }
  __forceinline const int8 operator ^( const int8& a, const int   b ) { return a ^ int8(b); }
  __forceinline const int8 operator ^( const int   a, const int8& b ) { return int8(a) ^ b; }

  __forceinline const int8 operator <<( const int8& a, const int n ) { return int8(_mm_slli_epi32(a.l, n), _mm_slli_epi32(a.h, n)); }
  __forceinline const int8 operator >>( const int8& a, const int n ) { return int8(_mm_srai_epi32(a.l, n), _mm_srai_epi32(a.h, n)); }

  __forceinline const int8 sra ( const int8& a, const int b ) { return int8(_mm_srai_epi32(a.l, b), _mm_srai_epi32(a.h, b)); }
  __forceinline const int8 srl ( const int8& a, const int b ) { return int8(_mm_srli_epi32(a.l, b), _mm_srli_epi32(a.h, b)); }
  
  __forceinline const int8 min( const int8& a, const int8& b ) { return int8(_mm_min_epi32(a.l, b.l), _mm_min_epi32(a.h, b.h)); }
  __forceinline const int8 min( const int8& a, const int   b ) { return min(a,int8(b)); }
  __forceinline const int8 min( const int   a, const int8& b ) { return min(int8(a),b); }

  __forceinline const int8 max( const int8& a, const int8& b ) { return int8(_mm_max_epi32(a.l, b.l), _mm_max_epi32(a.h, b.h)); }
  __forceinline const int8 max( const int8& a, const int   b ) { return max(a,int8(b)); }
  __forceinline const int8 max( const int   a, const int8& b ) { return max(int8(a),b); }

  ////////////////////////////////////////////////////////////////////////////////
  /// Assignment Operators
  ////////////////////////////////////////////////////////////////////////////////

  __forceinline int8& operator +=( int8& a, const int8& b ) { return a = a + b; }
  __forceinline int8& operator +=( int8& a, const int   b ) { return a = a + b; }
  
  __forceinline int8& operator -=( int8& a, const int8& b ) { return a = a - b; }
  __forceinline int8& operator -=( int8& a, const int   b ) { return a = a - b; }
  
  __forceinline int8& operator *=( int8& a, const int8& b ) { return a = a * b; }
  __forceinline int8& operator *=( int8& a, const int   b ) { return a = a * b; }
  
  __forceinline int8& operator &=( int8& a, const int8& b ) { return a = a & b; }
  __forceinline int8& operator &=( int8& a, const int   b ) { return a = a & b; }
  
  __forceinline int8& operator |=( int8& a, const int8& b ) { return a = a | b; }
  __forceinline int8& operator |=( int8& a, const int   b ) { return a = a | b; }
  
  __forceinline int8& operator <<=( int8& a, const int  b ) { return a = a << b; }
  __forceinline int8& operator >>=( int8& a, const int  b ) { return a = a >> b; }

  ////////////////////////////////////////////////////////////////////////////////
  /// Comparison Operators + Select
  ////////////////////////////////////////////////////////////////////////////////

  __forceinline const bool8 operator ==( const int8& a, const int8& b ) { return bool8(_mm_castsi128_ps(_mm_cmpeq_epi32 (a.l, b.l)), 
                                                                                     _mm_castsi128_ps(_mm_cmpeq_epi32 (a.h, b.h))); }
  __forceinline const bool8 operator ==( const int8& a, const int   b ) { return a == int8(b); }
  __forceinline const bool8 operator ==( const int   a, const int8& b ) { return int8(a) == b; }
  
  __forceinline const bool8 operator !=( const int8& a, const int8& b ) { return !(a == b); }
  __forceinline const bool8 operator !=( const int8& a, const int   b ) { return a != int8(b); }
  __forceinline const bool8 operator !=( const int   a, const int8& b ) { return int8(a) != b; }
  
  __forceinline const bool8 operator < ( const int8& a, const int8& b ) { return bool8(_mm_castsi128_ps(_mm_cmplt_epi32 (a.l, b.l)), 
                                                                                     _mm_castsi128_ps(_mm_cmplt_epi32 (a.h, b.h))); }
  __forceinline const bool8 operator < ( const int8& a, const int   b ) { return a <  int8(b); }
  __forceinline const bool8 operator < ( const int   a, const int8& b ) { return int8(a) <  b; }
  
  __forceinline const bool8 operator >=( const int8& a, const int8& b ) { return !(a <  b); }
  __forceinline const bool8 operator >=( const int8& a, const int   b ) { return a >= int8(b); }
  __forceinline const bool8 operator >=( const int   a, const int8& b ) { return int8(a) >= b; }

  __forceinline const bool8 operator > ( const int8& a, const int8& b ) { return bool8(_mm_castsi128_ps(_mm_cmpgt_epi32 (a.l, b.l)), 
                                                                                     _mm_castsi128_ps(_mm_cmpgt_epi32 (a.h, b.h))); }
  __forceinline const bool8 operator > ( const int8& a, const int   b ) { return a >  int8(b); }
  __forceinline const bool8 operator > ( const int   a, const int8& b ) { return int8(a) >  b; }

  __forceinline const bool8 operator <=( const int8& a, const int8& b ) { return !(a >  b); }
  __forceinline const bool8 operator <=( const int8& a, const int   b ) { return a <= int8(b); }
  __forceinline const bool8 operator <=( const int   a, const int8& b ) { return int8(a) <= b; }

  __forceinline const int8 select( const bool8& m, const int8& t, const int8& f ) { 
    return _mm256_castps_si256(_mm256_blendv_ps(_mm256_castsi256_ps(f), _mm256_castsi256_ps(t), m)); 
  }

  ////////////////////////////////////////////////////////////////////////////////
  /// Movement/Shifting/Shuffling Functions
  ////////////////////////////////////////////////////////////////////////////////

  __forceinline int8 unpacklo( const int8& a, const int8& b ) { return _mm256_castps_si256(_mm256_unpacklo_ps(_mm256_castsi256_ps(a), _mm256_castsi256_ps(b))); }
  __forceinline int8 unpackhi( const int8& a, const int8& b ) { return _mm256_castps_si256(_mm256_unpackhi_ps(_mm256_castsi256_ps(a), _mm256_castsi256_ps(b))); }

  template<size_t i> __forceinline const int8 shuffle( const int8& a ) {
    return _mm256_castps_si256(_mm256_permute_ps(_mm256_castsi256_ps(a), _MM_SHUFFLE(i, i, i, i)));
  }

  template<size_t i0, size_t i1> __forceinline const int8 shuffle( const int8& a ) {
    return _mm256_permute2f128_si256(a, a, (i1 << 4) | (i0 << 0));
  }

  template<size_t i0, size_t i1> __forceinline const int8 shuffle( const int8& a,  const int8& b) {
    return _mm256_permute2f128_si256(a, b, (i1 << 4) | (i0 << 0));
  }

  template<size_t i0, size_t i1, size_t i2, size_t i3> __forceinline const int8 shuffle( const int8& a ) {
    return _mm256_castps_si256(_mm256_permute_ps(_mm256_castsi256_ps(a), _MM_SHUFFLE(i3, i2, i1, i0)));
  }

  template<size_t i0, size_t i1, size_t i2, size_t i3> __forceinline const int8 shuffle( const int8& a, const int8& b ) {
    return _mm256_castps_si256(_mm256_shuffle_ps(_mm256_castsi256_ps(a), _mm256_castsi256_ps(b), _MM_SHUFFLE(i3, i2, i1, i0)));
  }

  template<> __forceinline const int8 shuffle<0, 0, 2, 2>( const int8& b ) { return _mm256_castps_si256(_mm256_moveldup_ps(_mm256_castsi256_ps(b))); }
  template<> __forceinline const int8 shuffle<1, 1, 3, 3>( const int8& b ) { return _mm256_castps_si256(_mm256_movehdup_ps(_mm256_castsi256_ps(b))); }
  template<> __forceinline const int8 shuffle<0, 1, 0, 1>( const int8& b ) { return _mm256_castps_si256(_mm256_castpd_ps(_mm256_movedup_pd(_mm256_castps_pd(_mm256_castsi256_ps(b))))); }

  __forceinline const int8 broadcast(const int* ptr) { return _mm256_castps_si256(_mm256_broadcast_ss((const float*)ptr)); }
  template<size_t i> __forceinline const int8 insert (const int8& a, const int4& b) { return _mm256_insertf128_si256 (a,b,i); }
  template<size_t i> __forceinline const int4 extract(const int8& a               ) { return _mm256_extractf128_si256(a  ,i); }

  ////////////////////////////////////////////////////////////////////////////////
  /// Reductions
  ////////////////////////////////////////////////////////////////////////////////

  __forceinline const int8 vreduce_min2(const int8& v) { return min(v,shuffle<1,0,3,2>(v)); }
  __forceinline const int8 vreduce_min4(const int8& v) { int8 v1 = vreduce_min2(v); return min(v1,shuffle<2,3,0,1>(v1)); }
  __forceinline const int8 vreduce_min (const int8& v) { int8 v1 = vreduce_min4(v); return min(v1,shuffle<1,0>(v1)); }

  __forceinline const int8 vreduce_max2(const int8& v) { return max(v,shuffle<1,0,3,2>(v)); }
  __forceinline const int8 vreduce_max4(const int8& v) { int8 v1 = vreduce_max2(v); return max(v1,shuffle<2,3,0,1>(v1)); }
  __forceinline const int8 vreduce_max (const int8& v) { int8 v1 = vreduce_max4(v); return max(v1,shuffle<1,0>(v1)); }

  __forceinline const int8 vreduce_add2(const int8& v) { return v + shuffle<1,0,3,2>(v); }
  __forceinline const int8 vreduce_add4(const int8& v) { int8 v1 = vreduce_add2(v); return v1 + shuffle<2,3,0,1>(v1); }
  __forceinline const int8 vreduce_add (const int8& v) { int8 v1 = vreduce_add4(v); return v1 + shuffle<1,0>(v1); }

  __forceinline int reduce_min(const int8& v) { return extract<0>(extract<0>(vreduce_min(v))); }
  __forceinline int reduce_max(const int8& v) { return extract<0>(extract<0>(vreduce_max(v))); }
  __forceinline int reduce_add(const int8& v) { return extract<0>(extract<0>(vreduce_add(v))); }

  __forceinline size_t select_min(const int8& v) { return __bsf(movemask(v == vreduce_min(v))); }
  __forceinline size_t select_max(const int8& v) { return __bsf(movemask(v == vreduce_max(v))); }

  __forceinline size_t select_min(const bool8& valid, const int8& v) { const int8 a = select(valid,v,int8(pos_inf)); return __bsf(movemask(valid & (a == vreduce_min(a)))); }
  __forceinline size_t select_max(const bool8& valid, const int8& v) { const int8 a = select(valid,v,int8(neg_inf)); return __bsf(movemask(valid & (a == vreduce_max(a)))); }

  ////////////////////////////////////////////////////////////////////////////////
  /// New stuff
  ////////////////////////////////////////////////////////////////////////////////

  __forceinline void store8i(void* ptr, const int8& i) {
    _mm256_store_ps((float*)ptr,_mm256_castsi256_ps(i));
  }

  __forceinline void store8i( const bool8 &mask, void *ptr, const int8& i ) { 
    _mm256_maskstore_ps((float*)ptr,(__m256i)mask,_mm256_castsi256_ps(i));
  }

  __forceinline void store8i_nt(void* ptr, const int8& v) {
    store4i_nt((int*)ptr+0,v.l);
    store4i_nt((int*)ptr+4,v.h);
  }

  ////////////////////////////////////////////////////////////////////////////////
  /// Output Operators
  ////////////////////////////////////////////////////////////////////////////////

  inline std::ostream& operator<<(std::ostream& cout, const int8& a) {
    return cout << "<" << a[0] << ", " << a[1] << ", " << a[2] << ", " << a[3] << ", " << a[4] << ", " << a[5] << ", " << a[6] << ", " << a[7] << ">";
  }
}
