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
  /* 16-wide AVX-512 integer type */
  template<>
    struct vint<16>
  {
    typedef vboolf16 Bool;
    typedef vint16   Int;
    typedef vfloat16 Float;

    enum  { size = 16 }; // number of SIMD elements
    union {              // data
      __m512i v; 
      int i[16]; 
    };
    
    ////////////////////////////////////////////////////////////////////////////////
    /// Constructors, Assignment & Cast Operators
    ////////////////////////////////////////////////////////////////////////////////
       
    __forceinline vint() {}
    __forceinline vint(const vint16& t) { v = t.v; }
    __forceinline vint16& operator=(const vint16& f) { v = f.v; return *this; }

    __forceinline vint(const __m512i& t) { v = t; }
    __forceinline operator __m512i () const { return v; }
#if defined(__AVX512F__)
    __forceinline operator __m256i () const { return _mm512_castsi512_si256(v); }
#endif

    __forceinline vint(const int i) {
      v = _mm512_set_1to16_epi32(i);
    }
    
    __forceinline vint(const int a, const int b, const int c, const int d) {
      v = _mm512_set_4to16_epi32(d,c,b,a);      
    }

    __forceinline vint(const int a0 , const int a1 , const int a2 , const int a3, 
                       const int a4 , const int a5 , const int a6 , const int a7, 
                       const int a8 , const int a9 , const int a10, const int a11, 
                       const int a12, const int a13, const int a14, const int a15)
    {
      v = _mm512_set_16to16_epi32(a15,a14,a13,a12,a11,a10,a9,a8,a7,a6,a5,a4,a3,a2,a1,a0);
    }

#if defined(__AVX512F__)
    __forceinline vint(const vint4 i) {
      v = _mm512_broadcast_i32x4(i);
    }

    __forceinline vint(const vint8 i) {
      v = _mm512_castps_si512(_mm512_castpd_ps(_mm512_broadcast_f64x4(_mm256_castsi256_pd(i))));
    }

#endif
   
    __forceinline explicit vint(const __m512 f) {
#if defined(__AVX512F__)
      // round to nearest is standard
      v = _mm512_cvt_roundps_epi32(f,_MM_FROUND_TO_NEAREST_INT | _MM_FROUND_NO_EXC); 
#else
      v = _mm512_cvtfxpnt_round_adjustps_epi32(f,_MM_FROUND_FLOOR,_MM_EXPADJ_NONE);
#endif
    }
    
    ////////////////////////////////////////////////////////////////////////////////
    /// Constants
    ////////////////////////////////////////////////////////////////////////////////
    
    __forceinline vint( ZeroTy   ) : v(_mm512_setzero_epi32()) {}
    __forceinline vint( OneTy    ) : v(_mm512_set_1to16_epi32(1)) {}
    __forceinline vint( PosInfTy ) : v(_mm512_set_1to16_epi32(pos_inf)) {}
    __forceinline vint( NegInfTy ) : v(_mm512_set_1to16_epi32(neg_inf)) {}
    __forceinline vint( StepTy   ) : v(_mm512_set_16to16_epi32(15,14,13,12,11,10,9,8,7,6,5,4,3,2,1,0)) {}

    __forceinline vint( ReverseStepTy )   : v(_mm512_setr_epi32(15,14,13,12,11,10,9,8,7,6,5,4,3,2,1,0)) {}

    __forceinline static vint16 zero() { return _mm512_setzero_epi32(); }
    __forceinline static vint16 one () { return _mm512_set_1to16_epi32(1); }
    __forceinline static vint16 neg_one () { return _mm512_set_1to16_epi32(-1); }

    ////////////////////////////////////////////////////////////////////////////////
    /// Loads and Stores
    ////////////////////////////////////////////////////////////////////////////////

    static __forceinline void store_nt(void *__restrict__ ptr, const vint16& a) {
#if defined(__AVX512F__)
      _mm512_stream_si512(ptr,a);
#else
      _mm512_storenr_ps(ptr,_mm512_castsi512_ps(a));
#endif
    }

/* only available on KNC */
#if defined(__MIC__)  
    static __forceinline void store_ngo(void *__restrict__ ptr, const vint16& a) {
      _mm512_storenrngo_ps(ptr,_mm512_castsi512_ps(a));
    }
#endif

    static __forceinline vint16 loadu(const void* addr)
    {
#if defined(__AVX512F__)
      return _mm512_loadu_si512(addr);
#else
      vint16 r = _mm512_undefined_epi32();
      r =_mm512_extloadunpacklo_epi32(r, addr, _MM_UPCONV_EPI32_NONE, _MM_HINT_NONE);
      return _mm512_extloadunpackhi_epi32(r, (int*)addr+16, _MM_UPCONV_EPI32_NONE, _MM_HINT_NONE);  
#endif
    }

    static __forceinline vint16 load(const vint16* addr) {
      return _mm512_load_si512(addr);
    }

    static __forceinline vint16 load(const int* addr) {
      return _mm512_load_si512(addr);
    }

    static __forceinline void store(void* ptr, const vint16& v) {
      //_mm512_extstore_epi32(addr,v2,_MM_DOWNCONV_EPI32_NONE,_MM_HINT_NONE);
      _mm512_store_si512(ptr,v);
    }

    static __forceinline void storeu(void* ptr, const vint16& v ) {
#if defined(__AVX512F__)
      _mm512_storeu_si512(ptr,v);
#else
      _mm512_extpackstorelo_ps((int*)ptr+0  ,_mm512_castsi512_ps(v), _MM_DOWNCONV_PS_NONE , 0);
      _mm512_extpackstorehi_ps((int*)ptr+16 ,_mm512_castsi512_ps(v), _MM_DOWNCONV_PS_NONE , 0);
#endif
    }

    static __forceinline void storeu(const vboolf16& mask, int* ptr, const vint16& f ) {
#if defined(__AVX512F__)
      _mm512_mask_storeu_epi32(ptr,mask,f);
#else
      __m512 r = _mm512_undefined();
      r = _mm512_extloadunpacklo_ps(r, ptr,    _MM_UPCONV_PS_NONE, _MM_HINT_NONE);
      r = _mm512_extloadunpackhi_ps(r, ptr+16, _MM_UPCONV_PS_NONE, _MM_HINT_NONE);  
      r = _mm512_mask_blend_ps(mask,r,_mm512_castsi512_ps(f));
      _mm512_extpackstorelo_ps(ptr,    r, _MM_DOWNCONV_PS_NONE , 0);
      _mm512_extpackstorehi_ps(ptr+16 ,r, _MM_DOWNCONV_PS_NONE , 0);
#endif
    }

    static __forceinline void store(const vboolf16& mask, void* addr, const vint16& v2) {
      _mm512_mask_store_epi32(addr,mask,v2);
    }

  /* pass by value to avoid compiler generating inefficient code */
    static __forceinline void storeu_compact(const vboolf16 mask,void * addr, const vint16 reg) {
#if defined(__AVX512F__)
      _mm512_mask_compressstoreu_epi32(addr,mask,reg);
#else
      _mm512_mask_extpackstorelo_epi32((int*)addr+0  ,mask, reg, _MM_DOWNCONV_EPI32_NONE, _MM_HINT_NONE);
      _mm512_mask_extpackstorehi_epi32((int*)addr+16 ,mask, reg, _MM_DOWNCONV_EPI32_NONE, _MM_HINT_NONE);
#endif
    }

    static __forceinline void storeu_compact_single(const vboolf16 mask,void * addr, const vint16 reg) {
#if defined(__AVX512F__)
      //_mm512_mask_compressstoreu_epi32(addr,mask,reg);
      *(float*)addr = _mm512_cvtss_f32(_mm512_mask_compress_ps(_mm512_castsi512_ps(reg),mask,_mm512_castsi512_ps(reg)));
#else
      _mm512_mask_extpackstorelo_epi32((int*)addr+0  ,mask, reg, _MM_DOWNCONV_EPI32_NONE, _MM_HINT_NONE);
#endif
    }


#if defined(__AVX512F__)
    static __forceinline vint16 compact64bit(const vboolf16& mask, vint16 &v) {
      return _mm512_mask_compress_epi64(v,mask,v);
    }

    static __forceinline vint16 compact(const vboolf16& mask, vint16 &v) {
      return _mm512_mask_compress_epi32(v,mask,v);
    }

    static __forceinline vint16 compact(const vboolf16& mask, const vint16 &a, vint16 &b) {
      return _mm512_mask_compress_epi32(a,mask,b);
    }

    static __forceinline vint16 broadcast64bit(size_t v) {
      return _mm512_set1_epi64(v);
    }

    static __forceinline size_t extracti64bit(const vint16& v)
    {
      return _mm_cvtsi128_si64(_mm512_castsi512_si128(v));
    }

    static __forceinline vint4 extracti128bit(const vint16& v)
    {
      return _mm512_castsi512_si128(v);
    }

    static __forceinline vint8 extracti256bit(const vint16& v)
    {
      return _mm512_castsi512_si256(v);
    }

#endif


    ////////////////////////////////////////////////////////////////////////////////
    /// Array Access
    ////////////////////////////////////////////////////////////////////////////////
    
    __forceinline       int& operator[](const size_t index)       { assert(index < 16); return i[index]; }
    __forceinline const int& operator[](const size_t index) const { assert(index < 16); return i[index]; }

    __forceinline unsigned int uint    (const size_t index) const { assert(index < 16); return ((unsigned int*)i)[index]; }
    __forceinline size_t&      uint64_t(const size_t index) const { assert(index < 8);  return ((size_t*)i)[index]; }
  };
  
  ////////////////////////////////////////////////////////////////////////////////
  /// Unary Operators
  ////////////////////////////////////////////////////////////////////////////////

  __forceinline const vint16 asInt     ( const __m512& a ) { return _mm512_castps_si512(a); }
  __forceinline const vint16 operator +( const vint16& a ) { return a; }
  __forceinline const vint16 operator -( const vint16& a ) { return _mm512_sub_epi32(_mm512_setzero_epi32(), a); }

  ////////////////////////////////////////////////////////////////////////////////
  /// Binary Operators
  ////////////////////////////////////////////////////////////////////////////////

  __forceinline const vint16 operator +( const vint16& a, const vint16& b ) { return _mm512_add_epi32(a, b); }
  __forceinline const vint16 operator +( const vint16& a, const int     b ) { return a + vint16(b); }
  __forceinline const vint16 operator +( const int     a, const vint16& b ) { return vint16(a) + b; }

  __forceinline const vint16 operator -( const vint16& a, const vint16& b ) { return _mm512_sub_epi32(a, b); }
  __forceinline const vint16 operator -( const vint16& a, const int     b ) { return a - vint16(b); }
  __forceinline const vint16 operator -( const int     a, const vint16& b ) { return vint16(a) - b; }

  __forceinline const vint16 operator *( const vint16& a, const vint16& b ) { return _mm512_mullo_epi32(a, b); }
  __forceinline const vint16 operator *( const vint16& a, const int     b ) { return a * vint16(b); }
  __forceinline const vint16 operator *( const int     a, const vint16& b ) { return vint16(a) * b; }

  __forceinline const vint16 operator &( const vint16& a, const vint16& b ) { return _mm512_and_epi32(a, b); }
  __forceinline const vint16 operator &( const vint16& a, const int     b ) { return a & vint16(b); }
  __forceinline const vint16 operator &( const int     a, const vint16& b ) { return vint16(a) & b; }

  __forceinline const vint16 operator |( const vint16& a, const vint16& b ) { return _mm512_or_epi32(a, b); }
  __forceinline const vint16 operator |( const vint16& a, const int     b ) { return a | vint16(b); }
  __forceinline const vint16 operator |( const int     a, const vint16& b ) { return vint16(a) | b; }

  __forceinline const vint16 operator ^( const vint16& a, const vint16& b ) { return _mm512_xor_epi32(a, b); }
  __forceinline const vint16 operator ^( const vint16& a, const int     b ) { return a ^ vint16(b); }
  __forceinline const vint16 operator ^( const int     a, const vint16& b ) { return vint16(a) ^ b; }

  __forceinline const vint16 operator <<( const vint16& a, const int n ) { return _mm512_slli_epi32(a, n); }
  __forceinline const vint16 operator >>( const vint16& a, const int n ) { return _mm512_srai_epi32(a, n); }

  __forceinline const vint16 operator <<( const vint16& a, const vint16& n ) { return _mm512_sllv_epi32(a, n); }
  __forceinline const vint16 operator >>( const vint16& a, const vint16& n ) { return _mm512_srav_epi32(a, n); }

  __forceinline const vint16 sra ( const vint16& a, const int b ) { return _mm512_srai_epi32(a, b); }
  __forceinline const vint16 srl ( const vint16& a, const int b ) { return _mm512_srli_epi32(a, b); }
  
  __forceinline const vint16 min( const vint16& a, const vint16& b ) { return _mm512_min_epi32(a, b); }
  __forceinline const vint16 min( const vint16& a, const int     b ) { return min(a,vint16(b)); }
  __forceinline const vint16 min( const int     a, const vint16& b ) { return min(vint16(a),b); }

  __forceinline const vint16 max( const vint16& a, const vint16& b ) { return _mm512_max_epi32(a, b); }
  __forceinline const vint16 max( const vint16& a, const int     b ) { return max(a,vint16(b)); }
  __forceinline const vint16 max( const int     a, const vint16& b ) { return max(vint16(a),b); }
  
  __forceinline const vint16 umin( const vint16& a, const vint16& b ) { return _mm512_min_epu32(a.v, b.v); }
  __forceinline const vint16 umax( const vint16& a, const vint16& b ) { return _mm512_max_epu32(a.v, b.v); }

  __forceinline const vint16 mask_add(const vboolf16& mask, vint16& c, const vint16& a, const vint16& b) { return _mm512_mask_add_epi32(c,mask,a,b); }
  __forceinline const vint16 mask_sub(const vboolf16& mask, vint16& c, const vint16& a, const vint16& b) { return _mm512_mask_sub_epi32(c,mask,a,b); }

  __forceinline const vint16 mask_and(const vboolf16& m,vint16& c, const vint16& a, const vint16& b) { return _mm512_mask_and_epi32(c,m,a,b); }
  __forceinline const vint16 mask_or (const vboolf16& m,vint16& c, const vint16& a, const vint16& b) { return _mm512_mask_or_epi32(c,m,a,b); }

  ////////////////////////////////////////////////////////////////////////////////
  /// Assignment Operators
  ////////////////////////////////////////////////////////////////////////////////

  __forceinline vint16& operator +=( vint16& a, const vint16& b ) { return a = a + b; }
  __forceinline vint16& operator +=( vint16& a, const int     b ) { return a = a + b; }
  
  __forceinline vint16& operator -=( vint16& a, const vint16& b ) { return a = a - b; }
  __forceinline vint16& operator -=( vint16& a, const int     b ) { return a = a - b; }

  __forceinline vint16& operator *=( vint16& a, const vint16& b ) { return a = a * b; }
  __forceinline vint16& operator *=( vint16& a, const int     b ) { return a = a * b; }
  
  __forceinline vint16& operator &=( vint16& a, const vint16& b ) { return a = a & b; }
  __forceinline vint16& operator &=( vint16& a, const int     b ) { return a = a & b; }
  
  __forceinline vint16& operator |=( vint16& a, const vint16& b ) { return a = a | b; }
  __forceinline vint16& operator |=( vint16& a, const int     b ) { return a = a | b; }
  
  __forceinline vint16& operator <<=( vint16& a, const int b ) { return a = a << b; }
  __forceinline vint16& operator >>=( vint16& a, const int b ) { return a = a >> b; }


  ////////////////////////////////////////////////////////////////////////////////
  /// Comparison Operators + Select
  ////////////////////////////////////////////////////////////////////////////////

  __forceinline const vboolf16 operator ==( const vint16& a, const vint16& b ) { return _mm512_cmp_epi32_mask(a,b,_MM_CMPINT_EQ); }
  __forceinline const vboolf16 operator ==( const vint16& a, const int     b ) { return a == vint16(b); }
  __forceinline const vboolf16 operator ==( const int     a, const vint16& b ) { return vint16(a) == b; }
  
  __forceinline const vboolf16 operator !=( const vint16& a, const vint16& b ) { return _mm512_cmp_epi32_mask(a,b,_MM_CMPINT_NE); }
  __forceinline const vboolf16 operator !=( const vint16& a, const int     b ) { return a != vint16(b); }
  __forceinline const vboolf16 operator !=( const int     a, const vint16& b ) { return vint16(a) != b; }
  
  __forceinline const vboolf16 operator < ( const vint16& a, const vint16& b ) { return _mm512_cmp_epi32_mask(a,b,_MM_CMPINT_LT); }
  __forceinline const vboolf16 operator < ( const vint16& a, const int     b ) { return a <  vint16(b); }
  __forceinline const vboolf16 operator < ( const int     a, const vint16& b ) { return vint16(a) <  b; }
  
  __forceinline const vboolf16 operator >=( const vint16& a, const vint16& b ) { return _mm512_cmp_epi32_mask(a,b,_MM_CMPINT_GE); }
  __forceinline const vboolf16 operator >=( const vint16& a, const int     b ) { return a >= vint16(b); }
  __forceinline const vboolf16 operator >=( const int     a, const vint16& b ) { return vint16(a) >= b; }

  __forceinline const vboolf16 operator > ( const vint16& a, const vint16& b ) { return _mm512_cmp_epi32_mask(a,b,_MM_CMPINT_GT); }
  __forceinline const vboolf16 operator > ( const vint16& a, const int     b ) { return a >  vint16(b); }
  __forceinline const vboolf16 operator > ( const int     a, const vint16& b ) { return vint16(a) >  b; }

  __forceinline const vboolf16 operator <=( const vint16& a, const vint16& b ) { return _mm512_cmp_epi32_mask(a,b,_MM_CMPINT_LE); }
  __forceinline const vboolf16 operator <=( const vint16& a, const int     b ) { return a <= vint16(b); }
  __forceinline const vboolf16 operator <=( const int     a, const vint16& b ) { return vint16(a) <= b; }

  __forceinline vboolf16 eq(                     const vint16& a, const vint16& b) { return _mm512_cmp_epi32_mask(a,b,_MM_CMPINT_EQ); }
  __forceinline vboolf16 eq(const vboolf16 mask, const vint16& a, const vint16& b) { return _mm512_mask_cmp_epi32_mask(mask,a,b,_MM_CMPINT_EQ); }
  
  __forceinline vboolf16 ne(                     const vint16& a, const vint16& b) { return _mm512_cmp_epi32_mask(a,b,_MM_CMPINT_NE); }
  __forceinline vboolf16 ne(const vboolf16 mask, const vint16& a, const vint16& b) { return _mm512_mask_cmp_epi32_mask(mask,a,b,_MM_CMPINT_NE); }

  __forceinline vboolf16 lt(                     const vint16& a, const vint16& b) { return _mm512_cmp_epi32_mask(a,b,_MM_CMPINT_LT); }
  __forceinline vboolf16 lt(const vboolf16 mask, const vint16& a, const vint16& b) { return _mm512_mask_cmp_epi32_mask(mask,a,b,_MM_CMPINT_LT); }
 
  __forceinline vboolf16 ge(                     const vint16& a, const vint16& b) { return _mm512_cmp_epi32_mask(a,b,_MM_CMPINT_GE); }
  __forceinline vboolf16 ge(const vboolf16 mask, const vint16& a, const vint16& b) { return _mm512_mask_cmp_epi32_mask(mask,a,b,_MM_CMPINT_GE); }
  
  __forceinline vboolf16 gt(                     const vint16& a, const vint16& b) { return _mm512_cmp_epi32_mask(a,b,_MM_CMPINT_GT); }
  __forceinline vboolf16 gt(const vboolf16 mask, const vint16& a, const vint16& b) { return _mm512_mask_cmp_epi32_mask(mask,a,b,_MM_CMPINT_GT); }
  
  __forceinline vboolf16 le(                     const vint16& a, const vint16& b) { return _mm512_cmp_epi32_mask(a,b,_MM_CMPINT_LE); }
  __forceinline vboolf16 le(const vboolf16 mask, const vint16& a, const vint16& b) { return _mm512_mask_cmp_epi32_mask(mask,a,b,_MM_CMPINT_LE); }


  __forceinline vboolf16 uint_le(                     const vint16& a, const vint16& b) { return _mm512_cmp_epu32_mask(a,b,_MM_CMPINT_LE); }
  __forceinline vboolf16 uint_le(const vboolf16 mask, const vint16& a, const vint16& b) { return _mm512_mask_cmp_epu32_mask(mask,a,b,_MM_CMPINT_LE); }

  __forceinline vboolf16 uint_gt(                     const vint16& a, const vint16& b) { return _mm512_cmp_epu32_mask(a,b,_MM_CMPINT_GT); }
  __forceinline vboolf16 uint_gt(const vboolf16 mask, const vint16& a, const vint16& b) { return _mm512_mask_cmp_epu32_mask(mask,a,b,_MM_CMPINT_GT); }
    
 
  __forceinline const vint16 select( const vboolf16& m, const vint16& t, const vint16& f ) {
    return _mm512_mask_or_epi32(f,m,t,t); 
  }

  __forceinline void xchg(const vboolf16& m, vint16& a, vint16& b) {
    const vint16 c = a; a = select(m,b,a); b = select(m,c,b);
  }

  __forceinline vboolf16 test(const vboolf16& m, const vint16& a, const vint16& b) {
    return _mm512_mask_test_epi32_mask(m,a,b);
  }

  __forceinline vboolf16 test(const vint16& a, const vint16& b) {
    return _mm512_test_epi32_mask(a,b);
  }

  ////////////////////////////////////////////////////////////////////////////////
  // Movement/Shifting/Shuffling Functions
  ////////////////////////////////////////////////////////////////////////////////

  __forceinline vint16 shuffle(const vint16& x,_MM_SWIZZLE_ENUM perm32 ) { return _mm512_swizzle_epi32(x,perm32); }
  __forceinline vint16 shuffle4(const vint16& x,_MM_PERM_ENUM    perm128) { return _mm512_permute4f128_epi32(x,perm128); }
  
  template<int D, int C, int B, int A> __forceinline vint16 shuffle   (const vint16& v) { return _mm512_shuffle_epi32(v,_MM_SHUF_PERM(D,C,B,A)); }
  template<int A>                      __forceinline vint16 shuffle   (const vint16& x) { return shuffle<A,A,A,A>(v); }
  template<>                           __forceinline vint16 shuffle<0>(const vint16& x) { return shuffle(x,_MM_SWIZ_REG_AAAA); }
  template<>                           __forceinline vint16 shuffle<1>(const vint16& x) { return shuffle(x,_MM_SWIZ_REG_BBBB); }
  template<>                           __forceinline vint16 shuffle<2>(const vint16& x) { return shuffle(x,_MM_SWIZ_REG_CCCC); }
  template<>                           __forceinline vint16 shuffle<3>(const vint16& x) { return shuffle(x,_MM_SWIZ_REG_DDDD); }

  template<int D, int C, int B, int A> __forceinline vint16 shuffle4(const vint16& v) { return shuffle4(v,_MM_SHUF_PERM(D,C,B,A)); }
  template<int A>                      __forceinline vint16 shuffle4(const vint16& x) { return shuffle4<A,A,A,A>(x); }

  __forceinline vint16 shuffle(const vint16& x,_MM_PERM_ENUM    perm128, _MM_SWIZZLE_ENUM perm32) { return shuffle(shuffle4(x,perm128),perm32); }
  
  __forceinline vint16 shuffle(const vboolf16& mask, vint16& v, const vint16& x,_MM_PERM_ENUM perm128, _MM_SWIZZLE_ENUM perm32)  {
    return _mm512_mask_swizzle_epi32(_mm512_mask_permute4f128_epi32(v,mask,x,perm128),mask,x,perm32);  
  }

  __forceinline vint16 swAAAA(const vint16& x) {
    return shuffle(x,_MM_SWIZ_REG_AAAA);
  }

  __forceinline vint16 swBBBB(const vint16& x) {
    return shuffle(x,_MM_SWIZ_REG_BBBB);
  }

  __forceinline vint16 swCCCC(const vint16& x) {
    return shuffle(x,_MM_SWIZ_REG_CCCC);
  }

  __forceinline vint16 swDDDD(const vint16& x) {
    return shuffle(x,_MM_SWIZ_REG_DDDD);
  }

  template<int i>
    __forceinline vint16 align_shift_right(const vint16& a, const vint16& b)
  {
    return _mm512_alignr_epi32(a,b,i); 
  };

  __forceinline int toScalar(const vint16& a)
  {
#if defined(__AVX512F__)
    return _mm_cvtsi128_si32(_mm512_castsi512_si128(a));
#else
    return a[0];
#endif
  }

#if defined(__AVX512F__)  
  template<size_t i> __forceinline const vint16 insert4(const vint16& a, const vint4& b) { return _mm512_inserti32x4(a, b, i); }
#endif

  ////////////////////////////////////////////////////////////////////////////////
  /// Reductions
  ////////////////////////////////////////////////////////////////////////////////

  __forceinline int reduce_add(vint16 a) { return _mm512_reduce_add_epi32(a); }
  __forceinline int reduce_mul(vint16 a) { return _mm512_reduce_mul_epi32(a); }
  __forceinline int reduce_min(vint16 a) { return _mm512_reduce_min_epi32(a); }
  __forceinline int reduce_max(vint16 a) { return _mm512_reduce_max_epi32(a); }
  __forceinline int reduce_and(vint16 a) { return _mm512_reduce_and_epi32(a); }
  
  __forceinline vint16 vreduce_min2(vint16 x) {                      return min(x,shuffle(x,_MM_SWIZ_REG_BADC)); }
  __forceinline vint16 vreduce_min4(vint16 x) { x = vreduce_min2(x); return min(x,shuffle(x,_MM_SWIZ_REG_CDAB)); }
  __forceinline vint16 vreduce_min8(vint16 x) { x = vreduce_min4(x); return min(x,shuffle4(x,_MM_SHUF_PERM(2,3,0,1))); }
  __forceinline vint16 vreduce_min (vint16 x) { x = vreduce_min8(x); return min(x,shuffle4(x,_MM_SHUF_PERM(1,0,3,2))); }

  __forceinline vint16 vreduce_max2(vint16 x) {                      return max(x,shuffle(x,_MM_SWIZ_REG_BADC)); }
  __forceinline vint16 vreduce_max4(vint16 x) { x = vreduce_max2(x); return max(x,shuffle(x,_MM_SWIZ_REG_CDAB)); }
  __forceinline vint16 vreduce_max8(vint16 x) { x = vreduce_max4(x); return max(x,shuffle4(x,_MM_SHUF_PERM(2,3,0,1))); }
  __forceinline vint16 vreduce_max (vint16 x) { x = vreduce_max8(x); return max(x,shuffle4(x,_MM_SHUF_PERM(1,0,3,2))); }

  __forceinline vint16 vreduce_and2(vint16 x) {                      return x & shuffle(x,_MM_SWIZ_REG_BADC); }
  __forceinline vint16 vreduce_and4(vint16 x) { x = vreduce_and2(x); return x & shuffle(x,_MM_SWIZ_REG_CDAB); }
  __forceinline vint16 vreduce_and8(vint16 x) { x = vreduce_and4(x); return x & shuffle4(x,_MM_SHUF_PERM(2,3,0,1)); }
  __forceinline vint16 vreduce_and (vint16 x) { x = vreduce_and8(x); return x & shuffle4(x,_MM_SHUF_PERM(1,0,3,2)); }

  __forceinline vint16 vreduce_or2(vint16 x) {                     return x | shuffle(x,_MM_SWIZ_REG_BADC); }
  __forceinline vint16 vreduce_or4(vint16 x) { x = vreduce_or2(x); return x | shuffle(x,_MM_SWIZ_REG_CDAB); }
  __forceinline vint16 vreduce_or8(vint16 x) { x = vreduce_or4(x); return x | shuffle4(x,_MM_SHUF_PERM(2,3,0,1)); }
  __forceinline vint16 vreduce_or (vint16 x) { x = vreduce_or8(x); return x | shuffle4(x,_MM_SHUF_PERM(1,0,3,2)); }

  __forceinline vint16 vreduce_add2(vint16 x) {                      return x + shuffle(x,_MM_SWIZ_REG_BADC); }
  __forceinline vint16 vreduce_add4(vint16 x) { x = vreduce_add2(x); return x + shuffle(x,_MM_SWIZ_REG_CDAB); }
  __forceinline vint16 vreduce_add8(vint16 x) { x = vreduce_add4(x); return x + shuffle4(x,_MM_SHUF_PERM(2,3,0,1)); }
  __forceinline vint16 vreduce_add (vint16 x) { x = vreduce_add8(x); return x + shuffle4(x,_MM_SHUF_PERM(1,0,3,2)); }

  ////////////////////////////////////////////////////////////////////////////////
  /// Memory load and store operations
  ////////////////////////////////////////////////////////////////////////////////


/* only available on KNC */
#if defined(__MIC__)  
  __forceinline vint16 load1i_uint8(const unsigned char *const ptr) {
    return _mm512_extload_epi32(ptr,_MM_UPCONV_EPI32_UINT8,_MM_BROADCAST_1X16,_MM_HINT_NONE);
  }

  __forceinline vint16 load16i_uint8(const unsigned char *const ptr) {
    return _mm512_extload_epi32(ptr,_MM_UPCONV_EPI32_UINT8,_MM_BROADCAST32_NONE,_MM_HINT_NONE);
  }  

  __forceinline vint16 broadcast1to16i(const int *const ptr) {
    return _mm512_extload_epi32(ptr,_MM_UPCONV_EPI32_NONE,_MM_BROADCAST_1X16,_MM_HINT_NONE);
  }  
    
  __forceinline vint16 uload16i_low(const vboolf16& mask, const void* addr) {
    vint16 v = _mm512_undefined_epi32();
    return _mm512_mask_extloadunpacklo_epi32(v, mask, addr, _MM_UPCONV_EPI32_NONE, _MM_HINT_NONE);
  }

  __forceinline void store1i(void *addr, const vint16& reg) {
    _mm512_mask_extpackstorelo_epi32((int*)addr+0  ,(vboolf16)1, reg, _MM_DOWNCONV_EPI32_NONE, _MM_HINT_NONE);
  }
  
  __forceinline void ustore16i_low(void *addr, const vint16& reg) {
    _mm512_extpackstorelo_epi32((int*)addr+0  ,reg, _MM_DOWNCONV_EPI32_NONE, _MM_HINT_NONE);
  }
  
  __forceinline void compactustore16i_high(const vboolf16 mask, int *addr, const vint16& reg) {
    _mm512_mask_extpackstorehi_epi32((int*)addr+16  ,mask, reg, _MM_DOWNCONV_EPI32_NONE, _MM_HINT_NONE);
  }

  __forceinline void store16i_uint8(const vboolf16& mask, void* __restrict__ addr, const vint16& v2) {
    _mm512_mask_extstore_epi32(addr,mask,v2,_MM_DOWNCONV_EPI32_UINT8,_MM_HINT_NONE);
  }

#endif
  

  __forceinline vint16 broadcast4to16i(const int *const ptr) {
    return _mm512_extload_epi32(ptr,_MM_UPCONV_EPI32_NONE,_MM_BROADCAST_4X16,_MM_HINT_NONE);
  }

  __forceinline vint16 gather16i_4i(const int *__restrict__ const ptr0,
                                    const int *__restrict__ const ptr1,
                                    const int *__restrict__ const ptr2,
                                    const int *__restrict__ const ptr3)
  {
    vint16 v =  broadcast4to16i(ptr0);
    v = select((vboolf16)0xf0  , broadcast4to16i(ptr1),v);
    v = select((vboolf16)0xf00 , broadcast4to16i(ptr2),v);
    v = select((vboolf16)0xf000, broadcast4to16i(ptr3),v);
    return v;
  }


  __forceinline vint16 gather16i_4i_align(const void *__restrict__ const ptr0,
                                          const void *__restrict__ const ptr1,
                                          const void *__restrict__ const ptr2,
                                          const void *__restrict__ const ptr3)
  {
    vint16 v = broadcast4to16i((const int*)ptr3);
    v = align_shift_right<12>(v,broadcast4to16i((const int*)ptr2));
    v = align_shift_right<12>(v,broadcast4to16i((const int*)ptr1));
    v = align_shift_right<12>(v,broadcast4to16i((const int*)ptr0));
    return v;
  }

  __forceinline vint16 gather16i_4i_align(const vint16& v0,
                                          const vint16& v1,
                                          const vint16& v2,
                                          const vint16& v3)
  {
    vint16 v = v3;
    v = align_shift_right<12>(v,v2);
    v = align_shift_right<12>(v,v1);
    v = align_shift_right<12>(v,v0);
    return v;
  }

  
  __forceinline vint16 gather16i(const vboolf16& mask, const int *const ptr, const vint16& index,const _MM_INDEX_SCALE_ENUM scale) {
    return _mm512_mask_i32extgather_epi32(_mm512_undefined_epi32(),mask,index,ptr,_MM_UPCONV_EPI32_NONE,scale,0);
  }
  
  __forceinline vint16 gather16i(const vboolf16& mask, vint16& dest, const int *const ptr, const vint16& index,const _MM_INDEX_SCALE_ENUM scale) {
    return _mm512_mask_i32extgather_epi32(dest,mask,index,ptr,_MM_UPCONV_EPI32_NONE,scale,0);
  }
  
  __forceinline void scatter16i(const vboolf16& mask,int *const ptr, const vint16& index,const vint16& v, const _MM_INDEX_SCALE_ENUM scale) {
    _mm512_mask_i32extscatter_epi32((int*)ptr,mask,index,v,_MM_DOWNCONV_EPI32_NONE,scale,0);
  }
    

  __forceinline void compactustore16i_low(const vboolf16 mask, void *addr, const vint16& reg) {
#if defined(__AVX512F__)
    _mm512_mask_compressstoreu_epi32(addr,mask,reg);
#else
    _mm512_mask_extpackstorelo_epi32((int*)addr+0  ,mask, reg, _MM_DOWNCONV_EPI32_NONE, _MM_HINT_NONE);
#endif
  }
  

  __forceinline vint16 convert_uint32_t(const __m512 f) {
#if defined(__AVX512F__)
    return _mm512_cvt_roundps_epu32(f,_MM_FROUND_TO_NEAREST_INT | _MM_FROUND_NO_EXC); 
#else
    return _mm512_cvtfxpnt_round_adjustps_epu32(f,_MM_FROUND_TO_ZERO,_MM_EXPADJ_NONE);
#endif
  }

  __forceinline vint16 permute(vint16 v,vint16 index)
  {
    return _mm512_permutev_epi32(index,v);  
  }

  __forceinline vint16 reverse(const vint16 &a)
  {
    return permute(a,vint16(reverse_step));
  }

  __forceinline vint16 prefix_sum(const vint16& a)
  {
    vint16 v = a;
    v = mask_add(0xaaaa,v,v,shuffle<2,2,0,0>(v));
    v = mask_add(0xcccc,v,v,shuffle<1,1,1,1>(v));
    const vint16 shuf_v0 = shuffle(v,(_MM_PERM_ENUM)_MM_SHUF_PERM(2,2,0,0),_MM_SWIZ_REG_DDDD);
    v = mask_add(0xf0f0,v,v,shuf_v0);
    const vint16 shuf_v1 = shuffle(v,(_MM_PERM_ENUM)_MM_SHUF_PERM(1,1,0,0),_MM_SWIZ_REG_DDDD);
    v = mask_add(0xff00,v,v,shuf_v1);
    return v;  
  }

  __forceinline vint16 reverse_prefix_sum(const vint16& a)
  {
    vint16 v = a;
    v = mask_add(0x5555,v,v,shuffle<3,3,1,1>(v));
    v = mask_add(0x3333,v,v,shuffle<2,2,2,2>(v));
    const vint16 shuf_v0 = shuffle(v,(_MM_PERM_ENUM)_MM_SHUF_PERM(3,3,1,1),_MM_SWIZ_REG_AAAA);
    v = mask_add(0x0f0f,v,v,shuf_v0);
    const vint16 shuf_v1 = shuffle(v,(_MM_PERM_ENUM)_MM_SHUF_PERM(2,2,2,2),_MM_SWIZ_REG_AAAA);
    v = mask_add(0x00ff,v,v,shuf_v1);

    return v;  
  }

#if defined(__AVX512F__)
  /* this should use a vbool8 and a vint8_64...*/
  __forceinline void gather_prefetch64(void const* base_addr,const vbool16 &mask, const vint16& offset, const int scale = 1, const int hint = _MM_HINT_T0)
  {
    _mm512_mask_prefetch_i64gather_pd(offset,mask,base_addr,scale,hint);
  }
#endif


  __forceinline vint16 sortNetwork(const vint16& v)
  {
    const vint16 a0 = v;
    const vint16 b0 = shuffle<1,0,3,2>(a0);
    const vint16 c0 = umin(a0,b0);
    const vint16 d0 = umax(a0,b0);
    const vint16 a1 = select(0x99 /* 0b10011001 */,c0,d0);
    const vint16 b1 = shuffle<2,3,0,1>(a1);
    const vint16 c1 = umin(a1,b1);
    const vint16 d1 = umax(a1,b1);
    const vint16 a2 = select(0xc3 /* 0b11000011 */,c1,d1);
    const vint16 b2 = shuffle<1,0,3,2>(a2);
    const vint16 c2 = umin(a2,b2);
    const vint16 d2 = umax(a2,b2);
    const vint16 a3 = select(0xa5 /* 0b10100101 */,c2,d2);
    const vint16 b3 = shuffle4<1,0,1,0>(a3);
    const vint16 c3 = umin(a3,b3);
    const vint16 d3 = umax(a3,b3);
    const vint16 a4 = select(0xf /* 0b00001111 */,c3,d3);
    const vint16 b4 = shuffle<2,3,0,1>(a4);
    const vint16 c4 = umin(a4,b4);
    const vint16 d4 = umax(a4,b4);
    const vint16 a5 = select(0x33 /* 0b00110011 */,c4,d4);
    const vint16 b5 = shuffle<1,0,3,2>(a5);
    const vint16 c5 = umin(a5,b5);
    const vint16 d5 = umax(a5,b5);
    const vint16 a6 = select(0x55 /* 0b01010101 */,c5,d5);
    return a6;
  }
  
  ////////////////////////////////////////////////////////////////////////////////
  /// Output Operators
  ////////////////////////////////////////////////////////////////////////////////
  
  __forceinline std::ostream& operator<<(std::ostream& cout, const vint16& v)
  {
    cout << "<" << v[0];
    for (int i=1; i<16; i++) cout << ", " << v[i];
    cout << ">";
    return cout;
  }
}
