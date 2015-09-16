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
  /*! 16-wide MIC integer type. */
  class int16
  {
  public:
    
    union  { 
      __m512i v; 
      int i[16]; 
    };
    
    ////////////////////////////////////////////////////////////////////////////////
    /// Constructors, Assignment & Cast Operators
    ////////////////////////////////////////////////////////////////////////////////
       
    __forceinline int16() {};
    __forceinline int16(const int16& t) { v = t.v; };
    __forceinline int16& operator=(const int16& f) { v = f.v; return *this; };

    __forceinline int16(const __m512i& t) { v = t; };
    __forceinline operator __m512i () const { return v; };

    __forceinline int16(const int i) { 
      v = _mm512_set_1to16_epi32(i);
    }
    
    __forceinline int16(const int a, const int b, const int c, const int d) { 
      v = _mm512_set_4to16_epi32(a,b,c,d);      
    }
   
    __forceinline explicit int16(const __m512 f) { 
#if defined(__AVX512F__)
      v = _mm512_cvt_roundps_epi32(f,_MM_FROUND_TO_NEAREST_INT | _MM_FROUND_NO_EXC); // FIXME: round down as default?
#else
      v = _mm512_cvtfxpnt_round_adjustps_epi32(f,_MM_FROUND_FLOOR,_MM_EXPADJ_NONE);
#endif
    }
    
    ////////////////////////////////////////////////////////////////////////////////
    /// Constants
    ////////////////////////////////////////////////////////////////////////////////
    
    __forceinline int16( ZeroTy   ) : v(_mm512_setzero_epi32()) {}
    __forceinline int16( OneTy    ) : v(_mm512_set_1to16_epi32(1)) {}
    __forceinline int16( PosInfTy ) : v(_mm512_set_1to16_epi32(pos_inf)) {}
    __forceinline int16( NegInfTy ) : v(_mm512_set_1to16_epi32(neg_inf)) {}
    __forceinline int16( StepTy )   : v(_mm512_set_16to16_epi32(15,14,13,12,11,10,9,8,7,6,5,4,3,2,1,0)) {}

    __forceinline static int16 zero() { return _mm512_setzero_epi32(); }
    __forceinline static int16 one () { return _mm512_set_1to16_epi32(1); }
    __forceinline static int16 neg_one () { return _mm512_set_1to16_epi32(-1); }


    static __forceinline int16 loadu(const void* addr) 
    {
#if defined(__AVX512F__)
      return _mm512_loadu_si512(addr);
#else
      int16 r = _mm512_undefined_epi32();
      r =_mm512_extloadunpacklo_epi32(r, addr, _MM_UPCONV_EPI32_NONE, _MM_HINT_NONE);
      return _mm512_extloadunpackhi_epi32(r, (int*)addr+16, _MM_UPCONV_EPI32_NONE, _MM_HINT_NONE);  
#endif
    }

    static __forceinline int16 load(const void* addr) {
      return _mm512_load_si512(addr);
    }

    static __forceinline void store(void* addr, const int16& v2) {
      _mm512_extstore_epi32(addr,v2,_MM_DOWNCONV_EPI32_NONE,_MM_HINT_NONE);
    }

    static __forceinline void storeu(void* ptr, const int16& f ) { 
#if defined(__AVX512F__)
       _mm512_storeu_si512(ptr,f);
#else
       _mm512_extpackstorelo_ps((int*)ptr+0  ,_mm512_castsi512_ps(f), _MM_DOWNCONV_PS_NONE , 0);
       _mm512_extpackstorehi_ps((int*)ptr+16 ,_mm512_castsi512_ps(f), _MM_DOWNCONV_PS_NONE , 0);
#endif
    }

    static __forceinline void storeu(const bool16& mask, int* ptr, const int16& f ) { 
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

    static __forceinline void store(const bool16& mask, void* addr, const int16& v2) {
      _mm512_mask_store_epi32(addr,mask,v2);
    }

    ////////////////////////////////////////////////////////////////////////////////
    /// Array Access
    ////////////////////////////////////////////////////////////////////////////////
    
    __forceinline int& operator[](const size_t index)       { return i[index]; };
    __forceinline const int& operator[](const size_t index) const { return i[index]; };

    __forceinline unsigned int       uint(const size_t index) const      { assert(index < 16); return ((unsigned int*)i)[index]; };
    __forceinline size_t&            uint64_t(const size_t index)  const     { assert(index < 8); return ((size_t*)i)[index]; }; 


  };
  
  ////////////////////////////////////////////////////////////////////////////////
  /// Unary Operators
  ////////////////////////////////////////////////////////////////////////////////

  __forceinline const int16 cast      ( const __m512& a) { return _mm512_castps_si512(a); }
  __forceinline const int16 operator +( const int16& a ) { return a; }
  __forceinline const int16 operator -( const int16& a ) { return _mm512_sub_epi32(_mm512_setzero_epi32(), a); }

  ////////////////////////////////////////////////////////////////////////////////
  /// Binary Operators
  ////////////////////////////////////////////////////////////////////////////////

  __forceinline const int16 operator +( const int16& a, const int16& b ) { return _mm512_add_epi32(a, b); }
  __forceinline const int16 operator +( const int16& a, const int    b ) { return a + int16(b); }
  __forceinline const int16 operator +( const int    a, const int16& b ) { return int16(a) + b; }

  __forceinline const int16 operator -( const int16& a, const int16& b ) { return _mm512_sub_epi32(a, b); }
  __forceinline const int16 operator -( const int16& a, const int    b ) { return a - int16(b); }
  __forceinline const int16 operator -( const int    a, const int16& b ) { return int16(a) - b; }

  __forceinline const int16 operator *( const int16& a, const int16& b ) { return _mm512_mullo_epi32(a, b); }
  __forceinline const int16 operator *( const int16& a, const int    b ) { return a * int16(b); }
  __forceinline const int16 operator *( const int    a, const int16& b ) { return int16(a) * b; }

  __forceinline const int16 operator &( const int16& a, const int16& b ) { return _mm512_and_epi32(a, b); }
  __forceinline const int16 operator &( const int16& a, const int    b ) { return a & int16(b); }
  __forceinline const int16 operator &( const int    a, const int16& b ) { return int16(a) & b; }

  __forceinline const int16 operator |( const int16& a, const int16& b ) { return _mm512_or_epi32(a, b); }
  __forceinline const int16 operator |( const int16& a, const int    b ) { return a | int16(b); }
  __forceinline const int16 operator |( const int    a, const int16& b ) { return int16(a) | b; }

  __forceinline const int16 operator ^( const int16& a, const int16& b ) { return _mm512_xor_epi32(a, b); }
  __forceinline const int16 operator ^( const int16& a, const int    b ) { return a ^ int16(b); }
  __forceinline const int16 operator ^( const int   a, const int16& b ) { return int16(a) ^ b; }

  __forceinline const int16 operator <<( const int16& a, const int n ) { return _mm512_slli_epi32(a, n); }
  __forceinline const int16 operator >>( const int16& a, const int n ) { return _mm512_srai_epi32(a, n); }

  __forceinline const int16 operator <<( const int16& a, const int16& n ) { return _mm512_sllv_epi32(a, n); }
  __forceinline const int16 operator >>( const int16& a, const int16& n ) { return _mm512_srav_epi32(a, n); }

  __forceinline const int16 sra ( const int16& a, const int b ) { return _mm512_srai_epi32(a, b); }
  __forceinline const int16 srl ( const int16& a, const int b ) { return _mm512_srli_epi32(a, b); }
  
  __forceinline const int16 min( const int16& a, const int16& b ) { return _mm512_min_epi32(a, b); }
  __forceinline const int16 min( const int16& a, const int    b ) { return min(a,int16(b)); }
  __forceinline const int16 min( const int    a, const int16& b ) { return min(int16(a),b); }

  __forceinline const int16 max( const int16& a, const int16& b ) { return _mm512_max_epi32(a, b); }
  __forceinline const int16 max( const int16& a, const int    b ) { return max(a,int16(b)); }
  __forceinline const int16 max( const int    a, const int16& b ) { return max(int16(a),b); }
  
  __forceinline const int16 mask_add(const bool16& mask, int16& c, const int16& a, const int16& b) { return _mm512_mask_add_epi32(c,mask,a,b); }; 
  __forceinline const int16 mask_sub(const bool16& mask, int16& c, const int16& a, const int16& b) { return _mm512_mask_sub_epi32(c,mask,a,b); }; 

  __forceinline const int16 mask_and(const bool16& m,int16& c, const int16& a, const int16& b) { return _mm512_mask_and_epi32(c,m,a,b); };
  __forceinline const int16 mask_or (const bool16& m,int16& c, const int16& a, const int16& b) { return _mm512_mask_or_epi32(c,m,a,b); };
 
  ////////////////////////////////////////////////////////////////////////////////
  /// Assignment Operators
  ////////////////////////////////////////////////////////////////////////////////

  __forceinline int16& operator +=( int16& a, const int16& b ) { return a = a + b; }
  __forceinline int16& operator +=( int16& a, const int    b ) { return a = a + b; }
  
  __forceinline int16& operator -=( int16& a, const int16& b ) { return a = a - b; }
  __forceinline int16& operator -=( int16& a, const int    b ) { return a = a - b; }

  __forceinline int16& operator *=( int16& a, const int16& b ) { return a = a * b; }
  __forceinline int16& operator *=( int16& a, const int    b ) { return a = a * b; }
  
  __forceinline int16& operator &=( int16& a, const int16& b ) { return a = a & b; }
  __forceinline int16& operator &=( int16& a, const int    b ) { return a = a & b; }
  
  __forceinline int16& operator |=( int16& a, const int16& b ) { return a = a | b; }
  __forceinline int16& operator |=( int16& a, const int    b ) { return a = a | b; }
  
  __forceinline int16& operator <<=( int16& a, const int b ) { return a = a << b; }
  __forceinline int16& operator >>=( int16& a, const int b ) { return a = a >> b; }


  ////////////////////////////////////////////////////////////////////////////////
  /// Comparison Operators + Select
  ////////////////////////////////////////////////////////////////////////////////

  __forceinline const bool16 operator ==( const int16& a, const int16& b ) { return _mm512_cmp_epi32_mask(a,b,_MM_CMPINT_EQ); }
  __forceinline const bool16 operator ==( const int16& a, const int    b ) { return a == int16(b); }
  __forceinline const bool16 operator ==( const int    a, const int16& b ) { return int16(a) == b; }
  
  __forceinline const bool16 operator !=( const int16& a, const int16& b ) { return _mm512_cmp_epi32_mask(a,b,_MM_CMPINT_NE); }
  __forceinline const bool16 operator !=( const int16& a, const int    b ) { return a != int16(b); }
  __forceinline const bool16 operator !=( const int    a, const int16& b ) { return int16(a) != b; }
  
  __forceinline const bool16 operator < ( const int16& a, const int16& b ) { return _mm512_cmp_epi32_mask(a,b,_MM_CMPINT_LT); }
  __forceinline const bool16 operator < ( const int16& a, const int    b ) { return a <  int16(b); }
  __forceinline const bool16 operator < ( const int    a, const int16& b ) { return int16(a) <  b; }
  
  __forceinline const bool16 operator >=( const int16& a, const int16& b ) { return _mm512_cmp_epi32_mask(a,b,_MM_CMPINT_GE); }
  __forceinline const bool16 operator >=( const int16& a, const int    b ) { return a >= int16(b); }
  __forceinline const bool16 operator >=( const int    a, const int16& b ) { return int16(a) >= b; }

  __forceinline const bool16 operator > ( const int16& a, const int16& b ) { return _mm512_cmp_epi32_mask(a,b,_MM_CMPINT_GT); }
  __forceinline const bool16 operator > ( const int16& a, const int    b ) { return a >  int16(b); }
  __forceinline const bool16 operator > ( const int    a, const int16& b ) { return int16(a) >  b; }

  __forceinline const bool16 operator <=( const int16& a, const int16& b ) { return _mm512_cmp_epi32_mask(a,b,_MM_CMPINT_LE); }
  __forceinline const bool16 operator <=( const int16& a, const int    b ) { return a <= int16(b); }
  __forceinline const bool16 operator <=( const int    a, const int16& b ) { return int16(a) <= b; }

  __forceinline bool16 eq(                   const int16& a, const int16& b) { return _mm512_cmp_epi32_mask(a,b,_MM_CMPINT_EQ); };
  __forceinline bool16 eq(const bool16 mask, const int16& a, const int16& b) { return _mm512_mask_cmp_epi32_mask(mask,a,b,_MM_CMPINT_EQ);  };
  
  __forceinline bool16 ne(                   const int16& a, const int16& b) { return _mm512_cmp_epi32_mask(a,b,_MM_CMPINT_NE); };
  __forceinline bool16 ne(const bool16 mask, const int16& a, const int16& b) { return _mm512_mask_cmp_epi32_mask(mask,a,b,_MM_CMPINT_NE); };

  __forceinline bool16 lt(                   const int16& a, const int16& b) { return _mm512_cmp_epi32_mask(a,b,_MM_CMPINT_LT); };
  __forceinline bool16 lt(const bool16 mask, const int16& a, const int16& b) { return _mm512_mask_cmp_epi32_mask(mask,a,b,_MM_CMPINT_LT); };
 
  __forceinline bool16 ge(                   const int16& a, const int16& b) { return _mm512_cmp_epi32_mask(a,b,_MM_CMPINT_GE); };
  __forceinline bool16 ge(const bool16 mask, const int16& a, const int16& b) { return _mm512_mask_cmp_epi32_mask(mask,a,b,_MM_CMPINT_GE); };
  
  __forceinline bool16 gt(                   const int16& a, const int16& b) { return _mm512_cmp_epi32_mask(a,b,_MM_CMPINT_GT); };
  __forceinline bool16 gt(const bool16 mask, const int16& a, const int16& b) { return _mm512_mask_cmp_epi32_mask(mask,a,b,_MM_CMPINT_GT); };
  
  __forceinline bool16 le(                   const int16& a, const int16& b) { return _mm512_cmp_epi32_mask(a,b,_MM_CMPINT_LE); };
  __forceinline bool16 le(const bool16 mask, const int16& a, const int16& b) { return _mm512_mask_cmp_epi32_mask(mask,a,b,_MM_CMPINT_LE); };
    
 
  __forceinline const int16 select( const bool16& m, const int16& t, const int16& f ) { 
    return _mm512_mask_or_epi32(f,m,t,t); 
  }

  __forceinline void xchg(const bool16& m, int16& a, int16& b) { 
    const int16 c = a; a = select(m,b,a); b = select(m,c,b);  
  }

  __forceinline bool16 test(const bool16& m, const int16& a, const int16& b) { 
    return _mm512_mask_test_epi32_mask(m,a,b);
  }

  __forceinline bool16 test(const int16& a, const int16& b) { 
    return _mm512_test_epi32_mask(a,b);
  }

  ////////////////////////////////////////////////////////////////////////////////
  // Movement/Shifting/Shuffling Functions
  ////////////////////////////////////////////////////////////////////////////////

  __forceinline int16 swizzle(const int16& x,_MM_SWIZZLE_ENUM perm32 ) { return _mm512_swizzle_epi32(x,perm32); }
  __forceinline int16 permute(const int16& x,_MM_PERM_ENUM    perm128) { return _mm512_permute4f128_epi32(x,perm128); }
  
  template<int D, int C, int B, int A> __forceinline int16 swizzle   (const int16& v) { return _mm512_shuffle_epi32(v,_MM_SHUF_PERM(D,C,B,A)); }
  template<int A>                      __forceinline int16 swizzle   (const int16& x) { return swizzle<A,A,A,A>(v); }
  template<>                           __forceinline int16 swizzle<0>(const int16& x) { return swizzle(x,_MM_SWIZ_REG_AAAA); }
  template<>                           __forceinline int16 swizzle<1>(const int16& x) { return swizzle(x,_MM_SWIZ_REG_BBBB); }
  template<>                           __forceinline int16 swizzle<2>(const int16& x) { return swizzle(x,_MM_SWIZ_REG_CCCC); }
  template<>                           __forceinline int16 swizzle<3>(const int16& x) { return swizzle(x,_MM_SWIZ_REG_DDDD); }

  template<int D, int C, int B, int A> __forceinline int16 permute(const int16& v) { return permute(v,_MM_SHUF_PERM(D,C,B,A)); }
  template<int A>                      __forceinline int16 permute(const int16& x) { return permute<A,A,A,A>(x); }

  __forceinline int16 shuffle(const int16& x,_MM_PERM_ENUM    perm128, _MM_SWIZZLE_ENUM perm32) { return swizzle(permute(x,perm128),perm32); }
  
  __forceinline int16 shuffle(const bool16& mask, int16& v, const int16& x,_MM_PERM_ENUM perm128, _MM_SWIZZLE_ENUM perm32)  {
    return _mm512_mask_swizzle_epi32(_mm512_mask_permute4f128_epi32(v,mask,x,perm128),mask,x,perm32);  
  }

  __forceinline int16 swAAAA(const int16& x) {
    return swizzle(x,_MM_SWIZ_REG_AAAA);
  }

  __forceinline int16 swBBBB(const int16& x) {
    return swizzle(x,_MM_SWIZ_REG_BBBB);
  }

  __forceinline int16 swCCCC(const int16& x) {
    return swizzle(x,_MM_SWIZ_REG_CCCC);
  }

  __forceinline int16 swDDDD(const int16& x) {
    return swizzle(x,_MM_SWIZ_REG_DDDD);
  }

  template<int i>
  __forceinline int16 align_shift_right(const int16& a, const int16& b)
  {
    return _mm512_alignr_epi32(a,b,i); 
  };

  
  ////////////////////////////////////////////////////////////////////////////////
  /// Reductions
  ////////////////////////////////////////////////////////////////////////////////

  __forceinline int reduce_add(int16 a) { return _mm512_reduce_add_epi32(a); }
  __forceinline int reduce_mul(int16 a) { return _mm512_reduce_mul_epi32(a); }
  __forceinline int reduce_min(int16 a) { return _mm512_reduce_min_epi32(a); }
  __forceinline int reduce_max(int16 a) { return _mm512_reduce_max_epi32(a); }
  __forceinline int reduce_and(int16 a) { return _mm512_reduce_and_epi32(a); }
  
  __forceinline int16 vreduce_min2(int16 x) {                      return min(x,swizzle(x,_MM_SWIZ_REG_BADC)); }
  __forceinline int16 vreduce_min4(int16 x) { x = vreduce_min2(x); return min(x,swizzle(x,_MM_SWIZ_REG_CDAB)); }
  __forceinline int16 vreduce_min8(int16 x) { x = vreduce_min4(x); return min(x,permute(x,_MM_SHUF_PERM(2,3,0,1))); }
  __forceinline int16 vreduce_min (int16 x) { x = vreduce_min8(x); return min(x,permute(x,_MM_SHUF_PERM(1,0,3,2))); }

  __forceinline int16 vreduce_max2(int16 x) {                      return max(x,swizzle(x,_MM_SWIZ_REG_BADC)); }
  __forceinline int16 vreduce_max4(int16 x) { x = vreduce_max2(x); return max(x,swizzle(x,_MM_SWIZ_REG_CDAB)); }
  __forceinline int16 vreduce_max8(int16 x) { x = vreduce_max4(x); return max(x,permute(x,_MM_SHUF_PERM(2,3,0,1))); }
  __forceinline int16 vreduce_max (int16 x) { x = vreduce_max8(x); return max(x,permute(x,_MM_SHUF_PERM(1,0,3,2))); }

  __forceinline int16 vreduce_and2(int16 x) {                      return x & swizzle(x,_MM_SWIZ_REG_BADC); }
  __forceinline int16 vreduce_and4(int16 x) { x = vreduce_and2(x); return x & swizzle(x,_MM_SWIZ_REG_CDAB); }
  __forceinline int16 vreduce_and8(int16 x) { x = vreduce_and4(x); return x & permute(x,_MM_SHUF_PERM(2,3,0,1)); }
  __forceinline int16 vreduce_and (int16 x) { x = vreduce_and8(x); return x & permute(x,_MM_SHUF_PERM(1,0,3,2)); }

  __forceinline int16 vreduce_or2(int16 x) {                     return x | swizzle(x,_MM_SWIZ_REG_BADC); }
  __forceinline int16 vreduce_or4(int16 x) { x = vreduce_or2(x); return x | swizzle(x,_MM_SWIZ_REG_CDAB); }
  __forceinline int16 vreduce_or8(int16 x) { x = vreduce_or4(x); return x | permute(x,_MM_SHUF_PERM(2,3,0,1)); }
  __forceinline int16 vreduce_or (int16 x) { x = vreduce_or8(x); return x | permute(x,_MM_SHUF_PERM(1,0,3,2)); }

  __forceinline int16 vreduce_add2(int16 x) {                      return x + swizzle(x,_MM_SWIZ_REG_BADC); }
  __forceinline int16 vreduce_add4(int16 x) { x = vreduce_add2(x); return x + swizzle(x,_MM_SWIZ_REG_CDAB); }
  __forceinline int16 vreduce_add8(int16 x) { x = vreduce_add4(x); return x + permute(x,_MM_SHUF_PERM(2,3,0,1)); }
  __forceinline int16 vreduce_add (int16 x) { x = vreduce_add8(x); return x + permute(x,_MM_SHUF_PERM(1,0,3,2)); }
  
  __forceinline int16 prefix_sum(const int16& a)
  {
    int16 v = a;
    v = mask_add(0xaaaa,v,v,swizzle(v,_MM_SWIZ_REG_CDAB));
    v = mask_add(0xcccc,v,v,swizzle(v,_MM_SWIZ_REG_BBBB));
    const int16 shuf_v0 = shuffle(v,(_MM_PERM_ENUM)_MM_SHUF_PERM(2,2,0,0),_MM_SWIZ_REG_DDDD);
    v = mask_add(0xf0f0,v,v,shuf_v0);
    const int16 shuf_v1 = shuffle(v,(_MM_PERM_ENUM)_MM_SHUF_PERM(1,1,0,0),_MM_SWIZ_REG_DDDD);
    v = mask_add(0xff00,v,v,shuf_v1);
    return v;  
  }

  ////////////////////////////////////////////////////////////////////////////////
  /// Memory load and store operations
  ////////////////////////////////////////////////////////////////////////////////

  __forceinline int16 load1i(const int *const ptr) { 
    return _mm512_extload_epi32(ptr,_MM_UPCONV_EPI32_NONE,_MM_BROADCAST_1X16,_MM_HINT_NONE);
  }

  __forceinline int16 load16i(const int *const ptr) {
    return _mm512_extload_epi32(ptr,_MM_UPCONV_EPI32_NONE,_MM_BROADCAST_16X16,_MM_HINT_NONE);
  }
  
  __forceinline int16 load1i_uint8(const unsigned char *const ptr) { 
    return _mm512_extload_epi32(ptr,_MM_UPCONV_EPI32_UINT8,_MM_BROADCAST_1X16,_MM_HINT_NONE);
  }

  __forceinline int16 load16i_uint8(const unsigned char *const ptr) {
    return _mm512_extload_epi32(ptr,_MM_UPCONV_EPI32_UINT8,_MM_BROADCAST32_NONE,_MM_HINT_NONE);
  }
  
  __forceinline int16 broadcast4to16i(const int *const ptr) {
    return _mm512_extload_epi32(ptr,_MM_UPCONV_EPI32_NONE,_MM_BROADCAST_4X16,_MM_HINT_NONE);
  }

  __forceinline int16 broadcast1to16i(const int *const ptr) {
    return _mm512_extload_epi32(ptr,_MM_UPCONV_EPI32_NONE,_MM_BROADCAST_1X16,_MM_HINT_NONE);
  }  
  
  __forceinline int16 uload16i(const int *const addr) {
    int16 r = _mm512_undefined_epi32();
    r =_mm512_extloadunpacklo_epi32(r, addr, _MM_UPCONV_EPI32_NONE, _MM_HINT_NONE);
    return _mm512_extloadunpackhi_epi32(r, addr+16, _MM_UPCONV_EPI32_NONE, _MM_HINT_NONE);  
  }
  
  __forceinline int16 uload16i_low(const bool16& mask, const void* addr) {
    int16 v = _mm512_undefined_epi32();
    return _mm512_mask_extloadunpacklo_epi32(v, mask, addr, _MM_UPCONV_EPI32_NONE, _MM_HINT_NONE);
  }

  __forceinline int16 uload16i(const bool16& mask,const int *const addr) {
    int16 r = _mm512_undefined_epi32();
    r =_mm512_mask_extloadunpacklo_epi32(r, mask,addr, _MM_UPCONV_EPI32_NONE, _MM_HINT_NONE);
    return _mm512_mask_extloadunpackhi_epi32(r, mask,addr+16, _MM_UPCONV_EPI32_NONE, _MM_HINT_NONE);  
  }
  

  __forceinline int16 gather16i_4i(const int *__restrict__ const ptr0,
                                   const int *__restrict__ const ptr1,
                                   const int *__restrict__ const ptr2,
                                   const int *__restrict__ const ptr3) 
  {
    int16 v =  broadcast4to16i(ptr0);
    v = select((bool16)0xf0  , broadcast4to16i(ptr1),v);
    v = select((bool16)0xf00 , broadcast4to16i(ptr2),v);
    v = select((bool16)0xf000, broadcast4to16i(ptr3),v);
    return v;
  }


  __forceinline int16 gather16i_4i_align(const void *__restrict__ const ptr0,
					 const void *__restrict__ const ptr1,
					 const void *__restrict__ const ptr2,
					 const void *__restrict__ const ptr3) 
  {
    int16 v = broadcast4to16i((const int*)ptr3);
    v = align_shift_right<12>(v,broadcast4to16i((const int*)ptr2));
    v = align_shift_right<12>(v,broadcast4to16i((const int*)ptr1));
    v = align_shift_right<12>(v,broadcast4to16i((const int*)ptr0));
    return v;
  }

  __forceinline int16 gather16i_4i_align(const int16& v0,
					 const int16& v1,
					 const int16& v2,
					 const int16& v3)
  {
    int16 v = v3;
    v = align_shift_right<12>(v,v2);
    v = align_shift_right<12>(v,v1);
    v = align_shift_right<12>(v,v0);
    return v;
  }

  
  __forceinline int16 gather16i(const bool16& mask, const int *const ptr, const int16& index,const _MM_INDEX_SCALE_ENUM scale) {
    return _mm512_mask_i32extgather_epi32(_mm512_undefined_epi32(),mask,index,ptr,_MM_UPCONV_EPI32_NONE,scale,0);
  }
  
  __forceinline int16 gather16i(const bool16& mask, int16& dest, const int *const ptr, const int16& index,const _MM_INDEX_SCALE_ENUM scale) {
    return _mm512_mask_i32extgather_epi32(dest,mask,index,ptr,_MM_UPCONV_EPI32_NONE,scale,0);
  }
  
  __forceinline void scatter16i(const bool16& mask,int *const ptr, const int16& index,const int16& v, const _MM_INDEX_SCALE_ENUM scale) {
    _mm512_mask_i32extscatter_epi32((int*)ptr,mask,index,v,_MM_DOWNCONV_EPI32_NONE,scale,0);
  }
  
  __forceinline void ustore16i(void *addr, const int16& reg) {
    _mm512_extpackstorelo_epi32((int*)addr+0  ,reg, _MM_DOWNCONV_EPI32_NONE, _MM_HINT_NONE);
    _mm512_extpackstorehi_epi32((int*)addr+16 ,reg, _MM_DOWNCONV_EPI32_NONE, _MM_HINT_NONE);
  }
  
  /* pass by value to avoid compiler generating inefficient code */
  __forceinline void compactustore16i(const bool16 mask,void * addr, const int16 reg) {
    _mm512_mask_extpackstorelo_epi32((int*)addr+0  ,mask, reg, _MM_DOWNCONV_EPI32_NONE, _MM_HINT_NONE);
    _mm512_mask_extpackstorehi_epi32((int*)addr+16 ,mask, reg, _MM_DOWNCONV_EPI32_NONE, _MM_HINT_NONE);
  }


  __forceinline void compactustore16i_low(const bool16 mask, void *addr, const int16& reg) {
    _mm512_mask_extpackstorelo_epi32((int*)addr+0  ,mask, reg, _MM_DOWNCONV_EPI32_NONE, _MM_HINT_NONE);
  }

  __forceinline void store1i(void *addr, const int16& reg) {
    _mm512_mask_extpackstorelo_epi32((int*)addr+0  ,(bool16)1, reg, _MM_DOWNCONV_EPI32_NONE, _MM_HINT_NONE);
  }
  
  __forceinline void ustore16i_low(void *addr, const int16& reg) {
    _mm512_extpackstorelo_epi32((int*)addr+0  ,reg, _MM_DOWNCONV_EPI32_NONE, _MM_HINT_NONE);
  }
  
  __forceinline void compactustore16i_high(const bool16 mask, int *addr, const int16& reg) {
    _mm512_mask_extpackstorehi_epi32((int*)addr+16  ,mask, reg, _MM_DOWNCONV_EPI32_NONE, _MM_HINT_NONE);
  }
  
  __forceinline void store16i_nr(void *__restrict__ ptr, const int16& a) {
    _mm512_storenr_ps(ptr,_mm512_castsi512_ps(a));
  }
  
  __forceinline void store16i_ngo(void *__restrict__ ptr, const int16& a) {
    _mm512_storenrngo_ps(ptr,_mm512_castsi512_ps(a));
  }
  
  __forceinline void store16i(const bool16& mask, void* __restrict__ addr, const int16& v2) {
    _mm512_mask_extstore_epi32(addr,mask,v2,_MM_DOWNCONV_EPI32_NONE,_MM_HINT_NONE);
  }


  __forceinline void store16i_nt(void* __restrict__ addr, const int16& v2) {
    _mm512_extstore_epi32(addr,v2,_MM_DOWNCONV_EPI32_NONE,_MM_HINT_NT);
  }
  
  __forceinline void store16i(void* __restrict__ addr, const int16& v2) {
    _mm512_extstore_epi32(addr,v2,_MM_DOWNCONV_EPI32_NONE,_MM_HINT_NONE);
  }
  
  __forceinline void store16i_uint8(const bool16& mask, void* __restrict__ addr, const int16& v2) {
    _mm512_mask_extstore_epi32(addr,mask,v2,_MM_DOWNCONV_EPI32_UINT8,_MM_HINT_NONE);
  }

  __forceinline int16 convert_uint32_t(const __m512 f) { 
#if defined(__AVX512F__)
    return _mm512_cvt_roundps_epu32(f,_MM_FROUND_TO_NEAREST_INT | _MM_FROUND_NO_EXC); // FIXME: round down as default?
#else
    return _mm512_cvtfxpnt_round_adjustps_epu32(f,_MM_FROUND_TO_ZERO,_MM_EXPADJ_NONE);
#endif
  }
  
  ////////////////////////////////////////////////////////////////////////////////
  /// Output Operators
  ////////////////////////////////////////////////////////////////////////////////
  
  __forceinline std::ostream& operator<<(std::ostream& cout, const int16& v)
  {
    cout << "<" << v[0];
    for (int i=1; i<16; i++) cout << ", " << v[i];
    cout << ">";
    return cout;
  }
}
