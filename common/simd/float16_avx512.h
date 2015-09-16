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
  /*! 16-wide MIC float type. */
  class float16 
  {
  public:
    typedef bool16 Mask;    // mask type for us

    enum   { size = 16 };  // number of SIMD elements

    union  { 
      __m512 v; 
      float f[16]; 
    };

    ////////////////////////////////////////////////////////////////////////////////
    /// Constructors, Assignment & Cast Operators
    ////////////////////////////////////////////////////////////////////////////////
        
    __forceinline float16() {}
    __forceinline float16(const float16& t) { v = t; };
    __forceinline float16& operator=(const float16& f) { v = f.v; return *this; };

    __forceinline float16(const __m512& t) { v = t; };
    __forceinline operator __m512 () const { return v; };
    
    __forceinline float16(const float& f) { 
      v = _mm512_set_1to16_ps(f);
    }
    __forceinline float16(const float& a, const float& b, const float& c, const float& d) { 
      v = _mm512_set_4to16_ps(a,b,c,d);  
    }
    
    __forceinline explicit float16(const __m512i& a) { 
#if defined(__AVX512F__)
      v = _mm512_cvt_roundepi32_ps(a,_MM_FROUND_TO_NEAREST_INT | _MM_FROUND_NO_EXC); // FIXME: round down as default?
#else
      v = _mm512_cvtfxpnt_round_adjustepi32_ps(a, _MM_FROUND_NO_EXC,_MM_EXPADJ_NONE);
#endif
    }

    static __forceinline void store(void* addr, const float16& v2) {
      _mm512_extstore_ps(addr,v2,_MM_DOWNCONV_PS_NONE,0);
    }

    static __forceinline void store(const bool16& mask, void* addr, const float16& v2) {
      _mm512_mask_extstore_ps(addr,mask,v2,_MM_DOWNCONV_PS_NONE,0);
    }

    static __forceinline void storeu(float* ptr, const float16& f ) { 
#if defined(__AVX512F__)
      _mm512_storeu_ps(ptr,f);
#else
    _mm512_extpackstorelo_ps(ptr+0  ,f, _MM_DOWNCONV_PS_NONE , 0);
    _mm512_extpackstorehi_ps(ptr+16 ,f, _MM_DOWNCONV_PS_NONE , 0);
#endif
    }

    static __forceinline void storeu(const bool16& mask, float* ptr, const float16& f ) { 
#if defined(__AVX512F__)
      _mm512_mask_storeu_ps(ptr,mask,f);
#else
      float16 r = float16::undefined();
      r = _mm512_extloadunpacklo_ps(r, ptr,    _MM_UPCONV_PS_NONE, _MM_HINT_NONE);
      r = _mm512_extloadunpackhi_ps(r, ptr+16, _MM_UPCONV_PS_NONE, _MM_HINT_NONE);  
      r = _mm512_mask_blend_ps(mask,r,f);
      _mm512_extpackstorelo_ps(ptr,    r, _MM_DOWNCONV_PS_NONE , 0);
      _mm512_extpackstorehi_ps(ptr+16 ,r, _MM_DOWNCONV_PS_NONE , 0);
#endif
    }

    ////////////////////////////////////////////////////////////////////////////////
    /// Constants
    ////////////////////////////////////////////////////////////////////////////////
    
    __forceinline float16( ZeroTy   ) : v(_mm512_setzero_ps()) {}
    __forceinline float16( OneTy    ) : v(_mm512_set_1to16_ps(1.0f)) {}
    __forceinline float16( PosInfTy ) : v(_mm512_set_1to16_ps(pos_inf)) {}
    __forceinline float16( NegInfTy ) : v(_mm512_set_1to16_ps(neg_inf)) {}
    __forceinline float16( StepTy )   : v(_mm512_set_16to16_ps(15,14,13,12,11,10,9,8,7,6,5,4,3,2,1,0)) {}
    __forceinline float16( NaNTy    ) : v(_mm512_set_1to16_ps(nan)) {}

    __forceinline static float16 undefined() { return _mm512_undefined(); }
    __forceinline static float16 zero() { return _mm512_setzero_ps(); }
    __forceinline static float16 one () { return _mm512_set_1to16_ps(1.0f); }
    __forceinline static float16 ulp () { return _mm512_set_1to16_ps(embree::ulp); }
    __forceinline static float16 inf () { return _mm512_set_1to16_ps((float)pos_inf); }
    __forceinline static float16 minus_inf () { return _mm512_set_1to16_ps((float)neg_inf); }

    static __forceinline float16 loadu(const void* addr) 
    {
#if defined(__AVX512F__)
      return _mm512_loadu_ps(addr);  
#else
      float16 r = float16::undefined();
      r =_mm512_extloadunpacklo_ps(r, addr, _MM_UPCONV_PS_NONE, _MM_HINT_NONE);
      return _mm512_extloadunpackhi_ps(r, (float*)addr+16, _MM_UPCONV_PS_NONE, _MM_HINT_NONE);  
#endif
    }

    static __forceinline float16 load(const void *f) { 
      return _mm512_extload_ps(f,_MM_UPCONV_PS_NONE,_MM_BROADCAST_16X16,_MM_HINT_NONE);  
    }

    ////////////////////////////////////////////////////////////////////////////////
    /// Array Access
    ////////////////////////////////////////////////////////////////////////////////
    
    __forceinline float&       operator[](const size_t index)       { return f[index]; };
    __forceinline const float& operator[](const size_t index) const { return f[index]; };
  };

  ////////////////////////////////////////////////////////////////////////////////
  /// Unary Operators
  ////////////////////////////////////////////////////////////////////////////////

  __forceinline const float16 cast      (const __m512i& a) { return _mm512_castsi512_ps(a); }
  __forceinline const float16 operator +( const float16& a ) { return a; }
  __forceinline const float16 operator -( const float16& a ) { return _mm512_mul_ps(a,float16(-1)); }
  __forceinline const float16 abs       ( const float16& a ) { 
#if defined(__AVX512F__)
    return _mm512_abs_ps(a); 
#else
    return _mm512_gmaxabs_ps(a,a); 
#endif
  }
  __forceinline const float16 signmsk   ( const float16& a ) { return _mm512_castsi512_ps(_mm512_and_epi32(_mm512_castps_si512(a),_mm512_set1_epi32(0x80000000))); }

  __forceinline const float16 rcp  ( const float16& a ) { 
#if defined(__AVX512F__)
    return _mm512_rcp28_ps(a); 
#else
    return _mm512_rcp23_ps(a); 
#endif
  };

  __forceinline const float16 sqr  ( const float16& a ) { return _mm512_mul_ps(a,a); }
  __forceinline const float16 sqrt ( const float16& a ) { return _mm512_sqrt_ps(a); }
  __forceinline const float16 rsqrt( const float16& a ) { return _mm512_invsqrt_ps(a); }

  __forceinline float16 exp(const float16& a) { return _mm512_exp_ps(a); }
  __forceinline float16 exp2(const float16& a) { return _mm512_exp2_ps(a); }
  __forceinline float16 pow(const float16& a, float16 b) { return _mm512_pow_ps(a,b); }
  
  __forceinline float16 log(const float16& a) { return _mm512_log_ps(a); }
  __forceinline float16 log2(const float16& a) { return _mm512_log2_ps(a); }
  __forceinline float16 log10(const float16& a) { return _mm512_log10_ps(a); }
  
  __forceinline float16 sin(const float16& a) { return _mm512_sin_ps(a); } 
  __forceinline float16 cos(const float16& a) { return _mm512_cos_ps(a); }
  __forceinline float16 tan(const float16& a) { return _mm512_tan_ps(a); } 
  
  __forceinline float16 asin(const float16& a) { return _mm512_asin_ps(a); }
  __forceinline float16 acos(const float16& a) { return _mm512_acos_ps(a); }
  __forceinline float16 atan(const float16& a) { return _mm512_atan_ps(a); }
  __forceinline float16 atan2(const float16& a, float16 b) { return _mm512_atan2_ps(a,b); }
  
  __forceinline float16 sinh(const float16& a) { return _mm512_sinh_ps(a); } 
  __forceinline float16 cosh(const float16& a) { return _mm512_cosh_ps(a); }
  __forceinline float16 tanh(const float16& a) { return _mm512_tan_ps(a); } 
  
  __forceinline float16 asinh(const float16& a) { return _mm512_asinh_ps(a); }
  __forceinline float16 acosh(const float16& a) { return _mm512_acosh_ps(a); }
  __forceinline float16 atanh(const float16& a) { return _mm512_atanh_ps(a); }
  
  ////////////////////////////////////////////////////////////////////////////////
  /// Binary Operators
  ////////////////////////////////////////////////////////////////////////////////

  __forceinline const float16 operator +( const float16& a, const float16& b ) { return _mm512_add_ps(a, b); }
  __forceinline const float16 operator +( const float16& a, const float& b ) { return a + float16(b); }
  __forceinline const float16 operator +( const float& a, const float16& b ) { return float16(a) + b; }

  __forceinline const float16 operator -( const float16& a, const float16& b ) { return _mm512_sub_ps(a, b); }
  __forceinline const float16 operator -( const float16& a, const float& b ) { return a - float16(b); }
  __forceinline const float16 operator -( const float& a, const float16& b ) { return float16(a) - b; }

  __forceinline const float16 operator *( const float16& a, const float16& b ) { return _mm512_mul_ps(a, b); }
  __forceinline const float16 operator *( const float16& a, const float& b ) { return a * float16(b); }
  __forceinline const float16 operator *( const float& a, const float16& b ) { return float16(a) * b; }

  __forceinline const float16 operator /( const float16& a, const float16& b ) { return _mm512_div_ps(a,b); }
  __forceinline const float16 operator /( const float16& a, const float& b ) { return a/float16(b); }
  __forceinline const float16 operator /( const float& a, const float16& b ) { return float16(a)/b; }
  
  __forceinline const float16 operator^(const float16& a, const float16& b) { 
    return  _mm512_castsi512_ps(_mm512_xor_epi32(_mm512_castps_si512(a),_mm512_castps_si512(b))); 
  }
  
  __forceinline const float16 min( const float16& a, const float16& b ) { 
#if defined(__AVX512F__)
    return _mm512_min_ps(a,b); 
#else
    return _mm512_gmin_ps(a,b); 
#endif
  }
  __forceinline const float16 min( const float16& a, const float& b ) { 
#if defined(__AVX512F__)
    return _mm512_min_ps(a,float16(b)); 
#else    
    return _mm512_gmin_ps(a,float16(b)); 
#endif
  }
  __forceinline const float16 min( const float& a, const float16& b ) { 
#if defined(__AVX512F__)
    return _mm512_min_ps(float16(a),b); 
#else
    return _mm512_gmin_ps(float16(a),b); 
#endif
  }

  __forceinline const float16 max( const float16& a, const float16& b ) { 
#if defined(__AVX512F__)
    return _mm512_max_ps(a,b); 
#else
    return _mm512_gmax_ps(a,b); 
#endif
  }
  __forceinline const float16 max( const float16& a, const float& b ) { 
#if defined(__AVX512F__)
    return _mm512_max_ps(a,float16(b)); 
#else
    return _mm512_gmax_ps(a,float16(b)); 
#endif
  }
  __forceinline const float16 max( const float& a, const float16& b ) { 
#if defined(__AVX512F__)
    return _mm512_max_ps(float16(a),b); 
#else
    return _mm512_gmax_ps(float16(a),b); 
#endif
  }

  __forceinline float16 mask_add(const bool16& mask, const float16& c, const float16& a, const float16& b) { return _mm512_mask_add_ps (c,mask,a,b); }; 
  __forceinline float16 mask_min(const bool16& mask, const float16& c, const float16& a, const float16& b) { 
#if defined(__AVX512F__)
    return _mm512_mask_min_ps(c,mask,a,b); 
#else
    return _mm512_mask_gmin_ps(c,mask,a,b); 
#endif
  }; 
  __forceinline float16 mask_max(const bool16& mask, const float16& c, const float16& a, const float16& b) { 
#if defined(__AVX512F__)
    return _mm512_mask_max_ps(c,mask,a,b); 
#else
    return _mm512_mask_gmax_ps(c,mask,a,b); 
#endif
  }; 

  ////////////////////////////////////////////////////////////////////////////////
  /// Ternary Operators
  ////////////////////////////////////////////////////////////////////////////////

  __forceinline float16 madd (const float16& a, const float16& b, const float16& c) { return _mm512_fmadd_ps(a,b,c); }

  __forceinline float16 msub (const float16& a, const float16& b, const float16& c) { return _mm512_fmsub_ps(a,b,c); }
  __forceinline float16 nmadd (const float16& a, const float16& b, const float16& c) { return _mm512_fnmadd_ps(a,b,c); }
  __forceinline float16 nmsub (const float16& a, const float16& b, const float16& c) { return _mm512_fnmsub_ps(a,b,c); }

  __forceinline float16 mask_msub (const bool16& mask,const float16& a, const float16& b, const float16& c) { return _mm512_mask_fmsub_ps(a,mask,b,c); }
  
  __forceinline float16 madd231 (const float16& a, const float16& b, const float16& c) { return _mm512_fmadd_ps(c,b,a); }
  __forceinline float16 msub213 (const float16& a, const float16& b, const float16& c) { return _mm512_fmsub_ps(a,b,c); }
  __forceinline float16 msub231 (const float16& a, const float16& b, const float16& c) { return _mm512_fmsub_ps(c,b,a); }
  __forceinline float16 msubr231(const float16& a, const float16& b, const float16& c) { return _mm512_fnmadd_ps(c,b,a); }


  ////////////////////////////////////////////////////////////////////////////////
  /// Operators with rounding
  ////////////////////////////////////////////////////////////////////////////////

  __forceinline float16 madd_round_down (const float16& a, const float16& b, const float16& c) { return _mm512_fmadd_round_ps(a,b,c,_MM_FROUND_TO_NEG_INF); }

  __forceinline float16 madd_round_up (const float16& a, const float16& b, const float16& c) { return _mm512_fmadd_round_ps(a,b,c,_MM_FROUND_TO_POS_INF); }

  __forceinline float16 mul_round_down (const float16& a, const float16& b) { return _mm512_mul_round_ps(a,b,_MM_FROUND_TO_NEG_INF); }
  __forceinline float16 mul_round_up   (const float16& a, const float16& b) { return _mm512_mul_round_ps(a,b,_MM_FROUND_TO_POS_INF); }

  __forceinline float16 add_round_down (const float16& a, const float16& b) { return _mm512_add_round_ps(a,b,_MM_FROUND_TO_NEG_INF); }
  __forceinline float16 add_round_up   (const float16& a, const float16& b) { return _mm512_add_round_ps(a,b,_MM_FROUND_TO_POS_INF); }

  __forceinline float16 sub_round_down (const float16& a, const float16& b) { return _mm512_sub_round_ps(a,b,_MM_FROUND_TO_NEG_INF); }
  __forceinline float16 sub_round_up   (const float16& a, const float16& b) { return _mm512_sub_round_ps(a,b,_MM_FROUND_TO_POS_INF); }

  __forceinline float16 div_round_down (const float16& a, const float16& b) { return _mm512_div_round_ps(a,b,_MM_FROUND_TO_NEG_INF); }
  __forceinline float16 div_round_up   (const float16& a, const float16& b) { return _mm512_div_round_ps(a,b,_MM_FROUND_TO_POS_INF); }

  __forceinline float16 mask_msub_round_down (const bool16& mask,const float16& a, const float16& b, const float16& c) { return _mm512_mask_fmsub_round_ps(a,mask,b,c,_MM_FROUND_TO_NEG_INF); }
  __forceinline float16 mask_msub_round_up   (const bool16& mask,const float16& a, const float16& b, const float16& c) { return _mm512_mask_fmsub_round_ps(a,mask,b,c,_MM_FROUND_TO_POS_INF); }
  
  __forceinline float16 mask_mul_round_down (const bool16& mask,const float16& a, const float16& b, const float16& c) { return _mm512_mask_mul_round_ps(a,mask,b,c,_MM_FROUND_TO_NEG_INF); }
  __forceinline float16 mask_mul_round_up   (const bool16& mask,const float16& a, const float16& b, const float16& c) { return _mm512_mask_mul_round_ps(a,mask,b,c,_MM_FROUND_TO_POS_INF); }

  __forceinline float16 mask_sub_round_down (const bool16& mask,const float16& a, const float16& b, const float16& c) { return _mm512_mask_sub_round_ps(a,mask,b,c,_MM_FROUND_TO_NEG_INF); }
  __forceinline float16 mask_sub_round_up   (const bool16& mask,const float16& a, const float16& b, const float16& c) { return _mm512_mask_sub_round_ps(a,mask,b,c,_MM_FROUND_TO_POS_INF); }

  ////////////////////////////////////////////////////////////////////////////////
  /// Assignment Operators
  ////////////////////////////////////////////////////////////////////////////////

  __forceinline float16& operator +=( float16& a, const float16& b ) { return a = a + b; }
  __forceinline float16& operator +=( float16& a, const float& b ) { return a = a + b; }
  
  __forceinline float16& operator -=( float16& a, const float16& b ) { return a = a - b; }
  __forceinline float16& operator -=( float16& a, const float& b ) { return a = a - b; }
  
  __forceinline float16& operator *=( float16& a, const float16& b ) { return a = a * b; }
  __forceinline float16& operator *=( float16& a, const float& b ) { return a = a * b; }

  __forceinline float16& operator /=( float16& a, const float16& b ) { return a = a / b; }
  __forceinline float16& operator /=( float16& a, const float& b ) { return a = a / b; }

  ////////////////////////////////////////////////////////////////////////////////
  /// Comparison Operators + Select
  ////////////////////////////////////////////////////////////////////////////////

  __forceinline const bool16 operator ==( const float16& a, const float16& b ) { return _mm512_cmp_ps_mask(a,b,_MM_CMPINT_EQ); }
  __forceinline const bool16 operator ==( const float16& a, const float& b ) { return a == float16(b); }
  __forceinline const bool16 operator ==( const float& a, const float16& b ) { return float16(a) == b; }

  __forceinline const bool16 operator !=( const float16& a, const float16& b ) { return _mm512_cmp_ps_mask(a,b,_MM_CMPINT_NE); }
  __forceinline const bool16 operator !=( const float16& a, const float& b ) { return a != float16(b); }
  __forceinline const bool16 operator !=( const float& a, const float16& b ) { return float16(a) != b; }

  __forceinline const bool16 operator < ( const float16& a, const float16& b ) { return _mm512_cmp_ps_mask(a,b,_MM_CMPINT_LT); }
  __forceinline const bool16 operator < ( const float16& a, const float& b ) { return a <  float16(b); }
  __forceinline const bool16 operator < ( const float& a, const float16& b ) { return float16(a) <  b; }

  __forceinline const bool16 operator >=( const float16& a, const float16& b ) { return _mm512_cmp_ps_mask(a,b,_MM_CMPINT_GE); }
  __forceinline const bool16 operator >=( const float16& a, const float& b ) { return a >= float16(b); }
  __forceinline const bool16 operator >=( const float& a, const float16& b ) { return float16(a) >= b; }

  __forceinline const bool16 operator > ( const float16& a, const float16& b ) { return _mm512_cmp_ps_mask(a,b,_MM_CMPINT_GT); }
  __forceinline const bool16 operator > ( const float16& a, const float& b ) { return a >  float16(b); }
  __forceinline const bool16 operator > ( const float& a, const float16& b ) { return float16(a) >  b; }

  __forceinline const bool16 operator <=( const float16& a, const float16& b ) { return _mm512_cmp_ps_mask(a,b,_MM_CMPINT_LE); }
  __forceinline const bool16 operator <=( const float16& a, const float& b ) { return a <= float16(b); }
  __forceinline const bool16 operator <=( const float& a, const float16& b ) { return float16(a) <= b; }

  __forceinline bool16 eq(                   const float16& a, const float16& b) { return _mm512_cmp_ps_mask(a,b,_MM_CMPINT_EQ); }
  __forceinline bool16 eq(const bool16& mask, const float16& a, const float16& b) { return _mm512_mask_cmp_ps_mask(mask,a,b,_MM_CMPINT_EQ); }

  __forceinline bool16 ne(                   const float16& a, const float16& b) { return _mm512_cmp_ps_mask(a,b,_MM_CMPINT_NE); }
  __forceinline bool16 ne(const bool16& mask, const float16& a, const float16& b) { return _mm512_mask_cmp_ps_mask(mask,a,b,_MM_CMPINT_NE); }

  __forceinline bool16 lt(                   const float16& a, const float16& b) { return _mm512_cmp_ps_mask(a,b,_MM_CMPINT_LT); }
  __forceinline bool16 lt(const bool16& mask, const float16& a, const float16& b) { return _mm512_mask_cmp_ps_mask(mask,a,b,_MM_CMPINT_LT); }
 
  __forceinline bool16 ge(                   const float16& a, const float16& b) { return _mm512_cmp_ps_mask(a,b,_MM_CMPINT_GE); }
  __forceinline bool16 ge(const bool16& mask, const float16& a, const float16& b) { return _mm512_mask_cmp_ps_mask(mask,a,b,_MM_CMPINT_GE); }

  __forceinline bool16 gt(                   const float16& a, const float16& b) { return _mm512_cmp_ps_mask(a,b,_MM_CMPINT_GT); }
  __forceinline bool16 gt(const bool16& mask, const float16& a, const float16& b) { return _mm512_mask_cmp_ps_mask(mask,a,b,_MM_CMPINT_GT); }
  
  __forceinline bool16 le(                   const float16& a, const float16& b) { return _mm512_cmp_ps_mask(a,b,_MM_CMPINT_LE); }
  __forceinline bool16 le(const bool16& mask, const float16& a, const float16& b) { return _mm512_mask_cmp_ps_mask(mask,a,b,_MM_CMPINT_LE); }
  
  __forceinline const float16 select( const bool16& s, const float16& t, const float16& f ) {
    return _mm512_mask_blend_ps(s, f, t);
  }

  __forceinline float16  lerp(const float16& a, const float16& b, const float16& t) {
    return madd(t, b, madd(-t, a, a));
  }

  __forceinline void xchg(bool16 m, float16& a, float16& b) 
  {
    float16 c = a;
    a = select(m,b,a);
    b = select(m,c,b); 
  }
  ////////////////////////////////////////////////////////////////////////////////
  /// Rounding Functions
  ////////////////////////////////////////////////////////////////////////////////

  __forceinline float16 vround(const float16& f, 
                             const _MM_ROUND_MODE_ENUM mode, 
                             const _MM_EXP_ADJ_ENUM exp = _MM_EXPADJ_NONE) 
  { 
    return _mm512_round_ps(f,mode,exp); 
  }
  
  __forceinline float16 floor(const float16& a) { return _mm512_round_ps(a,_MM_ROUND_MODE_DOWN, _MM_EXPADJ_NONE); }
  __forceinline float16 ceil (const float16& a) { return _mm512_round_ps(a,_MM_ROUND_MODE_UP  , _MM_EXPADJ_NONE); }
  __forceinline float16 trunc(const float16& a) { return _mm512_trunc_ps(a); } 
  __forceinline float16 frac( const float16& a ) { return a-trunc(a); }

  __forceinline const float16 rcp_nr  ( const float16& a ) { 
    const float16 ra = _mm512_rcp23_ps(a); 
    return (ra+ra) - (ra * a * ra);
  };

  ////////////////////////////////////////////////////////////////////////////////
  /// Movement/Shifting/Shuffling Functions
  ////////////////////////////////////////////////////////////////////////////////

  __forceinline float16 swizzle(const float16& x,_MM_SWIZZLE_ENUM perm32 ) { return _mm512_swizzle_ps(x,perm32); }
  __forceinline float16 permute(const float16& x,_MM_PERM_ENUM    perm128) { return _mm512_permute4f128_ps(x,perm128); }
  
  template<int D, int C, int B, int A> __forceinline float16 swizzle   (const float16& v) { return _mm512_shuffle_ps(v,_MM_SHUF_PERM(D,C,B,A)); }
  template<int A>                      __forceinline float16 swizzle   (const float16& x) { return swizzle<A,A,A,A>(v); }
  template<>                           __forceinline float16 swizzle<0>(const float16& x) { return swizzle(x,_MM_SWIZ_REG_AAAA); }
  template<>                           __forceinline float16 swizzle<1>(const float16& x) { return swizzle(x,_MM_SWIZ_REG_BBBB); }
  template<>                           __forceinline float16 swizzle<2>(const float16& x) { return swizzle(x,_MM_SWIZ_REG_CCCC); }
  template<>                           __forceinline float16 swizzle<3>(const float16& x) { return swizzle(x,_MM_SWIZ_REG_DDDD); }

  template<int D, int C, int B, int A> __forceinline float16 permute(const float16& v) { return permute(v,_MM_SHUF_PERM(D,C,B,A)); }
  template<int A>                      __forceinline float16 permute(const float16& x) { return permute<A,A,A,A>(x); }

  __forceinline float16 shuffle(const float16& x,_MM_PERM_ENUM perm128, _MM_SWIZZLE_ENUM perm32) { return swizzle(permute(x,perm128),perm32); }
  
  __forceinline float16 shuffle(const bool16& mask, float16& v, const float16& x,_MM_PERM_ENUM perm128, _MM_SWIZZLE_ENUM perm32)  {
    return _mm512_mask_swizzle_ps(_mm512_mask_permute4f128_ps(v,mask,x,perm128),mask,x,perm32);  
  }

  __forceinline float16 swAAAA(const float16 &x) {
    return swizzle(x,_MM_SWIZ_REG_AAAA);
  }

  __forceinline float16 swBBBB(const float16 &x) {
    return swizzle(x,_MM_SWIZ_REG_BBBB);
  }

  __forceinline float16 swCCCC(const float16 &x) {
    return swizzle(x,_MM_SWIZ_REG_CCCC);
  }

  __forceinline float16 swDDDD(const float16 &x) {
    return swizzle(x,_MM_SWIZ_REG_DDDD);
  }

  __forceinline float16 _mm512_permutev_ps(__m512i index, float16 v)
  {
    return _mm512_castsi512_ps(_mm512_permutev_epi32(index,_mm512_castps_si512(v)));  
  }

  __forceinline float16 permute16f(__m512i index, float16 v)
  {
    return _mm512_castsi512_ps(_mm512_permutev_epi32(index,_mm512_castps_si512(v)));  
  }

  template<int i>
  __forceinline float16 align_shift_right(const float16 &a, const float16 &b)
  {
    return _mm512_castsi512_ps(_mm512_alignr_epi32(_mm512_castps_si512(a),_mm512_castps_si512(b),i)); 
  };

  template<int i>
    __forceinline float16 mask_align_shift_right(const bool16 &mask,float16 &c,const float16 &a, const float16 &b)
    {
      return _mm512_castsi512_ps(_mm512_mask_alignr_epi32(_mm512_castps_si512(c),mask,_mm512_castps_si512(a),_mm512_castps_si512(b),i)); 
    };
 
  __forceinline float16 shl1_zero_extend(const float16 &a)
  {
    float16 z = float16::zero();
    return mask_align_shift_right<15>(0xfffe,z,a,a);
  }  


  ////////////////////////////////////////////////////////////////////////////////
  /// Reductions
  ////////////////////////////////////////////////////////////////////////////////

  __forceinline float reduce_add(float16 a) { return _mm512_reduce_add_ps(a); }
  __forceinline float reduce_mul(float16 a) { return _mm512_reduce_mul_ps(a); }
  __forceinline float reduce_min(float16 a) { 
#if defined(__AVX512F__)
    return _mm512_reduce_min_ps(a); 
#else
    return _mm512_reduce_gmin_ps(a); 
#endif
  }
  __forceinline float reduce_max(float16 a) { 
#if defined(__AVX512F__)
    return _mm512_reduce_max_ps(a); 
#else
    return _mm512_reduce_gmax_ps(a); 
#endif

  }

  __forceinline float16 vreduce_min2(float16 x) {                      return min(x,swizzle(x,_MM_SWIZ_REG_BADC)); }
  __forceinline float16 vreduce_min4(float16 x) { x = vreduce_min2(x); return min(x,swizzle(x,_MM_SWIZ_REG_CDAB)); }
  __forceinline float16 vreduce_min8(float16 x) { x = vreduce_min4(x); return min(x,permute(x,_MM_SHUF_PERM(2,3,0,1))); }
  __forceinline float16 vreduce_min (float16 x) { x = vreduce_min8(x); return min(x,permute(x,_MM_SHUF_PERM(1,0,3,2))); }

  __forceinline float16 vreduce_max2(float16 x) {                      return max(x,swizzle(x,_MM_SWIZ_REG_BADC)); }
  __forceinline float16 vreduce_max4(float16 x) { x = vreduce_max2(x); return max(x,swizzle(x,_MM_SWIZ_REG_CDAB)); }
  __forceinline float16 vreduce_max8(float16 x) { x = vreduce_max4(x); return max(x,permute(x,_MM_SHUF_PERM(2,3,0,1))); }
  __forceinline float16 vreduce_max (float16 x) { x = vreduce_max8(x); return max(x,permute(x,_MM_SHUF_PERM(1,0,3,2))); }

  __forceinline float16 vreduce_add2(float16 x) {                      return x + swizzle(x,_MM_SWIZ_REG_BADC); }
  __forceinline float16 vreduce_add4(float16 x) { x = vreduce_add2(x); return x + swizzle(x,_MM_SWIZ_REG_CDAB); }
  __forceinline float16 vreduce_add8(float16 x) { x = vreduce_add4(x); return x + permute(x,_MM_SHUF_PERM(2,3,0,1)); }
  __forceinline float16 vreduce_add (float16 x) { x = vreduce_add8(x); return x + permute(x,_MM_SHUF_PERM(1,0,3,2)); }

  __forceinline size_t select_min(const float16& v) { return __bsf(movemask(v == vreduce_min(v))); }
  __forceinline size_t select_max(const float16& v) { return __bsf(movemask(v == vreduce_max(v))); }

  __forceinline size_t select_min(const bool16& valid, const float16& v, const float16 &max_value) { const float16 a = select(valid,v,max_value); return __bsf(movemask(a == vreduce_min(a))); }

  __forceinline size_t select_max(const bool16& valid, const float16& v, const float16 &min_value) { const float16 a = select(valid,v,min_value); return __bsf(movemask(a == vreduce_max(a))); }

  __forceinline size_t select_max(const bool16& valid, const float16& v) { const float16 a = select(valid,v,float16(neg_inf)); return __bsf(movemask(valid & (a == vreduce_max(a)))); }

  __forceinline size_t select_min(const bool16& valid, const float16& v) { const float16 a = select(valid,v,float16(pos_inf)); return __bsf(movemask(valid & (a == vreduce_min(a)))); }
  
  __forceinline float16 prefix_sum(const float16& a)
  {
    float16 v = a;
    v = mask_add(0xaaaa,v,v,swizzle(v,_MM_SWIZ_REG_CDAB));
    v = mask_add(0xcccc,v,v,swizzle(v,_MM_SWIZ_REG_BBBB));
    const float16 shuf_v0 = shuffle(v,_MM_SHUF_PERM(2,2,0,0),_MM_SWIZ_REG_DDDD);
    v = mask_add(0xf0f0,v,v,shuf_v0);
    const float16 shuf_v1 = shuffle(v,_MM_SHUF_PERM(1,1,0,0),_MM_SWIZ_REG_DDDD);
    v = mask_add(0xff00,v,v,shuf_v1);
    return v;  
  }

  __forceinline float16 prefix_min(const float16& a)
  {
    float16 v = a;
    v = mask_min(0xaaaa,v,v,swizzle(v,_MM_SWIZ_REG_CDAB));
    v = mask_min(0xcccc,v,v,swizzle(v,_MM_SWIZ_REG_BBBB));
    const float16 shuf_v0 = shuffle(v,(_MM_PERM_ENUM)_MM_SHUF_PERM(2,2,0,0),_MM_SWIZ_REG_DDDD);
    v = mask_min(0xf0f0,v,v,shuf_v0);
    const float16 shuf_v1 = shuffle(v,(_MM_PERM_ENUM)_MM_SHUF_PERM(1,1,0,0),_MM_SWIZ_REG_DDDD);
    v = mask_min(0xff00,v,v,shuf_v1);
    return v;  
  }
  
  __forceinline float16 prefix_max(const float16& a)
  {
    float16 v = a;
    v = mask_max(0xaaaa,v,v,swizzle(v,_MM_SWIZ_REG_CDAB));
    v = mask_max(0xcccc,v,v,swizzle(v,_MM_SWIZ_REG_BBBB));
    const float16 shuf_v0 = shuffle(v,(_MM_PERM_ENUM)_MM_SHUF_PERM(2,2,0,0),_MM_SWIZ_REG_DDDD);
    v = mask_max(0xf0f0,v,v,shuf_v0);
    const float16 shuf_v1 = shuffle(v,(_MM_PERM_ENUM)_MM_SHUF_PERM(1,1,0,0),_MM_SWIZ_REG_DDDD);
    v = mask_max(0xff00,v,v,shuf_v1);
    return v;  
  }

  __forceinline float16 set_min4(float16 x) {
    x = min(x,swizzle(x,_MM_SWIZ_REG_BADC));
    x = min(x,swizzle(x,_MM_SWIZ_REG_CDAB));
    return x;
  }

  __forceinline float16 set_min_lanes(float16 x) {
    x = min(x,_mm512_permute4f128_ps(x, _MM_PERM_CDAB));
    x = min(x,_mm512_permute4f128_ps(x, _MM_PERM_BADC));
    return x;
  }

  __forceinline float16 set_max_lanes(float16 x) {
    x = max(x,_mm512_permute4f128_ps(x, _MM_PERM_CDAB));
    x = max(x,_mm512_permute4f128_ps(x, _MM_PERM_BADC));
    return x;
  }

  __forceinline float16 set_min16(float16 x) {
    return set_min_lanes(set_min4(x));
  }



  ////////////////////////////////////////////////////////////////////////////////
  /// Memory load and store operations
  ////////////////////////////////////////////////////////////////////////////////

  __forceinline float16 load1f(const void *f) { 
    return _mm512_extload_ps(f,_MM_UPCONV_PS_NONE,_MM_BROADCAST_1X16,_MM_HINT_NONE);  
  }
  
  __forceinline float16 load16f(const void *f) { 
    return _mm512_extload_ps(f,_MM_UPCONV_PS_NONE,_MM_BROADCAST_16X16,_MM_HINT_NONE);  
  }

  __forceinline float16 load16f(const float16 d, const bool16 valid, const void *f) { 
    return _mm512_mask_extload_ps(d,valid,f,_MM_UPCONV_PS_NONE,_MM_BROADCAST_16X16,_MM_HINT_NONE);  
  }

  __forceinline float16 broadcast1to16f(const void *f) { 
    return _mm512_extload_ps(f,_MM_UPCONV_PS_NONE,_MM_BROADCAST_1X16,_MM_HINT_NONE);  
  }
  
  __forceinline float16 broadcast4to16f(const void *f) { 
    return _mm512_extload_ps(f,_MM_UPCONV_PS_NONE,_MM_BROADCAST_4X16,0);  
  }

  __forceinline float16 broadcast8to16f(const void *f) { 
    return  _mm512_castpd_ps(_mm512_extload_pd(f,_MM_UPCONV_PD_NONE,_MM_BROADCAST_4X8,0));  
  }
    
  __forceinline float16 load16f_uint8(const unsigned char *const ptr) {
    return _mm512_mul_ps(_mm512_extload_ps(ptr,_MM_UPCONV_PS_UINT8,_MM_BROADCAST_16X16,_MM_HINT_NONE),float16(1.0f/255.0f));  
  }

  __forceinline float16 load16f_uint16(const unsigned short *const ptr) {
    return _mm512_mul_ps(_mm512_extload_ps(ptr,_MM_UPCONV_PS_UINT16,_MM_BROADCAST_16X16,_MM_HINT_NONE),float16(1.0f/65535.0f));  
  }

#if !defined(__AVX512F__)
  __forceinline float16 uload16f_low_uint8(const bool16& mask, const void* addr, const float16& v1) {
    return _mm512_mask_extloadunpacklo_ps(v1, mask, addr, _MM_UPCONV_PS_UINT8, _MM_HINT_NONE);
  }
#endif

  __forceinline float16 load16f_int8(const char *const ptr) {
    return _mm512_mul_ps(_mm512_extload_ps(ptr,_MM_UPCONV_PS_SINT8,_MM_BROADCAST_16X16,_MM_HINT_NONE),float16(1.0f/127.0f));  
  }


  __forceinline float16 gather16f_4f(const float *__restrict__ const ptr0,
                                   const float *__restrict__ const ptr1,
                                   const float *__restrict__ const ptr2,
                                   const float *__restrict__ const ptr3) 
  {
    float16 v = broadcast4to16f(ptr0);
    v = select((bool16)0x00f0,broadcast4to16f(ptr1),v);
    v = select((bool16)0x0f00,broadcast4to16f(ptr2),v);
    v = select((bool16)0xf000,broadcast4to16f(ptr3),v);
    return v;
  }
  
  __forceinline float16 gather16f_4f(const float16& v0,
                                   const float16& v1,
                                   const float16& v2,
                                   const float16& v3) 
  {
    float16 v = v0;
    v = select((bool16)0xf0  ,v1,v);
    v = select((bool16)0xf00 ,v2,v);
    v = select((bool16)0xf000,v3,v);
    return v;
  }

  __forceinline float16 gather_4f_zlc(const int16 &v_mask,
                                    const void *__restrict__ const ptr0,
                                    const void *__restrict__ const ptr1,
                                    const void *__restrict__ const ptr2,
                                    const void *__restrict__ const ptr3) 
  {
    const bool16 m_00f0 = 0x00f0;
    const bool16 m_0f00 = 0x0f00;
    const bool16 m_f000 = 0xf000;
    
    int16 v = v_mask &  broadcast4to16i((const int*)ptr0);
    v = mask_and(m_00f0,v,v_mask, broadcast4to16i((const int*)ptr1));
    v = mask_and(m_0f00,v,v_mask, broadcast4to16i((const int*)ptr2));
    v = mask_and(m_f000,v,v_mask, broadcast4to16i((const int*)ptr3));
    return cast(v);
  }

  __forceinline float16 gather_2f_zlc(const int16 &v_mask,
				    const bool16 &mask,
                                    const void *__restrict__ const ptr0,
                                    const void *__restrict__ const ptr1) 
  {
    int16 v = v_mask &  broadcast4to16i((const int*)ptr0);
    v = mask_and(mask,v,v_mask, broadcast4to16i((const int*)ptr1));
    return cast(v);
  }


  __forceinline float16 gather16f_4f_align(const void *__restrict__ const ptr0,
					 const void *__restrict__ const ptr1,
					 const void *__restrict__ const ptr2,
					 const void *__restrict__ const ptr3) 
  {
    float16 v = broadcast4to16f(ptr3);
    v = align_shift_right<12>(v,broadcast4to16f(ptr2));
    v = align_shift_right<12>(v,broadcast4to16f(ptr1));
    v = align_shift_right<12>(v,broadcast4to16f(ptr0));
    return v;
  }


  __forceinline float16 gather16f_4f_align(const float16& v0,
					 const float16& v1,
					 const float16& v2,
					 const float16& v3) 
  {
    float16 v = v3;
    v = align_shift_right<12>(v,v2);
    v = align_shift_right<12>(v,v1);
    v = align_shift_right<12>(v,v0);
    return v;
  }

  __forceinline float16 gather_4f_zlc_align(const int16 &v_mask,
					  const void *__restrict__ const ptr0,
					  const void *__restrict__ const ptr1,
					  const void *__restrict__ const ptr2,
					  const void *__restrict__ const ptr3) 
  {
    float16 v = gather16f_4f_align(ptr0,ptr1,ptr2,ptr3);
    return cast(cast(v) & v_mask);
  }


  __forceinline float16 uload16f(const bool16& mask,const float *const addr) {
    float16 r = float16::undefined();
    r =_mm512_mask_extloadunpacklo_ps(r, mask,addr, _MM_UPCONV_PS_NONE, _MM_HINT_NONE);
    r = _mm512_mask_extloadunpackhi_ps(r, mask, addr+16, _MM_UPCONV_PS_NONE, _MM_HINT_NONE);  
    return r;
  }

  __forceinline float16 uload16f(float16 &r, const bool16& mask, const float *const addr) {
    r =_mm512_mask_extloadunpacklo_ps(r, mask,addr, _MM_UPCONV_PS_NONE, _MM_HINT_NONE);
    r = _mm512_mask_extloadunpackhi_ps(r, mask, addr+16, _MM_UPCONV_PS_NONE, _MM_HINT_NONE);  
    return r;
  }

  __forceinline float16 uload16f(const float *const addr) {
    float16 r = float16::undefined();
    r =_mm512_extloadunpacklo_ps(r, addr, _MM_UPCONV_PS_NONE, _MM_HINT_NONE);
    return _mm512_extloadunpackhi_ps(r, addr+16, _MM_UPCONV_PS_NONE, _MM_HINT_NONE);  
  }
  
  __forceinline float16 uload16f_low(const float *const addr) {
    float16 r = float16::undefined();
    return _mm512_extloadunpacklo_ps(r, addr, _MM_UPCONV_PS_NONE, _MM_HINT_NONE);
  }
  
  __forceinline float16 uload16f_low(const bool16& mask, const void* addr, const float16& v1) {
    return _mm512_mask_extloadunpacklo_ps(v1, mask, addr, _MM_UPCONV_PS_NONE, _MM_HINT_NONE);
  }

  __forceinline float16 uload16f_low(const bool16& mask, const void* addr) {
    float16 v1 = float16::undefined();
    return _mm512_mask_extloadunpacklo_ps(v1, mask, addr, _MM_UPCONV_PS_NONE, _MM_HINT_NONE);
  }
  
  __forceinline void ustore16f(float *addr, const float16& reg) {
#if defined(__AVX512F__)
      _mm512_storeu_ps(addr,reg);
#else
    _mm512_extpackstorelo_ps(addr+0 ,reg, _MM_DOWNCONV_PS_NONE , 0);
    _mm512_extpackstorehi_ps(addr+16 ,reg, _MM_DOWNCONV_PS_NONE , 0);
#endif
  }
  
  /* pass by value to avoid compiler generating inefficient code */
  __forceinline void compactustore16f(const bool16& mask, float *addr, const float16 reg) {
    _mm512_mask_extpackstorelo_ps(addr+0 ,mask, reg, _MM_DOWNCONV_PS_NONE , 0);
    _mm512_mask_extpackstorehi_ps(addr+16 ,mask, reg, _MM_DOWNCONV_PS_NONE , 0);
  }
  
  __forceinline void compactustore16f_low(const bool16& mask, float * addr, const float16 &reg) {
    _mm512_mask_extpackstorelo_ps(addr+0 ,mask, reg, _MM_DOWNCONV_PS_NONE , 0);
  }

  __forceinline void compactustore16f_low_uint8(const bool16& mask, void * addr, const float16 &reg) {
    _mm512_mask_extpackstorelo_ps(addr+0 ,mask, reg, _MM_DOWNCONV_PS_UINT8 , 0);
  }
  
  __forceinline void ustore16f_low(float * addr, const float16& reg) {
    _mm512_extpackstorelo_ps(addr+0 ,reg, _MM_DOWNCONV_PS_NONE , 0);
  }
  
  __forceinline void compactustore16f_high(const bool16& mask, float *addr, const float16& reg) {
    _mm512_extpackstorehi_ps(addr+0 ,reg, _MM_DOWNCONV_PS_NONE , 0);
  }
  
  __forceinline void store16f(const bool16& mask, void* addr, const float16& v2) {
    _mm512_mask_extstore_ps(addr,mask,v2,_MM_DOWNCONV_PS_NONE,0);
  }

  
  __forceinline void store16f(void* addr, const float16& v2) {
    _mm512_extstore_ps(addr,v2,_MM_DOWNCONV_PS_NONE,0);
  }

  __forceinline void store16f_int8(void* addr, const float16& v2) {
    _mm512_extstore_ps(addr,v2,_MM_DOWNCONV_PS_SINT8,0);
  }

  __forceinline void store16f_uint16(void* addr, const float16& v2) {
    _mm512_extstore_ps(addr,v2,_MM_DOWNCONV_PS_UINT16,0);
  }

  __forceinline void store4f_int8(void* addr, const float16& v1) {
    assert((unsigned long)addr % 4 == 0);
    _mm512_mask_extpackstorelo_ps(addr,0xf, v1, _MM_DOWNCONV_PS_SINT8 , 0);
  }
  
  __forceinline void store4f(void* addr, const float16& v1) {
    assert((unsigned long)addr % 16 == 0);
    _mm512_mask_extpackstorelo_ps(addr,0xf, v1, _MM_DOWNCONV_PS_NONE , 0);
  }

  __forceinline void store3f(void* addr, const float16& v1) {
    assert((unsigned long)addr % 16 == 0);
    _mm512_mask_extpackstorelo_ps(addr,0x7, v1, _MM_DOWNCONV_PS_NONE , 0);
  }

  __forceinline void store4f_nt(void* addr, const float16& v1) {
    assert((unsigned long)addr % 16 == 0);
    _mm512_mask_extpackstorelo_ps(addr,0xf, v1, _MM_DOWNCONV_PS_NONE , _MM_HINT_NT);
  }
  
  __forceinline void store16f_nt(void *__restrict__ ptr, const float16& a) {
    _mm512_storenr_ps(ptr,a);
  }
  
  __forceinline void store16f_ngo(void *__restrict__ ptr, const float16& a) {
    _mm512_storenrngo_ps(ptr,a);
  }

  __forceinline void store1f(void *addr, const float16& reg) {
    _mm512_mask_extpackstorelo_ps((float*)addr+0  ,(bool16)1, reg, _MM_DOWNCONV_PS_NONE, _MM_HINT_NONE);
  }
  
  __forceinline float16 gather16f(const bool16& mask, const float *const ptr, __m512i index, const _MM_INDEX_SCALE_ENUM scale) {
    float16 r = float16::undefined();
    return _mm512_mask_i32extgather_ps(r,mask,index,ptr,_MM_UPCONV_PS_NONE,scale,0);
  }
  
  __forceinline void scatter16f(const bool16& mask,const float *const ptr, const __m512i index,const float16 v, const _MM_INDEX_SCALE_ENUM scale) {
    _mm512_mask_i32extscatter_ps((void*)ptr,mask,index,v,_MM_DOWNCONV_PS_NONE,scale,0);
  }

  __forceinline float16 loadAOS4to16f(const float& x,const float& y, const float& z)
  {
    float16 f = float16::zero();
    f = select(0x1111,broadcast1to16f(&x),f);
    f = select(0x2222,broadcast1to16f(&y),f);
    f = select(0x4444,broadcast1to16f(&z),f);
    return f;
  }

  __forceinline float16 loadAOS4to16f(const unsigned int index,
				    const float16 &x,
				    const float16 &y,
				    const float16 &z)
  {
    float16 f = float16::zero();
    f = select(0x1111,broadcast1to16f((float*)&x + index),f);
    f = select(0x2222,broadcast1to16f((float*)&y + index),f);
    f = select(0x4444,broadcast1to16f((float*)&z + index),f);
    return f;
  }

  __forceinline float16 loadAOS4to16f(const unsigned int index,
				    const float16 &x,
				    const float16 &y,
				    const float16 &z,
				    const float16 &fill)
  {
    float16 f = fill;
    f = select(0x1111,broadcast1to16f((float*)&x + index),f);
    f = select(0x2222,broadcast1to16f((float*)&y + index),f);
    f = select(0x4444,broadcast1to16f((float*)&z + index),f);
    return f;
  }

  __forceinline float16 gather16f_4f_unalign(const void *__restrict__ const ptr0,
					   const void *__restrict__ const ptr1,
					   const void *__restrict__ const ptr2,
					   const void *__restrict__ const ptr3) 
  {
    float16 v = permute<0>(uload16f((float*)ptr3));
    v = align_shift_right<12>(v,permute<0>(uload16f((float*)ptr2)));
    v = align_shift_right<12>(v,permute<0>(uload16f((float*)ptr1)));
    v = align_shift_right<12>(v,permute<0>(uload16f((float*)ptr0)));
    return v;
  }


  __forceinline float16 rcp_safe( const float16& a ) { return select(a != float16::zero(),_mm512_rcp23_ps(a),float16(1E-10f)); };

  ////////////////////////////////////////////////////////////////////////////////
  /// Euclidian Space Operators
  ////////////////////////////////////////////////////////////////////////////////

__forceinline float16 lcross_zxy(const float16 &ao, const float16 &bo) {
    float16 ao_bo = bo * swizzle(ao,_MM_SWIZ_REG_DACB);
    ao_bo = msub231(ao_bo,ao,swizzle(bo,_MM_SWIZ_REG_DACB));
    return ao_bo;
  }
  
  __forceinline float16 ldot16_zxy(const float16 &a,const float16 &v0, const float16 &v1, const float16 &v2) 
  {
    float16 v = v0 * swizzle(a,_MM_SWIZ_REG_BBBB);
    v = madd231(v,v1,swizzle(a,_MM_SWIZ_REG_CCCC));
    v = madd231(v,v2,swizzle(a,_MM_SWIZ_REG_AAAA));
    return v;
  }
  
  __forceinline float16 ldot16_xyz(const float16 &a,const float16 &v0, const float16 &v1, const float16 &v2) 
  {
    float16 v = v0 * swizzle(a,_MM_SWIZ_REG_AAAA);
    v = madd231(v, v1,swizzle(a,_MM_SWIZ_REG_BBBB));
    v = madd231(v, v2,swizzle(a,_MM_SWIZ_REG_CCCC));
    return v;
  }
  
  __forceinline float16 lcross_xyz(const float16 &a, const float16 &b) 
  {
    float16 c = b * swizzle(a,_MM_SWIZ_REG_DACB);
    c = msub231(c,a,swizzle(b,_MM_SWIZ_REG_DACB));
    c = swizzle(c,_MM_SWIZ_REG_DACB);
    return c;
  }
  
  __forceinline float16 ldot3_xyz(const float16 &ao, const float16 &normal) 
  {
    float16 vv = ao * normal;
    vv = _mm512_add_ps(vv,swizzle(vv,_MM_SWIZ_REG_CDAB));
    vv = _mm512_add_ps(vv,swizzle(vv,_MM_SWIZ_REG_BADC));
    return vv;        
  }

  __forceinline float16 lsum3_xyz(const float16 &v) 
  {
    float16 vv = v;
    vv = _mm512_add_ps(vv,swizzle(vv,_MM_SWIZ_REG_CDAB));
    vv = _mm512_add_ps(vv,swizzle(vv,_MM_SWIZ_REG_BADC));
    return vv;        
  }

  __forceinline float16 ldot3_xyz(const bool16 &m_mask, const float16 &ao, const float16 &normal) 
  {
    float16 vv = _mm512_mask_mul_ps(ao,m_mask,ao,normal);
    vv = _mm512_add_ps(vv,swizzle(vv,_MM_SWIZ_REG_CDAB));
    vv = _mm512_add_ps(vv,swizzle(vv,_MM_SWIZ_REG_BADC));
    return vv;        
  }
  
  __forceinline float16 ldot3_zxy(const float16 &ao, const float16 &normal) 
  {
    float16 vv = ao * swizzle(normal,_MM_SWIZ_REG_DACB);
    vv = _mm512_add_ps(vv,swizzle(vv,_MM_SWIZ_REG_CDAB));
    vv = _mm512_add_ps(vv,swizzle(vv,_MM_SWIZ_REG_BADC));
    return vv;        
  }

  ////////////////////////////////////////////////////////////////////////////////
  /// Output Operators
  ////////////////////////////////////////////////////////////////////////////////
  
  __forceinline std::ostream &operator<<(std::ostream& cout, const float16& v)
  {
    cout << "<" << v[0];
    for (int i=1; i<16; i++) cout << ", " << v[i];
    cout << ">";
    return cout;
  }
}
