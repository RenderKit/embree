// ======================================================================== //
// Copyright 2009-2017 Intel Corporation                                    //
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
  /* 16-wide AVX-512 float type */
  template<>
    struct vfloat<16>
  {
    typedef vboolf16 Bool;
    typedef vint16   Int;
    typedef vfloat16 Float;

    enum  { size = 16 }; // number of SIMD elements
    union {              // data
      __m512 v; 
      float f[16];
      int i[16];
    };

    ////////////////////////////////////////////////////////////////////////////////
    /// Constructors, Assignment & Cast Operators
    ////////////////////////////////////////////////////////////////////////////////
        
    __forceinline vfloat() {}
    __forceinline vfloat(const vfloat16& t) { v = t; }
    __forceinline vfloat16& operator=(const vfloat16& f) { v = f.v; return *this; }

    __forceinline vfloat(const __m512& t) { v = t; }
    __forceinline operator __m512 () const { return v; }
    __forceinline operator __m256 () const { return _mm512_castps512_ps256(v); }
    
    __forceinline vfloat(const float& f) {
      v = _mm512_set_1to16_ps(f);
    }
    __forceinline vfloat(const float& a, const float& b, const float& c, const float& d) {
      v = _mm512_set_4to16_ps(a,b,c,d);  
    }

    __forceinline vfloat(const vfloat4 &i) {
      v = _mm512_broadcast_f32x4(i);
    }

    __forceinline vfloat(const vfloat4 &a, const vfloat4 &b, const vfloat4 &c, const vfloat4 &d) {
      v = _mm512_broadcast_f32x4(a);
      v = _mm512_insertf32x4(v, b, 1);
      v = _mm512_insertf32x4(v, c, 2);
      v = _mm512_insertf32x4(v, d, 3);
    }

    __forceinline vfloat(const vboolf16& mask, const vfloat4 &a, const vfloat4 &b) { 
      v = _mm512_broadcast_f32x4(a);
      v = _mm512_mask_broadcast_f32x4(v,mask,b);
    }

    __forceinline vfloat(const vfloat8 &i) {
      v = _mm512_castpd_ps(_mm512_broadcast_f64x4(_mm256_castps_pd(i)));
    }

    __forceinline vfloat(const vfloat8 &a, const vfloat8 &b) { // FIXME: optimize
      const vfloat aa = _mm512_castpd_ps(_mm512_broadcast_f64x4(_mm256_castps_pd(a)));
      const vfloat bb = _mm512_castpd_ps(_mm512_broadcast_f64x4(_mm256_castps_pd(b)));
      v = _mm512_mask_blend_ps(0xff, bb, aa);
    }

    /* WARNING: due to f64x4 the mask is considered as an 8bit mask */
    __forceinline vfloat(const vboolf16& mask, const vfloat8 &a, const vfloat8 &b) { 
      __m512d aa = _mm512_broadcast_f64x4(_mm256_castps_pd(a));
      aa = _mm512_mask_broadcast_f64x4(aa,mask,_mm256_castps_pd(b));
      v = _mm512_castpd_ps(aa);
    }
    
    __forceinline explicit vfloat(const __m512i& a) {
      v = _mm512_cvtepi32_ps(a);
    }

    ////////////////////////////////////////////////////////////////////////////////
    /// Loads and Stores
    ////////////////////////////////////////////////////////////////////////////////

    static __forceinline vfloat16 load (const void* const ptr) { return _mm512_load_ps((float*)ptr);  }
    static __forceinline vfloat16 loadu(const void* const ptr) { return _mm512_loadu_ps((float*)ptr); }

    static __forceinline vfloat16 load (const vboolf16& mask, const void* const ptr) { return _mm512_mask_load_ps (_mm512_setzero_ps(),mask,(float*)ptr); }
    static __forceinline vfloat16 loadu(const vboolf16& mask, const void* const ptr) { return _mm512_mask_loadu_ps(_mm512_setzero_ps(),mask,(float*)ptr); }

    static __forceinline void store (void* const ptr, const vfloat16& v) { _mm512_store_ps ((float*)ptr,v); }
    static __forceinline void storeu(void* const ptr, const vfloat16& v) { _mm512_storeu_ps((float*)ptr,v); }

    static __forceinline void store (const vboolf16& mask, void* ptr, const vfloat16& v) { _mm512_mask_store_ps ((float*)ptr,mask,v); }
    static __forceinline void storeu(const vboolf16& mask, void* ptr, const vfloat16& v) { _mm512_mask_storeu_ps((float*)ptr,mask,v); }

    static __forceinline void store_nt(void *__restrict__ ptr, const vfloat16& a) {
      _mm512_stream_ps(ptr,a);
    }

    static __forceinline vfloat16 broadcast(const float *const f) {
      return _mm512_set1_ps(*f);
    }

    static __forceinline vfloat16 compact(const vboolf16& mask, vfloat16 &v) {
      return _mm512_mask_compress_ps(v,mask,v);
    }
    static __forceinline vfloat16 compact(const vboolf16& mask, const vfloat16 &a, vfloat16 &b) {
      return _mm512_mask_compress_ps(a,mask,b);
    }

    static __forceinline vfloat16 loadu_compact(const vboolf16& mask, const void *const ptr) {
      return _mm512_mask_expandloadu_ps(vfloat16::undefined(),mask,(float*)ptr);
    }

    static __forceinline void storeu_compact(const vboolf16& mask, float *addr, const vfloat16 reg) {
      _mm512_mask_compressstoreu_ps(addr,mask,reg);
    }
    
    static __forceinline void storeu_compact_single(const vboolf16& mask, float * addr, const vfloat16 &reg) {
      //_mm512_mask_compressstoreu_ps(addr,mask,reg);
      *addr = _mm512_cvtss_f32(_mm512_mask_compress_ps(reg,mask,reg));
    }


    ////////////////////////////////////////////////////////////////////////////////
    /// Constants
    ////////////////////////////////////////////////////////////////////////////////
    
    __forceinline vfloat( ZeroTy   ) : v(_mm512_setzero_ps()) {}
    __forceinline vfloat( OneTy    ) : v(_mm512_set_1to16_ps(1.0f)) {}
    __forceinline vfloat( PosInfTy ) : v(_mm512_set_1to16_ps(pos_inf)) {}
    __forceinline vfloat( NegInfTy ) : v(_mm512_set_1to16_ps(neg_inf)) {}
    __forceinline vfloat( StepTy )   : v(_mm512_set_16to16_ps(15,14,13,12,11,10,9,8,7,6,5,4,3,2,1,0)) {}
    __forceinline vfloat( NaNTy    ) : v(_mm512_set_1to16_ps(nan)) {}

    __forceinline static vfloat16 undefined() { return _mm512_undefined(); }
    __forceinline static vfloat16 zero() { return _mm512_setzero_ps(); }
    __forceinline static vfloat16 one () { return _mm512_set_1to16_ps(1.0f); }
    __forceinline static vfloat16 ulp () { return _mm512_set_1to16_ps(embree::ulp); }
    __forceinline static vfloat16 inf () { return _mm512_set_1to16_ps((float)pos_inf); }
    __forceinline static vfloat16 minus_inf () { return _mm512_set_1to16_ps((float)neg_inf); }

    ////////////////////////////////////////////////////////////////////////////////
    /// Array Access
    ////////////////////////////////////////////////////////////////////////////////
    
    __forceinline       float& operator [](const size_t index)       { assert(index < 16); return f[index]; }
    __forceinline const float& operator [](const size_t index) const { assert(index < 16); return f[index]; }
  };

  ////////////////////////////////////////////////////////////////////////////////
  /// Unary Operators
  ////////////////////////////////////////////////////////////////////////////////

  __forceinline const vfloat16 asFloat   ( const __m512i&  a ) { return _mm512_castsi512_ps(a); }
  __forceinline const vfloat16 operator +( const vfloat16& a ) { return a; }
  __forceinline const vfloat16 operator -( const vfloat16& a ) { return _mm512_mul_ps(a,vfloat16(-1)); }
  __forceinline const vfloat16 abs       ( const vfloat16& a ) { return _mm512_abs_ps(a); }
  
  __forceinline const vfloat16 signmsk   ( const vfloat16& a ) { return _mm512_castsi512_ps(_mm512_and_epi32(_mm512_castps_si512(a),_mm512_set1_epi32(0x80000000))); }

  __forceinline const vfloat16 rcp(const vfloat16& a) {
#if defined(__AVX512ER__)
    return _mm512_rcp28_ps(a);
#else
    const vfloat16 r = _mm512_rcp14_ps(a.v);
    return _mm512_mul_ps(r, _mm512_fnmadd_ps(r, a, vfloat16(2.0f)));
#endif
  }

  __forceinline const vfloat16 sqr  ( const vfloat16& a ) { return _mm512_mul_ps(a,a); }
  __forceinline const vfloat16 sqrt ( const vfloat16& a ) { return _mm512_sqrt_ps(a); }
  __forceinline const vfloat16 rsqrt( const vfloat16& a ) { return _mm512_invsqrt_ps(a); }

  __forceinline vfloat16 exp(const vfloat16& a) { return _mm512_exp_ps(a); }
  __forceinline vfloat16 exp2(const vfloat16& a) { return _mm512_exp2_ps(a); }
  __forceinline vfloat16 pow(const vfloat16& a, vfloat16 b) { return _mm512_pow_ps(a,b); }
  
  __forceinline vfloat16 log(const vfloat16& a) { return _mm512_log_ps(a); }
  __forceinline vfloat16 log2(const vfloat16& a) { return _mm512_log2_ps(a); }
  __forceinline vfloat16 log10(const vfloat16& a) { return _mm512_log10_ps(a); }
  
  __forceinline vfloat16 sin(const vfloat16& a) { return _mm512_sin_ps(a); }
  __forceinline vfloat16 cos(const vfloat16& a) { return _mm512_cos_ps(a); }
  __forceinline vfloat16 tan(const vfloat16& a) { return _mm512_tan_ps(a); }
  
  __forceinline vfloat16 asin(const vfloat16& a) { return _mm512_asin_ps(a); }
  __forceinline vfloat16 acos(const vfloat16& a) { return _mm512_acos_ps(a); }
  __forceinline vfloat16 atan(const vfloat16& a) { return _mm512_atan_ps(a); }
  __forceinline vfloat16 atan2(const vfloat16& a, vfloat16 b) { return _mm512_atan2_ps(a,b); }
  
  __forceinline vfloat16 sinh(const vfloat16& a) { return _mm512_sinh_ps(a); }
  __forceinline vfloat16 cosh(const vfloat16& a) { return _mm512_cosh_ps(a); }
  __forceinline vfloat16 tanh(const vfloat16& a) { return _mm512_tan_ps(a); }
  
  __forceinline vfloat16 asinh(const vfloat16& a) { return _mm512_asinh_ps(a); }
  __forceinline vfloat16 acosh(const vfloat16& a) { return _mm512_acosh_ps(a); }
  __forceinline vfloat16 atanh(const vfloat16& a) { return _mm512_atanh_ps(a); }
  
  ////////////////////////////////////////////////////////////////////////////////
  /// Binary Operators
  ////////////////////////////////////////////////////////////////////////////////

  __forceinline const vfloat16 operator +( const vfloat16& a, const vfloat16& b ) { return _mm512_add_ps(a, b); }
  __forceinline const vfloat16 operator +( const vfloat16& a, const float&    b ) { return a + vfloat16(b); }
  __forceinline const vfloat16 operator +( const float&    a, const vfloat16& b ) { return vfloat16(a) + b; }

  __forceinline const vfloat16 operator -( const vfloat16& a, const vfloat16& b ) { return _mm512_sub_ps(a, b); }
  __forceinline const vfloat16 operator -( const vfloat16& a, const float&    b ) { return a - vfloat16(b); }
  __forceinline const vfloat16 operator -( const float&    a, const vfloat16& b ) { return vfloat16(a) - b; }

  __forceinline const vfloat16 operator *( const vfloat16& a, const vfloat16& b ) { return _mm512_mul_ps(a, b); }
  __forceinline const vfloat16 operator *( const vfloat16& a, const float&    b ) { return a * vfloat16(b); }
  __forceinline const vfloat16 operator *( const float&    a, const vfloat16& b ) { return vfloat16(a) * b; }

  __forceinline const vfloat16 operator /( const vfloat16& a, const vfloat16& b ) { return _mm512_div_ps(a,b); }
  __forceinline const vfloat16 operator /( const vfloat16& a, const float&    b ) { return a/vfloat16(b); }
  __forceinline const vfloat16 operator /( const float&    a, const vfloat16& b ) { return vfloat16(a)/b; }
  
  __forceinline const vfloat16 operator^(const vfloat16& a, const vfloat16& b) {
    return  _mm512_castsi512_ps(_mm512_xor_epi32(_mm512_castps_si512(a),_mm512_castps_si512(b))); 
  }
  
  __forceinline const vfloat16 min( const vfloat16& a, const vfloat16& b ) {
    return _mm512_min_ps(a,b); 
  }
  __forceinline const vfloat16 min( const vfloat16& a, const float& b ) {
    return _mm512_min_ps(a,vfloat16(b));
  }
  __forceinline const vfloat16 min( const float& a, const vfloat16& b ) {
    return _mm512_min_ps(vfloat16(a),b);
  }

  __forceinline const vfloat16 max( const vfloat16& a, const vfloat16& b ) {
    return _mm512_max_ps(a,b); 
  }
  __forceinline const vfloat16 max( const vfloat16& a, const float& b ) {
    return _mm512_max_ps(a,vfloat16(b));
  }
  __forceinline const vfloat16 max( const float& a, const vfloat16& b ) {
    return _mm512_max_ps(vfloat16(a),b);
  }

  __forceinline vfloat16 mask_add(const vboolf16& mask, const vfloat16& c, const vfloat16& a, const vfloat16& b) { return _mm512_mask_add_ps (c,mask,a,b); }
  __forceinline vfloat16 mask_min(const vboolf16& mask, const vfloat16& c, const vfloat16& a, const vfloat16& b) {
    return _mm512_mask_min_ps(c,mask,a,b); 
  }; 
  __forceinline vfloat16 mask_max(const vboolf16& mask, const vfloat16& c, const vfloat16& a, const vfloat16& b) {
    return _mm512_mask_max_ps(c,mask,a,b); 
  }; 

  __forceinline vfloat16 mini(const vfloat16& a, const vfloat16& b) {
#if !defined(__AVX512ER__) // SKX
    const vint16 ai = _mm512_castps_si512(a);
    const vint16 bi = _mm512_castps_si512(b);
    const vint16 ci = _mm512_min_epi32(ai,bi);
    return _mm512_castsi512_ps(ci);
#else // KNL
    return min(a,b);
#endif
  }

  __forceinline vfloat16 maxi(const vfloat16& a, const vfloat16& b) {
#if !defined(__AVX512ER__) // SKX
    const vint16 ai = _mm512_castps_si512(a);
    const vint16 bi = _mm512_castps_si512(b);
    const vint16 ci = _mm512_max_epi32(ai,bi);
    return _mm512_castsi512_ps(ci);
#else // KNL
    return max(a,b);
#endif
  }

  ////////////////////////////////////////////////////////////////////////////////
  /// Ternary Operators
  ////////////////////////////////////////////////////////////////////////////////

  __forceinline vfloat16 madd (const vfloat16& a, const vfloat16& b, const vfloat16& c) { return _mm512_fmadd_ps(a,b,c); }

  __forceinline vfloat16 msub (const vfloat16& a, const vfloat16& b, const vfloat16& c) { return _mm512_fmsub_ps(a,b,c); }
  __forceinline vfloat16 nmadd (const vfloat16& a, const vfloat16& b, const vfloat16& c) { return _mm512_fnmadd_ps(a,b,c); }
  __forceinline vfloat16 nmsub (const vfloat16& a, const vfloat16& b, const vfloat16& c) { return _mm512_fnmsub_ps(a,b,c); }

  __forceinline vfloat16 mask_msub (const vboolf16& mask,const vfloat16& a, const vfloat16& b, const vfloat16& c) { return _mm512_mask_fmsub_ps(a,mask,b,c); }
  
  __forceinline vfloat16 madd231 (const vfloat16& a, const vfloat16& b, const vfloat16& c) { return _mm512_fmadd_ps(c,b,a); }
  __forceinline vfloat16 msub213 (const vfloat16& a, const vfloat16& b, const vfloat16& c) { return _mm512_fmsub_ps(a,b,c); }
  __forceinline vfloat16 msub231 (const vfloat16& a, const vfloat16& b, const vfloat16& c) { return _mm512_fmsub_ps(c,b,a); }
  __forceinline vfloat16 msubr231(const vfloat16& a, const vfloat16& b, const vfloat16& c) { return _mm512_fnmadd_ps(c,b,a); }


  ////////////////////////////////////////////////////////////////////////////////
  /// Operators with rounding
  ////////////////////////////////////////////////////////////////////////////////

  __forceinline vfloat16 madd_round_down (const vfloat16& a, const vfloat16& b, const vfloat16& c) { return _mm512_fmadd_round_ps(a,b,c,_MM_FROUND_TO_NEG_INF); }

  __forceinline vfloat16 madd_round_up (const vfloat16& a, const vfloat16& b, const vfloat16& c) { return _mm512_fmadd_round_ps(a,b,c,_MM_FROUND_TO_POS_INF); }

  __forceinline vfloat16 mul_round_down (const vfloat16& a, const vfloat16& b) { return _mm512_mul_round_ps(a,b,_MM_FROUND_TO_NEG_INF); }
  __forceinline vfloat16 mul_round_up   (const vfloat16& a, const vfloat16& b) { return _mm512_mul_round_ps(a,b,_MM_FROUND_TO_POS_INF); }

  __forceinline vfloat16 add_round_down (const vfloat16& a, const vfloat16& b) { return _mm512_add_round_ps(a,b,_MM_FROUND_TO_NEG_INF); }
  __forceinline vfloat16 add_round_up   (const vfloat16& a, const vfloat16& b) { return _mm512_add_round_ps(a,b,_MM_FROUND_TO_POS_INF); }

  __forceinline vfloat16 sub_round_down (const vfloat16& a, const vfloat16& b) { return _mm512_sub_round_ps(a,b,_MM_FROUND_TO_NEG_INF); }
  __forceinline vfloat16 sub_round_up   (const vfloat16& a, const vfloat16& b) { return _mm512_sub_round_ps(a,b,_MM_FROUND_TO_POS_INF); }

  __forceinline vfloat16 div_round_down (const vfloat16& a, const vfloat16& b) { return _mm512_div_round_ps(a,b,_MM_FROUND_TO_NEG_INF); }
  __forceinline vfloat16 div_round_up   (const vfloat16& a, const vfloat16& b) { return _mm512_div_round_ps(a,b,_MM_FROUND_TO_POS_INF); }

  __forceinline vfloat16 mask_msub_round_down (const vboolf16& mask,const vfloat16& a, const vfloat16& b, const vfloat16& c) { return _mm512_mask_fmsub_round_ps(a,mask,b,c,_MM_FROUND_TO_NEG_INF); }
  __forceinline vfloat16 mask_msub_round_up   (const vboolf16& mask,const vfloat16& a, const vfloat16& b, const vfloat16& c) { return _mm512_mask_fmsub_round_ps(a,mask,b,c,_MM_FROUND_TO_POS_INF); }
  
  __forceinline vfloat16 mask_mul_round_down (const vboolf16& mask,const vfloat16& a, const vfloat16& b, const vfloat16& c) { return _mm512_mask_mul_round_ps(a,mask,b,c,_MM_FROUND_TO_NEG_INF); }
  __forceinline vfloat16 mask_mul_round_up   (const vboolf16& mask,const vfloat16& a, const vfloat16& b, const vfloat16& c) { return _mm512_mask_mul_round_ps(a,mask,b,c,_MM_FROUND_TO_POS_INF); }

  __forceinline vfloat16 mask_sub_round_down (const vboolf16& mask,const vfloat16& a, const vfloat16& b, const vfloat16& c) { return _mm512_mask_sub_round_ps(a,mask,b,c,_MM_FROUND_TO_NEG_INF); }
  __forceinline vfloat16 mask_sub_round_up   (const vboolf16& mask,const vfloat16& a, const vfloat16& b, const vfloat16& c) { return _mm512_mask_sub_round_ps(a,mask,b,c,_MM_FROUND_TO_POS_INF); }

  ////////////////////////////////////////////////////////////////////////////////
  /// Assignment Operators
  ////////////////////////////////////////////////////////////////////////////////

  __forceinline vfloat16& operator +=( vfloat16& a, const vfloat16& b ) { return a = a + b; }
  __forceinline vfloat16& operator +=( vfloat16& a, const float&    b ) { return a = a + b; }
  
  __forceinline vfloat16& operator -=( vfloat16& a, const vfloat16& b ) { return a = a - b; }
  __forceinline vfloat16& operator -=( vfloat16& a, const float&    b ) { return a = a - b; }
  
  __forceinline vfloat16& operator *=( vfloat16& a, const vfloat16& b ) { return a = a * b; }
  __forceinline vfloat16& operator *=( vfloat16& a, const float&    b ) { return a = a * b; }

  __forceinline vfloat16& operator /=( vfloat16& a, const vfloat16& b ) { return a = a / b; }
  __forceinline vfloat16& operator /=( vfloat16& a, const float&    b ) { return a = a / b; }

  ////////////////////////////////////////////////////////////////////////////////
  /// Comparison Operators + Select
  ////////////////////////////////////////////////////////////////////////////////

  __forceinline const vboolf16 operator ==( const vfloat16& a, const vfloat16& b ) { return _mm512_cmp_ps_mask(a,b,_MM_CMPINT_EQ); }
  __forceinline const vboolf16 operator ==( const vfloat16& a, const float&    b ) { return a == vfloat16(b); }
  __forceinline const vboolf16 operator ==( const float&    a, const vfloat16& b ) { return vfloat16(a) == b; }

  __forceinline const vboolf16 operator !=( const vfloat16& a, const vfloat16& b ) { return _mm512_cmp_ps_mask(a,b,_MM_CMPINT_NE); }
  __forceinline const vboolf16 operator !=( const vfloat16& a, const float&    b ) { return a != vfloat16(b); }
  __forceinline const vboolf16 operator !=( const float&    a, const vfloat16& b ) { return vfloat16(a) != b; }

  __forceinline const vboolf16 operator < ( const vfloat16& a, const vfloat16& b ) { return _mm512_cmp_ps_mask(a,b,_MM_CMPINT_LT); }
  __forceinline const vboolf16 operator < ( const vfloat16& a, const float&    b ) { return a <  vfloat16(b); }
  __forceinline const vboolf16 operator < ( const float&    a, const vfloat16& b ) { return vfloat16(a) <  b; }

  __forceinline const vboolf16 operator >=( const vfloat16& a, const vfloat16& b ) { return _mm512_cmp_ps_mask(a,b,_MM_CMPINT_GE); }
  __forceinline const vboolf16 operator >=( const vfloat16& a, const float&    b ) { return a >= vfloat16(b); }
  __forceinline const vboolf16 operator >=( const float&    a, const vfloat16& b ) { return vfloat16(a) >= b; }

  __forceinline const vboolf16 operator > ( const vfloat16& a, const vfloat16& b ) { return _mm512_cmp_ps_mask(a,b,_MM_CMPINT_GT); }
  __forceinline const vboolf16 operator > ( const vfloat16& a, const float&    b ) { return a >  vfloat16(b); }
  __forceinline const vboolf16 operator > ( const float&    a, const vfloat16& b ) { return vfloat16(a) >  b; }

  __forceinline const vboolf16 operator <=( const vfloat16& a, const vfloat16& b ) { return _mm512_cmp_ps_mask(a,b,_MM_CMPINT_LE); }
  __forceinline const vboolf16 operator <=( const vfloat16& a, const float&    b ) { return a <= vfloat16(b); }
  __forceinline const vboolf16 operator <=( const float&    a, const vfloat16& b ) { return vfloat16(a) <= b; }

  __forceinline vboolf16 eq(const vfloat16& a, const vfloat16& b) { return _mm512_cmp_ps_mask(a,b,_MM_CMPINT_EQ); }
  __forceinline vboolf16 ne(const vfloat16& a, const vfloat16& b) { return _mm512_cmp_ps_mask(a,b,_MM_CMPINT_NE); }
  __forceinline vboolf16 lt(const vfloat16& a, const vfloat16& b) { return _mm512_cmp_ps_mask(a,b,_MM_CMPINT_LT); }
  __forceinline vboolf16 ge(const vfloat16& a, const vfloat16& b) { return _mm512_cmp_ps_mask(a,b,_MM_CMPINT_GE); }
  __forceinline vboolf16 gt(const vfloat16& a, const vfloat16& b) { return _mm512_cmp_ps_mask(a,b,_MM_CMPINT_GT); }
  __forceinline vboolf16 le(const vfloat16& a, const vfloat16& b) { return _mm512_cmp_ps_mask(a,b,_MM_CMPINT_LE); }

  __forceinline vboolf16 eq(const vboolf16& mask, const vfloat16& a, const vfloat16& b) { return _mm512_mask_cmp_ps_mask(mask,a,b,_MM_CMPINT_EQ); }
  __forceinline vboolf16 ne(const vboolf16& mask, const vfloat16& a, const vfloat16& b) { return _mm512_mask_cmp_ps_mask(mask,a,b,_MM_CMPINT_NE); }
  __forceinline vboolf16 lt(const vboolf16& mask, const vfloat16& a, const vfloat16& b) { return _mm512_mask_cmp_ps_mask(mask,a,b,_MM_CMPINT_LT); }
  __forceinline vboolf16 ge(const vboolf16& mask, const vfloat16& a, const vfloat16& b) { return _mm512_mask_cmp_ps_mask(mask,a,b,_MM_CMPINT_GE); }
  __forceinline vboolf16 gt(const vboolf16& mask, const vfloat16& a, const vfloat16& b) { return _mm512_mask_cmp_ps_mask(mask,a,b,_MM_CMPINT_GT); }
  __forceinline vboolf16 le(const vboolf16& mask, const vfloat16& a, const vfloat16& b) { return _mm512_mask_cmp_ps_mask(mask,a,b,_MM_CMPINT_LE); }
  
  __forceinline const vfloat16 select( const vboolf16& s, const vfloat16& t, const vfloat16& f ) {
    return _mm512_mask_blend_ps(s, f, t);
  }

  __forceinline vfloat16 lerp(const vfloat16& a, const vfloat16& b, const vfloat16& t) {
    return madd(t,b-a,a);
  }

  __forceinline void xchg(vboolf16 m, vfloat16& a, vfloat16& b)
  {
    vfloat16 c = a;
    a = select(m,b,a);
    b = select(m,c,b); 
  }

  ////////////////////////////////////////////////////////////////////////////////
  /// Rounding Functions
  ////////////////////////////////////////////////////////////////////////////////
  
  __forceinline vfloat16 floor(const vfloat16& a) {
    return _mm512_floor_ps(a);
  }
  __forceinline vfloat16 ceil (const vfloat16& a) {
    return _mm512_ceil_ps(a);
  }
  __forceinline vfloat16 trunc(const vfloat16& a) {
    return _mm512_trunc_ps(a);
  } 

  __forceinline vint16 floori (const vfloat16& a) {
    return _mm512_cvt_roundps_epi32(a, _MM_FROUND_TO_NEG_INF | _MM_FROUND_NO_EXC);
  }


  __forceinline vfloat16 frac( const vfloat16& a ) { return a-trunc(a); }

  __forceinline const vfloat16 rcp_nr( const vfloat16& a ) {
    const vfloat16 ra = rcp(a);
    return (ra+ra) - (ra * a * ra);
  };

  ////////////////////////////////////////////////////////////////////////////////
  /// Movement/Shifting/Shuffling Functions
  ////////////////////////////////////////////////////////////////////////////////

  __forceinline vfloat16 shuffle(const vfloat16& x, _MM_SWIZZLE_ENUM perm32) {
    return _mm512_swizzle_ps(x,perm32); 
  }
  __forceinline vfloat16 shuffle4(const vfloat16& x, _MM_PERM_ENUM perm128) { return _mm512_permute4f128_ps(x, perm128); }
  
  template<int D, int C, int B, int A> __forceinline vfloat16 shuffle   (const vfloat16& v) {
    return _mm512_permute_ps(v,_MM_SHUF_PERM(D,C,B,A)); 
  }
  template<int A>                      __forceinline vfloat16 shuffle   (const vfloat16& x) { return shuffle<A,A,A,A>(v); }
  template<>                           __forceinline vfloat16 shuffle<0>(const vfloat16& x) { return shuffle(x,_MM_SWIZ_REG_AAAA); }
  template<>                           __forceinline vfloat16 shuffle<1>(const vfloat16& x) { return shuffle(x,_MM_SWIZ_REG_BBBB); }
  template<>                           __forceinline vfloat16 shuffle<2>(const vfloat16& x) { return shuffle(x,_MM_SWIZ_REG_CCCC); }
  template<>                           __forceinline vfloat16 shuffle<3>(const vfloat16& x) { return shuffle(x,_MM_SWIZ_REG_DDDD); }

  template<int D, int C, int B, int A> __forceinline vfloat16 shuffle4(const vfloat16& v) { return shuffle4(v,_MM_SHUF_PERM(D,C,B,A)); }
  template<int A>                      __forceinline vfloat16 shuffle4(const vfloat16& x) { return shuffle4<A,A,A,A>(x); }

  __forceinline vfloat16 shuffle(const vfloat16& x,_MM_PERM_ENUM perm128, _MM_SWIZZLE_ENUM perm32) { return shuffle(shuffle4(x,perm128),perm32); }
  
  __forceinline vfloat16 shuffle(const vboolf16& mask, vfloat16& v, const vfloat16& x,_MM_PERM_ENUM perm128, _MM_SWIZZLE_ENUM perm32)  {
    return _mm512_mask_swizzle_ps(_mm512_mask_permute4f128_ps(v,mask,x,perm128),mask,x,perm32);  
  }

  __forceinline vfloat16 swAAAA(const vfloat16 &x) {
    return shuffle(x,_MM_SWIZ_REG_AAAA);
  }

  __forceinline vfloat16 swBBBB(const vfloat16 &x) {
    return shuffle(x,_MM_SWIZ_REG_BBBB);
  }

  __forceinline vfloat16 swCCCC(const vfloat16 &x) {
    return shuffle(x,_MM_SWIZ_REG_CCCC);
  }

  __forceinline vfloat16 swDDDD(const vfloat16 &x) {
    return shuffle(x,_MM_SWIZ_REG_DDDD);
  }

  __forceinline vfloat16 permute(vfloat16 v,__m512i index)
  {
    return _mm512_castsi512_ps(_mm512_permutevar_epi32(index,_mm512_castps_si512(v)));  
  }

  __forceinline vfloat16 reverse(const vfloat16 &a)
  {
    return permute(a,_mm512_setr_epi32(15,14,13,12,11,10,9,8,7,6,5,4,3,2,1,0));
  }

  template<int i>
    __forceinline vfloat16 align_shift_right(const vfloat16 &a, const vfloat16 &b)
  {
    return _mm512_castsi512_ps(_mm512_alignr_epi32(_mm512_castps_si512(a),_mm512_castps_si512(b),i)); 
  };

  template<int i>
    __forceinline vfloat16 mask_align_shift_right(const vboolf16 &mask,vfloat16 &c,const vfloat16 &a, const vfloat16 &b)
  {
    return _mm512_castsi512_ps(_mm512_mask_alignr_epi32(_mm512_castps_si512(c),mask,_mm512_castps_si512(a),_mm512_castps_si512(b),i)); 
  };
 
  __forceinline vfloat16 shl1_zero_extend(const vfloat16 &a)
  {
    vfloat16 z = vfloat16::zero();
    return mask_align_shift_right<15>(0xfffe,z,a,a);
  }

  __forceinline vfloat16 shift_right_1( const vfloat16& x) 
  {
    __m512 t0 = _mm512_permute_ps(x,_MM_PERM_ADCB);
    __m512 t1 = _mm512_permute4f128_ps(t0,_MM_PERM_ADCB);
    __m512 y  = _mm512_mask_blend_ps(0x8888,t0,t1);
    return y;
  }

  __forceinline float toScalar(const vfloat16& a) { return _mm512_cvtss_f32(a); }


  template<int i> __forceinline const vfloat16 insert4(const vfloat16& a, const vfloat4& b) { return _mm512_insertf32x4(a, b, i); }

  template<int N, int i>
  vfloat<N> extractN(const vfloat16& v);

  template<> __forceinline vfloat4 extractN<4,0>(const vfloat16& v) { return _mm512_castps512_ps128(v);    }
  template<> __forceinline vfloat4 extractN<4,1>(const vfloat16& v) { return _mm512_extractf32x4_ps(v, 1); }
  template<> __forceinline vfloat4 extractN<4,2>(const vfloat16& v) { return _mm512_extractf32x4_ps(v, 2); }
  template<> __forceinline vfloat4 extractN<4,3>(const vfloat16& v) { return _mm512_extractf32x4_ps(v, 3); }

  template<> __forceinline vfloat8 extractN<8,0>(const vfloat16& v) { return _mm512_castps512_ps256(v);    }
  template<> __forceinline vfloat8 extractN<8,1>(const vfloat16& v) { return _mm512_extractf32x8_ps(v, 1); }

  template<int i> __forceinline vfloat4 extract4   (const vfloat16& v) { return _mm512_extractf32x4_ps(v, i); }
  template<>      __forceinline vfloat4 extract4<0>(const vfloat16& v) { return _mm512_castps512_ps128(v);    }

  template<int i> __forceinline vfloat8 extract8   (const vfloat16& v) { return _mm512_extractf32x8_ps(v, i); }
  template<>      __forceinline vfloat8 extract8<0>(const vfloat16& v) { return _mm512_castps512_ps256(v);    }

  ////////////////////////////////////////////////////////////////////////////////
  /// Reductions
  ////////////////////////////////////////////////////////////////////////////////

  __forceinline float reduce_add(const vfloat16 &a) { return _mm512_reduce_add_ps(a); }
  __forceinline float reduce_mul(const vfloat16 &a) { return _mm512_reduce_mul_ps(a); }
  __forceinline float reduce_min(const vfloat16 &a) { return _mm512_reduce_min_ps(a); }
  __forceinline float reduce_max(const vfloat16 &a) { return _mm512_reduce_max_ps(a); }

  __forceinline vfloat16 vreduce_min2(vfloat16 x) {                      return min(x,shuffle(x,_MM_SWIZ_REG_BADC)); }
  __forceinline vfloat16 vreduce_min4(vfloat16 x) { x = vreduce_min2(x); return min(x,shuffle(x,_MM_SWIZ_REG_CDAB)); }
  __forceinline vfloat16 vreduce_min8(vfloat16 x) { x = vreduce_min4(x); return min(x,shuffle4(x,_MM_SHUF_PERM(2,3,0,1))); }
  __forceinline vfloat16 vreduce_min (vfloat16 x) { x = vreduce_min8(x); return min(x,shuffle4(x,_MM_SHUF_PERM(1,0,3,2))); }

  __forceinline vfloat16 vreduce_max2(vfloat16 x) {                      return max(x,shuffle(x,_MM_SWIZ_REG_BADC)); }
  __forceinline vfloat16 vreduce_max4(vfloat16 x) { x = vreduce_max2(x); return max(x,shuffle(x,_MM_SWIZ_REG_CDAB)); }
  __forceinline vfloat16 vreduce_max8(vfloat16 x) { x = vreduce_max4(x); return max(x,shuffle4(x,_MM_SHUF_PERM(2,3,0,1))); }
  __forceinline vfloat16 vreduce_max (vfloat16 x) { x = vreduce_max8(x); return max(x,shuffle4(x,_MM_SHUF_PERM(1,0,3,2))); }

  __forceinline vfloat16 vreduce_add2(vfloat16 x) {                      return x + shuffle(x,_MM_SWIZ_REG_BADC); }
  __forceinline vfloat16 vreduce_add4(vfloat16 x) { x = vreduce_add2(x); return x + shuffle(x,_MM_SWIZ_REG_CDAB); }
  __forceinline vfloat16 vreduce_add8(vfloat16 x) { x = vreduce_add4(x); return x + shuffle4(x,_MM_SHUF_PERM(2,3,0,1)); }
  __forceinline vfloat16 vreduce_add (vfloat16 x) { x = vreduce_add8(x); return x + shuffle4(x,_MM_SHUF_PERM(1,0,3,2)); }

  __forceinline size_t select_min(const vfloat16& v) { 
    return __bsf(_mm512_kmov(_mm512_cmp_epi32_mask(_mm512_castps_si512(v),_mm512_castps_si512(vreduce_min(v)),_MM_CMPINT_EQ)));
  }

  __forceinline size_t select_max(const vfloat16& v) { 
    return __bsf(_mm512_kmov(_mm512_cmp_epi32_mask(_mm512_castps_si512(v),_mm512_castps_si512(vreduce_max(v)),_MM_CMPINT_EQ)));
  }

  __forceinline size_t select_min(const vboolf16& valid, const vfloat16& v) 
  { 
    const vfloat16 a = select(valid,v,vfloat16(pos_inf)); 
    const vbool16 valid_min = valid & (a == vreduce_min(a));
    return __bsf(movemask(any(valid_min) ? valid_min : valid)); 
  }

  __forceinline size_t select_max(const vboolf16& valid, const vfloat16& v) 
  { 
    const vfloat16 a = select(valid,v,vfloat16(neg_inf)); 
    const vbool16 valid_max = valid & (a == vreduce_max(a));
    return __bsf(movemask(any(valid_max) ? valid_max : valid)); 
  }
  
  __forceinline vfloat16 prefix_sum(const vfloat16& a) 
  {
    const vfloat16 z(zero);
    vfloat16 v = a;
    v = v + align_shift_right<16-1>(v,z);
    v = v + align_shift_right<16-2>(v,z);
    v = v + align_shift_right<16-4>(v,z);
    v = v + align_shift_right<16-8>(v,z);
    return v;  
  }

  __forceinline vfloat16 reverse_prefix_sum(const vfloat16& a) 
  {
    const vfloat16 z(zero);
    vfloat16 v = a;
    v = v + align_shift_right<1>(z,v);
    v = v + align_shift_right<2>(z,v);
    v = v + align_shift_right<4>(z,v);
    v = v + align_shift_right<8>(z,v);
    return v;  
  }

  __forceinline vfloat16 prefix_min(const vfloat16& a)
  {
    const vfloat16 z(pos_inf);
    vfloat16 v = a;
    v = min(v,align_shift_right<16-1>(v,z));
    v = min(v,align_shift_right<16-2>(v,z));
    v = min(v,align_shift_right<16-4>(v,z));
    v = min(v,align_shift_right<16-8>(v,z));
    return v;  
  }

  __forceinline vfloat16 prefix_max(const vfloat16& a)
  {
    const vfloat16 z(neg_inf);
    vfloat16 v = a;
    v = max(v,align_shift_right<16-1>(v,z));
    v = max(v,align_shift_right<16-2>(v,z));
    v = max(v,align_shift_right<16-4>(v,z));
    v = max(v,align_shift_right<16-8>(v,z));
    return v;  
  }


  __forceinline vfloat16 reverse_prefix_min(const vfloat16& a)
  {
    const vfloat16 z(pos_inf);
    vfloat16 v = a;
    v = min(v,align_shift_right<1>(z,v));
    v = min(v,align_shift_right<2>(z,v));
    v = min(v,align_shift_right<4>(z,v));
    v = min(v,align_shift_right<8>(z,v));
    return v;  
  }

  __forceinline vfloat16 reverse_prefix_max(const vfloat16& a)
  {
    const vfloat16 z(neg_inf);
    vfloat16 v = a;
    v = max(v,align_shift_right<1>(z,v));
    v = max(v,align_shift_right<2>(z,v));
    v = max(v,align_shift_right<4>(z,v));
    v = max(v,align_shift_right<8>(z,v));
    return v;  
  }

  __forceinline vfloat16 set_min4(vfloat16 x) {
    x = min(x,shuffle(x,_MM_SWIZ_REG_BADC));
    x = min(x,shuffle(x,_MM_SWIZ_REG_CDAB));
    return x;
  }

  __forceinline vfloat16 set_min_lanes(vfloat16 x) {
    x = min(x,_mm512_permute4f128_ps(x, _MM_PERM_CDAB));
    x = min(x,_mm512_permute4f128_ps(x, _MM_PERM_BADC));
    return x;
  }

  __forceinline vfloat16 set_max_lanes(vfloat16 x) {
    x = max(x,_mm512_permute4f128_ps(x, _MM_PERM_CDAB));
    x = max(x,_mm512_permute4f128_ps(x, _MM_PERM_BADC));
    return x;
  }

  __forceinline vfloat16 set_min16(vfloat16 x) {
    return set_min_lanes(set_min4(x));
  }



  ////////////////////////////////////////////////////////////////////////////////
  /// Memory load and store operations
  ////////////////////////////////////////////////////////////////////////////////

  
  __forceinline vfloat16 broadcast4to16f(const void *f) {
    return _mm512_extload_ps(f,_MM_UPCONV_PS_NONE,_MM_BROADCAST_4X16,0);  
  }

    

  __forceinline vfloat16 gather16f_4f(const float *__restrict__ const ptr0,
                                      const float *__restrict__ const ptr1,
                                      const float *__restrict__ const ptr2,
                                      const float *__restrict__ const ptr3) 
  {
    vfloat16 v = broadcast4to16f(ptr0);
    v = select((vboolf16)0x00f0,broadcast4to16f(ptr1),v);
    v = select((vboolf16)0x0f00,broadcast4to16f(ptr2),v);
    v = select((vboolf16)0xf000,broadcast4to16f(ptr3),v);
    return v;
  }
  
  __forceinline vfloat16 gather16f_4f(const vfloat16& v0,
                                      const vfloat16& v1,
                                      const vfloat16& v2,
                                      const vfloat16& v3)
  {
    vfloat16 v = v0;
    v = select((vboolf16)0xf0  ,v1,v);
    v = select((vboolf16)0xf00 ,v2,v);
    v = select((vboolf16)0xf000,v3,v);
    return v;
  }

  __forceinline vfloat16 gather_4f_zlc(const vint16 &v_mask,
                                       const void *__restrict__ const ptr0,
                                       const void *__restrict__ const ptr1,
                                       const void *__restrict__ const ptr2,
                                       const void *__restrict__ const ptr3)
  {
    const vboolf16 m_00f0 = 0x00f0;
    const vboolf16 m_0f00 = 0x0f00;
    const vboolf16 m_f000 = 0xf000;
    
    vint16 v = v_mask &  broadcast4to16i((const int*)ptr0);
    v = mask_and(m_00f0,v,v_mask, broadcast4to16i((const int*)ptr1));
    v = mask_and(m_0f00,v,v_mask, broadcast4to16i((const int*)ptr2));
    v = mask_and(m_f000,v,v_mask, broadcast4to16i((const int*)ptr3));
    return asFloat(v);
  }

  __forceinline vfloat16 gather_2f_zlc(const vint16 &v_mask,
                                       const vboolf16 &mask,
                                       const void *__restrict__ const ptr0,
                                       const void *__restrict__ const ptr1)
  {
    vint16 v = v_mask &  broadcast4to16i((const int*)ptr0);
    v = mask_and(mask,v,v_mask, broadcast4to16i((const int*)ptr1));
    return asFloat(v);
  }


  __forceinline vfloat16 gather16f_4f_align(const void *__restrict__ const ptr0,
                                            const void *__restrict__ const ptr1,
                                            const void *__restrict__ const ptr2,
                                            const void *__restrict__ const ptr3)
  {
    vfloat16 v = broadcast4to16f(ptr3);
    v = align_shift_right<12>(v,broadcast4to16f(ptr2));
    v = align_shift_right<12>(v,broadcast4to16f(ptr1));
    v = align_shift_right<12>(v,broadcast4to16f(ptr0));
    return v;
  }


  __forceinline vfloat16 gather16f_4f_align(const vfloat16& v0,
                                            const vfloat16& v1,
                                            const vfloat16& v2,
                                            const vfloat16& v3)
  {
    vfloat16 v = v3;
    v = align_shift_right<12>(v,v2);
    v = align_shift_right<12>(v,v1);
    v = align_shift_right<12>(v,v0);
    return v;
  }

  __forceinline vfloat16 gather_4f_zlc_align(const vint16 &v_mask,
                                             const void *__restrict__ const ptr0,
                                             const void *__restrict__ const ptr1,
                                             const void *__restrict__ const ptr2,
                                             const void *__restrict__ const ptr3)
  {
    vfloat16 v = gather16f_4f_align(ptr0,ptr1,ptr2,ptr3);
    return asFloat(asInt(v) & v_mask);
  }

  __forceinline void compactustore16f_low(const vboolf16& mask, float * addr, const vfloat16 &reg) {
    _mm512_mask_compressstoreu_ps(addr,mask,reg);
  }

  __forceinline vfloat16 gather16f(const vboolf16& mask, const float *const ptr, __m512i index, const int scale= 4 ) {
    vfloat16 r = vfloat16::undefined();
    return _mm512_mask_i32gather_ps(r,mask,index,ptr,scale);
  }
  
  __forceinline void scatter16f(const vboolf16& mask,const float *const ptr, const __m512i index,const vfloat16 v, const int scale = 4) {
    _mm512_mask_i32scatter_ps((void*)ptr,mask,index,v,scale);
  }

  __forceinline vfloat16 loadAOS4to16f(const float& x,const float& y, const float& z)
  {
    vfloat16 f = vfloat16::zero();
    f = select(0x1111,vfloat16::broadcast(&x),f);
    f = select(0x2222,vfloat16::broadcast(&y),f);
    f = select(0x4444,vfloat16::broadcast(&z),f);
    return f;
  }

  __forceinline vfloat16 loadAOS4to16f(const unsigned int index,
                                       const vfloat16 &x,
                                       const vfloat16 &y,
                                       const vfloat16 &z)
  {
    vfloat16 f = vfloat16::zero();
    f = select(0x1111,vfloat16::broadcast((float*)&x + index),f);
    f = select(0x2222,vfloat16::broadcast((float*)&y + index),f);
    f = select(0x4444,vfloat16::broadcast((float*)&z + index),f);
    return f;
  }

  __forceinline vfloat16 loadAOS4to16f(const unsigned int index,
                                       const vfloat16 &x,
                                       const vfloat16 &y,
                                       const vfloat16 &z,
                                       const vfloat16 &fill)
  {
    vfloat16 f = fill;
    f = select(0x1111,vfloat16::broadcast((float*)&x + index),f);
    f = select(0x2222,vfloat16::broadcast((float*)&y + index),f);
    f = select(0x4444,vfloat16::broadcast((float*)&z + index),f);
    return f;
  }

  __forceinline vfloat16 gather16f_4f_unalign(const void *__restrict__ const ptr0,
                                              const void *__restrict__ const ptr1,
                                              const void *__restrict__ const ptr2,
                                              const void *__restrict__ const ptr3)
  {
    vfloat16 v = shuffle4<0>(vfloat16::loadu((float*)ptr3));
    v = align_shift_right<12>(v,shuffle4<0>(vfloat16::loadu((float*)ptr2)));
    v = align_shift_right<12>(v,shuffle4<0>(vfloat16::loadu((float*)ptr1)));
    v = align_shift_right<12>(v,shuffle4<0>(vfloat16::loadu((float*)ptr0)));
    return v;
  }


  __forceinline vfloat16 rcp_safe( const vfloat16& a ) { 
    return rcp(select(a != vfloat16::zero(),a,vfloat16(min_rcp_input))); 
  }

  ////////////////////////////////////////////////////////////////////////////////
  /// Euclidian Space Operators
  ////////////////////////////////////////////////////////////////////////////////

  __forceinline vfloat16 lcross_zxy(const vfloat16 &ao, const vfloat16 &bo) {
    vfloat16 ao_bo = bo * shuffle(ao,_MM_SWIZ_REG_DACB);
    ao_bo = msub231(ao_bo,ao,shuffle(bo,_MM_SWIZ_REG_DACB));
    return ao_bo;
  }
  
  __forceinline vfloat16 ldot16_zxy(const vfloat16 &a,const vfloat16 &v0, const vfloat16 &v1, const vfloat16 &v2)
  {
    vfloat16 v = v0 * shuffle(a,_MM_SWIZ_REG_BBBB);
    v = madd231(v,v1,shuffle(a,_MM_SWIZ_REG_CCCC));
    v = madd231(v,v2,shuffle(a,_MM_SWIZ_REG_AAAA));
    return v;
  }
  
  __forceinline vfloat16 ldot16_xyz(const vfloat16 &a,const vfloat16 &v0, const vfloat16 &v1, const vfloat16 &v2)
  {
    vfloat16 v = v0 * shuffle(a,_MM_SWIZ_REG_AAAA);
    v = madd231(v, v1,shuffle(a,_MM_SWIZ_REG_BBBB));
    v = madd231(v, v2,shuffle(a,_MM_SWIZ_REG_CCCC));
    return v;
  }
  
  __forceinline vfloat16 lcross_xyz(const vfloat16 &a, const vfloat16 &b)
  {
    vfloat16 c = b * shuffle(a,_MM_SWIZ_REG_DACB);
    c = msub231(c,a,shuffle(b,_MM_SWIZ_REG_DACB));
    c = shuffle(c,_MM_SWIZ_REG_DACB);
    return c;
  }
  
  __forceinline vfloat16 ldot3_xyz(const vfloat16 &ao, const vfloat16 &normal)
  {
    vfloat16 vv = ao * normal;
    vv = _mm512_add_ps(vv,shuffle(vv,_MM_SWIZ_REG_CDAB));
    vv = _mm512_add_ps(vv,shuffle(vv,_MM_SWIZ_REG_BADC));
    return vv;        
  }

  __forceinline vfloat16 lsum3_xyz(const vfloat16 &v)
  {
    vfloat16 vv = v;
    vv = _mm512_add_ps(vv,shuffle(vv,_MM_SWIZ_REG_CDAB));
    vv = _mm512_add_ps(vv,shuffle(vv,_MM_SWIZ_REG_BADC));
    return vv;        
  }

  __forceinline vfloat16 ldot3_xyz(const vboolf16 &m_mask, const vfloat16 &ao, const vfloat16 &normal)
  {
    vfloat16 vv = _mm512_mask_mul_ps(ao,m_mask,ao,normal);
    vv = _mm512_add_ps(vv,shuffle(vv,_MM_SWIZ_REG_CDAB));
    vv = _mm512_add_ps(vv,shuffle(vv,_MM_SWIZ_REG_BADC));
    return vv;        
  }
  
  __forceinline vfloat16 ldot3_zxy(const vfloat16 &ao, const vfloat16 &normal)
  {
    vfloat16 vv = ao * shuffle(normal,_MM_SWIZ_REG_DACB);
    vv = _mm512_add_ps(vv,shuffle(vv,_MM_SWIZ_REG_CDAB));
    vv = _mm512_add_ps(vv,shuffle(vv,_MM_SWIZ_REG_BADC));
    return vv;        
  }

  ////////////////////////////////////////////////////////////////////////////////
  /// Output Operators
  ////////////////////////////////////////////////////////////////////////////////
  
  __forceinline std::ostream &operator<<(std::ostream& cout, const vfloat16& v)
  {
    cout << "<" << v[0];
    for (int i=1; i<16; i++) cout << ", " << v[i];
    cout << ">";
    return cout;
  }
}
