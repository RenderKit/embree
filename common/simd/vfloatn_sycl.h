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

namespace embree
{
  /* N-wide SSE float type */
  template<int N>
  struct vfloat
  {
    //ALIGNED_STRUCT_(16);
    
    typedef vboolf<N> Bool;
    typedef vint<N>   Int;
    typedef vfloat<N> Float;
    
    enum  { size = N };                        // number of SIMD elements
    //union { __m128 v; float f[N]; int i[N]; }; // data
    float v;

    ////////////////////////////////////////////////////////////////////////////////
    /// Constructors, Assignment & Cast Operators
    ////////////////////////////////////////////////////////////////////////////////
    
    __forceinline vfloat() {}
    //__forceinline vfloat(const vfloat<N>& other) { v = other.v; }
    //__forceinline vfloat<N>& operator =(const vfloat<N>& other) { v = other.v; return *this; }

    //__forceinline vfloat(__m128 a) : v(a) {}
    //__forceinline operator const __m128&() const { return v; }
    //__forceinline operator       __m128&()       { return v; }

    __forceinline vfloat(float a) : v(a) {}
    __forceinline vfloat(float a, float b, float c, float d)
    {
      const uint lid = __spirv_BuiltInSubgroupLocalInvocationId;
      if (lid == 0) v = a;
      if (lid == 1) v = b;
      if (lid == 2) v = c;
      if (lid == 3) v = d;
    }

    __forceinline explicit vfloat(const vint<N>& a) : v((float)a.v) {}
  
    /*__forceinline explicit vfloat(const vuintN& x) {
      const __m128i a   = _mm_and_si128(x,_mm_set1_epi32(0x7FFFFFFF));
      const __m128i b   = _mm_and_si128(_mm_srai_epi32(x,31),_mm_set1_epi32(0x4F000000)); //0x4F000000 = 2^31 
      const __m128  af  = _mm_cvtepi32_ps(a);
      const __m128  bf  = _mm_castsi128_ps(b);  
      v  = _mm_add_ps(af,bf);
      }*/

    vfloat<N> fix_upper(float c) const {
      if (N == 16) return c;
      return vfloat<N>(__spirv_BuiltInSubgroupLocalInvocationId < N ? v : c);
    }

    static uint sgid() {
      if (N == 16) return __spirv_BuiltInSubgroupLocalInvocationId;
      return cl::sycl::min(__spirv_BuiltInSubgroupLocalInvocationId,unsigned(N-1));
    }

    ////////////////////////////////////////////////////////////////////////////////
    /// Constants
    ////////////////////////////////////////////////////////////////////////////////

    __forceinline vfloat(ZeroTy)   : v(0.0f) {}
    __forceinline vfloat(OneTy)    : v(1.0f) {}
    __forceinline vfloat(PosInfTy) : v(pos_inf) {}
    __forceinline vfloat(NegInfTy) : v(neg_inf) {}
    __forceinline vfloat(StepTy)   : v(__spirv_BuiltInSubgroupLocalInvocationId) {}
    //__forceinline vfloat(NaNTy)    : v(_mm_set1_ps(nan)) {}
    //__forceinline vfloat(UndefinedTy) : v(_mm_undefined_ps()) {}

    ////////////////////////////////////////////////////////////////////////////////
    /// Loads and Stores
    ////////////////////////////////////////////////////////////////////////////////

    static __forceinline vfloat<N> load (const void* a)
    {
      if (N == 16) {
        return vfloat<N>(__spirv_SubgroupBlockReadINTEL<float>((const __attribute__((opencl_global)) uint32_t*) a));
      } else {
        return ((float*)a)[vfloat<N>::sgid()];
      }
    }
    
    static __forceinline vfloat<N> loadu(const void* a) {
      return ((float*)a)[vfloat<N>::sgid()];
    }

    static __forceinline void store (void* ptr, const vfloat<N>& v)
    {
      if (N == 16) {
	__spirv_SubgroupBlockWriteINTEL<uint32_t>((__attribute__((opencl_global)) uint32_t*) ptr, *(uint32_t*)&v.v);
      } else {
        const uint lid = __spirv_BuiltInSubgroupLocalInvocationId;
        if (lid < N) ((float*)ptr)[lid] = v.v;
      }
    }
    
    static __forceinline void storeu(void* ptr, const vfloat<N>& v) {
      const uint lid = __spirv_BuiltInSubgroupLocalInvocationId;
      if (lid < N) ((float*)ptr)[lid] = v.v;
    }

    //static __forceinline vfloat<N> compact(const vboolf<N>& mask, vfloat<N> &v) {
    //  return _mm_mask_compress_ps(v, mask, v);
    //}
    //static __forceinline vfloat<N> compact(const vboolf<N>& mask, vfloat<N> &a, const vfloat<N>& b) {
    //  return _mm_mask_compress_ps(a, mask, b);
    //}

    static __forceinline vfloat<N> load (const vboolf<N>& mask, const void* ptr) {
      if (mask.v) return loadu(ptr); else return vfloat<N>(0.0f);
    }
    static __forceinline vfloat<N> loadu(const vboolf<N>& mask, const void* ptr) {
      if (mask.v) return loadu(ptr); else return vfloat<N>(0.0f);
    }

    static __forceinline void store (const vboolf<N>& mask, void* ptr, const vfloat<N>& v) {
      if (mask.v) storeu(ptr,v);
    }
    static __forceinline void storeu(const vboolf<N>& mask, void* ptr, const vfloat<N>& v) {
      if (mask.v) storeu(ptr,v);
    }

    static __forceinline vfloat<N> broadcast(const void* a) {
      return vfloat<N>(*(float*)a);
    }

    static __forceinline vfloat<N> load_nt (const float* ptr) {
      return load(ptr);
    }

    static __forceinline void store_nt(void* ptr, const vfloat<N>& v) {
      store(ptr,v);
    }

    //static __forceinline vfloat<N> load(const char* ptr) {
    //  return _mm_cvtepi32_ps(_mm_cvtepi8_epi32(_mm_loadu_si128((__m128i*)ptr)));
    //}

    //static __forceinline vfloat<N> load(const unsigned char* ptr) {
    //  return _mm_cvtepi32_ps(_mm_cvtepu8_epi32(_mm_loadu_si128((__m128i*)ptr)));
    //}

    //static __forceinline vfloat<N> load(const short* ptr) {
    //  return _mm_cvtepi32_ps(_mm_cvtepi16_epi32(_mm_loadu_si128((__m128i*)ptr)));
    //}

    //static __forceinline vfloat<N> load(const unsigned short* ptr) {
    //  return _mm_mul_ps(vfloat<N>(vint<N>::load(ptr)),vfloat<N>(1.0f/65535.0f));
    //}

    template<int scale = 4>
    static __forceinline vfloat<N> gather(const float* ptr, const vint<N>& index) {
      return *(float*)((char*)ptr + scale*index.v);
    }

    template<int scale = 4>
      static __forceinline vfloat<N> gather(const vboolf<N>& mask, const float* ptr, const vint<N>& index) {
      if (mask.v) return gather<scale>(ptr,index);
      else        return vfloat<N>(0.0f);
    }

    template<int scale = 4>
      static __forceinline void scatter(void* ptr, const vint<N>& index, const vfloat<N>& v) {
      *(float*) ((char*)ptr + scale*index.v) = v.v;
    }

    template<int scale = 4>
    static __forceinline void scatter(const vboolf<N>& mask, void* ptr, const vint<N>& index, const vfloat<N>& v) {
      if (mask.v) scatter<scale>(ptr,index,v);
    }

    static __forceinline void store(const vboolf<N>& mask, char* ptr, const vint<N>& ofs, const vfloat<N>& v) {
      scatter<1>(mask,ptr,ofs,v);
    }
    static __forceinline void store(const vboolf<N>& mask, float* ptr, const vint<N>& ofs, const vfloat<N>& v) {
      scatter<4>(mask,ptr,ofs,v);
    }
    
    ////////////////////////////////////////////////////////////////////////////////
    /// Array Access
    ////////////////////////////////////////////////////////////////////////////////

    __forceinline const float operator [](size_t index) const {
      assert(index < 4);
      return __spirv_GroupBroadcast<float>(__spv::Scope::Subgroup, v, index);
    }
    __forceinline float operator [](size_t index) {
      assert(index < 4);
      return __spirv_GroupBroadcast<float>(__spv::Scope::Subgroup, v, index);
    }

    friend __forceinline vfloat<N> select(const vboolf<N>& m, const vfloat<N>& t, const vfloat<N>& f) {
      return m.v ? t : f;
    }
  };


  ////////////////////////////////////////////////////////////////////////////////
  /// Unary Operators
  ////////////////////////////////////////////////////////////////////////////////

  template<int N> __forceinline vfloat<N> asFloat(const vint<N>&   a) { return vfloat<N>(__builtin_bit_cast(float,a.v)); }
  template<int N> __forceinline vint<N>   asInt  (const vfloat<N>& a) { return vint<N>(__builtin_bit_cast(int,a.v)); }
  //template<int N> __forceinline vuintN  asUInt (const vfloat<N>& a) { return _mm_castps_si128(a); }

  template<int N> __forceinline vfloat<N> operator +(const vfloat<N>& a) { return a; }
  template<int N> __forceinline vfloat<N> operator -(const vfloat<N>& a) { return vfloat<N>(-a.v); }

  template<int N> __forceinline vfloat<N> abs(const vfloat<N>& a) { return vfloat<N>(cl::sycl::fabs(a.v)); }
  template<int N> __forceinline vfloat<N> sign(const vfloat<N>& a) { return vfloat<N>(cl::sycl::sign(a.v)); }
  template<int N> __forceinline vfloat<N> signmsk(const vfloat<N>& a) { return asFloat(asInt(a.v) & 0x80000000); }
  
  template<int N> __forceinline vfloat<N> rcp(const vfloat<N>& a) { return vfloat<N>(cl::sycl::native::recip(a.v)); } 
  //template<int N> __forceinline vfloat<N> rcp(const vfloat<N>& a) { return vfloat<N>(__sycl_std::__invoke_native_recip<float>(a.v)); }

  template<int N> __forceinline vfloat<N> sqr (const vfloat<N>& a) { return vfloat<N>(a.v*a.v); }
  template<int N> __forceinline vfloat<N> sqrt(const vfloat<N>& a) { return vfloat<N>(cl::sycl::sqrt(a.v)); }

  template<int N> __forceinline vfloat<N> rsqrt(const vfloat<N>& a) { return vfloat<N>(cl::sycl::rsqrt(a.v)); }

  template<int N> __forceinline vboolf<N> isnan(const vfloat<N>& a) { return vboolf<N>(cl::sycl::isnan(a.v)); }

  ////////////////////////////////////////////////////////////////////////////////
  /// Binary Operators
  ////////////////////////////////////////////////////////////////////////////////

  template<int N> __forceinline vfloat<N> operator +(const vfloat<N>& a, const vfloat<N>& b) { return vfloat<N>(a.v+b.v); }
  template<int N> __forceinline vfloat<N> operator +(const vfloat<N>& a, float          b) { return a + vfloat<N>(b); }
  template<int N> __forceinline vfloat<N> operator +(float          a, const vfloat<N>& b) { return vfloat<N>(a) + b; }

  template<int N> __forceinline vfloat<N> operator -(const vfloat<N>& a, const vfloat<N>& b) { return vfloat<N>(a.v-b.v); }
  template<int N> __forceinline vfloat<N> operator -(const vfloat<N>& a, float          b) { return a - vfloat<N>(b); }
  template<int N> __forceinline vfloat<N> operator -(float          a, const vfloat<N>& b) { return vfloat<N>(a) - b; }

  template<int N> __forceinline vfloat<N> operator *(const vfloat<N>& a, const vfloat<N>& b) { return vfloat<N>(a.v*b.v); }
  template<int N> __forceinline vfloat<N> operator *(const vfloat<N>& a, float          b) { return a * vfloat<N>(b); }
  template<int N> __forceinline vfloat<N> operator *(float          a, const vfloat<N>& b) { return vfloat<N>(a) * b; }

  template<int N> __forceinline vfloat<N> operator /(const vfloat<N>& a, const vfloat<N>& b) { return vfloat<N>(a.v/b.v); }
  template<int N> __forceinline vfloat<N> operator /(const vfloat<N>& a, float          b) { return a/vfloat<N>(b); }
  template<int N> __forceinline vfloat<N> operator /(float          a, const vfloat<N>& b) { return vfloat<N>(a)/b; }

  template<int N> __forceinline vfloat<N> operator ^(const vfloat<N>& a, const vfloat<N>& b) { return vfloat<N>(asFloat(asInt(a.v) ^ asInt(b.v))); }
  template<int N> __forceinline vfloat<N> operator ^(const vfloat<N>& a, const vint<N>&   b) { return vfloat<N>(asFloat(asInt(a.v) ^ b.v)); }

  template<int N> __forceinline vfloat<N> min(const vfloat<N>& a, const vfloat<N>& b) { return vfloat<N>(cl::sycl::fmin(a.v,b.v)); }
  template<int N> __forceinline vfloat<N> min(const vfloat<N>& a, float          b) { return min(a,vfloat<N>(b)); }
  template<int N> __forceinline vfloat<N> min(float          a, const vfloat<N>& b) { return min(vfloat<N>(a),b); }

  template<int N> __forceinline vfloat<N> max(const vfloat<N>& a, const vfloat<N>& b) { return vfloat<N>(cl::sycl::fmax(a.v,b.v)); }
  template<int N> __forceinline vfloat<N> max(const vfloat<N>& a, float          b) { return max(a,vfloat<N>(b)); }
  template<int N> __forceinline vfloat<N> max(float          a, const vfloat<N>& b) { return max(vfloat<N>(a),b); }

  ////////////////////////////////////////////////////////////////////////////////
  /// Ternary Operators
  ////////////////////////////////////////////////////////////////////////////////

  template<int N> __forceinline vfloat<N> madd (const vfloat<N>& a, const vfloat<N>& b, const vfloat<N>& c) { return vfloat<N>(+cl::sycl::fma(+a.v,b.v,+c.v)); }
  template<int N> __forceinline vfloat<N> msub (const vfloat<N>& a, const vfloat<N>& b, const vfloat<N>& c) { return vfloat<N>(+cl::sycl::fma(+a.v,b.v,-c.v)); }
  template<int N> __forceinline vfloat<N> nmadd(const vfloat<N>& a, const vfloat<N>& b, const vfloat<N>& c) { return vfloat<N>(+cl::sycl::fma(-a.v,b.v,+c.v));}
  template<int N> __forceinline vfloat<N> nmsub(const vfloat<N>& a, const vfloat<N>& b, const vfloat<N>& c) { return vfloat<N>(-cl::sycl::fma(+a.v,b.v,+c.v)); }

  ////////////////////////////////////////////////////////////////////////////////
  /// Assignment Operators
  ////////////////////////////////////////////////////////////////////////////////

  template<int N> __forceinline vfloat<N>& operator +=(vfloat<N>& a, const vfloat<N>& b) { return a = a + b; }
  template<int N> __forceinline vfloat<N>& operator +=(vfloat<N>& a, float          b) { return a = a + b; }

  template<int N> __forceinline vfloat<N>& operator -=(vfloat<N>& a, const vfloat<N>& b) { return a = a - b; }
  template<int N> __forceinline vfloat<N>& operator -=(vfloat<N>& a, float          b) { return a = a - b; }

  template<int N> __forceinline vfloat<N>& operator *=(vfloat<N>& a, const vfloat<N>& b) { return a = a * b; }
  template<int N> __forceinline vfloat<N>& operator *=(vfloat<N>& a, float          b) { return a = a * b; }

  template<int N> __forceinline vfloat<N>& operator /=(vfloat<N>& a, const vfloat<N>& b) { return a = a / b; }
  template<int N> __forceinline vfloat<N>& operator /=(vfloat<N>& a, float          b) { return a = a / b; }

  ////////////////////////////////////////////////////////////////////////////////
  /// Comparison Operators + Select
  ////////////////////////////////////////////////////////////////////////////////

  template<int N> __forceinline vboolf<N> operator ==(const vfloat<N>& a, const vfloat<N>& b) { return vboolf<N>(a.v == b.v); }
  template<int N> __forceinline vboolf<N> operator !=(const vfloat<N>& a, const vfloat<N>& b) { return vboolf<N>(a.v != b.v); }
  template<int N> __forceinline vboolf<N> operator < (const vfloat<N>& a, const vfloat<N>& b) { return vboolf<N>(a.v <  b.v); }
  template<int N> __forceinline vboolf<N> operator >=(const vfloat<N>& a, const vfloat<N>& b) { return vboolf<N>(a.v >= b.v); }
  template<int N> __forceinline vboolf<N> operator > (const vfloat<N>& a, const vfloat<N>& b) { return vboolf<N>(a.v >  b.v); }
  template<int N> __forceinline vboolf<N> operator <=(const vfloat<N>& a, const vfloat<N>& b) { return vboolf<N>(a.v <= b.v); }

  template<int N> __forceinline vboolf<N> operator ==(const vfloat<N>& a, float          b) { return a == vfloat<N>(b); }
  template<int N> __forceinline vboolf<N> operator ==(float          a, const vfloat<N>& b) { return vfloat<N>(a) == b; }

  template<int N> __forceinline vboolf<N> operator !=(const vfloat<N>& a, float          b) { return a != vfloat<N>(b); }
  template<int N> __forceinline vboolf<N> operator !=(float          a, const vfloat<N>& b) { return vfloat<N>(a) != b; }

  template<int N> __forceinline vboolf<N> operator < (const vfloat<N>& a, float          b) { return a <  vfloat<N>(b); }
  template<int N> __forceinline vboolf<N> operator < (float          a, const vfloat<N>& b) { return vfloat<N>(a) <  b; }
  
  template<int N> __forceinline vboolf<N> operator >=(const vfloat<N>& a, float          b) { return a >= vfloat<N>(b); }
  template<int N> __forceinline vboolf<N> operator >=(float          a, const vfloat<N>& b) { return vfloat<N>(a) >= b; }

  template<int N> __forceinline vboolf<N> operator > (const vfloat<N>& a, float          b) { return a >  vfloat<N>(b); }
  template<int N> __forceinline vboolf<N> operator > (float          a, const vfloat<N>& b) { return vfloat<N>(a) >  b; }

  template<int N> __forceinline vboolf<N> operator <=(const vfloat<N>& a, float          b) { return a <= vfloat<N>(b); }
  template<int N> __forceinline vboolf<N> operator <=(float          a, const vfloat<N>& b) { return vfloat<N>(a) <= b; }

  template<int N> __forceinline vboolf<N> eq(const vfloat<N>& a, const vfloat<N>& b) { return a == b; }
  template<int N> __forceinline vboolf<N> ne(const vfloat<N>& a, const vfloat<N>& b) { return a != b; }
  template<int N> __forceinline vboolf<N> lt(const vfloat<N>& a, const vfloat<N>& b) { return a <  b; }
  template<int N> __forceinline vboolf<N> ge(const vfloat<N>& a, const vfloat<N>& b) { return a >= b; }
  template<int N> __forceinline vboolf<N> gt(const vfloat<N>& a, const vfloat<N>& b) { return a >  b; }
  template<int N> __forceinline vboolf<N> le(const vfloat<N>& a, const vfloat<N>& b) { return a <= b; }

  //template<int N> __forceinline vboolf<N> eq(const vboolf<N>& mask, const vfloat<N>& a, const vfloat<N>& b) { return _mm_mask_cmp_ps_mask(mask, a, b, _MM_CMPINT_EQ); }
  //template<int N> __forceinline vboolf<N> ne(const vboolf<N>& mask, const vfloat<N>& a, const vfloat<N>& b) { return _mm_mask_cmp_ps_mask(mask, a, b, _MM_CMPINT_NE); }
  //template<int N> __forceinline vboolf<N> lt(const vboolf<N>& mask, const vfloat<N>& a, const vfloat<N>& b) { return _mm_mask_cmp_ps_mask(mask, a, b, _MM_CMPINT_LT); }
  //template<int N> __forceinline vboolf<N> ge(const vboolf<N>& mask, const vfloat<N>& a, const vfloat<N>& b) { return _mm_mask_cmp_ps_mask(mask, a, b, _MM_CMPINT_GE); }
  //template<int N> __forceinline vboolf<N> gt(const vboolf<N>& mask, const vfloat<N>& a, const vfloat<N>& b) { return _mm_mask_cmp_ps_mask(mask, a, b, _MM_CMPINT_GT); }
  //template<int N> __forceinline vboolf<N> le(const vboolf<N>& mask, const vfloat<N>& a, const vfloat<N>& b) { return _mm_mask_cmp_ps_mask(mask, a, b, _MM_CMPINT_LE); }

/*
  template<int mask>
    template<int N> __forceinline vfloat<N> select(const vfloat<N>& t, const vfloat<N>& f)
  {
#if defined(__SSE4_1__) 
    return _mm_blend_ps(f, t, mask);
#else
    return select(vboolf<N>(mask), t, f);
#endif
  }
*/

  template<int N> __forceinline vfloat<N> lerp(const float& a, const float& b, const vfloat<N>& t) {
    return vfloat<N>(madd(t.v,b-a,a));
  }
   
  template<int N> __forceinline vfloat<N> lerp(const vfloat<N>& a, const vfloat<N>& b, const vfloat<N>& t) {
    return vfloat<N>(madd(t.v,b.v-a.v,a.v));
  }
  
  template<int N> __forceinline bool isvalid(const vfloat<N>& v) {
    return all((v > vfloat<N>(-FLT_LARGE)) & (v < vfloat<N>(+FLT_LARGE)));
  }

  template<int N> __forceinline bool is_finite(const vfloat<N>& a) {
    return all((a >= vfloat<N>(-FLT_MAX)) & (a <= vfloat<N>(+FLT_MAX)));
  }

  template<int N> __forceinline bool is_finite(const vboolf<N>& valid, const vfloat<N>& a) {
    return all(valid, (a >= vfloat<N>(-FLT_MAX)) & (a <= vfloat<N>(+FLT_MAX)));
  }
  
  ////////////////////////////////////////////////////////////////////////////////
  /// Rounding Functions
  ////////////////////////////////////////////////////////////////////////////////

  template<int N> __forceinline vfloat<N> floor(const vfloat<N>& a) { return vfloat<N>(cl::sycl::floor(a.v));  }
  template<int N> __forceinline vfloat<N> ceil (const vfloat<N>& a) { return vfloat<N>(cl::sycl::ceil(a.v)); }
  template<int N> __forceinline vfloat<N> trunc(const vfloat<N>& a) { return vfloat<N>(cl::sycl::trunc(a.v)); }
  template<int N> __forceinline vfloat<N> frac (const vfloat<N>& a) { return a-floor(a); }
  template<int N> __forceinline vint<N>   floori(const vfloat<N>& a){ return vint<N>((int)floor(a).v); }
  
  ////////////////////////////////////////////////////////////////////////////////
  /// Movement/Shifting/Shuffling Functions
  ////////////////////////////////////////////////////////////////////////////////

/*
  template<int N> __forceinline vfloat<N> unpacklo(const vfloat<N>& a, const vfloat<N>& b) { return _mm_unpacklo_ps(a, b); }
  template<int N> __forceinline vfloat<N> unpackhi(const vfloat<N>& a, const vfloat<N>& b) { return _mm_unpackhi_ps(a, b); }
*/
  
  template<int i0, int i1, int i2, int i3, int N>
    __forceinline vfloat<N> shuffle(const vfloat<N>& v) {
    return vfloat<N>(__spirv_SubgroupShuffleINTEL(v.v, vint<N>(i0,i1,i2,i3).v));
  }
/*
  template<int i0, int i1, int i2, int i3>
  template<int N> __forceinline vfloat<N> shuffle(const vfloat<N>& a, const vfloat<N>& b) {
    return _mm_shuffle_ps(a, b, _MM_SHUFFLE(i3, i2, i1, i0));
  }
*/
  
/*
  template<int N> __forceinline vfloat<N> shuffle8(const vfloat<N>& a, const vint<N>& shuf) {
    return _mm_castsi128_ps(_mm_shuffle_epi8(_mm_castps_si128(a), shuf)); 
  }
*/
      
  template<int i, int N>
    __forceinline vfloat<N> shuffle(const vfloat<N>& v) {
    return vfloat<N>(__spirv_GroupBroadcast(__spv::Scope::Subgroup, v.v, i));
  }
  
  template<int i, int N> __forceinline float extract(const vfloat<N>& a) {
    return __spirv_GroupBroadcast(__spv::Scope::Subgroup, a.v, i);
  }
/*
#if defined (__SSE4_1__)
  template<int dst, int src, int clr> template<int N> __forceinline vfloat<N> insert(const vfloat<N>& a, const vfloat<N>& b) { return _mm_insert_ps(a, b, (dst << 4) | (src << 6) | clr); }
  template<int dst, int src> template<int N> __forceinline vfloat<N> insert(const vfloat<N>& a, const vfloat<N>& b) { return insert<dst, src, 0>(a, b); }
  template<int dst> template<int N> __forceinline vfloat<N> insert(const vfloat<N>& a, const float b) { return insert<dst, 0>(a, _mm_set_ss(b)); }
#else
  template<int dst, int src> template<int N> __forceinline vfloat<N> insert(const vfloat<N>& a, const vfloat<N>& b) { vfloat<N> c = a; c[dst&3] = b[src&3]; return c; }
  template<int dst>  template<int N> __forceinline vfloat<N> insert(const vfloat<N>& a, float b) { vfloat<N> c = a; c[dst&3] = b; return c; }
#endif
*/
  template<int N> __forceinline float toScalar(const vfloat<N>& v) { return extract<0>(v); }

/*
  template<int N> __forceinline vfloat<N> broadcast4f(const vfloat<N>& a, size_t k) {
   return vfloat<N>::broadcast(&a[k]);
 }
*/
  
  template<int N> __forceinline vfloat<N> shift_right_1(const vfloat<N>& x) {
    return vfloat<N>(__spirv_SubgroupShuffleDownINTEL(x.v, x.v, 1));
  }

/*
  template<int N> __forceinline vfloat<N> permute(const vfloat<N> &a, const __m128i &index) {
    return _mm_permutevar_ps(a,index);
  }
*/
  template<int N> __forceinline vfloat<N> broadcast1f(const void* a) {
    return vfloat<N>::broadcast(a);
  }

/*
  template<int i>
  template<int N> __forceinline vfloat<N> align_shift_right(const vfloat<N>& a, const vfloat<N>& b) {
    return _mm_castsi128_ps(_mm_alignr_epi32(_mm_castps_si128(a), _mm_castps_si128(b), i));
  }  
*/

  ////////////////////////////////////////////////////////////////////////////////
  /// Sorting Network
  ////////////////////////////////////////////////////////////////////////////////

#if 0
  template<int N> __forceinline vfloat<N> sort_ascending(const vfloat<N>& v)
  {
    const vfloat<N> a0 = v;
    const vfloat<N> b0 = shuffle<1,0,3,2>(a0);
    const vfloat<N> c0 = min(a0,b0);
    const vfloat<N> d0 = max(a0,b0);
    const vfloat<N> a1 = select<0x5 /* 0b0101 */>(c0,d0);
    const vfloat<N> b1 = shuffle<2,3,0,1>(a1);
    const vfloat<N> c1 = min(a1,b1);
    const vfloat<N> d1 = max(a1,b1);
    const vfloat<N> a2 = select<0x3 /* 0b0011 */>(c1,d1);
    const vfloat<N> b2 = shuffle<0,2,1,3>(a2);
    const vfloat<N> c2 = min(a2,b2);
    const vfloat<N> d2 = max(a2,b2);
    const vfloat<N> a3 = select<0x2 /* 0b0010 */>(c2,d2);
    return a3;
  }

  template<int N> __forceinline vfloat<N> sort_descending(const vfloat<N>& v)
  {
    const vfloat<N> a0 = v;
    const vfloat<N> b0 = shuffle<1,0,3,2>(a0);
    const vfloat<N> c0 = max(a0,b0);
    const vfloat<N> d0 = min(a0,b0);
    const vfloat<N> a1 = select<0x5 /* 0b0101 */>(c0,d0);
    const vfloat<N> b1 = shuffle<2,3,0,1>(a1);
    const vfloat<N> c1 = max(a1,b1);
    const vfloat<N> d1 = min(a1,b1);
    const vfloat<N> a2 = select<0x3 /* 0b0011 */>(c1,d1);
    const vfloat<N> b2 = shuffle<0,2,1,3>(a2);
    const vfloat<N> c2 = max(a2,b2);
    const vfloat<N> d2 = min(a2,b2);
    const vfloat<N> a3 = select<0x2 /* 0b0010 */>(c2,d2);
    return a3;
  }
#endif
  
  ////////////////////////////////////////////////////////////////////////////////
  /// Transpose
  ////////////////////////////////////////////////////////////////////////////////

#if 0
  template<int N> __forceinline void transpose(const vfloat<N>& r0, const vfloat<N>& r1, const vfloat<N>& r2, const vfloat<N>& r3, vfloat<N>& c0, vfloat<N>& c1, vfloat<N>& c2, vfloat<N>& c3)
  {
    vfloat<N> l02 = unpacklo(r0,r2);
    vfloat<N> h02 = unpackhi(r0,r2);
    vfloat<N> l13 = unpacklo(r1,r3);
    vfloat<N> h13 = unpackhi(r1,r3);
    c0 = unpacklo(l02,l13);
    c1 = unpackhi(l02,l13);
    c2 = unpacklo(h02,h13);
    c3 = unpackhi(h02,h13);
  }

  template<int N> __forceinline void transpose(const vfloat<N>& r0, const vfloat<N>& r1, const vfloat<N>& r2, const vfloat<N>& r3, vfloat<N>& c0, vfloat<N>& c1, vfloat<N>& c2)
  {
    vfloat<N> l02 = unpacklo(r0,r2);
    vfloat<N> h02 = unpackhi(r0,r2);
    vfloat<N> l13 = unpacklo(r1,r3);
    vfloat<N> h13 = unpackhi(r1,r3);
    c0 = unpacklo(l02,l13);
    c1 = unpackhi(l02,l13);
    c2 = unpacklo(h02,h13);
  }
#endif 

  ////////////////////////////////////////////////////////////////////////////////
  /// Reductions
  ////////////////////////////////////////////////////////////////////////////////

  template<int N> __forceinline vfloat<N> vreduce_min(const vfloat<N>& v) {
    return cl::sycl::detail::calc<float, __spv::GroupOperation::Reduce>(v.fix_upper(+INFINITY).v, cl::sycl::intel::minimum<float>());
  }

  template<int N> __forceinline vfloat<N> vreduce_max(const vfloat<N>& v) {
    return cl::sycl::detail::calc<float, __spv::GroupOperation::Reduce>(v.fix_upper(-INFINITY).v, cl::sycl::intel::maximum<float>());
  }
  
  template<int N> __forceinline vfloat<N> vreduce_add(const vfloat<N>& v) {
    return cl::sycl::detail::calc<float, __spv::GroupOperation::Reduce>(v.fix_upper(0.0f).v, cl::sycl::intel::plus<float>());
  }

  template<int N> __forceinline float reduce_min(const vfloat<N>& v) { return vreduce_min(v).v; }
  template<int N> __forceinline float reduce_max(const vfloat<N>& v) { return vreduce_max(v).v; }
  template<int N> __forceinline float reduce_add(const vfloat<N>& v) { return vreduce_add(v).v; }

  template<int N> __forceinline size_t select_min(const vboolf<N>& valid, const vfloat<N>& v) 
  { 
    const vfloat<N> a = select(valid,v,vfloat<N>(pos_inf)); 
    const vboolf<N> valid_min = valid & (a == vreduce_min(a));
    return cl::sycl::intel::ctz(movemask(any(valid_min) ? valid_min : valid)); 
  }
  template<int N> __forceinline size_t select_max(const vboolf<N>& valid, const vfloat<N>& v) 
  { 
    const vfloat<N> a = select(valid,v,vfloat<N>(neg_inf)); 
    const vboolf<N> valid_max = valid & (a == vreduce_max(a));
    return cl::sycl::intel::ctz(movemask(any(valid_max) ? valid_max : valid)); 
  }
  
  ////////////////////////////////////////////////////////////////////////////////
  /// Euclidian Space Operators
  ////////////////////////////////////////////////////////////////////////////////

  template<int N> __forceinline float dot(const vfloat<N>& a, const vfloat<N>& b) {
    return cl::sycl::detail::calc<float, __spv::GroupOperation::Reduce>((a*b).fix_upper(0.0f).v, cl::sycl::intel::plus<float>());
  }

  template<int N> __forceinline vfloat<N> cross(const vfloat<N>& a, const vfloat<N>& b)
  {
    const vfloat<N> a0 = a;
    const vfloat<N> b0 = shuffle<1,2,0,3>(b);
    const vfloat<N> a1 = shuffle<1,2,0,3>(a);
    const vfloat<N> b1 = b;
    return shuffle<1,2,0,3>(msub(a0,b0,a1*b1));
  }

  ////////////////////////////////////////////////////////////////////////////////
  /// Output Operators
  ////////////////////////////////////////////////////////////////////////////////

  template<int N> inline const cl::sycl::stream& operator <<(const cl::sycl::stream& cout, const vfloat<N>& a)
  {
    cout << "<";
    for (int i=0; i<N; i++) {
      if (__spirv_BuiltInSubgroupLocalInvocationId == i) {
        cout << a.v;
        if (i != N-1) cout << ", ";
      }
    }
    return cout << ">";
  }

  template<int N> inline std::ostream& operator <<(std::ostream& cout, const vfloat<N>& a) {
    return cout;
  }
}
