// Copyright 2009-2021 Intel Corporation
// SPDX-License-Identifier: Apache-2.0

#pragma once

#define vboolf vboolf_impl
#define vboold vboold_impl
#define vint vint_impl
#define vuint vuint_impl
#define vllong vllong_impl
#define vfloat vfloat_impl
#define vdouble vdouble_impl

namespace embree
{
  /* 4-wide NEON float type */
  template<>
  struct vfloat<4>
  {
    ALIGNED_STRUCT_(16);

    typedef vboolf4 Bool;
    typedef vint4   Int;
    typedef vfloat4 Float;

    enum  { size = 4 };
    union { __m128 v; float f[4]; int i[4]; };

    // Constructors & Assignment
    __forceinline vfloat() {}
    __forceinline vfloat(const vfloat4& other) { v = other.v; }
    __forceinline vfloat4& operator =(const vfloat4& other) { v = other.v; return *this; }

    __forceinline vfloat(__m128 a) : v(a) {}
    __forceinline operator const __m128&() const { return v; }
    __forceinline operator       __m128&()       { return v; }

    __forceinline vfloat(float a) : v(vdupq_n_f32(a)) {}
    __forceinline vfloat(float a, float b, float c, float d) {
      float lanes[4] = {a, b, c, d};
      v = vld1q_f32(lanes);
    }

    __forceinline explicit vfloat(const vint4& a) : v(vcvtq_f32_s32(a.v)) {}
    __forceinline explicit vfloat(const vuint4& x) : v(vcvtq_f32_u32(vreinterpretq_u32_s32(x.v))) {}

    // Constants
    __forceinline vfloat(ZeroTy)      : v(vdupq_n_f32(0.0f)) {}
    __forceinline vfloat(OneTy)       : v(vdupq_n_f32(1.0f)) {}
    __forceinline vfloat(PosInfTy)    : v(vdupq_n_f32(pos_inf)) {}
    __forceinline vfloat(NegInfTy)    : v(vdupq_n_f32(neg_inf)) {}
    __forceinline vfloat(StepTy)      { float lanes[4] = {0.0f, 1.0f, 2.0f, 3.0f}; v = vld1q_f32(lanes); }
    __forceinline vfloat(NaNTy)       : v(vdupq_n_f32(nan)) {}
    __forceinline vfloat(UndefinedTy) : v(vdupq_n_f32(0.0f)) {}

    // Loads and Stores
    static __forceinline vfloat4 load (const void* a) { return vld1q_f32((const float*)a); }
    static __forceinline vfloat4 loadu(const void* a) { return vld1q_f32((const float*)a); }

    static __forceinline void store (void* ptr, const vfloat4& v_arg) { vst1q_f32((float*)ptr, v_arg.v); }
    static __forceinline void storeu(void* ptr, const vfloat4& v_arg) { vst1q_f32((float*)ptr, v_arg.v); }

    // Masked load - AND with mask
    static __forceinline vfloat4 load (const vboolf4& mask, const void* ptr) {
      return vreinterpretq_f32_u32(vandq_u32(vreinterpretq_u32_f32(vld1q_f32((const float*)ptr)), vreinterpretq_u32_f32(mask.v)));
    }
    static __forceinline vfloat4 loadu(const vboolf4& mask, const void* ptr) {
      return vreinterpretq_f32_u32(vandq_u32(vreinterpretq_u32_f32(vld1q_f32((const float*)ptr)), vreinterpretq_u32_f32(mask.v)));
    }

    // Masked store - blend and store
    static __forceinline void store (const vboolf4& mask, void* ptr, const vfloat4& v_arg) { store (ptr, select(mask, v_arg, load (ptr))); }
    static __forceinline void storeu(const vboolf4& mask, void* ptr, const vfloat4& v_arg) { storeu(ptr, select(mask, v_arg, loadu(ptr))); }

    static __forceinline vfloat4 broadcast(const void* a) { return vdupq_n_f32(*(const float*)a); }

    static __forceinline vfloat4 load_nt(const float* ptr) { return vld1q_f32(ptr); }

    // Load from char (4 bytes -> 4 floats)
    static __forceinline vfloat4 load(const char* ptr) {
      int8x8_t t0 = vld1_s8((const int8_t*)ptr);
      int16x8_t t1 = vmovl_s8(t0);
      int32x4_t t2 = vmovl_s16(vget_low_s16(t1));
      return vcvtq_f32_s32(t2);
    }

    // Load from unsigned char (4 bytes -> 4 floats)
    static __forceinline vfloat4 load(const unsigned char* ptr) {
      uint8x8_t t0 = vld1_u8(ptr);
      uint16x8_t t1 = vmovl_u8(t0);
      uint32x4_t t2 = vmovl_u16(vget_low_u16(t1));
      return vcvtq_f32_u32(t2);
    }

    // Load from short (4 shorts -> 4 floats)
    static __forceinline vfloat4 load(const short* ptr) {
      int16x8_t t0 = vld1q_s16(ptr);
      int32x4_t t1 = vmovl_s16(vget_low_s16(t0));
      return vcvtq_f32_s32(t1);
    }

    static __forceinline vfloat4 load(const unsigned short* ptr) {
      return vfloat4(vint4::load(ptr)) * vfloat4(1.0f/65535.0f);
    }

    static __forceinline void store_nt(void* ptr, const vfloat4& v_arg) { vst1q_f32((float*)ptr, v_arg.v); }

    template<int scale = 4>
    static __forceinline vfloat4 gather(const float* ptr, const vint4& index) {
      return vfloat4(
        *(float*)(((char*)ptr)+scale*index[0]),
        *(float*)(((char*)ptr)+scale*index[1]),
        *(float*)(((char*)ptr)+scale*index[2]),
        *(float*)(((char*)ptr)+scale*index[3]));
    }

    template<int scale = 4>
    static __forceinline vfloat4 gather(const vboolf4& mask, const float* ptr, const vint4& index) {
      vfloat4 r = zero;
      if (likely(mask[0])) r[0] = *(float*)(((char*)ptr)+scale*index[0]);
      if (likely(mask[1])) r[1] = *(float*)(((char*)ptr)+scale*index[1]);
      if (likely(mask[2])) r[2] = *(float*)(((char*)ptr)+scale*index[2]);
      if (likely(mask[3])) r[3] = *(float*)(((char*)ptr)+scale*index[3]);
      return r;
    }

    template<int scale = 4>
    static __forceinline void scatter(void* ptr, const vint4& index, const vfloat4& v_arg) {
      *(float*)(((char*)ptr)+scale*index[0]) = v_arg[0];
      *(float*)(((char*)ptr)+scale*index[1]) = v_arg[1];
      *(float*)(((char*)ptr)+scale*index[2]) = v_arg[2];
      *(float*)(((char*)ptr)+scale*index[3]) = v_arg[3];
    }

    template<int scale = 4>
    static __forceinline void scatter(const vboolf4& mask, void* ptr, const vint4& index, const vfloat4& v_arg) {
      if (likely(mask[0])) *(float*)(((char*)ptr)+scale*index[0]) = v_arg[0];
      if (likely(mask[1])) *(float*)(((char*)ptr)+scale*index[1]) = v_arg[1];
      if (likely(mask[2])) *(float*)(((char*)ptr)+scale*index[2]) = v_arg[2];
      if (likely(mask[3])) *(float*)(((char*)ptr)+scale*index[3]) = v_arg[3];
    }

    static __forceinline void store(const vboolf4& mask, char* ptr, const vint4& ofs, const vfloat4& v_arg) { scatter<1>(mask,ptr,ofs,v_arg); }
    static __forceinline void store(const vboolf4& mask, float* ptr, const vint4& ofs, const vfloat4& v_arg) { scatter<4>(mask,ptr,ofs,v_arg); }

    // Array Access
    __forceinline const float& operator [](size_t index) const { assert(index < 4); return f[index]; }
    __forceinline       float& operator [](size_t index)       { assert(index < 4); return f[index]; }

    friend __forceinline vfloat4 select(const vboolf4& m, const vfloat4& t, const vfloat4& f) {
      return vbslq_f32(vreinterpretq_u32_f32(m.v), t.v, f.v);
    }
  };

  // mem<> specialization
  template<> struct mem<vfloat4>
  {
    static __forceinline vfloat4 load (const vboolf4& mask, const void* ptr) { return vfloat4::load (mask,ptr); }
    static __forceinline vfloat4 loadu(const vboolf4& mask, const void* ptr) { return vfloat4::loadu(mask,ptr); }
    static __forceinline void store (const vboolf4& mask, void* ptr, const vfloat4& v) { vfloat4::store (mask,ptr,v); }
    static __forceinline void storeu(const vboolf4& mask, void* ptr, const vfloat4& v) { vfloat4::storeu(mask,ptr,v); }
  };

  // Unary Operators
  __forceinline vfloat4 asFloat(const vint4& a)  { return vreinterpretq_f32_s32(a.v); }
  __forceinline vint4   asInt  (const vfloat4& a) { return vreinterpretq_s32_f32(a.v); }
  __forceinline vuint4  asUInt (const vfloat4& a) { return vreinterpretq_s32_f32(a.v); }

  __forceinline vint4   toInt  (const vfloat4& a) { return vint4(a); }
  __forceinline vfloat4 toFloat(const vint4& a)   { return vfloat4(a); }

  __forceinline vfloat4 operator +(const vfloat4& a) { return a; }
  __forceinline vfloat4 operator -(const vfloat4& a) { return vnegq_f32(a.v); }

  __forceinline vfloat4 abs(const vfloat4& a) { return vabsq_f32(a.v); }

  __forceinline vfloat4 sign(const vfloat4& a) {
    float32x4_t zero_mask = vcltq_f32(a.v, vdupq_n_f32(0.0f));
    return vbslq_f32(zero_mask, vdupq_n_f32(-1.0f), vdupq_n_f32(1.0f));
  }
  __forceinline vfloat4 signmsk(const vfloat4& a) {
    return vreinterpretq_f32_u32(vandq_u32(vreinterpretq_u32_f32(a.v), vdupq_n_u32(0x80000000)));
  }

  // Native division on aarch64
  __forceinline vfloat4 rcp(const vfloat4& a) { return vdivq_f32(vdupq_n_f32(1.0f), a.v); }
  __forceinline vfloat4 sqr(const vfloat4& a) { return vmulq_f32(a.v, a.v); }
  __forceinline vfloat4 sqrt(const vfloat4& a) { return vsqrtq_f32(a.v); }

  __forceinline vfloat4 rsqrt(const vfloat4& a) {
    // Use NEON native rsqrt estimate + Newton-Raphson refinement
    float32x4_t r = vrsqrteq_f32(a.v);
    r = vmulq_f32(r, vrsqrtsq_f32(vmulq_f32(a.v, r), r));
    r = vmulq_f32(r, vrsqrtsq_f32(vmulq_f32(a.v, r), r));
    r = vmulq_f32(r, vrsqrtsq_f32(vmulq_f32(a.v, r), r));
    return r;
  }

  __forceinline vboolf4 isnan(const vfloat4& a) {
    uint32x4_t abs_a = vandq_u32(vreinterpretq_u32_f32(a.v), vdupq_n_u32(0x7fffffff));
    uint32x4_t cmp = vcgtq_u32(abs_a, vdupq_n_u32(0x7f800000));
    return vreinterpretq_f32_u32(cmp);
  }

  // Binary Operators
  __forceinline vfloat4 operator +(const vfloat4& a, const vfloat4& b) { return vaddq_f32(a.v, b.v); }
  __forceinline vfloat4 operator +(const vfloat4& a, float b) { return a + vfloat4(b); }
  __forceinline vfloat4 operator +(float a, const vfloat4& b) { return vfloat4(a) + b; }

  __forceinline vfloat4 operator -(const vfloat4& a, const vfloat4& b) { return vsubq_f32(a.v, b.v); }
  __forceinline vfloat4 operator -(const vfloat4& a, float b) { return a - vfloat4(b); }
  __forceinline vfloat4 operator -(float a, const vfloat4& b) { return vfloat4(a) - b; }

  __forceinline vfloat4 operator *(const vfloat4& a, const vfloat4& b) { return vmulq_f32(a.v, b.v); }
  __forceinline vfloat4 operator *(const vfloat4& a, float b) { return a * vfloat4(b); }
  __forceinline vfloat4 operator *(float a, const vfloat4& b) { return vfloat4(a) * b; }

  __forceinline vfloat4 operator /(const vfloat4& a, const vfloat4& b) { return vdivq_f32(a.v, b.v); }
  __forceinline vfloat4 operator /(const vfloat4& a, float b) { return a / vfloat4(b); }
  __forceinline vfloat4 operator /(float a, const vfloat4& b) { return vfloat4(a) / b; }

  __forceinline vfloat4 operator &(const vfloat4& a, const vfloat4& b) {
    return vreinterpretq_f32_u32(vandq_u32(vreinterpretq_u32_f32(a.v), vreinterpretq_u32_f32(b.v)));
  }
  __forceinline vfloat4 operator |(const vfloat4& a, const vfloat4& b) {
    return vreinterpretq_f32_u32(vorrq_u32(vreinterpretq_u32_f32(a.v), vreinterpretq_u32_f32(b.v)));
  }
  __forceinline vfloat4 operator ^(const vfloat4& a, const vfloat4& b) {
    return vreinterpretq_f32_u32(veorq_u32(vreinterpretq_u32_f32(a.v), vreinterpretq_u32_f32(b.v)));
  }
  __forceinline vfloat4 operator ^(const vfloat4& a, const vint4& b) {
    return vreinterpretq_f32_u32(veorq_u32(vreinterpretq_u32_f32(a.v), vreinterpretq_u32_s32(b.v)));
  }

  __forceinline vfloat4 min(const vfloat4& a, const vfloat4& b) { return vminq_f32(a.v, b.v); }
  __forceinline vfloat4 min(const vfloat4& a, float b) { return min(a, vfloat4(b)); }
  __forceinline vfloat4 min(float a, const vfloat4& b) { return min(vfloat4(a), b); }

  __forceinline vfloat4 max(const vfloat4& a, const vfloat4& b) { return vmaxq_f32(a.v, b.v); }
  __forceinline vfloat4 max(const vfloat4& a, float b) { return max(a, vfloat4(b)); }
  __forceinline vfloat4 max(float a, const vfloat4& b) { return max(vfloat4(a), b); }

  // Integer min/max on float bits (for ULP comparisons)
  __forceinline vfloat4 mini(const vfloat4& a, const vfloat4& b) {
    int32x4_t ai = vreinterpretq_s32_f32(a.v);
    int32x4_t bi = vreinterpretq_s32_f32(b.v);
    return vreinterpretq_f32_s32(vminq_s32(ai, bi));
  }
  __forceinline vfloat4 maxi(const vfloat4& a, const vfloat4& b) {
    int32x4_t ai = vreinterpretq_s32_f32(a.v);
    int32x4_t bi = vreinterpretq_s32_f32(b.v);
    return vreinterpretq_f32_s32(vmaxq_s32(ai, bi));
  }
  __forceinline vfloat4 minui(const vfloat4& a, const vfloat4& b) {
    uint32x4_t ai = vreinterpretq_u32_f32(a.v);
    uint32x4_t bi = vreinterpretq_u32_f32(b.v);
    return vreinterpretq_f32_u32(vminq_u32(ai, bi));
  }
  __forceinline vfloat4 maxui(const vfloat4& a, const vfloat4& b) {
    uint32x4_t ai = vreinterpretq_u32_f32(a.v);
    uint32x4_t bi = vreinterpretq_u32_f32(b.v);
    return vreinterpretq_f32_u32(vmaxq_u32(ai, bi));
  }

  // Ternary Operators - Native FMA on NEON!
  __forceinline vfloat4 madd (const vfloat4& a, const vfloat4& b, const vfloat4& c) { return vfmaq_f32(c.v, a.v, b.v); }
  __forceinline vfloat4 msub (const vfloat4& a, const vfloat4& b, const vfloat4& c) { return vnegq_f32(vfmsq_f32(c.v, a.v, b.v)); }
  __forceinline vfloat4 nmadd(const vfloat4& a, const vfloat4& b, const vfloat4& c) { return vfmsq_f32(c.v, a.v, b.v); }
  __forceinline vfloat4 nmsub(const vfloat4& a, const vfloat4& b, const vfloat4& c) { return vnegq_f32(vfmaq_f32(c.v, a.v, b.v)); }

  // Assignment Operators
  __forceinline vfloat4& operator +=(vfloat4& a, const vfloat4& b) { return a = a + b; }
  __forceinline vfloat4& operator +=(vfloat4& a, float b) { return a = a + b; }
  __forceinline vfloat4& operator -=(vfloat4& a, const vfloat4& b) { return a = a - b; }
  __forceinline vfloat4& operator -=(vfloat4& a, float b) { return a = a - b; }
  __forceinline vfloat4& operator *=(vfloat4& a, const vfloat4& b) { return a = a * b; }
  __forceinline vfloat4& operator *=(vfloat4& a, float b) { return a = a * b; }
  __forceinline vfloat4& operator /=(vfloat4& a, const vfloat4& b) { return a = a / b; }
  __forceinline vfloat4& operator /=(vfloat4& a, float b) { return a = a / b; }

  // Comparison Operators
  __forceinline vboolf4 operator ==(const vfloat4& a, const vfloat4& b) { return vreinterpretq_f32_u32(vceqq_f32(a.v, b.v)); }
  __forceinline vboolf4 operator !=(const vfloat4& a, const vfloat4& b) { return vreinterpretq_f32_u32(vmvnq_u32(vceqq_f32(a.v, b.v))); }
  __forceinline vboolf4 operator < (const vfloat4& a, const vfloat4& b) { return vreinterpretq_f32_u32(vcltq_f32(a.v, b.v)); }
  __forceinline vboolf4 operator >=(const vfloat4& a, const vfloat4& b) { return vreinterpretq_f32_u32(vcgeq_f32(a.v, b.v)); }
  __forceinline vboolf4 operator > (const vfloat4& a, const vfloat4& b) { return vreinterpretq_f32_u32(vcgtq_f32(a.v, b.v)); }
  __forceinline vboolf4 operator <=(const vfloat4& a, const vfloat4& b) { return vreinterpretq_f32_u32(vcleq_f32(a.v, b.v)); }

  __forceinline vboolf4 operator ==(const vfloat4& a, float b) { return a == vfloat4(b); }
  __forceinline vboolf4 operator ==(float a, const vfloat4& b) { return vfloat4(a) == b; }
  __forceinline vboolf4 operator !=(const vfloat4& a, float b) { return a != vfloat4(b); }
  __forceinline vboolf4 operator !=(float a, const vfloat4& b) { return vfloat4(a) != b; }
  __forceinline vboolf4 operator < (const vfloat4& a, float b) { return a <  vfloat4(b); }
  __forceinline vboolf4 operator < (float a, const vfloat4& b) { return vfloat4(a) <  b; }
  __forceinline vboolf4 operator >=(const vfloat4& a, float b) { return a >= vfloat4(b); }
  __forceinline vboolf4 operator >=(float a, const vfloat4& b) { return vfloat4(a) >= b; }
  __forceinline vboolf4 operator > (const vfloat4& a, float b) { return a >  vfloat4(b); }
  __forceinline vboolf4 operator > (float a, const vfloat4& b) { return vfloat4(a) >  b; }
  __forceinline vboolf4 operator <=(const vfloat4& a, float b) { return a <= vfloat4(b); }
  __forceinline vboolf4 operator <=(float a, const vfloat4& b) { return vfloat4(a) <= b; }

  __forceinline vboolf4 eq(const vfloat4& a, const vfloat4& b) { return a == b; }
  __forceinline vboolf4 ne(const vfloat4& a, const vfloat4& b) { return a != b; }
  __forceinline vboolf4 lt(const vfloat4& a, const vfloat4& b) { return a <  b; }
  __forceinline vboolf4 ge(const vfloat4& a, const vfloat4& b) { return a >= b; }
  __forceinline vboolf4 gt(const vfloat4& a, const vfloat4& b) { return a >  b; }
  __forceinline vboolf4 le(const vfloat4& a, const vfloat4& b) { return a <= b; }

  __forceinline vboolf4 eq(const vboolf4& mask, const vfloat4& a, const vfloat4& b) { return mask & (a == b); }
  __forceinline vboolf4 ne(const vboolf4& mask, const vfloat4& a, const vfloat4& b) { return mask & (a != b); }
  __forceinline vboolf4 lt(const vboolf4& mask, const vfloat4& a, const vfloat4& b) { return mask & (a <  b); }
  __forceinline vboolf4 ge(const vboolf4& mask, const vfloat4& a, const vfloat4& b) { return mask & (a >= b); }
  __forceinline vboolf4 gt(const vboolf4& mask, const vfloat4& a, const vfloat4& b) { return mask & (a >  b); }
  __forceinline vboolf4 le(const vboolf4& mask, const vfloat4& a, const vfloat4& b) { return mask & (a <= b); }

  template<int mask>
  __forceinline vfloat4 select(const vfloat4& t, const vfloat4& f) {
    return select(vboolf4(mask), t, f);
  }

  __forceinline vfloat4 lerp(const vfloat4& a, const vfloat4& b, const vfloat4& t) { return madd(t, b-a, a); }

  __forceinline bool isvalid(const vfloat4& v) {
    return all((v > vfloat4(-FLT_LARGE)) & (v < vfloat4(+FLT_LARGE)));
  }
  __forceinline bool is_finite(const vfloat4& a) {
    return all((a >= vfloat4(-FLT_MAX)) & (a <= vfloat4(+FLT_MAX)));
  }
  __forceinline bool is_finite(const vboolf4& valid, const vfloat4& a) {
    return all(valid, (a >= vfloat4(-FLT_MAX)) & (a <= vfloat4(+FLT_MAX)));
  }

  // Rounding Functions - Native NEON rounding!
  __forceinline vfloat4 floor(const vfloat4& a) { return vrndmq_f32(a.v); }
  __forceinline vfloat4 ceil (const vfloat4& a) { return vrndpq_f32(a.v); }
  __forceinline vfloat4 trunc(const vfloat4& a) { return vrndq_f32(a.v); }
  __forceinline vfloat4 round(const vfloat4& a) { return vrndnq_f32(a.v); }
  __forceinline vfloat4 frac(const vfloat4& a) { return a - floor(a); }

  __forceinline vint4 floori(const vfloat4& a) { return vcvtq_s32_f32(floor(a)); }

  // Movement/Shifting/Shuffling
  __forceinline vfloat4 unpacklo(const vfloat4& a, const vfloat4& b) {
    return vreinterpretq_f32_u32(vzip1q_u32(vreinterpretq_u32_f32(a.v), vreinterpretq_u32_f32(b.v)));
  }
  __forceinline vfloat4 unpackhi(const vfloat4& a, const vfloat4& b) {
    return vreinterpretq_f32_u32(vzip2q_u32(vreinterpretq_u32_f32(a.v), vreinterpretq_u32_f32(b.v)));
  }

  template<int i0, int i1, int i2, int i3>
  __forceinline vfloat4 shuffle(const vfloat4& v) {
    return vreinterpretq_f32_u8(vqtbl1q_u8(vreinterpretq_u8_f32(v.v), _MN_SHUFFLE(i0, i1, i2, i3)));
  }

  template<int i0, int i1, int i2, int i3>
  __forceinline vfloat4 shuffle(const vfloat4& a, const vfloat4& b) {
    return vreinterpretq_f32_u8(vqtbl2q_u8((uint8x16x2_t){vreinterpretq_u8_f32(a.v), vreinterpretq_u8_f32(b.v)}, _MF_SHUFFLE(i0, i1, i2, i3)));
  }

  template<int i0>
  __forceinline vfloat4 shuffle(const vfloat4& v) {
    return shuffle<i0,i0,i0,i0>(v);
  }

  template<int i> __forceinline float extract(const vfloat4& a) { return a[i]; }

  template<int dst, int src> __forceinline vfloat4 insert(const vfloat4& a, const vfloat4& b) { vfloat4 c = a; c[dst&3] = b[src&3]; return c; }
  template<int dst> __forceinline vfloat4 insert(const vfloat4& a, float b) { vfloat4 c = a; c[dst&3] = b; return c; }

  __forceinline float toScalar(const vfloat4& v) { return vgetq_lane_f32(v.v, 0); }

  __forceinline vfloat4 shift_right_1(const vfloat4& x) {
    return vreinterpretq_f32_s32(vextq_s32(vreinterpretq_s32_f32(x.v), vdupq_n_s32(0), 1));
  }

  // Sorting Networks
  __forceinline vfloat4 sort_ascending(const vfloat4& v) {
    const vfloat4 a0 = v;
    const vfloat4 b0 = shuffle<1,0,3,2>(a0);
    const vfloat4 c0 = min(a0,b0);
    const vfloat4 d0 = max(a0,b0);
    const vfloat4 a1 = select<0x5>(c0,d0);
    const vfloat4 b1 = shuffle<2,3,0,1>(a1);
    const vfloat4 c1 = min(a1,b1);
    const vfloat4 d1 = max(a1,b1);
    const vfloat4 a2 = select<0x3>(c1,d1);
    const vfloat4 b2 = shuffle<0,2,1,3>(a2);
    const vfloat4 c2 = min(a2,b2);
    const vfloat4 d2 = max(a2,b2);
    const vfloat4 a3 = select<0x2>(c2,d2);
    return a3;
  }

  __forceinline vfloat4 sort_descending(const vfloat4& v) {
    const vfloat4 a0 = v;
    const vfloat4 b0 = shuffle<1,0,3,2>(a0);
    const vfloat4 c0 = max(a0,b0);
    const vfloat4 d0 = min(a0,b0);
    const vfloat4 a1 = select<0x5>(c0,d0);
    const vfloat4 b1 = shuffle<2,3,0,1>(a1);
    const vfloat4 c1 = max(a1,b1);
    const vfloat4 d1 = min(a1,b1);
    const vfloat4 a2 = select<0x3>(c1,d1);
    const vfloat4 b2 = shuffle<0,2,1,3>(a2);
    const vfloat4 c2 = max(a2,b2);
    const vfloat4 d2 = min(a2,b2);
    const vfloat4 a3 = select<0x2>(c2,d2);
    return a3;
  }

  // Transpose
  __forceinline void transpose(const vfloat4& r0, const vfloat4& r1, const vfloat4& r2, const vfloat4& r3, vfloat4& c0, vfloat4& c1, vfloat4& c2, vfloat4& c3) {
    vfloat4 l02 = unpacklo(r0,r2);
    vfloat4 h02 = unpackhi(r0,r2);
    vfloat4 l13 = unpacklo(r1,r3);
    vfloat4 h13 = unpackhi(r1,r3);
    c0 = unpacklo(l02,l13);
    c1 = unpackhi(l02,l13);
    c2 = unpacklo(h02,h13);
    c3 = unpackhi(h02,h13);
  }

  __forceinline void transpose(const vfloat4& r0, const vfloat4& r1, const vfloat4& r2, const vfloat4& r3, vfloat4& c0, vfloat4& c1, vfloat4& c2) {
    vfloat4 l02 = unpacklo(r0,r2);
    vfloat4 h02 = unpackhi(r0,r2);
    vfloat4 l13 = unpacklo(r1,r3);
    vfloat4 h13 = unpackhi(r1,r3);
    c0 = unpacklo(l02,l13);
    c1 = unpackhi(l02,l13);
    c2 = unpacklo(h02,h13);
  }

  // Reductions - Native NEON horizontal operations!
  __forceinline vfloat4 vreduce_min(const vfloat4& v) { return vdupq_n_f32(vminvq_f32(v.v)); }
  __forceinline vfloat4 vreduce_max(const vfloat4& v) { return vdupq_n_f32(vmaxvq_f32(v.v)); }
  __forceinline vfloat4 vreduce_add(const vfloat4& v) { return vdupq_n_f32(vaddvq_f32(v.v)); }

  __forceinline float reduce_min(const vfloat4& v) { return vminvq_f32(v.v); }
  __forceinline float reduce_max(const vfloat4& v) { return vmaxvq_f32(v.v); }
  __forceinline float reduce_add(const vfloat4& v) { return vaddvq_f32(v.v); }

  __forceinline size_t select_min(const vboolf4& valid, const vfloat4& v) {
    const vfloat4 a = select(valid, v, vfloat4(pos_inf));
    const vboolf4 valid_min = valid & (a == vreduce_min(a));
    return bsf(movemask(any(valid_min) ? valid_min : valid));
  }
  __forceinline size_t select_max(const vboolf4& valid, const vfloat4& v) {
    const vfloat4 a = select(valid, v, vfloat4(neg_inf));
    const vboolf4 valid_max = valid & (a == vreduce_max(a));
    return bsf(movemask(any(valid_max) ? valid_max : valid));
  }

  // Euclidean Space Operators
  __forceinline float dot(const vfloat4& a, const vfloat4& b) { return reduce_add(a * b); }

  __forceinline vfloat4 cross(const vfloat4& a, const vfloat4& b) {
    const vfloat4 a0 = a;
    const vfloat4 b0 = shuffle<1,2,0,3>(b);
    const vfloat4 a1 = shuffle<1,2,0,3>(a);
    const vfloat4 b1 = b;
    return shuffle<1,2,0,3>(msub(a0,b0,a1*b1));
  }

  // Output
  __forceinline embree_ostream operator <<(embree_ostream cout, const vfloat4& a) {
    return cout << "<" << a[0] << ", " << a[1] << ", " << a[2] << ", " << a[3] << ">";
  }
}

#undef vboolf
#undef vboold
#undef vint
#undef vuint
#undef vllong
#undef vfloat
#undef vdouble
