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
  /* 8-wide native NEON float type */
  template<>
  struct vfloat<8>
  {
    ALIGNED_STRUCT_(32);

    typedef vboolf8 Bool;
    typedef vint8   Int;
    typedef vfloat8 Float;

    enum  { size = 8 };
    union { __m256 v; float f[8]; int i[8]; };

    ////////////////////////////////////////////////////////////////////////////////
    /// Constructors, Assignment & Cast Operators
    ////////////////////////////////////////////////////////////////////////////////

    __forceinline vfloat() {}
    __forceinline vfloat(const vfloat8& other) { v = other.v; }
    __forceinline vfloat8& operator =(const vfloat8& other) { v = other.v; return *this; }

    __forceinline vfloat(__m256 a) : v(a) {}
    __forceinline operator const __m256&() const { return v; }
    __forceinline operator       __m256&()       { return v; }

    __forceinline explicit vfloat(const vfloat4& a) { v.lo = a.v; v.hi = a.v; }
    __forceinline vfloat(const vfloat4& a, const vfloat4& b) { v.lo = a.v; v.hi = b.v; }

    __forceinline explicit vfloat(const char* a) {
      v.lo = vld1q_f32((const float*)a);
      v.hi = vld1q_f32((const float*)a + 4);
    }
    __forceinline vfloat(float a) { v.lo = vdupq_n_f32(a); v.hi = vdupq_n_f32(a); }
    __forceinline vfloat(float a, float b) {
      float lo[4] = {a, b, a, b}; float hi[4] = {a, b, a, b};
      v.lo = vld1q_f32(lo); v.hi = vld1q_f32(hi);
    }
    __forceinline vfloat(float a, float b, float c, float d) {
      float lo[4] = {a, b, c, d};
      v.lo = vld1q_f32(lo); v.hi = v.lo;
    }
    __forceinline vfloat(float a, float b, float c, float d, float e, float f, float g, float h) {
      float lo[4] = {a, b, c, d}; float hi[4] = {e, f, g, h};
      v.lo = vld1q_f32(lo); v.hi = vld1q_f32(hi);
    }

    __forceinline explicit vfloat(__m256i a) {
      v.lo = vcvtq_f32_s32(a.lo);
      v.hi = vcvtq_f32_s32(a.hi);
    }

    ////////////////////////////////////////////////////////////////////////////////
    /// Constants
    ////////////////////////////////////////////////////////////////////////////////

    __forceinline vfloat(ZeroTy)      { v.lo = vdupq_n_f32(0.0f); v.hi = vdupq_n_f32(0.0f); }
    __forceinline vfloat(OneTy)       { v.lo = vdupq_n_f32(1.0f); v.hi = vdupq_n_f32(1.0f); }
    __forceinline vfloat(PosInfTy)    { v.lo = vdupq_n_f32(pos_inf); v.hi = vdupq_n_f32(pos_inf); }
    __forceinline vfloat(NegInfTy)    { v.lo = vdupq_n_f32(neg_inf); v.hi = vdupq_n_f32(neg_inf); }
    __forceinline vfloat(StepTy) {
      float lo[4] = {0.0f, 1.0f, 2.0f, 3.0f}; float hi[4] = {4.0f, 5.0f, 6.0f, 7.0f};
      v.lo = vld1q_f32(lo); v.hi = vld1q_f32(hi);
    }
    __forceinline vfloat(NaNTy)       { v.lo = vdupq_n_f32(nan); v.hi = vdupq_n_f32(nan); }
    __forceinline vfloat(UndefinedTy) { v.lo = vdupq_n_f32(0.0f); v.hi = vdupq_n_f32(0.0f); }

    ////////////////////////////////////////////////////////////////////////////////
    /// Loads and Stores
    ////////////////////////////////////////////////////////////////////////////////

    static __forceinline vfloat8 broadcast(const void* a) {
      vfloat8 r;
      float32x4_t t = vld1q_dup_f32((const float*)a);
      r.v.lo = t; r.v.hi = t;
      return r;
    }

    static __forceinline vfloat8 load(const char* ptr) {
      vfloat8 r;
      int8x8_t t0 = vld1_s8((const int8_t*)ptr);
      int16x8_t t1 = vmovl_s8(t0);
      r.v.lo = vcvtq_f32_s32(vmovl_s16(vget_low_s16(t1)));
      r.v.hi = vcvtq_f32_s32(vmovl_s16(vget_high_s16(t1)));
      return r;
    }

    static __forceinline vfloat8 load(const unsigned char* ptr) {
      vfloat8 r;
      uint8x8_t t0 = vld1_u8(ptr);
      uint16x8_t t1 = vmovl_u8(t0);
      r.v.lo = vcvtq_f32_u32(vmovl_u16(vget_low_u16(t1)));
      r.v.hi = vcvtq_f32_u32(vmovl_u16(vget_high_u16(t1)));
      return r;
    }

    static __forceinline vfloat8 load(const short* ptr) {
      vfloat8 r;
      int16x8_t t0 = vld1q_s16(ptr);
      r.v.lo = vcvtq_f32_s32(vmovl_s16(vget_low_s16(t0)));
      r.v.hi = vcvtq_f32_s32(vmovl_s16(vget_high_s16(t0)));
      return r;
    }

    static __forceinline vfloat8 load (const void* ptr) {
      vfloat8 r;
      r.v.lo = vld1q_f32((const float*)ptr);
      r.v.hi = vld1q_f32((const float*)ptr + 4);
      return r;
    }
    static __forceinline vfloat8 loadu(const void* ptr) {
      vfloat8 r;
      r.v.lo = vld1q_f32((const float*)ptr);
      r.v.hi = vld1q_f32((const float*)ptr + 4);
      return r;
    }

    static __forceinline void store (void* ptr, const vfloat8& v_arg) {
      vst1q_f32((float*)ptr, v_arg.v.lo);
      vst1q_f32((float*)ptr + 4, v_arg.v.hi);
    }
    static __forceinline void storeu(void* ptr, const vfloat8& v_arg) {
      vst1q_f32((float*)ptr, v_arg.v.lo);
      vst1q_f32((float*)ptr + 4, v_arg.v.hi);
    }

    static __forceinline vfloat8 load (const vboolf8& mask, const void* ptr) {
      vfloat8 r;
      float32x4_t tlo = vld1q_f32((const float*)ptr);
      float32x4_t thi = vld1q_f32((const float*)ptr + 4);
      r.v.lo = vreinterpretq_f32_u32(vandq_u32(vreinterpretq_u32_f32(mask.vl), vreinterpretq_u32_f32(tlo)));
      r.v.hi = vreinterpretq_f32_u32(vandq_u32(vreinterpretq_u32_f32(mask.vh), vreinterpretq_u32_f32(thi)));
      return r;
    }
    static __forceinline vfloat8 loadu(const vboolf8& mask, const void* ptr) {
      return load(mask, ptr);
    }

    static __forceinline void store (const vboolf8& mask, void* ptr, const vfloat8& v_arg) {
      vfloat8 old = load(ptr);
      vfloat8 r;
      r.v.lo = vreinterpretq_f32_u32(vbslq_u32(vreinterpretq_u32_f32(mask.vl), vreinterpretq_u32_f32(v_arg.v.lo), vreinterpretq_u32_f32(old.v.lo)));
      r.v.hi = vreinterpretq_f32_u32(vbslq_u32(vreinterpretq_u32_f32(mask.vh), vreinterpretq_u32_f32(v_arg.v.hi), vreinterpretq_u32_f32(old.v.hi)));
      store(ptr, r);
    }
    static __forceinline void storeu(const vboolf8& mask, void* ptr, const vfloat8& v_arg) {
      vfloat8 old = loadu(ptr);
      vfloat8 r;
      r.v.lo = vreinterpretq_f32_u32(vbslq_u32(vreinterpretq_u32_f32(mask.vl), vreinterpretq_u32_f32(v_arg.v.lo), vreinterpretq_u32_f32(old.v.lo)));
      r.v.hi = vreinterpretq_f32_u32(vbslq_u32(vreinterpretq_u32_f32(mask.vh), vreinterpretq_u32_f32(v_arg.v.hi), vreinterpretq_u32_f32(old.v.hi)));
      storeu(ptr, r);
    }

    static __forceinline vfloat8 load_nt(void* ptr) {
      vfloat8 r;
      r.v.lo = vld1q_f32((const float*)ptr);
      r.v.hi = vld1q_f32((const float*)ptr + 4);
      return r;
    }

    static __forceinline void store_nt(void* ptr, const vfloat8& v) {
      vst1q_f32((float*)ptr, v.v.lo);
      vst1q_f32((float*)ptr + 4, v.v.hi);
    }

    template<int scale = 4>
    static __forceinline vfloat8 gather(const float* ptr, const vint8& index) {
      return vfloat8(
          *(float*)(((char*)ptr)+scale*index[0]),
          *(float*)(((char*)ptr)+scale*index[1]),
          *(float*)(((char*)ptr)+scale*index[2]),
          *(float*)(((char*)ptr)+scale*index[3]),
          *(float*)(((char*)ptr)+scale*index[4]),
          *(float*)(((char*)ptr)+scale*index[5]),
          *(float*)(((char*)ptr)+scale*index[6]),
          *(float*)(((char*)ptr)+scale*index[7]));
    }

    template<int scale = 4>
    static __forceinline vfloat8 gather(const vboolf8& mask, const float* ptr, const vint8& index) {
      vfloat8 r = zero;
      if (likely(mask[0])) r[0] = *(float*)(((char*)ptr)+scale*index[0]);
      if (likely(mask[1])) r[1] = *(float*)(((char*)ptr)+scale*index[1]);
      if (likely(mask[2])) r[2] = *(float*)(((char*)ptr)+scale*index[2]);
      if (likely(mask[3])) r[3] = *(float*)(((char*)ptr)+scale*index[3]);
      if (likely(mask[4])) r[4] = *(float*)(((char*)ptr)+scale*index[4]);
      if (likely(mask[5])) r[5] = *(float*)(((char*)ptr)+scale*index[5]);
      if (likely(mask[6])) r[6] = *(float*)(((char*)ptr)+scale*index[6]);
      if (likely(mask[7])) r[7] = *(float*)(((char*)ptr)+scale*index[7]);
      return r;
    }

    template<int scale = 4>
    static __forceinline void scatter(void* ptr, const vint8& ofs, const vfloat8& v)
    {
      *(float*)(((char*)ptr)+scale*ofs[0]) = v[0];
      *(float*)(((char*)ptr)+scale*ofs[1]) = v[1];
      *(float*)(((char*)ptr)+scale*ofs[2]) = v[2];
      *(float*)(((char*)ptr)+scale*ofs[3]) = v[3];
      *(float*)(((char*)ptr)+scale*ofs[4]) = v[4];
      *(float*)(((char*)ptr)+scale*ofs[5]) = v[5];
      *(float*)(((char*)ptr)+scale*ofs[6]) = v[6];
      *(float*)(((char*)ptr)+scale*ofs[7]) = v[7];
    }

    template<int scale = 4>
    static __forceinline void scatter(const vboolf8& mask, void* ptr, const vint8& ofs, const vfloat8& v)
    {
      if (likely(mask[0])) *(float*)(((char*)ptr)+scale*ofs[0]) = v[0];
      if (likely(mask[1])) *(float*)(((char*)ptr)+scale*ofs[1]) = v[1];
      if (likely(mask[2])) *(float*)(((char*)ptr)+scale*ofs[2]) = v[2];
      if (likely(mask[3])) *(float*)(((char*)ptr)+scale*ofs[3]) = v[3];
      if (likely(mask[4])) *(float*)(((char*)ptr)+scale*ofs[4]) = v[4];
      if (likely(mask[5])) *(float*)(((char*)ptr)+scale*ofs[5]) = v[5];
      if (likely(mask[6])) *(float*)(((char*)ptr)+scale*ofs[6]) = v[6];
      if (likely(mask[7])) *(float*)(((char*)ptr)+scale*ofs[7]) = v[7];
    }

    ////////////////////////////////////////////////////////////////////////////////
    /// Array Access
    ////////////////////////////////////////////////////////////////////////////////

    __forceinline const float& operator [](size_t index) const { assert(index < 8); return f[index]; }
    __forceinline       float& operator [](size_t index)       { assert(index < 8); return f[index]; }
  };


  ////////////////////////////////////////////////////////////////////////////////
  /// Unary Operators
  ////////////////////////////////////////////////////////////////////////////////

  __forceinline vfloat8 asFloat(const vint8& a) {
    vfloat8 r;
    r.v.lo = vreinterpretq_f32_s32(a.vl);
    r.v.hi = vreinterpretq_f32_s32(a.vh);
    return r;
  }
  __forceinline vint8 asInt(const vfloat8& a) {
    vint8 r;
    r.vl = vreinterpretq_s32_f32(a.v.lo);
    r.vh = vreinterpretq_s32_f32(a.v.hi);
    return r;
  }

  __forceinline vint8   toInt  (const vfloat8& a) { return vint8(a); }
  __forceinline vfloat8 toFloat(const vint8&   a) { return vfloat8(a); }

  __forceinline vfloat8 operator +(const vfloat8& a) { return a; }
  __forceinline vfloat8 operator -(const vfloat8& a) {
    vfloat8 r;
    r.v.lo = vnegq_f32(a.v.lo);
    r.v.hi = vnegq_f32(a.v.hi);
    return r;
  }

  __forceinline vfloat8 abs(const vfloat8& a) {
    vfloat8 r;
    r.v.lo = vabsq_f32(a.v.lo);
    r.v.hi = vabsq_f32(a.v.hi);
    return r;
  }

  __forceinline vfloat8 sign(const vfloat8& a) {
    vfloat8 r;
    uint32x4_t cmp_lo = vcltq_f32(a.v.lo, vdupq_n_f32(0.0f));
    uint32x4_t cmp_hi = vcltq_f32(a.v.hi, vdupq_n_f32(0.0f));
    r.v.lo = vbslq_f32(cmp_lo, vdupq_n_f32(-1.0f), vdupq_n_f32(1.0f));
    r.v.hi = vbslq_f32(cmp_hi, vdupq_n_f32(-1.0f), vdupq_n_f32(1.0f));
    return r;
  }

  __forceinline vfloat8 signmsk(const vfloat8& a) {
    vfloat8 r;
    uint32x4_t mask = vdupq_n_u32(0x80000000);
    r.v.lo = vreinterpretq_f32_u32(vandq_u32(vreinterpretq_u32_f32(a.v.lo), mask));
    r.v.hi = vreinterpretq_f32_u32(vandq_u32(vreinterpretq_u32_f32(a.v.hi), mask));
    return r;
  }

  static __forceinline vfloat8 rcp(const vfloat8& a)
  {
    vfloat8 r;
    const float32x4_t one = vdupq_n_f32(1.0f);
    r.v.lo = vdivq_f32(one, a.v.lo);
    r.v.hi = vdivq_f32(one, a.v.hi);
    return r;
  }

  __forceinline vfloat8 sqr(const vfloat8& a) {
    vfloat8 r;
    r.v.lo = vmulq_f32(a.v.lo, a.v.lo);
    r.v.hi = vmulq_f32(a.v.hi, a.v.hi);
    return r;
  }

  __forceinline vfloat8 sqrt(const vfloat8& a) {
    vfloat8 r;
    r.v.lo = vsqrtq_f32(a.v.lo);
    r.v.hi = vsqrtq_f32(a.v.hi);
    return r;
  }

  static __forceinline vfloat8 rsqrt(const vfloat8& a)
  {
    vfloat8 r;
    float32x4_t rlo = vrsqrteq_f32(a.v.lo);
    rlo = vmulq_f32(rlo, vrsqrtsq_f32(vmulq_f32(a.v.lo, rlo), rlo));
    rlo = vmulq_f32(rlo, vrsqrtsq_f32(vmulq_f32(a.v.lo, rlo), rlo));
    rlo = vmulq_f32(rlo, vrsqrtsq_f32(vmulq_f32(a.v.lo, rlo), rlo));
    float32x4_t rhi = vrsqrteq_f32(a.v.hi);
    rhi = vmulq_f32(rhi, vrsqrtsq_f32(vmulq_f32(a.v.hi, rhi), rhi));
    rhi = vmulq_f32(rhi, vrsqrtsq_f32(vmulq_f32(a.v.hi, rhi), rhi));
    rhi = vmulq_f32(rhi, vrsqrtsq_f32(vmulq_f32(a.v.hi, rhi), rhi));
    r.v.lo = rlo; r.v.hi = rhi;
    return r;
  }

  ////////////////////////////////////////////////////////////////////////////////
  /// Binary Operators
  ////////////////////////////////////////////////////////////////////////////////

  __forceinline vfloat8 operator +(const vfloat8& a, const vfloat8& b) {
    vfloat8 r;
    r.v.lo = vaddq_f32(a.v.lo, b.v.lo);
    r.v.hi = vaddq_f32(a.v.hi, b.v.hi);
    return r;
  }
  __forceinline vfloat8 operator +(const vfloat8& a, float b) { return a + vfloat8(b); }
  __forceinline vfloat8 operator +(float a, const vfloat8& b) { return vfloat8(a) + b; }

  __forceinline vfloat8 operator -(const vfloat8& a, const vfloat8& b) {
    vfloat8 r;
    r.v.lo = vsubq_f32(a.v.lo, b.v.lo);
    r.v.hi = vsubq_f32(a.v.hi, b.v.hi);
    return r;
  }
  __forceinline vfloat8 operator -(const vfloat8& a, float b) { return a - vfloat8(b); }
  __forceinline vfloat8 operator -(float a, const vfloat8& b) { return vfloat8(a) - b; }

  __forceinline vfloat8 operator *(const vfloat8& a, const vfloat8& b) {
    vfloat8 r;
    r.v.lo = vmulq_f32(a.v.lo, b.v.lo);
    r.v.hi = vmulq_f32(a.v.hi, b.v.hi);
    return r;
  }
  __forceinline vfloat8 operator *(const vfloat8& a, float b) { return a * vfloat8(b); }
  __forceinline vfloat8 operator *(float a, const vfloat8& b) { return vfloat8(a) * b; }

  __forceinline vfloat8 operator /(const vfloat8& a, const vfloat8& b) {
    vfloat8 r;
    r.v.lo = vdivq_f32(a.v.lo, b.v.lo);
    r.v.hi = vdivq_f32(a.v.hi, b.v.hi);
    return r;
  }
  __forceinline vfloat8 operator /(const vfloat8& a, float b) { return a / vfloat8(b); }
  __forceinline vfloat8 operator /(float a, const vfloat8& b) { return vfloat8(a) / b; }

  __forceinline vfloat8 operator &(const vfloat8& a, const vfloat8& b) {
    vfloat8 r;
    r.v.lo = vreinterpretq_f32_u32(vandq_u32(vreinterpretq_u32_f32(a.v.lo), vreinterpretq_u32_f32(b.v.lo)));
    r.v.hi = vreinterpretq_f32_u32(vandq_u32(vreinterpretq_u32_f32(a.v.hi), vreinterpretq_u32_f32(b.v.hi)));
    return r;
  }
  __forceinline vfloat8 operator |(const vfloat8& a, const vfloat8& b) {
    vfloat8 r;
    r.v.lo = vreinterpretq_f32_u32(vorrq_u32(vreinterpretq_u32_f32(a.v.lo), vreinterpretq_u32_f32(b.v.lo)));
    r.v.hi = vreinterpretq_f32_u32(vorrq_u32(vreinterpretq_u32_f32(a.v.hi), vreinterpretq_u32_f32(b.v.hi)));
    return r;
  }
  __forceinline vfloat8 operator ^(const vfloat8& a, const vfloat8& b) {
    vfloat8 r;
    r.v.lo = vreinterpretq_f32_u32(veorq_u32(vreinterpretq_u32_f32(a.v.lo), vreinterpretq_u32_f32(b.v.lo)));
    r.v.hi = vreinterpretq_f32_u32(veorq_u32(vreinterpretq_u32_f32(a.v.hi), vreinterpretq_u32_f32(b.v.hi)));
    return r;
  }
  __forceinline vfloat8 operator ^(const vfloat8& a, const vint8& b) {
    vfloat8 r;
    r.v.lo = vreinterpretq_f32_u32(veorq_u32(vreinterpretq_u32_f32(a.v.lo), vreinterpretq_u32_s32(b.vl)));
    r.v.hi = vreinterpretq_f32_u32(veorq_u32(vreinterpretq_u32_f32(a.v.hi), vreinterpretq_u32_s32(b.vh)));
    return r;
  }

  __forceinline vfloat8 min(const vfloat8& a, const vfloat8& b) {
    vfloat8 r;
    r.v.lo = vminq_f32(a.v.lo, b.v.lo);
    r.v.hi = vminq_f32(a.v.hi, b.v.hi);
    return r;
  }
  __forceinline vfloat8 min(const vfloat8& a, float b) { return min(a, vfloat8(b)); }
  __forceinline vfloat8 min(float a, const vfloat8& b) { return min(vfloat8(a), b); }

  __forceinline vfloat8 max(const vfloat8& a, const vfloat8& b) {
    vfloat8 r;
    r.v.lo = vmaxq_f32(a.v.lo, b.v.lo);
    r.v.hi = vmaxq_f32(a.v.hi, b.v.hi);
    return r;
  }
  __forceinline vfloat8 max(const vfloat8& a, float b) { return max(a, vfloat8(b)); }
  __forceinline vfloat8 max(float a, const vfloat8& b) { return max(vfloat8(a), b); }

  static __forceinline vfloat8 mini(const vfloat8& a, const vfloat8& b) {
    vfloat8 r;
    r.v.lo = vreinterpretq_f32_s32(vminq_s32(vreinterpretq_s32_f32(a.v.lo), vreinterpretq_s32_f32(b.v.lo)));
    r.v.hi = vreinterpretq_f32_s32(vminq_s32(vreinterpretq_s32_f32(a.v.hi), vreinterpretq_s32_f32(b.v.hi)));
    return r;
  }

  static __forceinline vfloat8 maxi(const vfloat8& a, const vfloat8& b) {
    vfloat8 r;
    r.v.lo = vreinterpretq_f32_s32(vmaxq_s32(vreinterpretq_s32_f32(a.v.lo), vreinterpretq_s32_f32(b.v.lo)));
    r.v.hi = vreinterpretq_f32_s32(vmaxq_s32(vreinterpretq_s32_f32(a.v.hi), vreinterpretq_s32_f32(b.v.hi)));
    return r;
  }

  static __forceinline vfloat8 minui(const vfloat8& a, const vfloat8& b) {
    vfloat8 r;
    r.v.lo = vreinterpretq_f32_u32(vminq_u32(vreinterpretq_u32_f32(a.v.lo), vreinterpretq_u32_f32(b.v.lo)));
    r.v.hi = vreinterpretq_f32_u32(vminq_u32(vreinterpretq_u32_f32(a.v.hi), vreinterpretq_u32_f32(b.v.hi)));
    return r;
  }

  static __forceinline vfloat8 maxui(const vfloat8& a, const vfloat8& b) {
    vfloat8 r;
    r.v.lo = vreinterpretq_f32_u32(vmaxq_u32(vreinterpretq_u32_f32(a.v.lo), vreinterpretq_u32_f32(b.v.lo)));
    r.v.hi = vreinterpretq_f32_u32(vmaxq_u32(vreinterpretq_u32_f32(a.v.hi), vreinterpretq_u32_f32(b.v.hi)));
    return r;
  }

  ////////////////////////////////////////////////////////////////////////////////
  /// Ternary Operators
  ////////////////////////////////////////////////////////////////////////////////

  static __forceinline vfloat8 madd(const vfloat8& a, const vfloat8& b, const vfloat8& c) {
    vfloat8 r;
    r.v.lo = vfmaq_f32(c.v.lo, a.v.lo, b.v.lo);
    r.v.hi = vfmaq_f32(c.v.hi, a.v.hi, b.v.hi);
    return r;
  }
  static __forceinline vfloat8 msub(const vfloat8& a, const vfloat8& b, const vfloat8& c) {
    vfloat8 r;
    r.v.lo = vnegq_f32(vfmsq_f32(c.v.lo, a.v.lo, b.v.lo));
    r.v.hi = vnegq_f32(vfmsq_f32(c.v.hi, a.v.hi, b.v.hi));
    return r;
  }
  static __forceinline vfloat8 nmadd(const vfloat8& a, const vfloat8& b, const vfloat8& c) {
    vfloat8 r;
    r.v.lo = vfmsq_f32(c.v.lo, a.v.lo, b.v.lo);
    r.v.hi = vfmsq_f32(c.v.hi, a.v.hi, b.v.hi);
    return r;
  }
  static __forceinline vfloat8 nmsub(const vfloat8& a, const vfloat8& b, const vfloat8& c) {
    vfloat8 r;
    r.v.lo = vnegq_f32(vfmaq_f32(c.v.lo, a.v.lo, b.v.lo));
    r.v.hi = vnegq_f32(vfmaq_f32(c.v.hi, a.v.hi, b.v.hi));
    return r;
  }

  ////////////////////////////////////////////////////////////////////////////////
  /// Assignment Operators
  ////////////////////////////////////////////////////////////////////////////////

  __forceinline vfloat8& operator +=(vfloat8& a, const vfloat8& b) { return a = a + b; }
  __forceinline vfloat8& operator +=(vfloat8& a, float b) { return a = a + b; }

  __forceinline vfloat8& operator -=(vfloat8& a, const vfloat8& b) { return a = a - b; }
  __forceinline vfloat8& operator -=(vfloat8& a, float b) { return a = a - b; }

  __forceinline vfloat8& operator *=(vfloat8& a, const vfloat8& b) { return a = a * b; }
  __forceinline vfloat8& operator *=(vfloat8& a, float b) { return a = a * b; }

  __forceinline vfloat8& operator /=(vfloat8& a, const vfloat8& b) { return a = a / b; }
  __forceinline vfloat8& operator /=(vfloat8& a, float b) { return a = a / b; }

  ////////////////////////////////////////////////////////////////////////////////
  /// Comparison Operators + Select
  ////////////////////////////////////////////////////////////////////////////////

  static __forceinline vboolf8 operator ==(const vfloat8& a, const vfloat8& b) {
    vboolf8 r;
    r.vl = vreinterpretq_f32_u32(vceqq_f32(a.v.lo, b.v.lo));
    r.vh = vreinterpretq_f32_u32(vceqq_f32(a.v.hi, b.v.hi));
    return r;
  }
  static __forceinline vboolf8 operator !=(const vfloat8& a, const vfloat8& b) {
    vboolf8 r;
    r.vl = vreinterpretq_f32_u32(vmvnq_u32(vceqq_f32(a.v.lo, b.v.lo)));
    r.vh = vreinterpretq_f32_u32(vmvnq_u32(vceqq_f32(a.v.hi, b.v.hi)));
    return r;
  }
  static __forceinline vboolf8 operator <(const vfloat8& a, const vfloat8& b) {
    vboolf8 r;
    r.vl = vreinterpretq_f32_u32(vcltq_f32(a.v.lo, b.v.lo));
    r.vh = vreinterpretq_f32_u32(vcltq_f32(a.v.hi, b.v.hi));
    return r;
  }
  static __forceinline vboolf8 operator >=(const vfloat8& a, const vfloat8& b) {
    vboolf8 r;
    r.vl = vreinterpretq_f32_u32(vcgeq_f32(a.v.lo, b.v.lo));
    r.vh = vreinterpretq_f32_u32(vcgeq_f32(a.v.hi, b.v.hi));
    return r;
  }
  static __forceinline vboolf8 operator >(const vfloat8& a, const vfloat8& b) {
    vboolf8 r;
    r.vl = vreinterpretq_f32_u32(vcgtq_f32(a.v.lo, b.v.lo));
    r.vh = vreinterpretq_f32_u32(vcgtq_f32(a.v.hi, b.v.hi));
    return r;
  }
  static __forceinline vboolf8 operator <=(const vfloat8& a, const vfloat8& b) {
    vboolf8 r;
    r.vl = vreinterpretq_f32_u32(vcleq_f32(a.v.lo, b.v.lo));
    r.vh = vreinterpretq_f32_u32(vcleq_f32(a.v.hi, b.v.hi));
    return r;
  }

  static __forceinline vfloat8 select(const vboolf8& m, const vfloat8& t, const vfloat8& f) {
    vfloat8 r;
    r.v.lo = vbslq_f32(vreinterpretq_u32_f32(m.vl), t.v.lo, f.v.lo);
    r.v.hi = vbslq_f32(vreinterpretq_u32_f32(m.vh), t.v.hi, f.v.hi);
    return r;
  }

  template<int mask>
  __forceinline vfloat8 select(const vfloat8& t, const vfloat8& f) {
    return select(vboolf8(mask), t, f);
  }

  __forceinline vboolf8 operator ==(const vfloat8& a, const float& b) { return a == vfloat8(b); }
  __forceinline vboolf8 operator ==(const float& a, const vfloat8& b) { return vfloat8(a) == b; }

  __forceinline vboolf8 operator !=(const vfloat8& a, const float& b) { return a != vfloat8(b); }
  __forceinline vboolf8 operator !=(const float& a, const vfloat8& b) { return vfloat8(a) != b; }

  __forceinline vboolf8 operator <(const vfloat8& a, const float& b) { return a < vfloat8(b); }
  __forceinline vboolf8 operator <(const float& a, const vfloat8& b) { return vfloat8(a) < b; }

  __forceinline vboolf8 operator >=(const vfloat8& a, const float& b) { return a >= vfloat8(b); }
  __forceinline vboolf8 operator >=(const float& a, const vfloat8& b) { return vfloat8(a) >= b; }

  __forceinline vboolf8 operator >(const vfloat8& a, const float& b) { return a > vfloat8(b); }
  __forceinline vboolf8 operator >(const float& a, const vfloat8& b) { return vfloat8(a) > b; }

  __forceinline vboolf8 operator <=(const vfloat8& a, const float& b) { return a <= vfloat8(b); }
  __forceinline vboolf8 operator <=(const float& a, const vfloat8& b) { return vfloat8(a) <= b; }

  __forceinline vboolf8 eq(const vfloat8& a, const vfloat8& b) { return a == b; }
  __forceinline vboolf8 ne(const vfloat8& a, const vfloat8& b) { return a != b; }
  __forceinline vboolf8 lt(const vfloat8& a, const vfloat8& b) { return a < b; }
  __forceinline vboolf8 ge(const vfloat8& a, const vfloat8& b) { return a >= b; }
  __forceinline vboolf8 gt(const vfloat8& a, const vfloat8& b) { return a > b; }
  __forceinline vboolf8 le(const vfloat8& a, const vfloat8& b) { return a <= b; }

  static __forceinline vboolf8 eq(const vboolf8& mask, const vfloat8& a, const vfloat8& b) { return mask & (a == b); }
  static __forceinline vboolf8 ne(const vboolf8& mask, const vfloat8& a, const vfloat8& b) { return mask & (a != b); }
  static __forceinline vboolf8 lt(const vboolf8& mask, const vfloat8& a, const vfloat8& b) { return mask & (a < b); }
  static __forceinline vboolf8 ge(const vboolf8& mask, const vfloat8& a, const vfloat8& b) { return mask & (a >= b); }
  static __forceinline vboolf8 gt(const vboolf8& mask, const vfloat8& a, const vfloat8& b) { return mask & (a > b); }
  static __forceinline vboolf8 le(const vboolf8& mask, const vfloat8& a, const vfloat8& b) { return mask & (a <= b); }

  __forceinline vfloat8 lerp(const vfloat8& a, const vfloat8& b, const vfloat8& t) {
    return madd(t,b-a,a);
  }

  __forceinline bool isvalid(const vfloat8& v) {
    return all((v > vfloat8(-FLT_LARGE)) & (v < vfloat8(+FLT_LARGE)));
  }

  __forceinline bool is_finite(const vfloat8& a) {
    return all((a >= vfloat8(-FLT_MAX)) & (a <= vfloat8(+FLT_MAX)));
  }

  __forceinline bool is_finite(const vboolf8& valid, const vfloat8& a) {
    return all(valid, (a >= vfloat8(-FLT_MAX)) & (a <= vfloat8(+FLT_MAX)));
  }

  ////////////////////////////////////////////////////////////////////////////////
  /// Rounding Functions
  ////////////////////////////////////////////////////////////////////////////////

  __forceinline vfloat8 floor(const vfloat8& a) {
    vfloat8 r;
    r.v.lo = vrndmq_f32(a.v.lo);
    r.v.hi = vrndmq_f32(a.v.hi);
    return r;
  }
  __forceinline vfloat8 ceil(const vfloat8& a) {
    vfloat8 r;
    r.v.lo = vrndpq_f32(a.v.lo);
    r.v.hi = vrndpq_f32(a.v.hi);
    return r;
  }
  __forceinline vfloat8 trunc(const vfloat8& a) {
    vfloat8 r;
    r.v.lo = vrndq_f32(a.v.lo);
    r.v.hi = vrndq_f32(a.v.hi);
    return r;
  }
  __forceinline vfloat8 round(const vfloat8& a) {
    vfloat8 r;
    r.v.lo = vrndnq_f32(a.v.lo);
    r.v.hi = vrndnq_f32(a.v.hi);
    return r;
  }

  __forceinline vfloat8 frac(const vfloat8& a) { return a-floor(a); }

  ////////////////////////////////////////////////////////////////////////////////
  /// Movement/Shifting/Shuffling Functions
  ////////////////////////////////////////////////////////////////////////////////

  __forceinline vfloat8 unpacklo(const vfloat8& a, const vfloat8& b) {
    vfloat8 r;
    r.v.lo = vreinterpretq_f32_u32(vzip1q_u32(vreinterpretq_u32_f32(a.v.lo), vreinterpretq_u32_f32(b.v.lo)));
    r.v.hi = vreinterpretq_f32_u32(vzip1q_u32(vreinterpretq_u32_f32(a.v.hi), vreinterpretq_u32_f32(b.v.hi)));
    return r;
  }
  __forceinline vfloat8 unpackhi(const vfloat8& a, const vfloat8& b) {
    vfloat8 r;
    r.v.lo = vreinterpretq_f32_u32(vzip2q_u32(vreinterpretq_u32_f32(a.v.lo), vreinterpretq_u32_f32(b.v.lo)));
    r.v.hi = vreinterpretq_f32_u32(vzip2q_u32(vreinterpretq_u32_f32(a.v.hi), vreinterpretq_u32_f32(b.v.hi)));
    return r;
  }

  template<int i>
  __forceinline vfloat8 shuffle(const vfloat8& v) {
    return _mm256_permute_ps(v, _MM_SHUFFLE(i, i, i, i));
  }

  template<int i0, int i1>
  __forceinline vfloat8 shuffle4(const vfloat8& v) {
    return _mm256_permute2f128_ps(v, v, (i1 << 4) | (i0 << 0));
  }

  template<int i0, int i1>
  __forceinline vfloat8 shuffle4(const vfloat8& a, const vfloat8& b) {
    return _mm256_permute2f128_ps(a, b, (i1 << 4) | (i0 << 0));
  }

  template<int i0, int i1, int i2, int i3>
  __forceinline vfloat8 shuffle(const vfloat8& v) {
    return _mm256_permute_ps(v, _MM_SHUFFLE(i3, i2, i1, i0));
  }

  template<int i0, int i1, int i2, int i3>
  __forceinline vfloat8 shuffle(const vfloat8& a, const vfloat8& b) {
    return _mm256_shuffle_ps(a, b, _MM_SHUFFLE(i3, i2, i1, i0));
  }

  __forceinline vfloat8 broadcast(const float* ptr) {
    vfloat8 r;
    float32x4_t t = vld1q_dup_f32(ptr);
    r.v.lo = t; r.v.hi = t;
    return r;
  }

  template<size_t i> __forceinline vfloat8 insert4(const vfloat8& a, const vfloat4& b) {
    vfloat8 r = a;
    if (i == 0) r.v.lo = b.v; else r.v.hi = b.v;
    return r;
  }

  template<size_t i> __forceinline vfloat4 extract4(const vfloat8& a) {
    vfloat4 r;
    if (i == 0) r.v = a.v.lo; else r.v = a.v.hi;
    return r;
  }
  template<> __forceinline vfloat4 extract4<0>(const vfloat8& a) { vfloat4 r; r.v = a.v.lo; return r; }

  __forceinline float toScalar(const vfloat8& v) { return vgetq_lane_f32(v.v.lo, 0); }

  static __forceinline vfloat8 shift_right_1(const vfloat8& x) {
    const vfloat8 t0 = shuffle<1,2,3,0>(x);
    const vfloat8 t1 = shuffle4<1,0>(t0);
    return _mm256_blend_ps(t0,t1,0x88);
  }

  __forceinline vint8 floori(const vfloat8& a) {
    return vint8(floor(a));
  }

  ////////////////////////////////////////////////////////////////////////////////
  /// Transpose
  ////////////////////////////////////////////////////////////////////////////////

  __forceinline void transpose(const vfloat8& r0, const vfloat8& r1, const vfloat8& r2, const vfloat8& r3, vfloat8& c0, vfloat8& c1, vfloat8& c2, vfloat8& c3)
  {
    vfloat8 l02 = unpacklo(r0,r2);
    vfloat8 h02 = unpackhi(r0,r2);
    vfloat8 l13 = unpacklo(r1,r3);
    vfloat8 h13 = unpackhi(r1,r3);
    c0 = unpacklo(l02,l13);
    c1 = unpackhi(l02,l13);
    c2 = unpacklo(h02,h13);
    c3 = unpackhi(h02,h13);
  }

  __forceinline void transpose(const vfloat8& r0, const vfloat8& r1, const vfloat8& r2, const vfloat8& r3, vfloat8& c0, vfloat8& c1, vfloat8& c2)
  {
    vfloat8 l02 = unpacklo(r0,r2);
    vfloat8 h02 = unpackhi(r0,r2);
    vfloat8 l13 = unpacklo(r1,r3);
    vfloat8 h13 = unpackhi(r1,r3);
    c0 = unpacklo(l02,l13);
    c1 = unpackhi(l02,l13);
    c2 = unpacklo(h02,h13);
  }

  __forceinline void transpose(const vfloat8& r0, const vfloat8& r1, const vfloat8& r2, const vfloat8& r3, const vfloat8& r4, const vfloat8& r5, const vfloat8& r6, const vfloat8& r7,
                               vfloat8& c0, vfloat8& c1, vfloat8& c2, vfloat8& c3, vfloat8& c4, vfloat8& c5, vfloat8& c6, vfloat8& c7)
  {
    vfloat8 h0,h1,h2,h3; transpose(r0,r1,r2,r3,h0,h1,h2,h3);
    vfloat8 h4,h5,h6,h7; transpose(r4,r5,r6,r7,h4,h5,h6,h7);
    c0 = shuffle4<0,2>(h0,h4);
    c1 = shuffle4<0,2>(h1,h5);
    c2 = shuffle4<0,2>(h2,h6);
    c3 = shuffle4<0,2>(h3,h7);
    c4 = shuffle4<1,3>(h0,h4);
    c5 = shuffle4<1,3>(h1,h5);
    c6 = shuffle4<1,3>(h2,h6);
    c7 = shuffle4<1,3>(h3,h7);
  }

  __forceinline void transpose(const vfloat4& r0, const vfloat4& r1, const vfloat4& r2, const vfloat4& r3, const vfloat4& r4, const vfloat4& r5, const vfloat4& r6, const vfloat4& r7,
                               vfloat8& c0, vfloat8& c1, vfloat8& c2, vfloat8& c3)
  {
    transpose(vfloat8(r0,r4), vfloat8(r1,r5), vfloat8(r2,r6), vfloat8(r3,r7), c0, c1, c2, c3);
  }

  __forceinline void transpose(const vfloat4& r0, const vfloat4& r1, const vfloat4& r2, const vfloat4& r3, const vfloat4& r4, const vfloat4& r5, const vfloat4& r6, const vfloat4& r7,
                               vfloat8& c0, vfloat8& c1, vfloat8& c2)
  {
    transpose(vfloat8(r0,r4), vfloat8(r1,r5), vfloat8(r2,r6), vfloat8(r3,r7), c0, c1, c2);
  }

  ////////////////////////////////////////////////////////////////////////////////
  /// Reductions
  ////////////////////////////////////////////////////////////////////////////////

  __forceinline float reduce_min(const vfloat8& v) { return vminvq_f32(vminq_f32(v.v.lo, v.v.hi)); }
  __forceinline float reduce_max(const vfloat8& v) { return vmaxvq_f32(vmaxq_f32(v.v.lo, v.v.hi)); }
  __forceinline vfloat8 vreduce_min(const vfloat8& v) { return vfloat8(reduce_min(v)); }
  __forceinline vfloat8 vreduce_max(const vfloat8& v) { return vfloat8(reduce_max(v)); }
  __forceinline float reduce_add(const vfloat8& v) { return vaddvq_f32(vaddq_f32(v.v.lo, v.v.hi)); }

  __forceinline size_t select_min(const vboolf8& valid, const vfloat8& v)
  {
    const vfloat8 a = select(valid,v,vfloat8(pos_inf));
    const vbool8 valid_min = valid & (a == vreduce_min(a));
    return bsf(movemask(any(valid_min) ? valid_min : valid));
  }

  __forceinline size_t select_max(const vboolf8& valid, const vfloat8& v)
  {
    const vfloat8 a = select(valid,v,vfloat8(neg_inf));
    const vbool8 valid_max = valid & (a == vreduce_max(a));
    return bsf(movemask(any(valid_max) ? valid_max : valid));
  }


  ////////////////////////////////////////////////////////////////////////////////
  /// Euclidean Space Operators (pairs of Vec3fa's)
  ////////////////////////////////////////////////////////////////////////////////

  __forceinline vfloat8 dot(const vfloat8& a, const vfloat8& b) {
    vfloat8 r;
    r.v.lo = vdupq_n_f32(vaddvq_f32(vmulq_f32(a.v.lo, b.v.lo)));
    r.v.hi = vdupq_n_f32(vaddvq_f32(vmulq_f32(a.v.hi, b.v.hi)));
    return r;
  }

  __forceinline vfloat8 cross(const vfloat8& a, const vfloat8& b)
  {
    const vfloat8 a0 = a;
    const vfloat8 b0 = shuffle<1,2,0,3>(b);
    const vfloat8 a1 = shuffle<1,2,0,3>(a);
    const vfloat8 b1 = b;
    return shuffle<1,2,0,3>(msub(a0,b0,a1*b1));
  }

  __forceinline vfloat<8> normalize(const vfloat<8>& a) { return a*rsqrt(dot(a,a)); }

  ////////////////////////////////////////////////////////////////////////////////
  /// In Register Sorting
  ////////////////////////////////////////////////////////////////////////////////

  __forceinline vfloat8 sort_ascending(const vfloat8& v)
  {
    const vfloat8 a0 = v;
    const vfloat8 b0 = shuffle<1,0,3,2>(a0);
    const vfloat8 c0 = min(a0,b0);
    const vfloat8 d0 = max(a0,b0);
    const vfloat8 a1 = select<0x99 /* 0b10011001 */>(c0,d0);
    const vfloat8 b1 = shuffle<2,3,0,1>(a1);
    const vfloat8 c1 = min(a1,b1);
    const vfloat8 d1 = max(a1,b1);
    const vfloat8 a2 = select<0xc3 /* 0b11000011 */>(c1,d1);
    const vfloat8 b2 = shuffle<1,0,3,2>(a2);
    const vfloat8 c2 = min(a2,b2);
    const vfloat8 d2 = max(a2,b2);
    const vfloat8 a3 = select<0xa5 /* 0b10100101 */>(c2,d2);
    const vfloat8 b3 = shuffle4<1,0>(a3);
    const vfloat8 c3 = min(a3,b3);
    const vfloat8 d3 = max(a3,b3);
    const vfloat8 a4 = select<0xf /* 0b00001111 */>(c3,d3);
    const vfloat8 b4 = shuffle<2,3,0,1>(a4);
    const vfloat8 c4 = min(a4,b4);
    const vfloat8 d4 = max(a4,b4);
    const vfloat8 a5 = select<0x33 /* 0b00110011 */>(c4,d4);
    const vfloat8 b5 = shuffle<1,0,3,2>(a5);
    const vfloat8 c5 = min(a5,b5);
    const vfloat8 d5 = max(a5,b5);
    const vfloat8 a6 = select<0x55 /* 0b01010101 */>(c5,d5);
    return a6;
  }

  __forceinline vfloat8 sort_descending(const vfloat8& v)
  {
    const vfloat8 a0 = v;
    const vfloat8 b0 = shuffle<1,0,3,2>(a0);
    const vfloat8 c0 = max(a0,b0);
    const vfloat8 d0 = min(a0,b0);
    const vfloat8 a1 = select<0x99 /* 0b10011001 */>(c0,d0);
    const vfloat8 b1 = shuffle<2,3,0,1>(a1);
    const vfloat8 c1 = max(a1,b1);
    const vfloat8 d1 = min(a1,b1);
    const vfloat8 a2 = select<0xc3 /* 0b11000011 */>(c1,d1);
    const vfloat8 b2 = shuffle<1,0,3,2>(a2);
    const vfloat8 c2 = max(a2,b2);
    const vfloat8 d2 = min(a2,b2);
    const vfloat8 a3 = select<0xa5 /* 0b10100101 */>(c2,d2);
    const vfloat8 b3 = shuffle4<1,0>(a3);
    const vfloat8 c3 = max(a3,b3);
    const vfloat8 d3 = min(a3,b3);
    const vfloat8 a4 = select<0xf /* 0b00001111 */>(c3,d3);
    const vfloat8 b4 = shuffle<2,3,0,1>(a4);
    const vfloat8 c4 = max(a4,b4);
    const vfloat8 d4 = min(a4,b4);
    const vfloat8 a5 = select<0x33 /* 0b00110011 */>(c4,d4);
    const vfloat8 b5 = shuffle<1,0,3,2>(a5);
    const vfloat8 c5 = max(a5,b5);
    const vfloat8 d5 = min(a5,b5);
    const vfloat8 a6 = select<0x55 /* 0b01010101 */>(c5,d5);
    return a6;
  }

  ////////////////////////////////////////////////////////////////////////////////
  /// Output Operators
  ////////////////////////////////////////////////////////////////////////////////

  __forceinline embree_ostream operator <<(embree_ostream cout, const vfloat8& a) {
    return cout << "<" << a[0] << ", " << a[1] << ", " << a[2] << ", " << a[3] << ", " << a[4] << ", " << a[5] << ", " << a[6] << ", " << a[7] << ">";
  }
}

#undef vboolf
#undef vboold
#undef vint
#undef vuint
#undef vllong
#undef vfloat
#undef vdouble
