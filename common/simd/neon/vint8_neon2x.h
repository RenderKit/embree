// Copyright 2009-2021 Intel Corporation
// SPDX-License-Identifier: Apache-2.0

#pragma once

#include "neon_base.h"

#define vboolf vboolf_impl
#define vboold vboold_impl
#define vint vint_impl
#define vuint vuint_impl
#define vllong vllong_impl
#define vfloat vfloat_impl
#define vdouble vdouble_impl

namespace embree
{
  /* 8-wide NEON2X integer type */
  template<>
  struct vint<8>
  {
    ALIGNED_STRUCT_(32);

    typedef vboolf8 Bool;
    typedef vint8   Int;
    typedef vfloat8 Float;

    enum  { size = 8 };
    union {
      struct { int32x4_t vl, vh; } v;
      int i[8];
    };

    ////////////////////////////////////////////////////////////////////////////////
    /// Constructors, Assignment & Cast Operators
    ////////////////////////////////////////////////////////////////////////////////

    __forceinline vint() {}
    __forceinline vint(const vint8& a) { v.vl = a.v.vl; v.vh = a.v.vh; }
    __forceinline vint8& operator =(const vint8& a) { v.vl = a.v.vl; v.vh = a.v.vh; return *this; }

    __forceinline explicit vint(const vint4& a) { v.vl = a.v; v.vh = a.v; }
    __forceinline vint(const vint4& a, const vint4& b) { v.vl = a.v; v.vh = b.v; }
    __forceinline vint(const int32x4_t& a, const int32x4_t& b) { v.vl = a; v.vh = b; }

    __forceinline explicit vint(const int* a) { v.vl = vld1q_s32(a); v.vh = vld1q_s32(a+4); }
    __forceinline vint(int a) { v.vl = vdupq_n_s32(a); v.vh = vdupq_n_s32(a); }
    __forceinline vint(int a, int b) {
      int lo[4] = {a, b, a, b};
      int hi[4] = {a, b, a, b};
      v.vl = vld1q_s32(lo); v.vh = vld1q_s32(hi);
    }
    __forceinline vint(int a, int b, int c, int d) {
      int lo[4] = {a, b, c, d};
      v.vl = vld1q_s32(lo); v.vh = v.vl;
    }
    __forceinline vint(int a, int b, int c, int d, int e, int f, int g, int h) {
      int lo[4] = {a, b, c, d};
      int hi[4] = {e, f, g, h};
      v.vl = vld1q_s32(lo); v.vh = vld1q_s32(hi);
    }

    __forceinline explicit vint(float32x4_t a_lo, float32x4_t a_hi) { v.vl = vcvtq_s32_f32(a_lo); v.vh = vcvtq_s32_f32(a_hi); }
    __forceinline explicit vint(const vboolf8& a) { v.vl = vreinterpretq_s32_f32(a.v.vl); v.vh = vreinterpretq_s32_f32(a.v.vh); }
    __forceinline vint(uint32x4_t a) { v.vl = vreinterpretq_s32_u32(a); v.vh = vdupq_n_s32(0); }
    __forceinline vint(neon_uint32x8_t a) { v.vl = vreinterpretq_s32_u32(a.lo); v.vh = vreinterpretq_s32_u32(a.hi); }

    ////////////////////////////////////////////////////////////////////////////////
    /// Constants
    ////////////////////////////////////////////////////////////////////////////////

    __forceinline vint(ZeroTy)        { v.vl = vdupq_n_s32(0); v.vh = vdupq_n_s32(0); }
    __forceinline vint(OneTy)         { v.vl = vdupq_n_s32(1); v.vh = vdupq_n_s32(1); }
    __forceinline vint(PosInfTy)      { v.vl = vdupq_n_s32(pos_inf); v.vh = vdupq_n_s32(pos_inf); }
    __forceinline vint(NegInfTy)      { v.vl = vdupq_n_s32(neg_inf); v.vh = vdupq_n_s32(neg_inf); }
    __forceinline vint(StepTy) {
      int lo[4] = {0, 1, 2, 3};
      int hi[4] = {4, 5, 6, 7};
      v.vl = vld1q_s32(lo); v.vh = vld1q_s32(hi);
    }
    __forceinline vint(ReverseStepTy) {
      int lo[4] = {7, 6, 5, 4};
      int hi[4] = {3, 2, 1, 0};
      v.vl = vld1q_s32(lo); v.vh = vld1q_s32(hi);
    }
    __forceinline vint(UndefinedTy)   { v.vl = vdupq_n_s32(0); v.vh = vdupq_n_s32(0); }

    ////////////////////////////////////////////////////////////////////////////////
    /// Loads and Stores
    ////////////////////////////////////////////////////////////////////////////////

    static __forceinline vint8 load(const unsigned char* ptr)
    {
      vint8 r;
      uint8x8_t t0 = vld1_u8(ptr);
      uint16x8_t t1 = vmovl_u8(t0);
      r.v.vl = vreinterpretq_s32_u32(vmovl_u16(vget_low_u16(t1)));
      r.v.vh = vreinterpretq_s32_u32(vmovl_u16(vget_high_u16(t1)));
      return r;
    }
    static __forceinline vint8 loadu(const unsigned char* ptr) { return load(ptr); }

    static __forceinline vint8 load(const unsigned short* ptr)
    {
      vint8 r;
      r.v.vl = vreinterpretq_s32_u32(vmovl_u16(vld1_u16(ptr)));
      r.v.vh = vreinterpretq_s32_u32(vmovl_u16(vld1_u16(ptr + 4)));
      return r;
    }
    static __forceinline vint8 loadu(const unsigned short* ptr) { return load(ptr); }

    static __forceinline vint8 load(const void* ptr)
    {
      vint8 r;
      r.v.vl = vld1q_s32((const int*)ptr);
      r.v.vh = vld1q_s32((const int*)ptr + 4);
      return r;
    }
    static __forceinline vint8 loadu(const void* ptr)
    {
      vint8 r;
      r.v.vl = vld1q_s32((const int*)ptr);
      r.v.vh = vld1q_s32((const int*)ptr + 4);
      return r;
    }

    static __forceinline void store(void* ptr, const vint8& v)
    {
      vst1q_s32((int*)ptr, v.v.vl);
      vst1q_s32((int*)ptr + 4, v.v.vh);
    }

    static __forceinline void storeu(void* ptr, const vint8& v)
    {
      vst1q_s32((int*)ptr, v.v.vl);
      vst1q_s32((int*)ptr + 4, v.v.vh);
    }

    static __forceinline vint8 load(const vboolf8& mask, const void* ptr)
    {
      vint8 r;
      int32x4_t vl = vld1q_s32((const int*)ptr);
      int32x4_t vh = vld1q_s32((const int*)ptr + 4);
      r.v.vl = vreinterpretq_s32_u32(vandq_u32(vreinterpretq_u32_f32(mask.v.vl), vreinterpretq_u32_s32(vl)));
      r.v.vh = vreinterpretq_s32_u32(vandq_u32(vreinterpretq_u32_f32(mask.v.vh), vreinterpretq_u32_s32(vh)));
      return r;
    }
    static __forceinline vint8 loadu(const vboolf8& mask, const void* ptr) { return load(mask, ptr); }

    static __forceinline void store(const vboolf8& mask, void* ptr, const vint8& v)
    {
      vint8 cur = load(ptr);
      vint8 r;
      r.v.vl = vreinterpretq_s32_u32(vbslq_u32(vreinterpretq_u32_f32(mask.v.vl), vreinterpretq_u32_s32(v.v.vl), vreinterpretq_u32_s32(cur.v.vl)));
      r.v.vh = vreinterpretq_s32_u32(vbslq_u32(vreinterpretq_u32_f32(mask.v.vh), vreinterpretq_u32_s32(v.v.vh), vreinterpretq_u32_s32(cur.v.vh)));
      store(ptr, r);
    }
    static __forceinline void storeu(const vboolf8& mask, void* ptr, const vint8& v)
    {
      vint8 cur = loadu(ptr);
      vint8 r;
      r.v.vl = vreinterpretq_s32_u32(vbslq_u32(vreinterpretq_u32_f32(mask.v.vl), vreinterpretq_u32_s32(v.v.vl), vreinterpretq_u32_s32(cur.v.vl)));
      r.v.vh = vreinterpretq_s32_u32(vbslq_u32(vreinterpretq_u32_f32(mask.v.vh), vreinterpretq_u32_s32(v.v.vh), vreinterpretq_u32_s32(cur.v.vh)));
      storeu(ptr, r);
    }

    static __forceinline vint8 load_nt(void* ptr)
    {
      vint8 r;
      r.v.vl = vld1q_s32((const int*)ptr);
      r.v.vh = vld1q_s32((const int*)ptr + 4);
      return r;
    }

    static __forceinline void store_nt(void* ptr, const vint8& v)
    {
      vst1q_s32((int*)ptr, v.v.vl);
      vst1q_s32((int*)ptr + 4, v.v.vh);
    }

    static __forceinline void store(unsigned char* ptr, const vint8& i)
    {
      for (size_t j=0; j<8; j++)
        ptr[j] = i[j];
    }

    static __forceinline void store(unsigned short* ptr, const vint8& v)
    {
      for (size_t i=0;i<8;i++)
        ptr[i] = (unsigned short)v[i];
    }

    template<int scale = 4>
    static __forceinline vint8 gather(const int *const ptr, const vint8& index)
    {
      vint8 r;
      for (size_t i=0; i<8; i++)
        r[i] = *(int*)(((char*)ptr) + scale * index[i]);
      return r;
    }

    template<int scale = 4>
    static __forceinline vint8 gather(const vboolf8& mask, const int *const ptr, const vint8& index)
    {
      vint8 r = zero;
      for (size_t i=0; i<8; i++)
        if (likely(mask[i]))
          r[i] = *(int*)(((char*)ptr) + scale * index[i]);
      return r;
    }

    template<int scale = 4>
    static __forceinline void scatter(void* ptr, const vint8& ofs, const vint8& v)
    {
      for (size_t i=0; i<8; i++)
        *(int*)(((char*)ptr) + scale * ofs[i]) = v[i];
    }

    template<int scale = 4>
    static __forceinline void scatter(const vboolf8& mask, void* ptr, const vint8& ofs, const vint8& v)
    {
      for (size_t i=0; i<8; i++)
        if (likely(mask[i]))
          *(int*)(((char*)ptr) + scale * ofs[i]) = v[i];
    }

    static __forceinline vint8 broadcast64(const long long& a)
    {
      vint8 r;
      int64x2_t b = vdupq_n_s64(a);
      r.v.vl = vreinterpretq_s32_s64(b);
      r.v.vh = vreinterpretq_s32_s64(b);
      return r;
    }

    ////////////////////////////////////////////////////////////////////////////////
    /// Array Access
    ////////////////////////////////////////////////////////////////////////////////

    __forceinline const int& operator [](size_t index) const { assert(index < 8); return i[index]; }
    __forceinline       int& operator [](size_t index)       { assert(index < 8); return i[index]; }
  };

  ////////////////////////////////////////////////////////////////////////////////
  /// Unary Operators
  ////////////////////////////////////////////////////////////////////////////////

  static __forceinline vboolf8 asBool(const vint8& a)
  {
    vboolf8 r;
    r.v.vl = vreinterpretq_f32_s32(a.v.vl);
    r.v.vh = vreinterpretq_f32_s32(a.v.vh);
    return r;
  }

  __forceinline vint8 operator +(const vint8& a) { return a; }
  __forceinline vint8 operator -(const vint8& a)
  {
    vint8 r;
    r.v.vl = vnegq_s32(a.v.vl);
    r.v.vh = vnegq_s32(a.v.vh);
    return r;
  }
  __forceinline vint8 abs(const vint8& a)
  {
    vint8 r;
    r.v.vl = vabsq_s32(a.v.vl);
    r.v.vh = vabsq_s32(a.v.vh);
    return r;
  }

  ////////////////////////////////////////////////////////////////////////////////
  /// Binary Operators
  ////////////////////////////////////////////////////////////////////////////////

  __forceinline vint8 operator +(const vint8& a, const vint8& b)
  {
    vint8 r;
    r.v.vl = vaddq_s32(a.v.vl, b.v.vl);
    r.v.vh = vaddq_s32(a.v.vh, b.v.vh);
    return r;
  }
  __forceinline vint8 operator +(const vint8& a, int          b) { return a + vint8(b); }
  __forceinline vint8 operator +(int          a, const vint8& b) { return vint8(a) + b; }

  __forceinline vint8 operator -(const vint8& a, const vint8& b)
  {
    vint8 r;
    r.v.vl = vsubq_s32(a.v.vl, b.v.vl);
    r.v.vh = vsubq_s32(a.v.vh, b.v.vh);
    return r;
  }
  __forceinline vint8 operator -(const vint8& a, int          b) { return a - vint8(b); }
  __forceinline vint8 operator -(int          a, const vint8& b) { return vint8(a) - b; }

  __forceinline vint8 operator *(const vint8& a, const vint8& b)
  {
    vint8 r;
    r.v.vl = vmulq_s32(a.v.vl, b.v.vl);
    r.v.vh = vmulq_s32(a.v.vh, b.v.vh);
    return r;
  }
  __forceinline vint8 operator *(const vint8& a, int          b) { return a * vint8(b); }
  __forceinline vint8 operator *(int          a, const vint8& b) { return vint8(a) * b; }

  __forceinline vint8 operator &(const vint8& a, const vint8& b)
  {
    vint8 r;
    r.v.vl = vandq_s32(a.v.vl, b.v.vl);
    r.v.vh = vandq_s32(a.v.vh, b.v.vh);
    return r;
  }
  __forceinline vint8 operator &(const vint8& a, int          b) { return a & vint8(b); }
  __forceinline vint8 operator &(int          a, const vint8& b) { return vint8(a) & b; }

  __forceinline vint8 operator |(const vint8& a, const vint8& b)
  {
    vint8 r;
    r.v.vl = vorrq_s32(a.v.vl, b.v.vl);
    r.v.vh = vorrq_s32(a.v.vh, b.v.vh);
    return r;
  }
  __forceinline vint8 operator |(const vint8& a, int          b) { return a | vint8(b); }
  __forceinline vint8 operator |(int          a, const vint8& b) { return vint8(a) | b; }

  __forceinline vint8 operator ^(const vint8& a, const vint8& b)
  {
    vint8 r;
    r.v.vl = veorq_s32(a.v.vl, b.v.vl);
    r.v.vh = veorq_s32(a.v.vh, b.v.vh);
    return r;
  }
  __forceinline vint8 operator ^(const vint8& a, int          b) { return a ^ vint8(b); }
  __forceinline vint8 operator ^(int          a, const vint8& b) { return vint8(a) ^ b; }

  __forceinline vint8 operator <<(const vint8& a, int n)
  {
    vint8 r;
    r.v.vl = vshlq_s32(a.v.vl, vdupq_n_s32((int32_t)n));
    r.v.vh = vshlq_s32(a.v.vh, vdupq_n_s32((int32_t)n));
    return r;
  }
  __forceinline vint8 operator >>(const vint8& a, int n)
  {
    vint8 r;
    r.v.vl = vshlq_s32(a.v.vl, vdupq_n_s32(-(int32_t)n));
    r.v.vh = vshlq_s32(a.v.vh, vdupq_n_s32(-(int32_t)n));
    return r;
  }

  __forceinline vint8 operator <<(const vint8& a, const vint8& n)
  {
    vint8 r;
    r.v.vl = vshlq_s32(a.v.vl, n.v.vl);
    r.v.vh = vshlq_s32(a.v.vh, n.v.vh);
    return r;
  }
  __forceinline vint8 operator >>(const vint8& a, const vint8& n)
  {
    vint8 r;
    r.v.vl = vshlq_s32(a.v.vl, vnegq_s32(n.v.vl));
    r.v.vh = vshlq_s32(a.v.vh, vnegq_s32(n.v.vh));
    return r;
  }

  __forceinline vint8 sll(const vint8& a, int b)
  {
    vint8 r;
    r.v.vl = vshlq_s32(a.v.vl, vdupq_n_s32((int32_t)b));
    r.v.vh = vshlq_s32(a.v.vh, vdupq_n_s32((int32_t)b));
    return r;
  }
  __forceinline vint8 sra(const vint8& a, int b)
  {
    vint8 r;
    r.v.vl = vshlq_s32(a.v.vl, vdupq_n_s32(-(int32_t)b));
    r.v.vh = vshlq_s32(a.v.vh, vdupq_n_s32(-(int32_t)b));
    return r;
  }
  __forceinline vint8 srl(const vint8& a, int b)
  {
    vint8 r;
    r.v.vl = vreinterpretq_s32_u32(vshlq_u32(vreinterpretq_u32_s32(a.v.vl), vdupq_n_s32(-(int32_t)b)));
    r.v.vh = vreinterpretq_s32_u32(vshlq_u32(vreinterpretq_u32_s32(a.v.vh), vdupq_n_s32(-(int32_t)b)));
    return r;
  }

  __forceinline vint8 sll(const vint8& a, const vint8& b)
  {
    vint8 r;
    r.v.vl = vshlq_s32(a.v.vl, b.v.vl);
    r.v.vh = vshlq_s32(a.v.vh, b.v.vh);
    return r;
  }
  __forceinline vint8 sra(const vint8& a, const vint8& b)
  {
    vint8 r;
    r.v.vl = vshlq_s32(a.v.vl, vnegq_s32(b.v.vl));
    r.v.vh = vshlq_s32(a.v.vh, vnegq_s32(b.v.vh));
    return r;
  }
  __forceinline vint8 srl(const vint8& a, const vint8& b)
  {
    vint8 r;
    r.v.vl = vreinterpretq_s32_u32(vshlq_u32(vreinterpretq_u32_s32(a.v.vl), vnegq_s32(vreinterpretq_s32_u32(b.v.vl))));
    r.v.vh = vreinterpretq_s32_u32(vshlq_u32(vreinterpretq_u32_s32(a.v.vh), vnegq_s32(vreinterpretq_s32_u32(b.v.vh))));
    return r;
  }

  __forceinline vint8 min(const vint8& a, const vint8& b)
  {
    vint8 r;
    r.v.vl = vminq_s32(a.v.vl, b.v.vl);
    r.v.vh = vminq_s32(a.v.vh, b.v.vh);
    return r;
  }
  __forceinline vint8 min(const vint8& a, int          b) { return min(a,vint8(b)); }
  __forceinline vint8 min(int          a, const vint8& b) { return min(vint8(a),b); }

  __forceinline vint8 max(const vint8& a, const vint8& b)
  {
    vint8 r;
    r.v.vl = vmaxq_s32(a.v.vl, b.v.vl);
    r.v.vh = vmaxq_s32(a.v.vh, b.v.vh);
    return r;
  }
  __forceinline vint8 max(const vint8& a, int          b) { return max(a,vint8(b)); }
  __forceinline vint8 max(int          a, const vint8& b) { return max(vint8(a),b); }

  __forceinline vint8 umin(const vint8& a, const vint8& b)
  {
    vint8 r;
    r.v.vl = vreinterpretq_s32_u32(vminq_u32(vreinterpretq_u32_s32(a.v.vl), vreinterpretq_u32_s32(b.v.vl)));
    r.v.vh = vreinterpretq_s32_u32(vminq_u32(vreinterpretq_u32_s32(a.v.vh), vreinterpretq_u32_s32(b.v.vh)));
    return r;
  }
  __forceinline vint8 umax(const vint8& a, const vint8& b)
  {
    vint8 r;
    r.v.vl = vreinterpretq_s32_u32(vmaxq_u32(vreinterpretq_u32_s32(a.v.vl), vreinterpretq_u32_s32(b.v.vl)));
    r.v.vh = vreinterpretq_s32_u32(vmaxq_u32(vreinterpretq_u32_s32(a.v.vh), vreinterpretq_u32_s32(b.v.vh)));
    return r;
  }

  ////////////////////////////////////////////////////////////////////////////////
  /// Assignment Operators
  ////////////////////////////////////////////////////////////////////////////////

  __forceinline vint8& operator +=(vint8& a, const vint8& b) { return a = a + b; }
  __forceinline vint8& operator +=(vint8& a, int          b) { return a = a + b; }

  __forceinline vint8& operator -=(vint8& a, const vint8& b) { return a = a - b; }
  __forceinline vint8& operator -=(vint8& a, int          b) { return a = a - b; }

  __forceinline vint8& operator *=(vint8& a, const vint8& b) { return a = a * b; }
  __forceinline vint8& operator *=(vint8& a, int          b) { return a = a * b; }

  __forceinline vint8& operator &=(vint8& a, const vint8& b) { return a = a & b; }
  __forceinline vint8& operator &=(vint8& a, int          b) { return a = a & b; }

  __forceinline vint8& operator |=(vint8& a, const vint8& b) { return a = a | b; }
  __forceinline vint8& operator |=(vint8& a, int          b) { return a = a | b; }

  __forceinline vint8& operator <<=(vint8& a, const int b) { return a = a << b; }
  __forceinline vint8& operator >>=(vint8& a, const int b) { return a = a >> b; }

  ////////////////////////////////////////////////////////////////////////////////
  /// Comparison Operators + Select
  ////////////////////////////////////////////////////////////////////////////////

  static __forceinline vboolf8 operator ==(const vint8& a, const vint8& b)
  {
    vboolf8 r;
    r.v.vl = vreinterpretq_f32_u32(vceqq_s32(a.v.vl, b.v.vl));
    r.v.vh = vreinterpretq_f32_u32(vceqq_s32(a.v.vh, b.v.vh));
    return r;
  }
  static __forceinline vboolf8 operator !=(const vint8& a, const vint8& b) { return !(a == b); }
  static __forceinline vboolf8 operator < (const vint8& a, const vint8& b)
  {
    vboolf8 r;
    r.v.vl = vreinterpretq_f32_u32(vcltq_s32(a.v.vl, b.v.vl));
    r.v.vh = vreinterpretq_f32_u32(vcltq_s32(a.v.vh, b.v.vh));
    return r;
  }
  static __forceinline vboolf8 operator >=(const vint8& a, const vint8& b) { return !(a <  b); }
  static __forceinline vboolf8 operator > (const vint8& a, const vint8& b)
  {
    vboolf8 r;
    r.v.vl = vreinterpretq_f32_u32(vcgtq_s32(a.v.vl, b.v.vl));
    r.v.vh = vreinterpretq_f32_u32(vcgtq_s32(a.v.vh, b.v.vh));
    return r;
  }
  static __forceinline vboolf8 operator <=(const vint8& a, const vint8& b) { return !(a >  b); }

  static __forceinline vint8 select(const vboolf8& m, const vint8& t, const vint8& f)
  {
    vint8 r;
    r.v.vl = vreinterpretq_s32_u32(vbslq_u32(vreinterpretq_u32_f32(m.v.vl),
                                             vreinterpretq_u32_s32(t.v.vl),
                                             vreinterpretq_u32_s32(f.v.vl)));
    r.v.vh = vreinterpretq_s32_u32(vbslq_u32(vreinterpretq_u32_f32(m.v.vh),
                                             vreinterpretq_u32_s32(t.v.vh),
                                             vreinterpretq_u32_s32(f.v.vh)));
    return r;
  }

  template<int mask>
  __forceinline vint8 select(const vint8& t, const vint8& f)
  {
    return select(vboolf8(mask), t, f);
  }

  __forceinline vboolf8 operator ==(const vint8& a, int          b) { return a == vint8(b); }
  __forceinline vboolf8 operator ==(int          a, const vint8& b) { return vint8(a) == b; }

  __forceinline vboolf8 operator !=(const vint8& a, int          b) { return a != vint8(b); }
  __forceinline vboolf8 operator !=(int          a, const vint8& b) { return vint8(a) != b; }

  __forceinline vboolf8 operator < (const vint8& a, int          b) { return a <  vint8(b); }
  __forceinline vboolf8 operator < (int          a, const vint8& b) { return vint8(a) <  b; }

  __forceinline vboolf8 operator >=(const vint8& a, int          b) { return a >= vint8(b); }
  __forceinline vboolf8 operator >=(int          a, const vint8& b) { return vint8(a) >= b; }

  __forceinline vboolf8 operator > (const vint8& a, int          b) { return a >  vint8(b); }
  __forceinline vboolf8 operator > (int          a, const vint8& b) { return vint8(a) >  b; }

  __forceinline vboolf8 operator <=(const vint8& a, int          b) { return a <= vint8(b); }
  __forceinline vboolf8 operator <=(int          a, const vint8& b) { return vint8(a) <= b; }

  __forceinline vboolf8 eq(const vint8& a, const vint8& b) { return a == b; }
  __forceinline vboolf8 ne(const vint8& a, const vint8& b) { return a != b; }
  __forceinline vboolf8 lt(const vint8& a, const vint8& b) { return a <  b; }
  __forceinline vboolf8 ge(const vint8& a, const vint8& b) { return a >= b; }
  __forceinline vboolf8 gt(const vint8& a, const vint8& b) { return a >  b; }
  __forceinline vboolf8 le(const vint8& a, const vint8& b) { return a <= b; }

  static __forceinline vboolf8 eq(const vboolf8& mask, const vint8& a, const vint8& b) { return mask & (a == b); }
  static __forceinline vboolf8 ne(const vboolf8& mask, const vint8& a, const vint8& b) { return mask & (a != b); }
  static __forceinline vboolf8 lt(const vboolf8& mask, const vint8& a, const vint8& b) { return mask & (a <  b); }
  static __forceinline vboolf8 ge(const vboolf8& mask, const vint8& a, const vint8& b) { return mask & (a >= b); }
  static __forceinline vboolf8 gt(const vboolf8& mask, const vint8& a, const vint8& b) { return mask & (a >  b); }
  static __forceinline vboolf8 le(const vboolf8& mask, const vint8& a, const vint8& b) { return mask & (a <= b); }

  ////////////////////////////////////////////////////////////////////////////////
  /// Movement/Shifting/Shuffling Functions
  ////////////////////////////////////////////////////////////////////////////////

  __forceinline vint8 unpacklo(const vint8& a, const vint8& b)
  {
    vint8 r;
    r.v.vl = vreinterpretq_s32_u32(vzip1q_u32(vreinterpretq_u32_s32(a.v.vl), vreinterpretq_u32_s32(b.v.vl)));
    r.v.vh = vreinterpretq_s32_u32(vzip1q_u32(vreinterpretq_u32_s32(a.v.vh), vreinterpretq_u32_s32(b.v.vh)));
    return r;
  }
  __forceinline vint8 unpackhi(const vint8& a, const vint8& b)
  {
    vint8 r;
    r.v.vl = vreinterpretq_s32_u32(vzip2q_u32(vreinterpretq_u32_s32(a.v.vl), vreinterpretq_u32_s32(b.v.vl)));
    r.v.vh = vreinterpretq_s32_u32(vzip2q_u32(vreinterpretq_u32_s32(a.v.vh), vreinterpretq_u32_s32(b.v.vh)));
    return r;
  }

  template<int i>
  __forceinline vint8 shuffle(const vint8& v)
  {
    int32x4x2_t r = neon_permute_epi32<i,i,i,i>(v.v.vl, v.v.vh);
    vint8 result;
    result.v.vl = r.val[0];
    result.v.vh = r.val[1];
    return result;
  }

  template<int i0, int i1>
  __forceinline vint8 shuffle4(const vint8& v)
  {
    int32x4x2_t r = neon_permute2f128_si<i0,i1>(v.v.vl, v.v.vh);
    vint8 result;
    result.v.vl = r.val[0];
    result.v.vh = r.val[1];
    return result;
  }

  template<int i0, int i1>
  __forceinline vint8 shuffle4(const vint8& a, const vint8& b)
  {
    int32x4x2_t r = neon_permute2f128_si<i0,i1>(a.v.vl, a.v.vh, b.v.vl, b.v.vh);
    vint8 result;
    result.v.vl = r.val[0];
    result.v.vh = r.val[1];
    return result;
  }

  template<int i0, int i1, int i2, int i3>
  __forceinline vint8 shuffle(const vint8& v)
  {
    int32x4x2_t r = neon_permute_epi32<i0,i1,i2,i3>(v.v.vl, v.v.vh);
    vint8 result;
    result.v.vl = r.val[0];
    result.v.vh = r.val[1];
    return result;
  }

  template<int i0, int i1, int i2, int i3>
  __forceinline vint8 shuffle(const vint8& a, const vint8& b)
  {
    int32x4x2_t r = neon_shuffle_epi32<i0,i1,i2,i3>(a.v.vl, a.v.vh, b.v.vl, b.v.vh);
    vint8 result;
    result.v.vl = r.val[0];
    result.v.vh = r.val[1];
    return result;
  }

  template<> __forceinline vint8 shuffle<0, 0, 2, 2>(const vint8& v)
  {
    float32x4x2_t r = neon_moveldup_ps(vreinterpretq_f32_s32(v.v.vl), vreinterpretq_f32_s32(v.v.vh));
    vint8 result;
    result.v.vl = vreinterpretq_s32_f32(r.val[0]);
    result.v.vh = vreinterpretq_s32_f32(r.val[1]);
    return result;
  }
  template<> __forceinline vint8 shuffle<1, 1, 3, 3>(const vint8& v)
  {
    float32x4x2_t r = neon_movehdup_ps(vreinterpretq_f32_s32(v.v.vl), vreinterpretq_f32_s32(v.v.vh));
    vint8 result;
    result.v.vl = vreinterpretq_s32_f32(r.val[0]);
    result.v.vh = vreinterpretq_s32_f32(r.val[1]);
    return result;
  }
  template<> __forceinline vint8 shuffle<0, 1, 0, 1>(const vint8& v)
  {
    float32x4x2_t r = neon_movedup_pd_as_ps(vreinterpretq_f32_s32(v.v.vl), vreinterpretq_f32_s32(v.v.vh));
    vint8 result;
    result.v.vl = vreinterpretq_s32_f32(r.val[0]);
    result.v.vh = vreinterpretq_s32_f32(r.val[1]);
    return result;
  }

  __forceinline vint8 broadcast(const int* ptr)
  {
    vint8 r;
    int32x4_t b = vld1q_dup_s32(ptr);
    r.v.vl = b;
    r.v.vh = b;
    return r;
  }

  template<int i> __forceinline vint8 insert4(const vint8& a, const vint4& b) {
    vint8 r = a;
    if (i == 0) r.v.vl = b.v; else r.v.vh = b.v;
    return r;
  }
  template<int i> __forceinline vint4 extract4(const vint8& a) {
    vint4 r;
    if (i == 0) r.v = a.v.vl; else r.v = a.v.vh;
    return r;
  }
  template<> __forceinline vint4 extract4<0>(const vint8& a) { vint4 r; r.v = a.v.vl; return r; }

  __forceinline int toScalar(const vint8& v) { return vgetq_lane_s32(v.v.vl, 0); }

  ////////////////////////////////////////////////////////////////////////////////
  /// Reductions
  ////////////////////////////////////////////////////////////////////////////////

  __forceinline vint8 vreduce_min2(const vint8& v) { return min(v,shuffle<1,0,3,2>(v)); }
  __forceinline vint8 vreduce_min4(const vint8& v) { vint8 v1 = vreduce_min2(v); return min(v1,shuffle<2,3,0,1>(v1)); }
  __forceinline vint8 vreduce_min (const vint8& v) { vint8 v1 = vreduce_min4(v); return min(v1,shuffle4<1,0>(v1)); }

  __forceinline vint8 vreduce_max2(const vint8& v) { return max(v,shuffle<1,0,3,2>(v)); }
  __forceinline vint8 vreduce_max4(const vint8& v) { vint8 v1 = vreduce_max2(v); return max(v1,shuffle<2,3,0,1>(v1)); }
  __forceinline vint8 vreduce_max (const vint8& v) { vint8 v1 = vreduce_max4(v); return max(v1,shuffle4<1,0>(v1)); }

  __forceinline vint8 vreduce_add2(const vint8& v) { return v + shuffle<1,0,3,2>(v); }
  __forceinline vint8 vreduce_add4(const vint8& v) { vint8 v1 = vreduce_add2(v); return v1 + shuffle<2,3,0,1>(v1); }
  __forceinline vint8 vreduce_add (const vint8& v) { vint8 v1 = vreduce_add4(v); return v1 + shuffle4<1,0>(v1); }

  __forceinline int reduce_min(const vint8& v) { return toScalar(vreduce_min(v)); }
  __forceinline int reduce_max(const vint8& v) { return toScalar(vreduce_max(v)); }
  __forceinline int reduce_add(const vint8& v) { return toScalar(vreduce_add(v)); }

  __forceinline size_t select_min(const vint8& v) { return bsf(movemask(v == vreduce_min(v))); }
  __forceinline size_t select_max(const vint8& v) { return bsf(movemask(v == vreduce_max(v))); }

  __forceinline size_t select_min(const vboolf8& valid, const vint8& v) { const vint8 a = select(valid,v,vint8(pos_inf)); return bsf(movemask(valid & (a == vreduce_min(a)))); }
  __forceinline size_t select_max(const vboolf8& valid, const vint8& v) { const vint8 a = select(valid,v,vint8(neg_inf)); return bsf(movemask(valid & (a == vreduce_max(a)))); }

  ////////////////////////////////////////////////////////////////////////////////
  /// Sorting networks
  ////////////////////////////////////////////////////////////////////////////////

  __forceinline vint8 usort_ascending(const vint8& v)
  {
    const vint8 a0 = v;
    const vint8 b0 = shuffle<1,0,3,2>(a0);
    const vint8 c0 = umin(a0,b0);
    const vint8 d0 = umax(a0,b0);
    const vint8 a1 = select<0x99 /* 0b10011001 */>(c0,d0);
    const vint8 b1 = shuffle<2,3,0,1>(a1);
    const vint8 c1 = umin(a1,b1);
    const vint8 d1 = umax(a1,b1);
    const vint8 a2 = select<0xc3 /* 0b11000011 */>(c1,d1);
    const vint8 b2 = shuffle<1,0,3,2>(a2);
    const vint8 c2 = umin(a2,b2);
    const vint8 d2 = umax(a2,b2);
    const vint8 a3 = select<0xa5 /* 0b10100101 */>(c2,d2);
    const vint8 b3 = shuffle4<1,0>(a3);
    const vint8 c3 = umin(a3,b3);
    const vint8 d3 = umax(a3,b3);
    const vint8 a4 = select<0xf /* 0b00001111 */>(c3,d3);
    const vint8 b4 = shuffle<2,3,0,1>(a4);
    const vint8 c4 = umin(a4,b4);
    const vint8 d4 = umax(a4,b4);
    const vint8 a5 = select<0x33 /* 0b00110011 */>(c4,d4);
    const vint8 b5 = shuffle<1,0,3,2>(a5);
    const vint8 c5 = umin(a5,b5);
    const vint8 d5 = umax(a5,b5);
    const vint8 a6 = select<0x55 /* 0b01010101 */>(c5,d5);
    return a6;
  }

  __forceinline vint8 usort_descending(const vint8& v)
  {
    const vint8 a0 = v;
    const vint8 b0 = shuffle<1,0,3,2>(a0);
    const vint8 c0 = umax(a0,b0);
    const vint8 d0 = umin(a0,b0);
    const vint8 a1 = select<0x99 /* 0b10011001 */>(c0,d0);
    const vint8 b1 = shuffle<2,3,0,1>(a1);
    const vint8 c1 = umax(a1,b1);
    const vint8 d1 = umin(a1,b1);
    const vint8 a2 = select<0xc3 /* 0b11000011 */>(c1,d1);
    const vint8 b2 = shuffle<1,0,3,2>(a2);
    const vint8 c2 = umax(a2,b2);
    const vint8 d2 = umin(a2,b2);
    const vint8 a3 = select<0xa5 /* 0b10100101 */>(c2,d2);
    const vint8 b3 = shuffle4<1,0>(a3);
    const vint8 c3 = umax(a3,b3);
    const vint8 d3 = umin(a3,b3);
    const vint8 a4 = select<0xf /* 0b00001111 */>(c3,d3);
    const vint8 b4 = shuffle<2,3,0,1>(a4);
    const vint8 c4 = umax(a4,b4);
    const vint8 d4 = umin(a4,b4);
    const vint8 a5 = select<0x33 /* 0b00110011 */>(c4,d4);
    const vint8 b5 = shuffle<1,0,3,2>(a5);
    const vint8 c5 = umax(a5,b5);
    const vint8 d5 = umin(a5,b5);
    const vint8 a6 = select<0x55 /* 0b01010101 */>(c5,d5);
    return a6;
  }

  ////////////////////////////////////////////////////////////////////////////////
  /// Output Operators
  ////////////////////////////////////////////////////////////////////////////////

  __forceinline embree_ostream operator <<(embree_ostream cout, const vint8& a) {
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
