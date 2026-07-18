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
  /* 8-wide NEON2X bool type */
  template<>
  struct vboolf<8>
  {
    ALIGNED_STRUCT_(32);

    typedef vboolf8 Bool;
    typedef vint8   Int;
    typedef vfloat8 Float;

    enum  { size = 8 };
    union {
      struct { float32x4_t vl, vh; } v;
      int i[8];
    };

    ////////////////////////////////////////////////////////////////////////////////
    /// Constructors, Assignment & Cast Operators
    ////////////////////////////////////////////////////////////////////////////////

    __forceinline vboolf() {}
    __forceinline vboolf(const vboolf8& a) { v.vl = a.v.vl; v.vh = a.v.vh; }
    __forceinline vboolf8& operator =(const vboolf8& a) { v.vl = a.v.vl; v.vh = a.v.vh; return *this; }

    __forceinline vboolf(float32x4x2_t a) { v.vl = a.val[0]; v.vh = a.val[1]; }
    __forceinline operator float32x4x2_t() const { float32x4x2_t r; r.val[0] = v.vl; r.val[1] = v.vh; return r; }

    __forceinline vboolf(int a)
    {
      assert(a >= 0 && a <= 255);
      v.vl = neon_lookupmask_ps(a & 0xF);
      v.vh = neon_lookupmask_ps(a >> 4);
    }

    __forceinline vboolf(const vboolf4& a) { v.vl = a.v; v.vh = a.v; }
    __forceinline vboolf(const vboolf4& a, const vboolf4& b) { v.vl = a.v; v.vh = b.v; }
    __forceinline vboolf(float32x4_t a, float32x4_t b) { v.vl = a; v.vh = b; }

    __forceinline vboolf(bool a) { vboolf4 t(a); v.vl = t.v; v.vh = t.v; }
    __forceinline vboolf(bool a, bool b) { vboolf4 t(a, b); v.vl = t.v; v.vh = t.v; }
    __forceinline vboolf(bool a, bool b, bool c, bool d) { v.vl = vboolf4(a, b).v; v.vh = vboolf4(c, d).v; }
    __forceinline vboolf(bool a, bool b, bool c, bool d, bool e, bool f, bool g, bool h) { v.vl = vboolf4(a, b, c, d).v; v.vh = vboolf4(e, f, g, h).v; }

    __forceinline neon_uint32x8_t mask32() const {
      neon_uint32x8_t r;
      r.lo = vreinterpretq_u32_f32(v.vl);
      r.hi = vreinterpretq_u32_f32(v.vh);
      return r;
    }

    ////////////////////////////////////////////////////////////////////////////////
    /// Constants
    ////////////////////////////////////////////////////////////////////////////////

    __forceinline vboolf(FalseTy) { v.vl = vdupq_n_f32(0.0f); v.vh = vdupq_n_f32(0.0f); }
    __forceinline vboolf(TrueTy)  { v.vl = vreinterpretq_f32_s32(vdupq_n_s32(-1)); v.vh = vreinterpretq_f32_s32(vdupq_n_s32(-1)); }

    ////////////////////////////////////////////////////////////////////////////////
    /// Array Access
    ////////////////////////////////////////////////////////////////////////////////

    __forceinline bool operator [](size_t index) const {
      assert(index < 8);
      uint32_t mask = neon_movemask_ps(v.vl) | (neon_movemask_ps(v.vh) << 4);
      return (mask >> index) & 1;
    }
    __forceinline int& operator [](size_t index)       { assert(index < 8); return i[index]; }
  };

  ////////////////////////////////////////////////////////////////////////////////
  /// Unary Operators
  ////////////////////////////////////////////////////////////////////////////////

  __forceinline vboolf8 operator !(const vboolf8& a) {
    vboolf8 r;
    r.v.vl = vreinterpretq_f32_u32(veorq_u32(vreinterpretq_u32_f32(a.v.vl), vreinterpretq_u32_f32(vboolf8(embree::True).v.vl)));
    r.v.vh = vreinterpretq_f32_u32(veorq_u32(vreinterpretq_u32_f32(a.v.vh), vreinterpretq_u32_f32(vboolf8(embree::True).v.vh)));
    return r;
  }

  ////////////////////////////////////////////////////////////////////////////////
  /// Binary Operators
  ////////////////////////////////////////////////////////////////////////////////

  __forceinline vboolf8 operator &(const vboolf8& a, const vboolf8& b) {
    vboolf8 r;
    r.v.vl = vreinterpretq_f32_u32(vandq_u32(vreinterpretq_u32_f32(a.v.vl), vreinterpretq_u32_f32(b.v.vl)));
    r.v.vh = vreinterpretq_f32_u32(vandq_u32(vreinterpretq_u32_f32(a.v.vh), vreinterpretq_u32_f32(b.v.vh)));
    return r;
  }
  __forceinline vboolf8 operator |(const vboolf8& a, const vboolf8& b) {
    vboolf8 r;
    r.v.vl = vreinterpretq_f32_u32(vorrq_u32(vreinterpretq_u32_f32(a.v.vl), vreinterpretq_u32_f32(b.v.vl)));
    r.v.vh = vreinterpretq_f32_u32(vorrq_u32(vreinterpretq_u32_f32(a.v.vh), vreinterpretq_u32_f32(b.v.vh)));
    return r;
  }
  __forceinline vboolf8 operator ^(const vboolf8& a, const vboolf8& b) {
    vboolf8 r;
    r.v.vl = vreinterpretq_f32_u32(veorq_u32(vreinterpretq_u32_f32(a.v.vl), vreinterpretq_u32_f32(b.v.vl)));
    r.v.vh = vreinterpretq_f32_u32(veorq_u32(vreinterpretq_u32_f32(a.v.vh), vreinterpretq_u32_f32(b.v.vh)));
    return r;
  }

  __forceinline vboolf8 andn(const vboolf8& a, const vboolf8& b) {
    vboolf8 r;
    r.v.vl = vreinterpretq_f32_u32(vbicq_u32(vreinterpretq_u32_f32(a.v.vl), vreinterpretq_u32_f32(b.v.vl)));
    r.v.vh = vreinterpretq_f32_u32(vbicq_u32(vreinterpretq_u32_f32(a.v.vh), vreinterpretq_u32_f32(b.v.vh)));
    return r;
  }

  __forceinline vboolf8& operator &=(vboolf8& a, const vboolf8& b) { return a = a & b; }
  __forceinline vboolf8& operator |=(vboolf8& a, const vboolf8& b) { return a = a | b; }
  __forceinline vboolf8& operator ^=(vboolf8& a, const vboolf8& b) { return a = a ^ b; }

  ////////////////////////////////////////////////////////////////////////////////
  /// Comparison Operators + Select
  ////////////////////////////////////////////////////////////////////////////////

  __forceinline vboolf8 operator !=(const vboolf8& a, const vboolf8& b) {
    vboolf8 r;
    r.v.vl = vreinterpretq_f32_u32(veorq_u32(vreinterpretq_u32_f32(a.v.vl), vreinterpretq_u32_f32(b.v.vl)));
    r.v.vh = vreinterpretq_f32_u32(veorq_u32(vreinterpretq_u32_f32(a.v.vh), vreinterpretq_u32_f32(b.v.vh)));
    return r;
  }
  __forceinline vboolf8 operator ==(const vboolf8& a, const vboolf8& b) {
    vboolf8 r;
    r.v.vl = vreinterpretq_f32_u32(vceqq_s32(vreinterpretq_s32_f32(a.v.vl), vreinterpretq_s32_f32(b.v.vl)));
    r.v.vh = vreinterpretq_f32_u32(vceqq_s32(vreinterpretq_s32_f32(a.v.vh), vreinterpretq_s32_f32(b.v.vh)));
    return r;
  }

  __forceinline vboolf8 select(const vboolf8& mask, const vboolf8& t, const vboolf8& f) {
    vboolf8 r;
    r.v.vl = vreinterpretq_f32_u32(vbslq_u32(vreinterpretq_u32_f32(mask.v.vl), vreinterpretq_u32_f32(t.v.vl), vreinterpretq_u32_f32(f.v.vl)));
    r.v.vh = vreinterpretq_f32_u32(vbslq_u32(vreinterpretq_u32_f32(mask.v.vh), vreinterpretq_u32_f32(t.v.vh), vreinterpretq_u32_f32(f.v.vh)));
    return r;
  }

  ////////////////////////////////////////////////////////////////////////////////
  /// Movement/Shifting/Shuffling Functions
  ////////////////////////////////////////////////////////////////////////////////

  __forceinline vboolf8 unpacklo(const vboolf8& a, const vboolf8& b) {
    vboolf8 result;
    result.v.vl = unpacklo(vboolf4(a.v.vl), vboolf4(b.v.vl));
    result.v.vh = unpacklo(vboolf4(a.v.vh), vboolf4(b.v.vh));
    return result;
  }
  __forceinline vboolf8 unpackhi(const vboolf8& a, const vboolf8& b) {
    vboolf8 result;
    result.v.vl = unpackhi(vboolf4(a.v.vl), vboolf4(b.v.vl));
    result.v.vh = unpackhi(vboolf4(a.v.vh), vboolf4(b.v.vh));
    return result;
  }

  template<int i>
  __forceinline vboolf8 shuffle(const vboolf8& v) {
    float32x4x2_t r = neon_permute_ps<i,i,i,i>(v.v.vl, v.v.vh);
    vboolf8 result;
    result.v.vl = r.val[0];
    result.v.vh = r.val[1];
    return result;
  }

  template<int i0, int i1>
  __forceinline vboolf8 shuffle4(const vboolf8& v) {
    float32x4x2_t r = neon_permute2f128_ps<i0,i1>(v.v.vl, v.v.vh);
    vboolf8 result;
    result.v.vl = r.val[0];
    result.v.vh = r.val[1];
    return result;
  }

  template<int i0, int i1>
  __forceinline vboolf8 shuffle4(const vboolf8& a, const vboolf8& b) {
    float32x4x2_t r = neon_permute2f128_ps<i0,i1>(a.v.vl, a.v.vh, b.v.vl, b.v.vh);
    vboolf8 result;
    result.v.vl = r.val[0];
    result.v.vh = r.val[1];
    return result;
  }

  template<int i0, int i1, int i2, int i3>
  __forceinline vboolf8 shuffle(const vboolf8& v) {
    float32x4x2_t r = neon_permute_ps<i0,i1,i2,i3>(v.v.vl, v.v.vh);
    vboolf8 result;
    result.v.vl = r.val[0];
    result.v.vh = r.val[1];
    return result;
  }

  template<int i0, int i1, int i2, int i3>
  __forceinline vboolf8 shuffle(const vboolf8& a, const vboolf8& b) {
    float32x4x2_t r = neon_shuffle_ps<i0,i1,i2,i3>(a.v.vl, a.v.vh, b.v.vl, b.v.vh);
    vboolf8 result;
    result.v.vl = r.val[0];
    result.v.vh = r.val[1];
    return result;
  }

  template<> __forceinline vboolf8 shuffle<0, 0, 2, 2>(const vboolf8& v) {
    float32x4x2_t r = neon_moveldup_ps(v.v.vl, v.v.vh);
    vboolf8 result;
    result.v.vl = r.val[0];
    result.v.vh = r.val[1];
    return result;
  }
  template<> __forceinline vboolf8 shuffle<1, 1, 3, 3>(const vboolf8& v) {
    float32x4x2_t r = neon_movehdup_ps(v.v.vl, v.v.vh);
    vboolf8 result;
    result.v.vl = r.val[0];
    result.v.vh = r.val[1];
    return result;
  }
  template<> __forceinline vboolf8 shuffle<0, 1, 0, 1>(const vboolf8& v) {
    float32x4x2_t r = neon_movedup_pd_as_ps(v.v.vl, v.v.vh);
    vboolf8 result;
    result.v.vl = r.val[0];
    result.v.vh = r.val[1];
    return result;
  }

  template<int i> __forceinline vboolf8 insert4(const vboolf8& a, const vboolf4& b) {
    vboolf8 r = a;
    if (i == 0) r.v.vl = b.v; else r.v.vh = b.v;
    return r;
  }
  template<int i> __forceinline vboolf4 extract4(const vboolf8& a) {
    vboolf4 r;
    if (i == 0) r.v = a.v.vl; else r.v = a.v.vh;
    return r;
  }
  template<>      __forceinline vboolf4 extract4<0>(const vboolf8& a) { vboolf4 r; r.v = a.v.vl; return r; }

  ////////////////////////////////////////////////////////////////////////////////
  /// Mask Operations
  ////////////////////////////////////////////////////////////////////////////////

  __forceinline unsigned int movemask(const vboolf8& a) { return neon_movemask_ps(a.v.vl) | (neon_movemask_ps(a.v.vh) << 4); }
  __forceinline size_t       popcnt  (const vboolf8& a) { return popcnt((size_t)movemask(a)); }

  ////////////////////////////////////////////////////////////////////////////////
  /// Reduction Operations
  ////////////////////////////////////////////////////////////////////////////////

  __forceinline bool reduce_and(const vboolf8& a) { return movemask(a) == 0xff; }
  __forceinline bool reduce_or (const vboolf8& a) { return movemask(a) != 0; }

  __forceinline bool all (const vboolf8& a) { return movemask(a) == 0xff; }
  __forceinline bool any (const vboolf8& a) { return movemask(a) != 0; }
  __forceinline bool none(const vboolf8& a) { return movemask(a) == 0; }

  __forceinline bool all (const vboolf8& valid, const vboolf8& b) { return all((!valid) | b); }
  __forceinline bool any (const vboolf8& valid, const vboolf8& b) { return any(valid & b); }
  __forceinline bool none(const vboolf8& valid, const vboolf8& b) { return none(valid & b); }

  ////////////////////////////////////////////////////////////////////////////////
  /// Get/Set Functions
  ////////////////////////////////////////////////////////////////////////////////

  __forceinline bool get(const vboolf8& a, size_t index) { return a[index]; }
  __forceinline void set(vboolf8& a, size_t index)       { a[index] = -1; }
  __forceinline void clear(vboolf8& a, size_t index)     { a[index] =  0; }

  ////////////////////////////////////////////////////////////////////////////////
  /// Output Operators
  ////////////////////////////////////////////////////////////////////////////////

  __forceinline embree_ostream operator <<(embree_ostream cout, const vboolf8& a) {
    return cout << "<" << a[0] << ", " << a[1] << ", " << a[2] << ", " << a[3] << ", "
                       << a[4] << ", " << a[5] << ", " << a[6] << ", " << a[7] << ">";
  }
}

#undef vboolf
#undef vboold
#undef vint
#undef vuint
#undef vllong
#undef vfloat
#undef vdouble
