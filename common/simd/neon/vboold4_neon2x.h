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
  /* 4-wide native NEON bool type for 64bit data types */
  template<>
  struct vboold<4>
  {
    ALIGNED_STRUCT_(32);

    typedef vboold4 Bool;

    enum  { size = 4 };
    union {
      struct { float64x2_t vl, vh; } v;
      long long i[4];
    };

    ////////////////////////////////////////////////////////////////////////////////
    /// Constructors, Assignment & Cast Operators
    ////////////////////////////////////////////////////////////////////////////////

    __forceinline vboold() {}
    __forceinline vboold(const vboold4& a) { v = a.v; }
    __forceinline vboold4& operator =(const vboold4& a) { v = a.v; return *this; }

    __forceinline vboold(const float64x2_t& a_lo, const float64x2_t& a_hi) { v.vl = a_lo; v.vh = a_hi; }

    __forceinline vboold(int a)
    {
      assert(a >= 0 && a <= 255);
      v.vl = neon_lookupmask_pd(a & 0x3);
      v.vh = neon_lookupmask_pd(a >> 2);
    }

    ////////////////////////////////////////////////////////////////////////////////
    /// Constants
    ////////////////////////////////////////////////////////////////////////////////

    __forceinline vboold(FalseTy) { v.vl = vdupq_n_f64(0.0); v.vh = vdupq_n_f64(0.0); }
    __forceinline vboold(TrueTy) {
      uint64x2_t ones = vreinterpretq_u64_s32(vmvnq_s32(vdupq_n_s32(0)));
      v.vl = vreinterpretq_f64_u64(ones);
      v.vh = vreinterpretq_f64_u64(ones);
    }

    ////////////////////////////////////////////////////////////////////////////////
    /// Array Access
    ////////////////////////////////////////////////////////////////////////////////

    __forceinline bool       operator [](size_t index) const { assert(index < 4); return ((neon_movemask_pd(v.vl) | (neon_movemask_pd(v.vh) << 2)) >> index) & 1; }
    __forceinline long long& operator [](size_t index)       { assert(index < 4); return i[index]; }
  };

  ////////////////////////////////////////////////////////////////////////////////
  /// Unary Operators
  ////////////////////////////////////////////////////////////////////////////////

  __forceinline vboold4 operator !(const vboold4& a) {
    vboold4 r;
    r.v.vl = vreinterpretq_f64_u32(vmvnq_u32(vreinterpretq_u32_f64(a.v.vl)));
    r.v.vh = vreinterpretq_f64_u32(vmvnq_u32(vreinterpretq_u32_f64(a.v.vh)));
    return r;
  }

  ////////////////////////////////////////////////////////////////////////////////
  /// Binary Operators
  ////////////////////////////////////////////////////////////////////////////////

  __forceinline vboold4 operator &(const vboold4& a, const vboold4& b) {
    vboold4 r;
    r.v.vl = vreinterpretq_f64_u64(vandq_u64(vreinterpretq_u64_f64(a.v.vl), vreinterpretq_u64_f64(b.v.vl)));
    r.v.vh = vreinterpretq_f64_u64(vandq_u64(vreinterpretq_u64_f64(a.v.vh), vreinterpretq_u64_f64(b.v.vh)));
    return r;
  }
  __forceinline vboold4 operator |(const vboold4& a, const vboold4& b) {
    vboold4 r;
    r.v.vl = vreinterpretq_f64_u64(vorrq_u64(vreinterpretq_u64_f64(a.v.vl), vreinterpretq_u64_f64(b.v.vl)));
    r.v.vh = vreinterpretq_f64_u64(vorrq_u64(vreinterpretq_u64_f64(a.v.vh), vreinterpretq_u64_f64(b.v.vh)));
    return r;
  }
  __forceinline vboold4 operator ^(const vboold4& a, const vboold4& b) {
    vboold4 r;
    r.v.vl = vreinterpretq_f64_u64(veorq_u64(vreinterpretq_u64_f64(a.v.vl), vreinterpretq_u64_f64(b.v.vl)));
    r.v.vh = vreinterpretq_f64_u64(veorq_u64(vreinterpretq_u64_f64(a.v.vh), vreinterpretq_u64_f64(b.v.vh)));
    return r;
  }

  __forceinline vboold4 andn(const vboold4& a, const vboold4& b) {
    vboold4 r;
    r.v.vl = vreinterpretq_f64_u64(vbicq_u64(vreinterpretq_u64_f64(a.v.vl), vreinterpretq_u64_f64(b.v.vl)));
    r.v.vh = vreinterpretq_f64_u64(vbicq_u64(vreinterpretq_u64_f64(a.v.vh), vreinterpretq_u64_f64(b.v.vh)));
    return r;
  }

  __forceinline vboold4& operator &=(vboold4& a, const vboold4& b) { return a = a & b; }
  __forceinline vboold4& operator |=(vboold4& a, const vboold4& b) { return a = a | b; }
  __forceinline vboold4& operator ^=(vboold4& a, const vboold4& b) { return a = a ^ b; }

  ////////////////////////////////////////////////////////////////////////////////
  /// Comparison Operators + Select
  ////////////////////////////////////////////////////////////////////////////////

  __forceinline vboold4 operator !=(const vboold4& a, const vboold4& b) {
    vboold4 r;
    r.v.vl = vreinterpretq_f64_u64(veorq_u64(vreinterpretq_u64_f64(a.v.vl), vreinterpretq_u64_f64(b.v.vl)));
    r.v.vh = vreinterpretq_f64_u64(veorq_u64(vreinterpretq_u64_f64(a.v.vh), vreinterpretq_u64_f64(b.v.vh)));
    return r;
  }
  __forceinline vboold4 operator ==(const vboold4& a, const vboold4& b) {
    vboold4 r;
    uint64x2_t xor_lo = veorq_u64(vreinterpretq_u64_f64(a.v.vl), vreinterpretq_u64_f64(b.v.vl));
    uint64x2_t xor_hi = veorq_u64(vreinterpretq_u64_f64(a.v.vh), vreinterpretq_u64_f64(b.v.vh));
    r.v.vl = vreinterpretq_f64_u32(vmvnq_u32(vreinterpretq_u32_u64(xor_lo)));
    r.v.vh = vreinterpretq_f64_u32(vmvnq_u32(vreinterpretq_u32_u64(xor_hi)));
    return r;
  }

  __forceinline vboold4 select(const vboold4& mask, const vboold4& t, const vboold4& f) {
    vboold4 r;
    r.v.vl = vreinterpretq_f64_u64(vbslq_u64(vreinterpretq_u64_f64(mask.v.vl), vreinterpretq_u64_f64(t.v.vl), vreinterpretq_u64_f64(f.v.vl)));
    r.v.vh = vreinterpretq_f64_u64(vbslq_u64(vreinterpretq_u64_f64(mask.v.vh), vreinterpretq_u64_f64(t.v.vh), vreinterpretq_u64_f64(f.v.vh)));
    return r;
  }

  ////////////////////////////////////////////////////////////////////////////////
  /// Movement/Shifting/Shuffling Functions
  ////////////////////////////////////////////////////////////////////////////////

  __forceinline vboold4 unpacklo(const vboold4& a, const vboold4& b) {
    vboold4 r;
    r.v.vl = vreinterpretq_f64_u64(vzip1q_u64(vreinterpretq_u64_f64(a.v.vl), vreinterpretq_u64_f64(b.v.vl)));
    r.v.vh = vreinterpretq_f64_u64(vzip1q_u64(vreinterpretq_u64_f64(a.v.vh), vreinterpretq_u64_f64(b.v.vh)));
    return r;
  }
  __forceinline vboold4 unpackhi(const vboold4& a, const vboold4& b) {
    vboold4 r;
    r.v.vl = vreinterpretq_f64_u64(vzip2q_u64(vreinterpretq_u64_f64(a.v.vl), vreinterpretq_u64_f64(b.v.vl)));
    r.v.vh = vreinterpretq_f64_u64(vzip2q_u64(vreinterpretq_u64_f64(a.v.vh), vreinterpretq_u64_f64(b.v.vh)));
    return r;
  }

  template<int i0, int i1, int i2, int i3>
  __forceinline vboold4 shuffle(const vboold4& v) {
    float64x2_t src[4] = { v.v.vl, v.v.vl, v.v.vh, v.v.vh };
    double r0 = vgetq_lane_f64(src[i0], i0 & 1);
    double r1 = vgetq_lane_f64(src[i1], i1 & 1);
    double r2 = vgetq_lane_f64(src[i2], i2 & 1);
    double r3 = vgetq_lane_f64(src[i3], i3 & 1);
    vboold4 r;
    r.v.vl = vcombine_f64(vcreate_f64(*(const uint64_t*)&r0), vcreate_f64(*(const uint64_t*)&r1));
    r.v.vh = vcombine_f64(vcreate_f64(*(const uint64_t*)&r2), vcreate_f64(*(const uint64_t*)&r3));
    return r;
  }

  template<int i>
  __forceinline vboold4 shuffle(const vboold4& v) {
    return shuffle<i, i, i, i>(v);
  }

  ////////////////////////////////////////////////////////////////////////////////
  /// Reduction Operations
  ////////////////////////////////////////////////////////////////////////////////

  __forceinline unsigned int movemask(const vboold4& a) { return neon_movemask_pd(a.v.vl) | (neon_movemask_pd(a.v.vh) << 2); }
  __forceinline bool reduce_and(const vboold4& a) { return movemask(a) == 0xf; }
  __forceinline bool reduce_or (const vboold4& a) { return movemask(a) != 0; }

  __forceinline bool all (const vboold4& a) { return movemask(a) == 0xf; }
  __forceinline bool any (const vboold4& a) { return movemask(a) != 0; }
  __forceinline bool none(const vboold4& a) { return movemask(a) == 0; }

  __forceinline bool all (const vboold4& valid, const vboold4& b) { return all((!valid) | b); }
  __forceinline bool any (const vboold4& valid, const vboold4& b) { return any(valid & b); }
  __forceinline bool none(const vboold4& valid, const vboold4& b) { return none(valid & b); }

  __forceinline size_t popcnt(const vboold4& a) { return popcnt((size_t)movemask(a)); }

  ////////////////////////////////////////////////////////////////////////////////
  /// Get/Set Functions
  ////////////////////////////////////////////////////////////////////////////////

  __forceinline bool get(const vboold4& a, size_t index) { return a[index]; }
  __forceinline void set  (vboold4& a, size_t index)     { a[index] = -1; }
  __forceinline void clear(vboold4& a, size_t index)     { a[index] =  0; }

  ////////////////////////////////////////////////////////////////////////////////
  /// Output Operators
  ////////////////////////////////////////////////////////////////////////////////

  __forceinline embree_ostream operator <<(embree_ostream cout, const vboold4& a) {
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
