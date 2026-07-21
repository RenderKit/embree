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
  /* 4-wide native NEON long long type */
  template<>
  struct vllong<4>
  {
    ALIGNED_STRUCT_(32);

    typedef vboold4 Bool;

    enum  { size = 4 };
    union {
      struct { int32x4_t lo, hi; } v;
      long long i[4];
    };

    ////////////////////////////////////////////////////////////////////////////////
    /// Constructors, Assignment & Cast Operators
    ////////////////////////////////////////////////////////////////////////////////

    __forceinline vllong() {}
    __forceinline vllong(const vllong4& t) { v = t.v; }
    __forceinline vllong4& operator =(const vllong4& f) { v = f.v; return *this; }

    __forceinline vllong(const int32x4_t& a_lo, const int32x4_t& a_hi) { v.lo = a_lo; v.hi = a_hi; }

    __forceinline vllong(long long a) {
      int64x2_t t = vdupq_n_s64(a);
      v.lo = vreinterpretq_s32_s64(t);
      v.hi = vreinterpretq_s32_s64(t);
    }

    __forceinline vllong(long long a, long long b, long long c, long long d) {
      long long lo[2] = { a, b };
      long long hi[2] = { c, d };
      v.lo = vreinterpretq_s32_s64(vld1q_s64(lo));
      v.hi = vreinterpretq_s32_s64(vld1q_s64(hi));
    }

    ////////////////////////////////////////////////////////////////////////////////
    /// Constants
    ////////////////////////////////////////////////////////////////////////////////

    __forceinline vllong(ZeroTy) {
      int64x2_t zero = vdupq_n_s64(0);
      v.lo = vreinterpretq_s32_s64(zero);
      v.hi = vreinterpretq_s32_s64(zero);
    }
    __forceinline vllong(OneTy) {
      int64x2_t one = vdupq_n_s64(1);
      v.lo = vreinterpretq_s32_s64(one);
      v.hi = vreinterpretq_s32_s64(one);
    }
    __forceinline vllong(StepTy) {
      long long lo[2] = { 0, 1 };
      long long hi[2] = { 2, 3 };
      v.lo = vreinterpretq_s32_s64(vld1q_s64(lo));
      v.hi = vreinterpretq_s32_s64(vld1q_s64(hi));
    }
    __forceinline vllong(ReverseStepTy) {
      long long lo[2] = { 3, 2 };
      long long hi[2] = { 1, 0 };
      v.lo = vreinterpretq_s32_s64(vld1q_s64(lo));
      v.hi = vreinterpretq_s32_s64(vld1q_s64(hi));
    }

    ////////////////////////////////////////////////////////////////////////////////
    /// Loads and Stores
    ////////////////////////////////////////////////////////////////////////////////

    static __forceinline void store_nt(void* __restrict__ ptr, const vllong4& a) {
      vst1q_s64((long long*)ptr,     vreinterpretq_s64_s32(a.v.lo));
      vst1q_s64((long long*)ptr + 2, vreinterpretq_s64_s32(a.v.hi));
    }

    static __forceinline vllong4 loadu(const void* addr) {
      vllong4 r;
      r.v.lo = vreinterpretq_s32_s64(vld1q_s64((const long long*)addr));
      r.v.hi = vreinterpretq_s32_s64(vld1q_s64((const long long*)addr + 2));
      return r;
    }

    static __forceinline vllong4 load(const vllong4* addr) {
      return loadu((const void*)addr);
    }

    static __forceinline vllong4 load(const long long* addr) {
      vllong4 r;
      r.v.lo = vreinterpretq_s32_s64(vld1q_s64(addr));
      r.v.hi = vreinterpretq_s32_s64(vld1q_s64(addr + 2));
      return r;
    }

    static __forceinline void store(void* ptr, const vllong4& v_arg) {
      vst1q_s64((long long*)ptr,     vreinterpretq_s64_s32(v_arg.v.lo));
      vst1q_s64((long long*)ptr + 2, vreinterpretq_s64_s32(v_arg.v.hi));
    }

    static __forceinline void storeu(void* ptr, const vllong4& v_arg) {
      vst1q_s64((long long*)ptr,     vreinterpretq_s64_s32(v_arg.v.lo));
      vst1q_s64((long long*)ptr + 2, vreinterpretq_s64_s32(v_arg.v.hi));
    }

    static __forceinline void storeu(const vboold4& mask, long long* ptr, const vllong4& f) {
      vllong4 tmp = loadu(ptr);
      vllong4 r;
      r.v.lo = vreinterpretq_s32_s64(vbslq_s64(vreinterpretq_s64_f64(mask.v.vl), vreinterpretq_s64_s32(f.v.lo), vreinterpretq_s64_s32(tmp.v.lo)));
      r.v.hi = vreinterpretq_s32_s64(vbslq_s64(vreinterpretq_s64_f64(mask.v.vh), vreinterpretq_s64_s32(f.v.hi), vreinterpretq_s64_s32(tmp.v.hi)));
      storeu(ptr, r);
    }

    static __forceinline void store(const vboold4& mask, void* ptr, const vllong4& f) {
      vllong4 tmp = loadu(ptr);
      vllong4 r;
      r.v.lo = vreinterpretq_s32_s64(vbslq_s64(vreinterpretq_s64_f64(mask.v.vl), vreinterpretq_s64_s32(f.v.lo), vreinterpretq_s64_s32(tmp.v.lo)));
      r.v.hi = vreinterpretq_s32_s64(vbslq_s64(vreinterpretq_s64_f64(mask.v.vh), vreinterpretq_s64_s32(f.v.hi), vreinterpretq_s64_s32(tmp.v.hi)));
      store(ptr, r);
    }

    ////////////////////////////////////////////////////////////////////////////////
    /// Array Access
    ////////////////////////////////////////////////////////////////////////////////

    __forceinline       long long& operator [](size_t index)       { assert(index < 4); return i[index]; }
    __forceinline const long long& operator [](size_t index) const { assert(index < 4); return i[index]; }
  };

  ////////////////////////////////////////////////////////////////////////////////
  /// Select
  ////////////////////////////////////////////////////////////////////////////////

  __forceinline vllong4 select(const vboold4& m, const vllong4& t, const vllong4& f) {
    vllong4 r;
    uint64x2_t mask_lo = vreinterpretq_u64_f64(m.v.vl);
    uint64x2_t mask_hi = vreinterpretq_u64_f64(m.v.vh);
    r.v.lo = vreinterpretq_s32_s64(vbslq_s64(mask_lo, vreinterpretq_s64_s32(t.v.lo), vreinterpretq_s64_s32(f.v.lo)));
    r.v.hi = vreinterpretq_s32_s64(vbslq_s64(mask_hi, vreinterpretq_s64_s32(t.v.hi), vreinterpretq_s64_s32(f.v.hi)));
    return r;
  }

  ////////////////////////////////////////////////////////////////////////////////
  /// Unary Operators
  ////////////////////////////////////////////////////////////////////////////////

  __forceinline vboold4 asBool(const vllong4& a) {
    vboold4 r;
    r.v.vl = vreinterpretq_f64_s64(vshrq_n_s64(vreinterpretq_s64_s32(a.v.lo), 63));
    r.v.vh = vreinterpretq_f64_s64(vshrq_n_s64(vreinterpretq_s64_s32(a.v.hi), 63));
    return r;
  }

  __forceinline vllong4 operator +(const vllong4& a) { return a; }
  __forceinline vllong4 operator -(const vllong4& a) {
    vllong4 r;
    r.v.lo = vreinterpretq_s32_s64(vnegq_s64(vreinterpretq_s64_s32(a.v.lo)));
    r.v.hi = vreinterpretq_s32_s64(vnegq_s64(vreinterpretq_s64_s32(a.v.hi)));
    return r;
  }

  ////////////////////////////////////////////////////////////////////////////////
  /// Binary Operators
  ////////////////////////////////////////////////////////////////////////////////

  __forceinline vllong4 operator +(const vllong4& a, const vllong4& b) {
    vllong4 r;
    r.v.lo = vreinterpretq_s32_s64(vaddq_s64(vreinterpretq_s64_s32(a.v.lo), vreinterpretq_s64_s32(b.v.lo)));
    r.v.hi = vreinterpretq_s32_s64(vaddq_s64(vreinterpretq_s64_s32(a.v.hi), vreinterpretq_s64_s32(b.v.hi)));
    return r;
  }
  __forceinline vllong4 operator +(const vllong4& a, long long b) { return a + vllong4(b); }
  __forceinline vllong4 operator +(long long a, const vllong4& b) { return vllong4(a) + b; }

  __forceinline vllong4 operator -(const vllong4& a, const vllong4& b) {
    vllong4 r;
    r.v.lo = vreinterpretq_s32_s64(vsubq_s64(vreinterpretq_s64_s32(a.v.lo), vreinterpretq_s64_s32(b.v.lo)));
    r.v.hi = vreinterpretq_s32_s64(vsubq_s64(vreinterpretq_s64_s32(a.v.hi), vreinterpretq_s64_s32(b.v.hi)));
    return r;
  }
  __forceinline vllong4 operator -(const vllong4& a, long long b) { return a - vllong4(b); }
  __forceinline vllong4 operator -(long long a, const vllong4& b) { return vllong4(a) - b; }

  /* only low 32bit part */
  __forceinline vllong4 operator *(const vllong4& a, const vllong4& b) {
    vllong4 r;
    for (size_t i = 0; i < 4; i++)
      r[i] = (long long)(int)a[i] * (long long)(int)b[i];
    return r;
  }
  __forceinline vllong4 operator *(const vllong4& a, long long b) { return a * vllong4(b); }
  __forceinline vllong4 operator *(long long a, const vllong4& b) { return vllong4(a) * b; }

  __forceinline vllong4 operator &(const vllong4& a, const vllong4& b) {
    vllong4 r;
    r.v.lo = vandq_s32(a.v.lo, b.v.lo);
    r.v.hi = vandq_s32(a.v.hi, b.v.hi);
    return r;
  }
  __forceinline vllong4 operator &(const vllong4& a, long long b) { return a & vllong4(b); }
  __forceinline vllong4 operator &(long long a, const vllong4& b) { return vllong4(a) & b; }

  __forceinline vllong4 operator |(const vllong4& a, const vllong4& b) {
    vllong4 r;
    r.v.lo = vorrq_s32(a.v.lo, b.v.lo);
    r.v.hi = vorrq_s32(a.v.hi, b.v.hi);
    return r;
  }
  __forceinline vllong4 operator |(const vllong4& a, long long b) { return a | vllong4(b); }
  __forceinline vllong4 operator |(long long a, const vllong4& b) { return vllong4(a) | b; }

  __forceinline vllong4 operator ^(const vllong4& a, const vllong4& b) {
    vllong4 r;
    r.v.lo = veorq_s32(a.v.lo, b.v.lo);
    r.v.hi = veorq_s32(a.v.hi, b.v.hi);
    return r;
  }
  __forceinline vllong4 operator ^(const vllong4& a, long long b) { return a ^ vllong4(b); }
  __forceinline vllong4 operator ^(long long a, const vllong4& b) { return vllong4(a) ^ b; }

  __forceinline vllong4 operator <<(const vllong4& a, long long n) {
    vllong4 r;
    int64x2_t shift = vdupq_n_s64((int64_t)n);
    r.v.lo = vreinterpretq_s32_s64(vshlq_s64(vreinterpretq_s64_s32(a.v.lo), shift));
    r.v.hi = vreinterpretq_s32_s64(vshlq_s64(vreinterpretq_s64_s32(a.v.hi), shift));
    return r;
  }

  __forceinline vllong4 operator <<(const vllong4& a, const vllong4& n) {
    vllong4 r;
    r.v.lo = vreinterpretq_s32_s64(vshlq_s64(vreinterpretq_s64_s32(a.v.lo), vreinterpretq_s64_s32(n.v.lo)));
    r.v.hi = vreinterpretq_s32_s64(vshlq_s64(vreinterpretq_s64_s32(a.v.hi), vreinterpretq_s64_s32(n.v.hi)));
    return r;
  }

  __forceinline vllong4 srl(const vllong4& a, long long b) {
    vllong4 r;
    int64x2_t shift = vdupq_n_s64(-(int64_t)b);
    r.v.lo = vreinterpretq_s32_s64(vshlq_u64(vreinterpretq_u64_s32(a.v.lo), shift));
    r.v.hi = vreinterpretq_s32_s64(vshlq_u64(vreinterpretq_u64_s32(a.v.hi), shift));
    return r;
  }

  __forceinline vllong4 mask_and(const vboold4& m, const vllong4& c, const vllong4& a, const vllong4& b) { return select(m, a & b, c); }
  __forceinline vllong4 mask_or (const vboold4& m, const vllong4& c, const vllong4& a, const vllong4& b) { return select(m, a | b, c); }

  ////////////////////////////////////////////////////////////////////////////////
  /// Assignment Operators
  ////////////////////////////////////////////////////////////////////////////////

  __forceinline vllong4& operator +=(vllong4& a, const vllong4& b) { return a = a + b; }
  __forceinline vllong4& operator +=(vllong4& a, long long b) { return a = a + b; }

  __forceinline vllong4& operator -=(vllong4& a, const vllong4& b) { return a = a - b; }
  __forceinline vllong4& operator -=(vllong4& a, long long b) { return a = a - b; }

  __forceinline vllong4& operator *=(vllong4& a, const vllong4& b) { return a = a * b; }
  __forceinline vllong4& operator *=(vllong4& a, long long b) { return a = a * b; }

  __forceinline vllong4& operator &=(vllong4& a, const vllong4& b) { return a = a & b; }
  __forceinline vllong4& operator &=(vllong4& a, long long b) { return a = a & b; }

  __forceinline vllong4& operator |=(vllong4& a, const vllong4& b) { return a = a | b; }
  __forceinline vllong4& operator |=(vllong4& a, long long b) { return a = a | b; }

  __forceinline vllong4& operator <<=(vllong4& a, long long b) { return a = a << b; }

  ////////////////////////////////////////////////////////////////////////////////
  /// Comparison Operators
  ////////////////////////////////////////////////////////////////////////////////

  __forceinline vboold4 operator ==(const vllong4& a, const vllong4& b) {
    vboold4 r;
    r.v.vl = vreinterpretq_f64_u64(vceqq_s64(vreinterpretq_s64_s32(a.v.lo), vreinterpretq_s64_s32(b.v.lo)));
    r.v.vh = vreinterpretq_f64_u64(vceqq_s64(vreinterpretq_s64_s32(a.v.hi), vreinterpretq_s64_s32(b.v.hi)));
    return r;
  }
  __forceinline vboold4 operator !=(const vllong4& a, const vllong4& b) { return !(a == b); }
  __forceinline vboold4 operator > (const vllong4& a, const vllong4& b) {
    vboold4 r;
    r.v.vl = vreinterpretq_f64_u64(vcgtq_s64(vreinterpretq_s64_s32(a.v.lo), vreinterpretq_s64_s32(b.v.lo)));
    r.v.vh = vreinterpretq_f64_u64(vcgtq_s64(vreinterpretq_s64_s32(a.v.hi), vreinterpretq_s64_s32(b.v.hi)));
    return r;
  }
  __forceinline vboold4 operator < (const vllong4& a, const vllong4& b) { return b > a; }
  __forceinline vboold4 operator >=(const vllong4& a, const vllong4& b) { return !(a < b); }
  __forceinline vboold4 operator <=(const vllong4& a, const vllong4& b) { return !(a > b); }

  __forceinline vboold4 operator ==(const vllong4& a, long long b) { return a == vllong4(b); }
  __forceinline vboold4 operator ==(long long a, const vllong4& b) { return vllong4(a) == b; }

  __forceinline vboold4 operator !=(const vllong4& a, long long b) { return a != vllong4(b); }
  __forceinline vboold4 operator !=(long long a, const vllong4& b) { return vllong4(a) != b; }

  __forceinline vboold4 operator > (const vllong4& a, long long b) { return a >  vllong4(b); }
  __forceinline vboold4 operator > (long long a, const vllong4& b) { return vllong4(a) >  b; }

  __forceinline vboold4 operator < (const vllong4& a, long long b) { return a <  vllong4(b); }
  __forceinline vboold4 operator < (long long a, const vllong4& b) { return vllong4(a) <  b; }

  __forceinline vboold4 operator >=(const vllong4& a, long long b) { return a >= vllong4(b); }
  __forceinline vboold4 operator >=(long long a, const vllong4& b) { return vllong4(a) >= b; }

  __forceinline vboold4 operator <=(const vllong4& a, long long b) { return a <= vllong4(b); }
  __forceinline vboold4 operator <=(long long a, const vllong4& b) { return vllong4(a) <= b; }

  __forceinline vboold4 eq(const vllong4& a, const vllong4& b) { return a == b; }
  __forceinline vboold4 ne(const vllong4& a, const vllong4& b) { return a != b; }
  __forceinline vboold4 lt(const vllong4& a, const vllong4& b) { return a <  b; }
  __forceinline vboold4 ge(const vllong4& a, const vllong4& b) { return a >= b; }
  __forceinline vboold4 gt(const vllong4& a, const vllong4& b) { return a >  b; }
  __forceinline vboold4 le(const vllong4& a, const vllong4& b) { return a <= b; }

  __forceinline vboold4 eq(const vboold4& mask, const vllong4& a, const vllong4& b) { return mask & (a == b); }
  __forceinline vboold4 ne(const vboold4& mask, const vllong4& a, const vllong4& b) { return mask & (a != b); }
  __forceinline vboold4 lt(const vboold4& mask, const vllong4& a, const vllong4& b) { return mask & (a <  b); }
  __forceinline vboold4 ge(const vboold4& mask, const vllong4& a, const vllong4& b) { return mask & (a >= b); }
  __forceinline vboold4 gt(const vboold4& mask, const vllong4& a, const vllong4& b) { return mask & (a >  b); }
  __forceinline vboold4 le(const vboold4& mask, const vllong4& a, const vllong4& b) { return mask & (a <= b); }

  ////////////////////////////////////////////////////////////////////////////////
  // Movement/Shifting/Shuffling Functions
  ////////////////////////////////////////////////////////////////////////////////

  template<int i0, int i1>
  __forceinline vllong4 shuffle(const vllong4& v) {
    vllong4 r;
    float64x1_t lo0 = i0 ? vget_high_f64(vreinterpretq_f64_s32(v.v.lo)) : vget_low_f64(vreinterpretq_f64_s32(v.v.lo));
    float64x1_t lo1 = i1 ? vget_high_f64(vreinterpretq_f64_s32(v.v.lo)) : vget_low_f64(vreinterpretq_f64_s32(v.v.lo));
    float64x1_t hi0 = i0 ? vget_high_f64(vreinterpretq_f64_s32(v.v.hi)) : vget_low_f64(vreinterpretq_f64_s32(v.v.hi));
    float64x1_t hi1 = i1 ? vget_high_f64(vreinterpretq_f64_s32(v.v.hi)) : vget_low_f64(vreinterpretq_f64_s32(v.v.hi));
    r.v.lo = vreinterpretq_s32_f64(vcombine_f64(lo0, lo1));
    r.v.hi = vreinterpretq_s32_f64(vcombine_f64(hi0, hi1));
    return r;
  }

  template<int i>
  __forceinline vllong4 shuffle(const vllong4& v) {
    return shuffle<i, i>(v);
  }

  template<int i0, int i1>
  __forceinline vllong4 shuffle2(const vllong4& v) {
    vllong4 r;
    r.v.lo = i0 ? v.v.hi : v.v.lo;
    r.v.hi = i1 ? v.v.hi : v.v.lo;
    return r;
  }

  __forceinline long long toScalar(const vllong4& v) {
    return vgetq_lane_s64(vreinterpretq_s64_s32(v.v.lo), 0);
  }

  ////////////////////////////////////////////////////////////////////////////////
  /// Reductions
  ////////////////////////////////////////////////////////////////////////////////

  __forceinline vllong4 vreduce_and2(const vllong4& x) { return x & shuffle<1,0>(x); }
  __forceinline vllong4 vreduce_and (const vllong4& y) { const vllong4 x = vreduce_and2(y); return x & shuffle2<1,0>(x); }

  __forceinline vllong4 vreduce_or2(const vllong4& x) { return x | shuffle<1,0>(x); }
  __forceinline vllong4 vreduce_or (const vllong4& y) { const vllong4 x = vreduce_or2(y); return x | shuffle2<1,0>(x); }

  __forceinline vllong4 vreduce_add2(const vllong4& x) { return x + shuffle<1,0>(x); }
  __forceinline vllong4 vreduce_add (const vllong4& y) {
    const vllong4 x = vreduce_add2(y);
    long long lo = vgetq_lane_s64(vreinterpretq_s64_s32(x.v.lo), 0) + vgetq_lane_s64(vreinterpretq_s64_s32(x.v.lo), 1);
    long long hi = vgetq_lane_s64(vreinterpretq_s64_s32(x.v.hi), 0) + vgetq_lane_s64(vreinterpretq_s64_s32(x.v.hi), 1);
    long long sum = lo + hi;
    return vllong4(sum, sum, sum, sum);
  }

  __forceinline long long reduce_add(const vllong4& a) { return toScalar(vreduce_add(a)); }
  __forceinline long long reduce_or (const vllong4& a) { return toScalar(vreduce_or(a)); }
  __forceinline long long reduce_and(const vllong4& a) { return toScalar(vreduce_and(a)); }

  ////////////////////////////////////////////////////////////////////////////////
  /// Output Operators
  ////////////////////////////////////////////////////////////////////////////////

  __forceinline embree_ostream operator <<(embree_ostream cout, const vllong4& v)
  {
    cout << "<" << v[0];
    for (size_t i=1; i<4; i++) cout << ", " << v[i];
    cout << ">";
    return cout;
  }
}

#undef vboolf
#undef vboold
#undef vint
#undef vuint
#undef vllong
#undef vfloat
#undef vdouble
