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
  /* 4-wide native NEON double type */
  template<>
  struct vdouble<4>
  {
    ALIGNED_STRUCT_(32);

    typedef vboold4 Bool;

    enum  { size = 4 };
    union {
      __m256d v;
      double i[4];
    };

    ////////////////////////////////////////////////////////////////////////////////
    /// Constructors, Assignment & Cast Operators
    ////////////////////////////////////////////////////////////////////////////////

    __forceinline vdouble() {}
    __forceinline vdouble(const vdouble4& t) { v = t.v; }
    __forceinline vdouble4& operator =(const vdouble4& f) { v = f.v; return *this; }

    __forceinline vdouble(const __m256d& t) { v = t; }
    __forceinline operator __m256d() const { return v; }

    __forceinline vdouble(double a) {
      float64x2_t t = vdupq_n_f64(a);
      v.lo = t; v.hi = t;
    }

    __forceinline vdouble(double a, double b, double c, double d) {
      double lo[2] = {a, b};
      double hi[2] = {c, d};
      v.lo = vld1q_f64(lo);
      v.hi = vld1q_f64(hi);
    }

    ////////////////////////////////////////////////////////////////////////////////
    /// Constants
    ////////////////////////////////////////////////////////////////////////////////

    __forceinline vdouble(ZeroTy) { v.lo = vdupq_n_f64(0.0); v.hi = vdupq_n_f64(0.0); }
    __forceinline vdouble(OneTy)  { v.lo = vdupq_n_f64(1.0); v.hi = vdupq_n_f64(1.0); }
    __forceinline vdouble(StepTy) {
      double lo[2] = {0.0, 1.0};
      double hi[2] = {2.0, 3.0};
      v.lo = vld1q_f64(lo);
      v.hi = vld1q_f64(hi);
    }
    __forceinline vdouble(ReverseStepTy) {
      double lo[2] = {3.0, 2.0};
      double hi[2] = {1.0, 0.0};
      v.lo = vld1q_f64(lo);
      v.hi = vld1q_f64(hi);
    }

    ////////////////////////////////////////////////////////////////////////////////
    /// Loads and Stores
    ////////////////////////////////////////////////////////////////////////////////

    static __forceinline void store_nt(double *__restrict__ ptr, const vdouble4& a) {
      vst1q_f64(ptr,     a.v.lo);
      vst1q_f64(ptr + 2, a.v.hi);
    }

    static __forceinline vdouble4 loadu(const double* addr) {
      vdouble4 r;
      r.v.lo = vld1q_f64(addr);
      r.v.hi = vld1q_f64(addr + 2);
      return r;
    }

    static __forceinline vdouble4 load(const vdouble4* addr) {
      return loadu((const double*)addr);
    }

    static __forceinline vdouble4 load(const double* addr) {
      vdouble4 r;
      r.v.lo = vld1q_f64(addr);
      r.v.hi = vld1q_f64(addr + 2);
      return r;
    }

    static __forceinline void store(double* ptr, const vdouble4& v_arg) {
      vst1q_f64(ptr,     v_arg.v.lo);
      vst1q_f64(ptr + 2, v_arg.v.hi);
    }

    static __forceinline void storeu(double* ptr, const vdouble4& v) {
      vst1q_f64(ptr,     v.v.lo);
      vst1q_f64(ptr + 2, v.v.hi);
    }

    static __forceinline vdouble4 broadcast(const void* a) {
      float64x2_t t = vdupq_n_f64(*(const double*)a);
      vdouble4 r;
      r.v.lo = t; r.v.hi = t;
      return r;
    }

    ////////////////////////////////////////////////////////////////////////////////
    /// Array Access
    ////////////////////////////////////////////////////////////////////////////////

    __forceinline       double& operator [](size_t index)       { assert(index < 4); return i[index]; }
    __forceinline const double& operator [](size_t index) const { assert(index < 4); return i[index]; }
  };

  ////////////////////////////////////////////////////////////////////////////////
  /// Unary Operators
  ////////////////////////////////////////////////////////////////////////////////

  __forceinline vdouble4 operator +(const vdouble4& a) { return a; }
  __forceinline vdouble4 operator -(const vdouble4& a) {
    vdouble4 r;
    r.v.lo = vnegq_f64(a.v.lo);
    r.v.hi = vnegq_f64(a.v.hi);
    return r;
  }

  ////////////////////////////////////////////////////////////////////////////////
  /// Binary Operators
  ////////////////////////////////////////////////////////////////////////////////

  __forceinline vdouble4 operator +(const vdouble4& a, const vdouble4& b) {
    vdouble4 r;
    r.v.lo = vaddq_f64(a.v.lo, b.v.lo);
    r.v.hi = vaddq_f64(a.v.hi, b.v.hi);
    return r;
  }
  __forceinline vdouble4 operator +(const vdouble4& a, double b) { return a + vdouble4(b); }
  __forceinline vdouble4 operator +(double a, const vdouble4& b) { return vdouble4(a) + b; }

  __forceinline vdouble4 operator -(const vdouble4& a, const vdouble4& b) {
    vdouble4 r;
    r.v.lo = vsubq_f64(a.v.lo, b.v.lo);
    r.v.hi = vsubq_f64(a.v.hi, b.v.hi);
    return r;
  }
  __forceinline vdouble4 operator -(const vdouble4& a, double b) { return a - vdouble4(b); }
  __forceinline vdouble4 operator -(double a, const vdouble4& b) { return vdouble4(a) - b; }

  __forceinline vdouble4 operator *(const vdouble4& a, const vdouble4& b) {
    vdouble4 r;
    r.v.lo = vmulq_f64(a.v.lo, b.v.lo);
    r.v.hi = vmulq_f64(a.v.hi, b.v.hi);
    return r;
  }
  __forceinline vdouble4 operator *(const vdouble4& a, double b) { return a * vdouble4(b); }
  __forceinline vdouble4 operator *(double a, const vdouble4& b) { return vdouble4(a) * b; }

  __forceinline vdouble4 operator /(const vdouble4& a, const vdouble4& b) {
    vdouble4 r;
    r.v.lo = vdivq_f64(a.v.lo, b.v.lo);
    r.v.hi = vdivq_f64(a.v.hi, b.v.hi);
    return r;
  }
  __forceinline vdouble4 operator /(const vdouble4& a, double b) { return a / vdouble4(b); }
  __forceinline vdouble4 operator /(double a, const vdouble4& b) { return vdouble4(a) / b; }

  __forceinline vdouble4 operator &(const vdouble4& a, const vdouble4& b) {
    vdouble4 r;
    r.v.lo = vreinterpretq_f64_s64(vandq_s64(vreinterpretq_s64_f64(a.v.lo), vreinterpretq_s64_f64(b.v.lo)));
    r.v.hi = vreinterpretq_f64_s64(vandq_s64(vreinterpretq_s64_f64(a.v.hi), vreinterpretq_s64_f64(b.v.hi)));
    return r;
  }
  __forceinline vdouble4 operator &(const vdouble4& a, double b) { return a & vdouble4(b); }
  __forceinline vdouble4 operator &(double a, const vdouble4& b) { return vdouble4(a) & b; }

  __forceinline vdouble4 operator |(const vdouble4& a, const vdouble4& b) {
    vdouble4 r;
    r.v.lo = vreinterpretq_f64_s64(vorrq_s64(vreinterpretq_s64_f64(a.v.lo), vreinterpretq_s64_f64(b.v.lo)));
    r.v.hi = vreinterpretq_f64_s64(vorrq_s64(vreinterpretq_s64_f64(a.v.hi), vreinterpretq_s64_f64(b.v.hi)));
    return r;
  }
  __forceinline vdouble4 operator |(const vdouble4& a, double b) { return a | vdouble4(b); }
  __forceinline vdouble4 operator |(double a, const vdouble4& b) { return vdouble4(a) | b; }

  __forceinline vdouble4 operator ^(const vdouble4& a, const vdouble4& b) {
    vdouble4 r;
    r.v.lo = vreinterpretq_f64_s64(veorq_s64(vreinterpretq_s64_f64(a.v.lo), vreinterpretq_s64_f64(b.v.lo)));
    r.v.hi = vreinterpretq_f64_s64(veorq_s64(vreinterpretq_s64_f64(a.v.hi), vreinterpretq_s64_f64(b.v.hi)));
    return r;
  }
  __forceinline vdouble4 operator ^(const vdouble4& a, double b) { return a ^ vdouble4(b); }
  __forceinline vdouble4 operator ^(double a, const vdouble4& b) { return vdouble4(a) ^ b; }

  __forceinline vdouble4 min(const vdouble4& a, const vdouble4& b) {
    vdouble4 r;
    r.v.lo = vminq_f64(a.v.lo, b.v.lo);
    r.v.hi = vminq_f64(a.v.hi, b.v.hi);
    return r;
  }
  __forceinline vdouble4 min(const vdouble4& a, double b) { return min(a, vdouble4(b)); }
  __forceinline vdouble4 min(double a, const vdouble4& b) { return min(vdouble4(a), b); }

  __forceinline vdouble4 max(const vdouble4& a, const vdouble4& b) {
    vdouble4 r;
    r.v.lo = vmaxq_f64(a.v.lo, b.v.lo);
    r.v.hi = vmaxq_f64(a.v.hi, b.v.hi);
    return r;
  }
  __forceinline vdouble4 max(const vdouble4& a, double b) { return max(a, vdouble4(b)); }
  __forceinline vdouble4 max(double a, const vdouble4& b) { return max(vdouble4(a), b); }

  ////////////////////////////////////////////////////////////////////////////////
  /// Ternary Operators
  ////////////////////////////////////////////////////////////////////////////////

  __forceinline vdouble4 madd (const vdouble4& a, const vdouble4& b, const vdouble4& c) {
    vdouble4 r;
    r.v.lo = vfmaq_f64(c.v.lo, a.v.lo, b.v.lo);
    r.v.hi = vfmaq_f64(c.v.hi, a.v.hi, b.v.hi);
    return r;
  }
  __forceinline vdouble4 msub (const vdouble4& a, const vdouble4& b, const vdouble4& c) {
    vdouble4 r;
    r.v.lo = vnegq_f64(vfmsq_f64(c.v.lo, a.v.lo, b.v.lo));
    r.v.hi = vnegq_f64(vfmsq_f64(c.v.hi, a.v.hi, b.v.hi));
    return r;
  }
  __forceinline vdouble4 nmadd(const vdouble4& a, const vdouble4& b, const vdouble4& c) {
    vdouble4 r;
    r.v.lo = vfmsq_f64(c.v.lo, a.v.lo, b.v.lo);
    r.v.hi = vfmsq_f64(c.v.hi, a.v.hi, b.v.hi);
    return r;
  }
  __forceinline vdouble4 nmsub(const vdouble4& a, const vdouble4& b, const vdouble4& c) {
    vdouble4 r;
    r.v.lo = vnegq_f64(vfmaq_f64(c.v.lo, a.v.lo, b.v.lo));
    r.v.hi = vnegq_f64(vfmaq_f64(c.v.hi, a.v.hi, b.v.hi));
    return r;
  }

  ////////////////////////////////////////////////////////////////////////////////
  /// Assignment Operators
  ////////////////////////////////////////////////////////////////////////////////

  __forceinline vdouble4& operator +=(vdouble4& a, const vdouble4& b) { return a = a + b; }
  __forceinline vdouble4& operator +=(vdouble4& a, double b) { return a = a + b; }

  __forceinline vdouble4& operator -=(vdouble4& a, const vdouble4& b) { return a = a - b; }
  __forceinline vdouble4& operator -=(vdouble4& a, double b) { return a = a - b; }

  __forceinline vdouble4& operator *=(vdouble4& a, const vdouble4& b) { return a = a * b; }
  __forceinline vdouble4& operator *=(vdouble4& a, double b) { return a = a * b; }

  __forceinline vdouble4& operator &=(vdouble4& a, const vdouble4& b) { return a = a & b; }
  __forceinline vdouble4& operator &=(vdouble4& a, double b) { return a = a & b; }

  __forceinline vdouble4& operator |=(vdouble4& a, const vdouble4& b) { return a = a | b; }
  __forceinline vdouble4& operator |=(vdouble4& a, double b) { return a = a | b; }

  ////////////////////////////////////////////////////////////////////////////////
  /// Comparison Operators + Select
  ////////////////////////////////////////////////////////////////////////////////

  __forceinline vboold4 operator ==(const vdouble4& a, const vdouble4& b) {
    vboold4 r;
    r.v.lo = vreinterpretq_f64_u64(vceqq_f64(a.v.lo, b.v.lo));
    r.v.hi = vreinterpretq_f64_u64(vceqq_f64(a.v.hi, b.v.hi));
    return r;
  }
  __forceinline vboold4 operator !=(const vdouble4& a, const vdouble4& b) {
    vboold4 r;
    r.v.lo = vreinterpretq_f64_u32(vmvnq_u32(vreinterpretq_u32_u64(vceqq_f64(a.v.lo, b.v.lo))));
    r.v.hi = vreinterpretq_f64_u32(vmvnq_u32(vreinterpretq_u32_u64(vceqq_f64(a.v.hi, b.v.hi))));
    return r;
  }
  __forceinline vboold4 operator < (const vdouble4& a, const vdouble4& b) {
    vboold4 r;
    r.v.lo = vreinterpretq_f64_u64(vcltq_f64(a.v.lo, b.v.lo));
    r.v.hi = vreinterpretq_f64_u64(vcltq_f64(a.v.hi, b.v.hi));
    return r;
  }
  __forceinline vboold4 operator >=(const vdouble4& a, const vdouble4& b) {
    vboold4 r;
    r.v.lo = vreinterpretq_f64_u64(vcgeq_f64(a.v.lo, b.v.lo));
    r.v.hi = vreinterpretq_f64_u64(vcgeq_f64(a.v.hi, b.v.hi));
    return r;
  }
  __forceinline vboold4 operator > (const vdouble4& a, const vdouble4& b) {
    vboold4 r;
    r.v.lo = vreinterpretq_f64_u64(vcgtq_f64(a.v.lo, b.v.lo));
    r.v.hi = vreinterpretq_f64_u64(vcgtq_f64(a.v.hi, b.v.hi));
    return r;
  }
  __forceinline vboold4 operator <=(const vdouble4& a, const vdouble4& b) {
    vboold4 r;
    r.v.lo = vreinterpretq_f64_u64(vcleq_f64(a.v.lo, b.v.lo));
    r.v.hi = vreinterpretq_f64_u64(vcleq_f64(a.v.hi, b.v.hi));
    return r;
  }

  __forceinline vboold4 operator ==(const vdouble4& a, double b) { return a == vdouble4(b); }
  __forceinline vboold4 operator ==(double a, const vdouble4& b) { return vdouble4(a) == b; }

  __forceinline vboold4 operator !=(const vdouble4& a, double b) { return a != vdouble4(b); }
  __forceinline vboold4 operator !=(double a, const vdouble4& b) { return vdouble4(a) != b; }

  __forceinline vboold4 operator < (const vdouble4& a, double b) { return a <  vdouble4(b); }
  __forceinline vboold4 operator < (double a, const vdouble4& b) { return vdouble4(a) <  b; }

  __forceinline vboold4 operator >=(const vdouble4& a, double b) { return a >= vdouble4(b); }
  __forceinline vboold4 operator >=(double a, const vdouble4& b) { return vdouble4(a) >= b; }

  __forceinline vboold4 operator > (const vdouble4& a, double b) { return a >  vdouble4(b); }
  __forceinline vboold4 operator > (double a, const vdouble4& b) { return vdouble4(a) >  b; }

  __forceinline vboold4 operator <=(const vdouble4& a, double b) { return a <= vdouble4(b); }
  __forceinline vboold4 operator <=(double a, const vdouble4& b) { return vdouble4(a) <= b; }

  __forceinline vboold4 eq(const vdouble4& a, const vdouble4& b) { return a == b; }
  __forceinline vboold4 ne(const vdouble4& a, const vdouble4& b) { return a != b; }
  __forceinline vboold4 lt(const vdouble4& a, const vdouble4& b) { return a <  b; }
  __forceinline vboold4 ge(const vdouble4& a, const vdouble4& b) { return a >= b; }
  __forceinline vboold4 gt(const vdouble4& a, const vdouble4& b) { return a >  b; }
  __forceinline vboold4 le(const vdouble4& a, const vdouble4& b) { return a <= b; }

  __forceinline vboold4 eq(const vboold4& mask, const vdouble4& a, const vdouble4& b) { return mask & (a == b); }
  __forceinline vboold4 ne(const vboold4& mask, const vdouble4& a, const vdouble4& b) { return mask & (a != b); }
  __forceinline vboold4 lt(const vboold4& mask, const vdouble4& a, const vdouble4& b) { return mask & (a <  b); }
  __forceinline vboold4 ge(const vboold4& mask, const vdouble4& a, const vdouble4& b) { return mask & (a >= b); }
  __forceinline vboold4 gt(const vboold4& mask, const vdouble4& a, const vdouble4& b) { return mask & (a >  b); }
  __forceinline vboold4 le(const vboold4& mask, const vdouble4& a, const vdouble4& b) { return mask & (a <= b); }

  __forceinline vdouble4 select(const vboold4& m, const vdouble4& t, const vdouble4& f) {
    vdouble4 r;
    uint64x2_t mask_lo = vreinterpretq_u64_f64(m.v.lo);
    uint64x2_t mask_hi = vreinterpretq_u64_f64(m.v.hi);
    r.v.lo = vbslq_f64(mask_lo, t.v.lo, f.v.lo);
    r.v.hi = vbslq_f64(mask_hi, t.v.hi, f.v.hi);
    return r;
  }

  ////////////////////////////////////////////////////////////////////////////////
  // Movement/Shifting/Shuffling Functions
  ////////////////////////////////////////////////////////////////////////////////

  template<int i0, int i1>
  __forceinline vdouble4 shuffle(const vdouble4& v) {
    vdouble4 r;
    float64x1_t lo0 = i0 ? vget_high_f64(v.v.lo) : vget_low_f64(v.v.lo);
    float64x1_t lo1 = i1 ? vget_high_f64(v.v.lo) : vget_low_f64(v.v.lo);
    float64x1_t hi0 = i0 ? vget_high_f64(v.v.hi) : vget_low_f64(v.v.hi);
    float64x1_t hi1 = i1 ? vget_high_f64(v.v.hi) : vget_low_f64(v.v.hi);
    r.v.lo = vcombine_f64(lo0, lo1);
    r.v.hi = vcombine_f64(hi0, hi1);
    return r;
  }

  template<int i>
  __forceinline vdouble4 shuffle(const vdouble4& v) {
    return shuffle<i, i>(v);
  }

  template<int i0, int i1>
  __forceinline vdouble4 shuffle2(const vdouble4& v) {
    vdouble4 r;
    r.v.lo = i0 ? v.v.hi : v.v.lo;
    r.v.hi = i1 ? v.v.hi : v.v.lo;
    return r;
  }

  __forceinline double toScalar(const vdouble4& v) {
    return vgetq_lane_f64(v.v.lo, 0);
  }

  ////////////////////////////////////////////////////////////////////////////////
  /// Reductions
  ////////////////////////////////////////////////////////////////////////////////

  __forceinline vdouble4 vreduce_min2(const vdouble4& x) { return min(x, shuffle<1,0>(x)); }
  __forceinline vdouble4 vreduce_min (const vdouble4& y) { const vdouble4 x = vreduce_min2(y); return min(x, shuffle2<1,0>(x)); }

  __forceinline vdouble4 vreduce_max2(const vdouble4& x) { return max(x, shuffle<1,0>(x)); }
  __forceinline vdouble4 vreduce_max (const vdouble4& y) { const vdouble4 x = vreduce_max2(y); return max(x, shuffle2<1,0>(x)); }

  __forceinline vdouble4 vreduce_and2(const vdouble4& x) { return x & shuffle<1,0>(x); }
  __forceinline vdouble4 vreduce_and (const vdouble4& y) { const vdouble4 x = vreduce_and2(y); return x & shuffle2<1,0>(x); }

  __forceinline vdouble4 vreduce_or2(const vdouble4& x) { return x | shuffle<1,0>(x); }
  __forceinline vdouble4 vreduce_or (const vdouble4& y) { const vdouble4 x = vreduce_or2(y); return x | shuffle2<1,0>(x); }

  __forceinline vdouble4 vreduce_add2(const vdouble4& x) { return x + shuffle<1,0>(x); }
  __forceinline vdouble4 vreduce_add (const vdouble4& y) {
    const vdouble4 x = vreduce_add2(y);
    double lo = vgetq_lane_f64(x.v.lo, 0) + vgetq_lane_f64(x.v.lo, 1);
    double hi = vgetq_lane_f64(x.v.hi, 0) + vgetq_lane_f64(x.v.hi, 1);
    double sum = lo + hi;
    return vdouble4(sum, sum, sum, sum);
  }

  __forceinline double reduce_add(const vdouble4& a) { return toScalar(vreduce_add(a)); }
  __forceinline double reduce_min(const vdouble4& a) { return toScalar(vreduce_min(a)); }
  __forceinline double reduce_max(const vdouble4& a) { return toScalar(vreduce_max(a)); }

  ////////////////////////////////////////////////////////////////////////////////
  /// Memory load and store operations
  ////////////////////////////////////////////////////////////////////////////////

  ////////////////////////////////////////////////////////////////////////////////
  /// Output Operators
  ////////////////////////////////////////////////////////////////////////////////

  __forceinline embree_ostream operator <<(embree_ostream cout, const vdouble4& v)
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
