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
  /* 4-wide native NEON bool type for 64bit data types */
  template<>
  struct vboold<4>
  {
    ALIGNED_STRUCT_(32);

    typedef vboold4 Bool;

    enum  { size = 4 };
    union {
      __m256d v;
      struct { __m128d vl,vh; };
      long long i[4];
    };

    ////////////////////////////////////////////////////////////////////////////////
    /// Constructors, Assignment & Cast Operators
    ////////////////////////////////////////////////////////////////////////////////

    __forceinline vboold() {}
    __forceinline vboold(const vboold4& a) { v = a.v; }
    __forceinline vboold4& operator =(const vboold4& a) { v = a.v; return *this; }

    __forceinline vboold(__m256d a) : v(a) {}
    __forceinline vboold(__m256i a) : v(_mm256_castsi256_pd(a)) {}

    __forceinline operator const __m256() const { return _mm256_castpd_ps(v); }
    __forceinline operator const __m256i() const { return _mm256_castpd_si256(v); }
    __forceinline operator const __m256d() const { return v; }

    __forceinline vboold(int a)
    {
      assert(a >= 0 && a <= 255);
      vl = mm_lookupmask_pd[a & 0x3];
      vh = mm_lookupmask_pd[a >> 2];
    }

    ////////////////////////////////////////////////////////////////////////////////
    /// Constants
    ////////////////////////////////////////////////////////////////////////////////

    __forceinline vboold(FalseTy) : v(_mm256_setzero_pd()) {}
    __forceinline vboold(TrueTy) {
      uint64x2_t ones = vreinterpretq_u64_s32(vmvnq_s32(vdupq_n_s32(0)));
      v.lo = vreinterpretq_f64_u64(ones);
      v.hi = vreinterpretq_f64_u64(ones);
    }

    ////////////////////////////////////////////////////////////////////////////////
    /// Array Access
    ////////////////////////////////////////////////////////////////////////////////

    __forceinline bool       operator [](size_t index) const { assert(index < 4); return (_mm256_movemask_pd(v) >> index) & 1; }
    __forceinline long long& operator [](size_t index)       { assert(index < 4); return i[index]; }
  };

  ////////////////////////////////////////////////////////////////////////////////
  /// Unary Operators
  ////////////////////////////////////////////////////////////////////////////////

  __forceinline vboold4 operator !(const vboold4& a) {
    vboold4 r;
    r.v.lo = vreinterpretq_f64_u32(vmvnq_u32(vreinterpretq_u32_f64(a.v.lo)));
    r.v.hi = vreinterpretq_f64_u32(vmvnq_u32(vreinterpretq_u32_f64(a.v.hi)));
    return r;
  }

  ////////////////////////////////////////////////////////////////////////////////
  /// Binary Operators
  ////////////////////////////////////////////////////////////////////////////////

  __forceinline vboold4 operator &(const vboold4& a, const vboold4& b) {
    vboold4 r;
    r.v.lo = vreinterpretq_f64_u64(vandq_u64(vreinterpretq_u64_f64(a.v.lo), vreinterpretq_u64_f64(b.v.lo)));
    r.v.hi = vreinterpretq_f64_u64(vandq_u64(vreinterpretq_u64_f64(a.v.hi), vreinterpretq_u64_f64(b.v.hi)));
    return r;
  }
  __forceinline vboold4 operator |(const vboold4& a, const vboold4& b) {
    vboold4 r;
    r.v.lo = vreinterpretq_f64_u64(vorrq_u64(vreinterpretq_u64_f64(a.v.lo), vreinterpretq_u64_f64(b.v.lo)));
    r.v.hi = vreinterpretq_f64_u64(vorrq_u64(vreinterpretq_u64_f64(a.v.hi), vreinterpretq_u64_f64(b.v.hi)));
    return r;
  }
  __forceinline vboold4 operator ^(const vboold4& a, const vboold4& b) {
    vboold4 r;
    r.v.lo = vreinterpretq_f64_u64(veorq_u64(vreinterpretq_u64_f64(a.v.lo), vreinterpretq_u64_f64(b.v.lo)));
    r.v.hi = vreinterpretq_f64_u64(veorq_u64(vreinterpretq_u64_f64(a.v.hi), vreinterpretq_u64_f64(b.v.hi)));
    return r;
  }

  __forceinline vboold4 andn(const vboold4& a, const vboold4& b) {
    vboold4 r;
    r.v.lo = vreinterpretq_f64_u64(vbicq_u64(vreinterpretq_u64_f64(a.v.lo), vreinterpretq_u64_f64(b.v.lo)));
    r.v.hi = vreinterpretq_f64_u64(vbicq_u64(vreinterpretq_u64_f64(a.v.hi), vreinterpretq_u64_f64(b.v.hi)));
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
    r.v.lo = vreinterpretq_f64_u64(veorq_u64(vreinterpretq_u64_f64(a.v.lo), vreinterpretq_u64_f64(b.v.lo)));
    r.v.hi = vreinterpretq_f64_u64(veorq_u64(vreinterpretq_u64_f64(a.v.hi), vreinterpretq_u64_f64(b.v.hi)));
    return r;
  }
  __forceinline vboold4 operator ==(const vboold4& a, const vboold4& b) {
    vboold4 r;
    uint64x2_t xor_lo = veorq_u64(vreinterpretq_u64_f64(a.v.lo), vreinterpretq_u64_f64(b.v.lo));
    uint64x2_t xor_hi = veorq_u64(vreinterpretq_u64_f64(a.v.hi), vreinterpretq_u64_f64(b.v.hi));
    r.v.lo = vreinterpretq_f64_u32(vmvnq_u32(vreinterpretq_u32_u64(xor_lo)));
    r.v.hi = vreinterpretq_f64_u32(vmvnq_u32(vreinterpretq_u32_u64(xor_hi)));
    return r;
  }

  __forceinline vboold4 select(const vboold4& mask, const vboold4& t, const vboold4& f) {
    vboold4 r;
    r.v.lo = vreinterpretq_f64_u64(vbslq_u64(vreinterpretq_u64_f64(mask.v.lo), vreinterpretq_u64_f64(t.v.lo), vreinterpretq_u64_f64(f.v.lo)));
    r.v.hi = vreinterpretq_f64_u64(vbslq_u64(vreinterpretq_u64_f64(mask.v.hi), vreinterpretq_u64_f64(t.v.hi), vreinterpretq_u64_f64(f.v.hi)));
    return r;
  }

  ////////////////////////////////////////////////////////////////////////////////
  /// Movement/Shifting/Shuffling Functions
  ////////////////////////////////////////////////////////////////////////////////

  __forceinline vboold4 unpacklo(const vboold4& a, const vboold4& b) {
    vboold4 r;
    r.v.lo = vreinterpretq_f64_u64(vzip1q_u64(vreinterpretq_u64_f64(a.v.lo), vreinterpretq_u64_f64(b.v.lo)));
    r.v.hi = vreinterpretq_f64_u64(vzip1q_u64(vreinterpretq_u64_f64(a.v.hi), vreinterpretq_u64_f64(b.v.hi)));
    return r;
  }
  __forceinline vboold4 unpackhi(const vboold4& a, const vboold4& b) {
    vboold4 r;
    r.v.lo = vreinterpretq_f64_u64(vzip2q_u64(vreinterpretq_u64_f64(a.v.lo), vreinterpretq_u64_f64(b.v.lo)));
    r.v.hi = vreinterpretq_f64_u64(vzip2q_u64(vreinterpretq_u64_f64(a.v.hi), vreinterpretq_u64_f64(b.v.hi)));
    return r;
  }

  template<int i0, int i1, int i2, int i3>
  __forceinline vboold4 shuffle(const vboold4& v) {
    return _mm256_permute4x64_pd(v, _MM_SHUFFLE(i3, i2, i1, i0));
  }

  template<int i>
  __forceinline vboold4 shuffle(const vboold4& v) {
    return _mm256_permute4x64_pd(v, _MM_SHUFFLE(i, i, i, i));
  }

  ////////////////////////////////////////////////////////////////////////////////
  /// Reduction Operations
  ////////////////////////////////////////////////////////////////////////////////

  __forceinline bool reduce_and(const vboold4& a) { return _mm256_movemask_pd(a) == (unsigned int)0xf; }
  __forceinline bool reduce_or (const vboold4& a) { return !_mm256_testz_pd(a,a); }

  __forceinline bool all (const vboold4& a) { return _mm256_movemask_pd(a) == (unsigned int)0xf; }
  __forceinline bool any (const vboold4& a) { return !_mm256_testz_pd(a,a); }
  __forceinline bool none(const vboold4& a) { return _mm256_testz_pd(a,a) != 0; }

  __forceinline bool all (const vboold4& valid, const vboold4& b) { return all((!valid) | b); }
  __forceinline bool any (const vboold4& valid, const vboold4& b) { return any(valid & b); }
  __forceinline bool none(const vboold4& valid, const vboold4& b) { return none(valid & b); }

  __forceinline unsigned int movemask(const vboold4& a) { return _mm256_movemask_pd(a); }
  __forceinline size_t       popcnt  (const vboold4& a) { return popcnt((size_t)_mm256_movemask_pd(a)); }

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
