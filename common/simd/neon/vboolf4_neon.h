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
  /* 4-wide NEON bool type */
  template<>
  struct vboolf<4>
  {
    ALIGNED_STRUCT_(16);

    typedef vboolf4 Bool;
    typedef vint4   Int;
    typedef vfloat4 Float;

    enum  { size = 4 };
    union { __m128 v; int i[4]; };

    ////////////////////////////////////////////////////////////////////////////////
    /// Constructors, Assignment & Cast Operators
    ////////////////////////////////////////////////////////////////////////////////

    __forceinline vboolf() {}
    __forceinline vboolf(const vboolf4& other) { v = other.v; }
    __forceinline vboolf4& operator =(const vboolf4& other) { v = other.v; return *this; }

    __forceinline vboolf(__m128 input) : v(input) {}
    __forceinline operator const __m128&() const { return v; }
    __forceinline operator const __m128i() const { return vreinterpretq_s32_f32(v); }
    __forceinline operator const __m128d() const { return vreinterpretq_f64_f32(v); }

    __forceinline vboolf(bool a) : v(vreinterpretq_f32_u32(vdupq_n_u32(a ? 0xFFFFFFFF : 0))) {}
    __forceinline vboolf(bool a, bool b) {
      uint32x4_t t = vcombine_u32(
        vcreate_u32(((uint64_t)(a ? 0xFFFFFFFF : 0)) | ((uint64_t)(b ? 0xFFFFFFFF : 0) << 32)),
        vcreate_u32(((uint64_t)(a ? 0xFFFFFFFF : 0)) | ((uint64_t)(b ? 0xFFFFFFFF : 0) << 32))
      );
      v = vreinterpretq_f32_u32(t);
    }
    __forceinline vboolf(bool a, bool b, bool c, bool d) {
      uint32_t lanes[4] = { a ? 0xFFFFFFFF : 0, b ? 0xFFFFFFFF : 0, c ? 0xFFFFFFFF : 0, d ? 0xFFFFFFFF : 0 };
      v = vreinterpretq_f32_u32(vld1q_u32(lanes));
    }
    __forceinline vboolf(int mask) { assert(mask >= 0 && mask < 16); v = mm_lookupmask_ps[mask]; }
    __forceinline vboolf(unsigned int mask) { assert(mask < 16); v = mm_lookupmask_ps[mask]; }

    __forceinline __m128i mask32() const { return vreinterpretq_s32_f32(v); }

    ////////////////////////////////////////////////////////////////////////////////
    /// Constants
    ////////////////////////////////////////////////////////////////////////////////

    __forceinline vboolf(FalseTy) : v(vdupq_n_f32(0.0f)) {}
    __forceinline vboolf(TrueTy)  : v(vreinterpretq_f32_u32(vmvnq_u32(vdupq_n_u32(0)))) {}

    ////////////////////////////////////////////////////////////////////////////////
    /// Array Access
    ////////////////////////////////////////////////////////////////////////////////

    __forceinline bool operator [](size_t index) const {
      assert(index < 4);
      return (i[index] >> 31) != 0;
    }
    __forceinline int& operator [](size_t index) { assert(index < 4); return i[index]; }
  };

  ////////////////////////////////////////////////////////////////////////////////
  /// Unary Operators
  ////////////////////////////////////////////////////////////////////////////////

  __forceinline vboolf4 operator !(const vboolf4& a) {
    return vreinterpretq_f32_u32(veorq_u32(vreinterpretq_u32_f32(a.v), vdupq_n_u32(0xFFFFFFFF)));
  }

  ////////////////////////////////////////////////////////////////////////////////
  /// Binary Operators
  ////////////////////////////////////////////////////////////////////////////////

  __forceinline vboolf4 operator &(const vboolf4& a, const vboolf4& b) {
    return vreinterpretq_f32_u32(vandq_u32(vreinterpretq_u32_f32(a.v), vreinterpretq_u32_f32(b.v)));
  }
  __forceinline vboolf4 operator |(const vboolf4& a, const vboolf4& b) {
    return vreinterpretq_f32_u32(vorrq_u32(vreinterpretq_u32_f32(a.v), vreinterpretq_u32_f32(b.v)));
  }
  __forceinline vboolf4 operator ^(const vboolf4& a, const vboolf4& b) {
    return vreinterpretq_f32_u32(veorq_u32(vreinterpretq_u32_f32(a.v), vreinterpretq_u32_f32(b.v)));
  }
  __forceinline vboolf4 andn(const vboolf4& a, const vboolf4& b) {
    return vreinterpretq_f32_u32(vbicq_u32(vreinterpretq_u32_f32(a.v), vreinterpretq_u32_f32(b.v)));
  }

  ////////////////////////////////////////////////////////////////////////////////
  /// Assignment Operators
  ////////////////////////////////////////////////////////////////////////////////

  __forceinline vboolf4& operator &=(vboolf4& a, const vboolf4& b) { return a = a & b; }
  __forceinline vboolf4& operator |=(vboolf4& a, const vboolf4& b) { return a = a | b; }
  __forceinline vboolf4& operator ^=(vboolf4& a, const vboolf4& b) { return a = a ^ b; }

  ////////////////////////////////////////////////////////////////////////////////
  /// Comparison Operators + Select
  ////////////////////////////////////////////////////////////////////////////////

  __forceinline vboolf4 operator !=(const vboolf4& a, const vboolf4& b) {
    return vreinterpretq_f32_u32(veorq_u32(vreinterpretq_u32_f32(a.v), vreinterpretq_u32_f32(b.v)));
  }
  __forceinline vboolf4 operator ==(const vboolf4& a, const vboolf4& b) {
    return vreinterpretq_f32_u32(vceqq_s32(vreinterpretq_s32_f32(a.v), vreinterpretq_s32_f32(b.v)));
  }

  __forceinline vboolf4 select(const vboolf4& m, const vboolf4& t, const vboolf4& f) {
    return vreinterpretq_f32_u32(vbslq_u32(vreinterpretq_u32_f32(m.v),
                                            vreinterpretq_u32_f32(t.v),
                                            vreinterpretq_u32_f32(f.v)));
  }

  ////////////////////////////////////////////////////////////////////////////////
  /// Movement/Shifting/Shuffling Functions
  ////////////////////////////////////////////////////////////////////////////////

  template<int i0, int i1, int i2, int i3>
  __forceinline vboolf4 shuffle(const vboolf4& v) {
    return vreinterpretq_f32_u8(vqtbl1q_u8(vreinterpretq_u8_f32(v.v), _MN_SHUFFLE(i0, i1, i2, i3)));
  }

  template<int i0, int i1, int i2, int i3>
  __forceinline vboolf4 shuffle(const vboolf4& a, const vboolf4& b) {
    return vreinterpretq_f32_u8(vqtbl2q_u8((uint8x16x2_t){vreinterpretq_u8_f32(a.v), vreinterpretq_u8_f32(b.v)}, _MF_SHUFFLE(i0, i1, i2, i3)));
  }

  template<int i0>
  __forceinline vboolf4 shuffle(const vboolf4& v) {
    return shuffle<i0,i0,i0,i0>(v);
  }

  __forceinline vboolf4 unpacklo(const vboolf4& a, const vboolf4& b) {
    return vreinterpretq_f32_u32(vzip1q_u32(vreinterpretq_u32_f32(a.v), vreinterpretq_u32_f32(b.v)));
  }
  __forceinline vboolf4 unpackhi(const vboolf4& a, const vboolf4& b) {
    return vreinterpretq_f32_u32(vzip2q_u32(vreinterpretq_u32_f32(a.v), vreinterpretq_u32_f32(b.v)));
  }

  ////////////////////////////////////////////////////////////////////////////////
  /// Reduction Operations (movemask must be defined before these)
  ////////////////////////////////////////////////////////////////////////////////

  __forceinline size_t movemask(const vboolf4& a) {
    uint32x4_t shifted = vshrq_n_u32(vreinterpretq_u32_f32(a.v), 31);
    static const uint32x4_t bits = {0, 1, 2, 3};
    uint32x4_t masked = vandq_u32(shifted, bits);
    return vaddvq_u32(masked);
  }

  __forceinline size_t popcnt(const vboolf4& a) {
    return vaddvq_s32(vandq_s32(vreinterpretq_s32_f32(a.v), vdupq_n_s32(1)));
  }

  __forceinline bool reduce_and(const vboolf4& a) { return movemask(a) == 0xf; }
  __forceinline bool reduce_or (const vboolf4& a) { return movemask(a) != 0x0; }

  __forceinline bool all (const vboolf4& b) { return movemask(b) == 0xf; }
  __forceinline bool any (const vboolf4& b) { return movemask(b) != 0x0; }
  __forceinline bool none(const vboolf4& b) { return movemask(b) == 0x0; }

  __forceinline bool all (const vboolf4& valid, const vboolf4& b) { return all((!valid) | b); }
  __forceinline bool any (const vboolf4& valid, const vboolf4& b) { return any(valid & b); }
  __forceinline bool none(const vboolf4& valid, const vboolf4& b) { return none(valid & b); }

  ////////////////////////////////////////////////////////////////////////////////
  /// Get/Set Functions
  ////////////////////////////////////////////////////////////////////////////////

  __forceinline bool get(const vboolf4& a, size_t index) { return a[index]; }
  __forceinline void set(vboolf4& a, size_t index)       { a[index] = -1; }
  __forceinline void clear(vboolf4& a, size_t index)     { a[index] =  0; }

  ////////////////////////////////////////////////////////////////////////////////
  /// Output Operators
  ////////////////////////////////////////////////////////////////////////////////

  __forceinline embree_ostream operator <<(embree_ostream cout, const vboolf4& a) {
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
