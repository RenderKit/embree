// Copyright 2009-2021 Intel Corporation
// SPDX-License-Identifier: Apache-2.0

#pragma once

#include "neon_base.h"

#include "../../math/emath.h"

#define vboolf vboolf_impl
#define vboold vboold_impl
#define vint vint_impl
#define vuint vuint_impl
#define vllong vllong_impl
#define vfloat vfloat_impl
#define vdouble vdouble_impl

namespace embree
{
  /* 4-wide NEON unsigned integer type */
  template <>
  struct vuint<4>
  {
    ALIGNED_STRUCT_(16);

    typedef vboolf4 Bool;
    typedef vuint4 Int;
    typedef vfloat4 Float;

    enum
    {
      size = 4
    };
    union
    {
      int32x4_t v;
      unsigned int i[4];
    };

    // Constructors & Assignment
    __forceinline vuint() {}
    __forceinline vuint(const vuint4 &a) { v = a.v; }
    __forceinline vuint4 &operator=(const vuint4 &a)
    {
      v = a.v;
      return *this;
    }

    __forceinline vuint(const int32x4_t a) : v(a) {}
    __forceinline operator const int32x4_t &() const { return v; }
    __forceinline operator int32x4_t &() { return v; }

    __forceinline vuint(unsigned int a) : v(vreinterpretq_s32_u32(vdupq_n_u32(a))) {}
    __forceinline vuint(unsigned int a, unsigned int b, unsigned int c, unsigned int d)
    {
      uint32_t lanes[4] = {a, b, c, d};
      v = vreinterpretq_s32_u32(vld1q_u32(lanes));
    }

    __forceinline explicit vuint(const vboolf4 &a) : v(vreinterpretq_s32_f32(a.v)) {}

    // Constants
    __forceinline vuint(ZeroTy) : v(vreinterpretq_s32_u32(vdupq_n_u32(0))) {}
    __forceinline vuint(OneTy) : v(vreinterpretq_s32_u32(vdupq_n_u32(1))) {}
    __forceinline vuint(PosInfTy) : v(vreinterpretq_s32_u32(vdupq_n_u32(unsigned(pos_inf)))) {}
    __forceinline vuint(StepTy)
    {
      unsigned int lanes[4] = {0, 1, 2, 3};
      v = vreinterpretq_s32_u32(vld1q_u32(lanes));
    }
    __forceinline vuint(TrueTy) : v(vreinterpretq_s32_u32(vmvnq_u32(vdupq_n_u32(0)))) {}
    __forceinline vuint(UndefinedTy) : v(vdupq_n_s32(0)) {}

    // Loads and Stores
    static __forceinline vuint4 load(const void *a) { return vld1q_s32((const int *)a); }
    static __forceinline vuint4 loadu(const void *a) { return vld1q_s32((const int *)a); }

    static __forceinline void store(void *ptr, const vuint4 &v_arg) { vst1q_s32((int *)ptr, v_arg.v); }
    static __forceinline void storeu(void *ptr, const vuint4 &v_arg) { vst1q_s32((int *)ptr, v_arg.v); }

    // Masked load/store
    static __forceinline vuint4 load(const vboolf4 &mask, const void *a)
    {
      return vreinterpretq_s32_u32(vandq_u32(vreinterpretq_u32_s32(vld1q_s32((const int *)a)), vreinterpretq_u32_f32(mask.v)));
    }
    static __forceinline vuint4 loadu(const vboolf4 &mask, const void *a)
    {
      return vreinterpretq_s32_u32(vandq_u32(vreinterpretq_u32_s32(vld1q_s32((const int *)a)), vreinterpretq_u32_f32(mask.v)));
    }

    static __forceinline void store(const vboolf4 &mask, void *ptr, const vuint4 &i) { store(ptr, select(mask, i, load(ptr))); }
    static __forceinline void storeu(const vboolf4 &mask, void *ptr, const vuint4 &i) { storeu(ptr, select(mask, i, loadu(ptr))); }

    // Load from unsigned char (4 bytes -> 4 uints, zero-extended)
    static __forceinline vuint4 load(const unsigned char *ptr)
    {
      uint8x8_t t0 = vld1_u8(ptr);
      uint16x8_t t1 = vmovl_u8(t0);
      uint32x4_t t2 = vmovl_u16(vget_low_u16(t1));
      return vreinterpretq_s32_u32(t2);
    }
    static __forceinline vuint4 loadu(const unsigned char *ptr) { return load(ptr); }

    // Load from unsigned short (4 shorts -> 4 uints, zero-extended)
    static __forceinline vuint4 load(const unsigned short *ptr)
    {
      uint16x8_t t0 = vld1q_u16(ptr);
      uint32x4_t t1 = vmovl_u16(vget_low_u16(t0));
      return vreinterpretq_s32_u32(t1);
    }

    // Non-temporal load/store
    static __forceinline vuint4 load_nt(void *ptr) { return vld1q_s32((const int *)ptr); }
    static __forceinline void store_nt(void *ptr, const vuint4 &v_arg) { vst1q_s32((int *)ptr, v_arg.v); }

    // Gather
    template <int scale = 4>
    static __forceinline vuint4 gather(const unsigned int *ptr, const vint4 &index)
    {
      return vuint4(
          *(unsigned int *)(((char *)ptr) + scale * index[0]),
          *(unsigned int *)(((char *)ptr) + scale * index[1]),
          *(unsigned int *)(((char *)ptr) + scale * index[2]),
          *(unsigned int *)(((char *)ptr) + scale * index[3]));
    }

    template <int scale = 4>
    static __forceinline vuint4 gather(const vboolf4 &mask, const unsigned int *ptr, const vint4 &index)
    {
      vuint4 r = zero;
      if (likely(mask[0]))
        r[0] = *(unsigned int *)(((char *)ptr) + scale * index[0]);
      if (likely(mask[1]))
        r[1] = *(unsigned int *)(((char *)ptr) + scale * index[1]);
      if (likely(mask[2]))
        r[2] = *(unsigned int *)(((char *)ptr) + scale * index[2]);
      if (likely(mask[3]))
        r[3] = *(unsigned int *)(((char *)ptr) + scale * index[3]);
      return r;
    }

    // Array Access
    __forceinline const unsigned int &operator[](size_t index) const
    {
      assert(index < 4);
      return i[index];
    }
    __forceinline unsigned int &operator[](size_t index)
    {
      assert(index < 4);
      return i[index];
    }

    friend __forceinline vuint4 select(const vboolf4 &m, const vuint4 &t, const vuint4 &f)
    {
      return vreinterpretq_s32_u32(vbslq_u32(vreinterpretq_u32_f32(m.v),
                                             vreinterpretq_u32_s32(t.v),
                                             vreinterpretq_u32_s32(f.v)));
    }
  };

  // Unary Operators
  __forceinline vboolf4 asBool(const vuint4 &a) { return vreinterpretq_f32_s32(a.v); }

  __forceinline vuint4 operator+(const vuint4 &a) { return a; }
  __forceinline vuint4 operator-(const vuint4 &a)
  {
    return vreinterpretq_s32_u32(vnegq_s32(vreinterpretq_s32_u32(a.v)));
  }

  // Binary Operators
  __forceinline vuint4 operator+(const vuint4 &a, const vuint4 &b)
  {
    return vreinterpretq_s32_u32(vaddq_u32(vreinterpretq_u32_s32(a.v), vreinterpretq_u32_s32(b.v)));
  }
  __forceinline vuint4 operator+(const vuint4 &a, unsigned int b) { return a + vuint4(b); }
  __forceinline vuint4 operator+(unsigned int a, const vuint4 &b) { return vuint4(a) + b; }

  __forceinline vuint4 operator-(const vuint4 &a, const vuint4 &b)
  {
    return vreinterpretq_s32_u32(vsubq_u32(vreinterpretq_u32_s32(a.v), vreinterpretq_u32_s32(b.v)));
  }
  __forceinline vuint4 operator-(const vuint4 &a, unsigned int b) { return a - vuint4(b); }
  __forceinline vuint4 operator-(unsigned int a, const vuint4 &b) { return vuint4(a) - b; }

  __forceinline vuint4 operator&(const vuint4 &a, const vuint4 &b) { return vandq_s32(a.v, b.v); }
  __forceinline vuint4 operator&(const vuint4 &a, unsigned int b) { return a & vuint4(b); }
  __forceinline vuint4 operator&(unsigned int a, const vuint4 &b) { return vuint4(a) & b; }

  __forceinline vuint4 operator|(const vuint4 &a, const vuint4 &b) { return vorrq_s32(a.v, b.v); }
  __forceinline vuint4 operator|(const vuint4 &a, unsigned int b) { return a | vuint4(b); }
  __forceinline vuint4 operator|(unsigned int a, const vuint4 &b) { return vuint4(a) | b; }

  __forceinline vuint4 operator^(const vuint4 &a, const vuint4 &b) { return veorq_s32(a.v, b.v); }
  __forceinline vuint4 operator^(const vuint4 &a, unsigned int b) { return a ^ vuint4(b); }
  __forceinline vuint4 operator^(unsigned int a, const vuint4 &b) { return vuint4(a) ^ b; }

  __forceinline vuint4 operator<<(const vuint4 &a, unsigned int n)
  {
    return vreinterpretq_s32_u32(vshlq_u32(vreinterpretq_u32_s32(a.v), vdupq_n_s32((int32_t)n)));
  }
  __forceinline vuint4 operator>>(const vuint4 &a, unsigned int n)
  {
    return vreinterpretq_s32_u32(vshlq_u32(vreinterpretq_u32_s32(a.v), vdupq_n_s32(-(int32_t)n)));
  }

  __forceinline vuint4 sll(const vuint4 &a, unsigned int b)
  {
    return vreinterpretq_s32_u32(vshlq_u32(vreinterpretq_u32_s32(a.v), vdupq_n_s32((int32_t)b)));
  }
  __forceinline vuint4 sra(const vuint4 &a, unsigned int b)
  {
    return vshlq_s32(vreinterpretq_s32_u32(a.v), vdupq_n_s32(-(int32_t)b));
  }
  __forceinline vuint4 srl(const vuint4 &a, unsigned int b)
  {
    return vreinterpretq_s32_u32(vshlq_u32(vreinterpretq_u32_s32(a.v), vdupq_n_s32(-(int32_t)b)));
  }

  // Assignment Operators
  __forceinline vuint4 &operator+=(vuint4 &a, const vuint4 &b) { return a = a + b; }
  __forceinline vuint4 &operator+=(vuint4 &a, unsigned int b) { return a = a + b; }
  __forceinline vuint4 &operator-=(vuint4 &a, const vuint4 &b) { return a = a - b; }
  __forceinline vuint4 &operator-=(vuint4 &a, unsigned int b) { return a = a - b; }
  __forceinline vuint4 &operator&=(vuint4 &a, const vuint4 &b) { return a = a & b; }
  __forceinline vuint4 &operator&=(vuint4 &a, unsigned int b) { return a = a & b; }
  __forceinline vuint4 &operator|=(vuint4 &a, const vuint4 &b) { return a = a | b; }
  __forceinline vuint4 &operator|=(vuint4 &a, unsigned int b) { return a = a | b; }
  __forceinline vuint4 &operator<<=(vuint4 &a, unsigned int b) { return a = a << b; }
  __forceinline vuint4 &operator>>=(vuint4 &a, unsigned int b) { return a = a >> b; }

  // Comparison Operators
  __forceinline vboolf4 operator==(const vuint4 &a, const vuint4 &b)
  {
    return vreinterpretq_f32_u32(vceqq_u32(vreinterpretq_u32_s32(a.v), vreinterpretq_u32_s32(b.v)));
  }
  __forceinline vboolf4 operator!=(const vuint4 &a, const vuint4 &b) { return !(a == b); }

  __forceinline vboolf4 operator==(const vuint4 &a, unsigned int b) { return a == vuint4(b); }
  __forceinline vboolf4 operator==(unsigned int a, const vuint4 &b) { return vuint4(a) == b; }
  __forceinline vboolf4 operator!=(const vuint4 &a, unsigned int b) { return a != vuint4(b); }
  __forceinline vboolf4 operator!=(unsigned int a, const vuint4 &b) { return vuint4(a) != b; }

  __forceinline vboolf4 eq(const vuint4 &a, const vuint4 &b) { return a == b; }
  __forceinline vboolf4 ne(const vuint4 &a, const vuint4 &b) { return a != b; }

  __forceinline vboolf4 eq(const vboolf4 &mask, const vuint4 &a, const vuint4 &b) { return mask & (a == b); }
  __forceinline vboolf4 ne(const vboolf4 &mask, const vuint4 &a, const vuint4 &b) { return mask & (a != b); }

  template <int mask>
  __forceinline vuint4 select(const vuint4 &t, const vuint4 &f)
  {
    return select(vboolf4(mask), t, f);
  }

  // Shuffles using vqtbl1q_u8
  __forceinline vuint4 unpacklo(const vuint4 &a, const vuint4 &b)
  {
    return vreinterpretq_s32_u32(vzip1q_u32(vreinterpretq_u32_s32(a.v), vreinterpretq_u32_s32(b.v)));
  }
  __forceinline vuint4 unpackhi(const vuint4 &a, const vuint4 &b)
  {
    return vreinterpretq_s32_u32(vzip2q_u32(vreinterpretq_u32_s32(a.v), vreinterpretq_u32_s32(b.v)));
  }

  template <int i0, int i1, int i2, int i3>
  __forceinline vuint4 shuffle(const vuint4 &v)
  {
    return vreinterpretq_s32_u8(vqtbl1q_u8(vreinterpretq_u8_s32(v.v), _MN_SHUFFLE(i0, i1, i2, i3)));
  }

  template <int i0, int i1, int i2, int i3>
  __forceinline vuint4 shuffle(const vuint4 &a, const vuint4 &b)
  {
    return vreinterpretq_s32_u8(vqtbl2q_u8((uint8x16x2_t){vreinterpretq_u8_s32(a.v), vreinterpretq_u8_s32(b.v)}, _MF_SHUFFLE(i0, i1, i2, i3)));
  }

  template <int i0>
  __forceinline vuint4 shuffle(const vuint4 &v)
  {
    return shuffle<i0, i0, i0, i0>(v);
  }

  template <int src>
  __forceinline unsigned int extract(const vuint4 &b) { return b[src & 3]; }
  template <>
  __forceinline unsigned int extract<0>(const vuint4 &b) { return vgetq_lane_u32(vreinterpretq_u32_s32(b.v), 0); }
  template <int dst>
  __forceinline vuint4 insert(const vuint4 &a, const unsigned b)
  {
    vuint4 c = a;
    c[dst & 3] = b;
    return c;
  }

  __forceinline unsigned int toScalar(const vuint4 &v) { return vgetq_lane_u32(vreinterpretq_u32_s32(v.v), 0); }

  // Output
  __forceinline embree_ostream operator<<(embree_ostream cout, const vuint4 &a)
  {
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
