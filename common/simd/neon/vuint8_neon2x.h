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
  /* 8-wide NEON2X unsigned integer type */
  template <>
  struct vuint<8>
  {
    ALIGNED_STRUCT_(32);

    typedef vboolf8 Bool;
    typedef vuint8 Int;
    typedef vfloat8 Float;

    enum
    {
      size = 8
    };
    union
    {
      struct
      {
        int32x4_t vl, vh;
      } v;
      unsigned int i[8];
    };

    ////////////////////////////////////////////////////////////////////////////////
    /// Constructors, Assignment & Cast Operators
    ////////////////////////////////////////////////////////////////////////////////

    __forceinline vuint() {}
    __forceinline vuint(const vuint8 &a)
    {
      v.vl = a.v.vl;
      v.vh = a.v.vh;
    }
    __forceinline vuint8 &operator=(const vuint8 &a)
    {
      v.vl = a.v.vl;
      v.vh = a.v.vh;
      return *this;
    }

    __forceinline explicit vuint(const vuint4 &a)
    {
      v.vl = a.v;
      v.vh = a.v;
    }
    __forceinline vuint(const vuint4 &a, const vuint4 &b)
    {
      v.vl = a.v;
      v.vh = b.v;
    }
    __forceinline vuint(const int32x4_t &a, const int32x4_t &b)
    {
      v.vl = a;
      v.vh = b;
    }

    __forceinline explicit vuint(const unsigned int *a)
    {
      v.vl = vld1q_s32((const int *)a);
      v.vh = vld1q_s32((const int *)a + 4);
    }
    __forceinline vuint(unsigned int a)
    {
      v.vl = vdupq_n_s32(a);
      v.vh = vdupq_n_s32(a);
    }
    __forceinline vuint(unsigned int a, unsigned int b)
    {
      int32x4_t lo = vreinterpretq_s32_u32(vcombine_u32(vdup_n_u32(a), vdup_n_u32(b)));
      v.vl = lo;
      v.vh = lo;
    }
    __forceinline vuint(unsigned int a, unsigned int b, unsigned int c, unsigned int d)
    {
      uint32_t lo[4] = {a, b, c, d};
      int32x4_t vlo = vreinterpretq_s32_u32(vld1q_u32(lo));
      v.vl = vlo;
      v.vh = vlo;
    }
    __forceinline vuint(unsigned int a, unsigned int b, unsigned int c, unsigned int d, unsigned int e, unsigned int f, unsigned int g, unsigned int h)
    {
      uint32_t lo[4] = {a, b, c, d};
      uint32_t hi[4] = {e, f, g, h};
      v.vl = vreinterpretq_s32_u32(vld1q_u32(lo));
      v.vh = vreinterpretq_s32_u32(vld1q_u32(hi));
    }

    __forceinline explicit vuint(float32x4_t a_lo, float32x4_t a_hi)
    {
      v.vl = vcvtq_s32_f32(a_lo);
      v.vh = vcvtq_s32_f32(a_hi);
    }

    __forceinline explicit vuint(const vboolf8 &a)
    {
      v.vl = vreinterpretq_s32_f32(a.v.vl);
      v.vh = vreinterpretq_s32_f32(a.v.vh);
    }

    ////////////////////////////////////////////////////////////////////////////////
    /// Constants
    ////////////////////////////////////////////////////////////////////////////////

    __forceinline vuint(ZeroTy)
    {
      v.vl = vdupq_n_s32(0);
      v.vh = vdupq_n_s32(0);
    }
    __forceinline vuint(OneTy)
    {
      v.vl = vdupq_n_s32(1);
      v.vh = vdupq_n_s32(1);
    }
    __forceinline vuint(PosInfTy)
    {
      v.vl = vdupq_n_s32(pos_inf);
      v.vh = vdupq_n_s32(pos_inf);
    }
    __forceinline vuint(NegInfTy)
    {
      v.vl = vdupq_n_s32(neg_inf);
      v.vh = vdupq_n_s32(neg_inf);
    }
    __forceinline vuint(StepTy)
    {
      uint32_t lo[4] = {0, 1, 2, 3};
      uint32_t hi[4] = {4, 5, 6, 7};
      v.vl = vreinterpretq_s32_u32(vld1q_u32(lo));
      v.vh = vreinterpretq_s32_u32(vld1q_u32(hi));
    }
    __forceinline vuint(UndefinedTy)
    {
      v.vl = vdupq_n_s32(0);
      v.vh = vdupq_n_s32(0);
    }

    ////////////////////////////////////////////////////////////////////////////////
    /// Loads and Stores
    ////////////////////////////////////////////////////////////////////////////////

    static __forceinline vuint8 load(const unsigned char *ptr)
    {
      vuint8 r;
      uint8x8_t t0 = vld1_u8(ptr);
      uint16x8_t t1 = vmovl_u8(t0);
      r.v.vl = vreinterpretq_s32_u32(vmovl_u16(vget_low_u16(t1)));
      t1 = vmovl_u8(vld1_u8(ptr + 8));
      r.v.vh = vreinterpretq_s32_u32(vmovl_u16(vget_low_u16(t1)));
      return r;
    }
    static __forceinline vuint8 loadu(const unsigned char *ptr) { return load(ptr); }

    static __forceinline vuint8 load(const unsigned short *ptr)
    {
      vuint8 r;
      r.v.vl = vreinterpretq_s32_u32(vmovl_u16(vld1_u16(ptr)));
      r.v.vh = vreinterpretq_s32_u32(vmovl_u16(vld1_u16(ptr + 4)));
      return r;
    }
    static __forceinline vuint8 loadu(const unsigned short *ptr) { return load(ptr); }

    static __forceinline vuint8 load(const void *ptr)
    {
      vuint8 r;
      r.v.vl = vld1q_s32((const int *)ptr);
      r.v.vh = vld1q_s32((const int *)ptr + 4);
      return r;
    }
    static __forceinline vuint8 loadu(const void *ptr)
    {
      vuint8 r;
      r.v.vl = vld1q_s32((const int *)ptr);
      r.v.vh = vld1q_s32((const int *)ptr + 4);
      return r;
    }

    static __forceinline void store(void *ptr, const vuint8 &v_arg)
    {
      vst1q_s32((int *)ptr, v_arg.v.vl);
      vst1q_s32((int *)ptr + 4, v_arg.v.vh);
    }

    static __forceinline void storeu(void *ptr, const vuint8 &v_arg)
    {
      vst1q_s32((int *)ptr, v_arg.v.vl);
      vst1q_s32((int *)ptr + 4, v_arg.v.vh);
    }

    static __forceinline vuint8 load(const vboolf8 &mask, const void *ptr)
    {
      vuint8 r;
      r.v.vl = vreinterpretq_s32_u32(vandq_u32(vreinterpretq_u32_f32(mask.v.vl), vreinterpretq_u32_s32(vld1q_s32((const int *)ptr))));
      r.v.vh = vreinterpretq_s32_u32(vandq_u32(vreinterpretq_u32_f32(mask.v.vh), vreinterpretq_u32_s32(vld1q_s32((const int *)ptr + 4))));
      return r;
    }
    static __forceinline vuint8 loadu(const vboolf8 &mask, const void *ptr) { return load(mask, ptr); }

    static __forceinline void store(const vboolf8 &mask, void *ptr, const vuint8 &v_arg)
    {
      vuint8 cur = load(ptr);
      vuint8 r;
      r.v.vl = vreinterpretq_s32_u32(vbslq_u32(vreinterpretq_u32_f32(mask.v.vl), vreinterpretq_u32_s32(v_arg.v.vl), vreinterpretq_u32_s32(cur.v.vl)));
      r.v.vh = vreinterpretq_s32_u32(vbslq_u32(vreinterpretq_u32_f32(mask.v.vh), vreinterpretq_u32_s32(v_arg.v.vh), vreinterpretq_u32_s32(cur.v.vh)));
      store(ptr, r);
    }
    static __forceinline void storeu(const vboolf8 &mask, void *ptr, const vuint8 &v_arg)
    {
      vuint8 cur = loadu(ptr);
      vuint8 r;
      r.v.vl = vreinterpretq_s32_u32(vbslq_u32(vreinterpretq_u32_f32(mask.v.vl), vreinterpretq_u32_s32(v_arg.v.vl), vreinterpretq_u32_s32(cur.v.vl)));
      r.v.vh = vreinterpretq_s32_u32(vbslq_u32(vreinterpretq_u32_f32(mask.v.vh), vreinterpretq_u32_s32(v_arg.v.vh), vreinterpretq_u32_s32(cur.v.vh)));
      storeu(ptr, r);
    }

    static __forceinline vuint8 load_nt(void *ptr)
    {
      vuint8 r;
      r.v.vl = vld1q_s32((const int *)ptr);
      r.v.vh = vld1q_s32((const int *)ptr + 4);
      return r;
    }

    static __forceinline void store_nt(void *ptr, const vuint8 &v_arg)
    {
      vst1q_s32((int *)ptr, v_arg.v.vl);
      vst1q_s32((int *)ptr + 4, v_arg.v.vh);
    }

    static __forceinline void store(unsigned char *ptr, const vuint8 &i)
    {
      for (size_t j = 0; j < 8; j++)
        ptr[j] = i[j];
    }

    static __forceinline void store(unsigned short *ptr, const vuint8 &v_arg)
    {
      for (size_t i = 0; i < 8; i++)
        ptr[i] = (unsigned short)v_arg[i];
    }

    template <int scale = 4>
    static __forceinline vuint8 gather(const unsigned int *const ptr, const vint8 &index)
    {
      vuint8 r;
      for (size_t i = 0; i < 8; i++)
        r[i] = *(unsigned int *)(((char *)ptr) + scale * index[i]);
      return r;
    }

    template <int scale = 4>
    static __forceinline vuint8 gather(const vboolf8 &mask, const unsigned int *const ptr, const vint8 &index)
    {
      vuint8 r = zero;
      for (size_t i = 0; i < 8; i++)
        if (likely(mask[i]))
          r[i] = *(unsigned int *)(((char *)ptr) + scale * index[i]);
      return r;
    }

    template <int scale = 4>
    static __forceinline void scatter(void *ptr, const vint8 &ofs, const vuint8 &v_arg)
    {
      for (size_t i = 0; i < 8; i++)
        *(unsigned int *)(((char *)ptr) + scale * ofs[i]) = v_arg[i];
    }

    template <int scale = 4>
    static __forceinline void scatter(const vboolf8 &mask, void *ptr, const vint8 &ofs, const vuint8 &v_arg)
    {
      for (size_t i = 0; i < 8; i++)
        if (likely(mask[i]))
          *(unsigned int *)(((char *)ptr) + scale * ofs[i]) = v_arg[i];
    }

    static __forceinline vuint8 broadcast64(const long long &a)
    {
      vuint8 r;
      uint64x2_t b = vdupq_n_u64(a);
      r.v.vl = vreinterpretq_s32_u64(b);
      r.v.vh = vreinterpretq_s32_u64(b);
      return r;
    }

    ////////////////////////////////////////////////////////////////////////////////
    /// Array Access
    ////////////////////////////////////////////////////////////////////////////////

    __forceinline const unsigned int &operator[](size_t index) const
    {
      assert(index < 8);
      return i[index];
    }
    __forceinline unsigned int &operator[](size_t index)
    {
      assert(index < 8);
      return i[index];
    }
  };

  ////////////////////////////////////////////////////////////////////////////////
  /// Unary Operators
  ////////////////////////////////////////////////////////////////////////////////

  static __forceinline vboolf8 asBool(const vuint8 &a)
  {
    vboolf8 r;
    r.v.vl = vreinterpretq_f32_s32(a.v.vl);
    r.v.vh = vreinterpretq_f32_s32(a.v.vh);
    return r;
  }

  __forceinline vuint8 operator+(const vuint8 &a) { return a; }

  ////////////////////////////////////////////////////////////////////////////////
  /// Binary Operators
  ////////////////////////////////////////////////////////////////////////////////

  __forceinline vuint8 operator+(const vuint8 &a, const vuint8 &b)
  {
    vuint8 r;
    r.v.vl = vreinterpretq_s32_u32(vaddq_u32(vreinterpretq_u32_s32(a.v.vl), vreinterpretq_u32_s32(b.v.vl)));
    r.v.vh = vreinterpretq_s32_u32(vaddq_u32(vreinterpretq_u32_s32(a.v.vh), vreinterpretq_u32_s32(b.v.vh)));
    return r;
  }
  __forceinline vuint8 operator+(const vuint8 &a, unsigned int b) { return a + vuint8(b); }
  __forceinline vuint8 operator+(unsigned int a, const vuint8 &b) { return vuint8(a) + b; }

  __forceinline vuint8 operator-(const vuint8 &a, const vuint8 &b)
  {
    vuint8 r;
    r.v.vl = vreinterpretq_s32_u32(vsubq_u32(vreinterpretq_u32_s32(a.v.vl), vreinterpretq_u32_s32(b.v.vl)));
    r.v.vh = vreinterpretq_s32_u32(vsubq_u32(vreinterpretq_u32_s32(a.v.vh), vreinterpretq_u32_s32(b.v.vh)));
    return r;
  }
  __forceinline vuint8 operator-(const vuint8 &a, unsigned int b) { return a - vuint8(b); }
  __forceinline vuint8 operator-(unsigned int a, const vuint8 &b) { return vuint8(a) - b; }

  //__forceinline vuint8 operator *(const vuint8& a, const vuint8& b) { return _mm256_mullo_epu32(a, b); }
  //__forceinline vuint8 operator *(const vuint8& a, unsigned int          b) { return a * vuint8(b); }
  //__forceinline vuint8 operator *(unsigned int          a, const vuint8& b) { return vuint8(a) * b; }

  __forceinline vuint8 operator&(const vuint8 &a, const vuint8 &b)
  {
    vuint8 r;
    r.v.vl = vandq_s32(a.v.vl, b.v.vl);
    r.v.vh = vandq_s32(a.v.vh, b.v.vh);
    return r;
  }
  __forceinline vuint8 operator&(const vuint8 &a, unsigned int b) { return a & vuint8(b); }
  __forceinline vuint8 operator&(unsigned int a, const vuint8 &b) { return vuint8(a) & b; }

  __forceinline vuint8 operator|(const vuint8 &a, const vuint8 &b)
  {
    vuint8 r;
    r.v.vl = vorrq_s32(a.v.vl, b.v.vl);
    r.v.vh = vorrq_s32(a.v.vh, b.v.vh);
    return r;
  }
  __forceinline vuint8 operator|(const vuint8 &a, unsigned int b) { return a | vuint8(b); }
  __forceinline vuint8 operator|(unsigned int a, const vuint8 &b) { return vuint8(a) | b; }

  __forceinline vuint8 operator^(const vuint8 &a, const vuint8 &b)
  {
    vuint8 r;
    r.v.vl = veorq_s32(a.v.vl, b.v.vl);
    r.v.vh = veorq_s32(a.v.vh, b.v.vh);
    return r;
  }
  __forceinline vuint8 operator^(const vuint8 &a, unsigned int b) { return a ^ vuint8(b); }
  __forceinline vuint8 operator^(unsigned int a, const vuint8 &b) { return vuint8(a) ^ b; }

  __forceinline vuint8 operator<<(const vuint8 &a, unsigned int n)
  {
    vuint8 r;
    r.v.vl = vreinterpretq_s32_u32(vshlq_u32(vreinterpretq_u32_s32(a.v.vl), vdupq_n_s32((int32_t)n)));
    r.v.vh = vreinterpretq_s32_u32(vshlq_u32(vreinterpretq_u32_s32(a.v.vh), vdupq_n_s32((int32_t)n)));
    return r;
  }
  __forceinline vuint8 operator>>(const vuint8 &a, unsigned int n)
  {
    vuint8 r;
    r.v.vl = vreinterpretq_s32_u32(vshlq_u32(vreinterpretq_u32_s32(a.v.vl), vdupq_n_s32(-(int32_t)n)));
    r.v.vh = vreinterpretq_s32_u32(vshlq_u32(vreinterpretq_u32_s32(a.v.vh), vdupq_n_s32(-(int32_t)n)));
    return r;
  }

  __forceinline vuint8 operator<<(const vuint8 &a, const vuint8 &n)
  {
    vuint8 r;
    r.v.vl = vreinterpretq_s32_u32(vshlq_u32(vreinterpretq_u32_s32(a.v.vl), vreinterpretq_s32_u32(n.v.vl)));
    r.v.vh = vreinterpretq_s32_u32(vshlq_u32(vreinterpretq_u32_s32(a.v.vh), vreinterpretq_s32_u32(n.v.vh)));
    return r;
  }
  __forceinline vuint8 operator>>(const vuint8 &a, const vuint8 &n)
  {
    vuint8 r;
    r.v.vl = vreinterpretq_s32_u32(vshlq_u32(vreinterpretq_u32_s32(a.v.vl), vnegq_s32(vreinterpretq_s32_u32(n.v.vl))));
    r.v.vh = vreinterpretq_s32_u32(vshlq_u32(vreinterpretq_u32_s32(a.v.vh), vnegq_s32(vreinterpretq_s32_u32(n.v.vh))));
    return r;
  }

  __forceinline vuint8 sll(const vuint8 &a, unsigned int b)
  {
    vuint8 r;
    r.v.vl = vreinterpretq_s32_u32(vshlq_u32(vreinterpretq_u32_s32(a.v.vl), vdupq_n_s32((int32_t)b)));
    r.v.vh = vreinterpretq_s32_u32(vshlq_u32(vreinterpretq_u32_s32(a.v.vh), vdupq_n_s32((int32_t)b)));
    return r;
  }
  __forceinline vuint8 sra(const vuint8 &a, unsigned int b)
  {
    vuint8 r;
    r.v.vl = vshlq_s32(a.v.vl, vdupq_n_s32(-(int32_t)b));
    r.v.vh = vshlq_s32(a.v.vh, vdupq_n_s32(-(int32_t)b));
    return r;
  }
  __forceinline vuint8 srl(const vuint8 &a, unsigned int b)
  {
    vuint8 r;
    r.v.vl = vreinterpretq_s32_u32(vshlq_u32(vreinterpretq_u32_s32(a.v.vl), vdupq_n_s32(-(int32_t)b)));
    r.v.vh = vreinterpretq_s32_u32(vshlq_u32(vreinterpretq_u32_s32(a.v.vh), vdupq_n_s32(-(int32_t)b)));
    return r;
  }

  __forceinline vuint8 sll(const vuint8 &a, const vuint8 &b)
  {
    vuint8 r;
    r.v.vl = vreinterpretq_s32_u32(vshlq_u32(vreinterpretq_u32_s32(a.v.vl), vreinterpretq_s32_u32(b.v.vl)));
    r.v.vh = vreinterpretq_s32_u32(vshlq_u32(vreinterpretq_u32_s32(a.v.vh), vreinterpretq_s32_u32(b.v.vh)));
    return r;
  }
  __forceinline vuint8 sra(const vuint8 &a, const vuint8 &b)
  {
    vuint8 r;
    r.v.vl = vshlq_s32(a.v.vl, vnegq_s32(b.v.vl));
    r.v.vh = vshlq_s32(a.v.vh, vnegq_s32(b.v.vh));
    return r;
  }
  __forceinline vuint8 srl(const vuint8 &a, const vuint8 &b)
  {
    vuint8 r;
    r.v.vl = vreinterpretq_s32_u32(vshlq_u32(vreinterpretq_u32_s32(a.v.vl), vnegq_s32(vreinterpretq_s32_u32(b.v.vl))));
    r.v.vh = vreinterpretq_s32_u32(vshlq_u32(vreinterpretq_u32_s32(a.v.vh), vnegq_s32(vreinterpretq_s32_u32(b.v.vh))));
    return r;
  }

  __forceinline vuint8 min(const vuint8 &a, const vuint8 &b)
  {
    vuint8 r;
    r.v.vl = vreinterpretq_s32_u32(vminq_u32(vreinterpretq_u32_s32(a.v.vl), vreinterpretq_u32_s32(b.v.vl)));
    r.v.vh = vreinterpretq_s32_u32(vminq_u32(vreinterpretq_u32_s32(a.v.vh), vreinterpretq_u32_s32(b.v.vh)));
    return r;
  }
  __forceinline vuint8 min(const vuint8 &a, unsigned int b) { return min(a, vuint8(b)); }
  __forceinline vuint8 min(unsigned int a, const vuint8 &b) { return min(vuint8(a), b); }

  __forceinline vuint8 max(const vuint8 &a, const vuint8 &b)
  {
    vuint8 r;
    r.v.vl = vreinterpretq_s32_u32(vmaxq_u32(vreinterpretq_u32_s32(a.v.vl), vreinterpretq_u32_s32(b.v.vl)));
    r.v.vh = vreinterpretq_s32_u32(vmaxq_u32(vreinterpretq_u32_s32(a.v.vh), vreinterpretq_u32_s32(b.v.vh)));
    return r;
  }
  __forceinline vuint8 max(const vuint8 &a, unsigned int b) { return max(a, vuint8(b)); }
  __forceinline vuint8 max(unsigned int a, const vuint8 &b) { return max(vuint8(a), b); }

  ////////////////////////////////////////////////////////////////////////////////
  /// Assignment Operators
  ////////////////////////////////////////////////////////////////////////////////

  __forceinline vuint8 &operator+=(vuint8 &a, const vuint8 &b) { return a = a + b; }
  __forceinline vuint8 &operator+=(vuint8 &a, unsigned int b) { return a = a + b; }

  __forceinline vuint8 &operator-=(vuint8 &a, const vuint8 &b) { return a = a - b; }
  __forceinline vuint8 &operator-=(vuint8 &a, unsigned int b) { return a = a - b; }

  //__forceinline vuint8& operator *=(vuint8& a, const vuint8& b) { return a = a * b; }
  //__forceinline vuint8& operator *=(vuint8& a, unsigned int          b) { return a = a * b; }

  __forceinline vuint8 &operator&=(vuint8 &a, const vuint8 &b) { return a = a & b; }
  __forceinline vuint8 &operator&=(vuint8 &a, unsigned int b) { return a = a & b; }

  __forceinline vuint8 &operator|=(vuint8 &a, const vuint8 &b) { return a = a | b; }
  __forceinline vuint8 &operator|=(vuint8 &a, unsigned int b) { return a = a | b; }

  __forceinline vuint8 &operator<<=(vuint8 &a, const unsigned int b) { return a = a << b; }
  __forceinline vuint8 &operator>>=(vuint8 &a, const unsigned int b) { return a = a >> b; }

  ////////////////////////////////////////////////////////////////////////////////
  /// Comparison Operators + Select
  ////////////////////////////////////////////////////////////////////////////////

  static __forceinline vboolf8 operator==(const vuint8 &a, const vuint8 &b)
  {
    vboolf8 r;
    r.v.vl = vreinterpretq_f32_u32(vceqq_u32(vreinterpretq_u32_s32(a.v.vl), vreinterpretq_u32_s32(b.v.vl)));
    r.v.vh = vreinterpretq_f32_u32(vceqq_u32(vreinterpretq_u32_s32(a.v.vh), vreinterpretq_u32_s32(b.v.vh)));
    return r;
  }
  __forceinline vboolf8 operator!=(const vuint8 &a, const vuint8 &b) { return !(a == b); }
  //__forceinline vboolf8 operator < (const vuint8& a, const vuint8& b) { return _mm256_castsi256_ps(_mm256_cmpgt_epu32(b, a)); }
  //__forceinline vboolf8 operator >=(const vuint8& a, const vuint8& b) { return !(a <  b); }
  //__forceinline vboolf8 operator > (const vuint8& a, const vuint8& b) { return _mm256_castsi256_ps(_mm256_cmpgt_epu32(a, b)); }
  //__forceinline vboolf8 operator <=(const vuint8& a, const vuint8& b) { return !(a >  b); }

  static __forceinline vuint8 select(const vboolf8 &m, const vuint8 &t, const vuint8 &f)
  {
    vuint8 r;
    r.v.vl = vreinterpretq_s32_u32(vbslq_u32(vreinterpretq_u32_f32(m.v.vl),
                                             vreinterpretq_u32_s32(t.v.vl),
                                             vreinterpretq_u32_s32(f.v.vl)));
    r.v.vh = vreinterpretq_s32_u32(vbslq_u32(vreinterpretq_u32_f32(m.v.vh),
                                             vreinterpretq_u32_s32(t.v.vh),
                                             vreinterpretq_u32_s32(f.v.vh)));
    return r;
  }

  template <int mask>
  __forceinline vuint8 select(const vuint8 &t, const vuint8 &f)
  {
    return select(vboolf8(mask), t, f);
  }

  __forceinline vboolf8 operator==(const vuint8 &a, unsigned int b) { return a == vuint8(b); }
  __forceinline vboolf8 operator==(unsigned int a, const vuint8 &b) { return vuint8(a) == b; }

  __forceinline vboolf8 operator!=(const vuint8 &a, unsigned int b) { return a != vuint8(b); }
  __forceinline vboolf8 operator!=(unsigned int a, const vuint8 &b) { return vuint8(a) != b; }

  //__forceinline vboolf8 operator < (const vuint8& a, unsigned int          b) { return a <  vuint8(b); }
  //__forceinline vboolf8 operator < (unsigned int          a, const vuint8& b) { return vuint8(a) <  b; }

  //__forceinline vboolf8 operator >=(const vuint8& a, unsigned int          b) { return a >= vuint8(b); }
  //__forceinline vboolf8 operator >=(unsigned int          a, const vuint8& b) { return vuint8(a) >= b; }

  //__forceinline vboolf8 operator > (const vuint8& a, unsigned int          b) { return a >  vuint8(b); }
  //__forceinline vboolf8 operator > (unsigned int          a, const vuint8& b) { return vuint8(a) >  b; }

  //__forceinline vboolf8 operator <=(const vuint8& a, unsigned int          b) { return a <= vuint8(b); }
  //__forceinline vboolf8 operator <=(unsigned int          a, const vuint8& b) { return vuint8(a) <= b; }

  __forceinline vboolf8 eq(const vuint8 &a, const vuint8 &b) { return a == b; }
  __forceinline vboolf8 ne(const vuint8 &a, const vuint8 &b) { return a != b; }
  //__forceinline vboolf8 lt(const vuint8& a, const vuint8& b) { return a <  b; }
  //__forceinline vboolf8 ge(const vuint8& a, const vuint8& b) { return a >= b; }
  //__forceinline vboolf8 gt(const vuint8& a, const vuint8& b) { return a >  b; }
  //__forceinline vboolf8 le(const vuint8& a, const vuint8& b) { return a <= b; }

  __forceinline vboolf8 eq(const vboolf8 &mask, const vuint8 &a, const vuint8 &b) { return mask & (a == b); }
  __forceinline vboolf8 ne(const vboolf8 &mask, const vuint8 &a, const vuint8 &b) { return mask & (a != b); }
  //__forceinline vboolf8 lt(const vboolf8& mask, const vuint8& a, const vuint8& b) { return mask & (a <  b); }
  //__forceinline vboolf8 ge(const vboolf8& mask, const vuint8& a, const vuint8& b) { return mask & (a >= b); }
  //__forceinline vboolf8 gt(const vboolf8& mask, const vuint8& a, const vuint8& b) { return mask & (a >  b); }
  //__forceinline vboolf8 le(const vboolf8& mask, const vuint8& a, const vuint8& b) { return mask & (a <= b); }

  ////////////////////////////////////////////////////////////////////////////////
  /// Movement/Shifting/Shuffling Functions
  ////////////////////////////////////////////////////////////////////////////////

  __forceinline vuint8 unpacklo(const vuint8 &a, const vuint8 &b)
  {
    vuint8 r;
    r.v.vl = vreinterpretq_s32_u32(vzip1q_u32(vreinterpretq_u32_s32(a.v.vl), vreinterpretq_u32_s32(b.v.vl)));
    r.v.vh = vreinterpretq_s32_u32(vzip1q_u32(vreinterpretq_u32_s32(a.v.vh), vreinterpretq_u32_s32(b.v.vh)));
    return r;
  }
  __forceinline vuint8 unpackhi(const vuint8 &a, const vuint8 &b)
  {
    vuint8 r;
    r.v.vl = vreinterpretq_s32_u32(vzip2q_u32(vreinterpretq_u32_s32(a.v.vl), vreinterpretq_u32_s32(b.v.vl)));
    r.v.vh = vreinterpretq_s32_u32(vzip2q_u32(vreinterpretq_u32_s32(a.v.vh), vreinterpretq_u32_s32(b.v.vh)));
    return r;
  }

  template <int i>
  __forceinline vuint8 shuffle(const vuint8 &v)
  {
    int32x4x2_t r = neon_permute_epi32<i, i, i, i>(v.v.vl, v.v.vh);
    vuint8 result;
    result.v.vl = r.val[0];
    result.v.vh = r.val[1];
    return result;
  }

  template <int i0, int i1>
  __forceinline vuint8 shuffle4(const vuint8 &v)
  {
    int32x4x2_t r = neon_permute2f128_si<i0, i1>(v.v.vl, v.v.vh);
    vuint8 result;
    result.v.vl = r.val[0];
    result.v.vh = r.val[1];
    return result;
  }

  template <int i0, int i1>
  __forceinline vuint8 shuffle4(const vuint8 &a, const vuint8 &b)
  {
    int32x4x2_t r = neon_permute2f128_si<i0, i1>(a.v.vl, a.v.vh, b.v.vl, b.v.vh);
    vuint8 result;
    result.v.vl = r.val[0];
    result.v.vh = r.val[1];
    return result;
  }

  template <int i0, int i1, int i2, int i3>
  __forceinline vuint8 shuffle(const vuint8 &v)
  {
    int32x4x2_t r = neon_permute_epi32<i0, i1, i2, i3>(v.v.vl, v.v.vh);
    vuint8 result;
    result.v.vl = r.val[0];
    result.v.vh = r.val[1];
    return result;
  }

  template <int i0, int i1, int i2, int i3>
  __forceinline vuint8 shuffle(const vuint8 &a, const vuint8 &b)
  {
    int32x4x2_t r = neon_shuffle_epi32<i0, i1, i2, i3>(a.v.vl, a.v.vh, b.v.vl, b.v.vh);
    vuint8 result;
    result.v.vl = r.val[0];
    result.v.vh = r.val[1];
    return result;
  }

  template <>
  __forceinline vuint8 shuffle<0, 0, 2, 2>(const vuint8 &v)
  {
    int32x4x2_t r = neon_permute_epi32<0, 0, 2, 2>(v.v.vl, v.v.vh);
    vuint8 result;
    result.v.vl = r.val[0];
    result.v.vh = r.val[1];
    return result;
  }
  template <>
  __forceinline vuint8 shuffle<1, 1, 3, 3>(const vuint8 &v)
  {
    int32x4x2_t r = neon_permute_epi32<1, 1, 3, 3>(v.v.vl, v.v.vh);
    vuint8 result;
    result.v.vl = r.val[0];
    result.v.vh = r.val[1];
    return result;
  }
  template <>
  __forceinline vuint8 shuffle<0, 1, 0, 1>(const vuint8 &v)
  {
    int32x4x2_t r = neon_permute_epi32<0, 1, 0, 1>(v.v.vl, v.v.vh);
    vuint8 result;
    result.v.vl = r.val[0];
    result.v.vh = r.val[1];
    return result;
  }

  template <int i>
  __forceinline vuint8 insert4(const vuint8 &a, const vuint4 &b)
  {
    vuint8 r = a;
    if (i == 0)
      r.v.vl = b.v;
    else
      r.v.vh = b.v;
    return r;
  }
  template <int i>
  __forceinline vuint4 extract4(const vuint8 &a)
  {
    vuint4 r;
    if (i == 0)
      r.v = a.v.vl;
    else
      r.v = a.v.vh;
    return r;
  }
  template <>
  __forceinline vuint4 extract4<0>(const vuint8 &a)
  {
    vuint4 r;
    r.v = a.v.vl;
    return r;
  }

  __forceinline int toScalar(const vuint8 &v) { return vgetq_lane_s32(v.v.vl, 0); }

  ////////////////////////////////////////////////////////////////////////////////
  /// Reductions
  ////////////////////////////////////////////////////////////////////////////////

  //__forceinline vuint8 vreduce_min2(const vuint8& v) { return min(v,shuffle<1,0,3,2>(v)); }
  //__forceinline vuint8 vreduce_min4(const vuint8& v) { vuint8 v1 = vreduce_min2(v); return min(v1,shuffle<2,3,0,1>(v1)); }
  //__forceinline vuint8 vreduce_min (const vuint8& v) { vuint8 v1 = vreduce_min4(v); return min(v1,shuffle4<1,0>(v1)); }

  //__forceinline vuint8 vreduce_max2(const vuint8& v) { return max(v,shuffle<1,0,3,2>(v)); }
  //__forceinline vuint8 vreduce_max4(const vuint8& v) { vuint8 v1 = vreduce_max2(v); return max(v1,shuffle<2,3,0,1>(v1)); }
  //__forceinline vuint8 vreduce_max (const vuint8& v) { vuint8 v1 = vreduce_max4(v); return max(v1,shuffle4<1,0>(v1)); }

  __forceinline vuint8 vreduce_add2(const vuint8 &v) { return v + shuffle<1, 0, 3, 2>(v); }
  __forceinline vuint8 vreduce_add4(const vuint8 &v)
  {
    vuint8 v1 = vreduce_add2(v);
    return v1 + shuffle<2, 3, 0, 1>(v1);
  }
  __forceinline vuint8 vreduce_add(const vuint8 &v)
  {
    vuint8 v1 = vreduce_add4(v);
    return v1 + shuffle4<1, 0>(v1);
  }

  //__forceinline int reduce_min(const vuint8& v) { return toScalar(vreduce_min(v)); }
  //__forceinline int reduce_max(const vuint8& v) { return toScalar(vreduce_max(v)); }
  __forceinline int reduce_add(const vuint8 &v) { return toScalar(vreduce_add(v)); }

  //__forceinline size_t select_min(const vuint8& v) { return bsf(movemask(v == vreduce_min(v))); }
  //__forceinline size_t select_max(const vuint8& v) { return bsf(movemask(v == vreduce_max(v))); }

  //__forceinline size_t select_min(const vboolf8& valid, const vuint8& v) { const vuint8 a = select(valid,v,vuint8(pos_inf)); return bsf(movemask(valid & (a == vreduce_min(a)))); }
  //__forceinline size_t select_max(const vboolf8& valid, const vuint8& v) { const vuint8 a = select(valid,v,vuint8(neg_inf)); return bsf(movemask(valid & (a == vreduce_max(a)))); }

  ////////////////////////////////////////////////////////////////////////////////
  /// Output Operators
  ////////////////////////////////////////////////////////////////////////////////

  __forceinline embree_ostream operator<<(embree_ostream cout, const vuint8 &a)
  {
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
