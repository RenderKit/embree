// Copyright 2009-2021 Intel Corporation
// SPDX-License-Identifier: Apache-2.0

#pragma once

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
  /* 4-wide NEON integer type */
  template<>
  struct vint<4>
  {
    ALIGNED_STRUCT_(16);
    
    typedef vboolf4 Bool;
    typedef vint4   Int;
    typedef vfloat4 Float;

    enum  { size = 4 };
    union { __m128i v; int i[4]; };

    // Constructors & Assignment
    __forceinline vint() {}
    __forceinline vint(const vint4& a) { v = a.v; }
    __forceinline vint4& operator =(const vint4& a) { v = a.v; return *this; }

    __forceinline vint(__m128i a) : v(a) {}
    __forceinline operator const __m128i&() const { return v; }
    __forceinline operator       __m128i&()       { return v; }

    __forceinline vint(int a) : v(vdupq_n_s32(a)) {}
    __forceinline vint(int a, int b, int c, int d) {
      int lanes[4] = {a, b, c, d};
      v = vld1q_s32(lanes);
    }

    __forceinline explicit vint(__m128 a) : v(vcvtq_s32_f32(a)) {}
    __forceinline explicit vint(const vboolf4& a) : v(vreinterpretq_s32_f32(a.v)) {}

    __forceinline vint(long long a, long long b) {
      int32x2_t lo = vcreate_s32(a);
      int32x2_t hi = vcreate_s32(b);
      v = vcombine_s32(lo, hi);
    }

    // Constants
    __forceinline vint(ZeroTy)        : v(vdupq_n_s32(0)) {}
    __forceinline vint(OneTy)         : v(vdupq_n_s32(1)) {}
    __forceinline vint(PosInfTy)      : v(vdupq_n_s32(pos_inf)) {}
    __forceinline vint(NegInfTy)      : v(vdupq_n_s32(neg_inf)) {}
    __forceinline vint(StepTy)        { int lanes[4] = {0,1,2,3}; v = vld1q_s32(lanes); }
    __forceinline vint(ReverseStepTy) { int lanes[4] = {3,2,1,0}; v = vld1q_s32(lanes); }
    __forceinline vint(TrueTy)        : v(vreinterpretq_s32_u32(vmvnq_u32(vdupq_n_u32(0)))) {}
    __forceinline vint(UndefinedTy)   : v(vdupq_n_s32(0)) {}

    // Loads and Stores
    static __forceinline vint4 load (const void* a) { return vld1q_s32((const int*)a); }
    static __forceinline vint4 loadu(const void* a) { return vld1q_s32((const int*)a); }

    static __forceinline void store (void* ptr, const vint4& v_arg) { vst1q_s32((int*)ptr, v_arg.v); }
    static __forceinline void storeu(void* ptr, const vint4& v_arg) { vst1q_s32((int*)ptr, v_arg.v); }

    // Masked load/store - scalar fallback (no AVX512 on ARM)
    static __forceinline vint4 load (const vbool4& mask, const void* a) { 
      return vreinterpretq_s32_u32(vandq_u32(vreinterpretq_u32_s32(vld1q_s32((const int*)a)), vreinterpretq_u32_s32(mask.v))); 
    }
    static __forceinline vint4 loadu(const vbool4& mask, const void* a) { 
      return vreinterpretq_s32_u32(vandq_u32(vreinterpretq_u32_s32(vld1q_s32((const int*)a)), vreinterpretq_u32_s32(mask.v))); 
    }

    static __forceinline void store (const vboolf4& mask, void* ptr, const vint4& i) { store (ptr,select(mask,i,load (ptr))); }
    static __forceinline void storeu(const vboolf4& mask, void* ptr, const vint4& i) { storeu(ptr,select(mask,i,loadu(ptr))); }

    // Load from unsigned char (4 bytes -> 4 ints, zero-extended)
    static __forceinline vint4 load(const unsigned char* ptr) {
      uint8x8_t t0 = vld1_u8(ptr);
      uint16x8_t t1 = vmovl_u8(t0);
      uint32x4_t t2 = vmovl_u16(vget_low_u16(t1));
      return vreinterpretq_s32_u32(t2);
    }
    static __forceinline vint4 loadu(const unsigned char* ptr) { return load(ptr); }

    // Load from unsigned short (4 shorts -> 4 ints, zero-extended)
    static __forceinline vint4 load(const unsigned short* ptr) {
      uint16x8_t t0 = vld1q_u16(ptr);
      uint32x4_t t1 = vmovl_u16(vget_low_u16(t0));
      return vreinterpretq_s32_u32(t1);
    }

    // Store to unsigned char (4 ints -> 4 bytes, with saturation)
    static __forceinline void store(unsigned char* ptr, const vint4& v_arg) {
      uint32x4_t x = vreinterpretq_u32_s32(v_arg.v);
      uint16x4_t y = vqmovn_u32(x);
      uint8x8_t z = vqmovn_u16(vcombine_u16(y, y));
      vst1_lane_u32((uint32_t*)ptr, vreinterpret_u32_u8(z), 0);
    }

    // Store to unsigned short (4 ints -> 4 shorts, with saturation)
    static __forceinline void store(unsigned short* ptr, const vint4& v_arg) {
      uint32x4_t x = vreinterpretq_u32_s32(v_arg.v);
      uint16x4_t y = vqmovn_u32(x);
      vst1_u16(ptr, y);
    }

    // Non-temporal load/store
    static __forceinline vint4 load_nt(void* ptr) { return vld1q_s32((const int*)ptr); }
    static __forceinline void store_nt(void* ptr, const vint4& v_arg) { vst1q_s32((int*)ptr, v_arg.v); }

    // Gather
    template<int scale = 4>
    static __forceinline vint4 gather(const int* ptr, const vint4& index) {
      return vint4(
        *(int*)(((char*)ptr)+scale*index[0]),
        *(int*)(((char*)ptr)+scale*index[1]),
        *(int*)(((char*)ptr)+scale*index[2]),
        *(int*)(((char*)ptr)+scale*index[3]));
    }

    template<int scale = 4>
    static __forceinline vint4 gather(const vboolf4& mask, const int* ptr, const vint4& index) {
      vint4 r = zero;
      if (likely(mask[0])) r[0] = *(int*)(((char*)ptr)+scale*index[0]);
      if (likely(mask[1])) r[1] = *(int*)(((char*)ptr)+scale*index[1]);
      if (likely(mask[2])) r[2] = *(int*)(((char*)ptr)+scale*index[2]);
      if (likely(mask[3])) r[3] = *(int*)(((char*)ptr)+scale*index[3]);
      return r;
    }

    // Scatter
    template<int scale = 4>
    static __forceinline void scatter(void* ptr, const vint4& index, const vint4& v_arg) {
      *(int*)(((char*)ptr)+scale*index[0]) = v_arg[0];
      *(int*)(((char*)ptr)+scale*index[1]) = v_arg[1];
      *(int*)(((char*)ptr)+scale*index[2]) = v_arg[2];
      *(int*)(((char*)ptr)+scale*index[3]) = v_arg[3];
    }

    template<int scale = 4>
    static __forceinline void scatter(const vboolf4& mask, void* ptr, const vint4& index, const vint4& v_arg) {
      if (likely(mask[0])) *(int*)(((char*)ptr)+scale*index[0]) = v_arg[0];
      if (likely(mask[1])) *(int*)(((char*)ptr)+scale*index[1]) = v_arg[1];
      if (likely(mask[2])) *(int*)(((char*)ptr)+scale*index[2]) = v_arg[2];
      if (likely(mask[3])) *(int*)(((char*)ptr)+scale*index[3]) = v_arg[3];
    }

#if defined(__aarch64__)
    static __forceinline vint4 broadcast64(long long a) { return vint4(a, a); }
#endif

    // Array Access
    __forceinline const int& operator [](size_t index) const { assert(index < 4); return i[index]; }
    __forceinline       int& operator [](size_t index)       { assert(index < 4); return i[index]; }

    friend __forceinline vint4 select(const vboolf4& m, const vint4& t, const vint4& f) {
      return vreinterpretq_s32_u32(vbslq_u32(vreinterpretq_u32_f32(m.v),
                                               vreinterpretq_u32_s32(t.v),
                                               vreinterpretq_u32_s32(f.v)));
    }
  };

  // Unary Operators
  __forceinline vboolf4 asBool(const vint4& a) { return vreinterpretq_f32_s32(a.v); }

  __forceinline vint4 operator +(const vint4& a) { return a; }
  __forceinline vint4 operator -(const vint4& a) { return vnegq_s32(a.v); }
  __forceinline vint4 abs(const vint4& a) { return vabsq_s32(a.v); }

  // Binary Operators
  __forceinline vint4 operator +(const vint4& a, const vint4& b) { return vaddq_s32(a.v, b.v); }
  __forceinline vint4 operator +(const vint4& a, int b) { return a + vint4(b); }
  __forceinline vint4 operator +(int a, const vint4& b) { return vint4(a) + b; }

  __forceinline vint4 operator -(const vint4& a, const vint4& b) { return vsubq_s32(a.v, b.v); }
  __forceinline vint4 operator -(const vint4& a, int b) { return a - vint4(b); }
  __forceinline vint4 operator -(int a, const vint4& b) { return vint4(a) - b; }

  __forceinline vint4 operator *(const vint4& a, const vint4& b) { return vmulq_s32(a.v, b.v); }
  __forceinline vint4 operator *(const vint4& a, int b) { return a * vint4(b); }
  __forceinline vint4 operator *(int a, const vint4& b) { return vint4(a) * b; }

  __forceinline vint4 operator &(const vint4& a, const vint4& b) { return vandq_s32(a.v, b.v); }
  __forceinline vint4 operator &(const vint4& a, int b) { return a & vint4(b); }
  __forceinline vint4 operator &(int a, const vint4& b) { return vint4(a) & b; }

  __forceinline vint4 operator |(const vint4& a, const vint4& b) { return vorrq_s32(a.v, b.v); }
  __forceinline vint4 operator |(const vint4& a, int b) { return a | vint4(b); }
  __forceinline vint4 operator |(int a, const vint4& b) { return vint4(a) | b; }

  __forceinline vint4 operator ^(const vint4& a, const vint4& b) { return veorq_s32(a.v, b.v); }
  __forceinline vint4 operator ^(const vint4& a, int b) { return a ^ vint4(b); }
  __forceinline vint4 operator ^(int a, const vint4& b) { return vint4(a) ^ b; }

  __forceinline vint4 operator <<(const vint4& a, const int n) { return vshlq_s32(a.v, vdupq_n_s32(-(int32_t)n)); }
  __forceinline vint4 operator >>(const vint4& a, const int n) { return vshlq_s32(a.v, vdupq_n_s32((int32_t)n)); }

  __forceinline vint4 sll(const vint4& a, int b) { return vshlq_s32(a.v, vdupq_n_s32(-(int32_t)b)); }
  __forceinline vint4 sra(const vint4& a, int b) { return vshlq_s32(a.v, vdupq_n_s32((int32_t)b)); }
  __forceinline vint4 srl(const vint4& a, int b) { return vreinterpretq_s32_u32(vshlq_u32(vreinterpretq_u32_s32(a.v), vdupq_n_s32((int32_t)b))); }

  // Assignment Operators
  __forceinline vint4& operator +=(vint4& a, const vint4& b) { return a = a + b; }
  __forceinline vint4& operator +=(vint4& a, int b) { return a = a + b; }
  __forceinline vint4& operator -=(vint4& a, const vint4& b) { return a = a - b; }
  __forceinline vint4& operator -=(vint4& a, int b) { return a = a - b; }
  __forceinline vint4& operator *=(vint4& a, const vint4& b) { return a = a * b; }
  __forceinline vint4& operator *=(vint4& a, int b) { return a = a * b; }
  __forceinline vint4& operator &=(vint4& a, const vint4& b) { return a = a & b; }
  __forceinline vint4& operator &=(vint4& a, int b) { return a = a & b; }
  __forceinline vint4& operator |=(vint4& a, const vint4& b) { return a = a | b; }
  __forceinline vint4& operator |=(vint4& a, int b) { return a = a | b; }
  __forceinline vint4& operator <<=(vint4& a, int b) { return a = a << b; }
  __forceinline vint4& operator >>=(vint4& a, int b) { return a = a >> b; }

  // Comparison Operators
  __forceinline vboolf4 operator ==(const vint4& a, const vint4& b) { return vreinterpretq_f32_u32(vceqq_s32(a.v, b.v)); }
  __forceinline vboolf4 operator !=(const vint4& a, const vint4& b) { return !(a == b); }
  __forceinline vboolf4 operator < (const vint4& a, const vint4& b) { return vreinterpretq_f32_u32(vcltq_s32(a.v, b.v)); }
  __forceinline vboolf4 operator >=(const vint4& a, const vint4& b) { return !(a <  b); }
  __forceinline vboolf4 operator > (const vint4& a, const vint4& b) { return vreinterpretq_f32_u32(vcgtq_s32(a.v, b.v)); }
  __forceinline vboolf4 operator <=(const vint4& a, const vint4& b) { return !(a >  b); }

  __forceinline vboolf4 operator ==(const vint4& a, int b) { return a == vint4(b); }
  __forceinline vboolf4 operator ==(int a, const vint4& b) { return vint4(a) == b; }
  __forceinline vboolf4 operator !=(const vint4& a, int b) { return a != vint4(b); }
  __forceinline vboolf4 operator !=(int a, const vint4& b) { return vint4(a) != b; }
  __forceinline vboolf4 operator < (const vint4& a, int b) { return a <  vint4(b); }
  __forceinline vboolf4 operator < (int a, const vint4& b) { return vint4(a) <  b; }
  __forceinline vboolf4 operator >=(const vint4& a, int b) { return a >= vint4(b); }
  __forceinline vboolf4 operator >=(int a, const vint4& b) { return vint4(a) >= b; }
  __forceinline vboolf4 operator > (const vint4& a, int b) { return a >  vint4(b); }
  __forceinline vboolf4 operator > (int a, const vint4& b) { return vint4(a) >  b; }
  __forceinline vboolf4 operator <=(const vint4& a, int b) { return a <= vint4(b); }
  __forceinline vboolf4 operator <=(int a, const vint4& b) { return vint4(a) <= b; }

  __forceinline vboolf4 eq(const vint4& a, const vint4& b) { return a == b; }
  __forceinline vboolf4 ne(const vint4& a, const vint4& b) { return a != b; }
  __forceinline vboolf4 lt(const vint4& a, const vint4& b) { return a <  b; }
  __forceinline vboolf4 ge(const vint4& a, const vint4& b) { return a >= b; }
  __forceinline vboolf4 gt(const vint4& a, const vint4& b) { return a >  b; }
  __forceinline vboolf4 le(const vint4& a, const vint4& b) { return a <= b; }

  __forceinline vboolf4 eq(const vboolf4& mask, const vint4& a, const vint4& b) { return mask & (a == b); }
  __forceinline vboolf4 ne(const vboolf4& mask, const vint4& a, const vint4& b) { return mask & (a != b); }
  __forceinline vboolf4 lt(const vboolf4& mask, const vint4& a, const vint4& b) { return mask & (a <  b); }
  __forceinline vboolf4 ge(const vboolf4& mask, const vint4& a, const vint4& b) { return mask & (a >= b); }
  __forceinline vboolf4 gt(const vboolf4& mask, const vint4& a, const vint4& b) { return mask & (a >  b); }
  __forceinline vboolf4 le(const vboolf4& mask, const vint4& a, const vint4& b) { return mask & (a <= b); }

  template<int mask>
  __forceinline vint4 select(const vint4& t, const vint4& f) {
    return select(vboolf4(mask), t, f);
  }

  // Min/Max - NEON has native signed min/max
  __forceinline vint4 min(const vint4& a, const vint4& b) { return vminq_s32(a.v, b.v); }
  __forceinline vint4 max(const vint4& a, const vint4& b) { return vmaxq_s32(a.v, b.v); }
  __forceinline vint4 umin(const vint4& a, const vint4& b) { return vreinterpretq_s32_u32(vminq_u32(vreinterpretq_u32_s32(a.v), vreinterpretq_u32_s32(b.v))); }
  __forceinline vint4 umax(const vint4& a, const vint4& b) { return vreinterpretq_s32_u32(vmaxq_u32(vreinterpretq_u32_s32(a.v), vreinterpretq_u32_s32(b.v))); }

  __forceinline vint4 min(const vint4& a, int b) { return min(a,vint4(b)); }
  __forceinline vint4 min(int a, const vint4& b) { return min(vint4(a),b); }
  __forceinline vint4 max(const vint4& a, int b) { return max(a,vint4(b)); }
  __forceinline vint4 max(int a, const vint4& b) { return max(vint4(a),b); }

  // Movement/Shifting/Shuffling
  __forceinline vint4 unpacklo(const vint4& a, const vint4& b) { 
    return vreinterpretq_s32_u32(vzip1q_u32(vreinterpretq_u32_s32(a.v), vreinterpretq_u32_s32(b.v))); 
  }
  __forceinline vint4 unpackhi(const vint4& a, const vint4& b) { 
    return vreinterpretq_s32_u32(vzip2q_u32(vreinterpretq_u32_s32(a.v), vreinterpretq_u32_s32(b.v))); 
  }

  // Shuffle using vqtbl1q_u8 (byte table lookup)
  template<int i0, int i1, int i2, int i3>
  __forceinline vint4 shuffle(const vint4& v) {
    return vreinterpretq_s32_u8(vqtbl1q_u8(vreinterpretq_u8_s32(v.v), _MN_SHUFFLE(i0, i1, i2, i3)));
  }

  template<int i0, int i1, int i2, int i3>
  __forceinline vint4 shuffle(const vint4& a, const vint4& b) {
    return vreinterpretq_s32_u8(vqtbl2q_u8((uint8x16x2_t){vreinterpretq_u8_s32(a.v), vreinterpretq_u8_s32(b.v)}, _MF_SHUFFLE(i0, i1, i2, i3)));
  }

  template<int i0>
  __forceinline vint4 shuffle(const vint4& v) {
    return shuffle<i0,i0,i0,i0>(v);
  }

  template<int src> __forceinline int extract(const vint4& b) { return b[src&3]; }
  template<> __forceinline int extract<0>(const vint4& b) { return vgetq_lane_s32(b.v, 0); }
  template<int dst> __forceinline vint4 insert(const vint4& a, int b) { vint4 c = a; c[dst&3] = b; return c; }

  __forceinline int toScalar(const vint4& v) { return vgetq_lane_s32(v.v, 0); }

  __forceinline size_t toSizeT(const vint4& v) {
    uint64x2_t x = vreinterpretq_u64_s32(v.v);
    return vgetq_lane_u64(x, 0);
  }

  // Reductions - NEON has native horizontal operations!
  __forceinline vint4 vreduce_min(const vint4& v) { return vdupq_n_s32(vminvq_s32(v.v)); }
  __forceinline vint4 vreduce_max(const vint4& v) { return vdupq_n_s32(vmaxvq_s32(v.v)); }
  __forceinline vint4 vreduce_add(const vint4& v) { return vdupq_n_s32(vaddvq_s32(v.v)); }

  __forceinline int reduce_min(const vint4& v) { return vminvq_s32(v.v); }
  __forceinline int reduce_max(const vint4& v) { return vmaxvq_s32(v.v); }
  __forceinline int reduce_add(const vint4& v) { return vaddvq_s32(v.v); }

  __forceinline size_t select_min(const vint4& v) { return bsf(movemask(v == vreduce_min(v))); }
  __forceinline size_t select_max(const vint4& v) { return bsf(movemask(v == vreduce_max(v))); }
  __forceinline size_t select_min(const vboolf4& valid, const vint4& v) { 
    const vint4 a = select(valid,v,vint4(pos_inf)); 
    return bsf(movemask(valid & (a == vreduce_min(a)))); 
  }
  __forceinline size_t select_max(const vboolf4& valid, const vint4& v) { 
    const vint4 a = select(valid,v,vint4(neg_inf)); 
    return bsf(movemask(valid & (a == vreduce_max(a)))); 
  }

  // Sorting networks
  __forceinline vint4 usort_ascending(const vint4& v)
  {
    const vint4 a0 = v;
    const vint4 b0 = shuffle<1,0,3,2>(a0);
    const vint4 c0 = umin(a0,b0);
    const vint4 d0 = umax(a0,b0);
    const vint4 a1 = select<0x5>(c0,d0);
    const vint4 b1 = shuffle<2,3,0,1>(a1);
    const vint4 c1 = umin(a1,b1);
    const vint4 d1 = umax(a1,b1);
    const vint4 a2 = select<0x3>(c1,d1);
    const vint4 b2 = shuffle<0,2,1,3>(a2);
    const vint4 c2 = umin(a2,b2);
    const vint4 d2 = umax(a2,b2);
    const vint4 a3 = select<0x2>(c2,d2);
    return a3;
  }

  __forceinline vint4 usort_descending(const vint4& v)
  {
    const vint4 a0 = v;
    const vint4 b0 = shuffle<1,0,3,2>(a0);
    const vint4 c0 = umax(a0,b0);
    const vint4 d0 = umin(a0,b0);
    const vint4 a1 = select<0x5>(c0,d0);
    const vint4 b1 = shuffle<2,3,0,1>(a1);
    const vint4 c1 = umax(a1,b1);
    const vint4 d1 = umin(a1,b1);
    const vint4 a2 = select<0x3>(c1,d1);
    const vint4 b2 = shuffle<0,2,1,3>(a2);
    const vint4 c2 = umax(a2,b2);
    const vint4 d2 = umin(a2,b2);
    const vint4 a3 = select<0x2>(c2,d2);
    return a3;
  }

  // Output
  __forceinline embree_ostream operator <<(embree_ostream cout, const vint4& a) {
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
