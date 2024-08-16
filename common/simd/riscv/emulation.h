#pragma once

#include "sse2rvv.h"

#define _MM_SHUFFLE(fp3, fp2, fp1, fp0) \
    (((fp3) << 6) | ((fp2) << 4) | ((fp1) << 2) | ((fp0)))

/* Rounding mode macros. */
#define _MM_FROUND_TO_NEAREST_INT 0x00
#define _MM_FROUND_TO_NEG_INF 0x01
#define _MM_FROUND_TO_POS_INF 0x02
#define _MM_FROUND_TO_ZERO 0x03
#define _MM_FROUND_CUR_DIRECTION 0x04
#define _MM_FROUND_NO_EXC 0x08
#define _MM_ROUND_NEAREST 0x0000
#define _MM_ROUND_DOWN 0x2000
#define _MM_ROUND_UP 0x4000
#define _MM_ROUND_TOWARD_ZERO 0x6000
/* Flush zero mode macros. */
#define _MM_FLUSH_ZERO_MASK 0x8000
#define _MM_FLUSH_ZERO_ON 0x8000
#define _MM_FLUSH_ZERO_OFF 0x0000

enum _mm_hint {
  _MM_HINT_NTA = 0,
  _MM_HINT_T0 = 1,
  _MM_HINT_T1 = 2,
  _MM_HINT_T2 = 3,
};

__forceinline __m128i _mm_cvtps_epi32(__m128 a) {
  return __riscv_vfcvt_x_f_v_i32m1(a, 4);
}

__forceinline int _mm_cvtsi128_si32(__m128i a) {
  return __riscv_vmv_x_s_i32m1_i32(a);
}

__forceinline float _mm_cvtss_f32 (__m128 a) {
  return __riscv_vfmv_f_s_f32m1_f32(a);
}

__forceinline __m128 _mm_dp_ps(__m128 a, __m128 b, const int imm8) {
  vfloat32m1_t zeros = __riscv_vfmv_v_f_f32m1(0, 4);
  vbool32_t high = __riscv_vreinterpret_v_i32m1_b32(__riscv_vmv_s_x_i32m1(imm8 >> 4, 1));
  vbool32_t low = __riscv_vreinterpret_v_i32m1_b32(__riscv_vmv_s_x_i32m1(imm8 & 0xf, 1));
  vfloat32m1_t sum = __riscv_vfredusum_vs_f32m1_f32m1_m(high, __riscv_vfmul(a, b, 4), zeros, 4);
  return vreinterpretq_f32_m128(__riscv_vrgather_vx_f32m1_mu(low, zeros, sum, 0, 4));
}

__forceinline __int64 _mm_cvtsi128_si64 (__m128i a) {
  return __riscv_vmv_x_s_i64m1_i64(__riscv_vreinterpret_v_i32m1_i64m1(a));
}

__forceinline unsigned int _mm_getcsr(void) {
  return 0;
}

__forceinline void _mm_setcsr(unsigned int a) {
  int rm;

  switch (a) {
  case _MM_ROUND_TOWARD_ZERO:
    // FIXME: I can't find the straightforward mapping of this.
    rm = 0b01;
    break;
  case _MM_ROUND_DOWN:
    rm = 0b10;
    break;
  case _MM_ROUND_UP:
    rm = 0b00;
    break;
  default:  //_MM_ROUND_NEAREST
    rm = 0b01;
  }

  asm volatile("csrw vxrm,%0" :: "r"(rm));
}

__forceinline void _mm_mfence (void) {
  __sync_synchronize();
}

__forceinline void _mm_pause (void) {
  __asm__ __volatile__("fence.i\n\t"
                       "fence r, r\n\t");
}

__forceinline void _mm_prefetch (char const* p, int i) {
  (void)i;
  __builtin_prefetch(p);
}

__forceinline __m128 _mm_round_ps(__m128 a, int rounding) {
  vfloat32m1_t _a = vreinterpretq_m128_f32(a);
  switch (rounding) {
  case (_MM_FROUND_TO_NEAREST_INT | _MM_FROUND_NO_EXC):
    return __riscv_vfcvt_f_x_v_f32m1(__riscv_vfcvt_x_f_v_i32m1_rm(_a, 0, 4), 4);
  case (_MM_FROUND_TO_NEG_INF | _MM_FROUND_NO_EXC):
    return _mm_floor_ps(a);
  case (_MM_FROUND_TO_POS_INF | _MM_FROUND_NO_EXC):
    return _mm_ceil_ps(a);
  case (_MM_FROUND_TO_ZERO | _MM_FROUND_NO_EXC):
    return __riscv_vfcvt_f_x_v_f32m1(__riscv_vfcvt_x_f_v_i32m1_rm(_a, 1, 4), 4);
  default:  //_MM_FROUND_CUR_DIRECTION
    return __riscv_vfcvt_f_x_v_f32m1(__riscv_vfcvt_x_f_v_i32m1(_a, 4), 4);
  }
}

__forceinline int _mm_popcnt_u32(unsigned int a) {
  return __builtin_popcount(a);
}

__forceinline int64_t _mm_popcnt_u64(uint64_t a) {
  return __builtin_popcount(a);
}

__forceinline __m128 _mm_fmadd_ps(__m128 a, __m128 b, __m128 c) {
  vfloat32m1_t _a = vreinterpretq_m128_f32(a);
  vfloat32m1_t _b = vreinterpretq_m128_f32(b);
  vfloat32m1_t _c = vreinterpretq_m128_f32(c);
  return vreinterpretq_f32_m128(__riscv_vfmacc_vv_f32m1(_c, _a, _b, 4));
}

__forceinline __m128 _mm_fmsub_ps(__m128 a, __m128 b, __m128 c) {
  vfloat32m1_t _a = vreinterpretq_m128_f32(a);
  vfloat32m1_t _b = vreinterpretq_m128_f32(b);
  vfloat32m1_t _c = vreinterpretq_m128_f32(c);
  return vreinterpretq_f32_m128(__riscv_vfmsac_vv_f32m1(_c, _a, _b, 4));
}

__forceinline __m128 _mm_fnmadd_ps(__m128 a, __m128 b, __m128 c) {
  vfloat32m1_t _a = vreinterpretq_m128_f32(a);
  vfloat32m1_t _b = vreinterpretq_m128_f32(b);
  vfloat32m1_t _c = vreinterpretq_m128_f32(c);
  return vreinterpretq_f32_m128(__riscv_vfnmsac_vv_f32m1(_c, _a, _b, 4));
}

__forceinline __m128 _mm_fnmsub_ps(__m128 a, __m128 b, __m128 c) {
  vfloat32m1_t _a = vreinterpretq_m128_f32(a);
  vfloat32m1_t _b = vreinterpretq_m128_f32(b);
  vfloat32m1_t _c = vreinterpretq_m128_f32(c);
  return vreinterpretq_f32_m128(__riscv_vfnmacc_vv_f32m1(_c, _a, _b, 4));
}

/* Dummy defines for floating point control */
#define _MM_MASK_MASK 0x1f80
#define _MM_MASK_DIV_ZERO 0x200
// #define _MM_FLUSH_ZERO_ON 0x8000
#define _MM_MASK_DENORM 0x100
#define _MM_SET_EXCEPTION_MASK(x)
// #define _MM_SET_FLUSH_ZERO_MODE(x)
