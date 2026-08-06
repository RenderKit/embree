#pragma once

#define SSE2RVV_PRECISE_DIV    1
#define SSE2RVV_PRECISE_SQRT   1
#define SSE2RVV_PRECISE_MINMAX 1

#include "sse2rvv.h"

#define _MM_SHUFFLE(fp3, fp2, fp1, fp0) \
    (((fp3) << 6) | ((fp2) << 4) | ((fp1) << 2) | ((fp0)))

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

__forceinline int _mm_cvtsi128_si32(__m128i a) {
  return __riscv_vmv_x_s_i32m1_i32(a);
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
  union {
    fcsr_bitfield field;
    uint32_t value;
  } r;

  __asm__ volatile("csrr %0, fcsr" : "=r"(r));

  switch (r.field.frm) {
  case __RISCV_FRM_RTZ:
    return _MM_ROUND_TOWARD_ZERO;
  case __RISCV_FRM_RDN:
    return _MM_ROUND_DOWN;
  case __RISCV_FRM_RUP:
    return _MM_ROUND_UP;
  default:
    return _MM_ROUND_NEAREST;
  }
}

__forceinline void _mm_setcsr(unsigned int a) {
  _MM_SET_ROUNDING_MODE(a);
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
#define _MM_SET_FLUSH_ZERO_MODE(x)
