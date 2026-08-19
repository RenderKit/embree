// Copyright 2009-2020 Intel Corporation
// SPDX-License-Identifier: Apache-2.0

#pragma once

#include "../../sys/platform.h"

/* Make precision match SSE, at the cost of some performance */
#if !defined(EMBREE_ARM64)
#  define SSE2NEON_PRECISE_DIV 1
#  define SSE2NEON_PRECISE_SQRT 1
#endif

#include "sse2neon.h"

__forceinline __m128 _mm_abs_ps(__m128 a) { return vabsq_f32(a); }

__forceinline __m128 _mm_fmadd_ps (__m128 a, __m128 b, __m128 c) { return vfmaq_f32(c, a, b); }
__forceinline __m128 _mm_fnmadd_ps(__m128 a, __m128 b, __m128 c) { return vfmsq_f32(c, a, b); }
__forceinline __m128 _mm_fnmsub_ps(__m128 a, __m128 b, __m128 c) { return vnegq_f32(vfmaq_f32(c, a, b)); }
__forceinline __m128 _mm_fmsub_ps (__m128 a, __m128 b, __m128 c) { return vnegq_f32(vfmsq_f32(c, a, b)); }

__forceinline __m128 _mm_broadcast_ss (float const * mem_addr)
{
    return vdupq_n_f32(*mem_addr);
}

// AVX2 emulation leverages Intel FMA defs above.  Include after them.
#include "avx2neon.h"

/*
__forceinline int _mm_getcsr()
{
  return 0;
}

__forceinline void _mm_mfence()
{
  __sync_synchronize();
}
*/

__forceinline __m128i _mm_load4epu8_epi32(__m128i *ptr)
{
    uint8x8_t  t0 = vld1_u8((uint8_t*)ptr);
    uint16x8_t t1 = vmovl_u8(t0);
    uint32x4_t t2 = vmovl_u16(vget_low_u16(t1));
    return vreinterpretq_s32_u32(t2);
}

__forceinline __m128i _mm_load4epu16_epi32(__m128i *ptr)
{
    uint16x8_t t0 = vld1q_u16((uint16_t*)ptr);
    uint32x4_t t1 = vmovl_u16(vget_low_u16(t0));
    return vreinterpretq_s32_u32(t1);
}

__forceinline __m128i _mm_load4epi8_f32(__m128i *ptr)
{
    int8x8_t    t0 = vld1_s8((int8_t*)ptr);
    int16x8_t   t1 = vmovl_s8(t0);
    int32x4_t   t2 = vmovl_s16(vget_low_s16(t1));
    float32x4_t t3 = vcvtq_f32_s32(t2);
    return vreinterpretq_s32_f32(t3);
}

__forceinline __m128i _mm_load4epu8_f32(__m128i *ptr)
{
    uint8x8_t   t0 = vld1_u8((uint8_t*)ptr);
    uint16x8_t  t1 = vmovl_u8(t0);
    uint32x4_t  t2 = vmovl_u16(vget_low_u16(t1));
    return vreinterpretq_s32_u32(t2);
}

__forceinline __m128i _mm_load4epi16_f32(__m128i *ptr)
{
    int16x8_t   t0 = vld1q_s16((int16_t*)ptr);
    int32x4_t   t1 = vmovl_s16(vget_low_s16(t0));
    float32x4_t t2 = vcvtq_f32_s32(t1);
    return vreinterpretq_s32_f32(t2);
}
