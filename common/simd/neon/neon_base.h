// Copyright 2009-2021 Intel Corporation
// SPDX-License-Identifier: Apache-2.0

#pragma once

#include <arm_neon.h>

////////////////////////////////////////////////////////////////////////////////
// Standalone NEON shuffle macros (independent of sse2neon.h)
////////////////////////////////////////////////////////////////////////////////

// Byte table index for vqtbl1q_u8: shuffles 4 float elements within one 128-bit vector.
// _MN_SHUFFLE(i0,i1,i2,i3) produces element i0 at lane 0, i1 at lane 1, etc.
#define _MN_SHUFFLE(fp3, fp2, fp1, fp0) ((uint8x16_t){                          \
    (((fp3) * 4) + 0), (((fp3) * 4) + 1), (((fp3) * 4) + 2), (((fp3) * 4) + 3), \
    (((fp2) * 4) + 0), (((fp2) * 4) + 1), (((fp2) * 4) + 2), (((fp2) * 4) + 3), \
    (((fp1) * 4) + 0), (((fp1) * 4) + 1), (((fp1) * 4) + 2), (((fp1) * 4) + 3), \
    (((fp0) * 4) + 0), (((fp0) * 4) + 1), (((fp0) * 4) + 2), (((fp0) * 4) + 3)})

// Byte table index for vqtbl2q_u8: shuffles from two 128-bit vectors.
// fp3,fp2 index from first vector (0-3), fp1,fp0 index from second vector (4-7).
#define _MF_SHUFFLE(fp3, fp2, fp1, fp0) ((uint8x16_t){                                              \
    (((fp3) * 4) + 0), (((fp3) * 4) + 1), (((fp3) * 4) + 2), (((fp3) * 4) + 3),                     \
    (((fp2) * 4) + 0), (((fp2) * 4) + 1), (((fp2) * 4) + 2), (((fp2) * 4) + 3),                     \
    (((fp1) * 4) + 16 + 0), (((fp1) * 4) + 16 + 1), (((fp1) * 4) + 16 + 2), (((fp1) * 4) + 16 + 3), \
    (((fp0) * 4) + 16 + 0), (((fp0) * 4) + 16 + 1), (((fp0) * 4) + 16 + 2), (((fp0) * 4) + 16 + 3)})

namespace embree
{
  ////////////////////////////////////////////////////////////////////////////////
  // Pair of uint32x4_t to carry 8-lane mask data (NEON equivalent of __m256i for masks)
  ////////////////////////////////////////////////////////////////////////////////

  struct neon_uint32x8_t
  {
    uint32x4_t lo, hi;
  };

  ////////////////////////////////////////////////////////////////////////////////
  // NEON-native mask lookup (replaces mm_lookupmask_ps/mm_lookupmask_pd)
  ////////////////////////////////////////////////////////////////////////////////

  __forceinline float32x4_t neon_lookupmask_ps(int mask)
  {
    uint32_t lanes[4] = {
        (mask & 1) ? 0xFFFFFFFF : 0u,
        (mask & 2) ? 0xFFFFFFFF : 0u,
        (mask & 4) ? 0xFFFFFFFF : 0u,
        (mask & 8) ? 0xFFFFFFFF : 0u};
    return vreinterpretq_f32_u32(vld1q_u32(lanes));
  }

  __forceinline float64x2_t neon_lookupmask_pd(int mask)
  {
    uint64_t lo = (mask & 1) ? 0xFFFFFFFFFFFFFFFFULL : 0ULL;
    uint64_t hi = (mask & 2) ? 0xFFFFFFFFFFFFFFFFULL : 0ULL;
    uint64x2_t r = vcombine_u64(vcreate_u64(lo), vcreate_u64(hi));
    return vreinterpretq_f64_u64(r);
  }

  ////////////////////////////////////////////////////////////////////////////////
  // 8-wide shuffle helpers (operate on pairs of 128-bit NEON registers)
  //
  // These replace _mm256_permute_ps, _mm256_shuffle_ps, _mm256_permute2f128_ps,
  // _mm256_blend_ps, _mm256_movemask_ps, _mm256_testz_ps, etc.
  ////////////////////////////////////////////////////////////////////////////////

  // Extract sign bits from float32x4_t and pack into 8-bit integer
  __forceinline uint32_t neon_movemask_ps(float32x4_t v)
  {
    uint32x4_t shifted = vshrq_n_u32(vreinterpretq_u32_f32(v), 31);
    static const uint32x4_t bits = {1, 2, 4, 8};
    return vaddvq_u32(vmulq_u32(shifted, bits));
  }

  // Extract sign bits from float64x2_t and pack into 4-bit integer
  __forceinline uint32_t neon_movemask_pd(float64x2_t v)
  {
    uint64x2_t shifted = vshrq_n_u64(vreinterpretq_u64_f64(v), 63);
    uint64_t s0 = vgetq_lane_u64(shifted, 0);
    uint64_t s1 = vgetq_lane_u64(shifted, 1);
    return (uint32_t)(s0 * 1 + s1 * 2);
  }

  // Test if both vectors have no set bits (equivalent to _mm256_testz_ps)
  __forceinline int neon_testz_ps(float32x4_t a, float32x4_t b)
  {
    uint32x4_t and_result = vandq_u32(vreinterpretq_u32_f32(a), vreinterpretq_u32_f32(b));
    return vmaxvq_u32(and_result) == 0;
  }

  // Test if both double vectors have no set bits
  __forceinline int neon_testz_pd(float64x2_t a, float64x2_t b)
  {
    uint64x2_t and_result = vandq_u64(vreinterpretq_u64_f64(a), vreinterpretq_u64_f64(b));
    uint64_t r0 = vgetq_lane_u64(and_result, 0);
    uint64_t r1 = vgetq_lane_u64(and_result, 1);
    return (r0 | r1) == 0;
  }

  // Permute 128-bit lanes: shuffle4<i0,i1>(v)
  // i0 selects source lane for low 128 bits, i1 for high 128 bits (0=lo, 1=hi)
  template <int i0, int i1>
  __forceinline float32x4x2_t neon_permute2f128_ps(float32x4_t lo, float32x4_t hi)
  {
    float32x4x2_t r;
    r.val[0] = (i0 == 0) ? lo : hi;
    r.val[1] = (i1 == 0) ? lo : hi;
    return r;
  }

  template <int i0, int i1>
  __forceinline int32x4x2_t neon_permute2f128_si(int32x4_t lo, int32x4_t hi)
  {
    int32x4x2_t r;
    r.val[0] = (i0 == 0) ? lo : hi;
    r.val[1] = (i1 == 0) ? lo : hi;
    return r;
  }

  template <int i0, int i1>
  __forceinline float64x2x2_t neon_permute2f128_pd(float64x2_t lo, float64x2_t hi)
  {
    float64x2x2_t r;
    r.val[0] = (i0 == 0) ? lo : hi;
    r.val[1] = (i1 == 0) ? lo : hi;
    return r;
  }

  // Permute 128-bit lanes across two sources: shuffle4<i0,i1>(a, b)
  // i0,i1 in {0=a_lo, 1=a_hi, 2=b_lo, 3=b_hi}
  template <int i0, int i1>
  __forceinline float32x4x2_t neon_permute2f128_ps(float32x4_t a_lo, float32x4_t a_hi,
                                                   float32x4_t b_lo, float32x4_t b_hi)
  {
    float32x4_t sources[4] = {a_lo, a_hi, b_lo, b_hi};
    float32x4x2_t r;
    r.val[0] = sources[i0];
    r.val[1] = sources[i1];
    return r;
  }

  template <int i0, int i1>
  __forceinline int32x4x2_t neon_permute2f128_si(int32x4_t a_lo, int32x4_t a_hi,
                                                 int32x4_t b_lo, int32x4_t b_hi)
  {
    int32x4_t sources[4] = {a_lo, a_hi, b_lo, b_hi};
    int32x4x2_t r;
    r.val[0] = sources[i0];
    r.val[1] = sources[i1];
    return r;
  }

  // Blend two float32x4_t using a bitmask (1 = select from t, 0 = select from f)
  // mask must be all-ones or all-zeros per lane
  __forceinline float32x4_t neon_blendv_ps(float32x4_t f, float32x4_t t, float32x4_t mask)
  {
    return vbslq_f32(vreinterpretq_u32_f32(mask), t, f);
  }

  // Broadcast 4-wide shuffle to both halves of 8-wide (replaces _mm256_permute_ps)
  template <int i0, int i1, int i2, int i3>
  __forceinline float32x4x2_t neon_permute_ps(float32x4_t lo, float32x4_t hi)
  {
    float32x4x2_t r;
    r.val[0] = vreinterpretq_f32_u8(vqtbl1q_u8(vreinterpretq_u8_f32(lo), _MN_SHUFFLE(i0, i1, i2, i3)));
    r.val[1] = vreinterpretq_f32_u8(vqtbl1q_u8(vreinterpretq_u8_f32(hi), _MN_SHUFFLE(i0, i1, i2, i3)));
    return r;
  }

  template <int i>
  __forceinline float32x4x2_t neon_permute_ps(float32x4_t lo, float32x4_t hi)
  {
    return neon_permute_ps<i, i, i, i>(lo, hi);
  }

  // Cross-vector 4-wide shuffle to both halves (replaces _mm256_shuffle_ps)
  template <int i0, int i1, int i2, int i3>
  __forceinline float32x4x2_t neon_shuffle_ps(float32x4_t a_lo, float32x4_t a_hi,
                                              float32x4_t b_lo, float32x4_t b_hi)
  {
    float32x4x2_t r;
    uint8x16x2_t tbl_lo = {vreinterpretq_u8_f32(a_lo), vreinterpretq_u8_f32(b_lo)};
    uint8x16x2_t tbl_hi = {vreinterpretq_u8_f32(a_hi), vreinterpretq_u8_f32(b_hi)};
    r.val[0] = vreinterpretq_f32_u8(vqtbl2q_u8(tbl_lo, _MF_SHUFFLE(i0, i1, i2, i3)));
    r.val[1] = vreinterpretq_f32_u8(vqtbl2q_u8(tbl_hi, _MF_SHUFFLE(i0, i1, i2, i3)));
    return r;
  }

  // Integer variants of the shuffle helpers
  template <int i0, int i1, int i2, int i3>
  __forceinline int32x4x2_t neon_permute_epi32(int32x4_t lo, int32x4_t hi)
  {
    int32x4x2_t r;
    r.val[0] = vreinterpretq_s32_u8(vqtbl1q_u8(vreinterpretq_u8_s32(lo), _MN_SHUFFLE(i0, i1, i2, i3)));
    r.val[1] = vreinterpretq_s32_u8(vqtbl1q_u8(vreinterpretq_u8_s32(hi), _MN_SHUFFLE(i0, i1, i2, i3)));
    return r;
  }

  template <int i>
  __forceinline int32x4x2_t neon_permute_epi32(int32x4_t lo, int32x4_t hi)
  {
    return neon_permute_epi32<i, i, i, i>(lo, hi);
  }

  template <int i0, int i1, int i2, int i3>
  __forceinline int32x4x2_t neon_shuffle_epi32(int32x4_t a_lo, int32x4_t a_hi,
                                               int32x4_t b_lo, int32x4_t b_hi)
  {
    int32x4x2_t r;
    uint8x16x2_t tbl_lo = {vreinterpretq_u8_s32(a_lo), vreinterpretq_u8_s32(b_lo)};
    uint8x16x2_t tbl_hi = {vreinterpretq_u8_s32(a_hi), vreinterpretq_u8_s32(b_hi)};
    r.val[0] = vreinterpretq_s32_u8(vqtbl2q_u8(tbl_lo, _MF_SHUFFLE(i0, i1, i2, i3)));
    r.val[1] = vreinterpretq_s32_u8(vqtbl2q_u8(tbl_hi, _MF_SHUFFLE(i0, i1, i2, i3)));
    return r;
  }

  // moveldup equivalent: {a[0],a[0],a[2],a[2]} per lane — vtrn1q duplicates even-indexed lanes
  __forceinline float32x4x2_t neon_moveldup_ps(float32x4_t lo, float32x4_t hi)
  {
    float32x4x2_t r;
    r.val[0] = vtrn1q_f32(lo, lo);
    r.val[1] = vtrn1q_f32(hi, hi);
    return r;
  }

  // movehdup equivalent: {a[1],a[1],a[3],a[3]} per lane — vtrn2q duplicates odd-indexed lanes
  __forceinline float32x4x2_t neon_movehdup_ps(float32x4_t lo, float32x4_t hi)
  {
    float32x4x2_t r;
    r.val[0] = vtrn2q_f32(lo, lo);
    r.val[1] = vtrn2q_f32(hi, hi);
    return r;
  }

  // movedup equivalent: {a[0],a[1],a[0],a[1]} per lane — duplicate low 64 bits
  __forceinline float32x4x2_t neon_movedup_pd_as_ps(float32x4_t lo, float32x4_t hi)
  {
    float32x4x2_t r;
    r.val[0] = vcombine_f32(vget_low_f32(lo), vget_low_f32(lo));
    r.val[1] = vcombine_f32(vget_low_f32(hi), vget_low_f32(hi));
    return r;
  }

} // namespace embree
