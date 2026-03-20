// Copyright 2009-2021 Intel Corporation
// SPDX-License-Identifier: Apache-2.0

#pragma once

#include "../../common/sys/platform.h"
#include "../../common/sys/sysinfo.h"

namespace embree
{
#define DEFINE_SYMBOL2(type,name)               \
  typedef type (*name##Func)();                 \
  name##Func name;
  
#define DECLARE_SYMBOL2(type,name)                                       \
  namespace sse2   { extern type name(); }                           \
  namespace sse42  { extern type name(); }                           \
  namespace avx    { extern type name(); }                           \
  namespace avx2   { extern type name(); }                           \
  namespace avx512 { extern type name(); }                           \
  namespace avx10_1 { extern type name(); }                          \
  namespace avx10_2 { extern type name(); }                          \
  namespace apx { extern type name(); }                              \
  void name##_error2() { throw_RTCError(RTC_ERROR_UNKNOWN,"internal error in ISA selection for " TOSTRING(name)); } \
  type name##_error() { return type(name##_error2); }                   \
  type name##_zero() { return type(nullptr); }

#define DECLARE_ISA_FUNCTION(type,symbol,args)                            \
  namespace sse2   { extern type symbol(args); }                       \
  namespace sse42  { extern type symbol(args); }                       \
  namespace avx    { extern type symbol(args); }                       \
  namespace avx2   { extern type symbol(args); }                       \
  namespace avx512 { extern type symbol(args); }                       \
  namespace avx10_1 { extern type symbol(args); }                      \
  namespace avx10_2 { extern type symbol(args); }                      \
  namespace apx { extern type symbol(args); }                          \
  inline type symbol##_error(args) { throw_RTCError(RTC_ERROR_UNSUPPORTED_CPU,"function " TOSTRING(symbol) " not supported by your CPU"); } \
  typedef type (*symbol##Ty)(args);                                       \
  
#define DEFINE_ISA_FUNCTION(type,symbol,args)   \
  typedef type (*symbol##Func)(args);           \
  symbol##Func symbol;
  
#define ZERO_SYMBOL(features,intersector)                      \
  intersector = intersector##_zero;

#define INIT_SYMBOL(features,intersector)                      \
  intersector = decltype(intersector)(intersector##_error);

#define SELECT_SYMBOL_DEFAULT(features,intersector) \
  intersector = isa::intersector;

#if defined(__SSE__) || defined(__ARM_NEON)
#if !defined(EMBREE_TARGET_SIMD4)
#define EMBREE_TARGET_SIMD4
#endif
#endif

#if defined(EMBREE_TARGET_SSE42)
#define SELECT_SYMBOL_SSE42(features,intersector) \
  if ((features & SSE42) == SSE42) intersector = sse42::intersector;
#else
#define SELECT_SYMBOL_SSE42(features,intersector)
#endif

#if defined(EMBREE_TARGET_AVX) || defined(__AVX__)
#if !defined(EMBREE_TARGET_SIMD8)
#define EMBREE_TARGET_SIMD8
#endif
#if defined(__AVX__) // if default ISA is >= AVX we treat AVX target as default target
#define SELECT_SYMBOL_AVX(features,intersector)                 \
  if ((features & ISA) == ISA) intersector = isa::intersector;
#else
#define SELECT_SYMBOL_AVX(features,intersector)                 \
  if ((features & AVX) == AVX) intersector = avx::intersector;
#endif
#else
#define SELECT_SYMBOL_AVX(features,intersector)
#endif

#if defined(EMBREE_TARGET_AVX2)
#if !defined(EMBREE_TARGET_SIMD8)
#define EMBREE_TARGET_SIMD8
#endif
#define SELECT_SYMBOL_AVX2(features,intersector) \
  if ((features & AVX2) == AVX2) intersector = avx2::intersector;
#else
#define SELECT_SYMBOL_AVX2(features,intersector)
#endif

#if defined(EMBREE_TARGET_AVX512)
#if !defined(EMBREE_TARGET_SIMD16)
#define EMBREE_TARGET_SIMD16
#endif
#define SELECT_SYMBOL_AVX512(features,intersector) \
  if ((features & AVX512) == AVX512) intersector = avx512::intersector;
#else
#define SELECT_SYMBOL_AVX512(features,intersector)
#endif

#if defined(EMBREE_TARGET_AVX10_1)
#if !defined(EMBREE_TARGET_SIMD16)
#define EMBREE_TARGET_SIMD16
#endif
#define SELECT_SYMBOL_AVX10_1(features,intersector) \
  if ((features & AVX10_1) == AVX10_1) intersector = avx10_1::intersector;
#else
#define SELECT_SYMBOL_AVX10_1(features,intersector)
#endif

#if defined(EMBREE_TARGET_AVX10_2)
#if !defined(EMBREE_TARGET_SIMD16)
#define EMBREE_TARGET_SIMD16
#endif
#define SELECT_SYMBOL_AVX10_2(features,intersector) \
  if ((features & AVX10_2) == AVX10_2) intersector = avx10_2::intersector;
#else
#define SELECT_SYMBOL_AVX10_2(features,intersector)
#endif

#if defined(EMBREE_TARGET_APX)
#if !defined(EMBREE_TARGET_SIMD16)
#define EMBREE_TARGET_SIMD16
#endif
#define SELECT_SYMBOL_APX(features,intersector) \
  if ((features & APX) == APX) intersector = apx::intersector;
#else
#define SELECT_SYMBOL_APX(features,intersector)
#endif


// Macro expansion helpers: define ISA combo macros from ISA lists.
// Usage: ISA_EXPAND_N(features,intersector, ISA1, ISA2, ..., ISAN)
// Example: ISA_EXPAND_3(f,i, DEFAULT, SSE42, AVX) expands to:
//   SELECT_SYMBOL_DEFAULT(f,i); SELECT_SYMBOL_SSE42(f,i); SELECT_SYMBOL_AVX(f,i);
#define ISA_EXPAND_2(f,i, A,B) \
  SELECT_SYMBOL_##A(f,i); SELECT_SYMBOL_##B(f,i)
#define ISA_EXPAND_3(f,i, A,B,C) \
  SELECT_SYMBOL_##A(f,i); SELECT_SYMBOL_##B(f,i); SELECT_SYMBOL_##C(f,i)
#define ISA_EXPAND_4(f,i, A,B,C,D) \
  SELECT_SYMBOL_##A(f,i); SELECT_SYMBOL_##B(f,i); SELECT_SYMBOL_##C(f,i); SELECT_SYMBOL_##D(f,i)
#define ISA_EXPAND_5(f,i, A,B,C,D,E) \
  SELECT_SYMBOL_##A(f,i); SELECT_SYMBOL_##B(f,i); SELECT_SYMBOL_##C(f,i); SELECT_SYMBOL_##D(f,i); SELECT_SYMBOL_##E(f,i)
#define ISA_EXPAND_6(f,i, A,B,C,D,E,F) \
  SELECT_SYMBOL_##A(f,i); SELECT_SYMBOL_##B(f,i); SELECT_SYMBOL_##C(f,i); SELECT_SYMBOL_##D(f,i); SELECT_SYMBOL_##E(f,i); SELECT_SYMBOL_##F(f,i)
#define ISA_EXPAND_7(f,i, A,B,C,D,E,F,G) \
  SELECT_SYMBOL_##A(f,i); SELECT_SYMBOL_##B(f,i); SELECT_SYMBOL_##C(f,i); SELECT_SYMBOL_##D(f,i); SELECT_SYMBOL_##E(f,i); SELECT_SYMBOL_##F(f,i); SELECT_SYMBOL_##G(f,i)
#define ISA_EXPAND_8(f,i, A,B,C,D,E,F,G,H) \
  SELECT_SYMBOL_##A(f,i); SELECT_SYMBOL_##B(f,i); SELECT_SYMBOL_##C(f,i); SELECT_SYMBOL_##D(f,i); SELECT_SYMBOL_##E(f,i); SELECT_SYMBOL_##F(f,i); SELECT_SYMBOL_##G(f,i); SELECT_SYMBOL_##H(f,i)

// Helper for initializer-style macros (INIT_, ZERO_)
#define INIT_EXPAND_2(f,i, A,B) \
  INIT_SYMBOL(f,i); SELECT_SYMBOL_##A(f,i); SELECT_SYMBOL_##B(f,i)
#define INIT_EXPAND_3(f,i, A,B,C) \
  INIT_SYMBOL(f,i); SELECT_SYMBOL_##A(f,i); SELECT_SYMBOL_##B(f,i); SELECT_SYMBOL_##C(f,i)
#define INIT_EXPAND_4(f,i, A,B,C,D) \
  INIT_SYMBOL(f,i); SELECT_SYMBOL_##A(f,i); SELECT_SYMBOL_##B(f,i); SELECT_SYMBOL_##C(f,i); SELECT_SYMBOL_##D(f,i)
#define INIT_EXPAND_5(f,i, A,B,C,D,E) \
  INIT_SYMBOL(f,i); SELECT_SYMBOL_##A(f,i); SELECT_SYMBOL_##B(f,i); SELECT_SYMBOL_##C(f,i); SELECT_SYMBOL_##D(f,i); SELECT_SYMBOL_##E(f,i)
#define INIT_EXPAND_6(f,i, A,B,C,D,E,F) \
  INIT_SYMBOL(f,i); SELECT_SYMBOL_##A(f,i); SELECT_SYMBOL_##B(f,i); SELECT_SYMBOL_##C(f,i); SELECT_SYMBOL_##D(f,i); SELECT_SYMBOL_##E(f,i); SELECT_SYMBOL_##F(f,i)
#define INIT_EXPAND_7(f,i, A,B,C,D,E,F,G) \
  INIT_SYMBOL(f,i); SELECT_SYMBOL_##A(f,i); SELECT_SYMBOL_##B(f,i); SELECT_SYMBOL_##C(f,i); SELECT_SYMBOL_##D(f,i); SELECT_SYMBOL_##E(f,i); SELECT_SYMBOL_##F(f,i); SELECT_SYMBOL_##G(f,i)

#define ZERO_EXPAND_5(f,i, A,B,C,D,E) \
  ZERO_SYMBOL(f,i); SELECT_SYMBOL_##A(f,i); SELECT_SYMBOL_##B(f,i); SELECT_SYMBOL_##C(f,i); SELECT_SYMBOL_##D(f,i); SELECT_SYMBOL_##E(f,i)

// ISA combination macros (add new combos here by defining with ISA_EXPAND_N or INIT_EXPAND_N)
#define SELECT_SYMBOL_DEFAULT_SSE42(features,intersector) \
  ISA_EXPAND_2(features,intersector, DEFAULT,SSE42)
#define SELECT_SYMBOL_DEFAULT_SSE42_AVX(features,intersector) \
  ISA_EXPAND_3(features,intersector, DEFAULT,SSE42,AVX)
#define SELECT_SYMBOL_DEFAULT_SSE42_AVX_AVX2(features,intersector) \
  ISA_EXPAND_4(features,intersector, DEFAULT,SSE42,AVX,AVX2)
#define SELECT_SYMBOL_DEFAULT_SSE42_AVX_AVX512(features,intersector) \
  ISA_EXPAND_4(features,intersector, DEFAULT,SSE42,AVX,AVX512)
#define SELECT_SYMBOL_DEFAULT_AVX(features,intersector) \
  ISA_EXPAND_2(features,intersector, DEFAULT,AVX)
#define SELECT_SYMBOL_DEFAULT_AVX_AVX2(features,intersector) \
  ISA_EXPAND_3(features,intersector, DEFAULT,AVX,AVX2)
#define SELECT_SYMBOL_DEFAULT_AVX_AVX512(features,intersector) \
  ISA_EXPAND_3(features,intersector, DEFAULT,AVX,AVX512)
#define SELECT_SYMBOL_DEFAULT_AVX_AVX2_AVX512(features,intersector) \
  ISA_EXPAND_4(features,intersector, DEFAULT,AVX,AVX2,AVX512)
#define SELECT_SYMBOL_DEFAULT_SSE42_AVX_AVX2_AVX512(features,intersector) \
  ISA_EXPAND_5(features,intersector, DEFAULT,SSE42,AVX,AVX2,AVX512)

#define SELECT_SYMBOL_INIT_AVX(features,intersector) \
  INIT_EXPAND_1(features,intersector, AVX)
#define SELECT_SYMBOL_INIT_AVX_AVX2(features,intersector) \
  INIT_EXPAND_2(features,intersector, AVX,AVX2)
#define SELECT_SYMBOL_INIT_AVX_AVX512(features,intersector) \
  INIT_EXPAND_2(features,intersector, AVX,AVX512)
#define SELECT_SYMBOL_INIT_AVX_AVX2_AVX512(features,intersector) \
  INIT_EXPAND_3(features,intersector, AVX,AVX2,AVX512)

#define SELECT_SYMBOL_INIT_SSE42_AVX_AVX2(features,intersector) \
  INIT_EXPAND_3(features,intersector, SSE42,AVX,AVX2)
#define SELECT_SYMBOL_INIT_SSE42_AVX_AVX2_AVX512(features,intersector) \
  INIT_EXPAND_4(features,intersector, SSE42,AVX,AVX2,AVX512)

#define SELECT_SYMBOL_ZERO_SSE42_AVX_AVX2_AVX512(features,intersector) \
  ZERO_EXPAND_5(features,intersector, DEFAULT,SSE42,AVX,AVX2,AVX512)

#define SELECT_SYMBOL_INIT_AVX512(features,intersector) \
  INIT_EXPAND_1(features,intersector, AVX512)

// AVX10_1 combinations
#define SELECT_SYMBOL_DEFAULT_AVX10_1(features,intersector) \
  ISA_EXPAND_2(features,intersector, DEFAULT,AVX10_1)
#define SELECT_SYMBOL_DEFAULT_AVX_AVX10_1(features,intersector) \
  ISA_EXPAND_3(features,intersector, DEFAULT,AVX,AVX10_1)
#define SELECT_SYMBOL_DEFAULT_AVX_AVX2_AVX10_1(features,intersector) \
  ISA_EXPAND_4(features,intersector, DEFAULT,AVX,AVX2,AVX10_1)
#define SELECT_SYMBOL_DEFAULT_AVX_AVX2_AVX512_AVX10_1(features,intersector) \
  ISA_EXPAND_5(features,intersector, DEFAULT,AVX,AVX2,AVX512,AVX10_1)
#define SELECT_SYMBOL_DEFAULT_SSE42_AVX_AVX2_AVX512_AVX10_1(features,intersector) \
  ISA_EXPAND_6(features,intersector, DEFAULT,SSE42,AVX,AVX2,AVX512,AVX10_1)

// AVX10_2 combinations
#define SELECT_SYMBOL_DEFAULT_AVX10_2(features,intersector) \
  ISA_EXPAND_2(features,intersector, DEFAULT,AVX10_2)
#define SELECT_SYMBOL_DEFAULT_AVX_AVX10_2(features,intersector) \
  ISA_EXPAND_3(features,intersector, DEFAULT,AVX,AVX10_2)
#define SELECT_SYMBOL_DEFAULT_AVX_AVX2_AVX10_2(features,intersector) \
  ISA_EXPAND_4(features,intersector, DEFAULT,AVX,AVX2,AVX10_2)
#define SELECT_SYMBOL_DEFAULT_AVX_AVX2_AVX512_AVX10_2(features,intersector) \
  ISA_EXPAND_5(features,intersector, DEFAULT,AVX,AVX2,AVX512,AVX10_2)
#define SELECT_SYMBOL_DEFAULT_SSE42_AVX_AVX2_AVX512_AVX10_2(features,intersector) \
  ISA_EXPAND_6(features,intersector, DEFAULT,SSE42,AVX,AVX2,AVX512,AVX10_2)

// APX combinations
#define SELECT_SYMBOL_DEFAULT_APX(features,intersector) \
  ISA_EXPAND_2(features,intersector, DEFAULT,APX)
#define SELECT_SYMBOL_DEFAULT_AVX_APX(features,intersector) \
  ISA_EXPAND_3(features,intersector, DEFAULT,AVX,APX)
#define SELECT_SYMBOL_DEFAULT_AVX_AVX2_APX(features,intersector) \
  ISA_EXPAND_4(features,intersector, DEFAULT,AVX,AVX2,APX)
#define SELECT_SYMBOL_DEFAULT_AVX_AVX2_AVX512_APX(features,intersector) \
  ISA_EXPAND_5(features,intersector, DEFAULT,AVX,AVX2,AVX512,APX)
#define SELECT_SYMBOL_DEFAULT_SSE42_AVX_AVX2_AVX512_APX(features,intersector) \
  ISA_EXPAND_6(features,intersector, DEFAULT,SSE42,AVX,AVX2,AVX512,APX)

// INIT_AVX10_1 combinations
#define SELECT_SYMBOL_INIT_AVX10_1(features,intersector) \
  INIT_EXPAND_1(features,intersector, AVX10_1)
#define SELECT_SYMBOL_INIT_AVX_AVX10_1(features,intersector) \
  INIT_EXPAND_2(features,intersector, AVX,AVX10_1)
#define SELECT_SYMBOL_INIT_AVX_AVX2_AVX10_1(features,intersector) \
  INIT_EXPAND_3(features,intersector, AVX,AVX2,AVX10_1)
#define SELECT_SYMBOL_INIT_AVX_AVX2_AVX512_AVX10_1(features,intersector) \
  INIT_EXPAND_4(features,intersector, AVX,AVX2,AVX512,AVX10_1)
#define SELECT_SYMBOL_INIT_SSE42_AVX_AVX2_AVX512_AVX10_1(features,intersector) \
  INIT_EXPAND_5(features,intersector, SSE42,AVX,AVX2,AVX512,AVX10_1)

// INIT_AVX10_2 combinations
#define SELECT_SYMBOL_INIT_AVX10_2(features,intersector) \
  INIT_EXPAND_1(features,intersector, AVX10_2)
#define SELECT_SYMBOL_INIT_AVX_AVX10_2(features,intersector) \
  INIT_EXPAND_2(features,intersector, AVX,AVX10_2)
#define SELECT_SYMBOL_INIT_AVX_AVX2_AVX10_2(features,intersector) \
  INIT_EXPAND_3(features,intersector, AVX,AVX2,AVX10_2)
#define SELECT_SYMBOL_INIT_AVX_AVX2_AVX512_AVX10_2(features,intersector) \
  INIT_EXPAND_4(features,intersector, AVX,AVX2,AVX512,AVX10_2)
#define SELECT_SYMBOL_INIT_SSE42_AVX_AVX2_AVX512_AVX10_2(features,intersector) \
  INIT_EXPAND_5(features,intersector, SSE42,AVX,AVX2,AVX512,AVX10_2)

// INIT_APX combinations
#define SELECT_SYMBOL_INIT_APX(features,intersector) \
  INIT_EXPAND_1(features,intersector, APX)
#define SELECT_SYMBOL_INIT_AVX_APX(features,intersector) \
  INIT_EXPAND_2(features,intersector, AVX,APX)
#define SELECT_SYMBOL_INIT_AVX_AVX2_APX(features,intersector) \
  INIT_EXPAND_3(features,intersector, AVX,AVX2,APX)
#define SELECT_SYMBOL_INIT_AVX_AVX2_AVX512_APX(features,intersector) \
  INIT_EXPAND_4(features,intersector, AVX,AVX2,AVX512,APX)
#define SELECT_SYMBOL_INIT_AVX512_AVX10_1_AVX10_2_APX(features,intersector) \
  INIT_EXPAND_4(features,intersector, AVX512,AVX10_1,AVX10_2,APX)
#define SELECT_SYMBOL_INIT_SSE42_AVX_AVX2_AVX512_APX(features,intersector) \
  INIT_EXPAND_5(features,intersector, SSE42,AVX,AVX2,AVX512,APX)

// AVX10_1 + AVX10_2 combinations
#define SELECT_SYMBOL_DEFAULT_AVX10_1_AVX10_2(features,intersector) \
  ISA_EXPAND_3(features,intersector, DEFAULT,AVX10_1,AVX10_2)
#define SELECT_SYMBOL_DEFAULT_AVX_AVX10_1_AVX10_2(features,intersector) \
  ISA_EXPAND_4(features,intersector, DEFAULT,AVX,AVX10_1,AVX10_2)
#define SELECT_SYMBOL_INIT_AVX10_1_AVX10_2(features,intersector) \
  INIT_EXPAND_2(features,intersector, AVX10_1,AVX10_2)
#define SELECT_SYMBOL_INIT_AVX_AVX10_1_AVX10_2(features,intersector) \
  INIT_EXPAND_3(features,intersector, AVX,AVX10_1,AVX10_2)

// AVX10_1 + AVX10_2 + APX combinations
#define SELECT_SYMBOL_DEFAULT_AVX10_1_AVX10_2_APX(features,intersector) \
  ISA_EXPAND_4(features,intersector, DEFAULT,AVX10_1,AVX10_2,APX)
#define SELECT_SYMBOL_DEFAULT_AVX_AVX10_1_AVX10_2_APX(features,intersector) \
  ISA_EXPAND_5(features,intersector, DEFAULT,AVX,AVX10_1,AVX10_2,APX)
#define SELECT_SYMBOL_INIT_AVX10_1_AVX10_2_APX(features,intersector) \
  INIT_EXPAND_3(features,intersector, AVX10_1,AVX10_2,APX)
#define SELECT_SYMBOL_INIT_AVX_AVX10_1_AVX10_2_APX(features,intersector) \
  INIT_EXPAND_4(features,intersector, AVX,AVX10_1,AVX10_2,APX)

// Full chains: DEFAULT_AVX_AVX2_AVX512_AVX10_1_AVX10_2_APX and SSE42 variants
#define SELECT_SYMBOL_DEFAULT_SSE42_AVX_AVX2_AVX512_AVX10_1_AVX10_2(features,intersector) \
  ISA_EXPAND_7(features,intersector, DEFAULT,SSE42,AVX,AVX2,AVX512,AVX10_1,AVX10_2)
#define SELECT_SYMBOL_DEFAULT_SSE42_AVX_AVX2_AVX512_AVX10_1_AVX10_2_APX(features,intersector) \
  ISA_EXPAND_8(features,intersector, DEFAULT,SSE42,AVX,AVX2,AVX512,AVX10_1,AVX10_2,APX)
#define SELECT_SYMBOL_DEFAULT_AVX_AVX2_AVX512_AVX10_1_AVX10_2_APX(features,intersector) \
  ISA_EXPAND_7(features,intersector, DEFAULT,AVX,AVX2,AVX512,AVX10_1,AVX10_2,APX)
#define SELECT_SYMBOL_INIT_SSE42_AVX_AVX2_AVX512_AVX10_1_AVX10_2_APX(features,intersector) \
  INIT_EXPAND_7(features,intersector, SSE42,AVX,AVX2,AVX512,AVX10_1,AVX10_2,APX)
#define SELECT_SYMBOL_INIT_AVX_AVX2_AVX512_AVX10_1_AVX10_2_APX(features,intersector) \
  INIT_EXPAND_6(features,intersector, AVX,AVX2,AVX512,AVX10_1,AVX10_2,APX)

// Micro helpers for 1-ISA initializers
#define INIT_EXPAND_1(f,i, A) \
  INIT_SYMBOL(f,i); SELECT_SYMBOL_##A(f,i)

#define SELECT_SYMBOL_SSE42_AVX_AVX2(features,intersector) \
  SELECT_SYMBOL_SSE42(features,intersector);         \
  SELECT_SYMBOL_AVX(features,intersector);           \
  SELECT_SYMBOL_AVX2(features,intersector);

  struct VerifyMultiTargetLinking {
    static __noinline int64_t getISA(int depth = 5) { 
      if (depth == 0) return ISA; 
      else return getISA(depth-1); 
    }
  };
  namespace sse2   { int64_t getISA(); };
  namespace sse42  { int64_t getISA(); };
  namespace avx    { int64_t getISA(); };
  namespace avx2   { int64_t getISA(); };
  namespace avx512 { int64_t getISA(); };
  namespace avx10_1 { int64_t getISA(); };
  namespace avx10_2 { int64_t getISA(); };
  namespace apx { int64_t getISA(); };
}
