// Copyright 2009-2024 Intel Corporation
// SPDX-License-Identifier: Apache-2.0

#pragma once

#include <immintrin.h>
#include "../../sys/platform.h"

namespace embree
{
#if defined(_MSC_VER) && defined(_ARM64_)
  // On ARM64 MSVC, __m128 and __m128i are both aliased to __n128 in arm_neon.h,
  // causing C++ overload ambiguity. We wrap them in unique types to disambiguate.
  
  struct __m128_wrapper {
    __m128 data;
    __forceinline __m128_wrapper() {}
    __forceinline __m128_wrapper(__m128 v) : data(v) {}
    __forceinline operator __m128() const { return data; }
    __forceinline operator __m128&() { return data; }
  };
  
  struct __m128i_wrapper {
    __m128i data;
    __forceinline __m128i_wrapper() {}
    __forceinline __m128i_wrapper(__m128i v) : data(v) {}
    __forceinline operator __m128i() const { return data; }
    __forceinline operator __m128i&() { return data; }
  };
  
  struct __m128d_wrapper {
    __m128d data;
    __forceinline __m128d_wrapper() {}
    __forceinline __m128d_wrapper(__m128d v) : data(v) {}
    __forceinline operator __m128d() const { return data; }
    __forceinline operator __m128d&() { return data; }
  };

#else
  // On other platforms, use identity "wrappers" that compile away to nothing
  template<typename T> struct identity_wrapper {
    T data;
    __forceinline identity_wrapper() {}
    __forceinline identity_wrapper(T v) : data(v) {}
    __forceinline operator T() const { return data; }
    __forceinline operator T&() { return data; }
  };
  
  using __m128_wrapper = identity_wrapper<__m128>;
  using __m128i_wrapper = identity_wrapper<__m128i>;
  using __m128d_wrapper = identity_wrapper<__m128d>;
#endif
}
