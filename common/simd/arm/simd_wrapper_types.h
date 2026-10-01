// Copyright 2009-2024 Intel Corporation
// SPDX-License-Identifier: Apache-2.0

#pragma once

#include "../../sys/platform.h"

namespace embree
{
#if defined(_MSC_VER) && defined(_M_ARM64)
  // On ARM64 MSVC, __m128 and __m128i are both aliased to __n128 in arm_neon.h,
  // causing C++ overload ambiguity (and silent, incorrect overload resolution)
  // whenever both an implicit "operator __m128()" and an implicit "operator
  // __m128i()" are reachable for the same call. We wrap them in unique types
  // to disambiguate the conversions at compile time.

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
  // On other platforms, keep concrete wrappers to avoid attribute-bearing
  // template arguments such as __m128 in identity_wrapper<__m128>.
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
#endif
}
