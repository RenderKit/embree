// Copyright 2009-2021 Intel Corporation
// SPDX-License-Identifier: Apache-2.0

#pragma once

#include "platform.h"

using sycl::float16;
using sycl::float8;
using sycl::float4;
using sycl::float3;
using sycl::float2;
using sycl::int16;
using sycl::int8;
using sycl::int4;
using sycl::int3;
using sycl::int2;
using sycl::uint16;
using sycl::uint8;
using sycl::uint4;
using sycl::uint3;
using sycl::uint2;
using sycl::uchar16;
using sycl::uchar8;
using sycl::uchar4;
using sycl::uchar3;
using sycl::uchar2;
using sycl::ushort16;
using sycl::ushort8;
using sycl::ushort4;
using sycl::ushort3;
using sycl::ushort2;

#ifdef __SYCL_DEVICE_ONLY__
SYCL_EXTERNAL extern "C" unsigned int intel_sub_group_ballot(bool valid);
#else
inline uint32_t intel_sub_group_ballot(bool valid) { return 0; }
#endif

#if !defined(__forceinline)
#define __forceinline          inline __attribute__((always_inline))
#endif

namespace embree
{
  __forceinline const sycl::sub_group this_sub_group() {
#if __LIBSYCL_MAJOR_VERSION >= 8
    return sycl::ext::oneapi::this_work_item::get_sub_group();
#else
    return sycl::ext::oneapi::experimental::this_sub_group();
#endif
  }

  __forceinline const uint32_t get_sub_group_local_id() {
    return this_sub_group().get_local_id()[0];
  }
}
