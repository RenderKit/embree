// Copyright 2009-2021 Intel Corporation
// SPDX-License-Identifier: Apache-2.0

#pragma once

#include "../../../common/sys/alloc.h"

#if defined(EMBREE_SYCL_SUPPORT)
#include <sycl/sycl.hpp>
#endif

namespace embree
{
#if defined(EMBREE_SYCL_SUPPORT)

  /* enables SYCL USM allocation */
  void enableUSMAllocTutorial(sycl::context* context, sycl::device* device);

  /* disables SYCL USM allocation */
  void disableUSMAllocTutorial();

#endif

#define ALIGNED_STRUCT_USM_(align)                                          \
  void* operator new(size_t size) { return alignedUSMMalloc(size,align); }   \
  void operator delete(void* ptr) { alignedUSMFree(ptr); }                   \
  void* operator new[](size_t size) { return alignedUSMMalloc(size,align); } \
  void operator delete[](void* ptr) { alignedUSMFree(ptr); }

  /*! aligned allocation using SYCL USM */
  void* alignedUSMMalloc(size_t size, size_t align = 16, EmbreeUSMMode mode = EmbreeUSMMode::DEVICE_READ_ONLY);
  void alignedUSMFree(void* ptr);

}