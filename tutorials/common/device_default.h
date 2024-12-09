// Copyright 2009-2021 Intel Corporation
// SPDX-License-Identifier: Apache-2.0

#pragma once

#if defined(EMBREE_SYCL_SUPPORT)
// If we use the internal clang frontend of the dpc++ compiler directly __INTEL_LLVM_COMPILER is not set.
// I think doing this is fine because normal clang can not compile dpcpp/sycl code anyway.
#if defined(__WIN32__) and defined(__clang__) and !defined(__INTEL_LLVM_COMPILER)
  #define __INTEL_LLVM_COMPILER
#endif
#endif

#if defined(EMBREE_SYCL_TUTORIAL)
#  define __SYCL_USE_NON_VARIADIC_SPIRV_OCL_PRINTF__
#  pragma clang diagnostic push
#  pragma clang diagnostic ignored "-Wdeprecated-declarations"
#  pragma clang diagnostic ignored "-W#pragma-messages"
#  include <sycl/sycl.hpp>
#  pragma clang diagnostic pop
#else
#  if !defined(SYCL_EXTERNAL)
#    define SYCL_EXTERNAL
#  endif
#endif

/* include embree API */
#include "../../include/embree4/rtcore.h"
RTC_NAMESPACE_USE
#include "../../kernels/config.h"

#include "alloc/alloc.h"

namespace embree
{
#if defined(EMBREE_SYCL_TUTORIAL) && defined(EMBREE_SYCL_SUPPORT)
  
  extern sycl::queue   *global_gpu_queue;
  extern sycl::context *global_gpu_context;
  extern sycl::device  *global_gpu_device;
 
  /* returns function pointer to be usable in SYCL kernel */
  template<auto F>
    inline decltype(F) getFunctionPointer() {
    return rtcGetSYCLDeviceFunctionPointer<F>(*global_gpu_queue);
  }
#define GET_FUNCTION_POINTER(f) getFunctionPointer<f>()

#else
  
#define GET_FUNCTION_POINTER(f) f

#endif

}
