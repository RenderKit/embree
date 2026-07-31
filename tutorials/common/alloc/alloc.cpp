// Copyright 2009-2021 Intel Corporation
// SPDX-License-Identifier: Apache-2.0

#include "alloc.h"

////////////////////////////////////////////////////////////////////////////////
/// All Platforms
////////////////////////////////////////////////////////////////////////////////
  
namespace embree
{
#if defined(EMBREE_SYCL_SUPPORT)
  
  __thread sycl::context* tls_context = nullptr;
  __thread sycl::device* tls_device = nullptr;
  
  void enableUSMAllocTutorial(sycl::context* context, sycl::device* device)
  {
    tls_context = context;
    tls_device = device;
  }

  void disableUSMAllocTutorial()
  {
    tls_context = nullptr;
    tls_device = nullptr;
  }

#endif

  void* alignedUSMMalloc(size_t size, size_t align, EmbreeUSMMode mode)
  {
#if defined(EMBREE_SYCL_SUPPORT)
    if (tls_context)
      return alignedSYCLMalloc(tls_context,tls_device,size,align,mode);
    else
#endif
      return alignedMalloc(size,align);
  }

  void alignedUSMFree(void* ptr)
  {
#if defined(EMBREE_SYCL_SUPPORT)
    if (tls_context)
      return alignedSYCLFree(tls_context,ptr);
    else
#endif
      return alignedFree(ptr);
  }

}