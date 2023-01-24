// Copyright 2009-2021 Intel Corporation
// SPDX-License-Identifier: Apache-2.0

#pragma once

#include "../../common/sys/platform.h"
#include "../../common/sys/sycl.h"
#include "../../common/sys/vector.h"
#include "../../common/math/bbox.h"
#include "../../include/embree4/rtcore.h"
 
namespace embree
{
  class Scene;
  
  void* rthwifAllocAccelBuffer(size_t bytes, sycl::device device, sycl::context context);

  void rthwifFreeAccelBuffer(void* ptr, sycl::context context);
  
  /*! allocator that performs BVH memory allocations */
  template<typename T>
    struct AccelAllocator
    {
      typedef T value_type;
      typedef T* pointer;
      typedef const T* const_pointer;
      typedef T& reference;
      typedef const T& const_reference;
      typedef std::size_t size_type;
      typedef std::ptrdiff_t difference_type;
      
      AccelAllocator() {}
      
      AccelAllocator(sycl::device device, sycl::context context)
        : device(device), context(context) {}
      
      __forceinline pointer allocate( size_type n ) {
        return (pointer) rthwifAllocAccelBuffer(n*sizeof(T),device,context);
      }
      
      __forceinline void deallocate( pointer p, size_type n ) {
        rthwifFreeAccelBuffer(p,context);
      }
      
      __forceinline void construct( pointer p, const_reference val ) {
        new (p) T(val);
      }
      
      __forceinline void destroy( pointer p ) {
        p->~T();
      }

      sycl::device device;
      sycl::context context;
    };

  typedef vector_t<char,AccelAllocator<char>> AccelBuffer;
    
  void* rthwifInit(sycl::device device, sycl::context context);
  
  void rthwifCleanup(void* dispatchGlobalsPtr, sycl::context context);

  int rthwifIsSYCLDeviceSupported(const sycl::device& sycl_device);
  
  BBox3f rthwifBuild(Scene* scene, AccelBuffer& buffer_o);
}
