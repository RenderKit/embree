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

  void* rthwifAllocAccelBuffer(Device* embree_device, size_t bytes, sycl::device device, sycl::context context, sycl::usm::alloc alloc_type);

  void rthwifFreeAccelBuffer(Device* embree_device, void* ptr, size_t bytes, sycl::context context);

  void* zeRTASInitExp(sycl::device device, sycl::context context);

  void rthwifCleanup(Device* embree_device, void* dispatchGlobalsPtr, sycl::context context);

  int rthwifIsSYCLDeviceSupported(const sycl::device& sycl_device);

  const bool isPVC(const ze_device_handle_t hDevice);

  /*! allocator that performs BVH memory allocations */
  template <typename T>
  struct AccelAllocator
  {
    typedef T value_type;
    typedef T *pointer;
    typedef const T *const_pointer;
    typedef T &reference;
    typedef const T &const_reference;
    typedef std::size_t size_type;
    typedef std::ptrdiff_t difference_type;

    AccelAllocator()
        : device(nullptr), context(nullptr), alloc_type(sycl::usm::alloc::unknown) {}

    AccelAllocator(Device *embree_device, const sycl::device &device, const sycl::context &context, sycl::usm::alloc alloc_type)
        : embree_device(embree_device), device(&device), context(&context), alloc_type(alloc_type) {}

    __forceinline pointer allocate(size_type n)
    {
      if (context && device)
        return (pointer)rthwifAllocAccelBuffer(embree_device, n * sizeof(T), *device, *context, alloc_type);
      else
        return nullptr;
    }

    __forceinline void deallocate(pointer p, size_type n)
    {
      if (context)
        rthwifFreeAccelBuffer(embree_device, p, n * sizeof(T), *context);
    }

    __forceinline void construct(pointer p, const_reference val)
    {
      new (p) T(val);
    }

    __forceinline void destroy(pointer p)
    {
      p->~T();
    }

  private:
    Device *embree_device;
    const sycl::device *device;
    const sycl::context *context;
    sycl::usm::alloc alloc_type;
  };

  typedef vector_t<char,AccelAllocator<char>> AccelBufferData;

  std::tuple<BBox3f, size_t> rthwifBuild(Scene* scene, AccelBufferData& buffer_o);

  // The buffers containing the HW acceleration structures corresponding to the scene. One for each time segment, stored in a contiguous chunk of memory.
  // On devices with unified memory a USM shared allocation is used and hwaccelHost and hwaccelDevice point to the same USM shared memory.
  // On devices without unified memory, a USM host allocation is used for BVH building and on scene commit the data is copied to a USM device allocation on the device
  struct AccelBuffer {
      AccelBuffer() {};
      AccelBuffer(Device *device);

      __forceinline char* getHWAccel(uint32_t time_segment) const {
#if defined(__SYCL_DEVICE_ONLY__)
        return hwaccel + time_segment * hwaccel_stride;
        //return (char*)accelBufferShared.data() + time_segment * hwaccel_stride;
#else
        if (unifiedMemory)
          return (char*)accelBufferShared.data() + time_segment * hwaccel_stride;
        else
          return (char*)accelBufferHost.data() + time_segment * hwaccel_stride;
#endif
      }

      __forceinline size_t getHWAccelSize() const {
        if (unifiedMemory)
          return accelBufferShared.size();
        else
          return accelBufferHost.size();
      }

#if !defined(__SYCL_DEVICE_ONLY__)
      __forceinline AccelBufferData& getAccelBufferData() {
        if (unifiedMemory)
          return accelBufferShared;
        else
          return accelBufferHost;
      }

      __forceinline char* getAccelBufferDeviceData(uint32_t time_segment) {
        if (unifiedMemory)
          return (char*)accelBufferShared.data() + time_segment * hwaccel_stride;
        else
          return (char*)accelBufferDevice.data() + time_segment * hwaccel_stride;
      }


      inline BBox3f const& getBounds() { return hwaccel_bounds; }

      void build(Scene* scene);
      sycl::event commit(sycl::queue queue);
#endif

  private:
    AccelBufferData accelBufferHost;   // only used for systems were unified memory is not available (dGPU)
    AccelBufferData accelBufferDevice; // only used for systems were unified memory is not available (dGPU)
    AccelBufferData accelBufferShared; // only used when system has unified memory between CPU and GPU (iGPU)
    char* hwaccel;                     // pointer to the accel buffer on the device, only valid after scene commit

    BBox3f hwaccel_bounds = empty;
    size_t hwaccel_stride; // the stride between two HW acceleration structures for different time segments stored in hwaccel.
    bool unifiedMemory;
    Device* device;
  };

}
