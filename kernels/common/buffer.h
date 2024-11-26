// Copyright 2009-2021 Intel Corporation
// SPDX-License-Identifier: Apache-2.0

#pragma once

#include "default.h"
#include "device.h"

namespace embree
{
  /*! Implements an API data buffer object. This class may or may not own the data. */
  class Buffer : public RefCount
  {
  private:
    char* alloc(void* ptr_in, bool &shared, EmbreeMemoryType memoryType)
    {
      if (ptr_in)
      {
        shared = true;
        return (char*)ptr_in;
      }
      else
      {
        shared = false;
        device->memoryMonitor(this->bytes(), false);
        size_t b = (this->bytes()+15) & ssize_t(-16);
        return (char*)device->malloc(b,16,memoryType);
      }
    }

  public:
    Buffer(Device* device, size_t numBytes_in, void* ptr_in)
      : device(device), numBytes(numBytes_in)
    {
      device->refInc();

      ptr = alloc(ptr_in, shared, EmbreeMemoryType::SHARED);
#if defined(EMBREE_SYCL_SUPPORT)
      dshared = true;
      dptr = ptr;
#endif
    }

    Buffer(Device* device, size_t numBytes_in, void* ptr_in, void* dptr_in)
      : device(device), numBytes(numBytes_in)
    {
      device->refInc();

#if defined(EMBREE_SYCL_SUPPORT)
      if (device->is_gpu())
      {
        ptr  = alloc( ptr_in,  shared, EmbreeMemoryType::UNKNOWN);
        dptr = alloc(dptr_in, dshared, EmbreeMemoryType::DEVICE);
      }
      else
#endif
      {
        ptr = alloc(ptr_in, shared, EmbreeMemoryType::SHARED);
#if defined(EMBREE_SYCL_SUPPORT)
        dshared = true;
        dptr = ptr;
#endif
      }
    }

    /*! Buffer destruction */
    virtual ~Buffer() {
      free();
      device->refDec();
    }

    /*! this class is not copyable */
  private:
    Buffer(const Buffer& other) DELETED; // do not implement
    Buffer& operator =(const Buffer& other) DELETED; // do not implement

  public:

    /*! frees the buffer */
    virtual void free()
    {
      if (!shared && ptr) {
        device->free(ptr);
        device->memoryMonitor(-ssize_t(this->bytes()), true);
        ptr = nullptr;
      }
#if defined(EMBREE_SYCL_SUPPORT)
      if (!dshared && dptr) {
        device->free(dptr);
        device->memoryMonitor(-ssize_t(this->bytes()), true);
        dptr = nullptr;
      }
#endif
    }

    /*! gets buffer pointer */
    void* data()
    {
      /* report error if buffer is not existing */
      if (!device)
        throw_RTCError(RTC_ERROR_INVALID_ARGUMENT, "invalid buffer specified");

      /* return buffer */
      return ptr;
    }

    /*! returns pointer to first element */
    __forceinline char* getPtr() const {
      return ptr;
    }

    /*! returns pointer to first element */
    __forceinline virtual char* getHostPtr() const {
      return ptr;
    }

    /*! returns pointer to first element */
    __forceinline virtual char* getDevicePtr() const {
#if defined(EMBREE_SYCL_SUPPORT)
      return dptr;
#else
      return ptr;
#endif
    }

    /*! returns the number of bytes of the buffer */
    __forceinline size_t bytes() const {
      return numBytes;
    }

    /*! returns true of the buffer is not empty */
    __forceinline operator bool() const {
      return ptr;
    }

    __forceinline void commit() {
#if defined(EMBREE_SYCL_SUPPORT)
      DeviceGPU* gpu_device = dynamic_cast<DeviceGPU*>(device);
      if (gpu_device) {
        sycl::queue queue(gpu_device->getGPUDevice());
        commit(queue);
        queue.wait_and_throw();
      }
#endif
    }

#if defined(EMBREE_SYCL_SUPPORT)
    __forceinline void commit(sycl::queue queue) {
      if (dptr == ptr) return;
      std::cout << "buffer memcpy host to device" << std::endl;
      queue.memcpy(dptr, ptr, numBytes);
    }
#endif

  public:
    Device* device;      //!< device to report memory usage to
    size_t numBytes;        //!< number of bytes in the buffer
    char* ptr;              //!< pointer to buffer data
    bool shared;            //!< set if memory is shared with application
#if defined(EMBREE_SYCL_SUPPORT)
    char* dptr;
    bool dshared;           //!< set if memory is shared with application
#endif
  };

  /*! An untyped contiguous range of a buffer. This class does not own the buffer content. */
  class RawBufferView
  {
  public:
    /*! Buffer construction */
    RawBufferView()
      : ptr_ofs(nullptr), dptr_ofs(nullptr), stride(0), num(0), format(RTC_FORMAT_UNDEFINED), modCounter(1), modified(true), userData(0) {}

  public:
    /*! sets the buffer view */
    void set(const Ref<Buffer>& buffer_in, size_t offset_in, size_t stride_in, size_t num_in, RTCFormat format_in)
    {
      if ((offset_in + stride_in * num_in) > (stride_in * buffer_in->numBytes))
        throw_RTCError(RTC_ERROR_INVALID_ARGUMENT, "buffer range out of bounds");

      ptr_ofs = buffer_in->getHostPtr() + offset_in;
      dptr_ofs = buffer_in->getDevicePtr() + offset_in;
      stride = stride_in;
      num = num_in;
      format = format_in;
      modCounter++;
      modified = true;
      buffer = buffer_in;
    }

    /*! returns pointer to the first element */
    __forceinline char* getPtr() const {
      #if defined(__SYCL_DEVICE_ONLY__)
        return dptr_ofs;
      #else
        return ptr_ofs;
      #endif
    }

    /*! returns pointer to the i'th element */
    __forceinline char* getPtr(size_t i) const
    {
      #if defined(__SYCL_DEVICE_ONLY__)
        assert(i<num);
        return dptr_ofs + i*stride;
      #else
        return ptr_ofs + i*stride;
      #endif
    }

    /*! returns the number of elements of the buffer */
    __forceinline size_t size() const {
      return num;
    }

    /*! returns the number of bytes of the buffer */
    __forceinline size_t bytes() const {
      return num*stride;
    }

    /*! returns the buffer stride */
    __forceinline unsigned getStride() const
    {
      assert(stride <= unsigned(inf));
      return unsigned(stride);
    }

    /*! return the buffer format */
    __forceinline RTCFormat getFormat() const {
      return format;
    }

    /*! mark buffer as modified or unmodified */
    __forceinline void setModified() {
      modCounter++;
      modified = true;
    }

    /*! mark buffer as modified or unmodified */
    __forceinline bool isModified(unsigned int otherModCounter) const {
      return modCounter > otherModCounter;
    }

     /*! mark buffer as modified or unmodified */
    __forceinline bool isLocalModified() const {
      return modified;
    }

    /*! clear local modified flag */
    __forceinline void clearLocalModified() {
      modified = false;
    }

    /*! returns true of the buffer is not empty */
    __forceinline operator bool() const { 
      return ptr_ofs;
    }

    /*! checks padding to 16 byte check, fails hard */
    __forceinline void checkPadding16() const
    {
      if (ptr_ofs && num)
        volatile int MAYBE_UNUSED w = *((int*)getPtr(size()-1)+3); // FIXME: is failing hard avoidable?
    }

  public:
    char* ptr_ofs;      //!< base pointer plus offset
    char* dptr_ofs;     //!< base pointer plus offset in device memory
    size_t stride;      //!< stride of the buffer in bytes
    size_t num;         //!< number of elements in the buffer
    RTCFormat format;   //!< format of the buffer
    unsigned int modCounter; //!< version ID of this buffer
    bool modified;      //!< local modified data
    int userData;       //!< special data
    Ref<Buffer> buffer; //!< reference to the parent buffer
  };

  /*! A typed contiguous range of a buffer. This class does not own the buffer content. */
  template<typename T>
  class BufferView : public RawBufferView
  {
  public:
    typedef T value_type;

#if defined(__SYCL_DEVICE_ONLY__)
    /*! access to the ith element of the buffer */
    __forceinline       T& operator [](size_t i)       { assert(i<num); return *(T*)(dptr_ofs + i*stride); }
    __forceinline const T& operator [](size_t i) const { assert(i<num); return *(T*)(dptr_ofs + i*stride); }
#else
    /*! access to the ith element of the buffer */
    __forceinline       T& operator [](size_t i)       { assert(i<num); return *(T*)(ptr_ofs + i*stride); }
    __forceinline const T& operator [](size_t i) const { assert(i<num); return *(T*)(ptr_ofs + i*stride); }
#endif
  };

  template<>
  class BufferView<Vec3fa> : public RawBufferView
  {
  public:
    typedef Vec3fa value_type;

#if defined(EMBREE_SYCL_SUPPORT) && defined(__SYCL_DEVICE_ONLY__)

     /*! access to the ith element of the buffer */
    __forceinline const Vec3fa operator [](size_t i) const
    {
      assert(i<num);
      return Vec3fa::loadu(dptr_ofs + i*stride);
    }
    
    /*! writes the i'th element */
    __forceinline void store(size_t i, const Vec3fa& v)
    {
      assert(i<num);
      Vec3fa::storeu(dptr_ofs + i*stride, v);
    }
    
#else

    /*! access to the ith element of the buffer */
    __forceinline const Vec3fa operator [](size_t i) const
    {
      assert(i<num);
      return Vec3fa(vfloat4::loadu((float*)(ptr_ofs + i*stride)));
    }
    
    /*! writes the i'th element */
    __forceinline void store(size_t i, const Vec3fa& v)
    {
      assert(i<num);
      vfloat4::storeu((float*)(ptr_ofs + i*stride), (vfloat4)v);
    }
#endif
  };
}
