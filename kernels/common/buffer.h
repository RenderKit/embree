// ======================================================================== //
// Copyright 2009-2017 Intel Corporation                                    //
//                                                                          //
// Licensed under the Apache License, Version 2.0 (the "License");          //
// you may not use this file except in compliance with the License.         //
// You may obtain a copy of the License at                                  //
//                                                                          //
//     http://www.apache.org/licenses/LICENSE-2.0                           //
//                                                                          //
// Unless required by applicable law or agreed to in writing, software      //
// distributed under the License is distributed on an "AS IS" BASIS,        //
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. //
// See the License for the specific language governing permissions and      //
// limitations under the License.                                           //
// ======================================================================== //

#pragma once

#include "default.h"

namespace embree
{
  /*! An untyped contiguous range of a buffer. This class does not own the buffer content. */
  class RawBufferView
  {
  public:
    /*! Buffer construction */
    RawBufferView(size_t num = 0, size_t stride = 0)
      : ptr_ofs(nullptr), stride(stride), num(num) {}

  public:
    /*! sets shared buffer */
    void set(char* ptr_ofs_in, size_t stride_in)
    {
      ptr_ofs = ptr_ofs_in;
      stride = stride_in;
    }

    /*! returns pointer to first element */
    __forceinline const char* getPtr() const {
      return ptr_ofs;
    }

    /*! returns pointer to first element */
    __forceinline const char* getPtr(size_t i) const
    {
      assert(i<num);
      return ptr_ofs + i*stride;
    }

    /*! returns the number of elements of the buffer */
    __forceinline size_t size() const { 
      return num; 
    }

    /*! returns the number of bytes of the buffer */
    __forceinline size_t bytes() const { 
      return num*stride; 
    }
    
    /*! returns buffer stride */
    __forceinline unsigned getStride() const
    {
      assert(stride <= unsigned(inf));
      return unsigned(stride);
    }

  protected:
    char* ptr_ofs;   //!< base pointer plus offset
    size_t stride;   //!< stride of the buffer in bytes
    size_t num;      //!< number of elements in the buffer
  };

  /*! A typed contiguous range of a buffer. This class does not own the buffer content. */
  template<typename T>
  class BufferView : public RawBufferView
  {
  public:
    typedef T value_type;

    BufferView(size_t num = 0, size_t stride = 0) 
      : RawBufferView(num, stride) {}

    /*! access to the ith element of the buffer */
    __forceinline       T& operator[](size_t i)       { assert(i<num); return *(T*)(ptr_ofs + i*stride); }
    __forceinline const T& operator[](size_t i) const { assert(i<num); return *(T*)(ptr_ofs + i*stride); }
  };

  template<>
  class BufferView<Vec3fa> : public RawBufferView
  {
  public:
    typedef Vec3fa value_type;

    BufferView(size_t num = 0, size_t stride = 0) 
      : RawBufferView(num, stride) {}

    /*! access to the ith element of the buffer */
    __forceinline const Vec3fa operator[](size_t i) const
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
  };

  /*! Implements an API data buffer object. This class may or may not own the data. */
  template<typename T>
  class Buffer : public BufferView<T>
  {
  public:
    /*! Buffer construction */
    Buffer() 
      : device(nullptr), ptr(nullptr), shared(false), modified(true), userdata(0) {}
    
    Buffer(const BufferView<T>& other) 
      : BufferView<T>(other), device(nullptr), ptr(nullptr), shared(true), modified(true), userdata(0) {}

    /*! Buffer construction */
    Buffer(MemoryMonitorInterface* device, size_t num_in, size_t stride_in, bool allocate = false) 
      : BufferView<T>(num_in, stride_in), device(device), ptr(nullptr), shared(false), modified(true), userdata(0) 
    {
      if (allocate) alloc();
    }
    
    /*! Buffer destruction */
    ~Buffer() {
      free();
    }
    
    /*! this class is not copyable */
  private:
    Buffer(const Buffer& other) DELETED; // do not implement
    Buffer& operator=(const Buffer& other) DELETED; // do not implement
    
    /*! make the class movable */
  public:
    Buffer(Buffer&& other) : BufferView<T>(std::move(other))
    {
      device = other.device;       other.device = nullptr;
      ptr = other.ptr;             other.ptr = nullptr;
      shared = other.shared;       other.shared = false;
      modified = other.modified;   other.modified = false;
      userdata = other.userdata;   other.userdata = 0;
    }
    
    Buffer& operator=(Buffer&& other)
    {
      device = other.device;       other.device = nullptr;
      ptr = other.ptr;             other.ptr = nullptr;
      shared = other.shared;       other.shared = false;
      modified = other.modified;   other.modified = false;
      userdata = other.userdata;   other.userdata = 0;
      BufferView<T>::operator=(std::move(other));
      return *this;
    }
    
  public:
    /* inits the buffer */
    void newBuffer(MemoryMonitorInterface* device_in, size_t num_in, size_t stride_in) 
    {
      init(device_in, num_in, stride_in);
      alloc();
    }
    
    /* inits the buffer */
    void init(MemoryMonitorInterface* device_in, size_t num_in, size_t stride_in) 
    {
      free();
      device = device_in;
      ptr = nullptr;
      this->ptr_ofs = nullptr;
      this->num = num_in;
      this->stride = stride_in;
      shared = false;
      modified = true;
    }

    /*! sets shared buffer */
    void set(MemoryMonitorInterface* device_in, void* ptr_in, size_t ofs_in, size_t stride_in, size_t num_in)
    {
      free();
      device = device_in;
      ptr = (char*)ptr_in;
      if (num_in != (size_t)-1) this->num = num_in;
      shared = true;

      BufferView<T>::set(ptr+ofs_in, stride_in);
    }
    
    /*! allocated buffer */
    void alloc()
    {
      if (device) device->memoryMonitor(this->bytes(),false);
      size_t b = (this->bytes()+15)&ssize_t(-16);
      ptr = this->ptr_ofs = (char*)alignedMalloc(b);
    }
    
    /*! frees the buffer */
    void free()
    {
      if (shared) return;
      alignedFree(ptr); 
      if (device) device->memoryMonitor(-ssize_t(this->bytes()),true);
      ptr = nullptr; this->ptr_ofs = nullptr;
    }
    
    /*! gets buffer pointer */
    void* get()
    {
      /* report error if buffer is not existing */
      if (!device)
        throw_RTCError(RTC_ERROR_INVALID_ARGUMENT,"invalid buffer specified");
      
      /* return buffer */
      return ptr;
    }
    
    /*! mark buffer as modified or unmodified */
    __forceinline void setModified(bool b) {
      modified = b;
    }
    
    /*! mark buffer as modified or unmodified */
    __forceinline bool isModified() const {
      return modified;
    }
    
    /*! returns true of the buffer is not empty */
    __forceinline operator bool() const { 
      return ptr; 
    }
    
    /*! checks padding to 16 byte check, fails hard */
    __forceinline void checkPadding16() const 
    {
      if (ptr && RawBufferView::size()) 
        volatile int MAYBE_UNUSED w = *((int*)RawBufferView::getPtr(RawBufferView::size()-1)+3); // FIXME: is failing hard avoidable?
    }

  protected:
    MemoryMonitorInterface* device; //!< device to report memory usage to 
    char* ptr;       //!< pointer to buffer data
    bool shared;     //!< set if memory is shared with application
    bool modified;   //!< true if the buffer got modified
  public:
    int userdata;    //!< special data
  };
}
