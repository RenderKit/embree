// ======================================================================== //
// Copyright 2009-2015 Intel Corporation                                    //
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
  /*! Implements a data buffer. */
  class Buffer
  {
  public:

    /*! Buffer construction */
    Buffer (); 

    /*! Buffer construction */
    Buffer (MemoryMonitorInterface* device, size_t num_in, size_t stride_in); 
    
    /*! Buffer destruction */
    ~Buffer ();
      
  public:
    
    /*! initialized the buffer */
    void init(MemoryMonitorInterface* device, size_t num_in, size_t stride_in);

    /*! sets shared buffer */
    void set(void* ptr_in, size_t ofs_in, size_t stride_in);

    /*! sets shared buffer */
    void set(void* ptr);

    /*! allocated buffer */
    void alloc();

    /*! fees the buffer */
    void free();

    /*! maps the buffer */
    void* map(atomic_t& cntr);
    
    /*! unmaps the buffer */
    void unmap(atomic_t& cntr);

    /*! checks if the buffer is mapped */
    __forceinline bool isMapped() const {
      return mapped; 
    }

    /*! mark buffer as modified or unmodified */
    __forceinline void setModified(bool b) {
      modified = b;
    }

    /*! mark buffer as modified or unmodified */
    __forceinline bool isModified() const {
      return modified;
    }

    /*! returns the number of elements of the buffer */
    __forceinline size_t size() const { 
      return num; 
    }

    /*! returns true of the buffer is not empty */
    __forceinline operator bool() { 
      return ptr; 
    }

    /*! returns pointer to first element */
    __forceinline const char* getPtr( size_t i = 0 ) const {
      return ptr_ofs + i*stride;
    }

    /*! returns buffer stride */
    __forceinline unsigned getStride() const {
      return stride;
    }

    /*! checks padding to 16 byte check, fails hard */
    __forceinline void checkPadding16() const 
    {
       if (size()) 
         volatile int w = *((int*)getPtr(size()-1)+3); // FIXME: is failing hard avoidable?
    }

  protected:
    MemoryMonitorInterface* device; //!< device to report memory usage to 
    bool shared;     //!< set if memory is shared with application
    bool mapped;     //!< set if buffer is mapped
    bool modified;   //!< true if the buffer got modified
    unsigned stride; //!< stride of the stream in bytes
    char* ptr;       //!< pointer to buffer data
    char* ptr_ofs;   //!< base pointer plus offset
    size_t bytes;    //!< size of buffer in bytes
    size_t num;      //!< number of elements in the stream
  };

  /*! Implements a data stream inside a data buffer. */
  template<typename T>
    class BufferT : public Buffer
  {
  public:

    typedef T value_type;

    /*! access to the ith element of the buffer stream */
    __forceinline const T& operator[](size_t i) const 
    {
      assert(i<num);
#if defined(RTCORE_BUFFER_STRIDE)
      return *(T*)(ptr_ofs + i*stride);
#else
      return *(T*)(ptr_ofs + i*sizeof(T));
#endif
    }

#if defined(__MIC__)
    __forceinline char* getPtr( size_t i = 0 ) const 
    {
      assert(i<num);
#if defined(RTCORE_BUFFER_STRIDE)
      return ptr_ofs + i*stride;
#else
      return ptr_ofs + i*sizeof(T);
#endif
    }

    __forceinline unsigned int getBufferStride() const {
#if defined(RTCORE_BUFFER_STRIDE)
      return stride;
#else
      return sizeof(T);
#endif
    }
#endif
  };

  /*! Implements a data stream inside a data buffer. */
  template<>
    class BufferT<Vec3fa> : public Buffer
  {
  public:

    typedef Vec3fa value_type;

    /*! access to the ith element of the buffer stream */
    __forceinline const Vec3fa operator[](size_t i) const
    {
      assert(i<num);
#if defined(RTCORE_BUFFER_STRIDE)
#if defined(__MIC__)
      return *(Vec3fa*)(ptr_ofs + i*stride);
#else
      return Vec3fa(float4::loadu((float*)(ptr_ofs + i*stride)));
#endif
#else
      return *(Vec3fa*)(ptr_ofs + i*sizeof(Vec3fa));
#endif
    }

    __forceinline char* getPtr( size_t i = 0 ) const 
    {
      assert(i<num);
#if defined(RTCORE_BUFFER_STRIDE)
      return ptr_ofs + i*stride;
#else
      return ptr_ofs + i*sizeof(Vec3fa);
#endif
    }

#if defined(__MIC__)
    __forceinline unsigned int getBufferStride() const {
#if defined(RTCORE_BUFFER_STRIDE)
      return stride;
#else
      return sizeof(Vec3fa);
#endif
    }
#endif
  };
}
