// ======================================================================== //
// Copyright 2009-2013 Intel Corporation                                    //
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

#ifndef __EMBREE_BUFFER_H__
#define __EMBREE_BUFFER_H__

#include "common/default.h"

namespace embree
{
  /*! Implements a data buffer. */
  class Buffer
  {
  public:

    /*! Buffer construction */
    Buffer (); 
    
    /*! Buffer destruction */
    ~Buffer ();
      
  public:
    
    /*! initialized the buffer */
    void init(size_t bytes);

    /*! fees the buffer */
    void free();

    /*! maps the buffer */
    void* map(atomic_t& cntr);
    
    /*! unmaps the buffer */
    void unmap(atomic_t& cntr);

    /*! sets shared buffer */
    void set(void* ptr);

    /*! allocated buffer */
    void alloc();

    /*! checks if the buffer is mapped */
    __forceinline bool isMapped() const {
      return mapped; 
    }

  protected:
    char* ptr;       //!< pointer to buffer data
    size_t bytes;    //!< size of buffer in bytes
    bool shared;     //!< set if memory is shared with application
    bool mapped;     //!< set if buffer is mapped
  };

  /*! Implements a data stream inside a data buffer. */
  template<typename T>
    class BufferStream : public Buffer
  {
  public:
    
    /*! construction of buffer stream */
    BufferStream ()
      : num(0), ofs(0), stride(0) {}

    /*! initialized the buffer */
    __forceinline void init(size_t num_in, size_t stride_in = sizeof(T))
    {
      num = num_in;
      stride = stride_in;
      Buffer::init(num*stride);
    }

    /*! access to the ith element of the buffer stream */
    __forceinline T& operator[](size_t i) 
    {
      assert(i<num);
#if defined(__RTCORE_BUFFER_STRIDE__)
      return *(T*)(ptr + ofs + i*stride);
#else
      return *(T*)(ptr + i*sizeof(T));
#endif
    }

    /*! access to the ith element of the buffer stream */
    __forceinline const T& operator[](size_t i) const 
    {
      assert(i<num);
#if defined(__RTCORE_BUFFER_STRIDE__)
      return *(const T*)(ptr + ofs + i*stride);
#else
      return *(const T*)(ptr + i*sizeof(T));
#endif
    }

    /*! sets shared buffer */
    __forceinline void set(void* ptr, size_t ofs_in, size_t stride_in) 
    {
      Buffer::set(ptr);
      ofs = ofs_in;
      stride = stride_in;

#if !defined(__RTCORE_BUFFER_STRIDE__)
      if (ofs != 0 || stride != sizeof(T)) {
        recordError(RTC_INVALID_OPERATION);
      }
#endif
    }

  private:
    size_t num;        //!< number of elements in the stream
    size_t ofs;        //!< start offset of the stream in bytes
    size_t stride;     //!< stride of the stream in bytes
  };
}

#endif
