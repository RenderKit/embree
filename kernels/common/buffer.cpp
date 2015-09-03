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

#include "buffer.h"

namespace embree
{
  Buffer::Buffer () 
    : device(nullptr), ptr(nullptr), bytes(0), ptr_ofs(nullptr), stride(0), num(0), shared(false), mapped(false), modified(true) {}
  
  Buffer::Buffer (MemoryMonitorInterface* device_in, size_t num_in, size_t stride_in) 
    : device(nullptr), ptr(nullptr), bytes(0), ptr_ofs(nullptr), stride(0), num(0), shared(false), mapped(false), modified(true)
	//  : Buffer() // FIXME: not supported by VS2010
  {
    init(device_in,num_in,stride_in);
  }
  
  Buffer::~Buffer () {
    free();
  }

  void Buffer::init(MemoryMonitorInterface* device_in, size_t num_in, size_t stride_in) 
  {
    device = device_in;
    ptr = nullptr;
    bytes = num_in*stride_in;
    ptr_ofs = nullptr;
    num = num_in;
    stride = stride_in;
    shared = false;
    mapped = false;
    modified = true;
  }

  void Buffer::set(void* ptr_in, size_t ofs_in, size_t stride_in)
  {
    /* report error if buffer is not existing */
    if (!device)
      throw_RTCError(RTC_INVALID_ARGUMENT,"invalid buffer specified");

#if !defined(RTCORE_BUFFER_STRIDE)
    if (stride_in != stride) {
      throw_RTCError(RTC_INVALID_OPERATION,"buffer stride feature disabled at compile time and specified stride does not match default stride");
      return;
    }
#endif

    ptr = (char*) ptr_in;
    bytes = 0;
    ptr_ofs = (char*) ptr_in + ofs_in;
    stride = stride_in;
    shared = true;
  }

  void Buffer::alloc() {
    if (device) device->memoryMonitor(bytes,false);
    ptr = ptr_ofs = (char*) alignedMalloc(bytes);
  }

  void Buffer::free()
  {
    if (shared || !ptr) return;
    alignedFree(ptr); 
    if (device) device->memoryMonitor(-bytes,true);
    ptr = nullptr; ptr_ofs = nullptr; bytes = 0;
  }

  void* Buffer::map(atomic_t& cntr)
  {
    /* report error if buffer is not existing */
    if (!device)
      throw_RTCError(RTC_INVALID_ARGUMENT,"invalid buffer specified");

    /* report error if buffer is already mapped */
    if (mapped)
      throw_RTCError(RTC_INVALID_OPERATION,"buffer is already mapped");

    /* allocate buffer */
    if (!ptr && !shared && bytes)
      alloc();

    /* return mapped buffer */
    atomic_add(&cntr,+1); 
    mapped = true;
    return ptr;
  }

  void Buffer::unmap(atomic_t& cntr)
  {
    /* report error if buffer not mapped */
    if (!mapped)
      throw_RTCError(RTC_INVALID_OPERATION,"buffer is not mapped");

    /* unmap buffer */
    atomic_add(&cntr,-1); 
    mapped = false;
  }
}
