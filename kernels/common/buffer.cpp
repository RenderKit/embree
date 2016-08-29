// ======================================================================== //
// Copyright 2009-2016 Intel Corporation                                    //
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
    : device(nullptr), ptr(nullptr), ptr_ofs(nullptr), bytes(0), num(0), stride(0), shared(false), mapped(false), modified(true) {}
  
  Buffer::Buffer (MemoryMonitorInterface* device_in, size_t num_in, size_t stride_in) 
    : device(nullptr), ptr(nullptr), ptr_ofs(nullptr), bytes(0), num(0), stride(0), shared(false), mapped(false), modified(true)
	//  : Buffer() // FIXME: not supported by VS2012
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
    if (device) device->memoryMonitor(-ssize_t(bytes),true);
    ptr = nullptr; ptr_ofs = nullptr; bytes = 0;
  }

  void* Buffer::map(std::atomic<size_t>& cntr)
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
    cntr++;
    mapped = true;
    return ptr;
  }

  void Buffer::unmap(std::atomic<size_t>& cntr)
  {
    /* report error if buffer not mapped */
    if (!mapped)
      throw_RTCError(RTC_INVALID_OPERATION,"buffer is not mapped");

    /* unmap buffer */
    cntr--;
    mapped = false;
  }
}
