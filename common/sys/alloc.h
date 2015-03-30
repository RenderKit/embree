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

#include "platform.h"

namespace embree
{
#define ALIGN_PTR(ptr,alignment) \
  ((((size_t)ptr)+alignment-1)&((size_t)-(ssize_t)alignment))

#define ALIGNED_STRUCT                                           \
  void* operator new(size_t size) { return alignedMalloc(size); }       \
  void operator delete(void* ptr) { alignedFree(ptr); }      \
  void* operator new[](size_t size) { return alignedMalloc(size); }  \
  void operator delete[](void* ptr) { alignedFree(ptr); }    \

#define ALIGNED_STRUCT_(align)                                           \
  void* operator new(size_t size) { return alignedMalloc(size,align); } \
  void operator delete(void* ptr) { alignedFree(ptr); }                 \
  void* operator new[](size_t size) { return alignedMalloc(size,align); } \
  void operator delete[](void* ptr) { alignedFree(ptr); }               \

#define ALIGNED_CLASS                                                \
  public:                                                            \
    ALIGNED_STRUCT                                                  \
  private:

#define ALIGNED_CLASS_(align)                                           \
 public:                                                               \
    ALIGNED_STRUCT_(align)                                              \
 private:
  
  /*! aligned allocation */
  void* alignedMalloc(size_t size, size_t align = 64);
  void alignedFree(const void* ptr);

  /*! allocates pages directly from OS */
  void* os_malloc (size_t bytes);
  void* os_reserve(size_t bytes);
  void  os_commit (void* ptr, size_t bytes);
  void  os_shrink (void* ptr, size_t bytesNew, size_t bytesOld);
  void  os_free   (void* ptr, size_t bytes);
  void* os_realloc(void* ptr, size_t bytesNew, size_t bytesOld);
}

