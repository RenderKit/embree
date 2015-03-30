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

#include "platform.h"
#include "intrinsics.h"

////////////////////////////////////////////////////////////////////////////////
/// Windows Platform
////////////////////////////////////////////////////////////////////////////////

#ifdef _WIN32

#define WIN32_LEAN_AND_MEAN
#include <windows.h>

namespace embree
{
  void* os_malloc(size_t bytes) 
  {
    char* ptr = (char*) VirtualAlloc(NULL,bytes,MEM_COMMIT|MEM_RESERVE,PAGE_READWRITE);
    if (ptr == NULL) throw std::bad_alloc();
    return ptr;
  }

  void* os_reserve(size_t bytes)
  {
    char* ptr = (char*) VirtualAlloc(NULL,bytes,MEM_RESERVE,PAGE_READWRITE);
    if (ptr == NULL) throw std::bad_alloc();
    return ptr;
  }

  void os_commit (void* ptr, size_t bytes) {
    VirtualAlloc(ptr,bytes,MEM_COMMIT,PAGE_READWRITE);
  }

  void os_shrink(void* ptr, size_t bytesNew, size_t bytesOld) 
  {
    size_t pageSize = 4096;
    if (bytesNew & (pageSize-1)) 
      bytesNew = (bytesNew+pageSize) & (pageSize-1);

    VirtualFree((char*)ptr+bytesNew,bytesOld-bytesNew,MEM_DECOMMIT);
  }

  void os_free(void* ptr, size_t bytes) {
    if (bytes == 0) return;
    VirtualFree(ptr,0,MEM_RELEASE);
  }

  void* os_realloc (void* ptr, size_t bytesNew, size_t bytesOld) {
    NOT_IMPLEMENTED;
  }
}
#endif

////////////////////////////////////////////////////////////////////////////////
/// Unix Platform
////////////////////////////////////////////////////////////////////////////////

#if defined(__UNIX__)

#include <sys/mman.h>
#include <errno.h>
#include <string.h>

namespace embree
{
  void* os_malloc(size_t bytes)
  {
    int flags = MAP_PRIVATE | MAP_ANON;
#if defined(__MIC__)
    if (bytes > 16*4096) {
      flags |= MAP_HUGETLB | MAP_POPULATE;
      bytes = (bytes+2*1024*1024-1)&ssize_t(-2*1024*1024);
    } else {
      bytes = (bytes+4095)&ssize_t(-4096);
    }
#endif
    char* ptr = (char*) mmap(0, bytes, PROT_READ | PROT_WRITE, flags, -1, 0);
    if (ptr == NULL || ptr == MAP_FAILED) throw std::bad_alloc();
    return ptr;
  }

  void* os_reserve(size_t bytes)
  {
    int flags = MAP_PRIVATE | MAP_ANON | MAP_NORESERVE;
#if defined(__MIC__)
    if (bytes > 16*4096) {
      flags |= MAP_HUGETLB;
      bytes = (bytes+2*1024*1024-1)&ssize_t(-2*1024*1024);
    } else {
      bytes = (bytes+4095)&ssize_t(-4096);
    }
#endif
    char* ptr = (char*) mmap(0, bytes, PROT_READ | PROT_WRITE, flags, -1, 0);
    if (ptr == NULL || ptr == MAP_FAILED) throw std::bad_alloc();
    return ptr;
  }

  void os_commit (void* ptr, size_t bytes) {
  }

  void os_shrink(void* ptr, size_t bytesNew, size_t bytesOld) 
  {
    size_t pageSize = 4096;
#if defined(__MIC__)
    if (bytesOld > 16*4096) pageSize = 2*1024*1024;
#endif
    if (bytesNew & (pageSize-1)) 
      bytesNew = (bytesNew+pageSize) & (pageSize-1);

    os_free((char*)ptr+bytesNew,bytesOld-bytesNew);
  }

  void os_free(void* ptr, size_t bytes) 
  {
    if (bytes == 0)
      return;

#if defined(__MIC__)
    if (bytes > 16*4096) {
      bytes = (bytes+2*1024*1024-1)&ssize_t(-2*1024*1024);
    } else {
      bytes = (bytes+4095)&ssize_t(-4096);
    }
#endif
    if (munmap(ptr,bytes) == -1) {
      throw std::bad_alloc();
    }
  }

  void* os_realloc (void* old_ptr, size_t bytesNew, size_t bytesOld)
  {
#if defined(__MIC__)
    if (bytesOld > 16*4096)
      bytesOld = (bytesOld+2*1024*1024-1)&ssize_t(-2*1024*1024);
    else
      bytesOld = (bytesOld+4095)&ssize_t(-4096);

    if (bytesNew > 16*4096)
      bytesNew = (bytesNew+2*1024*1024-1)&ssize_t(-2*1024*1024);
    else
      bytesNew = (bytesNew+4095)&ssize_t(-4096);

    char *ptr = (char*)mremap(old_ptr,bytesOld,bytesNew,MREMAP_MAYMOVE);

    if (ptr == NULL || ptr == MAP_FAILED) {
      perror("os_realloc ");
      throw std::bad_alloc();
    }
    return ptr;
#else
    NOT_IMPLEMENTED;
    return NULL;
#endif

  }
}

#endif

////////////////////////////////////////////////////////////////////////////////
/// All Platforms
////////////////////////////////////////////////////////////////////////////////

#include <stdlib.h>
#include <string.h>

namespace embree
{
  void* alignedMalloc(size_t size, size_t align)
  {
    if (size == 0) return NULL;
    char* base = (char*)malloc(size+align+sizeof(int));
    if (base == NULL) throw std::bad_alloc();

    char* unaligned = base + sizeof(int);
    char*   aligned = unaligned + align - ((size_t)unaligned & (align-1));
    ((int*)aligned)[-1] = (int)((size_t)aligned-(size_t)base);
    return aligned;
  }
  
  void alignedFree(const void* ptr) {
    if (ptr == NULL) return;
    int ofs = ((int*)ptr)[-1];
    free((char*)ptr-ofs);
  }
}
