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
#include "config.h"
#include "alloc.h"
#include "intrinsics.h"
#if defined(TASKING_TBB)
#  define __TBB_NO_IMPLICIT_LINKAGE 1
#  include "tbb/scalable_allocator.h"
#endif

////////////////////////////////////////////////////////////////////////////////
/// Windows Platform
////////////////////////////////////////////////////////////////////////////////

#ifdef _WIN32

#define WIN32_LEAN_AND_MEAN
#include <windows.h>
#include <malloc.h>

namespace embree
{
  void* os_malloc(size_t bytes, const int additional_flags) 
  {
    int flags = MEM_COMMIT|MEM_RESERVE|additional_flags;
    char* ptr = (char*) VirtualAlloc(nullptr,bytes,flags,PAGE_READWRITE);
    if (ptr == nullptr) throw std::bad_alloc();
    return ptr;
  }

  void* os_reserve(size_t bytes)
  {
    char* ptr = (char*) VirtualAlloc(nullptr,bytes,MEM_RESERVE,PAGE_READWRITE);
    if (ptr == nullptr) throw std::bad_alloc();
    return ptr;
  }

  void os_commit (void* ptr, size_t bytes) {
    VirtualAlloc(ptr,bytes,MEM_COMMIT,PAGE_READWRITE);
  }

  size_t os_shrink(void* ptr, size_t bytesNew, size_t bytesOld) 
  {
    size_t pageSize = 4096;
    bytesNew = (bytesNew+pageSize-1) & ~(pageSize-1);
    assert(bytesNew <= bytesOld);
    if (bytesNew < bytesOld)
      VirtualFree((char*)ptr+bytesNew,bytesOld-bytesNew,MEM_DECOMMIT);
    return bytesNew;
  }

  void os_free(void* ptr, size_t bytes) {
    if (bytes == 0) return;
    VirtualFree(ptr,0,MEM_RELEASE);
  }
}
#endif

////////////////////////////////////////////////////////////////////////////////
/// Unix Platform
////////////////////////////////////////////////////////////////////////////////

#if defined(__UNIX__)

#include <sys/mman.h>
#include <errno.h>
#include <stdlib.h>
#include <string.h>

#if defined(RTCORE_MEMKIND_ALLOCATOR)
#include <hbwmalloc.h>
#endif

#if defined(__MIC__)
#define USE_HUGE_PAGES 1
#else
#define USE_HUGE_PAGES 0
#endif

#define UPGRADE_TO_2M_PAGE_LIMIT (256*1024) 
#define PAGE_SIZE_2M (2*1024*1024)
#define PAGE_SIZE_4K (4*1024)

namespace embree
{

  __forceinline bool useHugePages(const size_t bytes) 
  {
#if USE_HUGE_PAGES
    /* try to use huge pages for large allocations */
    if (bytes >= PAGE_SIZE_2M)
    {
      /* multiple of page size */
      if ((bytes % PAGE_SIZE_2M) == 0) 
        return true;
      /* multiple of page size with rest >= UPGRADE_TO_2M_PAGE_LIMIT */
      else if (bytes >= UPGRADE_TO_2M_PAGE_LIMIT)
        return true;
    }
#endif
    return false;
  }

#if defined(RTCORE_MEMKIND_ALLOCATOR)
  void* mk_malloc(size_t bytes)
  {
    assert(hbw_check_available());
    void *ptr = NULL;
    if ((bytes / PAGE_SIZE_2M) >= 1 && ((bytes % PAGE_SIZE_2M) == 0))
    {
      if (hbw_posix_memalign_psize(&ptr,PAGE_SIZE_2M,bytes,HBW_PAGESIZE_2MB) == 0)
      {
#if defined(DEBUG)
        PRINT("2M PATH");
        PRINT(bytes);
        PRINT(ptr);
#endif
        return ptr;
      }
    }
    /* standard 4k allocation */
    if (hbw_posix_memalign(&ptr,PAGE_SIZE_4K,bytes) == 0)
      return ptr;

#if defined(DEBUG)
    PRINT("could not allocate hbw memory");
#endif
    return NULL;
  }

  void mk_free(void* ptr) 
  {
    hbw_free(ptr);
  }
#endif
  
  void* os_malloc(size_t bytes, const int additional_flags)
  {
    int flags = MAP_PRIVATE | MAP_ANON | additional_flags;
    
#if defined(RTCORE_MEMKIND_ALLOCATOR)
    char *memkind_ptr = (char*)mk_malloc(bytes);
    if (memkind_ptr) return memkind_ptr;
#endif
    
    if (useHugePages(bytes)) 
    {
#if USE_HUGE_PAGES
      flags |= MAP_HUGETLB;
#endif
#if defined(__MIC__)
      flags |= MAP_POPULATE;
#endif
      bytes = (bytes+PAGE_SIZE_2M-1)&ssize_t(-PAGE_SIZE_2M);
    } 
    else 
    {
      bytes = (bytes+PAGE_SIZE_4K-1)&ssize_t(-PAGE_SIZE_4K);
    }

    char* ptr = (char*) mmap(0, bytes, PROT_READ | PROT_WRITE, flags, -1, 0);

    assert( ptr != MAP_FAILED );
    if (ptr == nullptr || ptr == MAP_FAILED) throw std::bad_alloc();
    return ptr;
  }

  void* os_reserve(size_t bytes)
  {
    int flags = MAP_PRIVATE | MAP_ANON | MAP_NORESERVE;

#if defined(RTCORE_MEMKIND_ALLOCATOR)
    char *memkind_ptr = (char*)mk_malloc(bytes);
    if (memkind_ptr) return memkind_ptr;
#endif

    if (useHugePages(bytes)) 
    {
#if USE_HUGE_PAGES
      flags |= MAP_HUGETLB;
#endif
#if defined(__MIC__)
      flags |= MAP_POPULATE;
#endif
      bytes = (bytes+PAGE_SIZE_2M-1)&ssize_t(-PAGE_SIZE_2M);
    } 
    else 
    {
      bytes = (bytes+PAGE_SIZE_4K-1)&ssize_t(-PAGE_SIZE_4K);
    }

    char* ptr = (char*) mmap(0, bytes, PROT_READ | PROT_WRITE, flags, -1, 0);
    assert( ptr != MAP_FAILED );
    if (ptr == nullptr || ptr == MAP_FAILED) throw std::bad_alloc();
    return ptr;
  }

  void os_commit (void* ptr, size_t bytes) {
  }

  size_t os_shrink(void* ptr, size_t bytesNew, size_t bytesOld) 
  {
#if defined(RTCORE_MEMKIND_ALLOCATOR)
    return bytesOld;
#endif

    size_t pageSize = PAGE_SIZE_4K;
    if (useHugePages(bytesOld)) 
    {
      //pageSize = PAGE_SIZE_2M;
      /* cannot shrink a huge page to a smaller size*/
      return bytesOld;
    }

    bytesNew = (bytesNew+pageSize-1) & ~(pageSize-1);

    assert(bytesNew <= bytesOld);
    if (bytesNew < bytesOld)
      if (munmap((char*)ptr+bytesNew,bytesOld-bytesNew) == -1)
      {
        throw std::bad_alloc();
      }

    return bytesNew;
  }

  void os_free(void* ptr, size_t bytes) 
  {
    if (bytes == 0)
      return;

#if defined(RTCORE_MEMKIND_ALLOCATOR)
    mk_free(ptr);
    return;
#endif

    if (useHugePages(bytes)) {
      bytes = (bytes+PAGE_SIZE_2M-1)&ssize_t(-PAGE_SIZE_2M);
    } else {
      bytes = (bytes+PAGE_SIZE_4K-1)&ssize_t(-PAGE_SIZE_4K);
    }

    if (munmap(ptr,bytes) == -1)
    {
      throw std::bad_alloc();
    }
  }
}

#endif

////////////////////////////////////////////////////////////////////////////////
/// All Platforms
////////////////////////////////////////////////////////////////////////////////
  
namespace embree
{
  void* alignedMalloc(size_t size, size_t align) 
  {
    assert((align & (align-1)) == 0);
//#if defined(TASKING_TBB) // FIXME: have to disable this for now as the TBB allocator itself seems to access some uninitialized value when using valgrind
//    return scalable_aligned_malloc(size,align);
//#else
    return _mm_malloc(size,align);
//#endif
  }
  
  void alignedFree(void* ptr) 
  {
//#if defined(TASKING_TBB)
//    scalable_aligned_free(ptr);
//#else
    _mm_free(ptr);
//#endif
  }
}
