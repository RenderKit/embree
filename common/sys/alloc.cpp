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
#  define TBB_IMPLEMENT_CPP0X 0
#  define __TBB_NO_IMPLICIT_LINKAGE 1
#  define __TBBMALLOC_NO_IMPLICIT_LINKAGE 1
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

/* hint for transparent huge pages (THP) */
#if defined(__MACOSX__)
#define USE_MADVISE 0
#else
#define USE_MADVISE 1
#endif

#define UPGRADE_TO_2M_PAGE_LIMIT (256*1024) 
#define PAGE_SIZE_2M (2*1024*1024)
#define PAGE_SIZE_4K (4*1024)

namespace embree
{

  __forceinline bool isHugePageCandidate(const size_t bytes) 
  {
#if defined(__MIC__)
    return true;
#endif
    /* try to use huge pages for large allocations */
    if (bytes >= PAGE_SIZE_2M)
    {
      /* multiple of page size */
      if ((bytes % PAGE_SIZE_2M) == 0) 
        return true;
      else if (bytes >= 64 * PAGE_SIZE_2M) /* will only introduce a 3% overhead */
        return true;
    }
    return false;
  }

// ============================================
// ============================================
// ============================================

  bool tryDirectHugePageAllocation = true;

#if USE_MADVISE
  void os_madvise(void *ptr, size_t bytes)
  {
#ifdef MADV_HUGEPAGE
    int res = madvise(ptr,bytes,MADV_HUGEPAGE); 
#if defined(DEBUG)
    if (res)
      perror("madvise failed: ");
#endif
#endif
  }
#endif
  
  void* os_malloc(size_t bytes, const int additional_flags)
  {
    int flags = MAP_PRIVATE | MAP_ANON | additional_flags;
        
    if (isHugePageCandidate(bytes)) 
    {
#if defined(__MIC__)
      flags |= MAP_POPULATE;
#endif
      bytes = (bytes+PAGE_SIZE_2M-1)&ssize_t(-PAGE_SIZE_2M);

#if !defined(__MACOSX__)
      /* try direct huge page allocation first */
      if (tryDirectHugePageAllocation)
      {
        const int hugePageFlags = MAP_ANONYMOUS | MAP_PRIVATE | MAP_HUGETLB;
        void* ptr = (char*) mmap(0, bytes, PROT_READ | PROT_WRITE, hugePageFlags, -1, 0);
        
        if (ptr == nullptr || ptr == MAP_FAILED)
        {
          /* direct huge page allocation failed, disable it for the future */
          tryDirectHugePageAllocation = false;     
        }
        else
          return ptr;
      }
#endif
    } 
    else
      bytes = (bytes+PAGE_SIZE_4K-1)&ssize_t(-PAGE_SIZE_4K);

    /* standard mmap call */
    void* ptr = (char*) mmap(0, bytes, PROT_READ | PROT_WRITE, flags, -1, 0);
    assert( ptr != MAP_FAILED );
    if (ptr == nullptr || ptr == MAP_FAILED) throw std::bad_alloc();

#if USE_MADVISE
    /* advise huge page hint for THP */
    os_madvise(ptr,bytes);
#endif

    return ptr;
  }

  void* os_reserve(size_t bytes)
  {
    int flags = MAP_PRIVATE | MAP_ANON | MAP_NORESERVE;

    if (isHugePageCandidate(bytes)) 
    {
#if defined(__MIC__)
      flags |= MAP_POPULATE;
#endif
      bytes = (bytes+PAGE_SIZE_2M-1)&ssize_t(-PAGE_SIZE_2M);

#if !defined(__MACOSX__)
      /* try direct huge page allocation first */
      if (tryDirectHugePageAllocation)
      {
        const int hugePageFlags = MAP_ANONYMOUS | MAP_PRIVATE | MAP_HUGETLB;
        void* ptr = (char*) mmap(0, bytes, PROT_READ | PROT_WRITE, hugePageFlags, -1, 0);
      
        if (ptr == nullptr || ptr == MAP_FAILED)
        {
          /* direct huge page allocation failed, disable it for the future */
          tryDirectHugePageAllocation = false;     
        }
        else
          return ptr;          
      }
#endif
    } 
    else
      bytes = (bytes+PAGE_SIZE_4K-1)&ssize_t(-PAGE_SIZE_4K);

    /* standard mmap call */
    char* ptr = (char*) mmap(0, bytes, PROT_READ | PROT_WRITE, flags, -1, 0);
    assert( ptr != MAP_FAILED );
    if (ptr == nullptr || ptr == MAP_FAILED) throw std::bad_alloc();

#if USE_MADVISE
    /* advise huge page hint for THP */
    os_madvise(ptr,bytes);
#endif

    return ptr;
  }

  void os_commit (void* ptr, size_t bytes) {
  }

  size_t os_shrink(void* ptr, size_t bytesNew, size_t bytesOld) 
  {
#if USE_MADVISE
    return bytesOld;
#endif

    size_t pageSize = PAGE_SIZE_4K;
    if (isHugePageCandidate(bytesOld)) 
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

    if (isHugePageCandidate(bytes)) {
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

// #if USE_MADVISE
//     if (size >= 16*PAGE_SIZE_2M) 
//     {
//       align = PAGE_SIZE_2M;
//       void *ptr = _mm_malloc(size,align);
//       os_madvise(ptr,size);
//       return ptr;
//      }
// #endif

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
