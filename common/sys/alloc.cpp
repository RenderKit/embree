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

#include "config.h"
#include "alloc.h"
#include "intrinsics.h"
#include "sysinfo.h"

////////////////////////////////////////////////////////////////////////////////
/// All Platforms
////////////////////////////////////////////////////////////////////////////////
  
namespace embree
{
  void* alignedMalloc(size_t size, size_t align) 
  {
    assert((align & (align-1)) == 0);
    void* ptr = _mm_malloc(size,align);

    if (size != 0 && ptr == nullptr) 
      throw std::bad_alloc();
    
    return ptr;
  }
  
  void alignedFree(void* ptr) {
    _mm_free(ptr);
  }

  static bool tryDirectHugePageAllocation = true;

  __forceinline bool isHugePageCandidate(const size_t bytes) 
  {
#if defined(__WIN32__)
    if (GetLargePageMinimum() != PAGE_SIZE_2M)
      return false;
#endif

    /* try to use huge pages for large allocations */
    if (bytes >= PAGE_SIZE_2M)
    {
      /* multiple of page size */
      if ((bytes % PAGE_SIZE_2M) == 0) 
        return true;
      else if (bytes >= 64 * PAGE_SIZE_2M) /* will only introduce a 1.5% overhead */
        return true;
    }
    return false;
  }
}

////////////////////////////////////////////////////////////////////////////////
/// Windows Platform
////////////////////////////////////////////////////////////////////////////////

#ifdef _WIN32

#define WIN32_LEAN_AND_MEAN
#include <windows.h>
#include <malloc.h>

namespace embree
{
  bool win_enable_hugepages(bool verbose) 
  {
    HANDLE hToken;
    if (!OpenProcessToken(GetCurrentProcess(), TOKEN_QUERY | TOKEN_ADJUST_PRIVILEGES, &hToken)) {
      if (verbose) std::cout << "OpenProcessToken failed while trying to enable SeLockMemoryPrivilege: " << GetLastError() << std::endl;
      return false;
    }

    TOKEN_PRIVILEGES tp;
    tp.PrivilegeCount = 1;
    tp.Privileges[0].Attributes = SE_PRIVILEGE_ENABLED;

    if (!LookupPrivilegeValueW(nullptr, L"SeLockMemoryPrivilege", &tp.Privileges[0].Luid)) {
      if (verbose) std::cout << "LookupPrivilegeValue failed while trying to enable SeLockMemoryPrivilege: " << GetLastError() << std::endl;
      return false;
    }
    
    SetLastError(ERROR_SUCCESS);
    if (!AdjustTokenPrivileges(hToken, FALSE, &tp, sizeof(tp), nullptr, 0)) {
      if (verbose) std::cout << "AdjustTokenPrivileges failed while trying to enable SeLockMemoryPrivilege" << std::endl;
      return false;
    }
    
    if (GetLastError() == ERROR_NOT_ALL_ASSIGNED) {
      tryDirectHugePageAllocation = false;
      if (verbose) std::cout << "AdjustTokenPrivileges failed to enable SeLockMemoryPrivilege: Add SeLockMemoryPrivilege for current user and run process in elevated mode (Run as administrator)." << std::endl;
      return false;
    } 

    tryDirectHugePageAllocation = true;
    return true;
  }

  void* os_malloc(size_t bytes, bool* huge_pages)
  {
    if (bytes == 0) {
      if (huge_pages) *huge_pages = false;
      return nullptr;
    }

    if (isHugePageCandidate(bytes)) 
    {
      /* try direct huge page allocation first */
      if (tryDirectHugePageAllocation)
      {
        int flags = MEM_COMMIT | MEM_RESERVE | MEM_LARGE_PAGES;
        char* ptr = (char*) VirtualAlloc(nullptr,bytes,flags,PAGE_READWRITE);
        if (ptr != nullptr) {
          if (huge_pages) *huge_pages = true;
          return ptr;
        }

        /* direct huge page allocation failed, disable it for the future */
        tryDirectHugePageAllocation = false;     
      }
    } 

    /* fall back to 4k pages */
    int flags = MEM_COMMIT | MEM_RESERVE;
    char* ptr = (char*) VirtualAlloc(nullptr,bytes,flags,PAGE_READWRITE);
    if (ptr == nullptr) throw std::bad_alloc();
    if (huge_pages) *huge_pages = false;
    return ptr;
  }

  size_t os_shrink(void* ptr, size_t bytesNew, size_t bytesOld) 
  {
    /* first try with 4KB pages */
    bytesNew = (bytesNew+PAGE_SIZE_4K-1) & ~(PAGE_SIZE_4K-1);
    if (bytesNew >= bytesOld)
      return bytesOld;

    if (VirtualFree((char*)ptr+bytesNew,bytesOld-bytesNew,MEM_DECOMMIT))
      return bytesNew;

#if 0 // decommitting huge pages seems not to work under Windows

    /* now try with 2MB pages */
    bytesNew = (bytesNew+PAGE_SIZE_2M-1) & ~(PAGE_SIZE_2M-1);
    if (bytesNew >= bytesOld)
      return bytesOld;

    if (VirtualFree((char*)ptr+bytesNew,bytesOld-bytesNew,MEM_DECOMMIT))
      return bytesNew;
    throw std::bad_alloc();

#else
    return bytesOld;
#endif
  }

  void os_free(void* ptr, size_t bytes) 
  {
    if (bytes == 0) return;
    if (!VirtualFree(ptr,0,MEM_RELEASE))
      /*throw std::bad_alloc()*/ return;  // we on purpose do not throw an exception when an error occurs, to avoid throwing an exception during error handling
  }

  void os_advise(void *ptr, size_t bytes)
  {
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

#if defined(__MACOSX__)
#include <mach/vm_statistics.h>
#endif

namespace embree
{
  void* os_malloc(size_t bytes, bool* huge_pages)
  { 
    if (bytes == 0) {
      if (huge_pages) *huge_pages = false;
      return nullptr;
    }

    if (isHugePageCandidate(bytes)) 
    {
#if !defined(__MACOSX__)
      
      /* try direct huge page allocation first */
      if (tryDirectHugePageAllocation)
      {
#if defined(__MACOSX__)
        void* ptr = mmap(0, bytes, PROT_READ | PROT_WRITE, MAP_PRIVATE | MAP_ANON, VM_FLAGS_SUPERPAGE_SIZE_2MB, 0);
        if (ptr != MAP_FAILED) {
          if (huge_pages) *huge_pages = true;
          return ptr;
        }
#else
        void* ptr = mmap(0, bytes, PROT_READ | PROT_WRITE, MAP_PRIVATE | MAP_ANON | MAP_HUGETLB, -1, 0);
        if (ptr != MAP_FAILED) {
          if (huge_pages) *huge_pages = true;
          return ptr;
        }
#endif
        /* direct huge page allocation failed, disable it for the future */
        tryDirectHugePageAllocation = false;     
      }
#endif
    } 

    /* standard mmap call */
    void* ptr = (char*) mmap(0, bytes, PROT_READ | PROT_WRITE, MAP_PRIVATE | MAP_ANON, -1, 0);
    if (ptr == MAP_FAILED) throw std::bad_alloc();
    if (huge_pages) *huge_pages = false;

    /* advise huge page hint for THP */
    os_advise(ptr,bytes);
    return ptr;
  }

  size_t os_shrink(void* ptr, size_t bytesNew, size_t bytesOld) 
  {
    /* first try with 4KB pages */
    bytesNew = (bytesNew+PAGE_SIZE_4K-1) & ~(PAGE_SIZE_4K-1);
    if (bytesNew >= bytesOld)
      return bytesOld;

    if (munmap((char*)ptr+bytesNew,bytesOld-bytesNew) != -1)
      return bytesNew;

    /* now try with 2MB pages */
    bytesNew = (bytesNew+PAGE_SIZE_2M-1) & ~(PAGE_SIZE_2M-1);
    if (bytesNew >= bytesOld)
      return bytesOld;

    if (munmap((char*)ptr+bytesNew,bytesOld-bytesNew) != -1)
      return bytesNew;

    throw std::bad_alloc();
  }

  void os_free(void* ptr, size_t bytes) 
  {
    if (bytes == 0)
      return;

    if (munmap(ptr,bytes) == -1)
      /*throw std::bad_alloc()*/ return;  // we on purpose do not throw an exception when an error occurs, to avoid throwing an exception during error handling
  }

  /* hint for transparent huge pages (THP) */
  void os_advise(void* pptr, size_t bytes)
  {
#if defined(MADV_HUGEPAGE)
    madvise(pptr,bytes,MADV_HUGEPAGE); 
#endif
  }
}

#endif
