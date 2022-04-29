// Copyright 2009-2021 Intel Corporation
// SPDX-License-Identifier: Apache-2.0

#include "alloc.h"
#include "intrinsics.h"
#include "sysinfo.h"
#include "mutex.h"

#define ALLOCATION_WORKAROUND 1
#define USM_SHARED_MEMORY_DEVICE_READONLY_HINTS 0

#if USM_SHARED_MEMORY_DEVICE_READONLY_HINTS
#  include <level_zero/ze_api.h>
#endif

////////////////////////////////////////////////////////////////////////////////
/// All Platforms
////////////////////////////////////////////////////////////////////////////////
  
namespace embree
{
  size_t total_allocations = 0;

#if defined(EMBREE_DPCPP_SUPPORT)
  
  __thread sycl::context* tls_context_tutorial = nullptr;
  __thread sycl::device* tls_device_tutorial = nullptr;
  __thread sycl::queue* tls_queue_tutorial = nullptr;
  
  
  __thread sycl::context* tls_context_embree = nullptr;
  __thread sycl::device* tls_device_embree = nullptr;
  __thread sycl::queue* tls_queue_embree = nullptr;
  
  void enableUSMAllocEmbree(sycl::context* context, sycl::device* device, sycl::queue* queue)
  {
    if (tls_context_embree != nullptr) throw std::runtime_error("USM allocation already enabled");
    if (tls_device_embree != nullptr) throw std::runtime_error("USM allocation already enabled");
    tls_context_embree = context;
    tls_device_embree = device;
    tls_queue_embree = queue;
  }

  void disableUSMAllocEmbree()
  {
    if (tls_context_embree  == nullptr) throw std::runtime_error("USM allocation not enabled");
    if (tls_device_embree  == nullptr) throw std::runtime_error("USM allocation not enabled");
    tls_context_embree = nullptr;
    tls_device_embree = nullptr;
    tls_queue_embree = nullptr;    
  }

  void enableUSMAllocTutorial(sycl::context* context, sycl::device* device, sycl::queue* queue)
  {
    //if (tls_context_tutorial != nullptr) throw std::runtime_error("USM allocation already enabled");
    //if (tls_device_tutorial != nullptr) throw std::runtime_error("USM allocation already enabled");
    tls_context_tutorial = context;
    tls_device_tutorial = device;
    tls_queue_tutorial = queue;    
  }

  void disableUSMAllocTutorial()
  {
    if (tls_context_tutorial  == nullptr) throw std::runtime_error("USM allocation not enabled");
    if (tls_device_tutorial  == nullptr) throw std::runtime_error("USM allocation not enabled");
    if (tls_queue_tutorial  == nullptr) throw std::runtime_error("USM allocation not enabled");
    
    tls_context_tutorial = nullptr;
    tls_device_tutorial = nullptr;
    tls_queue_tutorial = nullptr;        
  }

#endif
  
  void* alignedMalloc(size_t size, size_t align)
  {
    if (size == 0)
      return nullptr;

    assert((align & (align-1)) == 0);
    void* ptr = _mm_malloc(size,align);
    if (size != 0 && ptr == nullptr)
      throw std::bad_alloc();
    return ptr;
  }

  void alignedFree(void* ptr)
  {
    if (ptr)
      _mm_free(ptr);
  }

#if defined(EMBREE_DPCPP_SUPPORT)
  
  void makeUSMDeviceWriteable(void* ptr, size_t size)
  {
#if USM_SHARED_MEMORY_DEVICE_READONLY_HINTS == 1
    sycl::queue* queue = nullptr;
    if (tls_context_tutorial) queue = tls_queue_tutorial;
    if (tls_context_embree  ) queue = tls_queue_embree;
    if (queue) {
      //pi_mem_advice advise_readonly = static_cast<pi_mem_advice>(ZE_MEMORY_ADVICE_SET_READ_MOSTLY);
      //pi_mem_advice advise_device = static_cast<pi_mem_advice>(ZE_MEMORY_ADVICE_SET_PREFERRED_LOCATION);
      pi_mem_advice advise_readonly_off = static_cast<pi_mem_advice>(ZE_MEMORY_ADVICE_CLEAR_READ_MOSTLY);
      pi_mem_advice advise_device_off = static_cast<pi_mem_advice>(ZE_MEMORY_ADVICE_CLEAR_PREFERRED_LOCATION);
      
      assert(ptr);
      queue->mem_advise(ptr, size, advise_readonly_off);
      queue->mem_advise(ptr, size, advise_device_off);
    }
#endif    
  }

  void* alignedSYCLMalloc(sycl::context* context, sycl::device* device, sycl::queue* queue, size_t size, size_t align, sycl::usm::alloc alloc_mode)
  {
    assert(context);
    assert(device);
    assert(queue);
    
    if (size == 0)
      return nullptr;

    assert((align & (align-1)) == 0);
    total_allocations++;    

    void* ptr = sycl::aligned_alloc(align,size,*device,*context,alloc_mode);
    if (size != 0 && ptr == nullptr)
      throw std::bad_alloc();

#if USM_SHARED_MEMORY_DEVICE_READONLY_HINTS == 1
    if (mode == sycl::usm::alloc::shared)
    {
      /* default: shared USM is read-only on the device to avoid back-migration from device to host if host reads the data again */
      pi_mem_advice advise_readonly = static_cast<pi_mem_advice>(ZE_MEMORY_ADVICE_SET_READ_MOSTLY);
      pi_mem_advice advise_device = static_cast<pi_mem_advice>(ZE_MEMORY_ADVICE_SET_PREFERRED_LOCATION);
      queue->mem_advise(ptr, size, advise_readonly);
      queue->mem_advise(ptr, size, advise_device);
    }
#endif    
    return ptr;
  }
  
  struct AllocationBlock
  {
    AllocationBlock(size_t total_bytes)
      : total_bytes(total_bytes) {}
    
    AllocationBlock(sycl::context* context, sycl::device* device, sycl::queue* queue, size_t total_bytes, sycl::usm::alloc mode)
      : cur(0), total_bytes(total_bytes), base(alignedSYCLMalloc(context,device,queue,total_bytes,4096, mode)) {} // USM blocks are aligned to 2MB boundaries anyway

    void* alloc(size_t size, size_t align)
    {
      assert((align & (align-1)) == 0);
      assert(align <= 128);

      align = std::max(size_t(16),align); // FIXME: for some reason that is required!

      if (base == nullptr)
        return nullptr;

      cur = (cur+align-1)&(-align);
      if (cur+size > total_bytes) return nullptr;

      void* ptr = (char*)base+cur;
      cur += size;
      return ptr;
    }
    
    size_t cur = 0;
    size_t total_bytes = 0;
    void* base = nullptr;
    
//  } g_block[3] = { 256*1024, 256*1024, 128*1024*1024 };
  } g_block[3] = { 2*1024*1024, 2*1024*1024, 8*1024*1024 };

  static MutexSys g_alloc_mutex;
  //static SYCLMallocMode g_alloc_mode = SYCLMallocMode::DEFAULT;
#if ALLOCATION_WORKAROUND
  static SYCLMallocMode g_alloc_mode = SYCLMallocMode::DEFAULT;
#else  
  static SYCLMallocMode g_alloc_mode = SYCLMallocMode::SYCL_MALLOC;
#endif
  
  SYCLMallocMode setSYCLMallocMode(SYCLMallocMode mode)
  {
    SYCLMallocMode prev_mode = g_alloc_mode;
    g_alloc_mode = mode;
    return prev_mode;
  }
  
  void* alignedSYCLMallocWorkaround(sycl::context* context, sycl::device* device, sycl::queue* queue, size_t size, size_t align, sycl::usm::alloc mode)
  {
#if ALLOCATION_WORKAROUND
    Lock<MutexSys> lock(g_alloc_mutex);

#define REGULAR_ALLOC_THRESHOLD 512*1024    
    if (g_alloc_mode == SYCLMallocMode::SYCL_MALLOC || size > REGULAR_ALLOC_THRESHOLD)
      return alignedSYCLMalloc(context,device,queue,size,align,mode);


    int slot = (int) g_alloc_mode;
    assert(slot < 3);
    
    void* ptr = g_block[slot].alloc(size,align);
    if (ptr) return ptr;
    g_block[slot] = AllocationBlock(context,device,queue,g_block[slot].total_bytes, mode);
    ptr = g_block[slot].alloc(size,align);
    if (ptr) return ptr;
    throw std::runtime_error("allocation failure");
    return nullptr;
#else
    return alignedSYCLMalloc(context,device,queue,size,align,mode);
#endif
  }

  void* alignedSYCLMalloc(size_t size, size_t align, sycl::usm::alloc mode)
  {
    if (tls_context_tutorial) return alignedSYCLMallocWorkaround(tls_context_tutorial, tls_device_tutorial, tls_queue_tutorial, size, align, mode);
    if (tls_context_embree  ) return alignedSYCLMallocWorkaround(tls_context_embree,   tls_device_embree,   tls_queue_embree, size, align, mode);
    return nullptr;
  }

  void alignedSYCLFree(sycl::context* context, void* ptr)
  {
#if !ALLOCATION_WORKAROUND
    assert(context);
    if (ptr) {
      sycl::free(ptr,*context);
    }
#endif
  }

  void alignedSYCLFree(void* ptr)
  {
    if (tls_context_tutorial) return alignedSYCLFree(tls_context_tutorial, ptr);
    if (tls_context_embree  ) return alignedSYCLFree(tls_context_embree, ptr);
  }

#endif

  void* alignedUSMMalloc(size_t size, size_t align) //, sycl::usm::alloc mode)
  {
#if defined(EMBREE_DPCPP_SUPPORT)
    if (tls_context_embree || tls_context_tutorial)
      return alignedSYCLMalloc(size,align, sycl::usm::alloc::shared); //mode);
    else
#endif
      return alignedMalloc(size,align);
  }

  void alignedUSMFree(void* ptr)
  {
#if defined(EMBREE_DPCPP_SUPPORT)
    if (tls_context_embree || tls_context_tutorial)
      return alignedSYCLFree(ptr);
    else
#endif
      return alignedFree(ptr);
  }

  static bool huge_pages_enabled = false;
  static MutexSys os_init_mutex;

  __forceinline bool isHugePageCandidate(const size_t bytes)
  {
    if (!huge_pages_enabled)
      return false;

    /* use huge pages only when memory overhead is low */
    const size_t hbytes = (bytes+PAGE_SIZE_2M-1) & ~size_t(PAGE_SIZE_2M-1);
    return 66*(hbytes-bytes) < bytes; // at most 1.5% overhead
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
  bool win_enable_selockmemoryprivilege (bool verbose)
  {
    HANDLE hToken;
    if (!OpenProcessToken(GetCurrentProcess(), TOKEN_QUERY | TOKEN_ADJUST_PRIVILEGES, &hToken)) {
      if (verbose) std::cout << "WARNING: OpenProcessToken failed while trying to enable SeLockMemoryPrivilege: " << GetLastError() << std::endl;
      return false;
    }

    TOKEN_PRIVILEGES tp;
    tp.PrivilegeCount = 1;
    tp.Privileges[0].Attributes = SE_PRIVILEGE_ENABLED;

    if (!LookupPrivilegeValueW(nullptr, L"SeLockMemoryPrivilege", &tp.Privileges[0].Luid)) {
      if (verbose) std::cout << "WARNING: LookupPrivilegeValue failed while trying to enable SeLockMemoryPrivilege: " << GetLastError() << std::endl;
      return false;
    }
    
    SetLastError(ERROR_SUCCESS);
    if (!AdjustTokenPrivileges(hToken, FALSE, &tp, sizeof(tp), nullptr, 0)) {
      if (verbose) std::cout << "WARNING: AdjustTokenPrivileges failed while trying to enable SeLockMemoryPrivilege" << std::endl;
      return false;
    }
    
    if (GetLastError() == ERROR_NOT_ALL_ASSIGNED) {
      if (verbose) std::cout << "WARNING: AdjustTokenPrivileges failed to enable SeLockMemoryPrivilege: Add SeLockMemoryPrivilege for current user and run process in elevated mode (Run as administrator)." << std::endl;
      return false;
    } 

    return true;
  }

  bool os_init(bool hugepages, bool verbose) 
  {
    Lock<MutexSys> lock(os_init_mutex);

    if (!hugepages) {
      huge_pages_enabled = false;
      return true;
    }

    if (GetLargePageMinimum() != PAGE_SIZE_2M) {
      huge_pages_enabled = false;
      return false;
    }

    huge_pages_enabled = true;
    return true;
  }

  void* os_malloc(size_t bytes, bool& hugepages)
  {
    if (bytes == 0) {
      hugepages = false;
      return nullptr;
    }

    /* try direct huge page allocation first */
    if (isHugePageCandidate(bytes)) 
    {
      int flags = MEM_COMMIT | MEM_RESERVE | MEM_LARGE_PAGES;
      char* ptr = (char*) VirtualAlloc(nullptr,bytes,flags,PAGE_READWRITE);
      if (ptr != nullptr) {
        hugepages = true;
        return ptr;
      }
    } 

    /* fall back to 4k pages */
    int flags = MEM_COMMIT | MEM_RESERVE;
    char* ptr = (char*) VirtualAlloc(nullptr,bytes,flags,PAGE_READWRITE);
    if (ptr == nullptr) throw std::bad_alloc();
    hugepages = false;
    return ptr;
  }

  size_t os_shrink(void* ptr, size_t bytesNew, size_t bytesOld, bool hugepages) 
  {
    if (hugepages) // decommitting huge pages seems not to work under Windows
      return bytesOld;

    const size_t pageSize = hugepages ? PAGE_SIZE_2M : PAGE_SIZE_4K;
    bytesNew = (bytesNew+pageSize-1) & ~(pageSize-1);
    bytesOld = (bytesOld+pageSize-1) & ~(pageSize-1);
    if (bytesNew >= bytesOld)
      return bytesOld;

    if (!VirtualFree((char*)ptr+bytesNew,bytesOld-bytesNew,MEM_DECOMMIT))
      throw std::bad_alloc();

    return bytesNew;
  }

  void os_free(void* ptr, size_t bytes, bool hugepages) 
  {
    if (bytes == 0) 
      return;

    if (!VirtualFree(ptr,0,MEM_RELEASE))
      throw std::bad_alloc();
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
#include <sstream>

#if defined(__MACOSX__)
#include <mach/vm_statistics.h>
#endif

namespace embree
{
  bool os_init(bool hugepages, bool verbose) 
  {
    Lock<MutexSys> lock(os_init_mutex);

    if (!hugepages) {
      huge_pages_enabled = false;
      return true;
    }

#if defined(__LINUX__)

    int hugepagesize = 0;

    std::ifstream file; 
    file.open("/proc/meminfo",std::ios::in);
    if (!file.is_open()) {
      if (verbose) std::cout << "WARNING: Could not open /proc/meminfo. Huge page support cannot get enabled!" << std::endl;
      huge_pages_enabled = false;
      return false;
    }
    
    std::string line;
    while (getline(file,line))
    {
      std::stringstream sline(line);
      while (!sline.eof() && sline.peek() == ' ') sline.ignore();
      std::string tag; getline(sline,tag,' ');
      while (!sline.eof() && sline.peek() == ' ') sline.ignore();
      std::string val; getline(sline,val,' ');
      while (!sline.eof() && sline.peek() == ' ') sline.ignore();
      std::string unit; getline(sline,unit,' ');
      if (tag == "Hugepagesize:" && unit == "kB") {
	hugepagesize = std::stoi(val)*1024;
	break;
      }
    }
    
    if (hugepagesize != PAGE_SIZE_2M) 
    {
      if (verbose) std::cout << "WARNING: Only 2MB huge pages supported. Huge page support cannot get enabled!" << std::endl;
      huge_pages_enabled = false;
      return false;
    }
#endif

    huge_pages_enabled = true;
    return true;
  }

  void* os_malloc(size_t bytes, bool& hugepages)
  { 
    if (bytes == 0) {
      hugepages = false;
      return nullptr;
    }

    /* try direct huge page allocation first */
    if (isHugePageCandidate(bytes)) 
    {
#if defined(__MACOSX__)
      void* ptr = mmap(0, bytes, PROT_READ | PROT_WRITE, MAP_PRIVATE | MAP_ANON, VM_FLAGS_SUPERPAGE_SIZE_2MB, 0);
      if (ptr != MAP_FAILED) {
        hugepages = true;
        return ptr;
      }
#elif defined(MAP_HUGETLB)
      void* ptr = mmap(0, bytes, PROT_READ | PROT_WRITE, MAP_PRIVATE | MAP_ANON | MAP_HUGETLB, -1, 0);
      if (ptr != MAP_FAILED) {
        hugepages = true;
        return ptr;
      }
#endif
    } 

    /* fallback to 4k pages */
    void* ptr = (char*) mmap(0, bytes, PROT_READ | PROT_WRITE, MAP_PRIVATE | MAP_ANON, -1, 0);
    if (ptr == MAP_FAILED) throw std::bad_alloc();
    hugepages = false;

    /* advise huge page hint for THP */
    os_advise(ptr,bytes);
    return ptr;
  }

  size_t os_shrink(void* ptr, size_t bytesNew, size_t bytesOld, bool hugepages) 
  {
    const size_t pageSize = hugepages ? PAGE_SIZE_2M : PAGE_SIZE_4K;
    bytesNew = (bytesNew+pageSize-1) & ~(pageSize-1);
    bytesOld = (bytesOld+pageSize-1) & ~(pageSize-1);
    if (bytesNew >= bytesOld)
      return bytesOld;

    if (munmap((char*)ptr+bytesNew,bytesOld-bytesNew) == -1)
      throw std::bad_alloc();

    return bytesNew;
  }

  void os_free(void* ptr, size_t bytes, bool hugepages) 
  {
    if (bytes == 0)
      return;

    /* for hugepages we need to also align the size */
    const size_t pageSize = hugepages ? PAGE_SIZE_2M : PAGE_SIZE_4K;
    bytes = (bytes+pageSize-1) & ~(pageSize-1);
    if (munmap(ptr,bytes) == -1)
      throw std::bad_alloc();
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
