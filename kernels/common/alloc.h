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

#pragma once

#include "default.h"
#include "device.h"
#include "scene.h"
#include "primref.h"

namespace embree
{
  class FastAllocator 
  {
    /*! maximal supported alignment */
    static const size_t maxAlignment = 64;

    /*! maximal allocation size */

    /* default settings */
    //static const size_t defaultBlockSize = 4096;
#define maxAllocationSize size_t(4*1024*1024-maxAlignment)
    static const size_t MAX_THREAD_USED_BLOCK_SLOTS = 8;
    
  public:

    enum AllocationType { ALIGNED_MALLOC, OS_MALLOC, SHARED, ANY_TYPE };

    /*! Per thread structure holding the current memory block. */
    struct __aligned(64) ThreadLocal 
    {
      ALIGNED_CLASS_(64);
    public:

      /*! Constructor for usage with ThreadLocalData */
      __forceinline ThreadLocal (FastAllocator* alloc = nullptr) 
	: alloc(alloc), ptr(nullptr), cur(0), end(0), allocBlockSize(0), bytesUsed(0), bytesWasted(0) 
      {
        if (alloc) allocBlockSize = alloc->defaultBlockSize;
      }

      /*! bind to fast allocator */
      void bind(FastAllocator* alloc_i) 
      {
        assert(alloc_i);
        if (alloc == alloc_i) return;
        Lock<SpinLock> lock(mutex);
        //if (alloc == alloc_i) return; // not required as only one thread calls bind
        if (alloc) {
          alloc->bytesUsed += getUsedBytes();
          alloc->bytesWasted += getWastedBytes();
        }
        ptr = nullptr;
	cur = end = 0;
	bytesWasted = 0;
        bytesUsed = 0;
        alloc = alloc_i;
        allocBlockSize = alloc->defaultBlockSize;
        alloc->join(this);
      }

      /*! unbind to fast allocator */
      void unbind(FastAllocator* alloc_i) 
      {
        assert(alloc_i);
        if (alloc != alloc_i) return;
        Lock<SpinLock> lock(mutex);
        if (alloc != alloc_i) return; // required as a different thread calls unbind
        alloc->bytesUsed += getUsedBytes();
        alloc->bytesWasted += getWastedBytes();
        ptr = nullptr;
	cur = end = 0;
	bytesWasted = 0;
        bytesUsed = 0;
        alloc = nullptr;
        allocBlockSize = 0;
      }
      
      /* Allocate aligned memory from the threads memory block. */
      __forceinline void* malloc(FastAllocator* alloc_i, size_t bytes, size_t align = 16) 
      {
        bind(alloc_i);

        assert(align <= maxAlignment);
	bytesUsed += bytes;
	
        /* try to allocate in local block */
	size_t ofs = (align - cur) & (align-1); 
        cur += bytes + ofs;
        if (likely(cur <= end)) { bytesWasted += ofs; return &ptr[cur - bytes]; }
	cur -= bytes + ofs;
        
        /* if allocation is too large allocate with parent allocator */
        if (4*bytes > allocBlockSize) {
          return alloc->malloc(bytes,maxAlignment,false);
	}

        /* get new partial block if allocation failed */
        size_t blockSize = allocBlockSize;
        ptr = (char*) alloc->malloc(blockSize,maxAlignment,true);
 	bytesWasted += end-cur;
	cur = 0; end = blockSize;
	
        /* retry allocation */
	ofs = (align - cur) & (align-1); 
        cur += bytes + ofs;
        if (likely(cur <= end)) { bytesWasted += ofs; return &ptr[cur - bytes]; }
	cur -= bytes + ofs;

        /* get new full block if allocation failed */
        blockSize = allocBlockSize;
        ptr = (char*) alloc->malloc(blockSize,maxAlignment,false);
	bytesWasted += end-cur;
	cur = 0; end = blockSize;
	
        /* retry allocation */
	ofs = (align - cur) & (align-1); 
        cur += bytes + ofs;
        if (likely(cur <= end)) { bytesWasted += ofs; return &ptr[cur - bytes]; }
	cur -= bytes + ofs;
	
        /* should never happen as large allocations get handled specially above */
        assert(false);
        return nullptr;
      }

      /*! returns amount of used bytes */
      size_t getUsedBytes() const { return bytesUsed; }
      
      /*! returns amount of wasted bytes */
      size_t getWastedBytes() const { return bytesWasted + (end-cur); }

    public:
      FastAllocator* alloc;  //!< parent allocator
      char*  ptr;            //!< pointer to memory block
      size_t cur;            //!< current location of the allocator
      size_t end;            //!< end of the memory block
      size_t allocBlockSize; //!< block size for allocations
    private:
      SpinLock mutex;        //!< required as unbind is called from other threads
      size_t bytesUsed;      //!< number of total bytes allocated
      size_t bytesWasted;    //!< number of bytes wasted
    };

    /*! Two thread local structures. */
    struct __aligned(64) ThreadLocal2
    {
      ALIGNED_CLASS_(64);
    public:
      ThreadLocal alloc0;
      ThreadLocal alloc1;
    };

    FastAllocator (Device* device, bool osAllocation) 
      : device(device), slotMask(0), usedBlocks(nullptr), freeBlocks(nullptr), use_single_mode(false), defaultBlockSize(PAGE_SIZE), 
        growSize(PAGE_SIZE), log2_grow_size_scale(0), bytesUsed(0), bytesWasted(0), atype(osAllocation ? OS_MALLOC : ALIGNED_MALLOC),
        primrefarray(device)
    {
      for (size_t i=0; i<MAX_THREAD_USED_BLOCK_SLOTS; i++)
      {
        threadUsedBlocks[i] = nullptr;
        threadBlocks[i] = nullptr;
        assert(!slotMutex[i].isLocked());
      }
    } 

    ~FastAllocator () { 
      clear();
    }

    /*! returns the device attached to this allocator */
    Device* getDevice() {
      return device;
    }

    void share(mvector<PrimRef>& primrefarray_i) {
      primrefarray = std::move(primrefarray_i);
    }

    void unshare(mvector<PrimRef>& primrefarray_o) {
      primrefarray_o = std::move(primrefarray);
    }

    /*! returns first fast thread local allocator */
    __forceinline ThreadLocal* _threadLocal() {
      return &threadLocal2()->alloc0;
    }

  private:

    /*! returns both fast thread local allocators */
    __forceinline ThreadLocal2* threadLocal2() 
    {
      ThreadLocal2* alloc = thread_local_allocator2;
      if (alloc == nullptr) 
        thread_local_allocator2 = alloc = new ThreadLocal2;
      return alloc;
    }
  public:

    __forceinline void join(ThreadLocal* alloc)
    {
      Lock<SpinLock> lock(thread_local_allocators_lock);
      thread_local_allocators.push_back(alloc);
    }

  public:

    struct CachedAllocator
    {
      __forceinline CachedAllocator(void* ptr)
        : alloc(nullptr), talloc0(nullptr), talloc1(nullptr) 
      {
        assert(ptr == nullptr);
      }

      __forceinline CachedAllocator(FastAllocator* alloc, ThreadLocal2* talloc)
        : alloc(alloc), talloc0(&talloc->alloc0), talloc1(alloc->use_single_mode ? &talloc->alloc0 : &talloc->alloc1) {}

      __forceinline operator bool () const {
        return alloc != nullptr;
      }

      __forceinline void* operator() (size_t bytes, size_t align = 16) const {
        return talloc0->malloc(alloc,bytes,align);
      }

      __forceinline void* malloc0 (size_t bytes, size_t align = 16) const {
        return talloc0->malloc(alloc,bytes,align);
      }

      __forceinline void* malloc1 (size_t bytes, size_t align = 16) const {
        return talloc1->malloc(alloc,bytes,align);
      }

    public:
      FastAllocator* alloc;
      ThreadLocal* talloc0;
      ThreadLocal* talloc1;
    };

    __forceinline CachedAllocator getCachedAllocator() {
      return CachedAllocator(this,threadLocal2());
    }

    /*! Builder interface to create thread local allocator */
    struct Create
    {
    public:
      __forceinline Create (FastAllocator* allocator) : allocator(allocator) {}
      __forceinline CachedAllocator operator() () const { return allocator->getCachedAllocator();  }

    private:
      FastAllocator* allocator;
    };

    /*! initializes the grow size */
    __forceinline void initGrowSizeAndNumSlots(size_t bytesAllocate, bool compact) 
    {
      bytesAllocate  = ((bytesAllocate +PAGE_SIZE-1) & ~(PAGE_SIZE-1)); // always consume full pages
      
      growSize = clamp(bytesAllocate,size_t(PAGE_SIZE),maxAllocationSize); // PAGE_SIZE -maxAlignment ?
      log2_grow_size_scale = 0;
      slotMask = 0x0;
      if (!compact) {
        if (MAX_THREAD_USED_BLOCK_SLOTS >= 2 && bytesAllocate >  4*maxAllocationSize) slotMask = 0x1;
        if (MAX_THREAD_USED_BLOCK_SLOTS >= 4 && bytesAllocate >  8*maxAllocationSize) slotMask = 0x3;
        if (MAX_THREAD_USED_BLOCK_SLOTS >= 8 && bytesAllocate > 16*maxAllocationSize) slotMask = 0x7;
      }
    }

    void internal_fix_used_blocks()
    {
      /* move thread local blocks to global block list */
      for (size_t i = 0; i < MAX_THREAD_USED_BLOCK_SLOTS; i++)
      {
        while (threadBlocks[i].load() != nullptr) {
          Block* nextUsedBlock = threadBlocks[i].load()->next;
          threadBlocks[i].load()->next = usedBlocks.load();
          usedBlocks = threadBlocks[i].load();
          threadBlocks[i] = nextUsedBlock;
        }
        threadBlocks[i] = nullptr;
      }
    }

    /*! initializes the allocator */
    void init(size_t bytesAllocate, size_t bytesReserve = 0) 
    {     
      internal_fix_used_blocks();
      /* distribute the allocation to multiple thread block slots */
      slotMask = MAX_THREAD_USED_BLOCK_SLOTS-1;      
      if (usedBlocks.load() || freeBlocks.load()) { reset(); return; }
      if (bytesReserve == 0) bytesReserve = bytesAllocate;
      freeBlocks = Block::create(device,bytesAllocate,bytesReserve,nullptr,atype);
      defaultBlockSize = clamp(bytesAllocate/4,size_t(128),size_t(PAGE_SIZE+maxAlignment)); 
      initGrowSizeAndNumSlots(bytesAllocate,false);
    }

    /*! initializes the allocator */
    void init_estimate(size_t bytesAllocate, const bool single_mode = false, const bool compact = false) 
    {      
      internal_fix_used_blocks();
      if (usedBlocks.load() || freeBlocks.load()) { reset(); return; }
      /* single allocator mode ? */
      use_single_mode = single_mode; 
      defaultBlockSize = clamp(bytesAllocate/4,size_t(128),size_t(PAGE_SIZE+maxAlignment)); 
      initGrowSizeAndNumSlots(bytesAllocate,compact);
    }

    /*! frees state not required after build */
    __forceinline void cleanup() 
    {
      internal_fix_used_blocks();

      /* unbind all thread local allocators */
      for (auto alloc : thread_local_allocators) alloc->unbind(this);
      thread_local_allocators.clear();
    }

    /*! shrinks all memory blocks to the actually used size */
    void shrink () 
    {
      for (size_t i=0; i<MAX_THREAD_USED_BLOCK_SLOTS; i++)
        if (threadUsedBlocks[i].load() != nullptr) threadUsedBlocks[i].load()->shrink_list(device);
      if (usedBlocks.load() != nullptr) usedBlocks.load()->shrink_list(device);
      if (freeBlocks.load() != nullptr) freeBlocks.load()->clear_list(device); freeBlocks = nullptr;
    }

    /*! resets the allocator, memory blocks get reused */
    void reset () 
    {
      internal_fix_used_blocks();

      bytesUsed.store(0);
      bytesWasted.store(0);
      
      /* reset all used blocks and move them to begin of free block list */
      while (usedBlocks.load() != nullptr) {
        usedBlocks.load()->reset_block();
        Block* nextUsedBlock = usedBlocks.load()->next;
        usedBlocks.load()->next = freeBlocks.load();
        freeBlocks = usedBlocks.load();
        usedBlocks = nextUsedBlock;
      }

      /* remove all shared blocks as they are re-added during build */
      freeBlocks.store(Block::remove_shared_blocks(freeBlocks.load()));

      for (size_t i=0; i<MAX_THREAD_USED_BLOCK_SLOTS; i++) 
      {
        threadUsedBlocks[i] = nullptr;
        threadBlocks[i] = nullptr;
      }
      
      /* unbind all thread local allocators */
      for (auto alloc : thread_local_allocators) alloc->unbind(this);
      thread_local_allocators.clear();
    }

    /*! frees all allocated memory */
    __forceinline void clear()
    {
      cleanup();
      bytesUsed.store(0);
      bytesWasted.store(0);
      if (usedBlocks.load() != nullptr) usedBlocks.load()->clear_list(device); usedBlocks = nullptr;
      if (freeBlocks.load() != nullptr) freeBlocks.load()->clear_list(device); freeBlocks = nullptr;
      for (size_t i=0; i<MAX_THREAD_USED_BLOCK_SLOTS; i++) {
        threadUsedBlocks[i] = nullptr;
        threadBlocks[i] = nullptr;
      }
      primrefarray.clear();
    }

    __forceinline size_t incGrowSizeScale()
    {
      size_t scale = log2_grow_size_scale.fetch_add(1)+1;
      return size_t(1) << min(size_t(16),scale);
    }

    /*! thread safe allocation of memory */
    void* malloc(size_t& bytes, size_t align, bool partial) 
    {
      assert(align <= maxAlignment);

      while (true) 
      {
        /* allocate using current block */
        size_t threadIndex = TaskScheduler::threadIndex();
        size_t slot = threadIndex & slotMask;
	Block* myUsedBlocks = threadUsedBlocks[slot];
        if (myUsedBlocks) {
          void* ptr = myUsedBlocks->malloc(device,bytes,align,partial); 
          if (ptr) return ptr;
        }
        
        /* throw error if allocation is too large */
        if (bytes > maxAllocationSize)
          throw_RTCError(RTC_UNKNOWN_ERROR,"allocation is too large");

        /* parallel block creation in case of no freeBlocks, avoids single global mutex */
        if (likely(freeBlocks.load() == nullptr)) 
        {
          Lock<SpinLock> lock(slotMutex[slot]);
          if (myUsedBlocks == threadUsedBlocks[slot]) {
            const size_t allocSize = min(max(growSize,bytes),size_t(maxAllocationSize));
            assert(allocSize >= bytes);
            threadBlocks[slot] = threadUsedBlocks[slot] = Block::create(device,allocSize,allocSize,threadBlocks[slot],atype);
          }
          continue;
        }        

        /* if this fails allocate new block */
        {
          Lock<SpinLock> lock(mutex);
	  if (myUsedBlocks == threadUsedBlocks[slot])
	  {
            if (freeBlocks.load() != nullptr) {
	      Block* nextFreeBlock = freeBlocks.load()->next;
	      freeBlocks.load()->next = usedBlocks;
	      __memory_barrier();
	      usedBlocks = freeBlocks.load();
              threadUsedBlocks[slot] = freeBlocks.load();
	      freeBlocks = nextFreeBlock;
	    } else {
	      //growSize = min(2*growSize,size_t(maxAllocationSize+maxAlignment));
              const size_t allocSize = min(growSize * incGrowSizeScale(),size_t(maxAllocationSize+maxAlignment))-maxAlignment;
	      usedBlocks = threadUsedBlocks[slot] = Block::create(device,allocSize,allocSize,usedBlocks,atype);
	    }
	  }
        }
      }
    }

    /*! add new block */
    void addBlock(void* ptr, ssize_t bytes)
    {
      Lock<SpinLock> lock(mutex);
      const size_t sizeof_Header = offsetof(Block,data[0]);
      void* aptr = (void*) ((((size_t)ptr)+maxAlignment-1) & ~(maxAlignment-1));
      size_t ofs = (size_t) aptr - (size_t) ptr;
      bytes -= ofs;
      if (bytes < 4096) return; // ignore empty or very small blocks
      freeBlocks = new (aptr) Block(SHARED,bytes-sizeof_Header,bytes-sizeof_Header,freeBlocks,ofs);
    }

    /* special allocation only used from morton builder only a single time for each build */
    void* specialAlloc(size_t bytes) 
    {
      assert(freeBlocks.load() != nullptr && freeBlocks.load()->getBlockAllocatedBytes() >= bytes);
      return freeBlocks.load()->ptr();
    }

    struct Statistics
    {
      Statistics () 
      : bytesAllocated(0), bytesReserved(0), bytesFree(0) {}

      std::string str(size_t numPrimitives) 
      {
        std::stringstream str;
        str.setf(std::ios::fixed, std::ios::floatfield);
        str << "allocated = " << std::setw(7) << std::setprecision(3) << 1E-6f*bytesAllocated << " MB, "
            << "reserved = " << std::setw(7) << std::setprecision(3) << 1E-6f*bytesReserved << " MB, "
            << "free = " << std::setw(7) << std::setprecision(3) << 1E-6f*bytesFree << "(" << std::setw(6) << std::setprecision(2) << 100.0f*bytesFree/bytesAllocated << "%), "
            << "#bytes/prim = " << std::setw(6) << std::setprecision(2) << double(bytesAllocated+bytesFree)/double(numPrimitives);
        return str.str();
      }
    
      size_t bytesAllocated;
      size_t bytesReserved;
      size_t bytesFree;
    };

    Statistics getStatistics(AllocationType atype, bool huge_pages = false) const 
    {
      Statistics stat;
      if (freeBlocks.load()) stat.bytesAllocated += freeBlocks.load()->getTotalAllocatedBytes(atype,huge_pages);
      if (usedBlocks.load()) stat.bytesAllocated += usedBlocks.load()->getTotalAllocatedBytes(atype,huge_pages);
      if (freeBlocks.load()) stat.bytesReserved += freeBlocks.load()->getTotalReservedBytes(atype,huge_pages);
      if (usedBlocks.load()) stat.bytesReserved += usedBlocks.load()->getTotalReservedBytes(atype,huge_pages);
      if (freeBlocks.load()) stat.bytesFree += freeBlocks.load()->getTotalAllocatedBytes(atype,huge_pages);
      if (usedBlocks.load()) stat.bytesFree += usedBlocks.load()->getFreeBytes(atype,huge_pages);
      return stat;
    }

    size_t getUsedBytes() {
      return bytesUsed;
    }

    size_t getWastedBytes() {
      return bytesWasted;
    }

    void print_statistics(bool verbose, size_t numPrimitives)
    {
      std::cout << "  slotMask = " << slotMask << ", use_single_mode = " << use_single_mode << ", defaultBlockSize = " << defaultBlockSize << std::endl;
      size_t bytesUsed = getUsedBytes();
      size_t bytesWasted = getWastedBytes();
      Statistics stat_all = getStatistics(ANY_TYPE);
      Statistics stat_malloc = getStatistics(ALIGNED_MALLOC);
      Statistics stat_4K = getStatistics(OS_MALLOC,false);
      Statistics stat_2M = getStatistics(OS_MALLOC,true);
      Statistics stat_shared = getStatistics(SHARED);
      std::cout << "  total : " << stat_all.str(numPrimitives);
      printf(", used = %3.3f MB (%3.2f%%), wasted = %3.3f MB (%3.2f%%)\n",
	     1E-6f*bytesUsed, 100.0f*bytesUsed/stat_all.bytesAllocated,
	     1E-6f*bytesWasted, 100.0f*bytesWasted/stat_all.bytesAllocated);
      std::cout << "  4K    : " << stat_4K.str(numPrimitives) << std::endl;
      std::cout << "  2M    : " << stat_2M.str(numPrimitives) << std::endl;
      std::cout << "  malloc: " << stat_malloc.str(numPrimitives) << std::endl;
      std::cout << "  shared: " << stat_shared.str(numPrimitives) << std::endl;

      if (verbose) 
      {
        std::cout << "  used blocks = ";
        if (usedBlocks.load() != nullptr) usedBlocks.load()->print_list();
        std::cout << "[END]" << std::endl;
        
        std::cout << "  free blocks = ";
        if (freeBlocks.load() != nullptr) freeBlocks.load()->print_list();
        std::cout << "[END]" << std::endl;
      }
    }

  private:

    struct Block 
    {
      static Block* create(MemoryMonitorInterface* device, size_t bytesAllocate, size_t bytesReserve, Block* next, AllocationType atype)
      {
        const size_t sizeof_Header = offsetof(Block,data[0]);
        bytesAllocate = ((sizeof_Header+bytesAllocate+PAGE_SIZE-1) & ~(PAGE_SIZE-1)); // always consume full pages
        bytesReserve  = ((sizeof_Header+bytesReserve +PAGE_SIZE-1) & ~(PAGE_SIZE-1)); // always consume full pages
       
        /* either use alignedMalloc or os_malloc */
        void *ptr = nullptr;
        if (atype == ALIGNED_MALLOC) 
        {
          /* special handling for default block size */
          if (bytesAllocate == (2*PAGE_SIZE_2M))
          {
            /* full 2M alignment for very first block using os_malloc */
            if (next == NULL) {
              if (device) device->memoryMonitor(bytesAllocate,false);
              bool huge_pages; ptr = os_malloc(bytesReserve,huge_pages);
              return new (ptr) Block(OS_MALLOC,bytesAllocate-sizeof_Header,bytesReserve-sizeof_Header,next,0,huge_pages);
            }
            
            const size_t alignment = maxAlignment;
            if (device) device->memoryMonitor(bytesAllocate+alignment,false);
            ptr = alignedMalloc(bytesAllocate,alignment);           

            /* give hint to transparently convert these pages to 2MB pages */
            const size_t ptr_aligned_begin = ((size_t)ptr) & ~size_t(PAGE_SIZE_2M-1);
            os_advise((void*)(ptr_aligned_begin +              0),PAGE_SIZE_2M); // may fail if no memory mapped before block
            os_advise((void*)(ptr_aligned_begin + 1*PAGE_SIZE_2M),PAGE_SIZE_2M);
            os_advise((void*)(ptr_aligned_begin + 2*PAGE_SIZE_2M),PAGE_SIZE_2M); // may fail if no memory mapped after block

            return new (ptr) Block(ALIGNED_MALLOC,bytesAllocate-sizeof_Header,bytesAllocate-sizeof_Header,next,alignment);
          }
          else 
          {
            const size_t alignment = maxAlignment;
            if (device) device->memoryMonitor(bytesAllocate+alignment,false);
            ptr = alignedMalloc(bytesAllocate,alignment);
            return new (ptr) Block(ALIGNED_MALLOC,bytesAllocate-sizeof_Header,bytesAllocate-sizeof_Header,next,alignment);
          }
        } 
        else if (atype == OS_MALLOC)
        {
          if (device) device->memoryMonitor(bytesAllocate,false);
          bool huge_pages; ptr = os_malloc(bytesReserve,huge_pages);
          return new (ptr) Block(OS_MALLOC,bytesAllocate-sizeof_Header,bytesReserve-sizeof_Header,next,0,huge_pages);
        }
        else
          assert(false);
        return NULL;
      }

      Block (AllocationType atype, size_t bytesAllocate, size_t bytesReserve, Block* next, size_t wasted, bool huge_pages = false) 
      : cur(0), allocEnd(bytesAllocate), reserveEnd(bytesReserve), next(next), wasted(wasted), atype(atype), huge_pages(huge_pages)
      {
        assert((((size_t)&data[0]) & (maxAlignment-1)) == 0);
        //for (size_t i=0; i<allocEnd; i+=defaultBlockSize) data[i] = 0;
      }

      static Block* remove_shared_blocks(Block* head)
      {
        Block** prev_next = &head;
        for (Block* block = head; block; block = block->next) {
          if (block->atype == SHARED) *prev_next = block->next;
          else                         prev_next = &block->next;
        }
        return head;
      }

      void clear_list(MemoryMonitorInterface* device) 
      {
        Block* block = this;
        while (block) {
          Block* next = block->next;
          block->clear_block(device);
          block = next;
        }
      }

      void clear_block (MemoryMonitorInterface* device) 
      {
        const size_t sizeof_Header = offsetof(Block,data[0]);
        const ssize_t sizeof_Alloced = wasted+sizeof_Header+getBlockAllocatedBytes();

        if (atype == ALIGNED_MALLOC) {
          alignedFree(this);
          if (device) device->memoryMonitor(-sizeof_Alloced,true);
        } 

        else if (atype == OS_MALLOC) {
         size_t sizeof_This = sizeof_Header+reserveEnd;
         os_free(this,sizeof_This,huge_pages);
         if (device) device->memoryMonitor(-sizeof_Alloced,true);
        } 

        else /* if (atype == SHARED) */ {
        }
      }
      
      void* malloc(MemoryMonitorInterface* device, size_t& bytes_in, size_t align, bool partial) 
      {
        size_t bytes = bytes_in;
        assert(align <= maxAlignment);
        bytes = (bytes+(align-1)) & ~(align-1);
	if (unlikely(cur+bytes > reserveEnd && !partial)) return nullptr;
	const size_t i = cur.fetch_add(bytes);
	if (unlikely(i+bytes > reserveEnd && !partial)) return nullptr;
        if (unlikely(i > reserveEnd)) return nullptr;
        bytes_in = bytes = min(bytes,reserveEnd-i);
        
	if (i+bytes > allocEnd) {
          if (device) device->memoryMonitor(i+bytes-max(i,allocEnd),true);
        }
	return &data[i];
      }
      
      void* ptr() {
        return &data[cur];
      }

      void reset_block () 
      {
        allocEnd = max(allocEnd,(size_t)cur);
        cur = 0;
      }

      void shrink_list (MemoryMonitorInterface* device) 
      {
        for (Block* block = this; block; block = block->next)
          block->shrink_block(device);
      }
   
      void shrink_block (MemoryMonitorInterface* device) 
      {
        if (atype == OS_MALLOC)
        {
          const size_t sizeof_Header = offsetof(Block,data[0]);
          size_t newSize = os_shrink(this,sizeof_Header+getBlockUsedBytes(),reserveEnd+sizeof_Header,huge_pages);
          if (device) device->memoryMonitor(newSize-sizeof_Header-allocEnd,true);
          reserveEnd = allocEnd = newSize-sizeof_Header;
        }
      }

      size_t getBlockUsedBytes() const {
        return min(size_t(cur),reserveEnd);
      }

      size_t getBlockAllocatedBytes() const {
        return min(max(allocEnd,size_t(cur)),reserveEnd);
      }

      size_t getBlockTotalAllocatedBytes() const {
        const size_t sizeof_Header = offsetof(Block,data[0]);
        return min(max(allocEnd,size_t(cur)),reserveEnd) + sizeof_Header + wasted;
      }

      size_t getBlockTotalReservedBytes() const {
        const size_t sizeof_Header = offsetof(Block,data[0]);
        return reserveEnd + sizeof_Header + wasted;
      }

      size_t getBlockFreeBytes() const {
	return max(allocEnd,size_t(cur))-cur;
      }

      bool hasType(AllocationType atype_i, bool huge_pages_i) const
      {
        if      (atype_i == ANY_TYPE ) return true;
        else if (atype   == OS_MALLOC) return atype_i == atype && huge_pages_i == huge_pages;
        else                           return atype_i == atype;
      }

      size_t getUsedBytes(AllocationType atype, bool huge_pages = false) const {
        size_t bytes = 0;
        for (const Block* block = this; block; block = block->next) {
          if (!block->hasType(atype,huge_pages)) continue;
          bytes += block->getBlockUsedBytes();
        }
        return bytes;
      }

      size_t getTotalAllocatedBytes(AllocationType atype, bool huge_pages = false) const {
        size_t bytes = 0;
        for (const Block* block = this; block; block = block->next) {
          if (!block->hasType(atype,huge_pages)) continue;
          bytes += block->getBlockTotalAllocatedBytes();
        }
        return bytes;
      }

      size_t getTotalReservedBytes(AllocationType atype, bool huge_pages = false) const {
        size_t bytes = 0;
        for (const Block* block = this; block; block = block->next){
          if (!block->hasType(atype,huge_pages)) continue;
          bytes += block->getBlockTotalReservedBytes();
        }
        return bytes;
      }

      size_t getFreeBytes(AllocationType atype, bool huge_pages = false) const {
        size_t bytes = 0;
        for (const Block* block = this; block; block = block->next) {
          if (!block->hasType(atype,huge_pages)) continue;
          bytes += block->getBlockFreeBytes();
        }
        return bytes;
      }

      void print_list () 
      {
        for (const Block* block = this; block; block = block->next)
          block->print_block();
      }

      void print_block() const 
      {
        if (atype == ALIGNED_MALLOC) std::cout << "A";
        else if (atype == OS_MALLOC) std::cout << "O";
        else if (atype == SHARED) std::cout << "S";
        if (huge_pages) std::cout << "H";
        std::cout << "[" << getBlockUsedBytes() << ", " << getBlockTotalAllocatedBytes() << ", " << getBlockTotalReservedBytes() << "] ";
      }

    public:
      std::atomic<size_t> cur;        //!< current location of the allocator
      std::atomic<size_t> allocEnd;   //!< end of the allocated memory region
      std::atomic<size_t> reserveEnd; //!< end of the reserved memory region
      Block* next;               //!< pointer to next block in list
      size_t wasted;             //!< amount of memory wasted through block alignment
      AllocationType atype;      //!< allocation mode of the block
      bool huge_pages;           //!< whether the block uses huge pages
      char align[maxAlignment-5*sizeof(size_t)-sizeof(AllocationType)-sizeof(bool)]; //!< align data to maxAlignment
      char data[1];              //!< here starts memory to use for allocations
    };

  private:
    Device* device;
    SpinLock mutex;
    size_t slotMask;
    std::atomic<Block*> threadUsedBlocks[MAX_THREAD_USED_BLOCK_SLOTS];
    std::atomic<Block*> usedBlocks;
    std::atomic<Block*> freeBlocks;

    std::atomic<Block*> threadBlocks[MAX_THREAD_USED_BLOCK_SLOTS];
    SpinLock slotMutex[MAX_THREAD_USED_BLOCK_SLOTS];
    
    bool use_single_mode;
    size_t defaultBlockSize;
    size_t growSize;
    std::atomic<size_t> log2_grow_size_scale; //!< log2 of scaling factor for grow size
    std::atomic<size_t> bytesUsed;            //!< number of total bytes used
    std::atomic<size_t> bytesWasted;          //!< number of total wasted bytes
    static __thread ThreadLocal2* thread_local_allocator2;
    SpinLock thread_local_allocators_lock;
    std::vector<ThreadLocal*> thread_local_allocators;
    AllocationType atype;
    mvector<PrimRef> primrefarray;     //!< primrefarray used to allocate nodes
  };
}
