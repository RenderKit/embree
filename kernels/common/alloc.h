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

#include "default.h"

namespace embree
{
  class FastAllocator 
  {
    /*! maximal supported alignment */
    static const size_t maxAlignment = 64;

    /*! maximal allocation size */
    //static const size_t maxAllocationSize = 2*1024*1024-maxAlignment;
    static const size_t maxAllocationSize = 4*1024*1024-maxAlignment;

  public:

    /*! Per thread structure holding the current memory block. */
    struct __aligned(64) ThreadLocal 
    {
      ALIGNED_CLASS_(64);
    public:

      /*! Constructor for usage with ThreadLocalData */
      __forceinline ThreadLocal (void* alloc) 
	: alloc((FastAllocator*)alloc), ptr(nullptr), cur(0), end(0), allocBlockSize(4096), bytesUsed(0), bytesWasted(0) {}

      /*! Default constructor. */
      __forceinline ThreadLocal (FastAllocator* alloc, const size_t allocBlockSize = 4096) 
	: alloc(alloc), ptr(nullptr), cur(0), end(0), allocBlockSize(allocBlockSize), bytesUsed(0), bytesWasted(0)  {}

      /*! resets the allocator */
      __forceinline void reset() 
      {
	ptr = nullptr;
	cur = end = 0;
	bytesWasted = bytesUsed = 0;
      }

      /* Allocate aligned memory from the threads memory block. */
      __forceinline void* operator() (size_t bytes, size_t align = 16) {
        return malloc(bytes,align);
      }

      /* Allocate aligned memory from the threads memory block. */
      __forceinline void* malloc(size_t bytes, size_t align = 16) 
      {
        assert(align <= maxAlignment);
	bytesUsed += bytes;
	
        /* try to allocate in local block */
	size_t ofs = (align - cur) & (align-1); 
        cur += bytes + ofs;
        if (likely(cur <= end)) { bytesWasted += ofs; return &ptr[cur - bytes]; }
	cur -= bytes + ofs;

        /* if allocation is too large allocate with parent allocator */
        if (4*bytes > allocBlockSize) {
          return alloc->malloc(bytes,maxAlignment);
	}

#if 0 // FIXME: this optimization is broken

        /* get new partial block if allocation failed */
	if (alloc->usedBlocks) 
	{
	  size_t blockSize = allocBlockSize;
	  ptr = (char*) alloc->usedBlocks->malloc_some(blockSize,maxAlignment);
	  bytesWasted += end-cur;
	  cur = 0; end = blockSize;
	  
	  /* retry allocation */
	  size_t ofs = (align - cur) & (align-1); 
	  cur += bytes + ofs;
	  if (likely(cur <= end)) { bytesWasted += ofs; return &ptr[cur - bytes]; }
	  cur -= bytes + ofs;
	}
#endif

        /* get new full block if allocation failed */
        size_t blockSize = allocBlockSize;
	ptr = (char*) alloc->malloc(blockSize,maxAlignment);
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

      /* returns current address */
      __forceinline void* curPtr() {
        if (ptr == nullptr) ptr = (char*) alloc->malloc(allocBlockSize,maxAlignment);
        return &ptr[bytesUsed];
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
      size_t bytesWasted;    //!< number of bytes wasted
      size_t bytesUsed; //!< bumber of total bytes allocated
    };

    /*! Two thread local structures. */
    struct __aligned(64) ThreadLocal2
    {
      ALIGNED_STRUCT;

      /*! Constructor for usage with ThreadLocalData */
      __forceinline ThreadLocal2 (void* alloc) 
        : alloc0(alloc), alloc1(alloc) {}

      /*! Default constructor. */
      __forceinline ThreadLocal2 (FastAllocator* alloc, const size_t allocBlockSize = 4096) 
        : alloc0(alloc,allocBlockSize), alloc1(alloc,allocBlockSize) {}

      /*! resets the allocator */
      __forceinline void reset() {
        alloc0.reset();
        alloc1.reset();
      }

      /*! returns amount of used bytes */
      size_t getUsedBytes() const { return alloc0.getUsedBytes() + alloc1.getUsedBytes(); }
      
      /*! returns amount of wasted bytes */
      size_t getWastedBytes() const { return alloc0.getWastedBytes() + alloc1.getWastedBytes(); }
    
    public:  
      ThreadLocal alloc0;
      ThreadLocal alloc1;
    };

    FastAllocator (MemoryMonitorInterface* device) 
      : device(device), growSize(4096), usedBlocks(nullptr), freeBlocks(nullptr), slotMask(0),
        thread_local_allocators(this), thread_local_allocators2(this) 
    {
      for (size_t i=0; i<4; i++)
        threadUsedBlocks[i] = nullptr;
    }

    ~FastAllocator () { 
      clear();
    }

    __forceinline void clear()
    {
      cleanup();
      if (usedBlocks) usedBlocks->clear(device); usedBlocks = nullptr;
      if (freeBlocks) freeBlocks->clear(device); freeBlocks = nullptr;
      for (size_t i=0; i<4; i++) threadUsedBlocks[i] = nullptr;
    }

    /*! returns a fast thread local allocator */
    __forceinline ThreadLocal* threadLocal() {
      return thread_local_allocators.get();
    }

    /*! returns a fast thread local allocator */
    __forceinline ThreadLocal2* threadLocal2() {
      return thread_local_allocators2.get();
    }

    /*! frees state not required after build */
    __forceinline void cleanup() {
      thread_local_allocators.clear();
      thread_local_allocators2.clear();
    }

    /*! initializes the allocator */
    void init(size_t bytesAllocate, size_t bytesReserve = 0) {
      if (usedBlocks || freeBlocks) { reset(); return; }
      if (bytesReserve == 0) bytesReserve = bytesAllocate;
      usedBlocks = Block::create(device,bytesAllocate,bytesReserve);
      growSize = max(size_t(4096),bytesReserve);
    }

    /*! initializes the allocator */
    void init_estimate(size_t bytesAllocate) 
    {
      if (usedBlocks || freeBlocks) { reset(); return; }
      growSize = max(size_t(4096),bytesAllocate);
      if (bytesAllocate > 4*maxAllocationSize) slotMask = 0x1;
      if (bytesAllocate > 16*maxAllocationSize) slotMask = 0x3;
    }

    /*! resets the allocator, memory blocks get reused */
    void reset () 
    {
      /* first reset all used blocks */
      if (usedBlocks) usedBlocks->reset();

      /* move all used blocks (except last) to begin of free block list */
      while (usedBlocks && usedBlocks->next) {
        Block* nextUsedBlock = usedBlocks->next;
        usedBlocks->next = freeBlocks;
        freeBlocks = usedBlocks;
        usedBlocks = nextUsedBlock;
      }
      for (size_t i=0; i<4; i++) 
        threadUsedBlocks[i] = nullptr;
     
      /* reset all thread local allocators */
      thread_local_allocators.reset();
      thread_local_allocators2.reset();
    }

    /*! shrinks all memory blocks to the actually used size */
    void shrink () {
      if (usedBlocks) usedBlocks->shrink(device);
      if (freeBlocks) freeBlocks->clear(device); freeBlocks = nullptr;
    }

    /*! thread safe allocation of memory */
    void* malloc(size_t bytes, size_t align) 
    {
      assert(align <= maxAlignment);

      while (true) 
      {
        /* allocate using current block */
	// FIXME: MIC
#if defined(__MIC__)
        size_t threadIndex = 0;
#else
        size_t threadIndex = TaskSchedulerTBB::threadIndex();
#endif
        size_t slot = threadIndex & slotMask;
	Block* myUsedBlocks = threadUsedBlocks[slot];
        if (myUsedBlocks) {
          void* ptr = myUsedBlocks->malloc(device,bytes,align); 
          if (ptr) return ptr;
        }

        /* throw error if allocation is too large */
        if (bytes > maxAllocationSize)
          THROW_RUNTIME_ERROR("allocation is too large");

        /* if this fails allocate new block */
        {
          Lock<AtomicMutex> lock(mutex);
	  if (myUsedBlocks == threadUsedBlocks[slot])
	  {
	    if (freeBlocks) {
	      Block* nextFreeBlock = freeBlocks->next;
	      freeBlocks->next = usedBlocks;
	      __memory_barrier();
	      usedBlocks = freeBlocks;
              threadUsedBlocks[slot] = freeBlocks;
	      freeBlocks = nextFreeBlock;
	    } else {
	      growSize = min(2*growSize,size_t(maxAllocationSize+maxAlignment));
	      usedBlocks = threadUsedBlocks[slot] = Block::create(device,growSize-maxAlignment, growSize-maxAlignment, usedBlocks);
	    }
	  }
        }
      }
    }

    void* ptr() {
      return usedBlocks->ptr();
    }

    size_t getAllocatedBytes() const 
    {
      size_t bytesAllocated = 0;
      if (freeBlocks) bytesAllocated += freeBlocks->getAllocatedBytes();
      if (usedBlocks) bytesAllocated += usedBlocks->getAllocatedBytes();
      return bytesAllocated;
    }

    size_t getReservedBytes() const 
    {
      size_t bytesReserved = 0;
      if (freeBlocks) bytesReserved += freeBlocks->getReservedBytes();
      if (usedBlocks) bytesReserved += usedBlocks->getReservedBytes();
      return bytesReserved;
    }

    size_t getUsedBytes() const 
    {
      size_t bytesUsed = 0;

      for (size_t t=0; t<thread_local_allocators.threads.size(); t++)
	bytesUsed += thread_local_allocators.threads[t]->getUsedBytes();

      for (size_t t=0; t<thread_local_allocators2.threads.size(); t++)
	bytesUsed += thread_local_allocators2.threads[t]->getUsedBytes();

      return bytesUsed;
    }

    size_t getFreeBytes() const 
    {
      size_t bytesFree = 0;
      if (freeBlocks) bytesFree += freeBlocks->getAllocatedBytes();
      if (usedBlocks) bytesFree += usedBlocks->getFreeBytes();
      return bytesFree;
    }

    size_t getWastedBytes() const 
    {
      size_t bytesWasted = 0;
      if (usedBlocks) {
	Block* cur = usedBlocks;
	while ((cur = cur->next) != nullptr)
	  bytesWasted += cur->getFreeBytes();
      }

      for (size_t t=0; t<thread_local_allocators.threads.size(); t++)
	bytesWasted += thread_local_allocators.threads[t]->getWastedBytes();

      for (size_t t=0; t<thread_local_allocators2.threads.size(); t++)
	bytesWasted += thread_local_allocators2.threads[t]->getWastedBytes();
      
      return bytesWasted;
    }

    void print_statistics()
    {
      size_t bytesFree = getFreeBytes();
      size_t bytesAllocated = getAllocatedBytes();
      size_t bytesReserved = getReservedBytes();
      size_t bytesUsed = getUsedBytes();
      size_t bytesWasted = getWastedBytes();
      printf("  allocated = %3.2fMB, reserved = %3.2fMB, used = %3.2fMB (%3.2f%%), wasted = %3.2fMB (%3.2f%%), free = %3.2fMB (%3.2f%%)\n",
	     1E-6f*bytesAllocated, 1E-6f*bytesReserved,
	     1E-6f*bytesUsed, 100.0f*bytesUsed/bytesAllocated,
	     1E-6f*bytesWasted, 100.0f*bytesWasted/bytesAllocated,
	     1E-6f*bytesFree, 100.0f*bytesFree/bytesAllocated);
      
      //if (State::instance()->verbosity(3)) 
      {
        std::cout << "  used blocks = ";
        if (usedBlocks) usedBlocks->print();
        std::cout << "[END]" << std::endl;
        
        std::cout << "  free blocks = ";
        if (freeBlocks) freeBlocks->print();
        std::cout << "[END]" << std::endl;
      }
    }

  private:

    struct Block 
    {
      static Block* create(MemoryMonitorInterface* device, size_t bytesAllocate, size_t bytesReserve, Block* next = nullptr)
      {
        const size_t sizeof_Header = offsetof(Block,data[0]);
        bytesAllocate = ((sizeof_Header+bytesAllocate+4095) & ~(4095)); // always consume full pages
        bytesReserve  = ((sizeof_Header+bytesReserve +4095) & ~(4095)); // always consume full pages
        if (device) device->memoryMonitor(bytesAllocate,false);
        void* ptr = os_reserve(bytesReserve);
        os_commit(ptr,bytesAllocate);
        return new (ptr) Block(bytesAllocate-sizeof_Header,bytesReserve-sizeof_Header,next);
      }

      Block (size_t bytesAllocate, size_t bytesReserve, Block* next) 
      : cur(0), allocEnd(bytesAllocate), reserveEnd(bytesReserve), next(next) 
      {
        //for (size_t i=0; i<allocEnd; i+=4096) data[i] = 0;
      }

      void clear (MemoryMonitorInterface* device) {
	if (next) next->clear(device); next = nullptr;
        const size_t sizeof_Header = offsetof(Block,data[0]);
        size_t sizeof_This = sizeof_Header+reserveEnd;
        const size_t sizeof_Alloced = sizeof_Header+getBlockAllocatedBytes();
        os_free(this,sizeof_This);
        if (device) device->memoryMonitor(-sizeof_Alloced,true);
      }
      
      void* malloc(MemoryMonitorInterface* device, size_t bytes, size_t align = 16) 
      {
        assert(align <= maxAlignment);
        bytes = (bytes+(align-1)) & ~(align-1);
	if (unlikely(cur+bytes > reserveEnd)) return nullptr;
	const size_t i = atomic_add(&cur,bytes);
	if (unlikely(i+bytes > reserveEnd)) return nullptr;
	if (i+bytes > allocEnd) {
          if (device) device->memoryMonitor(i+bytes-max(i,allocEnd),true);
          os_commit(&data[i],bytes); // FIXME: optimize, may get called frequently
        }
	return &data[i];
      }
      
      void* malloc_some(MemoryMonitorInterface* device, size_t& bytes, size_t align = 16) 
      {
        assert(align <= maxAlignment);
        bytes = (bytes+(align-1)) & ~(align-1);
	const size_t i = atomic_add(&cur,bytes);
	if (unlikely(i+bytes > reserveEnd)) bytes = reserveEnd-i;
	if (i+bytes > allocEnd) {
          if (device) device->memoryMonitor(i+bytes-max(i,allocEnd),true);
          os_commit(&data[i],bytes); // FIXME: optimize, may get called frequently
        }
	return &data[i];
      }

      void* ptr() {
        return &data[cur];
      }

      void reset () 
      {
        allocEnd = max(allocEnd,(size_t)cur);
        cur = 0;
        if (next) next->reset();
      }

      void shrink (MemoryMonitorInterface* device) 
      {
        const size_t sizeof_Header = offsetof(Block,data[0]);
        size_t newSize = os_shrink(this,sizeof_Header+getRequiredBytes(),reserveEnd+sizeof_Header);
        if (device) device->memoryMonitor(newSize-sizeof_Header-allocEnd,true);
        reserveEnd = allocEnd = newSize-sizeof_Header;
        if (next) next->shrink(device);
      }

      size_t getRequiredBytes() const {
        return min(size_t(cur),reserveEnd);
      }

      size_t getBlockAllocatedBytes() const {
	return max(allocEnd,size_t(cur));
      }

      size_t getAllocatedBytes() const {
	return max(allocEnd,size_t(cur)) + (next ? next->getAllocatedBytes() : 0);
      }

      size_t getReservedBytes() const {
	return reserveEnd + (next ? next->getReservedBytes() : 0);
      }

      size_t getFreeBytes() const {
	return max(allocEnd,size_t(cur))-cur;
      }

      void print() const {
        std::cout << "[" << cur << ", " << max(size_t(cur),allocEnd) << ", " << reserveEnd << "] ";
        if (next) next->print();
      }

    public:
      atomic_t cur;              //!< current location of the allocator
      size_t allocEnd;           //!< end of the allocated memory region
      size_t reserveEnd;         //!< end of the reserved memory region
      Block* next;               //!< pointer to next block in list
      char align[maxAlignment-4*sizeof(size_t)]; //!< align data to maxAlignment
      char data[1];              //!< here starts memory to use for allocations
    };

  private:
    MemoryMonitorInterface* device;
    AtomicMutex mutex;
    size_t slotMask;
    Block* volatile threadUsedBlocks[4];
    Block* volatile usedBlocks;
    Block* volatile freeBlocks;
    size_t growSize;

    ThreadLocalData<ThreadLocal> thread_local_allocators; //!< thread local allocators
    ThreadLocalData<ThreadLocal2> thread_local_allocators2; //!< thread local allocators

  private:
    size_t bytesWasted;    //!< number of bytes wasted
    size_t bytesUsed;      //!< bumber of total bytes allocated
  };
}
