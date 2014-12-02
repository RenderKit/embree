// ======================================================================== //
// Copyright 2009-2014 Intel Corporation                                    //
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

#include "sys/sysinfo.h"
#include "sys/sync/mutex.h"
#include "sys/taskscheduler.h"
#include "math/math.h"

#include <vector>

namespace embree
{
  /*! Global memory pool. Node, triangle, and intermediary build data
      is allocated from this memory pool and returned to it. The pool
      does not return memory to the operating system unless the clear function
      is called. */
  class Alloc
  {
  public:

    /*! Allocation block size. */
    //enum { blockSize = 512*4096 };
    enum { blockSize = 16*4096 };
    //enum { blockSize = 4*4096 };
    
    /*! single allocator object */
    static Alloc global;

    /*! Allocator default construction. */
    Alloc ();

    /*! Allocator destructor. */
    ~Alloc ();

    /*! returns size of memory pool */
    size_t size() const;
    
    /*! frees all available memory */
    void clear();
    
    /*! allocates a memory block */
    void* malloc();
    
    /*! frees a memory block */
    void free(void* ptr);
    
  private:
    MutexSys mutex;                 //<! Mutex to protect access to blocks vector
    std::vector<void*> blocks;      //<! list of available memory blocks
  };

  /*! Base class for a each memory allocator. Allocates from blocks of the 
    Alloc class and returns these blocks on destruction. */
  class AllocatorBase 
  {
  public:

    /*! Default constructor. */
    AllocatorBase () : ptr(NULL), cur(0), end(0) {
    }
    
    /*! Returns all allocated blocks to Alloc class. */
    ~AllocatorBase () {
      clear();
    }

    /*! clears the allocator */
    void clear () 
    {
      for (size_t i=0; i<blocks.size(); i++) {
        Alloc::global.free(blocks[i]); 
      }
      ptr = NULL;
      cur = end = 0;
      blocks.resize(0);
    }

    /*! returns number of bytes allocated */
    size_t bytes () {
      return blocks.size() * Alloc::blockSize;
    }

    /*! Allocates some number of bytes. */
    void* malloc(size_t bytes) 
    {
      Lock<MutexSys> lock(mutex);
      cur += bytes;
      if (cur <= end) return &ptr[cur - bytes];
      ptr = (char*) Alloc::global.malloc();
      blocks.push_back(ptr);
      cur = 0;
      end = Alloc::blockSize;
      assert(bytes<=Alloc::blockSize);
      cur += bytes;
      return &ptr[cur - bytes];
    }
    
  private:
    MutexSys mutex;                  //!< mutex to protect access to this class
    char*  ptr;                      //!< pointer to memory block
    size_t cur;                      //!< Current location of the allocator.
    size_t end;                      //!< End of the memory block.
    std::vector<void*> blocks;       //!< available memory blocks
  };

  /*! This class implements an efficient multi-threaded memory
   *  allocation scheme. The per thread allocator allocates from its
   *  current memory block or requests a new block from the slower
   *  global allocator when its block is full. */
  class LinearAllocatorPerThread : public RefCount
  {
    ALIGNED_CLASS;

  public:

     /*! each thread handles block of that many bytes locally */
    enum { allocBlockSize = 4096 };

    /*! Per thread structure holding the current memory block. */
    struct __aligned(64) ThreadAllocator 
    {
      ALIGNED_CLASS_(64);
    public:

       /*! each thread handles block of that many bytes locally */
      enum { blockSize = allocBlockSize };

      /*! Default constructor. */
      __forceinline ThreadAllocator (LinearAllocatorPerThread* alloc = NULL) 
	: alloc(alloc), ptr(NULL), cur(0), end(0) {}

      /* Allocate aligned memory from the threads memory block. */
      __forceinline void* malloc(size_t bytes, size_t align = 16) 
      {
        cur += bytes + ((align - cur) & (align-1));
        if (likely(cur <= end)) return &ptr[cur - bytes];
        ptr = (char*) alloc->block.malloc(allocBlockSize);
        cur = 0;
        end = allocBlockSize;
        if (bytes > allocBlockSize) 
          THROW_RUNTIME_ERROR("allocated block is too large");
        cur += bytes;
        return &ptr[cur - bytes];
      }

      /*! clears the allocator */
      void clear () {
        ptr = NULL;
        cur = end = 0;
      }

    public:
      LinearAllocatorPerThread* alloc;
      char*  ptr;      //!< pointer to memory block
      size_t cur;      //!< Current location of the allocator.
      size_t end;      //!< End of the memory block.
    };

    /*! Allocator default construction. */
    LinearAllocatorPerThread () {}

    /*! Return pointer to start of memory region */
    __forceinline       void* base()       { return block.ptr; }
    __forceinline const void* base() const { return block.ptr; }
    __forceinline       void* curPtr()     { return block.ptr+block.cur; }

    /*! clears the allocator */
    void clear () {
      block.clear();
    }

    /*! initializes the allocator */
    void init (size_t bytesAllocate, size_t bytesReserve) 
    {
      clear();
      const size_t numThreads = getNumberOfLogicalThreads(); // FIXME: should get passed from outside
      bytesReserve = max(bytesAllocate,bytesReserve);
      size_t bytesReserved = max(bytesReserve,size_t(allocBlockSize*numThreads));
      block.init(bytesAllocate,bytesReserved);
    }

    /*! returns number of committed bytes */
    size_t bytes () const {
      return block.cur;
    }

    void shrink () {
      block.shrink();
    }

  private:

    struct Block 
    {
      Block () 
      : ptr(NULL), cur(0), end(0), bytesAllocated(0), next(NULL) {}
      
      Block (size_t bytes, Block* next = NULL) 
      : ptr(NULL), cur(0), end(bytes), bytesAllocated(0), next(next) {}

      ~Block () {
	if (ptr) os_free(ptr,end); ptr = NULL;
	cur = end = 0;
	if (next) delete next; next = NULL;
      }

      __forceinline void init (size_t bytesAllocate, size_t bytesReserved)
      {
	if (bytesReserved != size_t(end) || bytesAllocate != bytesAllocated) 
	{
	  bytesAllocated = bytesAllocate;
	  if (ptr) os_free(ptr,end);
	  ptr = (char*) os_reserve(bytesReserved);
	  os_commit(ptr,bytesAllocated);
	  end = bytesReserved;
	}
      }

      __forceinline void clear() {
	cur = 0;
      }

      /*! Allocates some number of bytes. */
      void* malloc(size_t bytes) 
      {
	ssize_t i = atomic_add(&cur,bytes);
	if (unlikely(i+(ssize_t)bytes > end)) THROW_RUNTIME_ERROR("build out of memory");
	void* p = &ptr[i];
	if (i+(ssize_t)bytes > bytesAllocated)
	  os_commit(p,bytes);
	return p;
      }

      void shrink () {
	if (ptr == NULL) return;
	os_shrink(ptr,cur,end);
	end = cur;
	bytesAllocated = cur;
      }

    public:
      char*  ptr;                //!< pointer to memory
      atomic_t cur;              //!< Current location of the allocator.
      atomic_t end;              //!< End of the memory block.
      Block* next;
      atomic_t bytesAllocated;
    };

  private:
    Block block;
  };



  class FastAllocator 
  {
    /*! maximal supported alignment */
    static const size_t maxAlignment = 64;

    /*! maximal allocation size */
    static const size_t maxAllocationSize = 2*1024*1024-maxAlignment;

  public:

    /*! Per thread structure holding the current memory block. */
    struct __aligned(64) Thread 
    {
      ALIGNED_CLASS_(64);
    public:

      /*! Default constructor. */
      __forceinline Thread (FastAllocator* alloc, const size_t allocBlockSize = 4096) 
	: alloc(alloc), ptr(NULL), cur(0), end(0), allocBlockSize(allocBlockSize) {}

      /* Allocate aligned memory from the threads memory block. */
      __forceinline void* malloc(size_t bytes, size_t align = 16) 
      {
        assert(align <= maxAlignment);

        /* try to allocate in local block */
        cur += bytes + ((align - cur) & (align-1));
        if (likely(cur <= end)) return &ptr[cur - bytes];

        /* if allocation is too large allocate with parent allocator */
        if (4*bytes > allocBlockSize) {
          cur -= bytes;
          return alloc->malloc(bytes,maxAlignment);
        }

#if 0
        /* get new partial block if allocation failed */
	if (alloc->usedBlocks) 
	{
	  size_t blockSize = allocBlockSize;
	  ptr = (char*) alloc->usedBlocks->malloc_some(blockSize,maxAlignment);
	  end = blockSize;
	  
	  /* retry allocation */
	  cur = bytes + ((align - cur) & (align-1));
	  if (likely(cur <= end)) return &ptr[cur - bytes];
	}
#endif

        /* get new full block if allocation failed */
        size_t blockSize = allocBlockSize;
        ptr = (char*) alloc->malloc(blockSize,maxAlignment);
        end = blockSize;

        /* retry allocation */
        cur = bytes + ((align - cur) & (align-1));
        if (likely(cur <= end)) return &ptr[cur - bytes];

        /* should never happen as large allocations get handled specially above */
        assert(false);
        return NULL;
      }

      /*! clears the allocator */
      void clear () {
        ptr = NULL;
        cur = end = 0;
      }

    public:
      FastAllocator* alloc;  //!< parent allocator
      char*  ptr;            //!< pointer to memory block
      size_t cur;            //!< current location of the allocator
      size_t end;            //!< end of the memory block
      size_t allocBlockSize; //!< block size for allocations
    };

    FastAllocator () 
      : growSize(4096), usedBlocks(NULL), freeBlocks(NULL) {}

    ~FastAllocator () { 
      if (usedBlocks) usedBlocks->~Block(); usedBlocks = NULL;
      if (freeBlocks) freeBlocks->~Block(); freeBlocks = NULL;
    }

    /*! initializes the allocator */
    void init(size_t bytesAllocate, size_t bytesReserve) {
      usedBlocks = Block::create(bytesAllocate,bytesReserve);
      growSize = bytesReserve;
    }

    /*! resets the allocator, memory blocks get reused */
    void reset () 
    {
      /* first reset all used blocks */
      if (usedBlocks) usedBlocks->reset();

      /* find end of free block list */
      Block* volatile& freeBlocksEnd = freeBlocks;
      while (freeBlocksEnd) freeBlocksEnd = freeBlocksEnd->next;

      /* add previously used blocks to end of free block list */
      freeBlocksEnd = usedBlocks;
      usedBlocks = NULL;
    }

    /*! shrinks all memory blocks to the actually used size */
    void shrink () {
      usedBlocks->shrink();
      if (freeBlocks) freeBlocks->~Block(); freeBlocks = NULL;
    }

    /*! thread safe allocation of memory */
    void* malloc(size_t bytes, size_t align) 
    {
      assert(align <= maxAlignment);

      while (true) 
      {
        /* allocate using current block */
	Block* myUsedBlocks = usedBlocks;
        if (myUsedBlocks) {
          void* ptr = usedBlocks->malloc(bytes,align);
          if (ptr) return ptr;
        }

        /* throw error if allocation is too large */
        if (bytes > maxAllocationSize)
          THROW_RUNTIME_ERROR("allocation is too large");

        /* if this fails allocate new block */
        {
          Lock<AtomicMutex> lock(mutex);
	  if (myUsedBlocks == usedBlocks)
	  {
	    if (freeBlocks) {
	      Block* nextFreeBlock = freeBlocks->next;
	      freeBlocks->next = usedBlocks;
	      __memory_barrier();
	      usedBlocks = freeBlocks;
	      freeBlocks = nextFreeBlock;
	    } else {
	      growSize = min(2*growSize,size_t(maxAllocationSize+maxAlignment));
	      usedBlocks = Block::create(growSize-maxAlignment, growSize-maxAlignment, usedBlocks);
	    }
	  }
        }
      }
    }

  private:

    struct Block 
    {
      static Block* create(size_t bytesAllocate, size_t bytesReserve, Block* next = NULL)
      {
        void* ptr = os_reserve(sizeof(Block)+bytesReserve);
        os_commit(ptr,sizeof(Block)+bytesAllocate);
        bytesAllocate = ((sizeof(Block)+bytesAllocate+4095) & ~(4095)) - sizeof(Block); // always comsume full pages
        bytesReserve  = ((sizeof(Block)+bytesReserve +4095) & ~(4095)) - sizeof(Block); // always comsume full pages
        return new (ptr) Block(bytesAllocate,bytesReserve,next);
      }

      Block (size_t bytesAllocate, size_t bytesReserve, Block* next) 
      : cur(0), allocEnd(bytesAllocate), reserveEnd(bytesReserve), next(next) {}

      ~Block () {
	if (next) next->~Block(); next = NULL;
        os_free(this,sizeof(Block)+reserveEnd);
      }

      void* malloc(size_t bytes, size_t align = 16) 
      {
        assert(align <= maxAlignment);
        bytes = (bytes+(align-1)) & ~(align-1); // FIXME: works only if all alignments are equal
	if (unlikely(cur+bytes > reserveEnd)) return NULL;
	const size_t i = atomic_add(&cur,bytes);
	if (unlikely(i+bytes > reserveEnd)) return NULL;
	if (i+bytes > allocEnd) os_commit(&data[i],bytes); // FIXME: optimize, may get called frequently
	return &data[i];
      }

      void* malloc_some(size_t& bytes, size_t align = 16) 
      {
        assert(align <= maxAlignment);
        bytes = (bytes+(align-1)) & ~(align-1); // FIXME: works only if all alignments are equal
	const size_t i = atomic_add(&cur,bytes);
	if (unlikely(i+bytes > reserveEnd)) bytes = reserveEnd-i;
	if (i+bytes > allocEnd) os_commit(&data[i],bytes); // FIXME: optimize, may get called frequently
	return &data[i];
      }

      void reset () 
      {
        allocEnd = max(allocEnd,(size_t)cur);
        cur = 0;
        if (next) next->reset();
      }

      void shrink () 
      {
        os_shrink(&data[0],cur,reserveEnd);
        reserveEnd = allocEnd = cur;
        if (next) next->shrink();
      }

    public:
      atomic_t cur;              //!< current location of the allocator
      size_t allocEnd;           //!< end of the allocated memory region
      size_t reserveEnd;         //!< end of the reserved memory region
      Block* next;               //!< pointer to next block in list
      char align[maxAlignment-4*sizeof(size_t)]; //!< align data to maxAlignment
      char data[];               //!< here starts memory to use for allocations
    };

  private:
    AtomicMutex mutex;
    Block* volatile usedBlocks;
    Block* volatile freeBlocks;
    size_t growSize;
  };
}
