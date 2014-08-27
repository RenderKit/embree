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
  class AllocatorPerThread : public AllocatorBase
  {
    ALIGNED_CLASS;

     /*! Allocation block size. Number of bytes to request from the
      *  base allocator when the memory block of a thread is empty. */
    //enum { allocBlockSize = 4096*16 };
    enum { allocBlockSize = 4*4096 };

  public:

    /*! Allocator default construction. */
    AllocatorPerThread () {
      thread = new ThreadAllocator[getNumberOfLogicalThreads()];
    }

    /*! Allocator destructor. */
    ~AllocatorPerThread() {
      delete[] thread; thread = NULL;
    }

    /*! Aligned memory allocation */
    __forceinline void* malloc(size_t tinfo, size_t bytes, size_t align = 16) {
      return thread[tinfo].malloc(bytes,align,this);
    }

    /*! clears the allocator */
    void clear () 
    {
      AllocatorBase::clear();
      for (size_t i=0; i<getNumberOfLogicalThreads(); i++) 
        thread[i].clear();
    }

  private:

     /*! Per thread structure holding the current memory block. */
    struct __aligned(4096) ThreadAllocator 
    {
      ALIGNED_CLASS_(4096);
    public:

      /*! Default constructor. */
      __forceinline ThreadAllocator () : ptr(NULL), cur(0), end(0) {}

      /* Allocate aligned memory from the threads memory block. */
      __forceinline void* malloc(size_t bytes, size_t align, AllocatorBase* alloc) 
      {
        cur += (align - cur) & (align-1);
        cur += bytes;
        if (cur <= end) return &ptr[cur - bytes];
        ptr = (char*) alloc->malloc(allocBlockSize);
        cur = 0;
        end = allocBlockSize;
        if (bytes > allocBlockSize) 
          throw std::runtime_error("allocated block is too large");
        cur += bytes;
        return &ptr[cur - bytes];
      }

      /*! clears the allocator */
      void clear () {
        ptr = NULL;
        cur = end = 0;
      }

    public:
      char*  ptr;      //!< pointer to memory block
      size_t cur;      //!< Current location of the allocator.
      size_t end;      //!< End of the memory block.
    };

  private:
    ThreadAllocator* thread;   //!< one allocator for each thread
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
          throw std::runtime_error("allocated block is too large");
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

    /*! Aligned memory allocation */
    __forceinline void* malloc(size_t tinfo, size_t bytes, size_t align = 16) {
      return thread[tinfo].malloc(bytes,align);
    }

    /*! clears the allocator */
    void clear () 
    {
      block.clear();
      for (size_t i=0; i<thread.size(); i++) 
	thread[i].clear();
    }

    /*! initializes the allocator */
    void init (size_t bytesAllocate, size_t bytesReserve) 
    {
      clear();
      const size_t numThreads = getNumberOfLogicalThreads();
      if (thread.size() != numThreads) {
	thread.resize(numThreads);
	for (size_t i=0; i<numThreads; i++) thread[i].alloc = this;
      }
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
      : ptr(NULL), cur(0), end(0), bytesAllocated(bytesAllocated), next(NULL) {}
      
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
	if (unlikely(i > end)) throw std::runtime_error("build out of memory");
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
    std::vector<ThreadAllocator> thread;   //!< one allocator for each thread
    Block block;
  };

  class __aligned(64) GlobalAllocator
  {
  public:
    
    class __aligned(64) ThreadAllocator
    {
    public:        
      static const size_t blockSize = 2*4096;
      
      __forceinline ThreadAllocator(GlobalAllocator& global) 
        : global(global), begin(NULL), end(NULL) {}
      
      __forceinline void* malloc(size_t bytes) 
      {
        if (unlikely(begin+bytes > end)) {
          begin = (char*) global.malloc(blockSize); 
          end = begin+blockSize;
        }
        
        char* next = begin;
        begin+=bytes;
        
        if (unlikely(begin > end))
          throw std::runtime_error("allocation too large");
        
        return next;
      }
      
    public:
      GlobalAllocator& global;
      char* begin;
      char* end;
      size_t align;
    };
    
  public:
    GlobalAllocator () 
      : data(NULL), bytesAllocated(0), bytesReserved(0), next(0) {}
    
    void init(size_t bytesAllocate, size_t bytesReserve) 
    {
      if (bytesAllocated == bytesAllocate && bytesReserved == bytesReserve)
        return;
      
      if (bytesAllocate > bytesReserve)
        bytesReserve = bytesAllocate;

      if (data) os_free(data,bytesReserved);
      
      data = (char*) os_reserve(bytesReserve);
      bytesReserved = bytesReserve;
      
      os_commit(data,bytesAllocate);
      memset(data,0,bytesAllocate);
      bytesAllocated = bytesAllocate;
      
      next = 0;
    }
    
    void reset () {
      next = 0;
    }
    
    void shrink () {
      if (data == NULL) return;
      os_shrink(data,next,bytesReserved);
      bytesAllocated = next;
      bytesReserved = next;
    }
    
    __forceinline void* malloc(size_t bytes) 
    {
      size_t n = atomic_add(&next,bytes);
      if (unlikely(n+bytes > bytesAllocated)) {
        if (n+bytes > bytesReserved) {
          throw std::runtime_error("reserved memory region too small");
        }
        os_commit(data+n,bytes);
      }
      return &data[n];
    }

    __forceinline void* base() {
      return data;
    }

  public:
    atomic_t next;
    char* data;
    size_t bytesAllocated;
    size_t bytesReserved;
  };
}
