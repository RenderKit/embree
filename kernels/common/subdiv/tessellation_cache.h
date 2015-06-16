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

#include "../default.h"
//#include "subdivpatch1base.h"

#define CACHE_DBG(x) 

/* force a complete cache invalidation when running out of allocation space */
#define FORCE_SIMPLE_FLUSH 0

#if defined(__MIC__)
#define NEW_TCACHE_SYNC 0
#else
#define NEW_TCACHE_SYNC 0
#endif


#if defined(DEBUG)
#define CACHE_STATS(x) 
#else
#define CACHE_STATS(x) 
#endif


namespace embree
{
  void resizeTessellationCache(const size_t new_size);
  void clearTessellationCache();

  /* alloc cache memory */
  float *alloc_tessellation_cache_mem(const size_t blocks);

  /* free cache memory */
  void free_tessellation_cache_mem(void *mem, const size_t blocks = 0);


#if defined(__MIC__)
 typedef unsigned int InputTagType;
#else
 typedef size_t InputTagType;
#endif

 static __forceinline unsigned int toTag(InputTagType prim)
 {
#if defined(__MIC__)
   return prim;
#else
   return prim / 320;
#endif
 }
 ////////////////////////////////////////////////////////////////////////////////
 ////////////////////////////////////////////////////////////////////////////////
 ////////////////////////////////////////////////////////////////////////////////

 struct __aligned(64) ThreadWorkState {
   AtomicCounter counter;
   ThreadWorkState *next;
   
   ThreadWorkState() { assert( ((size_t)this % 64) == 0 ); counter = 0; next = NULL; }
   __forceinline void reset() { counter = 0; }
 };


 class __aligned(64) SharedLazyTessellationCache 
 {
 public:
   static const size_t MAX_TESSELLATION_CACHE_SIZE     = 512*1024*1024; // 512 MB = 2^28, need 4 lowest bit for BVH node types
   static const size_t DEFAULT_TESSELLATION_CACHE_SIZE = MAX_TESSELLATION_CACHE_SIZE; 

    /*! Per thread tessellation ref cache */
   static __thread ThreadWorkState* init_t_state;

   /*! Creates per thread tessellation cache */
   static void createLocalThreadInfo();
   
   static __forceinline ThreadWorkState *threadState() 
   {
     if (unlikely(!init_t_state))
       /* sets init_t_state, can't return due to macosx icc bug*/
       SharedLazyTessellationCache::sharedLazyTessellationCache.getNextRenderThreadWorkState();
     return init_t_state;
   }

   struct Tag
   {
     __forceinline Tag() : data(0) {}

     __forceinline Tag(void* ptr)
     {
       int64_t new_root_ref = (int64_t) ptr;
       new_root_ref -= (int64_t)SharedLazyTessellationCache::sharedLazyTessellationCache.getDataPtr();                                
       assert( new_root_ref <= 0xffffffff );
       static const size_t REF_TAG      = 1;
       assert( !(new_root_ref & REF_TAG) );
       new_root_ref |= REF_TAG;
       new_root_ref |= (int64_t)sharedLazyTessellationCache.getCurrentIndex() << 32; 
       data = new_root_ref;
     }

     volatile int64_t data;
   };

   struct CacheEntry
   {
     RWMutex mutex;
     Tag tag;
   };

 private:

   float *data;
   size_t size;
   size_t maxBlocks;
   size_t numMaxRenderThreads;
   ThreadWorkState *threadWorkState;
      
   __aligned(64) AtomicCounter index;
   __aligned(64) AtomicCounter next_block;
   __aligned(64) AtomicMutex   reset_state;
   __aligned(64) AtomicCounter switch_block_threshold;
   __aligned(64) AtomicCounter numRenderThreads;



 public:

#if defined(__MIC__)
   static const size_t NUM_CACHE_SEGMENTS = 4;
#else
   static const size_t NUM_CACHE_SEGMENTS = 8;
   //static const size_t NUM_CACHE_SEGMENTS = 16;

#endif
      
   SharedLazyTessellationCache();

   void getNextRenderThreadWorkState();

   __forceinline size_t getCurrentIndex() { return index; }
   __forceinline void   addCurrentIndex(const size_t i=1) { index.add(i); }

   __forceinline unsigned int lockThread  (ThreadWorkState *const t_state) { return t_state->counter.add(1);  }
   __forceinline unsigned int unlockThread(ThreadWorkState *const t_state) { return t_state->counter.add(-1); }

   static __forceinline void lock  () { sharedLazyTessellationCache.lockThread(threadState()); }
   static __forceinline void unlock() { sharedLazyTessellationCache.unlockThread(threadState()); }

   /* per thread lock */
   __forceinline void lockThreadLoop (ThreadWorkState *const t_state) 
   { 
     while(1)
     {
       unsigned int lock = SharedLazyTessellationCache::sharedLazyTessellationCache.lockThread(t_state);
       if (unlikely(lock == 1))
       {
         /* lock failed wait until sync phase is over */
         sharedLazyTessellationCache.unlockThread(t_state);	       
         sharedLazyTessellationCache.waitForUsersLessEqual(t_state,0);
       }
       else
         break;
     }
   }

   static __forceinline void* lookup(volatile Tag* tag)
   {
     static const size_t REF_TAG      = 1;
     static const size_t REF_TAG_MASK = (~REF_TAG) & 0xffffffff;
       
     const int64_t subdiv_patch_root_ref = tag->data; 
     
     if (likely(subdiv_patch_root_ref)) 
     {
       const size_t subdiv_patch_root = (subdiv_patch_root_ref & REF_TAG_MASK) + (size_t)sharedLazyTessellationCache.getDataPtr();
       const size_t subdiv_patch_cache_index = subdiv_patch_root_ref >> 32;
       
       if (likely( sharedLazyTessellationCache.validCacheIndex(subdiv_patch_cache_index) ))
       {
         CACHE_STATS(SharedTessellationCacheStats::cache_hits++);
         return (void*) subdiv_patch_root;
       }
     }
     CACHE_STATS(SharedTessellationCacheStats::cache_misses++);
     return nullptr;
   }

   template<typename Constructor>
     static __forceinline auto lookup (CacheEntry& entry, const Constructor constructor) -> decltype(constructor())
   {
     ThreadWorkState *t_state = SharedLazyTessellationCache::threadState();

     while (true)
     {
       sharedLazyTessellationCache.lockThreadLoop(t_state);
       void* patch = SharedLazyTessellationCache::lookup(&entry.tag);
       if (patch) return (decltype(constructor())) patch;
       
       if (entry.mutex.try_write_lock())
       {
         if (!validTag(entry.tag)) 
         {
           auto ret = constructor();
           __memory_barrier();
           entry.tag = SharedLazyTessellationCache::Tag(ret);
           __memory_barrier();
           entry.mutex.write_unlock();
           return ret;
         }
         entry.mutex.write_unlock();
       }
       SharedLazyTessellationCache::sharedLazyTessellationCache.unlockThread(t_state);
     }
   }
   
   static __forceinline size_t lookupIndex(volatile Tag* tag)
   {
     static const size_t REF_TAG      = 1;
     static const size_t REF_TAG_MASK = (~REF_TAG) & 0xffffffff;
       
     const int64_t subdiv_patch_root_ref = tag->data; 
     
     if (likely(subdiv_patch_root_ref)) 
     {
       const size_t subdiv_patch_root = (subdiv_patch_root_ref & REF_TAG_MASK);
       const size_t subdiv_patch_cache_index = subdiv_patch_root_ref >> 32;
       
       if (likely( sharedLazyTessellationCache.validCacheIndex(subdiv_patch_cache_index) ))
       {
         CACHE_STATS(SharedTessellationCacheStats::cache_hits++);
         return subdiv_patch_root;
       }
     }
     CACHE_STATS(SharedTessellationCacheStats::cache_misses++);
     return -1;
   }

   __forceinline void prefetchThread(ThreadWorkState *const t_state) { 
#if defined(__MIC__)
     prefetch<PFHINT_L1EX>(&t_state->counter);  
#endif
   }


   __forceinline bool validCacheIndex(const size_t i)
   {
#if FORCE_SIMPLE_FLUSH == 1
     return i == index;
#else
     return i+(NUM_CACHE_SEGMENTS-1) >= index;
#endif
   }

    static __forceinline bool validTag(const Tag& tag)
    {
      const int64_t subdiv_patch_root_ref = tag.data; 
      if (subdiv_patch_root_ref == 0) return false;
      const size_t subdiv_patch_cache_index = subdiv_patch_root_ref >> 32;
      return sharedLazyTessellationCache.validCacheIndex(subdiv_patch_cache_index);
    }

   void waitForUsersLessEqual(ThreadWorkState *const t_state,
			      const unsigned int users);
    
   __forceinline size_t alloc(const size_t blocks)
   {
     size_t index = next_block.add(blocks);
     if (unlikely(index + blocks >= switch_block_threshold)) return (size_t)-1;
     return index;
   }

   static __forceinline size_t allocIndexLoop(ThreadWorkState *const t_state, const size_t blocks)
   {
     size_t block_index = -1;
     while (true)
     {
       block_index = sharedLazyTessellationCache.alloc(blocks);
       if (block_index == (size_t)-1)
       {
         sharedLazyTessellationCache.unlockThread(t_state);		  
         sharedLazyTessellationCache.resetCache();
         sharedLazyTessellationCache.lockThread(t_state);
         continue; 
       }
       break;
     }
     return block_index;
   }

   static __forceinline void* allocLoop(ThreadWorkState *const t_state, const size_t bytes)
   {
     size_t block_index = -1;
     while (true)
     {
       block_index = sharedLazyTessellationCache.alloc((bytes+63)/64);
       if (block_index == (size_t)-1)
       {
         sharedLazyTessellationCache.unlockThread(t_state);		  
         sharedLazyTessellationCache.resetCache();
         sharedLazyTessellationCache.lockThread(t_state);
         continue; 
       }
       break;
     }
     return sharedLazyTessellationCache.getBlockPtr(block_index);
   }

   static __forceinline void* malloc(const size_t bytes)
   {
     size_t block_index = -1;
     ThreadWorkState *const t_state = threadState();
     while (true)
     {
       block_index = sharedLazyTessellationCache.alloc((bytes+63)/64);
       if (block_index == (size_t)-1)
       {
         sharedLazyTessellationCache.unlockThread(t_state);		  
         sharedLazyTessellationCache.resetCache();
         sharedLazyTessellationCache.lockThread(t_state);
         continue; 
       }
       break;
     }
     return sharedLazyTessellationCache.getBlockPtr(block_index);
   }

   __forceinline void *getBlockPtr(const size_t block_index)
   {
     assert(block_index < maxBlocks);
     return (void*)&data[block_index*16];
   }

   __forceinline void*  getDataPtr()      { return data; }
   __forceinline size_t getNumUsedBytes() { return next_block * 64; }
   __forceinline size_t getMaxBlocks()    { return maxBlocks; }
   __forceinline size_t getSize()         { return size; }

   void resetCache();
   void realloc(const size_t newSize);

   static SharedLazyTessellationCache sharedLazyTessellationCache;
    
 };

 ////////////////////////////////////////////////////////////////////////////////
 ////////////////////////////////////////////////////////////////////////////////
 ////////////////////////////////////////////////////////////////////////////////


 class SharedTessellationCacheStats
 {
 public:
    /* stats */
   static AtomicCounter cache_accesses;
   static AtomicCounter cache_hits;
   static AtomicCounter cache_misses;
   static AtomicCounter cache_flushes;                
   static AtomicCounter *cache_patch_builds;                
   static size_t        cache_num_patches;
   static float **      cache_new_delete_ptr;  
   __aligned(64) static AtomicMutex mtx;

    /* print stats for debugging */                 
    static void printStats();
    static void clearStats();
    static void incPatchBuild(const size_t ID, const size_t numPatches);
    static void newDeletePatchPtr(const size_t ID,  const size_t numPatches, const size_t size);

 };

  // =========================================================================================================
  // =========================================================================================================
  // =========================================================================================================

};
