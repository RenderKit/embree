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

#include "tessellation_cache.h"

namespace embree
{

  // void TessellationCache::printStats()
  // {
  //   assert(cache_hits + cache_misses == cache_accesses);
  //   DBG_PRINT(cache_accesses);
  //   DBG_PRINT(cache_misses);
  //   DBG_PRINT(cache_hits);
  //   DBG_PRINT(cache_evictions);
  //   DBG_PRINT(100.0f * cache_hits / cache_accesses);
  //   DBG_PRINT(cache_clears);
  // }

  // void TessellationCache::clearStats()
  // {
  //   TessellationCache::cache_accesses  = 0;
  //   TessellationCache::cache_hits      = 0;
  //   TessellationCache::cache_misses    = 0;
  //   TessellationCache::cache_evictions = 0;          
  // }

  struct LocalTessellationCacheMemoryHandler
  {
    static const size_t SAVED_MEMORY_POINTERS =  16;

    size_t numSaved;
    struct SaveMem {
      void *ptr;
      size_t blocks;
    } saved[SAVED_MEMORY_POINTERS];

    __forceinline void reset()
    {
      numSaved = 0;
    }

    LocalTessellationCacheMemoryHandler() {
      reset();
    }

    void insert(void *ptr, size_t blocks)
    {
      if (numSaved < SAVED_MEMORY_POINTERS)
	{
	  /* saved memory pointer */
	  saved[numSaved].ptr    = ptr;
	  saved[numSaved].blocks = blocks;
	  numSaved++;
	}
      else
	{
	  /* select smallest item as eviction candidate */
	  size_t size = saved[0].blocks;
	  size_t index = 0;
	  for (size_t i=1;i<numSaved;i++)
	    if (saved[i].blocks < size)
	      {
		size = saved[i].blocks;
		index = i;
	      }
	  /* free eviction candidate */
	  _mm_free(saved[index].ptr);
	  /* overwrite eviction candidate*/
	  saved[index].ptr    = ptr;
	  saved[index].blocks = blocks;
	}
    }

    void *lookup(size_t blocks)
    {
      size_t index    = (size_t)-1;
      size_t s_blocks = (size_t)-1;
      for (size_t i=0;i<numSaved;i++)
	if (saved[i].blocks >= blocks &&
	    saved[i].blocks < s_blocks)
	  {
	    index = i;
	    s_blocks = saved[i].blocks;
	  }
      if (index == (size_t)-1)
	{
	  return (float*)_mm_malloc(64 * blocks,64);
	}

      void *t = saved[index].ptr;
      saved[index] = saved[numSaved-1];
      numSaved--;
      return t;
    }
  };

  __thread LocalTessellationCacheMemoryHandler *perThreadMemHandler = NULL;

  /* alloc cache memory */
  float *alloc_tessellation_cache_mem(const size_t blocks)
  {
    //DBG_PRINT(blocks);

#if 0
    if (!perThreadMemHandler)
      perThreadMemHandler = new LocalTessellationCacheMemoryHandler();

    return (float*)perThreadMemHandler->lookup(blocks);
#else
    return (float*)_mm_malloc(64 * blocks,64);
#endif
  }
  
  /* free cache memory */
  void free_tessellation_cache_mem(void *mem, const size_t blocks)
  {
    assert(mem);
#if 0
    if (!perThreadMemHandler)
      perThreadMemHandler = new LocalTessellationCacheMemoryHandler();

    perThreadMemHandler->insert(mem,blocks);
#else
    _mm_free(mem);
#endif
  }

  
  AtomicCounter SharedTessellationCacheStats::cache_accesses  = 0;
  AtomicCounter SharedTessellationCacheStats::cache_hits      = 0;
  AtomicCounter SharedTessellationCacheStats::cache_misses    = 0;
  AtomicCounter SharedTessellationCacheStats::cache_evictions = 0;                
  
  void SharedTessellationCacheStats::printStats()
  {
    DBG_PRINT(cache_accesses);
    DBG_PRINT(cache_misses);
    DBG_PRINT(cache_hits);
    DBG_PRINT(cache_evictions);
    DBG_PRINT(100.0f * cache_hits / cache_accesses);
    assert(cache_hits + cache_misses == cache_accesses);                
  }

  void SharedTessellationCacheStats::clearStats()
  {
    SharedTessellationCacheStats::cache_accesses  = 0;
    SharedTessellationCacheStats::cache_hits      = 0;
    SharedTessellationCacheStats::cache_misses    = 0;
    SharedTessellationCacheStats::cache_evictions = 0;          
  }



  AtomicCounter DistributedTessellationCacheStats::cache_accesses  = 0;
  AtomicCounter DistributedTessellationCacheStats::cache_hits      = 0;
  AtomicCounter DistributedTessellationCacheStats::cache_misses    = 0;
  AtomicCounter DistributedTessellationCacheStats::cache_evictions = 0;                
  
  void DistributedTessellationCacheStats::printStats()
  {
    DBG_PRINT(cache_accesses);
    DBG_PRINT(cache_misses);
    DBG_PRINT(cache_hits);
    DBG_PRINT(cache_evictions);
    DBG_PRINT(100.0f * cache_hits / cache_accesses);
    assert(cache_hits + cache_misses == cache_accesses);                
  }


  void DistributedTessellationCacheStats::clearStats()
  {
    DistributedTessellationCacheStats::cache_accesses  = 0;
    DistributedTessellationCacheStats::cache_hits      = 0;
    DistributedTessellationCacheStats::cache_misses    = 0;
    DistributedTessellationCacheStats::cache_evictions = 0;          
  }
  
  // AtomicCounter TessellationCache::cache_accesses  = 0;
  // AtomicCounter TessellationCache::cache_hits      = 0;
  // AtomicCounter TessellationCache::cache_misses    = 0;
  // AtomicCounter TessellationCache::cache_clears    = 0;
  // AtomicCounter TessellationCache::cache_evictions = 0;                

};

extern "C" void printTessCacheStats()
{
  DBG_PRINT("SHARED TESSELLATION CACHE");
  embree::SharedTessellationCacheStats::printStats();
  embree::SharedTessellationCacheStats::clearStats();
#if 1
  DBG_PRINT("PER THREAD TESSELLATION CACHE");  
  embree::DistributedTessellationCacheStats::printStats();
  embree::DistributedTessellationCacheStats::clearStats();
#else
  DBG_PRINT("PER THREAD TESSELLATION CACHE");  
  embree::TessellationCache::printStats();
  embree::TessellationCache::clearStats();
#endif  
}
