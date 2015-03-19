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
  SharedLazyTessellationCache SharedLazyTessellationCache::sharedLazyTessellationCache;

  void resizeTessellationCache(const size_t new_size)
  {
    if (SharedLazyTessellationCache::sharedLazyTessellationCache.getSize() != new_size)
      SharedLazyTessellationCache::sharedLazyTessellationCache.realloc(new_size);
  }

  void clearTessellationCache()
  {
    SharedLazyTessellationCache::sharedLazyTessellationCache.addCurrentIndex(SharedLazyTessellationCache::NUM_CACHE_REGIONS);
  }
  
  /* alloc cache memory */
  float *alloc_tessellation_cache_mem(const size_t blocks)
  {
    return (float*)_mm_malloc(64 * blocks,64);
  }
  
  /* free cache memory */
  void free_tessellation_cache_mem(void *mem, const size_t blocks)
  {
    assert(mem);
    _mm_free(mem);
  }


  SharedLazyTessellationCache::SharedLazyTessellationCache()
  {
    size                   = DEFAULT_TESSELLATION_CACHE_SIZE;
    data                   = (float*)os_malloc(size);
    maxBlocks              = size/64;
    index                  = 0; // 1
    next_block             = 0;
    numRenderThreads       = 0;
#if FORCE_SIMPLE_FLUSH == 1
    switch_block_threshold = maxBlocks;
#else
    switch_block_threshold = maxBlocks/NUM_CACHE_REGIONS;
#endif

    reset_state.reset();
  }

  void SharedLazyTessellationCache::resetCache() 
  {
    if (reset_state.try_lock())
      {
	if (next_block >= switch_block_threshold)
	  {
	    //double msec = getSeconds();

	    for (size_t i=0;i<numRenderThreads;i++)
	      lockThread(i);

	    for (size_t i=0;i<numRenderThreads;i++)
	      waitForUsersLessEqual(i,1);

	    addCurrentIndex();
	    CACHE_STATS(DBG_PRINT("RESET TESS CACHE"));

#if FORCE_SIMPLE_FLUSH == 1
	    next_block = 0;
	    switch_block_threshold = maxBlocks;
#else
	    const size_t region = index % NUM_CACHE_REGIONS;
	    next_block = region * (maxBlocks/NUM_CACHE_REGIONS);
	    switch_block_threshold = next_block + (maxBlocks/NUM_CACHE_REGIONS);

#if 0
	    DBG_PRINT( region );
	    DBG_PRINT( maxBlocks );
	    DBG_PRINT( NUM_CACHE_REGIONS );
	    DBG_PRINT( maxBlocks/NUM_CACHE_REGIONS );
	    DBG_PRINT( next_block );
	    DBG_PRINT( switch_block_threshold );
#endif

	    assert( switch_block_threshold <= maxBlocks );

#endif

	    CACHE_STATS(SharedTessellationCacheStats::cache_flushes++);

	    for (size_t i=0;i<numRenderThreads;i++)
	      unlockThread(i);

	    //msec = getSeconds()-msec;    
	    //DBG_PRINT( 1000.0f * msec );

	  }
	reset_state.unlock();
      }
    else
      reset_state.wait_until_unlocked();	    
  }

  void SharedLazyTessellationCache::realloc(const size_t new_size)
  {
    if (data)
      {
	os_free(data,size);
      }
    size      = new_size;
    data      = (float*)os_malloc(size);
    maxBlocks = size/64;    
#if FORCE_SIMPLE_FLUSH == 1
    switch_block_threshold = maxBlocks;
#else
    switch_block_threshold = maxBlocks/NUM_CACHE_REGIONS;
#endif

    std::cout << "Reallocating tessellation cache to " << size << " bytes, " << maxBlocks << " 64-byte blocks" << std::endl;
  }


  ////////////////////////////////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////////////////////////////////

  AtomicCounter SharedTessellationCacheStats::cache_accesses           = 0;
  AtomicCounter SharedTessellationCacheStats::cache_hits               = 0;
  AtomicCounter SharedTessellationCacheStats::cache_misses             = 0;
  AtomicCounter SharedTessellationCacheStats::cache_flushes            = 0;                

  void SharedTessellationCacheStats::printStats()
  {
    DBG_PRINT(cache_accesses);
    DBG_PRINT(cache_misses);
    DBG_PRINT(cache_hits);
    DBG_PRINT(cache_flushes);
    DBG_PRINT(100.0f * cache_hits / cache_accesses);
    assert(cache_hits + cache_misses == cache_accesses);                
  }

  void SharedTessellationCacheStats::clearStats()
  {
    SharedTessellationCacheStats::cache_accesses  = 0;
    SharedTessellationCacheStats::cache_hits      = 0;
    SharedTessellationCacheStats::cache_misses    = 0;
    SharedTessellationCacheStats::cache_flushes   = 0;          
  }

};

extern "C" void printTessCacheStats()
{
  DBG_PRINT("SHARED TESSELLATION CACHE");
  embree::SharedTessellationCacheStats::printStats();
  embree::SharedTessellationCacheStats::clearStats();
}
