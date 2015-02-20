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

#include <tbb/scalable_allocator.h>
using namespace tbb;

namespace embree
{

#if defined (__MIC__)
#define USE_TBB_ALLOCATOR 1
#else
#define USE_TBB_ALLOCATOR 0
#endif

  //void*scalable_aligned_malloc(size_t size, size_t align);
  //void scalable_aligned_free(void* ptr );
  //void*scalable_aligned_realloc(void* ptr,size_t size,size_t align);

  /* alloc cache memory */
  float *alloc_tessellation_cache_mem(const size_t blocks)
  {
    //DBG_PRINT(blocks);

#if USE_TBB_ALLOCATOR == 1
    return (float*)scalable_aligned_malloc(64 * blocks,64);
#else
    return (float*)_mm_malloc(64 * blocks,64);
#endif
  }
  
  /* free cache memory */
  void free_tessellation_cache_mem(void *mem, const size_t blocks)
  {
    assert(mem);
#if USE_TBB_ALLOCATOR == 1
    scalable_aligned_free(mem);
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
