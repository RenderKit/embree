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

#include "tessellation_cache.h"

namespace embree
{

  void TessellationCache::printStats()
  {
    assert(cache_hits + cache_misses == cache_accesses);
    DBG_PRINT(cache_accesses);
    DBG_PRINT(cache_misses);
    DBG_PRINT(cache_hits);
    DBG_PRINT(cache_evictions);
    DBG_PRINT(100.0f * cache_hits / cache_accesses);
    DBG_PRINT(cache_clears);
  }

  void TessellationCache::clearStats()
  {
    TessellationCache::cache_accesses  = 0;
    TessellationCache::cache_hits      = 0;
    TessellationCache::cache_misses    = 0;
    TessellationCache::cache_evictions = 0;          
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
  
  AtomicCounter TessellationCache::cache_accesses  = 0;
  AtomicCounter TessellationCache::cache_hits      = 0;
  AtomicCounter TessellationCache::cache_misses    = 0;
  AtomicCounter TessellationCache::cache_clears    = 0;
  AtomicCounter TessellationCache::cache_evictions = 0;                

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
