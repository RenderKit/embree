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

  SharedTessellationCache SharedTessellationCache::sharedTessellationCache;

  void TessellationCache::printStats()
  {
    CACHE_STATS(
                assert(cache_hits + cache_misses == cache_accesses);
                DBG_PRINT(cache_accesses);
                DBG_PRINT(cache_misses);
                DBG_PRINT(cache_hits);
                DBG_PRINT(cache_evictions);
                DBG_PRINT(100.0f * cache_hits / cache_accesses);
                DBG_PRINT(cache_clears);
                );
  }

  void TessellationCache::clearStats()
  {
  CACHE_STATS(
              TessellationCache::cache_accesses  = 0;
              TessellationCache::cache_hits      = 0;
              TessellationCache::cache_misses    = 0;
              TessellationCache::cache_evictions = 0;          
              );
  }

  void AdaptiveTessellationCache::clearStats()
  {
  CACHE_STATS(
              AdaptiveTessellationCache::cache_accesses  = 0;
              AdaptiveTessellationCache::cache_hits      = 0;
              AdaptiveTessellationCache::cache_misses    = 0;
              AdaptiveTessellationCache::cache_evictions = 0;          
              );
  }

  void AdaptiveTessellationCache::printStats()
  {
    CACHE_STATS(
                assert(cache_hits + cache_misses == cache_accesses);
                DBG_PRINT(CACHE_ENTRIES);
                DBG_PRINT(CACHE_ENTRIES * sizeof(AdaptiveTessellationCache::CacheTag));
                DBG_PRINT(cache_accesses);
                DBG_PRINT(cache_misses);
                DBG_PRINT(cache_hits);
                DBG_PRINT(cache_evictions);
                DBG_PRINT(100.0f * cache_hits / cache_accesses);
                DBG_PRINT(cache_clears);
                );
  }

  
  void SharedTessellationCache::printStats()
  {
    CACHE_STATS(
                DBG_PRINT(CACHE_ENTRIES);
                DBG_PRINT(CACHE_ENTRIES * sizeof(SharedTessellationCache::CacheTag));                
                DBG_PRINT(cache_accesses);
                DBG_PRINT(cache_misses);
                DBG_PRINT(cache_hits);
                DBG_PRINT(cache_evictions);
                DBG_PRINT(100.0f * cache_hits / cache_accesses);
                assert(cache_hits + cache_misses == cache_accesses);                
                );
  }

  void SharedTessellationCache::clearStats()
  {
    CACHE_STATS(
                SharedTessellationCache::cache_accesses  = 0;
                SharedTessellationCache::cache_hits      = 0;
                SharedTessellationCache::cache_misses    = 0;
                SharedTessellationCache::cache_evictions = 0;          
                );
  }
  
  CACHE_STATS(
              AtomicCounter TessellationCache::cache_accesses  = 0;
              AtomicCounter TessellationCache::cache_hits      = 0;
              AtomicCounter TessellationCache::cache_misses    = 0;
              AtomicCounter TessellationCache::cache_clears    = 0;
              AtomicCounter TessellationCache::cache_evictions = 0;                
              );           

  CACHE_STATS(
              AtomicCounter AdaptiveTessellationCache::cache_accesses  = 0;
              AtomicCounter AdaptiveTessellationCache::cache_hits      = 0;
              AtomicCounter AdaptiveTessellationCache::cache_misses    = 0;
              AtomicCounter AdaptiveTessellationCache::cache_clears    = 0;
              AtomicCounter AdaptiveTessellationCache::cache_evictions = 0;                
              );           

  CACHE_STATS(
              AtomicCounter SharedTessellationCache::cache_accesses  = 0;
              AtomicCounter SharedTessellationCache::cache_hits      = 0;
              AtomicCounter SharedTessellationCache::cache_misses    = 0;
              AtomicCounter SharedTessellationCache::cache_evictions = 0;                
              );           


};

extern "C" void printTessCacheStats()
{
  DBG_PRINT("SHARED TESSELLATION CACHE");
  embree::SharedTessellationCache::printStats();
  embree::SharedTessellationCache::clearStats();
#if 1
  DBG_PRINT("PER THREAD TESSELLATION CACHE");  
  embree::AdaptiveTessellationCache::printStats();
  embree::AdaptiveTessellationCache::clearStats();
#else
  DBG_PRINT("PER THREAD TESSELLATION CACHE");  
  embree::TessellationCache::printStats();
  embree::TessellationCache::clearStats();
#endif  
}
