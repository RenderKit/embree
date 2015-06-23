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

  __thread ThreadWorkState* SharedLazyTessellationCache::init_t_state = nullptr;
  ThreadWorkState* SharedLazyTessellationCache::current_t_state = nullptr;

  void resizeTessellationCache(const size_t new_size)
  {    
    if (SharedLazyTessellationCache::MAX_TESSELLATION_CACHE_SIZE >= new_size &&
	SharedLazyTessellationCache::sharedLazyTessellationCache.getSize() != new_size) 
      SharedLazyTessellationCache::sharedLazyTessellationCache.realloc(new_size);    
  }

  void clearTessellationCache()
  {
    SharedLazyTessellationCache::sharedLazyTessellationCache.addCurrentIndex(SharedLazyTessellationCache::NUM_CACHE_SEGMENTS);
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
#if defined(_MSC_VER)
    data                   = (float*)os_malloc(size);
#else
    data                   = (float*)os_reserve(size);
#endif

    maxBlocks              = size/64;
    localTime              = 0; // 1
    next_block             = 0;
    numRenderThreads       = 0;
#if FORCE_SIMPLE_FLUSH == 1
    switch_block_threshold = maxBlocks;
#else
    switch_block_threshold = maxBlocks/NUM_CACHE_SEGMENTS;
#endif
    threadWorkState     = (ThreadWorkState*)_mm_malloc(sizeof(ThreadWorkState)*NUM_PREALLOC_THREAD_WORK_STATES,64);

    for (size_t i=0;i<NUM_PREALLOC_THREAD_WORK_STATES;i++)
      threadWorkState[i].reset();

    reset_state.reset();
  }

  void SharedLazyTessellationCache::getNextRenderThreadWorkState() 
  {
    reset_state.lock();
    const size_t id = numRenderThreads.add(1); 
    if (id >= NUM_PREALLOC_THREAD_WORK_STATES) 
      { 
        init_t_state = (ThreadWorkState*)_mm_malloc(sizeof(ThreadWorkState),64);
        init_t_state->reset();
      }   
    else
    {
      init_t_state = &threadWorkState[id];
    }

    init_t_state->prev = current_t_state;
    current_t_state = init_t_state;

    reset_state.unlock();
  }

  void SharedLazyTessellationCache::waitForUsersLessEqual(ThreadWorkState *const t_state,
							  const unsigned int users)
   {
     while( !(t_state->counter <= users) )
       {
#if defined(__MIC__)
	 _mm_delay_32(128);
#else
	 _mm_pause();
	 _mm_pause();
	 _mm_pause();
	 _mm_pause();
#endif
       }
   }

  void SharedLazyTessellationCache::resetCache() 
  {
    if (reset_state.try_lock())
      {
	if (next_block >= switch_block_threshold)
	  {
	    //double msec = getSeconds();

	    for (ThreadWorkState *t=current_t_state;t!=nullptr;t=t->prev)
            {
	      if (lockThread(t) == 1)
		waitForUsersLessEqual(t,1);
            }


	    addCurrentIndex();
	    CACHE_STATS(PRINT("RESET TESS CACHE"));
	    //PRINT("RESET TESS CACHE");

#if FORCE_SIMPLE_FLUSH == 1
	    next_block = 0;
	    switch_block_threshold = maxBlocks;
#else
	    const size_t region = localTime % NUM_CACHE_SEGMENTS;
	    next_block = region * (maxBlocks/NUM_CACHE_SEGMENTS);
	    switch_block_threshold = next_block + (maxBlocks/NUM_CACHE_SEGMENTS);
#if 0
	    PRINT( region );
	    PRINT( maxBlocks );
	    PRINT( NUM_CACHE_SEGMENTS );
	    PRINT( maxBlocks/NUM_CACHE_SEGMENTS );
	    PRINT( next_block );
	    PRINT( switch_block_threshold );
#endif

	    assert( switch_block_threshold <= maxBlocks );

#endif

	    CACHE_STATS(SharedTessellationCacheStats::cache_flushes++);

	    for (ThreadWorkState *t=current_t_state;t!=nullptr;t=t->prev)
	      unlockThread(t);
	    
	    //msec = getSeconds()-msec;    
	    //PRINT( 1000.0f * msec );

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
    switch_block_threshold = maxBlocks/NUM_CACHE_SEGMENTS;
#endif

    if (State::instance()->verbose >= 1)
      std::cout << "Reallocating tessellation cache to " << size << " bytes, " << maxBlocks << " 64-byte blocks" << std::endl;
  }


  ////////////////////////////////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////////////////////////////////

  AtomicCounter SharedTessellationCacheStats::cache_accesses           = 0;
  AtomicCounter SharedTessellationCacheStats::cache_hits               = 0;
  AtomicCounter SharedTessellationCacheStats::cache_misses             = 0;
  AtomicCounter SharedTessellationCacheStats::cache_flushes            = 0;  
  AtomicMutex   SharedTessellationCacheStats::mtx;  
  AtomicCounter *SharedTessellationCacheStats::cache_patch_builds      = NULL;                
  size_t SharedTessellationCacheStats::cache_num_patches               = 0;
  float **SharedTessellationCacheStats::cache_new_delete_ptr           = NULL;

  void SharedTessellationCacheStats::printStats()
  {
    PRINT(cache_accesses);
    PRINT(cache_misses);
    PRINT(cache_hits);
    PRINT(cache_flushes);
    PRINT(100.0f * cache_hits / cache_accesses);
    assert(cache_hits + cache_misses == cache_accesses);
    PRINT(cache_num_patches);
    size_t patches = 0;
    size_t builds  = 0;
    for (size_t i=0;i<cache_num_patches;i++)
      if (cache_patch_builds[i])
	{
	  patches++;
	  builds += cache_patch_builds[i];
	}
    PRINT(patches);
    PRINT(builds);
    PRINT((double)builds/patches);
  }

  void SharedTessellationCacheStats::clearStats()
  {
    SharedTessellationCacheStats::cache_accesses  = 0;
    SharedTessellationCacheStats::cache_hits      = 0;
    SharedTessellationCacheStats::cache_misses    = 0;
    SharedTessellationCacheStats::cache_flushes   = 0;
    for (size_t i=0;i<cache_num_patches;i++)
      cache_patch_builds[i] = 0;
  }

  void SharedTessellationCacheStats::incPatchBuild(const size_t ID, const size_t numPatches)
  {
    if (!cache_patch_builds)
      {
	mtx.lock();
	if (!cache_patch_builds)
	  {
	    PRINT(numPatches);
	    cache_num_patches = numPatches;
	    cache_patch_builds = (AtomicCounter*)os_malloc(numPatches*sizeof(AtomicCounter));
	    memset(cache_patch_builds,0,numPatches*sizeof(AtomicCounter));
	  }
	mtx.unlock();
      }
    assert(ID < cache_num_patches);
    cache_patch_builds[ID].add(1);
  }

  void SharedTessellationCacheStats::newDeletePatchPtr(const size_t ID, const size_t numPatches, const size_t size)
  {
    assert(ID < numPatches);
    if(!cache_new_delete_ptr)
      {
	mtx.lock();
	if(!cache_new_delete_ptr)
	  {
	    PRINT(numPatches);
	    cache_num_patches = numPatches;
	    cache_new_delete_ptr = new float*[numPatches];
	    memset(cache_new_delete_ptr,0,sizeof(float*)*numPatches);
	  }
	mtx.unlock();
      }
    if (cache_new_delete_ptr[ID])
      free(cache_new_delete_ptr[ID]);
    cache_new_delete_ptr[ID] = (float*)malloc(size);
    memset(cache_new_delete_ptr[ID],0,size);
  }

};

extern "C" void printTessCacheStats()
{
  PRINT("SHARED TESSELLATION CACHE");
  embree::SharedTessellationCacheStats::printStats();
  embree::SharedTessellationCacheStats::clearStats();
}
