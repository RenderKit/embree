// ======================================================================== //
// Copyright 2009-2016 Intel Corporation                                    //
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

  void resizeTessellationCache(size_t new_size)
  {    
    if (new_size >= SharedLazyTessellationCache::MAX_TESSELLATION_CACHE_SIZE)
      new_size = SharedLazyTessellationCache::MAX_TESSELLATION_CACHE_SIZE;
    if (SharedLazyTessellationCache::sharedLazyTessellationCache.getSize() != new_size) 
      SharedLazyTessellationCache::sharedLazyTessellationCache.realloc(new_size);    
  }

  void resetTessellationCache()
  {
    //SharedLazyTessellationCache::sharedLazyTessellationCache.addCurrentIndex(SharedLazyTessellationCache::NUM_CACHE_SEGMENTS);
    SharedLazyTessellationCache::sharedLazyTessellationCache.reset();
  }
  
  SharedLazyTessellationCache::SharedLazyTessellationCache()
  {
    size = 0;
    data = nullptr;
    maxBlocks              = size/64;
    localTime              = NUM_CACHE_SEGMENTS;
    next_block             = 0;
    numRenderThreads       = 0;
#if FORCE_SIMPLE_FLUSH == 1
    switch_block_threshold = maxBlocks;
#else
    switch_block_threshold = maxBlocks/NUM_CACHE_SEGMENTS;
#endif
    threadWorkState     = new ThreadWorkState[NUM_PREALLOC_THREAD_WORK_STATES];

    //reset_state.reset();
    //linkedlist_mtx.reset();
  }

  SharedLazyTessellationCache::~SharedLazyTessellationCache() 
  {
    for (ThreadWorkState* t=current_t_state; t!=nullptr; ) 
    {
      ThreadWorkState* next = t->next;
      if (t->allocated) delete t;
      t = next;
    }

    delete[] threadWorkState;
  }

  void SharedLazyTessellationCache::getNextRenderThreadWorkState() 
  {
    const size_t id = numRenderThreads.fetch_add(1); 
    if (id >= NUM_PREALLOC_THREAD_WORK_STATES) init_t_state = new ThreadWorkState(true);
    else                                       init_t_state = &threadWorkState[id];
    
    /* critical section for updating link list with new thread state */
    linkedlist_mtx.lock();
    init_t_state->next = current_t_state;
    current_t_state = init_t_state;
    linkedlist_mtx.unlock();
  }

  void SharedLazyTessellationCache::waitForUsersLessEqual(ThreadWorkState *const t_state,
							  const unsigned int users)
   {
     while( !(t_state->counter <= users) )
     {
       _mm_pause();
       _mm_pause();
       _mm_pause();
       _mm_pause();
     }
   }

  void SharedLazyTessellationCache::allocNextSegment() 
  {
    if (reset_state.try_lock())
      {
	if (next_block >= switch_block_threshold)
	  {
            /* lock the linked list of thread states */

            linkedlist_mtx.lock();

            /* block all threads */
	    for (ThreadWorkState *t=current_t_state;t!=nullptr;t=t->next)
	      if (lockThread(t) == 1)
		waitForUsersLessEqual(t,1);


            /* switch to the next segment */
	    addCurrentIndex();
	    CACHE_STATS(PRINT("RESET TESS CACHE"));

#if FORCE_SIMPLE_FLUSH == 1
	    next_block = 0;
	    switch_block_threshold = maxBlocks;
#else
	    const size_t region = localTime % NUM_CACHE_SEGMENTS;
	    next_block = region * (maxBlocks/NUM_CACHE_SEGMENTS);
	    switch_block_threshold = next_block + (maxBlocks/NUM_CACHE_SEGMENTS);
	    assert( switch_block_threshold <= maxBlocks );
#endif

	    CACHE_STATS(SharedTessellationCacheStats::cache_flushes++);

            /* release all blocked threads */

	    for (ThreadWorkState *t=current_t_state;t!=nullptr;t=t->next)
	      unlockThread(t);

            /* unlock the linked list of thread states */

            linkedlist_mtx.unlock();
	    

	  }
	reset_state.unlock();
      }
    else
      reset_state.wait_until_unlocked();	   
  }


  void SharedLazyTessellationCache::reset()
  {
    /* lock the reset_state */
    reset_state.lock();

    /* lock the linked list of thread states */
    linkedlist_mtx.lock();

    /* block all threads */
    for (ThreadWorkState *t=current_t_state;t!=nullptr;t=t->next)
      if (lockThread(t) == 1)
        waitForUsersLessEqual(t,1);

    /* reset to the first segment */
    next_block = 0;
#if FORCE_SIMPLE_FLUSH == 1
    switch_block_threshold = maxBlocks;
#else
    switch_block_threshold = maxBlocks/NUM_CACHE_SEGMENTS;
#endif

    /* reset local time */
    localTime = NUM_CACHE_SEGMENTS;

    /* release all blocked threads */
    for (ThreadWorkState *t=current_t_state;t!=nullptr;t=t->next)
      unlockThread(t);

    /* unlock the linked list of thread states */
    linkedlist_mtx.unlock();	    

    /* unlock the reset_state */
    reset_state.unlock();
  }

  void SharedLazyTessellationCache::realloc(const size_t new_size)
  {
    /* lock the reset_state */
    reset_state.lock();

    /* lock the linked list of thread states */
    linkedlist_mtx.lock();

    /* block all threads */
    for (ThreadWorkState *t=current_t_state;t!=nullptr;t=t->next)
      if (lockThread(t) == 1)
        waitForUsersLessEqual(t,1);

    /* reallocate data */
    if (data) os_free(data,size);
    size      = new_size;
    data      = nullptr;
    if (size) data = (float*)os_malloc(size); // FIXME: do os_reserve under linux
    maxBlocks = size/64;    

    /* invalidate entire cache */
    localTime += NUM_CACHE_SEGMENTS; 

    /* reset to the first segment */
#if FORCE_SIMPLE_FLUSH == 1
    next_block = 0;
    switch_block_threshold = maxBlocks;
#else
    const size_t region = localTime % NUM_CACHE_SEGMENTS;
    next_block = region * (maxBlocks/NUM_CACHE_SEGMENTS);
    switch_block_threshold = next_block + (maxBlocks/NUM_CACHE_SEGMENTS);
    assert( switch_block_threshold <= maxBlocks );
#endif

    /* release all blocked threads */
    for (ThreadWorkState *t=current_t_state;t!=nullptr;t=t->next)
      unlockThread(t);

    /* unlock the linked list of thread states */
    linkedlist_mtx.unlock();	    

    /* unlock the reset_state */
    reset_state.unlock();
  }


  ////////////////////////////////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////////////////////////////////

  std::atomic<size_t> SharedTessellationCacheStats::cache_accesses(0);
  std::atomic<size_t> SharedTessellationCacheStats::cache_hits(0);
  std::atomic<size_t> SharedTessellationCacheStats::cache_misses(0);
  std::atomic<size_t> SharedTessellationCacheStats::cache_flushes(0);  
  SpinLock   SharedTessellationCacheStats::mtx;  
  std::atomic<size_t> *SharedTessellationCacheStats::cache_patch_builds(nullptr);
  size_t SharedTessellationCacheStats::cache_num_patches(0);

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
	    cache_patch_builds = (std::atomic<size_t>*)os_malloc(numPatches*sizeof(std::atomic<size_t>));
	    memset(cache_patch_builds,0,numPatches*sizeof(std::atomic<size_t>));
	  }
	mtx.unlock();
      }
    assert(ID < cache_num_patches);
    cache_patch_builds[ID]++;
  }
};

extern "C" void printTessCacheStats()
{
  PRINT("SHARED TESSELLATION CACHE");
  embree::SharedTessellationCacheStats::printStats();
  embree::SharedTessellationCacheStats::clearStats();
}
