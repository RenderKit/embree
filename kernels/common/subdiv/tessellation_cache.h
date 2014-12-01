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

#include "xeon/bvh4/bvh4.h"

namespace embree
{
  class __aligned(64) TessellationCache {

  private:
    /* default sizes */
    static const size_t DEFAULT_64B_BLOCKS = 8192;
    static const size_t CACHE_ENTRIES      = 8;

    /* 64bit pointers cache tags for now */
    void         *prim_tag[CACHE_ENTRIES];

    /* bvh4 subtree roots per cache entry */
    BVH4::NodeRef bvh4_ref[CACHE_ENTRIES];

    /* allocated memory to store cache entries */
    float *lazymem;

    /* total number of allocated 64bytes blocks */
    size_t allocated64BytesBlocks;

    /* current number of allocated blocks */
    size_t blockCounter;


    /* stats */
#if DEBUG
    size_t cache_accesses;
    size_t cache_hits;
    size_t cache_misses;
#endif

    /* alloc cache memory */
    __forceinline float *alloc_mem(const size_t blocks)
    {
      return (float*)_mm_malloc(64 * blocks,64);
    }

    /* free cache memory */
    __forceinline void free_mem(float *mem)
    {
      _mm_free(mem);
    }

  public:

    /* reset cache */
    __forceinline void clear()
    {
      blockCounter =  0;	  
      for (size_t i=0;i<CACHE_ENTRIES;i++)
        {
          prim_tag[i] = NULL;
          bvh4_ref[i] = 0;	
        }

#if DEBUG
      cache_accesses = 0;
      cache_hits     = 0;
      cache_misses   = 0;
#endif
    }

    __forceinline float *getPtr()
    {
      return lazymem;
    }


    TessellationCache()  {}

    /* initialize cache */
    __forceinline void init()
    {
      clear();
      allocated64BytesBlocks = DEFAULT_64B_BLOCKS;	
      lazymem = alloc_mem( allocated64BytesBlocks );
      assert((size_t)lazymem % 64 == 0);
    }


    /* lookup cache entry using 64bit pointer as tag */
    __forceinline BVH4::NodeRef lookup(void *primID)
    {
      ssize_t index = -1;

      for (size_t i=0;i<CACHE_ENTRIES;i++)
        if (prim_tag[i] == primID)
          {
            index = i;
            break;
          }

#if DEBUG
      cache_accesses++;
#endif

      if (likely(index != -1))
        {
#if DEBUG
          cache_hits++;
#endif

          const BVH4::NodeRef ref = bvh4_ref[index];
          /* move most recently accessed entry to the beginning of the array */
          // not correct but fast
          std::swap(bvh4_ref[index],bvh4_ref[0]);
          std::swap(prim_tag[index],prim_tag[0]);
          return ref;
        }

#if DEBUG
      cache_misses++;
#endif

      return BVH4::invalidNode;
    }

    /* insert entry using 'neededBlocks' cachelines into cache */
    __forceinline BVH4::NodeRef insert(void *primID, const size_t neededBlocks)
    {
      /* can't hold required subtree space => flush and reset cache */
      if (unlikely(blockCounter + neededBlocks >= allocated64BytesBlocks))
        clear();

      /* can the cache hold this subtree space at all in each cache entries? */
      if (unlikely(CACHE_ENTRIES*neededBlocks > allocated64BytesBlocks)) 
        {
          const unsigned int new_allocated64BytesBlocks = CACHE_ENTRIES*neededBlocks;

          std::cout << "EXTENDING TESSELLATION CACHE (PER THREAD) FROM " 
                    << allocated64BytesBlocks << "TO " 
                    << new_allocated64BytesBlocks << " BLOCKS = " 
                    << new_allocated64BytesBlocks*64 << " BYTES" << std::endl << std::flush;

          free_mem(lazymem);
          allocated64BytesBlocks = new_allocated64BytesBlocks; 
          lazymem = alloc_mem(allocated64BytesBlocks);
          assert(lazymem);

          /* realloc */
          clear();
        }

      const size_t currentIndex = blockCounter;
      blockCounter += neededBlocks;

      BVH4::NodeRef curNode = BVH4::encodeNode( (BVH4::Node*)&lazymem[currentIndex*16] );

      /* shift old cache entries */
      for (size_t i=CACHE_ENTRIES-1;i>0;i--)
        {
          prim_tag[i] = prim_tag[i-1];
          bvh4_ref[i] = bvh4_ref[i-1];
        }

      /* insert new entry at the beginning */
      prim_tag[0]  = primID;
      bvh4_ref[0] = curNode;

      return curNode;
    }

    /* print stats for debugging */                 
    void printStats() const
    {
#if DEBUG
      if (cache_accesses)
        {
          assert(cache_hits + cache_misses == cache_accesses);
          DBG_PRINT(cache_accesses);
          DBG_PRINT(cache_misses);
          DBG_PRINT(cache_hits);
          DBG_PRINT(100.0f * cache_hits / cache_accesses);
        }
#endif
    }
  };

};
