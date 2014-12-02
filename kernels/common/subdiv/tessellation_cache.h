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
#include <bitset>

namespace embree
{
  // FIXME: implement 4 or 8 way associative cache on Xeon using ssei or avxi
  
  class __aligned(64) TessellationCache {

  private:
    /* default sizes */
    static const size_t DEFAULT_64B_BLOCKS = (1<<14);
    static const size_t CACHE_ENTRIES      = DEFAULT_64B_BLOCKS / 4;

    struct CacheTag {
      void *prim_tag;
      int commit_tag;
      int usedBlocks;
      BVH4::NodeRef bvh4_subtree_root;     

      __forceinline void reset() 
      {
        prim_tag          = NULL;
        commit_tag        = -1;
        bvh4_subtree_root = 0;
        usedBlocks        = 0;
      }

      __forceinline bool match(void *primID, const unsigned int commitCounter)
      {
        return prim_tag == primID && commit_tag == commitCounter;
      }

      __forceinline void set(void *primID, 
                             const unsigned int commitCounter,
                             BVH4::NodeRef root,
                             const unsigned int blocks)
      {
        prim_tag          = primID;
        commit_tag        = commitCounter;
        bvh4_subtree_root = root;
        usedBlocks        = blocks;
      }

      __forceinline void set(void *primID, 
                             const unsigned int commitCounter)
      {
        prim_tag          = primID;
        commit_tag        = commitCounter;
      }

    } tags[CACHE_ENTRIES];


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
    size_t cache_clears;
    size_t cache_evictions;
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
#if DEBUG
      cache_clears++;
#endif
      blockCounter =  0;	  
      for (size_t i=0;i<CACHE_ENTRIES;i++)
        tags[i].reset();
    }

    __forceinline float *getPtr()
    {
      return lazymem;
    }


    TessellationCache()  
      {
      }

    /* initialize cache */
    __forceinline void init()
    {

      clear();
      allocated64BytesBlocks = DEFAULT_64B_BLOCKS;	
      lazymem = alloc_mem( allocated64BytesBlocks );
      assert((size_t)lazymem % 64 == 0);

#if DEBUG
      cache_accesses  = 0;
      cache_hits      = 0;
      cache_misses    = 0;
      cache_clears    = 0;
      cache_evictions = 0;
#endif

    }

    __forceinline unsigned int addrToCacheIndex(void *primAddr)
    {
      return (((size_t)primAddr)>>6) % CACHE_ENTRIES;
    }

    /* lookup cache entry using 64bit pointer as tag */
    __forceinline BVH4::NodeRef lookup(void *primID, const unsigned int commitCounter)
    {

#if DEBUG
      cache_accesses++;
#endif
      
      /* direct mapped */
      const unsigned int index = addrToCacheIndex(primID);
      if (likely(tags[index].match(primID,commitCounter)))
        {
#if DEBUG
          cache_hits++;
#endif
          return tags[index].bvh4_subtree_root;
        }

#if DEBUG
      cache_misses++;
#endif

      return BVH4::invalidNode;
    }

    /* insert entry using 'neededBlocks' cachelines into cache */
    __forceinline BVH4::NodeRef insert(void *primID, const unsigned int commitCounter, const size_t neededBlocks)
    {

      const unsigned int index = addrToCacheIndex(primID);
      assert(!tags[index].match(primID,commitCounter));
      if (tags[index].usedBlocks >= neededBlocks)
        {
#if DEBUG
          if (tags[index].prim_tag != NULL) cache_evictions++;
#endif

          tags[index].set(primID,commitCounter);
          return tags[index].bvh4_subtree_root;
        }


      /* not enough space to hold entry? */
      if (unlikely(blockCounter + neededBlocks >= allocated64BytesBlocks))
        {          
          /* can the cache hold this subtree space at all in each cache entries? */
          if (unlikely(CACHE_ENTRIES*neededBlocks > allocated64BytesBlocks)) 
            {
              const unsigned int new_allocated64BytesBlocks = CACHE_ENTRIES*neededBlocks;

              std::cout << "EXTENDING TESSELLATION CACHE (PER THREAD) FROM " 
                        << allocated64BytesBlocks << " TO " 
                        << new_allocated64BytesBlocks << " BLOCKS = " 
                        << new_allocated64BytesBlocks*64 << " BYTES" << std::endl << std::flush;

              free_mem(lazymem);
              allocated64BytesBlocks = new_allocated64BytesBlocks; 
              lazymem = alloc_mem(allocated64BytesBlocks);
              assert(lazymem);

            }
          //std::cout << "FLUSH" << std::endl;
          /* realloc */
          clear();
        }

      /* allocate entry */
      const size_t currentIndex = blockCounter;
      blockCounter += neededBlocks;

      BVH4::NodeRef curNode = BVH4::encodeNode( (BVH4::Node*)&lazymem[currentIndex*16] );

      /* insert new entry at the beginning */
      tags[index].set(primID,commitCounter,curNode,neededBlocks);
      return curNode;     
    }

    /* print stats for debugging */                 
    void printStats() const
    {
#if DEBUG
      //if (cache_accesses)
        {
          assert(cache_hits + cache_misses == cache_accesses);
          DBG_PRINT(cache_accesses);
          DBG_PRINT(cache_misses);
          DBG_PRINT(cache_hits);
          DBG_PRINT(100.0f * cache_hits / cache_accesses);
          DBG_PRINT(cache_clears);
        }
#endif
    }
  };

};
