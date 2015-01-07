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

#include "common/default.h"

#if defined(DEBUG)
#define CACHE_STATS(x) x
#else
#define CACHE_STATS(x) 
#endif

#define CACHE_DBG(x) 

namespace embree
{
  
  /* alloc cache memory */
  static __forceinline float *alloc_tessellation_cache_mem(const size_t blocks)
  {
    return (float*)_mm_malloc(64 * blocks,64);
  }

  /* free cache memory */
  static __forceinline void free_tessellation_cache_mem(void *mem)
  {
    assert(mem);
    _mm_free(mem);
  }

 class MultipleReaderSingleWriterMutex
 {
 private:
   AtomicMutex writer_mtx;
   volatile int readers;

 public:

 MultipleReaderSingleWriterMutex() : readers(0) {
     assert(sizeof(MultipleReaderSingleWriterMutex) == 8);
   }

   __forceinline void reset()
   {
     readers = 0;
     writer_mtx.reset();
   }
   __forceinline void pause()
   {
#if !defined(__MIC__)
     _mm_pause(); 
     _mm_pause();
#else
     _mm_delay_32(128); 
#endif      
   }
    
   __forceinline void read_lock()
   {
     while(1)
       {
         atomic_add(&readers,1);
         if (likely(!writer_mtx.isLocked())) break;
         atomic_add(&readers,-1);
         while(writer_mtx.isLocked())
           pause();
       }
   }

   __forceinline void read_unlock()
   {
     atomic_add(&readers,-1);      
   }

   __forceinline void write_lock()
   {
     writer_mtx.lock();
     while(readers)
       pause();
   }

   __forceinline void write_unlock()
   {
     writer_mtx.unlock();
   }

   __forceinline void write_unlock_set_read_lock()
   {
     atomic_add(&readers,1);     
     writer_mtx.unlock();
   }

 };

#if defined(__MIC__)
 typedef unsigned int InputTagType;
#else
 typedef size_t InputTagType;
#endif

 // FIXME: must be the same of all, move outside class
 static __forceinline unsigned int toTag(InputTagType prim)
 {
#if defined(__MIC__)
   return prim;
#else
   return prim / 320; // (((size_t)prim) >> 6);
#endif
 }



 class __aligned(32) TessellationCacheTag 
 {
 private:
   unsigned int prim_tag;
   unsigned int commit_tag;
   unsigned int usedBlocks;
   unsigned int access_timestamp;
   size_t       subtree_root;     
   MultipleReaderSingleWriterMutex mtx;

 public:

   __forceinline void read_lock()    { mtx.read_lock();   }
   __forceinline void read_unlock()  { mtx.read_unlock(); }
   __forceinline void write_lock()   { mtx.write_lock();   }
   __forceinline void write_unlock() { mtx.write_unlock(); }
   __forceinline void write_unlock_set_read_lock() { mtx.write_unlock_set_read_lock(); }


   __forceinline TessellationCacheTag() {}

   __forceinline void reset() 
   {
     assert(sizeof(TessellationCacheTag) == 32);
     prim_tag         = (unsigned int)-1;
     commit_tag       = (unsigned int)-1;
     usedBlocks       = 0;
     access_timestamp = 0;
     subtree_root     = 0;
     mtx.reset();    
   }

   __forceinline void clearRootRefBits()
   {
#if defined(__MIC__)
     /* bvh4i currently requires a different 'reset' */
     // FIXME
     if (subtree_root & ((size_t)1<<3))
       subtree_root &= ~(((size_t)1 << 4)-1);
     else
       subtree_root &= ~(((size_t)1 << 5)-1);
#else
     subtree_root &= ~(((size_t)1 << 4)-1);
#endif
   }

   __forceinline bool match(InputTagType primID, const unsigned int commitCounter)
   {
     return prim_tag == toTag(primID) && commit_tag == commitCounter;
   }

   __forceinline void set(const InputTagType primID, 
                          const unsigned int commitCounter,
                          const size_t root,
                          const unsigned int blocks)
   {
     prim_tag     = toTag(primID);
     commit_tag   = commitCounter;
     subtree_root = root;
     usedBlocks   = blocks;
   }

   __forceinline void update(const InputTagType primID, 
                             const unsigned int commitCounter)
   {
     prim_tag   = toTag(primID);
     commit_tag = commitCounter;
   }

   __forceinline void updateRootRef(const size_t root)
   {
     subtree_root = root;
   }

   /* update with NFU replacement policy */
   __forceinline void updateNFUStat()
   {
     access_timestamp >>= 1;
   }
   __forceinline void markAsMRU()
   {
     access_timestamp |= (unsigned int)1 << 31;
   }


   __forceinline size_t       &getRootRef()              { return subtree_root;     }
   __forceinline unsigned int getNumBlocks() const       { return usedBlocks;       }
   __forceinline unsigned int getPrimTag()   const       { return prim_tag;         }
   __forceinline unsigned int getCommitTag() const       { return commit_tag;       }
   __forceinline unsigned int getAccessTimeStamp() const { return access_timestamp; }


   __forceinline MultipleReaderSingleWriterMutex* getMutexPtr()     { return &mtx;              }
   

   __forceinline bool empty() const { return prim_tag == (unsigned int)-1; }

   __forceinline void print() {
     std::cout << "prim_tag " << prim_tag << " commit_tag " << commit_tag << " blocks " << usedBlocks << " subtree_root " << subtree_root << " ptr "<< getPtr() << " access time stamp " << access_timestamp << std::endl;
   }

   __forceinline void *getPtr() const
   {
     //FIXME: bvh4i
     return (void *)(subtree_root & (~(((size_t)1 << 4)-1)));
   }    

 };

 class SharedTessellationCacheStats
 {
 public:
    /* stats */
   static AtomicCounter cache_accesses;
   static AtomicCounter cache_hits;
   static AtomicCounter cache_misses;
   static AtomicCounter cache_evictions;                

    /* print stats for debugging */                 
    static void printStats();
    static void clearStats();

 };

 template<size_t CACHE_ENTRIES>
  class __aligned(64) SharedTessellationCache {

  private:
    TessellationCacheTag tags[CACHE_ENTRIES];            

  public:

  __forceinline unsigned int getNumBlocks() 
  {
    unsigned int b = 0;
    for (size_t i=0;i<CACHE_ENTRIES;i++) 
      b += tags[i].getNumBlocks();
    return b;
  }

      
  __forceinline void print() {
    std::cout << "CACHE-TAGS:" << std::endl;
    for (size_t i=0;i<SharedTessellationCache::CACHE_ENTRIES;i++)
      {
        std::cout << "i = " << i << " -> ";
        tags[i].print();
      }
  }
      

    /* reset cache */
    __forceinline void clear()
    {
      for (size_t i=0;i<CACHE_ENTRIES;i++)
        tags[i].reset();
    }
    

    SharedTessellationCache()  
      {
        clear();
      }

    __forceinline unsigned int allocated64ByteBlocks() 
    {
      unsigned int b = 0;
      for (size_t i=0;i<CACHE_ENTRIES;i++)
        b += tags[i].getNumBlocks();
      return b;
    }

    /* lookup cache entry using 64bit pointer as tag */
    __forceinline TessellationCacheTag *getTag(InputTagType primID)
    {
      CACHE_DBG(PING);
      /* direct mapped */
      const size_t t = toTag(primID) % CACHE_ENTRIES;
      return &tags[t];
    }

  };


  // =========================================================================================================
  // =========================================================================================================
  // =========================================================================================================

 class DistributedTessellationCacheStats
 {
 public:
    /* stats */
   static AtomicCounter cache_accesses;
   static AtomicCounter cache_hits;
   static AtomicCounter cache_misses;
   static AtomicCounter cache_evictions;                

    /* print stats for debugging */                 
    static void printStats();
    static void clearStats();

 };


  class __aligned(64) AdaptiveTessellationCache {
  public:
    /* default sizes */

    static const size_t CACHE_ENTRIES = 256; 
    static const size_t CACHE_WAYS    = 4;  // 4-way associative
    static const size_t CACHE_SETS    = CACHE_ENTRIES / CACHE_WAYS; 

  public:

    static const size_t CACHE_MISS = (size_t)-1;



    class CacheTagSet {
    public:
      TessellationCacheTag tags[CACHE_WAYS];

      __forceinline TessellationCacheTag &getCacheTagAndUpdateNFU(size_t index)
      {
        assert(index < CACHE_WAYS);
        for (size_t i=0;i<CACHE_WAYS;i++)
          tags[i].updateNFUStat();

        tags[index].markAsMRU();

        return tags[index];
      }

      __forceinline size_t lookup(InputTagType primID, const unsigned int commitCounter)
      {
        for (size_t i=0;i<CACHE_WAYS;i++)
          if (tags[i].match(primID,commitCounter))
            return i;
        return CACHE_MISS;
      };


      __forceinline void reset() 
      {
        for (size_t i=0;i<CACHE_WAYS;i++) tags[i].reset();
      }

      __forceinline unsigned int getNumBlocks() 
      {
        unsigned int b = 0;
        for (size_t i=0;i<CACHE_WAYS;i++) 
          b += tags[i].getNumBlocks();
        return b;
      }

      __forceinline TessellationCacheTag& getEvictionCandidate(const unsigned int neededBlocks)
      {
        /* fill empty slots first */
        for (size_t i=0;i<CACHE_WAYS;i++)
          if (tags[i].empty()) 
            {
              return getCacheTagAndUpdateNFU(i);
            }

        unsigned int min_access_timestamp = (unsigned int)-1;

        //if (tags[i].blocks() >= neededBlocks)

        /* use NFU replacement policy */
        size_t index = (size_t)-1;
        for (size_t i=0;i<CACHE_WAYS;i++)          
          {
            if (tags[i].getAccessTimeStamp() < min_access_timestamp)
              {
                min_access_timestamp = tags[i].getAccessTimeStamp();
                index = i;
              }
          }
        assert(index != (size_t)-1);

        /* update NFU status */
        return getCacheTagAndUpdateNFU(index);
      }
      
      __forceinline void print() {
        std::cout << "CACHE-TAG-SET:" << std::endl;
        for (size_t i=0;i<AdaptiveTessellationCache::CACHE_WAYS;i++)
          {
            std::cout << "i = " << i << " -> ";
            tags[i].print();
          }
      }
      
    };

  private:
    CacheTagSet sets[CACHE_SETS];
                    
    __forceinline size_t addrToCacheSetIndex(InputTagType primID)
    {      
      const size_t cache_set = toTag(primID) % CACHE_SETS;
      return cache_set;
    }

    /* reset cache */
    __forceinline void clear()
    {
      for (size_t i=0;i<CACHE_SETS;i++)
        sets[i].reset();
    }


  public:

    __forceinline void print()
    {
      for (size_t i=0;i<CACHE_SETS;i++)
        sets[i].print();
    }


    AdaptiveTessellationCache()  
      {
      }

    /* initialize cache */
    void init()
    {
      clear();
    }

    __forceinline unsigned int allocated64ByteBlocks() 
    {
      unsigned int b = 0;
      for (size_t i=0;i<CACHE_SETS;i++)
        b += sets[i].getNumBlocks();
      return b;
    }

    /* lookup cache entry using 64bit pointer as tag */
    __forceinline size_t lookup(InputTagType primID, const unsigned int commitCounter)
    {
      CACHE_DBG(PING);
      /* direct mapped */
      const size_t set = addrToCacheSetIndex(primID);
      assert(set < CACHE_SETS);
      const size_t index = sets[set].lookup(primID,commitCounter);

      CACHE_DBG(
                DBG_PRINT( index == CACHE_MISS );

                DBG_PRINT(primID);
                DBG_PRINT(toTag(primID));
                DBG_PRINT(set);

                DBG_PRINT(index);
                sets[set].print();
                );

      if (unlikely(index == CACHE_MISS)) 
        {
          return CACHE_MISS;
        }

      TessellationCacheTag &t = sets[set].getCacheTagAndUpdateNFU(index);

      assert( t.match(primID,commitCounter) );
      return t.getRootRef();
    }
    
    /* insert entry using 'neededBlocks' cachelines into cache */
    __forceinline TessellationCacheTag &request(InputTagType primID, 
                                                const unsigned int commitCounter, 
                                                const size_t neededBlocks)
    {
      CACHE_DBG(PING);
      const size_t set = addrToCacheSetIndex(primID);
      CACHE_DBG(DBG_PRINT(set));
      
      TessellationCacheTag &t = sets[set].getEvictionCandidate(neededBlocks);
      
      assert( t.getAccessTimeStamp() & ((unsigned int)1 << 31));
      assert(!t.match(primID,commitCounter));

      if (!t.empty() && t.getNumBlocks() >= neededBlocks)
        {
          assert(t.getNumBlocks() >= neededBlocks);
	  CACHE_DBG(DBG_PRINT("EVICT"));
          t.clearRootRefBits();
          t.update(primID,commitCounter);
          return t;
        }

      /* allocate entry */
      CACHE_DBG(DBG_PRINT("NEW ALLOC"));

      CACHE_DBG(DBG_PRINT(set));
      CACHE_DBG(DBG_PRINT(t.getPtr()));
      
      if (t.getPtr() != NULL)
        {
          assert(t.getNumBlocks() != 0);
          assert(t.getPrimTag() != (unsigned int)-1);
          CACHE_DBG(DBG_PRINT(t.getPtr()));
          free_tessellation_cache_mem(t.getPtr());
        }
      else
        {
          assert(t.getNumBlocks() == 0);
          assert(t.getPrimTag() == (unsigned int)-1);
        }
      float *new_mem = alloc_tessellation_cache_mem(neededBlocks);
      CACHE_DBG(DBG_PRINT(new_mem));

      /* insert new entry at the beginning */
      CACHE_DBG(t.print());
      t.set(primID,commitCounter,(size_t)new_mem,neededBlocks);
      CACHE_DBG(sets[set].print());
      return t;     
    }

  };


  // =========================================================================================================
  // =========================================================================================================
  // =========================================================================================================


  class __aligned(64) TessellationCache {
  public:
    /* default sizes */
#if defined(__MIC__)
    static const size_t DEFAULT_64B_BLOCKS = ((size_t)1<<15); // 2MB 
#else
    static const size_t DEFAULT_64B_BLOCKS = ((size_t)1<<17); // 8MB 
#endif

    static const size_t MAX_64B_BLOCKS     = (1<<19); // 32MB

    static const size_t CACHE_SETS = 1<<11; // 2048 sets
    static const size_t CACHE_WAYS = 1<<2;  // 4-way associative

  public:

    static const size_t CACHE_MISS = (size_t)-1;

  class __aligned(16) CacheTag {
  public:
    unsigned int prim_tag;
    unsigned int commit_tag;
    unsigned int usedBlocks;
    unsigned int subtree_root;     

  public:

    __forceinline void reset() 
    {
      assert(sizeof(CacheTag) == 16);
      prim_tag     = (unsigned int)-1;
      commit_tag   = (unsigned int)-1;
      subtree_root = (unsigned int)-1;
      usedBlocks   = 0;
    }

    __forceinline bool match(InputTagType primID, const unsigned int commitCounter)
    {
      return prim_tag == toTag(primID) && commit_tag == commitCounter;
    }

    __forceinline void set(InputTagType primID, 
                           const unsigned int commitCounter,
                           const unsigned int root32bit,
                           const unsigned int blocks)
    {
      prim_tag     = toTag(primID);
      commit_tag   = commitCounter;
      subtree_root = root32bit;
      usedBlocks   = blocks;
    }

    __forceinline void update(InputTagType primID, 
                              const unsigned int commitCounter)
    {
      prim_tag   = toTag(primID);
      commit_tag = commitCounter;
    }

    __forceinline void updateRootRef(const unsigned int root32bit)
    {
      subtree_root = root32bit;
    }

    __forceinline unsigned int getRootRef() const
    {
      return subtree_root;
    }

    __forceinline void clearRootRefBits()
    {
#if defined(__MIC__)
      /* bvh4i currently requires a different 'reset' */
      // FIXME
      if (subtree_root & ((unsigned int)1<<3))
        subtree_root >>= 4;
      else
        subtree_root >>= 4+1;
#else
      subtree_root &= ~(((unsigned int)1 << 4)-1);
#endif
    }

    __forceinline unsigned int blocks() const
    {
      return usedBlocks;
    }

    __forceinline bool empty() const
    {
      return prim_tag == (unsigned int)-1;
    }

    __forceinline void print() {
      std::cout << "prim_tag " << prim_tag << " commit_tag " << commit_tag << " blocks " << usedBlocks << " subtree_root " << subtree_root << std::endl;
    }

  };


    class CacheTagSet {
    public:
      CacheTag tags[CACHE_WAYS];

      __forceinline CacheTag &getCacheTag(size_t index)
      {
        assert(index < CACHE_WAYS);
        return tags[index];
      }

      __forceinline void moveToFront(const size_t index)
      {
        CACHE_DBG(PING);
        CacheTag tmp = tags[index];
        for (ssize_t i = index-1;i>=0;i--)
          tags[i+1] = tags[i];
        tags[0] = tmp;
      }

      __forceinline size_t lookup(InputTagType primID, const unsigned int commitCounter)
      {
        for (size_t i=0;i<CACHE_WAYS;i++)
          if (tags[i].match(primID,commitCounter))
            {
              moveToFront(i);
              return 0;
            }
        return CACHE_MISS;
      };


      __forceinline void reset() 
      {
        for (size_t i=0;i<CACHE_WAYS;i++) tags[i].reset();
      }

      __forceinline size_t getEvictionCandidate(const unsigned int neededBlocks)
      {
        /* fill empty slots first */
        if (unlikely(tags[CACHE_WAYS-1].empty())) return (size_t)-1;

        for (ssize_t i=CACHE_WAYS-1;i>=0;i--)
          if (tags[i].blocks() >= neededBlocks)
            return i;
        return (size_t)-1;
      }
      
      __forceinline CacheTag &getLRU()
      {
        moveToFront(CACHE_WAYS-1);
        return tags[0];
      }

      __forceinline void print() {
        std::cout << "CACHE-TAG-SET:" << std::endl;
        for (size_t i=0;i<TessellationCache::CACHE_WAYS;i++)
          {
            std::cout << "i = " << i << " -> ";
            tags[i].print();
          }
      }
      
    };

  private:
    CacheTagSet sets[CACHE_SETS];


    /* allocated memory to store cache entries */
    float *lazymem;

    /* total number of allocated 64bytes blocks */
    size_t allocated64BytesBlocks;

    /* current number of allocated blocks */
    size_t blockCounter;


    /* stats */
    static AtomicCounter cache_accesses;
    static AtomicCounter cache_hits;
    static AtomicCounter cache_misses;
    static AtomicCounter cache_clears;
    static AtomicCounter cache_evictions;                
                    
    __forceinline size_t addrToCacheSetIndex(InputTagType primID)
    {      
      const size_t cache_set = toTag(primID) % CACHE_SETS;
      return cache_set;
    }

    /* reset cache */
    __forceinline void clear()
    {
      CACHE_STATS(cache_clears++);
      blockCounter =  0;	  
      for (size_t i=0;i<CACHE_SETS;i++)
        sets[i].reset();
    }

  public:

    __forceinline float *getPtr() const
    {
      return lazymem;
    }


    TessellationCache()  
      {
      }

    /* initialize cache */
    void init()
    {
      clear();
      allocated64BytesBlocks = DEFAULT_64B_BLOCKS;	
      lazymem = alloc_tessellation_cache_mem( allocated64BytesBlocks );
      assert((size_t)lazymem % 64 == 0);
    }

    __forceinline unsigned int allocated64ByteBlocks() 
    {
      return allocated64BytesBlocks;
    }

    /* lookup cache entry using 64bit pointer as tag */
    __forceinline size_t lookup(InputTagType primID, const unsigned int commitCounter)
    {
      CACHE_STATS(cache_accesses++);
      CACHE_DBG(PING);
      /* direct mapped */
      const size_t set = addrToCacheSetIndex(primID);
      assert(set < CACHE_SETS);
      const size_t index = sets[set].lookup(primID,commitCounter);
      CACHE_DBG(
                DBG_PRINT( index == CACHE_MISS );

                DBG_PRINT(primID);
                DBG_PRINT(toTag(primID));
                DBG_PRINT(set);

                DBG_PRINT(index);
                sets[set].print();
                );

      if (unlikely(index == CACHE_MISS)) 
        {
          CACHE_STATS(cache_misses++);
          return CACHE_MISS;
        }
      CACHE_STATS(cache_hits++);

      CacheTag &t = sets[set].getCacheTag(index);

      assert( t.match(primID,commitCounter) );
#if defined(__MIC__)
      return t.getRootRef();
#else
      return (size_t)getPtr() + t.getRootRef();
#endif
    }

    /* insert entry using 'neededBlocks' cachelines into cache */
    __forceinline CacheTag &request(InputTagType primID, 
                                    const unsigned int commitCounter, 
                                    const size_t neededBlocks)
    {
      CACHE_DBG(PING);
      const size_t set = addrToCacheSetIndex(primID);
      CACHE_DBG(DBG_PRINT(set));
      const size_t eviction_index = sets[set].getEvictionCandidate(neededBlocks);
      CACHE_DBG(DBG_PRINT(eviction_index));

      if (eviction_index != (size_t)-1)
        {
          CacheTag &t = sets[set].getCacheTag(eviction_index);
          assert(t.blocks() >= neededBlocks);
	  CACHE_DBG(DBG_PRINT("EVICT"));
          CACHE_STATS(cache_evictions++);
          t.update(primID,commitCounter);
          t.clearRootRefBits();
          return t;
        }

      /* not enough space to hold entry? */
      if (unlikely(blockCounter + neededBlocks >= allocated64BytesBlocks))
        {   
          const unsigned int new_allocated64BytesBlocks = max(2*allocated64BytesBlocks,2*neededBlocks);
	  if (new_allocated64BytesBlocks <= MAX_64B_BLOCKS)
            {
#if DEBUG
              std::cout << "EXTENDING TESSELLATION CACHE (PER THREAD) FROM " 
                        << allocated64BytesBlocks << " TO " 
                        << new_allocated64BytesBlocks << " BLOCKS = " 
                        << new_allocated64BytesBlocks*64 << " BYTES" << std::endl << std::flush;
#endif
              free_tessellation_cache_mem(lazymem);
              allocated64BytesBlocks = new_allocated64BytesBlocks; 
              lazymem = alloc_tessellation_cache_mem(allocated64BytesBlocks);
              assert(lazymem);
            }
	  
	  if (unlikely(new_allocated64BytesBlocks < neededBlocks))
	    FATAL("can't enlarge tessellation cache to handle " << neededBlocks << ", increase maximum size limit");

          clear();
        }

      CACHE_DBG(DBG_PRINT("NEW ALLOC"));
	  
      /* allocate entry */
      const size_t currentIndex = blockCounter;
      blockCounter += neededBlocks;
      assert(blockCounter <= allocated64BytesBlocks);

#if defined(__MIC__)
      unsigned int curNode = currentIndex;
#else
      size_t curNode = (size_t)&lazymem[currentIndex*16] - (size_t)getPtr(); //(size_t)&lazymem[currentIndex*16];
#endif
      /* insert new entry at the beginning */
      CACHE_DBG(DBG_PRINT(set));
      CacheTag &t = sets[set].getLRU();      
      CACHE_DBG(t.print());
      t.set(primID,commitCounter,curNode,neededBlocks);
      CACHE_DBG(sets[set].print());
      return t;     
    }

    __forceinline char *getCacheMemoryPtr(const CacheTag &t) const
    {
      return (char*)((char*)getPtr() + t.getRootRef());
    }

#if defined(__MIC__)
    __forceinline void updateRootRef(CacheTag &t, unsigned int new_root)
    {
      t.updateRootRef( new_root );      
    }
#else
    __forceinline void updateRootRef(CacheTag &t, size_t new_root)
    {
      t.updateRootRef( new_root - (size_t)getPtr() );      
    }
#endif    
    
    /* print stats for debugging */                 
    static void printStats();
    static void clearStats();

  };




};
