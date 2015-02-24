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

#pragma once

#include "common/default.h"

#define CACHE_DBG(x) 

namespace embree
{
  
  /* alloc cache memory */
  float *alloc_tessellation_cache_mem(const size_t blocks);

  /* free cache memory */
  void free_tessellation_cache_mem(void *mem, const size_t blocks = 0);


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

 ////////////////////////////////////////////////////////////////////////////////
 ////////////////////////////////////////////////////////////////////////////////
 ////////////////////////////////////////////////////////////////////////////////


  struct __aligned(32) TessellationRefCacheTag {
   unsigned int prim_tag;
   unsigned int commit_tag;
   size_t subtree_root;
   void  *patch_ptr;
   size_t dummy;

   __forceinline void set(const InputTagType primID, 
                          const unsigned int commitCounter,
                          const size_t root,
			  void *ptr = NULL)
   {
     prim_tag     = toTag(primID);
     commit_tag   = commitCounter;
     subtree_root = root;
     patch_ptr    = ptr;
     dummy        = 0;
   }

   __forceinline void reset() 
   {
     assert(sizeof(TessellationRefCacheTag) == 32);
     prim_tag         = (unsigned int)-1;
     commit_tag       = (unsigned int)-1;
     subtree_root     = 0;
     patch_ptr        = NULL;
     dummy            = NULL;
   }

   __forceinline bool match(InputTagType primID, const unsigned int commitCounter)
   {
     return prim_tag == toTag(primID) && commit_tag == commitCounter;
   }

   __forceinline unsigned int getPrimTag()    const { return prim_tag;     }
   __forceinline unsigned int getCommitTag()  const { return commit_tag;   }
   __forceinline size_t       getRootRef()    const { return subtree_root; }
   __forceinline void        *getPatchPtr()   const { return patch_ptr;    }

   __forceinline bool         empty()        const { return prim_tag == (unsigned int)-1; }


  };

  template<size_t CACHE_ENTRIES>
  class __aligned(64) TessellationRefCacheT {

    static const size_t CACHE_WAYS    = 8;  
    static const size_t CACHE_SETS    = CACHE_ENTRIES / CACHE_WAYS; 

    TessellationRefCacheTag tags[CACHE_ENTRIES];

    class __aligned(4 * sizeof(CACHE_WAYS)) CacheTagSet {
    public:
      unsigned int NFU_stat[CACHE_WAYS];
      
      __forceinline void reset()
      {
	for (size_t i=0;i<CACHE_WAYS;i++)
	  NFU_stat[i] = 0;	
      }

      __forceinline void updateNFUStat()
      {
	for (size_t i=0;i<CACHE_WAYS;i++)
	  NFU_stat[i] >>= 1;	
      }

      __forceinline void markMRU(const size_t index)
      {
	NFU_stat[index] |= (unsigned int)1 << 31;
      }
      __forceinline size_t getNFUIndex()
      {
	size_t index = 0;
	for (size_t i=0;i<CACHE_WAYS;i++)
	  if (NFU_stat[i] < NFU_stat[index])
	    index = i;
	assert(index < CACHE_WAYS);
	return index;
      }
    };

    CacheTagSet sets[CACHE_SETS];
    float *scratch_mem;
    size_t scratch_mem_blocks;

  public:


    __forceinline void reset()
    {
      for (size_t i=0;i<CACHE_ENTRIES;i++)
        tags[i].reset(); 

      for (size_t i=0;i<CACHE_SETS;i++)
	sets[i].reset();
    }

    __forceinline TessellationRefCacheTag *lookUpTag(InputTagType primID,
						     const unsigned int commitCounter)
    {
#if 0 
      /* direct mapped */
      const size_t t = toTag(primID) % CACHE_ENTRIES;
      return &tags[t];
#else
      const size_t set_index = toTag(primID) % CACHE_SETS;
      TessellationRefCacheTag *set_tags = &tags[set_index*CACHE_WAYS];
      for (size_t i=0;i<CACHE_WAYS;i++)
	if (set_tags[i].match(primID,commitCounter))
	  {
	    /* cache hit */
	    sets[set_index].updateNFUStat();
	    sets[set_index].markMRU(i);
	    return &set_tags[i];
	  }
      /* cache miss */
      size_t evict_index = sets[set_index].getNFUIndex();
      sets[set_index].markMRU(evict_index);
      return &set_tags[evict_index];
#endif
    }

    __forceinline TessellationRefCacheT(const size_t scratch_mem_blocks) : scratch_mem_blocks(scratch_mem_blocks)
    {
      scratch_mem = (float*) alloc_tessellation_cache_mem(scratch_mem_blocks);
      reset();
    }

    __forceinline float *getScratchMemPtr()    { return scratch_mem; }
    __forceinline size_t getScratchMemBlocks() { return scratch_mem_blocks; }

    __forceinline void reallocScratchMem(size_t new_blocks) {
      assert(scratch_mem);
      scratch_mem_blocks = new_blocks;
      free_tessellation_cache_mem(scratch_mem);
      scratch_mem = alloc_tessellation_cache_mem(scratch_mem_blocks);
    }

  };



 //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
 //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
 //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
 //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


 class __aligned(32) TessellationCacheTag 
 {
 public:
   //typedef MultipleReaderSingleWriterMutex MutexType;
   typedef RWMutex MutexType;

 private:
   unsigned int prim_tag;
   unsigned int commit_tag;
   unsigned int usedBlocks;
   unsigned int access_timestamp;
   size_t       subtree_root;     
   RWMutex mtx;

 public:

   __forceinline void read_lock()    { mtx.read_lock();   }
   __forceinline void read_unlock()  { mtx.read_unlock(); }
   __forceinline void write_lock()   { mtx.write_lock();   }
   __forceinline void write_unlock() { mtx.write_unlock(); }
   __forceinline void upgrade_write_to_read_lock() { mtx.upgrade_write_to_read_lock(); }
   __forceinline void upgrade_read_to_write_lock() { mtx.upgrade_read_to_write_lock(); }
   __forceinline bool try_read_lock()  { return mtx.try_read_lock();   }
   __forceinline bool try_write_lock() { return mtx.try_write_lock();  }


   __forceinline TessellationCacheTag() {}

   __forceinline void reset(const size_t pre_alloc_blocks = 0) 
   {
     assert(sizeof(TessellationCacheTag) == 32);
     prim_tag         = (unsigned int)-1;
     commit_tag       = (unsigned int)-1;
     usedBlocks       = 0;
     access_timestamp = 0;
     subtree_root     = 0;
     mtx.reset();    

     if (pre_alloc_blocks != 0)
       {
	 float *mem   = alloc_tessellation_cache_mem(pre_alloc_blocks);
	 usedBlocks   = pre_alloc_blocks;
	 subtree_root = (size_t)mem;
	 memset(mem,0,64*pre_alloc_blocks);
       }
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

   __forceinline size_t       getRootRef()               { return subtree_root;     }
   __forceinline unsigned int getNumBlocks() const       { return usedBlocks;       }
   __forceinline unsigned int getPrimTag()   const       { return prim_tag;         }
   __forceinline unsigned int getCommitTag() const       { return commit_tag;       }
   __forceinline unsigned int getAccessTimeStamp() const { return access_timestamp; }


   __forceinline MutexType* getMutexPtr()     { return (MutexType*)&mtx;              }
   

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
    for (size_t i=0;i<CACHE_ENTRIES;i++)
      {
        std::cout << "i = " << i << " -> ";
        tags[i].print();
      }
  }
      

    /* reset cache */
    __forceinline void reset()
    {
      for (size_t i=0;i<CACHE_ENTRIES;i++)
        tags[i].reset();
    }
    

    SharedTessellationCache()  
      {
        reset();
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

 template<size_t CACHE_ENTRIES>
  class __aligned(64) AdaptiveTessellationCache {
  public:
    /* default sizes */

    static const size_t CACHE_WAYS    = 4;  // 4-way associative
    static const size_t CACHE_SETS    = CACHE_ENTRIES / CACHE_WAYS; 

  public:

    class CacheTagSet {
    public:
      TessellationCacheTag tags[CACHE_WAYS];

      __forceinline void updateNFU_markMRU(TessellationCacheTag *t)
      {
        for (size_t i=0;i<CACHE_WAYS;i++)
          tags[i].updateNFUStat();

        t->markAsMRU();
      }


      __forceinline TessellationCacheTag *lookup(InputTagType primID, const unsigned int commitCounter)
      {
        for (size_t i=0;i<CACHE_WAYS;i++)
          if (tags[i].match(primID,commitCounter))
            return &tags[i];
        return NULL;
      };


      __forceinline void reset(const size_t pre_alloc_blocks = 0) 
      {
        for (size_t i=0;i<CACHE_WAYS;i++) tags[i].reset(pre_alloc_blocks);
      }

      __forceinline unsigned int getNumBlocks() 
      {
        unsigned int b = 0;
        for (size_t i=0;i<CACHE_WAYS;i++) 
          b += tags[i].getNumBlocks();
        return b;
      }

      __forceinline TessellationCacheTag* getEvictionCandidate(const unsigned int neededBlocks)
      {
        /* fill empty slots first */
        for (size_t i=0;i<CACHE_WAYS;i++)
          if (tags[i].empty()) 
            {
	      updateNFU_markMRU(&tags[i]);
	      return &tags[i];
            }

        unsigned int min_access_timestamp = (unsigned int)-1;


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
	updateNFU_markMRU(&tags[index]);
	return &tags[index];
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
    __forceinline void reset(const size_t pre_alloc_blocks)
    {
      for (size_t i=0;i<CACHE_SETS;i++)
        sets[i].reset(pre_alloc_blocks);
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
    void init(const size_t pre_alloc_blocks = 0)
    {
      reset(pre_alloc_blocks);
    }

    __forceinline unsigned int allocated64ByteBlocks() 
    {
      unsigned int b = 0;
      for (size_t i=0;i<CACHE_SETS;i++)
        b += sets[i].getNumBlocks();
      return b;
    }

    /* lookup cache entry using 64bit pointer as tag */
    __forceinline TessellationCacheTag *lookup(InputTagType primID, const unsigned int commitCounter)
    {
      CACHE_DBG(PING);
      /* direct mapped */
      const size_t set = addrToCacheSetIndex(primID);
      assert(set < CACHE_SETS);
      TessellationCacheTag *t = sets[set].lookup(primID,commitCounter);

      CACHE_DBG(
                DBG_PRINT( t == NULL );

                DBG_PRINT(primID);
                DBG_PRINT(toTag(primID));
                DBG_PRINT(set);

                DBG_PRINT(index);
                sets[set].print();
                );

      if (unlikely(t == NULL)) 
        {
          return NULL;
        }

      sets[set].updateNFU_markMRU(t);
      return t;
    }
    
    /* insert entry using 'neededBlocks' cachelines into cache */
    __forceinline TessellationCacheTag *request(InputTagType primID, 
                                                const unsigned int commitCounter, 
                                                const size_t neededBlocks)
    {
      CACHE_DBG(PING);
      const size_t set = addrToCacheSetIndex(primID);
      CACHE_DBG(DBG_PRINT(set));
      
      TessellationCacheTag *t = sets[set].getEvictionCandidate(neededBlocks);
      
      assert( t->getAccessTimeStamp() & ((unsigned int)1 << 31));
      assert(!t->match(primID,commitCounter));

#if 1
      if (/* !t->empty() && */ t->getNumBlocks() >= neededBlocks)
        {
          assert(t->getNumBlocks() >= neededBlocks);
	  CACHE_DBG(DBG_PRINT("EVICT"));
          t->clearRootRefBits();
          t->update(primID,commitCounter);
          return t;
        }
#endif
      /* allocate entry */
      CACHE_DBG(DBG_PRINT("NEW ALLOC"));

      CACHE_DBG(DBG_PRINT(set));
      CACHE_DBG(DBG_PRINT(t->getPtr()));
      
      if (t->getPtr() != NULL)
        {
          assert(t->getNumBlocks() != 0);
          CACHE_DBG(DBG_PRINT(t->getPtr()));
          free_tessellation_cache_mem(t->getPtr(),t->getNumBlocks());
        }
      else
        {
          assert(t->getNumBlocks() == 0);
          assert(t->getPrimTag() == (unsigned int)-1);
        }
      float *new_mem = alloc_tessellation_cache_mem(neededBlocks);
      CACHE_DBG(DBG_PRINT(new_mem));

      /* insert new entry at the beginning */
      CACHE_DBG(t->print());
      t->set(primID,commitCounter,(size_t)new_mem,neededBlocks);
      CACHE_DBG(sets[set].print());
      return t;     
    }

  };


  // =========================================================================================================
  // =========================================================================================================
  // =========================================================================================================

};
