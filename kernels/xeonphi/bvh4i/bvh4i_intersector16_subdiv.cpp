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

#include "bvh4i_intersector16_subdiv.h"
#include "bvh4i_leaf_intersector.h"
#include "geometry/subdivpatch1.h"
#include "common/subdiv/tessellation_cache.h"

#define TIMER(x) 

#if defined(DEBUG)
#define CACHE_STATS(x) 
#else
#define CACHE_STATS(x) 
#endif

#define SHARED_TESSELLATION_CACHE_ENTRIES 16*1024
#define LOCAL_TESSELLATION_CACHE_ENTRIES  32

#define TESSELLATION_REF_CACHE_ENTRIES  32
#define NUM_SCRATCH_MEM_BLOCKS 1024

//#define ONLY_SHARED_CACHE
#define CACHE_HIERARCHY

#define LAZY_BUILD 1

#define SHARED_LAZY_CACHE 1

#define SIZE_SHARED_LAZY_CACHE 200*1024*1024

namespace embree
{
  __aligned(64) AtomicCounter globalRenderThreads = 0;

  typedef AdaptiveTessellationCache<LOCAL_TESSELLATION_CACHE_ENTRIES> TessellationCache;
  SharedTessellationCache<SHARED_TESSELLATION_CACHE_ENTRIES> sharedTessellationCache;

  typedef TessellationRefCacheT<TESSELLATION_REF_CACHE_ENTRIES> TessellationRefCache;

  static double msec = 0.0;

  namespace isa
  {
    struct LocalThreadInfo
    {
      unsigned int id;

      LocalThreadInfo(const unsigned int id) : id(id) {}  
    };

    __thread LocalThreadInfo* localThreadInfo = NULL;



    class __aligned(64) SharedLazyTessellationCache 
    {
      static const size_t SIZE = SIZE_SHARED_LAZY_CACHE;
      static const size_t CLEAR_BLOCK_SIZE = 128;
    private:
      float *data;
      size_t maxBlocks;
      __aligned(64) AtomicCounter next_block;
      __aligned(64) AtomicMutex reset_state;
      __aligned(64) volatile size_t clear_index;

      struct __aligned(64) ThreadWorkState {
	RWMutex mtx;	
	ThreadWorkState() { mtx.reset(); }
      };

      __aligned(64) ThreadWorkState threadWorkState[MAX_MIC_THREADS];

    public:
      SharedLazyTessellationCache()
	{
	  data         = (float*)os_malloc(SIZE);
	  maxBlocks    = SIZE/64;
	  next_block   = 0;
	  clear_index  = 0;
	  reset_state.reset();   
	}

      __forceinline void readLockThread(const unsigned int threadID)
      {
	assert(threadWorkState[threadID].mtx.num_readers() == 0);
	threadWorkState[threadID].mtx.read_lock();
      }

      __forceinline void readUnlockThread(const unsigned int threadID)
      {
	assert(threadWorkState[threadID].mtx.num_readers() == 1);
	threadWorkState[threadID].mtx.read_unlock();
      }

      __forceinline void writeLockThread(const unsigned int threadID)
      {
	threadWorkState[threadID].mtx.write_lock();
      }

      __forceinline void writeUnlockThread(const unsigned int threadID)
      {
	threadWorkState[threadID].mtx.write_unlock();
      }

    
      __noinline void resetCache(SubdivPatch1* const patches,
				 const size_t numPatches) 
      {

#if 0

	while( next_block >= maxBlocks )
	  {
	    size_t start = 0;
	    size_t end   = 0;
	    
	    reset_state.lock();

	    if (next_block >= maxBlocks)
	      {
		if (clear_index == 0) msec = getSeconds();

		if (clear_index < numPatches)
		  {
		    start = clear_index;
		    end   = min(start+CLEAR_BLOCK_SIZE,numPatches);
		    assert(end <= numPatches);
		    clear_index = end;
		  }
	      }	  
	    reset_state.unlock();

	    for (size_t i=start;i<end;i++)
	      {
		prefetch<PFHINT_L1EX>(&patches[i+2]);
		prefetch<PFHINT_L2EX>(&patches[i+16]);
		patches[i].write_lock();
		patches[i].resetRootRef();
		patches[i].write_unlock();

	      }

	    if (end == numPatches)
	      {
		next_block = 0;
		clear_index = 0;
	    
		msec = getSeconds()-msec;    
		DBG_PRINT( 1000.0f * msec );
	      }
	  }

	

#else

	if (reset_state.try_lock())
	  {
	    if (next_block >= maxBlocks)
	      {
		msec = getSeconds();

		for (size_t i=0;i<globalRenderThreads;i++)
		  writeLockThread(i);

		for (size_t i=0;i<numPatches;i++)
		  {
		    prefetch<PFHINT_L1EX>(&patches[i+2]);
		    prefetch<PFHINT_L2EX>(&patches[i+16]);
		    patches[i].resetRootRef();
		  }

		for (size_t i=0;i<globalRenderThreads;i++)
		  writeUnlockThread(i);

		next_block = 0;

		msec = getSeconds()-msec;    
		DBG_PRINT( 1000.0f * msec );

	      }
	    reset_state.unlock();
	  }
	else
	  reset_state.wait_until_unlocked();	
#endif
      }

      __noinline size_t alloc(const size_t blocks)
      {
	size_t index = next_block.add(blocks);
	if (unlikely(index + blocks >= maxBlocks)) return (size_t)-1;
	return index;
      }

      __forceinline mic_f *getBlockPtr(const size_t block_index)
      {
	assert(block_index < maxBlocks);
	return (mic_f*)&data[block_index*16];
      }


      __forceinline size_t getNumAllocatedBytes() { 
	return next_block * 64; 
      }


      
    // __noinline void sync()
    // {      
    //   const unsigned int id = sync_threads.add(1);

    //   //DBG_PRINT(id);
    //   //DBG_PRINT(globalRenderThreads);

    //   if (id == 0)
    // 	{
    // 	  DBG_PRINT("WRITE LOCK ALL THREADS");
    // 	  for (size_t i=0;i<globalRenderThreads;i++)
    // 	    threadWorkState[i].mtx.write_lock();

    // 	  /* last thread */
    // 	  DBG_PRINT("CACHE FLUSH");
    // 	  resetCache();
    // 	  next_block = 0;
    // 	  sync_threads = 0;

    // 	  DBG_PRINT("WRITE UNLOCK ALL THREADS");
    // 	  for (size_t i=0;i<globalRenderThreads;i++)
    // 	    threadWorkState[i].mtx.write_lock();

    // 	  DBG_PRINT("DONE");
    // 	}

    //   while(sync_threads != 0);
    //   _mm_delay_32(256);
      
    //   //while(next_block != 0)
                  
    // }

    };

    SharedLazyTessellationCache sharedLazyTessellationCache;


    __thread TessellationRefCache *tess_ref_cache = NULL;

    __thread TessellationCache *tess_cache = NULL;

    static __forceinline void * extractBVH4iPtr(const size_t &subtree_root)
    {
      if (likely(subtree_root & ((size_t)1<<3)))
	return (void*)(subtree_root & ~(((size_t)1 << 4)-1));
      else
	return (void*)(subtree_root & ~(((size_t)1 << 5)-1));
    }

    static __forceinline BVH4i::NodeRef extractBVH4iNodeRef(const size_t &subtree_root)
    {
      if (likely(subtree_root & ((size_t)1<<3)))
	return (unsigned int)(subtree_root & (((size_t)1 << 4)-1));
      else
	return (unsigned int)(subtree_root & (((size_t)1 << 5)-1));
    }


    struct __aligned(64) Quad4x4 {
      mic3f vtx;
      unsigned short uu[16];
      unsigned short vv[16];

      __forceinline void prefetchData() const
      {
	prefetch<PFHINT_NT>(&vtx.x);
	prefetch<PFHINT_NT>(&vtx.y);
	prefetch<PFHINT_NT>(&vtx.z);
	prefetch<PFHINT_NT>(uu);
      }
      
      __forceinline mic_f getU() const
      {
	return load16f_uint16(uu);
      }

      __forceinline mic_f getV() const
      {
	return load16f_uint16(vv);
      }

      __forceinline void set(const mic3f &v3, const mic_f &u, const mic_f &v)
      {
#if 0
	vtx = v3;
	store16f_uint16(uu,u*65535.0f);
	store16f_uint16(vv,v*65535.0f);
#else
	__aligned(64) unsigned short local[32]; 
	store16f_uint16(local +  0,u*65535.0f);
	store16f_uint16(local + 16,v*65535.0f);
	store16f_ngo(&vtx.x,v3.x);
	store16f_ngo(&vtx.y,v3.y);
	store16f_ngo(&vtx.z,v3.z);
	store16f_ngo(&uu,load16f(local));
#endif
      }

    };

    __forceinline void createSubPatchBVH4iLeaf(BVH4i::NodeRef &ref,
					       const unsigned int patchIndex) 
    {
      *(volatile unsigned int*)&ref = (patchIndex << BVH4i::encodingBits) | BVH4i::leaf_mask;
    }


    BBox3fa createSubTree(BVH4i::NodeRef &curNode,
			  mic_f *const lazymem,
			  const SubdivPatch1 &patch,
			  const float *const grid_u_array,
			  const float *const grid_v_array,
			  const GridRange &range,
			  unsigned int &localCounter,
			  const SubdivMesh* const geom)
    {
      if (range.hasLeafSize())
	{
	  const unsigned int u_start = range.u_start;
	  const unsigned int u_end   = range.u_end;
	  const unsigned int v_start = range.v_start;
	  const unsigned int v_end   = range.v_end;

	  const unsigned int u_size = u_end-u_start+1;
	  const unsigned int v_size = v_end-v_start+1;
	  const unsigned int currentIndex = localCounter;
	  localCounter += 4; // x,y,z + 16bit u,v


	  mic_f uu = mic_f::inf();
	  mic_f vv = mic_f::inf();

#if 0
	  DBG_PRINT(u_start);
	  DBG_PRINT(u_end);
	  DBG_PRINT(v_start);
	  DBG_PRINT(v_end);

	  DBG_PRINT(u_size);
	  DBG_PRINT(v_size);
#endif

	  const size_t offset  = v_start * patch.grid_u_res + u_start;

	  /* fast path */
	  if (u_size == 4 && v_size == 4)
	    {
	      const size_t offset0 = offset + 0 * patch.grid_u_res;
	      const size_t offset1 = offset + 1 * patch.grid_u_res;
	      const size_t offset2 = offset + 2 * patch.grid_u_res;
	      const size_t offset3 = offset + 3 * patch.grid_u_res;
	      uu = gather16f_4f_unalign(&grid_u_array[offset0],&grid_u_array[offset1],&grid_u_array[offset2],&grid_u_array[offset3]);
	      vv = gather16f_4f_unalign(&grid_v_array[offset0],&grid_v_array[offset1],&grid_v_array[offset2],&grid_v_array[offset3]);
	    }
	  else
	    {
	      for (unsigned int v=0;v<v_size;v++)
		{
#pragma novector
		  for (unsigned int u=0;u<u_size;u++)
		    {
		      uu[4 * v + u] = grid_u_array[ offset + v * patch.grid_u_res + u ];
		      vv[4 * v + u] = grid_v_array[ offset + v * patch.grid_u_res + u ];
		    }
		}

	      /* set invalid grid u,v value to border elements */

	      for (unsigned int x=u_size-1;x<4;x++)
		for (unsigned int y=0;y<4;y++)
		  {
		    uu[4 * y + x] = uu[4 * y + u_size-1];
		    vv[4 * y + x] = vv[4 * y + u_size-1];
		  }

	      for (unsigned int y=v_size-1;y<4;y++)
		for (unsigned int x=0;x<4;x++)
		  {
		    uu[4 * y + x] = uu[4 * (v_size-1) + x];
		    vv[4 * y + x] = vv[4 * (v_size-1) + x];
		  }
	    }
	  
	  mic3f vtx = patch.eval16(uu,vv);

	  const Vec2f uv0 = patch.getUV(0);
	  const Vec2f uv1 = patch.getUV(1);
	  const Vec2f uv2 = patch.getUV(2);
	  const Vec2f uv3 = patch.getUV(3);

	  if (unlikely(geom->displFunc != NULL))
	    {
	      mic3f normal      = patch.normal16(uu,vv);
	      normal = normalize(normal);

	      const mic_f patch_uu = bilinear_interpolate(uv0.x,uv1.x,uv2.x,uv3.x,uu,vv);
	      const mic_f patch_vv = bilinear_interpolate(uv0.y,uv1.y,uv2.y,uv3.y,uu,vv);

	      geom->displFunc(geom->userPtr,
			      patch.geom,
			      patch.prim,
			      (const float*)&patch_uu,
			      (const float*)&patch_vv,
			      (const float*)&normal.x,
			      (const float*)&normal.y,
			      (const float*)&normal.z,
			      (float*)&vtx.x,
			      (float*)&vtx.y,
			      (float*)&vtx.z,
			      16);
	    }
	  const BBox3fa leafGridBounds = getBBox3fa(vtx);

	  Quad4x4 *quad4x4 = (Quad4x4*)&lazymem[currentIndex];
	  quad4x4->set(vtx,uu,vv);

	  createSubPatchBVH4iLeaf( curNode, currentIndex);	  
	  return leafGridBounds;
	}
      /* allocate new bvh4i node */
      const size_t num64BytesBlocksPerNode = 2;
      const size_t currentIndex = localCounter;
      localCounter += num64BytesBlocksPerNode;

      createBVH4iNode<2>(curNode,currentIndex);

      BVH4i::Node &node = *(BVH4i::Node*)curNode.node((BVH4i::Node*)lazymem);

      node.setInvalid();

      __aligned(64) GridRange r[4];
      prefetch<PFHINT_L1EX>(r);
      
      const unsigned int children = range.splitIntoSubRanges(r);
      
      /* create four subtrees */
      BBox3fa bounds( empty );
      for (unsigned int i=0;i<children;i++)
	{
	  BBox3fa bounds_subtree = createSubTree( node.child(i), 
						  lazymem, 
						  patch, 
						  grid_u_array,
						  grid_v_array,
						  r[i],
						  localCounter,
						  geom);
	  node.setBounds(i, bounds_subtree);
	  bounds.extend( bounds_subtree );
	}
      return bounds;      
    }

    BVH4i::NodeRef initLocalLazySubdivTree(const SubdivPatch1 &patch,
					   unsigned int currentIndex,
					   mic_f *lazymem,
					   const SubdivMesh* const geom)
    {
      TIMER(if (patch.grid_size_simd_blocks > 50) PING);
      TIMER(double msec = 0.0);
      TIMER(msec = getSeconds());

      __aligned(64) float u_array[(patch.grid_size_simd_blocks+1)*16]; // for unaligned access
      __aligned(64) float v_array[(patch.grid_size_simd_blocks+1)*16];

      gridUVTessellator(patch.level,
			patch.grid_u_res,
			patch.grid_v_res,
			u_array,
			v_array);

      /* stich different tessellation levels in u/v grid */
      if (patch.needsStiching())
	stichUVGrid(patch.level,patch.grid_u_res,patch.grid_v_res,u_array,v_array);

      TIMER(msec = getSeconds()-msec);    
      TIMER(if (patch.grid_size_simd_blocks > 50) DBG_PRINT(1000. * msec));
      TIMER(msec = getSeconds());

      BVH4i::NodeRef subtree_root = 0;
      const unsigned int oldIndex = currentIndex;

      BBox3fa bounds = createSubTree( subtree_root,
				      lazymem,
				      patch,
				      u_array,
				      v_array,
				      GridRange(0,patch.grid_u_res-1,0,patch.grid_v_res-1),
				      currentIndex,
				      geom);

      assert(currentIndex - oldIndex == patch.grid_subtree_size_64b_blocks);
      TIMER(msec = getSeconds()-msec);    
      return subtree_root;
    }


    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    void createTessellationCache()
    {
      const size_t blocks = (sizeof(TessellationCache)+sizeof(TessellationCache)-1)/64;
      TessellationCache *cache = (TessellationCache *)alloc_tessellation_cache_mem(blocks);
      assert( (size_t)cache % 64 == 0 );
      cache->init();	
      tess_cache = cache;
    }

    __noinline TessellationCacheTag *fillLocalTessellationCache(TessellationCache *local_cache,
								const unsigned int patchIndex,
								const unsigned int commitCounter,
								const SubdivPatch1* const patches,
								Scene *const scene)
    {
      InputTagType tag = (InputTagType)patchIndex;

      const SubdivPatch1& subdiv_patch = patches[patchIndex];
      
      subdiv_patch.prefetchData();

      const SubdivMesh* const geom = (SubdivMesh*)scene->get(subdiv_patch.geom); // FIXME: test flag first
      
      const unsigned int blocks = subdiv_patch.grid_subtree_size_64b_blocks;
      TessellationCacheTag *t = local_cache->request(tag,commitCounter,blocks);		      
      mic_f *local_mem = (mic_f*)t->getPtr(); 

      unsigned int currentIndex = 0;
      
      BVH4i::NodeRef bvh4i_root = initLocalLazySubdivTree(subdiv_patch,currentIndex,local_mem,geom);		      
      assert( bvh4i_root != BVH4i::invalidNode);

      //local_cache->updateRootRef(t,subtree_root);
      t->updateRootRef((size_t)bvh4i_root + (size_t)local_mem);

      assert( bvh4i_root == extractBVH4iNodeRef( t->getRootRef() ));
      assert( local_mem  == extractBVH4iPtr( t->getRootRef() ));
      assert( (size_t)extractBVH4iNodeRef( t->getRootRef() ) + (size_t)extractBVH4iPtr( t->getRootRef() ) == t->getRootRef() );
      return t;
    }

    __forceinline TessellationCacheTag *lookUpLocalTessellationCache(TessellationCache *local_cache,
								     const unsigned int patchIndex,
								     const unsigned int commitCounter,
								     const SubdivPatch1* const patches,
								     Scene *const scene)
    {
      CACHE_STATS(DistributedTessellationCacheStats::cache_accesses++);

      InputTagType tag = (InputTagType)patchIndex;

     TessellationCacheTag *t = local_cache->lookup(tag,commitCounter);
      if (unlikely(t == NULL))
	{
	  CACHE_STATS(DistributedTessellationCacheStats::cache_misses++);
	  return fillLocalTessellationCache(local_cache,patchIndex,commitCounter,patches,scene);
	}
      CACHE_STATS(DistributedTessellationCacheStats::cache_hits++);

      return t;
    }


    __noinline TessellationCacheTag *lookUpSharedTessellationCache(TessellationCache *local_cache,
								   const unsigned int patchIndex,
								   const unsigned int commitCounter,
								   const SubdivPatch1* const patches,
								   Scene *const scene)
    {
      InputTagType tag = (InputTagType)patchIndex;
      CACHE_STATS(SharedTessellationCacheStats::cache_accesses++);
      TessellationCacheTag *t = sharedTessellationCache.getTag(tag);
      t->read_lock();
      if (unlikely(!t->match(tag,commitCounter)))
        {
          CACHE_STATS(SharedTessellationCacheStats::cache_misses++);
	  t->read_unlock();
	  const SubdivPatch1& subdiv_patch = patches[patchIndex];
          subdiv_patch.prefetchData();

#if 1
          const unsigned int needed_blocks = subdiv_patch.grid_subtree_size_64b_blocks;
	  mic_f* local_mem = (mic_f*)alloc_tessellation_cache_mem(needed_blocks);
	  const SubdivMesh* const geom = (SubdivMesh*)scene->get(subdiv_patch.geom); // FIXME: test flag first
	  unsigned int currentIndex = 0;
	  BVH4i::NodeRef bvh4i_root = initLocalLazySubdivTree(subdiv_patch,currentIndex,local_mem,geom);		      

	  t->write_lock();
          mic_f* old_mem = (mic_f*)t->getPtr();
	  if (old_mem != NULL)
	    free_tessellation_cache_mem(old_mem,t->getNumBlocks()); 

	  t->set(tag,commitCounter,(size_t)local_mem,needed_blocks);
	  t->updateRootRef((size_t)bvh4i_root + (size_t)local_mem);

	  assert( bvh4i_root == extractBVH4iNodeRef( t->getRootRef() ));
	  assert( local_mem  == extractBVH4iPtr( t->getRootRef() ));
	  assert( (size_t)extractBVH4iNodeRef( t->getRootRef() ) + (size_t)extractBVH4iPtr( t->getRootRef() ) == t->getRootRef() );

          t->upgrade_write_to_read_lock();
	  return t;
	  
#else
          t->write_lock();
          const unsigned int needed_blocks = subdiv_patch.grid_subtree_size_64b_blocks;
          mic_f* local_mem = (mic_f*)t->getPtr();
          if (t->getNumBlocks() < needed_blocks)
            {
	      if (local_mem != NULL)
		free_tessellation_cache_mem(local_mem); 
	      local_mem = (mic_f*)alloc_tessellation_cache_mem(needed_blocks);
	      CACHE_STATS(SharedTessellationCacheStats::cache_evictions++);              
	      
	      t->set(tag,commitCounter,(size_t)local_mem,needed_blocks);
	    }
	  else
            {
              t->update(tag,commitCounter);              
              assert(local_mem != NULL);
            }

	  const SubdivMesh* const geom = (SubdivMesh*)scene->get(subdiv_patch.geom); // FIXME: test flag first

	  unsigned int currentIndex = 0;
	  BVH4i::NodeRef bvh4i_root = initLocalLazySubdivTree(subdiv_patch,currentIndex,local_mem,geom);		      
	  
	  t->updateRootRef((size_t)bvh4i_root + (size_t)local_mem);

	  assert( bvh4i_root == extractBVH4iNodeRef( t->getRootRef() ));
	  assert( local_mem  == extractBVH4iPtr( t->getRootRef() ));
	  assert( (size_t)extractBVH4iNodeRef( t->getRootRef() ) + (size_t)extractBVH4iPtr( t->getRootRef() ) == t->getRootRef() );
          t->upgrade_write_to_read_lock();
	  return t;
#endif
	}
      CACHE_STATS(SharedTessellationCacheStats::cache_hits++);
      return t;
    }



    __noinline TessellationCacheTag *try_lookUpSharedTessellationCache(const unsigned int patchIndex,
								       const unsigned int commitCounter)
    {
      InputTagType tag = (InputTagType)patchIndex;
      TessellationCacheTag *t = sharedTessellationCache.getTag(tag);
      if (t->try_read_lock())
	{
	  CACHE_STATS(SharedTessellationCacheStats::cache_accesses++);
	  if (unlikely(!t->match(tag,commitCounter)))
	    {
	      CACHE_STATS(SharedTessellationCacheStats::cache_misses++);
	      t->read_unlock();
	      return NULL;	  
	    }
	  CACHE_STATS(SharedTessellationCacheStats::cache_hits++);
	  return t;
	}
      return NULL;
    }

    __noinline void try_writeSharedTessellationCache(const unsigned int patchIndex,
						     const unsigned int commitCounter,
						     mic_f *mem,
						     const unsigned int blocks,
						     const BVH4i::NodeRef root)
    {
      InputTagType tag = (InputTagType)patchIndex;
      TessellationCacheTag *t = sharedTessellationCache.getTag(tag);
      assert(t);
      if (t->try_write_lock())
	{
	  if (unlikely(t->match(tag,commitCounter)))
	    {
	      //someone else already wrote it => do nothing
	    }
	  else
	    {
	      mic_f* local_mem = (mic_f*)t->getPtr();

	      if (t->getNumBlocks() < blocks)
		{
		  if (local_mem != NULL) free_tessellation_cache_mem(local_mem,t->getNumBlocks()); 	      
		  local_mem = (mic_f*)alloc_tessellation_cache_mem(blocks);
		}

	      memcpy(local_mem,mem,64 * blocks);
	      
	      t->set(tag,commitCounter,(size_t)local_mem,blocks);
	      t->updateRootRef((size_t)root + (size_t)local_mem);	      

	      assert( root == extractBVH4iNodeRef( t->getRootRef() ));
	      assert( local_mem  == extractBVH4iPtr( t->getRootRef() ));
	      assert( (size_t)extractBVH4iNodeRef( t->getRootRef() ) + (size_t)extractBVH4iPtr( t->getRootRef() ) == t->getRootRef() );

	    }
	  t->write_unlock();
	}
    }


    __forceinline TessellationCacheTag *lookUpTessellationCacheHierarchy(TessellationCache *local_cache,
									 const unsigned int patchIndex,
									 const unsigned int commitCounter,
									 const SubdivPatch1* const patches,
									 Scene *const scene)
    {
      CACHE_STATS(DistributedTessellationCacheStats::cache_accesses++);

      InputTagType tag = (InputTagType)patchIndex;

      TessellationCacheTag *t_l1 = local_cache->lookup(tag,commitCounter);

      if (unlikely(t_l1 == NULL))
	{
	  CACHE_STATS(DistributedTessellationCacheStats::cache_misses++);

#if 1
	  TessellationCacheTag *t_l2 = try_lookUpSharedTessellationCache(patchIndex,commitCounter);
	  ////////////////////////////////////////////////////////////////////////////////////
	  ////////////////////////////////////////////////////////////////////////////////////
	  ////////////////////////////////////////////////////////////////////////////////////

	  if (t_l2 != NULL)
	    {
	      /* L2 holds valid data */
	      BVH4i::NodeRef t_ref = extractBVH4iNodeRef(t_l2->getRootRef());
	      const SubdivPatch1& subdiv_patch = patches[patchIndex];
	      const unsigned int blocks = subdiv_patch.grid_subtree_size_64b_blocks;

	      t_l1 = local_cache->request(tag,commitCounter,blocks);		      

	      mic_f *local_mem = (mic_f*)t_l1->getPtr(); 

	      assert(t_l2->getNumBlocks() >= blocks);
	      assert(t_l1->getNumBlocks() >= blocks);

	      memcpy(local_mem,t_l2->getPtr(),64*blocks);
	  
	      t_l2->read_unlock();

	      t_l1->set(tag,commitCounter,(size_t)local_mem + (size_t)t_ref,blocks);

	      return t_l1;
	    }
	  ////////////////////////////////////////////////////////////////////////////////////
	  ////////////////////////////////////////////////////////////////////////////////////
	  ////////////////////////////////////////////////////////////////////////////////////

	  TessellationCacheTag *t_l1 = fillLocalTessellationCache(local_cache,patchIndex,commitCounter,patches,scene);
	  size_t subtree_root = t_l1->getRootRef();

	  const SubdivPatch1& subdiv_patch = patches[patchIndex];
	  const unsigned int needed_blocks = subdiv_patch.grid_subtree_size_64b_blocks;

	  mic_f *local_mem = (mic_f*)t_l1->getPtr(); 
	      
	  try_writeSharedTessellationCache(patchIndex,commitCounter,local_mem,needed_blocks,extractBVH4iNodeRef(subtree_root));
	      	  
	  
	  return t_l1;
#else
	  /* second level lookup */
	  TessellationCacheTag *t_l2 = lookUpSharedTessellationCache(local_cache,patchIndex,commitCounter,patches,scene);

	  BVH4i::NodeRef t_ref = extractBVH4iNodeRef(t_l2->getRootRef());

	  const unsigned int blocks = t_l2->getNumBlocks();

	  TessellationCacheTag &t_l1 = local_cache->request(tag,commitCounter,blocks);		      
	  mic_f *local_mem = (mic_f*)t_l1->getPtr(); 
	  memcpy(local_mem,t_l2->getPtr(),64*blocks);
	  
	  t_l2->read_unlock();

	  //local_cache->updateRootRef(t,subtree_root);
	  t_l1->set(tag,commitCounter,(size_t)local_mem + (size_t)t_ref,blocks);

	  assert( t_ref == extractBVH4iNodeRef( t_l1->getRootRef() ));
	  assert( local_mem  == extractBVH4iPtr( t_l1->getRootRef() ));
	  assert( (size_t)extractBVH4iNodeRef( t_l1->getRootRef() ) + (size_t)extractBVH4iPtr( t_l1->getRootRef() ) == t_l1->getRootRef() );

	  return t_l1;
#endif
	}
      CACHE_STATS(DistributedTessellationCacheStats::cache_hits++);

      return t_l1;
    }

    __forceinline TessellationCacheTag *lookUpTessellationCache(TessellationCache *local_cache,
								const unsigned int patchIndex,
								const unsigned int commitCounter,
								const SubdivPatch1* const patches,
								Scene *const scene)
    {
#if !defined(ONLY_SHARED_CACHE)

#if defined(CACHE_HIERARCHY)
      TessellationCacheTag *t = lookUpTessellationCacheHierarchy(local_cache,patchIndex,commitCounter,patches,scene);
#else
      TessellationCacheTag *t = lookUpLocalTessellationCache(local_cache,patchIndex,commitCounter,patches,scene);
#endif
#else
      TessellationCacheTag *t = lookUpSharedTessellationCache(local_cache,patchIndex,commitCounter,patches,scene);	      
#endif
      return t;
    }



    __noinline size_t lazyBuildPatchIntoScratchMem(const unsigned int patchIndex,
						   const SubdivPatch1* const subdiv_patch,
						   Scene *const scene,
						   TessellationRefCache *ref_cache)
    {
      const unsigned int needed_blocks = subdiv_patch->grid_subtree_size_64b_blocks;          

      if (unlikely(ref_cache->getScratchMemBlocks() < needed_blocks))
	ref_cache->reallocScratchMem(needed_blocks);

      mic_f *local_mem = (mic_f*)ref_cache->getScratchMemPtr();

      /* build subtree */
      const SubdivMesh* const geom = (SubdivMesh*)scene->get(subdiv_patch->geom); // FIXME: test flag first

      unsigned int currentIndex = 0;
      BVH4i::NodeRef bvh4i_root = initLocalLazySubdivTree(*subdiv_patch,currentIndex,local_mem,geom);		      
      
      size_t new_root = (size_t)bvh4i_root + (size_t)local_mem;
      return new_root;

    }

    static __forceinline void flushLocalTessellationCache(TessellationRefCache *ref_cache)
    {
      TessellationRefCacheTag *t = ref_cache->getTagsPtr();
      for (size_t i=0;i<ref_cache->getNumTags();i++)
	if (!t[i].empty())
	  {
	    TessellationCacheTag *old = sharedTessellationCache.getTagBy32BitID(t[i].getPrimTag());
	    old->read_unlock();
	    t[i].reset();
	  }
    }

    __noinline size_t lazyBuildPatchMissHandler(const unsigned int patchIndex,
						const unsigned int commitCounter,
						SubdivPatch1* const subdiv_patch,
						Scene *const scene,
						TessellationRefCacheTag *t,
						TessellationRefCache *ref_cache)
    {
      const unsigned int tag = patchIndex;

      if (!t->empty()) {
	TessellationCacheTag *old = sharedTessellationCache.getTag(t->getPrimTag());
	old->read_unlock();
	t->reset();
      };      

      TessellationCacheTag *s = sharedTessellationCache.getTag(tag);

      /* data is in shared cache ? */
      CACHE_STATS(SharedTessellationCacheStats::cache_accesses++);
      s->read_lock();
      if (s->match(tag,commitCounter))
	{
	  CACHE_STATS(SharedTessellationCacheStats::cache_hits++);
	  size_t patch_root = s->getRootRef();
	  t->set(tag,commitCounter,patch_root);
	  return patch_root;
	}
      CACHE_STATS(else SharedTessellationCacheStats::cache_misses++);
      s->read_unlock();

      CACHE_STATS(SharedTessellationCacheStats::cache_updates++);
      if (s->try_write_lock())
	{
	  CACHE_STATS(SharedTessellationCacheStats::cache_updates_successful++);

	  if (s->match(tag,commitCounter))
	    {
	      size_t patch_root = s->getRootRef();
	      t->set(tag,commitCounter,patch_root);

	      s->upgrade_write_to_read_lock();
	      return patch_root;
	    }
	  else
	    {
	      mic_f* local_mem = (mic_f*)s->getPtr();
	      const unsigned int needed_blocks = subdiv_patch->grid_subtree_size_64b_blocks;          
	      if (s->getNumBlocks() < needed_blocks)
		{
		  if (local_mem != NULL)
		    free_tessellation_cache_mem(local_mem);
		  local_mem = (mic_f*)alloc_tessellation_cache_mem(needed_blocks);              	      
		}
	      
	      const SubdivMesh* const geom = (SubdivMesh*)scene->get(subdiv_patch->geom); 
	      unsigned int currentIndex = 0;
	      BVH4i::NodeRef bvh4i_root = initLocalLazySubdivTree(*subdiv_patch,currentIndex,local_mem,geom);		      
	      size_t new_root = (size_t)bvh4i_root + (size_t)local_mem;

	      s->set(tag,commitCounter,new_root,needed_blocks);
	      /* insert new patch data into local cache */
	      t->set(tag,commitCounter,new_root);

	      s->upgrade_write_to_read_lock();
	      return new_root;
	    }
	}
      
      CACHE_STATS(SharedTessellationCacheStats::cache_fallbacks++);      
      subdiv_patch->prefetchData();
      size_t scratch_root = lazyBuildPatchIntoScratchMem(patchIndex,subdiv_patch,scene,ref_cache);
      return scratch_root;
    }

    __forceinline size_t lazyBuildPatch(const unsigned int patchIndex,
					const unsigned int commitCounter,
					SubdivPatch1* const patches,
					const size_t numPatches,
					Scene *const scene,
					TessellationRefCache *ref_cache,
					LocalThreadInfo *threadInfo)
    {
#if SHARED_LAZY_CACHE == 1
      sharedLazyTessellationCache.readLockThread(threadInfo->id);
      SubdivPatch1* subdiv_patch = &patches[patchIndex];

	  
      CACHE_STATS(SharedTessellationCacheStats::cache_accesses++);
      if (likely(subdiv_patch->root_ref)) 
	{
	  CACHE_STATS(SharedTessellationCacheStats::cache_hits++);
	  return subdiv_patch->root_ref;
	}
      else
	{
	  CACHE_STATS(SharedTessellationCacheStats::cache_misses++);
	  sharedLazyTessellationCache.readUnlockThread(threadInfo->id);

	  while(1)
	    {
	      sharedLazyTessellationCache.readLockThread(threadInfo->id);
	      subdiv_patch->write_lock();
	      if (subdiv_patch->root_ref == 0)
		{

		  const SubdivMesh* const geom = (SubdivMesh*)scene->get(subdiv_patch->geom); 
		  size_t block_index = sharedLazyTessellationCache.alloc(subdiv_patch->grid_subtree_size_64b_blocks);
		  if (block_index == (size_t)-1)
		    {
		      /* cannot allocate => flush the cache */
		      subdiv_patch->write_unlock();
		      sharedLazyTessellationCache.readUnlockThread(threadInfo->id);

		      sharedLazyTessellationCache.resetCache(patches,numPatches);
		      continue;
		    }
		  //DBG_PRINT( sharedLazyTessellationCache.getNumAllocatedBytes() );
		  mic_f* local_mem   = sharedLazyTessellationCache.getBlockPtr(block_index);
		  unsigned int currentIndex = 0;
		  BVH4i::NodeRef bvh4i_root = initLocalLazySubdivTree(*subdiv_patch,currentIndex,local_mem,geom);		      
		  size_t new_root = (size_t)bvh4i_root + (size_t)local_mem;
		  subdiv_patch->root_ref = new_root;
		}
	      subdiv_patch->write_unlock();
	      assert(subdiv_patch->root_ref);
	      return subdiv_patch->root_ref;
	    }
	}

	
      
#else
      /* new commitCounter flush local per thread cache */
      if (unlikely(ref_cache->needCommitCounterUpdate(commitCounter)))
	flushLocalTessellationCache(ref_cache);


      /* lookup in per thread reference cache */
      CACHE_STATS(DistributedTessellationCacheStats::cache_accesses++);
      TessellationRefCacheTag *t = ref_cache->lookUpTag(patchIndex,commitCounter);
      if (likely(t->match(patchIndex,commitCounter)))
	{
	  CACHE_STATS(DistributedTessellationCacheStats::cache_hits++);
	  return t->getRootRef();
	}
      CACHE_STATS(DistributedTessellationCacheStats::cache_misses++);

      SubdivPatch1* subdiv_patch = &patches[patchIndex];
      return lazyBuildPatchMissHandler(patchIndex,commitCounter,subdiv_patch,scene,t,ref_cache);
#endif
    }


    static unsigned int BVH4I_LEAF_MASK = BVH4i::leaf_mask; // needed due to compiler efficiency bug
    static unsigned int M_LANE_7777 = 0x7777;               // needed due to compiler efficiency bug

    // ============================================================================================
    // ============================================================================================
    // ============================================================================================


    void BVH4iIntersector16Subdiv::intersect(mic_i* valid_i, BVH4i* bvh, Ray16& ray16)
    {
      /* near and node stack */
      __aligned(64) float   stack_dist[4*BVH4i::maxDepth+1];
      __aligned(64) NodeRef stack_node[4*BVH4i::maxDepth+1];

      /* setup */
      const mic_m m_valid    = *(mic_i*)valid_i != mic_i(0);
      const mic3f rdir16     = rcp_safe(ray16.dir);
      const mic_f inf        = mic_f(pos_inf);
      const mic_f zero       = mic_f::zero();

      store16f(stack_dist,inf);
      ray16.primID = select(m_valid,mic_i(-1),ray16.primID);
      ray16.geomID = select(m_valid,mic_i(-1),ray16.geomID);

      Scene *const scene                         = (Scene*)bvh->geometry;
      const Node      * __restrict__ const nodes = (Node     *)bvh->nodePtr();
      Triangle1 * __restrict__ const accel       = (Triangle1*)bvh->triPtr();
      const unsigned int commitCounter           = scene->commitCounter;

#if LAZY_BUILD == 1
      /* query per thread tessellation cache */
      TessellationRefCache *local_ref_cache = NULL;
      if (unlikely(!tess_ref_cache))
	{
	  tess_ref_cache = new TessellationRefCache(  NUM_SCRATCH_MEM_BLOCKS );
	}
      local_ref_cache = tess_ref_cache;

      LocalThreadInfo *threadInfo = NULL;
      if (unlikely(!localThreadInfo))
	{
	  const unsigned int id = globalRenderThreads.add(1);
	  localThreadInfo = new LocalThreadInfo( id );
	}
      threadInfo = localThreadInfo;
#else
      /* query per thread tessellation cache */
      TessellationCache *local_cache = NULL;
      if (!tess_cache)
	createTessellationCache();
      local_cache = tess_cache;
#endif

      stack_node[0] = BVH4i::invalidNode;
      long rayIndex = -1;
      while((rayIndex = bitscan64(rayIndex,toInt(m_valid))) != BITSCAN_NO_BIT_SET_64)	    
        {
	  stack_node[1] = bvh->root;
	  size_t sindex = 2;

	  const mic_f org_xyz      = loadAOS4to16f(rayIndex,ray16.org.x,ray16.org.y,ray16.org.z);
	  const mic_f dir_xyz      = loadAOS4to16f(rayIndex,ray16.dir.x,ray16.dir.y,ray16.dir.z);
	  const mic_f rdir_xyz     = loadAOS4to16f(rayIndex,rdir16.x,rdir16.y,rdir16.z);
	  //const mic_f org_rdir_xyz = org_xyz * rdir_xyz;
	  const mic_f min_dist_xyz = broadcast1to16f(&ray16.tnear[rayIndex]);
	  mic_f       max_dist_xyz = broadcast1to16f(&ray16.tfar[rayIndex]);

	  const unsigned int leaf_mask = BVH4I_LEAF_MASK;
	  const Precalculations precalculations(org_xyz,rdir_xyz);

	  while (1)
	    {

	      NodeRef curNode = stack_node[sindex-1];
	      sindex--;

	      traverse_single_intersect<false,true>(curNode,
						    sindex,
						    precalculations,
						    min_dist_xyz,
						    max_dist_xyz,
						    stack_node,
						    stack_dist,
						    nodes,
						    leaf_mask);
		   

	      /* return if stack is empty */
	      if (unlikely(curNode == BVH4i::invalidNode)) break;

	      STAT3(normal.trav_leaves,1,1,1);
	      STAT3(normal.trav_prims,1,1,1);

	      //////////////////////////////////////////////////////////////////////////////////////////////////

	      const unsigned int patchIndex = curNode.offsetIndex();

	      // ----------------------------------------------------------------------------------------------------
#if LAZY_BUILD == 1
	      size_t cached_64bit_root = lazyBuildPatch(patchIndex,commitCounter,(SubdivPatch1*)accel,bvh->numPrimitives,scene,local_ref_cache,threadInfo);
#else
	      TessellationCacheTag *t = lookUpTessellationCache(local_cache,patchIndex,commitCounter,(SubdivPatch1*)accel,scene);
	      size_t cached_64bit_root = t->getRootRef();	      
#endif
	      BVH4i::NodeRef subtree_root = extractBVH4iNodeRef(cached_64bit_root); 
	      mic_f     * const __restrict__ lazymem     = (mic_f*)extractBVH4iPtr(cached_64bit_root); 
	      // ----------------------------------------------------------------------------------------------------


	      // -------------------------------------
	      // -------------------------------------
	      // -------------------------------------

	      float   * __restrict__ const sub_stack_dist = &stack_dist[sindex];
	      NodeRef * __restrict__ const sub_stack_node = &stack_node[sindex];
	      sub_stack_node[0] = BVH4i::invalidNode;
	      sub_stack_node[1] = subtree_root;
	      ustore16f(sub_stack_dist,inf);
	      size_t sub_sindex = 2;

	      while (1)
		{
		  curNode = sub_stack_node[sub_sindex-1];
		  sub_sindex--;

		  traverse_single_intersect<false, true>(curNode,
							 sub_sindex,
							 precalculations,
							 min_dist_xyz,
							 max_dist_xyz,
							 sub_stack_node,
							 sub_stack_dist,
							 (BVH4i::Node*)lazymem,
							 leaf_mask);
		 		   

		  /* return if stack is empty */
		  if (unlikely(curNode == BVH4i::invalidNode)) break;

		  const unsigned int uvIndex = curNode.offsetIndex();
		  
		  const mic_m m_active = 0x777;
		  const Quad4x4 *__restrict__ const quad4x4 = (Quad4x4*)&lazymem[uvIndex];
		  quad4x4->prefetchData();
		  const mic_f uu = quad4x4->getU();
		  const mic_f vv = quad4x4->getV();

		  intersect1_quad16(rayIndex, 
				    dir_xyz,
				    org_xyz,
				    ray16,
				    quad4x4->vtx,
				    uu,
				    vv,
				    4,
				    m_active,
				    patchIndex);
		}

#if SHARED_LAZY_CACHE == 1
	      sharedLazyTessellationCache.readUnlockThread(threadInfo->id);
#endif

#if defined(ONLY_SHARED_CACHE)
	      t->read_unlock();
#endif

	      // -------------------------------------
	      // -------------------------------------
	      // -------------------------------------

	      compactStack(stack_node,stack_dist,sindex,mic_f(ray16.tfar[rayIndex]));

	      // ------------------------
	    }
	}


      /* update primID/geomID and compute normals */
      mic_m m_hit = (ray16.primID != -1) & m_valid;
      rayIndex = -1;
      while((rayIndex = bitscan64(rayIndex,toInt(m_hit))) != BITSCAN_NO_BIT_SET_64)	    
        {
	  const SubdivPatch1& subdiv_patch = ((SubdivPatch1*)accel)[ray16.primID[rayIndex]];
	  ray16.primID[rayIndex] = subdiv_patch.prim;
	  ray16.geomID[rayIndex] = subdiv_patch.geom;
#if defined(RTCORE_RETURN_SUBDIV_NORMAL)

	  if (unlikely(!subdiv_patch.hasDisplacement()))
	    {
	      const Vec3fa normal    = subdiv_patch.normal(ray16.u[rayIndex],ray16.v[rayIndex]);
	      ray16.Ng.x[rayIndex]   = normal.x;
	      ray16.Ng.y[rayIndex]   = normal.y;
	      ray16.Ng.z[rayIndex]   = normal.z;
	    }
#endif

#if FORCE_TRIANGLE_UV == 0

	  const Vec2f uv0 = subdiv_patch.getUV(0);
	  const Vec2f uv1 = subdiv_patch.getUV(1);
	  const Vec2f uv2 = subdiv_patch.getUV(2);
	  const Vec2f uv3 = subdiv_patch.getUV(3);
	  
	  const float patch_u = bilinear_interpolate(uv0.x,uv1.x,uv2.x,uv3.x,ray16.u[rayIndex],ray16.v[rayIndex]);
	  const float patch_v = bilinear_interpolate(uv0.y,uv1.y,uv2.y,uv3.y,ray16.u[rayIndex],ray16.v[rayIndex]);
	  ray16.u[rayIndex] = patch_u;
	  ray16.v[rayIndex] = patch_v;
#endif
	}

    }

    void BVH4iIntersector16Subdiv::occluded(mic_i* valid_i, BVH4i* bvh, Ray16& ray16)
    {
      /* near and node stack */
      __aligned(64) NodeRef stack_node[4*BVH4i::maxDepth+1];

      /* setup */
      const mic_m m_valid = *(mic_i*)valid_i != mic_i(0);
      const mic3f rdir16  = rcp_safe(ray16.dir);
      mic_m terminated    = !m_valid;
      const mic_f inf     = mic_f(pos_inf);
      const mic_f zero    = mic_f::zero();

      Scene *const scene                   = (Scene*)bvh->geometry;
      const Node      * __restrict__ nodes = (Node     *)bvh->nodePtr();
      const Triangle1 * __restrict__ accel = (Triangle1*)bvh->triPtr();
      const unsigned int commitCounter           = scene->commitCounter;

#if LAZY_BUILD == 1
      /* query per thread tessellation cache */
      TessellationRefCache *local_ref_cache = NULL;
      if (unlikely(!tess_ref_cache))
	tess_ref_cache = new TessellationRefCache(  NUM_SCRATCH_MEM_BLOCKS  );
      local_ref_cache = tess_ref_cache;

      LocalThreadInfo *threadInfo = NULL;
      if (unlikely(!localThreadInfo))
	{
	  const unsigned int id = globalRenderThreads.add(1);
	  localThreadInfo = new LocalThreadInfo( id );
	}
      threadInfo = localThreadInfo;

#else
      /* query per thread tessellation cache */
      TessellationCache *local_cache = NULL;
      if (!tess_cache)
	createTessellationCache();
      local_cache = tess_cache;
#endif

      stack_node[0] = BVH4i::invalidNode;
      ray16.primID = select(m_valid,mic_i(-1),ray16.primID);
      ray16.geomID = select(m_valid,mic_i(-1),ray16.geomID);

      long rayIndex = -1;
      while((rayIndex = bitscan64(rayIndex,toInt(m_valid))) != BITSCAN_NO_BIT_SET_64)	    
        {
	  stack_node[1] = bvh->root;
	  size_t sindex = 2;

	  const mic_f org_xyz      = loadAOS4to16f(rayIndex,ray16.org.x,ray16.org.y,ray16.org.z);
	  const mic_f dir_xyz      = loadAOS4to16f(rayIndex,ray16.dir.x,ray16.dir.y,ray16.dir.z);
	  const mic_f rdir_xyz     = loadAOS4to16f(rayIndex,rdir16.x,rdir16.y,rdir16.z);
	  //const mic_f org_rdir_xyz = org_xyz * rdir_xyz;
	  const mic_f min_dist_xyz = broadcast1to16f(&ray16.tnear[rayIndex]);
	  const mic_f max_dist_xyz = broadcast1to16f(&ray16.tfar[rayIndex]);
	  const mic_i v_invalidNode(BVH4i::invalidNode);
	  const unsigned int leaf_mask = BVH4I_LEAF_MASK;
	  const Precalculations precalculations(org_xyz,rdir_xyz);

	  while (1)
	    {
	      NodeRef curNode = stack_node[sindex-1];
	      sindex--;

	      traverse_single_occluded< false, true >(curNode,
						      sindex,
						      precalculations,
						      min_dist_xyz,
						      max_dist_xyz,
						      stack_node,
						      nodes,
						      leaf_mask);

	      /* return if stack is empty */
	      if (unlikely(curNode == BVH4i::invalidNode)) break;

	      STAT3(shadow.trav_leaves,1,1,1);
	      STAT3(shadow.trav_prims,1,1,1);

	      //////////////////////////////////////////////////////////////////////////////////////////////////

	      

	      const unsigned int patchIndex = curNode.offsetIndex();

	      // ----------------------------------------------------------------------------------------------------
#if LAZY_BUILD == 1
	      size_t cached_64bit_root = lazyBuildPatch(patchIndex,commitCounter,(SubdivPatch1*)accel,bvh->numPrimitives,scene,local_ref_cache,threadInfo);
#else
	      TessellationCacheTag *t = lookUpTessellationCache(local_cache,patchIndex,commitCounter,(SubdivPatch1*)accel,scene);
	      size_t cached_64bit_root = t->getRootRef();	      
#endif
	      BVH4i::NodeRef subtree_root = extractBVH4iNodeRef(cached_64bit_root); 
	      mic_f     * const __restrict__ lazymem     = (mic_f*)extractBVH4iPtr(cached_64bit_root); 
	      // ----------------------------------------------------------------------------------------------------

	      {
		    
		// -------------------------------------
		// -------------------------------------
		// -------------------------------------

		__aligned(64) NodeRef sub_stack_node[64];
		sub_stack_node[0] = BVH4i::invalidNode;
		sub_stack_node[1] = subtree_root;
		size_t sub_sindex = 2;

		///////////////////////////////////////////////////////////////////////////////////////////////////////
		while (1)
		  {
		    curNode = sub_stack_node[sub_sindex-1];
		    sub_sindex--;

		    traverse_single_occluded<false, true>(curNode,
							  sub_sindex,
							  precalculations,
							  min_dist_xyz,
							  max_dist_xyz,
							  sub_stack_node,
							  (BVH4i::Node*)lazymem,
							  leaf_mask);
		 		   

		    /* return if stack is empty */
		    if (unlikely(curNode == BVH4i::invalidNode)) break;

		    const unsigned int uvIndex = curNode.offsetIndex();
		  
		    const mic_m m_active = 0x777;
		    const Quad4x4 *__restrict__ const quad4x4 = (Quad4x4*)&lazymem[uvIndex];
		    quad4x4->prefetchData();
		    const mic_f uu = quad4x4->getU();
		    const mic_f vv = quad4x4->getV();
		  
		    if (unlikely(occluded1_quad16(rayIndex, 
						  dir_xyz,
						  org_xyz,
						  ray16,
						  quad4x4->vtx,
						  uu,
						  vv,
						  4,
						  m_active,
						  patchIndex)))
		      {
			terminated |= (mic_m)((unsigned int)1 << rayIndex);
			break;
		      }
		  }

#if SHARED_LAZY_CACHE == 1
		sharedLazyTessellationCache.readUnlockThread(threadInfo->id);
#endif

#if defined(ONLY_SHARED_CACHE)
		t->read_unlock();
#endif
	      }

	      if (unlikely(terminated & (mic_m)((unsigned int)1 << rayIndex))) break;
	      //////////////////////////////////////////////////////////////////////////////////////////////////

	    }

	  if (unlikely(all(toMask(terminated)))) break;
	}


      store16i(m_valid & toMask(terminated),&ray16.geomID,0);

    }


    void BVH4iIntersector1Subdiv::intersect(BVH4i* bvh, Ray& ray)
    {
      /* near and node stack */
      __aligned(64) float   stack_dist[3*BVH4i::maxDepth+1];
      __aligned(64) NodeRef stack_node[3*BVH4i::maxDepth+1];

      /* setup */
      const mic3f rdir16     = rcp_safe(mic3f(mic_f(ray.dir.x),mic_f(ray.dir.y),mic_f(ray.dir.z)));
      const mic_f inf        = mic_f(pos_inf);
      const mic_f zero       = mic_f::zero();

      store16f(stack_dist,inf);

      const Node      * __restrict__ nodes = (Node    *)bvh->nodePtr();
      const Triangle1 * __restrict__ accel = (Triangle1*)bvh->triPtr();
      Scene *const scene                   = (Scene*)bvh->geometry;
      const unsigned int commitCounter     = scene->commitCounter;

#if LAZY_BUILD == 1
      /* query per thread tessellation cache */
      TessellationRefCache *local_ref_cache = NULL;
      if (unlikely(!tess_ref_cache))
	tess_ref_cache = new TessellationRefCache(  NUM_SCRATCH_MEM_BLOCKS  );
      local_ref_cache = tess_ref_cache;

      LocalThreadInfo *threadInfo = NULL;
      if (unlikely(!localThreadInfo))
	{
	  const unsigned int id = globalRenderThreads.add(1);
	  localThreadInfo = new LocalThreadInfo( id );
	}
      threadInfo = localThreadInfo;

#else
      /* query per thread tessellation cache */
      TessellationCache *local_cache = NULL;
      if (!tess_cache)
	createTessellationCache();
      local_cache = tess_cache;
#endif

      stack_node[0] = BVH4i::invalidNode;      
      stack_node[1] = bvh->root;

      size_t sindex = 2;

      const mic_f org_xyz      = loadAOS4to16f(ray.org.x,ray.org.y,ray.org.z);
      const mic_f dir_xyz      = loadAOS4to16f(ray.dir.x,ray.dir.y,ray.dir.z);
      const mic_f rdir_xyz     = loadAOS4to16f(rdir16.x[0],rdir16.y[0],rdir16.z[0]);
      //const mic_f org_rdir_xyz = org_xyz * rdir_xyz;
      const mic_f min_dist_xyz = broadcast1to16f(&ray.tnear);
      mic_f       max_dist_xyz = broadcast1to16f(&ray.tfar);
	  
      const unsigned int leaf_mask = BVH4I_LEAF_MASK;
      const Precalculations precalculations(org_xyz,rdir_xyz);
	  
      while (1)
	{
	  NodeRef curNode = stack_node[sindex-1];
	  sindex--;

	  traverse_single_intersect<false, true>(curNode,
						 sindex,
						 precalculations,
						 min_dist_xyz,
						 max_dist_xyz,
						 stack_node,
						 stack_dist,
						 nodes,
						 leaf_mask);            		    

	  /* return if stack is empty */
	  if (unlikely(curNode == BVH4i::invalidNode)) break;

	  STAT3(normal.trav_leaves,1,1,1);
	  STAT3(normal.trav_prims,1,1,1);


	  //////////////////////////////////////////////////////////////////////////////////////////////////

	  const unsigned int patchIndex = curNode.offsetIndex();

	  // ----------------------------------------------------------------------------------------------------
#if LAZY_BUILD == 1
	  size_t cached_64bit_root = lazyBuildPatch(patchIndex,commitCounter,(SubdivPatch1*)accel,bvh->numPrimitives,scene,local_ref_cache,threadInfo);
#else
	  TessellationCacheTag *t = lookUpTessellationCache(local_cache,patchIndex,commitCounter,(SubdivPatch1*)accel,scene);
	  size_t cached_64bit_root = t->getRootRef();	      
#endif
	  BVH4i::NodeRef subtree_root = extractBVH4iNodeRef(cached_64bit_root); 
	  mic_f     * const __restrict__ lazymem     = (mic_f*)extractBVH4iPtr(cached_64bit_root); 
	  // ----------------------------------------------------------------------------------------------------


	  // -------------------------------------
	  // -------------------------------------
	  // -------------------------------------

	  float   * __restrict__ const sub_stack_dist = &stack_dist[sindex];
	  NodeRef * __restrict__ const sub_stack_node = &stack_node[sindex];
	  sub_stack_node[0] = BVH4i::invalidNode;
	  sub_stack_node[1] = subtree_root;
	  ustore16f(sub_stack_dist,inf);
	  size_t sub_sindex = 2;

	  while (1)
	    {
	      curNode = sub_stack_node[sub_sindex-1];
	      sub_sindex--;

	      traverse_single_intersect<false,true>(curNode,
						    sub_sindex,
						    precalculations,
						    min_dist_xyz,
						    max_dist_xyz,
						    sub_stack_node,
						    sub_stack_dist,
						    (BVH4i::Node*)lazymem,
						    leaf_mask);
		 		   

	      /* return if stack is empty */
	      if (unlikely(curNode == BVH4i::invalidNode)) break;

	      const unsigned int uvIndex = curNode.offsetIndex();
		  
	      const mic_m m_active = 0x777;
	      const Quad4x4 *__restrict__ const quad4x4 = (Quad4x4*)&lazymem[uvIndex];
	      quad4x4->prefetchData();
	      const mic_f uu = quad4x4->getU();
	      const mic_f vv = quad4x4->getV();

	      intersect1_quad16(dir_xyz,
				org_xyz,
				ray,
				quad4x4->vtx,
				uu,
				vv,
				4,
				m_active,
				patchIndex);
	    }

#if SHARED_LAZY_CACHE == 1
	  sharedLazyTessellationCache.readUnlockThread(threadInfo->id);
#endif

#if defined(ONLY_SHARED_CACHE)
	  t->read_unlock();
#endif



	  compactStack(stack_node,stack_dist,sindex,max_dist_xyz);
	}
    }

    void BVH4iIntersector1Subdiv::occluded(BVH4i* bvh, Ray& ray)
    {
      /* near and node stack */
      __aligned(64) NodeRef stack_node[3*BVH4i::maxDepth+1];

      /* setup */
      const mic3f rdir16      = rcp_safe(mic3f(ray.dir.x,ray.dir.y,ray.dir.z));
      const mic_f inf         = mic_f(pos_inf);
      const mic_f zero        = mic_f::zero();

      const Node      * __restrict__ nodes = (Node     *)bvh->nodePtr();
      const Triangle1 * __restrict__ accel = (Triangle1*)bvh->triPtr();
      Scene *const scene                   = (Scene*)bvh->geometry;
      const unsigned int commitCounter     = scene->commitCounter;

#if LAZY_BUILD == 1
      /* query per thread tessellation cache */
      TessellationRefCache *local_ref_cache = NULL;
      if (unlikely(!tess_ref_cache))
	tess_ref_cache = new TessellationRefCache(  NUM_SCRATCH_MEM_BLOCKS  );
      local_ref_cache = tess_ref_cache;

      LocalThreadInfo *threadInfo = NULL;
      if (unlikely(!localThreadInfo))
	{
	  const unsigned int id = globalRenderThreads.add(1);
	  localThreadInfo = new LocalThreadInfo( id );
	}
      threadInfo = localThreadInfo;

#else
      /* query per thread tessellation cache */
      TessellationCache *local_cache = NULL;
      if (!tess_cache)
	createTessellationCache();
      local_cache = tess_cache;
#endif

      stack_node[0] = BVH4i::invalidNode;
      stack_node[1] = bvh->root;
      size_t sindex = 2;

      const mic_f org_xyz      = loadAOS4to16f(ray.org.x,ray.org.y,ray.org.z);
      const mic_f dir_xyz      = loadAOS4to16f(ray.dir.x,ray.dir.y,ray.dir.z);
      const mic_f rdir_xyz     = loadAOS4to16f(rdir16.x[0],rdir16.y[0],rdir16.z[0]);
      //const mic_f org_rdir_xyz = org_xyz * rdir_xyz;
      const mic_f min_dist_xyz = broadcast1to16f(&ray.tnear);
      const mic_f max_dist_xyz = broadcast1to16f(&ray.tfar);

      const unsigned int leaf_mask = BVH4I_LEAF_MASK;
      const Precalculations precalculations(org_xyz,rdir_xyz);
	  
      while (1)
	{
	  NodeRef curNode = stack_node[sindex-1];
	  sindex--;
            
	  
	  traverse_single_occluded< false, true >(curNode,
						  sindex,
						  precalculations,
						  min_dist_xyz,
						  max_dist_xyz,
						  stack_node,
						  nodes,
						  leaf_mask);	    

	  /* return if stack is empty */
	  if (unlikely(curNode == BVH4i::invalidNode)) break;


	  STAT3(shadow.trav_leaves,1,1,1);
	  STAT3(shadow.trav_prims,1,1,1);

	  //////////////////////////////////////////////////////////////////////////////////////////////////

	      

	  const unsigned int patchIndex = curNode.offsetIndex();

	  // ----------------------------------------------------------------------------------------------------
#if LAZY_BUILD == 1
	  size_t cached_64bit_root = lazyBuildPatch(patchIndex,commitCounter,(SubdivPatch1*)accel,bvh->numPrimitives,scene,local_ref_cache,threadInfo);
#else
	  TessellationCacheTag *t = lookUpTessellationCache(local_cache,patchIndex,commitCounter,(SubdivPatch1*)accel,scene);
	  size_t cached_64bit_root = t->getRootRef();	      
#endif
	  BVH4i::NodeRef subtree_root = extractBVH4iNodeRef(cached_64bit_root); 
	  mic_f     * const __restrict__ lazymem     = (mic_f*)extractBVH4iPtr(cached_64bit_root); 
	  // ----------------------------------------------------------------------------------------------------



		    
	  // -------------------------------------
	  // -------------------------------------
	  // -------------------------------------

	  __aligned(64) NodeRef sub_stack_node[64];
	  sub_stack_node[0] = BVH4i::invalidNode;
	  sub_stack_node[1] = subtree_root;
	  size_t sub_sindex = 2;

	  ///////////////////////////////////////////////////////////////////////////////////////////////////////
	  while (1)
	    {
	      curNode = sub_stack_node[sub_sindex-1];
	      sub_sindex--;

	      traverse_single_occluded<false,true>(curNode,
						   sub_sindex,
						   precalculations,
						   min_dist_xyz,
						   max_dist_xyz,
						   sub_stack_node,
						   (BVH4i::Node*)lazymem,
						   leaf_mask);
		 		   

	      /* return if stack is empty */
	      if (unlikely(curNode == BVH4i::invalidNode)) break;

	      const unsigned int uvIndex = curNode.offsetIndex();
		  
	      const mic_m m_active = 0x777;
	      const Quad4x4 *__restrict__ const quad4x4 = (Quad4x4*)&lazymem[uvIndex];
	      quad4x4->prefetchData();
	      const mic_f uu = quad4x4->getU();
	      const mic_f vv = quad4x4->getV();
		  
	      if (unlikely(occluded1_quad16(dir_xyz,
					    org_xyz,
					    ray,
					    quad4x4->vtx,
					    uu,
					    vv,
					    4,
					    m_active,
					    patchIndex)))
		{

#if SHARED_LAZY_CACHE == 1
		  sharedLazyTessellationCache.readUnlockThread(threadInfo->id);
#endif

#if defined(ONLY_SHARED_CACHE)
		  t->read_unlock();
#endif
		  ray.geomID = 0;
		  return;
		}
	    }

#if defined(ONLY_SHARED_CACHE)
	  t->read_unlock();
#endif

#if SHARED_LAZY_CACHE == 1
	  sharedLazyTessellationCache.readUnlockThread(threadInfo->id);
#endif



	  //////////////////////////////////////////////////////////////////////////////////////////////////

	}
    }

    // ----------------------------------------------------------------------------------------------------------------
    // ----------------------------------------------------------------------------------------------------------------
    // ----------------------------------------------------------------------------------------------------------------



    typedef BVH4iIntersector16Subdiv SubdivIntersector16SingleMoellerFilter;
    typedef BVH4iIntersector16Subdiv SubdivIntersector16SingleMoellerNoFilter;

    DEFINE_INTERSECTOR16   (BVH4iSubdivMeshIntersector16        , SubdivIntersector16SingleMoellerFilter);
    DEFINE_INTERSECTOR16   (BVH4iSubdivMeshIntersector16NoFilter, SubdivIntersector16SingleMoellerNoFilter);

    typedef BVH4iIntersector1Subdiv SubdivMeshIntersector1MoellerFilter;
    typedef BVH4iIntersector1Subdiv SubdivMeshIntersector1MoellerNoFilter;

    DEFINE_INTERSECTOR1    (BVH4iSubdivMeshIntersector1        , SubdivMeshIntersector1MoellerFilter);
    DEFINE_INTERSECTOR1    (BVH4iSubdivMeshIntersector1NoFilter, SubdivMeshIntersector1MoellerNoFilter);

  }
}
