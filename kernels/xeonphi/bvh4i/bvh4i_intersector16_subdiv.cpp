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
#include "../geometry/subdivpatch1.h"
#include "../../common/subdiv/tessellation_cache.h"

#define TIMER(x) 

namespace embree
{

  __aligned(64) const int Quad3x5::tri_permute_v0[16] = { 0,5,1,6,2,7,3,8, 5,10,6,11,7,12,8,13 };
  __aligned(64) const int Quad3x5::tri_permute_v1[16] = { 5,1,6,2,7,3,8,4, 10,6,11,7,12,8,13,9 };
  __aligned(64) const int Quad3x5::tri_permute_v2[16] = { 1,6,2,7,3,8,4,9, 6,11,7,12,8,13,9,14 };


  static const unsigned int U_BLOCK_SIZE = 5;
  static const unsigned int V_BLOCK_SIZE = 3;

  __forceinline float16 load4x4f_unalign(const void *__restrict__ const ptr0,
				       const void *__restrict__ const ptr1,
				       const void *__restrict__ const ptr2,
				       const void *__restrict__ const ptr3) 
  {
    float16 v = uload16f((float*)ptr0);
    v = uload16f(v,0xf0,(float*)ptr1);
    v = uload16f(v,0xf00,(float*)ptr2);
    v = uload16f(v,0xf000,(float*)ptr3);
    return v;
  }


  static double msec = 0.0;

  namespace isa
  {

    __thread LocalTessellationCacheThreadInfo* localThreadInfo = nullptr;


    static __forceinline size_t extractBVH4iOffset(const size_t &subtree_root)
    {
#if 0
      if (likely(subtree_root & ((size_t)1<<3)))
	return (void*)(subtree_root & ~(((size_t)1 << 4)-1));
      else
	return (void*)(subtree_root & ~(((size_t)1 << 5)-1));
#else
      return subtree_root & ~(((size_t)1 << 6)-1);
#endif
    }

    static __forceinline BVH4i::NodeRef extractBVH4iNodeRef(const size_t &subtree_root)
    {
#if 0
      if (likely(subtree_root & ((size_t)1<<3)))
	return (unsigned int)(subtree_root & (((size_t)1 << 4)-1));
      else
	return (unsigned int)(subtree_root & (((size_t)1 << 5)-1));
#else
      return (unsigned int)(subtree_root & (((size_t)1 << 6)-1));      
#endif
    }


    __forceinline void createSubPatchBVH4iLeaf(BVH4i::NodeRef &ref,
					       const unsigned int patchIndex) 
    {
      *(volatile unsigned int*)&ref = (patchIndex << BVH4i::encodingBits) | BVH4i::leaf_mask;
    }


    BBox3fa createSubTreeCompact(BVH4i::NodeRef &curNode,
				 float16 *const lazymem,
				 const SubdivPatch1 &patch,
				 const float *const grid_array,
				 const size_t grid_array_elements,				 
				 const GridRange &range,
				 unsigned int &localCounter)
    {
      if (range.hasLeafSize())
	{
	  const float *const grid_x_array = grid_array + 0 * grid_array_elements;
	  const float *const grid_y_array = grid_array + 1 * grid_array_elements;
	  const float *const grid_z_array = grid_array + 2 * grid_array_elements;

	  /* compute the bounds just for the range! */

	  unsigned int u_start = range.u_start * (U_BLOCK_SIZE-1);
	  unsigned int v_start = range.v_start * (V_BLOCK_SIZE-1);

	  const unsigned int u_end   = min(u_start+U_BLOCK_SIZE,patch.grid_u_res);
	  const unsigned int v_end   = min(v_start+V_BLOCK_SIZE,patch.grid_v_res);

	  size_t offset = v_start * patch.grid_u_res + u_start;


	  const unsigned int u_size = u_end-u_start;
	  const unsigned int v_size = v_end-v_start;

#if 0
	  PRINT(u_start);
	  PRINT(v_start);
	  PRINT(u_end);
	  PRINT(v_end);
	  PRINT(u_size);
	  PRINT(v_size);
#endif
	  //size_t offset = range.v_start * patch.grid_u_res + range.u_start;

	  //const unsigned int u_size = range.u_end-range.u_start+1;
	  //const unsigned int v_size = range.v_end-range.v_start+1;
	  const bool16 m_mask = ((unsigned int)1 << u_size)-1;

	  float16 min_x = pos_inf;
	  float16 min_y = pos_inf;
	  float16 min_z = pos_inf;
	  float16 max_x = neg_inf;
	  float16 max_y = neg_inf;
	  float16 max_z = neg_inf;

#if 0
	  for (size_t v = 0; v<v_size; v++)
	    {
	      prefetch<PFHINT_NT>(&grid_x_array[ offset ]);
	      prefetch<PFHINT_NT>(&grid_y_array[ offset ]);
	      prefetch<PFHINT_NT>(&grid_z_array[ offset ]);

	      const float16 x = uload16f(&grid_x_array[ offset ]);
	      const float16 y = uload16f(&grid_y_array[ offset ]);
	      const float16 z = uload16f(&grid_z_array[ offset ]);
	      min_x = min(min_x,x);
	      max_x = max(max_x,x);
	      min_y = min(min_y,y);
	      max_y = max(max_y,y);
	      min_z = min(min_z,z);
	      max_z = max(max_z,z);
	      offset += patch.grid_u_res;
	    }	
	  min_x = select(m_mask,min_x,pos_inf);
	  min_y = select(m_mask,min_y,pos_inf);
	  min_z = select(m_mask,min_z,pos_inf);

	  max_x = select(m_mask,max_x,neg_inf);
	  max_y = select(m_mask,max_y,neg_inf);
	  max_z = select(m_mask,max_z,neg_inf);

	  min_x = vreduce_min4(min_x);
	  min_y = vreduce_min4(min_y);
	  min_z = vreduce_min4(min_z);

	  max_x = vreduce_max4(max_x);
	  max_y = vreduce_max4(max_y);
	  max_z = vreduce_max4(max_z);
#else

	  for (size_t v = 0; v<v_size; v++,offset+=patch.grid_u_res)
	    {
	      prefetch<PFHINT_NT>(&grid_x_array[ offset ]);
	      prefetch<PFHINT_NT>(&grid_y_array[ offset ]);
	      prefetch<PFHINT_NT>(&grid_z_array[ offset ]);


#pragma novector
	    for (size_t u = 0; u<u_size; u++)
	      {
		const float x = grid_x_array[ offset + u ];
		const float y = grid_y_array[ offset + u ];
		const float z = grid_z_array[ offset + u ];
		min_x = min(min_x,x);
		min_y = min(min_y,y);
		min_z = min(min_z,z);
		max_x = max(max_x,x);
		max_y = max(max_y,y);
		max_z = max(max_z,z);

	      }
	    }

#endif

	  BBox3fa bounds;
	  store1f(&bounds.lower.x,min_x);
	  store1f(&bounds.lower.y,min_y);
	  store1f(&bounds.lower.z,min_z);
	  store1f(&bounds.upper.x,max_x);
	  store1f(&bounds.upper.y,max_y);
	  store1f(&bounds.upper.z,max_z);

#if 1
          if (unlikely(u_size < 5)) 
	    { 
	      const unsigned int delta_u = 5 - u_size;
	      if (u_start >= delta_u) u_start -= delta_u; else u_start = 0;
	    }

          if (unlikely(v_size < 3)) 
	    { 
	      const unsigned int delta_v = 3 - v_size;
	      if (v_start >= delta_v) v_start -= delta_v; else v_start = 0;
	    }
#endif

	  const size_t grid_offset4x4 = v_start * patch.grid_u_res + u_start;

	  const size_t offset_bytes = (size_t)&grid_x_array[ grid_offset4x4 ] - (size_t)lazymem; //(size_t)SharedLazyTessellationCache::sharedLazyTessellationCache.getDataPtr();
	  createSubPatchBVH4iLeaf( curNode, offset_bytes);	  
	  return bounds;
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
	  BBox3fa bounds_subtree = createSubTreeCompact( node.child(i), 
							 lazymem, 
							 patch, 
							 grid_array,
							 grid_array_elements,
							 r[i],
							 localCounter);
	  node.setBounds(i, bounds_subtree);
	  bounds.extend( bounds_subtree );
	}
      return bounds;      
    }



    BVH4i::NodeRef initLocalLazySubdivTreeCompact(const SubdivPatch1 &patch,
						  unsigned int currentIndex,
						  float16 *basemem,
						  const SubdivMesh* const geom)
    {
      __aligned(64) float grid_u[(patch.grid_size_simd_blocks+1)*16]; // for unaligned access
      __aligned(64) float grid_v[(patch.grid_size_simd_blocks+1)*16];

      float16 *lazymem = &basemem[currentIndex];

      TIMER(double msec);
      TIMER(msec = getSeconds());    

      const size_t array_elements = patch.grid_size_simd_blocks * 16;

      const size_t grid_offset = patch.grid_bvh_size_64b_blocks * 16;

      float *const grid_x  = (float*)lazymem + grid_offset + 0 * array_elements;
      float *const grid_y  = (float*)lazymem + grid_offset + 1 * array_elements;
      float *const grid_z  = (float*)lazymem + grid_offset + 2 * array_elements;
      int   *const grid_uv = (int*)  lazymem + grid_offset + 3 * array_elements;

      assert( patch.grid_subtree_size_64b_blocks * 16 >= grid_offset + 4 * array_elements);

      evalGrid(patch,grid_x,grid_y,grid_z,grid_u,grid_v,geom);
      
      for (size_t i=0;i<array_elements;i+=16)
	{
	  prefetch<PFHINT_L1EX>(&grid_uv[i]);
	  const float16 u = load16f(&grid_u[i]);
	  const float16 v = load16f(&grid_v[i]);
	  const int16 u_i = int16(u * 65535.0f/2.0f);
	  const int16 v_i = int16(v * 65535.0f/2.0f);
	  const int16 uv_i = (u_i << 16) | v_i;
	  store16i(&grid_uv[i],uv_i);
	}

#if 0
      TIMER(msec = getSeconds()-msec);    
      TIMER(PRINT("tess"));
      TIMER(PRINT(patch.grid_u_res));
      TIMER(PRINT(patch.grid_v_res));
      TIMER(PRINT(1000.0f * msec));
      TIMER(msec = getSeconds());    
#endif

      BVH4i::NodeRef subtree_root = 0;
      const unsigned int oldIndex = currentIndex;

      const unsigned int grid_u_blocks = (patch.grid_u_res + U_BLOCK_SIZE-2) / (U_BLOCK_SIZE-1);
      const unsigned int grid_v_blocks = (patch.grid_v_res + V_BLOCK_SIZE-2) / (V_BLOCK_SIZE-1);

      //PRINT(grid_u_blocks);
      //PRINT(grid_v_blocks);

      unsigned int localCounter = 0;
      BBox3fa bounds = createSubTreeCompact( subtree_root,
					     basemem,
					     patch,
					     grid_x,
					     array_elements,
					     GridRange(0,grid_u_blocks,0,grid_v_blocks),
					     localCounter);

      //assert(currentIndex - oldIndex == patch.grid_bvh_size_64b_blocks);
      assert(localCounter == patch.grid_bvh_size_64b_blocks);
      TIMER(msec = getSeconds()-msec);    
      TIMER(PRINT("tess+bvh"));
      TIMER(PRINT(patch.grid_u_res));
      TIMER(PRINT(patch.grid_v_res));
      TIMER(PRINT(patch.grid_subtree_size_64b_blocks*64));

      TIMER(PRINT(1000.0f * msec));
      TIMER(double throughput = 1.0 / (1000*msec));

      TIMER(msec = getSeconds());    
      TIMER(PRINT(throughput));

      return subtree_root;
    }


    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////



    __forceinline size_t lazyBuildPatch(const unsigned int patchIndex,
					const unsigned int commitCounter,
					SubdivPatch1* const patches,
					Scene *const scene,
					LocalTessellationCacheThreadInfo *threadInfo,
					BVH4i* bvh)
    {
#if NEW_TCACHE_SYNC == 1
      while(1)
	{
	  /* per thread lock */
	 SharedLazyTessellationCache::sharedLazyTessellationCache.lockThread(threadInfo->id);	       

	 SubdivPatch1* subdiv_patch = &patches[patchIndex];
      
	  static const size_t REF_TAG      = 1;
	  static const size_t REF_TAG_MASK = (~REF_TAG) & 0xffffffff;
	  
	  /* fast path for cache hit */
	  {
	    CACHE_STATS(SharedTessellationCacheStats::cache_accesses++);
	    const size_t subdiv_patch_root_ref    = subdiv_patch->root_ref.data;
	    
	    if (likely(subdiv_patch_root_ref)) 
	      {
		const size_t subdiv_patch_root = (subdiv_patch_root_ref & REF_TAG_MASK); // + (size_t)SharedLazyTessellationCache::sharedLazyTessellationCache.getDataPtr();
		const size_t subdiv_patch_cache_index = subdiv_patch_root_ref >> 32;
		
		if (likely( SharedLazyTessellationCache::sharedLazyTessellationCache.validCacheIndex(subdiv_patch_cache_index) ))
		  {
		    CACHE_STATS(SharedTessellationCacheStats::cache_hits++);	      
		    return subdiv_patch_root;
		  }
	      }
	  }

	  /* cache miss */
	  CACHE_STATS(SharedTessellationCacheStats::cache_misses++);
	  unsigned int lock = SharedLazyTessellationCache::sharedLazyTessellationCache.unlockThread(threadInfo->id);		  
	  if (unlikely(lock == 1))
	      SharedLazyTessellationCache::sharedLazyTessellationCache.waitForUsersLessEqual(threadInfo->id,0);

	  subdiv_patch->write_lock();
	  {
	    const size_t subdiv_patch_root_ref    = subdiv_patch->root_ref.data;
	    const size_t subdiv_patch_cache_index = subdiv_patch_root_ref >> 32;

	    /* do we still need to create the subtree data? */
	    if (subdiv_patch_root_ref == 0 || !SharedLazyTessellationCache::sharedLazyTessellationCache.validCacheIndex(subdiv_patch_cache_index))
	      {	      
		const SubdivMesh* const geom = (SubdivMesh*)scene->get(subdiv_patch->geom); 

		size_t block_index = block_index = SharedLazyTessellationCache::sharedLazyTessellationCache.alloc(subdiv_patch->grid_subtree_size_64b_blocks);
		if (block_index == (size_t)-1)
		  {
		    /* cannot allocate => flush the cache */
		    subdiv_patch->write_unlock();
		    SharedLazyTessellationCache::sharedLazyTessellationCache.resetCache();
		    continue;
		    
		  }
		float16* local_mem   = (float16*)SharedLazyTessellationCache::sharedLazyTessellationCache.getBlockPtr(block_index);
		unsigned int currentIndex = 0;
		BVH4i::NodeRef bvh4i_root = initLocalLazySubdivTreeCompact(*subdiv_patch,currentIndex,local_mem,geom);
		size_t new_root_ref = (size_t)bvh4i_root + (size_t)local_mem - (size_t)SharedLazyTessellationCache::sharedLazyTessellationCache.getDataPtr();
	    

		assert( !(new_root_ref & REF_TAG) );
		new_root_ref |= REF_TAG;
		new_root_ref |= SharedLazyTessellationCache::sharedLazyTessellationCache.getCurrentIndex() << 32; 
		subdiv_patch->root_ref.data = new_root_ref;
	      }
	  }
	  subdiv_patch->write_unlock();
	}
#else

#if 1
       while(1)
	{
	  /* per thread lock */
	  while(1)
	    {
	      unsigned int lock = SharedLazyTessellationCache::sharedLazyTessellationCache.lockThread(threadInfo->id);	       
	      if (unlikely(lock == 1))
		{
		  /* lock failed wait until sync phase is over */
		  SharedLazyTessellationCache::sharedLazyTessellationCache.unlockThread(threadInfo->id);	       
		  SharedLazyTessellationCache::sharedLazyTessellationCache.waitForUsersLessEqual(threadInfo->id,0);
		}
	      else
		break;
	    }

	  SubdivPatch1* subdiv_patch = &patches[patchIndex];
      
	  static const size_t REF_TAG      = 1;
	  static const size_t REF_TAG_MASK = (~REF_TAG) & 0xffffffff;

	  /* fast path for cache hit */
	  {
	    CACHE_STATS(SharedTessellationCacheStats::cache_accesses++);
	    const size_t subdiv_patch_root_ref    = subdiv_patch->root_ref.data;
	    
	    if (likely(subdiv_patch_root_ref)) 
	      {
		const size_t subdiv_patch_root = (subdiv_patch_root_ref & REF_TAG_MASK); // + (size_t)SharedLazyTessellationCache::sharedLazyTessellationCache.getDataPtr();
		const size_t subdiv_patch_cache_index = subdiv_patch_root_ref >> 32;
		
		if (likely( SharedLazyTessellationCache::sharedLazyTessellationCache.validCacheIndex(subdiv_patch_cache_index) ))
		  {
		    CACHE_STATS(SharedTessellationCacheStats::cache_hits++);	      
		    return subdiv_patch_root;
		  }
	      }
	  }

	  /* cache miss */
	  CACHE_STATS(SharedTessellationCacheStats::cache_misses++);

	  subdiv_patch->write_lock();
	  {
	    const size_t subdiv_patch_root_ref    = subdiv_patch->root_ref.data;
	    const size_t subdiv_patch_cache_index = subdiv_patch_root_ref >> 32;

	    /* do we still need to create the subtree data? */
	    if (subdiv_patch_root_ref == 0 || !SharedLazyTessellationCache::sharedLazyTessellationCache.validCacheIndex(subdiv_patch_cache_index))
	      {	      
		const SubdivMesh* const geom = (SubdivMesh*)scene->get(subdiv_patch->geom); 
		size_t block_index = SharedLazyTessellationCache::sharedLazyTessellationCache.alloc(subdiv_patch->grid_subtree_size_64b_blocks);
		if (block_index == (size_t)-1)
		  {
		    /* cannot allocate => flush the cache */
		    subdiv_patch->write_unlock();
		    SharedLazyTessellationCache::sharedLazyTessellationCache.unlockThread(threadInfo->id);		  
		    SharedLazyTessellationCache::sharedLazyTessellationCache.resetCache();
		    continue;
		  }
		float16* local_mem   = (float16*)SharedLazyTessellationCache::sharedLazyTessellationCache.getBlockPtr(block_index);
		unsigned int currentIndex = 0;
		BVH4i::NodeRef bvh4i_root = initLocalLazySubdivTreeCompact(*subdiv_patch,currentIndex,local_mem,geom);
		size_t new_root_ref = (size_t)bvh4i_root + (size_t)local_mem - (size_t)SharedLazyTessellationCache::sharedLazyTessellationCache.getDataPtr();

		CACHE_STATS(SharedTessellationCacheStats::incPatchBuild(patchIndex,bvh->numPrimitives));

		assert( !(new_root_ref & REF_TAG) );
		new_root_ref |= REF_TAG;
		new_root_ref |= SharedLazyTessellationCache::sharedLazyTessellationCache.getCurrentIndex() << 32; 
		subdiv_patch->root_ref.data = new_root_ref;
	      }
	  }
	  subdiv_patch->write_unlock();
	  SharedLazyTessellationCache::sharedLazyTessellationCache.unlockThread(threadInfo->id);		  
	}	      

#else

      while(1)
      {
        SharedLazyTessellationCache::sharedLazyTessellationCache.lockThreadLoop(threadInfo->id);
        
        SubdivPatch1* subdiv_patch = &patches[patchIndex];
        
        const size_t index = SharedLazyTessellationCache::lookupIndex(&subdiv_patch->root_ref);
        if (index != -1) return index;
        
        if (subdiv_patch->try_write_lock())
        {
          const SubdivMesh* const geom = (SubdivMesh*)scene->get(subdiv_patch->geom); 
          size_t block_index = SharedLazyTessellationCache::sharedLazyTessellationCache.allocIndexLoop(threadInfo->id,subdiv_patch->grid_subtree_size_64b_blocks);
          float16* local_mem   = (float16*)SharedLazyTessellationCache::sharedLazyTessellationCache.getBlockPtr(block_index);
          unsigned int currentIndex = 0;
          BVH4i::NodeRef bvh4i_root = initLocalLazySubdivTreeCompact(*subdiv_patch,currentIndex,local_mem,geom);
          size_t new_root_ref = (size_t)bvh4i_root + (size_t)local_mem; // - (size_t)SharedLazyTessellationCache::sharedLazyTessellationCache.getDataPtr();
          
          CACHE_STATS(SharedTessellationCacheStats::incPatchBuild(patchIndex,bvh->numPrimitives));
          
          subdiv_patch->root_ref = SharedLazyTessellationCache::Tag((void*)new_root_ref);
          subdiv_patch->write_unlock();
        }
        SharedLazyTessellationCache::sharedLazyTessellationCache.unlockThread(threadInfo->id);		  
      }	      
#endif

#endif
    }
    

    static unsigned int BVH4I_LEAF_MASK = BVH4i::leaf_mask; // needed due to compiler efficiency bug
    static unsigned int M_LANE_7777 = 0x7777;               // needed due to compiler efficiency bug

    // ============================================================================================
    // ============================================================================================
    // ============================================================================================


    void BVH4iIntersector16Subdiv::intersect(int16* valid_i, BVH4i* bvh, Ray16& ray16)
    {
      /* near and node stack */
      __aligned(64) float   stack_dist[4*BVH4i::maxDepth+1];
      __aligned(64) NodeRef stack_node[4*BVH4i::maxDepth+1];

      /* setup */
      const bool16 m_valid    = *(int16*)valid_i != int16(0);
      const Vec3f16 rdir16     = rcp_safe(ray16.dir);
      const float16 inf        = float16(pos_inf);
      const float16 zero       = float16::zero();

      store16f(stack_dist,inf);
      ray16.primID = select(m_valid,int16(-1),ray16.primID);
      ray16.geomID = select(m_valid,int16(-1),ray16.geomID);

      Scene *const scene                         = (Scene*)bvh->geometry;
      const Node      * __restrict__ const nodes = (Node     *)bvh->nodePtr();
      Triangle1 * __restrict__ const accel       = (Triangle1*)bvh->triPtr();
      const unsigned int commitCounter           = scene->commitCounter;

      LocalTessellationCacheThreadInfo *threadInfo = nullptr;
      if (unlikely(!localThreadInfo))
	{
	  const unsigned int id = SharedLazyTessellationCache::sharedLazyTessellationCache.getNextRenderThreadID();
	  localThreadInfo = new LocalTessellationCacheThreadInfo( id );
	}
      threadInfo = localThreadInfo;

      stack_node[0] = BVH4i::invalidNode;
      long rayIndex = -1;
      while((rayIndex = bitscan64(rayIndex,toInt(m_valid))) != BITSCAN_NO_BIT_SET_64)	    
        {

	  STAT3(normal.travs,1,1,1);

	  stack_node[1] = bvh->root;
	  size_t sindex = 2;

	  const float16 org_xyz      = loadAOS4to16f(rayIndex,ray16.org.x,ray16.org.y,ray16.org.z);
	  const float16 dir_xyz      = loadAOS4to16f(rayIndex,ray16.dir.x,ray16.dir.y,ray16.dir.z);
	  const float16 rdir_xyz     = loadAOS4to16f(rayIndex,rdir16.x,rdir16.y,rdir16.z);
	  //const float16 org_rdir_xyz = org_xyz * rdir_xyz;
	  const float16 min_dist_xyz = broadcast1to16f(&ray16.tnear[rayIndex]);
	  float16       max_dist_xyz = broadcast1to16f(&ray16.tfar[rayIndex]);

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

	      //////////////////////////////////////////////////////////////////////////////////////////////////


	      // ----------------------------------------------------------------------------------------------------
	      const unsigned int patchIndex = curNode.offsetIndex();
	      SubdivPatch1 &patch = ((SubdivPatch1*)accel)[patchIndex];
	      const size_t cached_64bit_root = lazyBuildPatch(patchIndex,commitCounter,(SubdivPatch1*)accel,scene,threadInfo,bvh);
	      const BVH4i::NodeRef subtree_root = extractBVH4iNodeRef(cached_64bit_root); 
	      float *const lazyCachePtr = (float*)((size_t)SharedLazyTessellationCache::sharedLazyTessellationCache.getDataPtr() + (size_t)extractBVH4iOffset(cached_64bit_root));
	      // ----------------------------------------------------------------------------------------------------

	      STAT3(normal.trav_prims,1,1,1);

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
							 (BVH4i::Node*)lazyCachePtr,
							 leaf_mask);
		 		   

		  /* return if stack is empty */
		  if (unlikely(curNode == BVH4i::invalidNode)) break;

		  Quad3x5 quad3x5;
		  quad3x5.init( curNode.offsetIndex(), patch, lazyCachePtr);
		  quad3x5.intersect1_tri16_precise(rayIndex,dir_xyz,org_xyz,ray16,precalculations,patchIndex);		  
		}

	      SharedLazyTessellationCache::sharedLazyTessellationCache.unlockThread(threadInfo->id);

	      // -------------------------------------
	      // -------------------------------------
	      // -------------------------------------

	      compactStack(stack_node,stack_dist,sindex,float16(ray16.tfar[rayIndex]));

	      // ------------------------
	    }
	}


      /* update primID/geomID and compute normals */
      bool16 m_hit = (ray16.primID != -1) & m_valid;
      rayIndex = -1;
      while((rayIndex = bitscan64(rayIndex,toInt(m_hit))) != BITSCAN_NO_BIT_SET_64)	    
        {
	  const SubdivPatch1& subdiv_patch = ((SubdivPatch1*)accel)[ray16.primID[rayIndex]];
	  ray16.primID[rayIndex] = subdiv_patch.prim;
	  ray16.geomID[rayIndex] = subdiv_patch.geom;
#if defined(RTCORE_RETURN_SUBDIV_NORMAL)

	  if (unlikely(!subdiv_patch.hasDisplacement()))
	    {
	      const Vec3fa normal    = subdiv_patch.normal(ray16.v[rayIndex],ray16.u[rayIndex]);
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
	  
	  const float patch_u = bilinear_interpolate(uv0.x,uv1.x,uv2.x,uv3.x,ray16.v[rayIndex],ray16.u[rayIndex]);
	  const float patch_v = bilinear_interpolate(uv0.y,uv1.y,uv2.y,uv3.y,ray16.v[rayIndex],ray16.u[rayIndex]);
	  ray16.u[rayIndex] = patch_u;
	  ray16.v[rayIndex] = patch_v;
#endif
	}

    }

    void BVH4iIntersector16Subdiv::occluded(int16* valid_i, BVH4i* bvh, Ray16& ray16)
    {
      /* near and node stack */
      __aligned(64) NodeRef stack_node[4*BVH4i::maxDepth+1];

      /* setup */
      const bool16 m_valid = *(int16*)valid_i != int16(0);
      const Vec3f16 rdir16  = rcp_safe(ray16.dir);
      bool16 terminated    = !m_valid;
      const float16 inf     = float16(pos_inf);
      const float16 zero    = float16::zero();

      Scene *const scene                   = (Scene*)bvh->geometry;
      const Node      * __restrict__ nodes = (Node     *)bvh->nodePtr();
      const Triangle1 * __restrict__ accel = (Triangle1*)bvh->triPtr();
      const unsigned int commitCounter           = scene->commitCounter;

      LocalTessellationCacheThreadInfo *threadInfo = nullptr;
      if (unlikely(!localThreadInfo))
	{
	  const unsigned int id = SharedLazyTessellationCache::sharedLazyTessellationCache.getNextRenderThreadID();
	  localThreadInfo = new LocalTessellationCacheThreadInfo( id );
	}
      threadInfo = localThreadInfo;

      stack_node[0] = BVH4i::invalidNode;
      ray16.primID = select(m_valid,int16(-1),ray16.primID);
      ray16.geomID = select(m_valid,int16(-1),ray16.geomID);

      long rayIndex = -1;
      while((rayIndex = bitscan64(rayIndex,toInt(m_valid))) != BITSCAN_NO_BIT_SET_64)	    
        {
	  stack_node[1] = bvh->root;
	  size_t sindex = 2;

	  STAT3(shadow.travs,1,1,1);

	  const float16 org_xyz      = loadAOS4to16f(rayIndex,ray16.org.x,ray16.org.y,ray16.org.z);
	  const float16 dir_xyz      = loadAOS4to16f(rayIndex,ray16.dir.x,ray16.dir.y,ray16.dir.z);
	  const float16 rdir_xyz     = loadAOS4to16f(rayIndex,rdir16.x,rdir16.y,rdir16.z);
	  //const float16 org_rdir_xyz = org_xyz * rdir_xyz;
	  const float16 min_dist_xyz = broadcast1to16f(&ray16.tnear[rayIndex]);
	  const float16 max_dist_xyz = broadcast1to16f(&ray16.tfar[rayIndex]);
	  const int16 v_invalidNode(BVH4i::invalidNode);
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

	      //////////////////////////////////////////////////////////////////////////////////////////////////

	      STAT3(shadow.trav_prims,1,1,1);
	      
	      // ----------------------------------------------------------------------------------------------------
	      const unsigned int patchIndex = curNode.offsetIndex();
	      SubdivPatch1 &patch = ((SubdivPatch1*)accel)[patchIndex];
	      const size_t cached_64bit_root = lazyBuildPatch(patchIndex,commitCounter,(SubdivPatch1*)accel,scene,threadInfo,bvh);
	      const BVH4i::NodeRef subtree_root = extractBVH4iNodeRef(cached_64bit_root); 
	      float *const lazyCachePtr = (float*)((size_t)SharedLazyTessellationCache::sharedLazyTessellationCache.getDataPtr() + (size_t)extractBVH4iOffset(cached_64bit_root));
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
							  (BVH4i::Node*)lazyCachePtr,
							  leaf_mask);
		 		   

		    /* return if stack is empty */
		    if (unlikely(curNode == BVH4i::invalidNode)) break;

		    Quad3x5 quad3x5;
		    quad3x5.init( curNode.offsetIndex(), patch, lazyCachePtr);
		    if (unlikely(quad3x5.occluded1_tri16_precise(rayIndex,
								 dir_xyz,
								 org_xyz,
								 ray16,
								 precalculations)))
		      {
			terminated |= (bool16)((unsigned int)1 << rayIndex);
			break;
		      }		  
		  }

		SharedLazyTessellationCache::sharedLazyTessellationCache.unlockThread(threadInfo->id);
	      }

	      if (unlikely(terminated & (bool16)((unsigned int)1 << rayIndex))) break;
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

      STAT3(normal.travs,1,1,1);

      /* setup */
      const Vec3f16 rdir16     = rcp_safe(Vec3f16(float16(ray.dir.x),float16(ray.dir.y),float16(ray.dir.z)));
      const float16 inf        = float16(pos_inf);
      const float16 zero       = float16::zero();

      const unsigned int oldID = ray.primID;

      store16f(stack_dist,inf);

      const Node      * __restrict__ nodes = (Node    *)bvh->nodePtr();
      const Triangle1 * __restrict__ accel = (Triangle1*)bvh->triPtr();
      Scene *const scene                   = (Scene*)bvh->geometry;
      const unsigned int commitCounter     = scene->commitCounter;

      LocalTessellationCacheThreadInfo *threadInfo = nullptr;
      if (unlikely(!localThreadInfo))
	{
	  const unsigned int id = SharedLazyTessellationCache::sharedLazyTessellationCache.getNextRenderThreadID();
	  localThreadInfo = new LocalTessellationCacheThreadInfo( id );
	}
      threadInfo = localThreadInfo;

      stack_node[0] = BVH4i::invalidNode;      
      stack_node[1] = bvh->root;

      size_t sindex = 2;

      const float16 org_xyz      = loadAOS4to16f(ray.org.x,ray.org.y,ray.org.z);
      const float16 dir_xyz      = loadAOS4to16f(ray.dir.x,ray.dir.y,ray.dir.z);
      const float16 rdir_xyz     = loadAOS4to16f(rdir16.x[0],rdir16.y[0],rdir16.z[0]);
      //const float16 org_rdir_xyz = org_xyz * rdir_xyz;
      const float16 min_dist_xyz = broadcast1to16f(&ray.tnear);
      float16       max_dist_xyz = broadcast1to16f(&ray.tfar);
	  
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



	  //////////////////////////////////////////////////////////////////////////////////////////////////

	  // ----------------------------------------------------------------------------------------------------
	  const unsigned int patchIndex = curNode.offsetIndex();
	  SubdivPatch1 &patch = ((SubdivPatch1*)accel)[patchIndex];
	  const size_t cached_64bit_root = lazyBuildPatch(patchIndex,commitCounter,(SubdivPatch1*)accel,scene,threadInfo,bvh);
	  const BVH4i::NodeRef subtree_root = extractBVH4iNodeRef(cached_64bit_root); 
	  float *const lazyCachePtr = (float*)((size_t)SharedLazyTessellationCache::sharedLazyTessellationCache.getDataPtr() + (size_t)extractBVH4iOffset(cached_64bit_root));
	  // ----------------------------------------------------------------------------------------------------

	  STAT3(normal.trav_prims,1,1,1);

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
						    (BVH4i::Node*)lazyCachePtr,
						    leaf_mask);
		 		   

	      /* return if stack is empty */
	      if (unlikely(curNode == BVH4i::invalidNode)) break;

	      Quad3x5 quad3x5;
	      quad3x5.init( curNode.offsetIndex(), patch, lazyCachePtr);
	      quad3x5.intersect1_tri16_precise(dir_xyz,org_xyz,ray,precalculations,patchIndex);		  
	    }

	  SharedLazyTessellationCache::sharedLazyTessellationCache.unlockThread(threadInfo->id);
	  compactStack(stack_node,stack_dist,sindex,max_dist_xyz);
	}

      if (oldID != ray.primID)
	{
	  /* update primID/geomID and compute normals */
	  const SubdivPatch1& subdiv_patch = ((SubdivPatch1*)accel)[ray.primID];
	  ray.primID = subdiv_patch.prim;
	  ray.geomID = subdiv_patch.geom;
#if defined(RTCORE_RETURN_SUBDIV_NORMAL)

	  if (unlikely(!subdiv_patch.hasDisplacement()))
	    {
	      const Vec3fa normal = subdiv_patch.normal(ray.v,ray.u);
	      ray.Ng.x = normal.x;
	      ray.Ng.y = normal.y;
	      ray.Ng.z = normal.z;
	    }
#endif

#if FORCE_TRIANGLE_UV == 0

	  const Vec2f uv0 = subdiv_patch.getUV(0);
	  const Vec2f uv1 = subdiv_patch.getUV(1);
	  const Vec2f uv2 = subdiv_patch.getUV(2);
	  const Vec2f uv3 = subdiv_patch.getUV(3);
	  
	  const float patch_u = bilinear_interpolate(uv0.x,uv1.x,uv2.x,uv3.x,ray.v,ray.u);
	  const float patch_v = bilinear_interpolate(uv0.y,uv1.y,uv2.y,uv3.y,ray.v,ray.u);
	  ray.u = patch_u;
	  ray.v = patch_v;
#endif
	}

    }

    void BVH4iIntersector1Subdiv::occluded(BVH4i* bvh, Ray& ray)
    {
      /* near and node stack */
      __aligned(64) NodeRef stack_node[3*BVH4i::maxDepth+1];

      STAT3(shadow.travs,1,1,1);

      /* setup */
      const Vec3f16 rdir16      = rcp_safe(Vec3f16(ray.dir.x,ray.dir.y,ray.dir.z));
      const float16 inf         = float16(pos_inf);
      const float16 zero        = float16::zero();

      const Node      * __restrict__ nodes = (Node     *)bvh->nodePtr();
      const Triangle1 * __restrict__ accel = (Triangle1*)bvh->triPtr();
      Scene *const scene                   = (Scene*)bvh->geometry;
      const unsigned int commitCounter     = scene->commitCounter;

      LocalTessellationCacheThreadInfo *threadInfo = nullptr;
      if (unlikely(!localThreadInfo))
	{
	  const unsigned int id = SharedLazyTessellationCache::sharedLazyTessellationCache.getNextRenderThreadID();
	  localThreadInfo = new LocalTessellationCacheThreadInfo( id );
	}
      threadInfo = localThreadInfo;

      stack_node[0] = BVH4i::invalidNode;
      stack_node[1] = bvh->root;
      size_t sindex = 2;

      const float16 org_xyz      = loadAOS4to16f(ray.org.x,ray.org.y,ray.org.z);
      const float16 dir_xyz      = loadAOS4to16f(ray.dir.x,ray.dir.y,ray.dir.z);
      const float16 rdir_xyz     = loadAOS4to16f(rdir16.x[0],rdir16.y[0],rdir16.z[0]);
      //const float16 org_rdir_xyz = org_xyz * rdir_xyz;
      const float16 min_dist_xyz = broadcast1to16f(&ray.tnear);
      const float16 max_dist_xyz = broadcast1to16f(&ray.tfar);

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

	  //////////////////////////////////////////////////////////////////////////////////////////////////

	 
	  STAT3(shadow.trav_prims,1,1,1);
     
	  // ----------------------------------------------------------------------------------------------------
	  const unsigned int patchIndex = curNode.offsetIndex();
	  SubdivPatch1 &patch = ((SubdivPatch1*)accel)[patchIndex];
	  const size_t cached_64bit_root = lazyBuildPatch(patchIndex,commitCounter,(SubdivPatch1*)accel,scene,threadInfo,bvh);
	  const BVH4i::NodeRef subtree_root = extractBVH4iNodeRef(cached_64bit_root); 
	  float *const lazyCachePtr = (float*)((size_t)SharedLazyTessellationCache::sharedLazyTessellationCache.getDataPtr() + (size_t)extractBVH4iOffset(cached_64bit_root));
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
						   (BVH4i::Node*)lazyCachePtr,
						   leaf_mask);
		 		   

	      /* return if stack is empty */
	      if (unlikely(curNode == BVH4i::invalidNode)) break;

	      Quad3x5 quad3x5;
	      quad3x5.init( curNode.offsetIndex(), patch, lazyCachePtr);
	      if (unlikely(quad3x5.occluded1_tri16_precise(dir_xyz,
							   org_xyz,
							   ray,
							   precalculations)))
		{
		  SharedLazyTessellationCache::sharedLazyTessellationCache.unlockThread(threadInfo->id);
		  ray.geomID = 0;
		  return;
		}		  
	    }


	  SharedLazyTessellationCache::sharedLazyTessellationCache.unlockThread(threadInfo->id);



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
