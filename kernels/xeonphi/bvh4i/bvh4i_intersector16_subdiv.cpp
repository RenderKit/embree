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

// FIXME: instead of 4x4 better 3x5?

namespace embree
{

  static double msec = 0.0;

  namespace isa
  {

    __thread LocalTessellationCacheThreadInfo* localThreadInfo = NULL;


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

      __forceinline void init(size_t offset_bytes, 
			      const SubdivPatch1Base &patch,
			      float *lazyCachePtr)
      {
	const size_t dim_offset    = patch.grid_size_simd_blocks * 16;
	const size_t line_offset   = patch.grid_u_res;
	const float *const grid_x  = (float*)(offset_bytes + (size_t)lazyCachePtr);
	const float *const grid_y  = grid_x + 1 * dim_offset;
	const float *const grid_z  = grid_x + 2 * dim_offset;
	const float *const grid_uv = grid_x + 3 * dim_offset;
	const size_t offset0 = 0*line_offset;
	const size_t offset1 = 1*line_offset;
	const size_t offset2 = 2*line_offset;
	const size_t offset3 = 3*line_offset;

	vtx.x = gather16f_4f_unalign(&grid_x[offset0],&grid_x[offset1],&grid_x[offset2],&grid_x[offset3]);
	vtx.y = gather16f_4f_unalign(&grid_y[offset0],&grid_y[offset1],&grid_y[offset2],&grid_y[offset3]);
	vtx.z = gather16f_4f_unalign(&grid_z[offset0],&grid_z[offset1],&grid_z[offset2],&grid_z[offset3]);

	const mic_f uv = gather16f_4f_unalign(&grid_uv[offset0],&grid_uv[offset1],&grid_uv[offset2],&grid_uv[offset3]);
	store16f((float*)uu,uv);
       
      }
      __forceinline void prefetchData() const
      {
	prefetch<PFHINT_NT>(&vtx.x);
	prefetch<PFHINT_NT>(&vtx.y);
	prefetch<PFHINT_NT>(&vtx.z);
	prefetch<PFHINT_NT>(uu);
      }
      
      __forceinline mic_f getU() const
      {
#if COMPACT == 1
	mic_i uv = load16i((int*)uu);
	mic_i  u = uv & 0xffff;
	return mic_f(u) * 2.0f/65535.0f;
#else
	return load16f_uint16(uu);
#endif
      }

      __forceinline mic_f getV() const
      {
#if COMPACT == 1
	mic_i uv = load16i((int*)uu);
	mic_i  v = uv >> 16;
	return mic_f(v) * 2.0f/65535.0f;
#else
	return load16f_uint16(vv);
#endif
      }

      __forceinline void set(const mic3f &v3, const mic_f &u, const mic_f &v)
      {
	__aligned(64) unsigned short local[32]; 
	store16f_uint16(local +  0,u*65535.0f);
	store16f_uint16(local + 16,v*65535.0f);
	store16f_ngo(&vtx.x,v3.x);
	store16f_ngo(&vtx.y,v3.y);
	store16f_ngo(&vtx.z,v3.z);
	store16f_ngo(&uu,load16f(local));
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


    BBox3fa createSubTreeCompact(BVH4i::NodeRef &curNode,
				 mic_f *const lazymem,
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
	  //BBox3fa bounds( empty );
	  size_t offset = range.v_start * patch.grid_u_res + range.u_start;
	  mic_f min_x = pos_inf;
	  mic_f min_y = pos_inf;
	  mic_f min_z = pos_inf;
	  mic_f max_x = neg_inf;
	  mic_f max_y = neg_inf;
	  mic_f max_z = neg_inf;

	  const unsigned int u_size = range.u_end-range.u_start+1;
	  const unsigned int v_size = range.v_end-range.v_start+1;
	  const mic_m m_mask = ((unsigned int)1 << u_size)-1;

#if 0
	  size_t local_offset = 0;
	  for (size_t v = 0; v<v_size; v++)
	    {
	      const mic_f x = uload16f(&grid_x_array[ offset ]);
	      const mic_f y = uload16f(&grid_y_array[ offset ]);
	      const mic_f z = uload16f(&grid_z_array[ offset ]);
	      compactustore16f_low(m_mask,&min_x[local_offset],x);
	      compactustore16f_low(m_mask,&max_x[local_offset],x);
	      compactustore16f_low(m_mask,&min_y[local_offset],y);
	      compactustore16f_low(m_mask,&max_y[local_offset],y);
	      compactustore16f_low(m_mask,&min_z[local_offset],z);
	      compactustore16f_low(m_mask,&max_z[local_offset],z);
	      offset       += patch.grid_u_res;
	      local_offset += u_size;
	    }	  
	  min_x = vreduce_min(min_x);
	  min_y = vreduce_min(min_y);
	  min_z = vreduce_min(min_z);

	  max_x = vreduce_max(max_x);
	  max_y = vreduce_max(max_y);
	  max_z = vreduce_max(max_z);
#else
	  for (size_t v = 0; v<v_size; v++,offset+=patch.grid_u_res)
	    {
#pragma novector
	    for (size_t u = 0; u<u_size; u++)
	      {
		const float x = grid_x_array[ offset + u ];
		const float y = grid_y_array[ offset + u ];
		const float z = grid_z_array[ offset + u ];
		//bounds.extend( Vec3fa(x,y,z) );
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

	  


	  unsigned int u_start = range.u_start;
	  unsigned int v_start = range.v_start;

	  const unsigned int u_end   = range.u_end;
	  const unsigned int v_end   = range.v_end;

          if (unlikely(u_end-u_start+1 < 4)) 
	    { 
	      const unsigned int delta_u = 4 - (u_end-u_start+1);
	      if (u_start >= delta_u) 
		u_start -= delta_u; 
	      else
		u_start = 0;
	    }
          if (unlikely(v_end-v_start+1 < 4)) 
	    { 
	      const unsigned int delta_v = 4 - (v_end-v_start+1);
	      if (v_start >= delta_v) 
		v_start -= delta_v; 
	      else
		v_start = 0;
	    }


	  
	  const size_t grid_offset4x4    = v_start * patch.grid_u_res + u_start;

	  const size_t offset_bytes = (size_t)&grid_x_array[ grid_offset4x4 ] - (size_t)SharedLazyTessellationCache::sharedLazyTessellationCache.getDataPtr();
          //assert( (value & 2) == 0 );

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


    BVH4i::NodeRef initLocalLazySubdivTree(const SubdivPatch1 &patch,
					   unsigned int currentIndex,
					   mic_f *lazymem,
					   const SubdivMesh* const geom)
    {
      __aligned(64) float u_array[(patch.grid_size_simd_blocks+1)*16]; // for unaligned access
      __aligned(64) float v_array[(patch.grid_size_simd_blocks+1)*16];

      TIMER(double msec);
      TIMER(msec = getSeconds());    

      gridUVTessellator(patch.level,
			patch.grid_u_res,
			patch.grid_v_res,
			u_array,
			v_array);

      /* stich different tessellation levels in u/v grid */
      if (patch.needsStitching())
	stitchUVGrid(patch.level,patch.grid_u_res,patch.grid_v_res,u_array,v_array);

      TIMER(msec = getSeconds()-msec);    
      TIMER(DBG_PRINT("tess"));
      TIMER(DBG_PRINT(patch.grid_u_res));
      TIMER(DBG_PRINT(patch.grid_v_res));
      TIMER(DBG_PRINT(1000.0f * msec));
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
      TIMER(DBG_PRINT("bvh"));
      TIMER(DBG_PRINT(1000.0f * msec));
      TIMER(DBG_PRINT(patch.grid_subtree_size_64b_blocks * 64));

      return subtree_root;
    }

    BVH4i::NodeRef initLocalLazySubdivTreeCompact(const SubdivPatch1 &patch,
						  unsigned int currentIndex,
						  mic_f *lazymem,
						  const SubdivMesh* const geom)
    {
      __aligned(64) float grid_u[(patch.grid_size_simd_blocks+1)*16]; // for unaligned access
      __aligned(64) float grid_v[(patch.grid_size_simd_blocks+1)*16];

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
	  const mic_f u = load16f(&grid_u[i]);
	  const mic_f v = load16f(&grid_v[i]);
	  const mic_i u_i = mic_i(u * 65535.0f/2.0f);
	  const mic_i v_i = mic_i(v * 65535.0f/2.0f);
	  const mic_i uv_i = (v_i << 16) | u_i;
	  store16i(&grid_uv[i],uv_i);
	}

#if 0
      TIMER(msec = getSeconds()-msec);    
      TIMER(DBG_PRINT("tess"));
      TIMER(DBG_PRINT(patch.grid_u_res));
      TIMER(DBG_PRINT(patch.grid_v_res));
      TIMER(DBG_PRINT(1000.0f * msec));
      TIMER(msec = getSeconds());    
#endif

      BVH4i::NodeRef subtree_root = 0;
      const unsigned int oldIndex = currentIndex;

      BBox3fa bounds = createSubTreeCompact( subtree_root,
					     lazymem,
					     patch,
					     grid_x,
					     array_elements,
					     GridRange(0,patch.grid_u_res-1,0,patch.grid_v_res-1),
					     currentIndex);

      assert(currentIndex - oldIndex == patch.grid_bvh_size_64b_blocks);
      TIMER(msec = getSeconds()-msec);    
      TIMER(DBG_PRINT("tess+bvh"));
      TIMER(DBG_PRINT(patch.grid_u_res));
      TIMER(DBG_PRINT(patch.grid_v_res));
      TIMER(DBG_PRINT(patch.grid_subtree_size_64b_blocks*64));

      TIMER(DBG_PRINT(1000.0f * msec));
      TIMER(double throughput = 1.0 / (1000*msec));

      TIMER(msec = getSeconds());    
      TIMER(DBG_PRINT(throughput));

      return subtree_root;
    }


    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////



    __forceinline size_t lazyBuildPatch(const unsigned int patchIndex,
					const unsigned int commitCounter,
					SubdivPatch1* const patches,
					Scene *const scene,
					LocalTessellationCacheThreadInfo *threadInfo)
    {
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
	    const size_t subdiv_patch_root_ref    = subdiv_patch->root_ref;
	    
	    if (likely(subdiv_patch_root_ref)) 
	      {
		const size_t subdiv_patch_root = (subdiv_patch_root_ref & REF_TAG_MASK) + (size_t)SharedLazyTessellationCache::sharedLazyTessellationCache.getDataPtr();
		const size_t subdiv_patch_cache_index = subdiv_patch_root_ref >> 32;
		
		if (likely( subdiv_patch_cache_index == SharedLazyTessellationCache::sharedLazyTessellationCache.getCurrentIndex()))
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
	    const size_t subdiv_patch_root_ref    = subdiv_patch->root_ref;
	    const size_t subdiv_patch_cache_index = subdiv_patch_root_ref >> 32;

	    /* do we still need to create the subtree data? */
	    if (subdiv_patch_root_ref == 0 || subdiv_patch_cache_index != SharedLazyTessellationCache::sharedLazyTessellationCache.getCurrentIndex())
	      {	      
		const SubdivMesh* const geom = (SubdivMesh*)scene->get(subdiv_patch->geom); 
		size_t block_index = SharedLazyTessellationCache::sharedLazyTessellationCache.alloc(subdiv_patch->grid_subtree_size_64b_blocks);
		if (block_index == (size_t)-1)
		  {
		    /* cannot allocate => flush the cache */
		    subdiv_patch->write_unlock();
		    SharedLazyTessellationCache::sharedLazyTessellationCache.unlockThread(threadInfo->id);		  
		    SharedLazyTessellationCache::sharedLazyTessellationCache.resetCache();
		    //DBG_PRINT("RESET");
		    continue;
		  }
		//DBG_PRINT( SharedLazyTessellationCache::sharedLazyTessellationCache.getNumUsedBytes() );
		mic_f* local_mem   = (mic_f*)SharedLazyTessellationCache::sharedLazyTessellationCache.getBlockPtr(block_index);
		unsigned int currentIndex = 0;
#if COMPACT == 1
		BVH4i::NodeRef bvh4i_root = initLocalLazySubdivTreeCompact(*subdiv_patch,currentIndex,local_mem,geom);
#else
		BVH4i::NodeRef bvh4i_root = initLocalLazySubdivTree(*subdiv_patch,currentIndex,local_mem,geom);
#endif
		size_t new_root_ref = (size_t)bvh4i_root + (size_t)local_mem - (size_t)SharedLazyTessellationCache::sharedLazyTessellationCache.getDataPtr();
		assert( !(new_root_ref & REF_TAG) );
		new_root_ref |= REF_TAG;
		new_root_ref |= SharedLazyTessellationCache::sharedLazyTessellationCache.getCurrentIndex() << 32; 
		subdiv_patch->root_ref = new_root_ref;
	      }
	  }
	  subdiv_patch->write_unlock();
	  SharedLazyTessellationCache::sharedLazyTessellationCache.unlockThread(threadInfo->id);		  
	}	      
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

      LocalTessellationCacheThreadInfo *threadInfo = NULL;
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
	      SubdivPatch1 &patch = ((SubdivPatch1*)accel)[patchIndex];

	      size_t cached_64bit_root = lazyBuildPatch(patchIndex,commitCounter,(SubdivPatch1*)accel,scene,threadInfo);
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

		  
		  const mic_m m_active = 0x777;
#if COMPACT == 1
		  float *lazyCachePtr = (float*)SharedLazyTessellationCache::sharedLazyTessellationCache.getDataPtr();
		  Quad4x4 quad4x4;
		  quad4x4.init( curNode.offsetIndex(), patch, lazyCachePtr);
		  const mic_f uu = quad4x4.getU();
		  const mic_f vv = quad4x4.getV();
		  const mic3f &vtx = quad4x4.vtx;
#else
		  const unsigned int uvIndex = curNode.offsetIndex();
		  const Quad4x4 *__restrict__ const quad4x4 = (Quad4x4*)&lazymem[uvIndex];
		  quad4x4->prefetchData();
		  const mic_f uu = quad4x4->getU();
		  const mic_f vv = quad4x4->getV();
		  const mic3f &vtx = quad4x4->vtx;
#endif
		  intersect1_quad16(rayIndex, 
				    dir_xyz,
				    org_xyz,
				    ray16,
				    vtx,
				    uu,
				    vv,
				    4,
				    m_active,
				    patchIndex);
		}

	      SharedLazyTessellationCache::sharedLazyTessellationCache.unlockThread(threadInfo->id);

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

      LocalTessellationCacheThreadInfo *threadInfo = NULL;
      if (unlikely(!localThreadInfo))
	{
	  const unsigned int id = SharedLazyTessellationCache::sharedLazyTessellationCache.getNextRenderThreadID();
	  localThreadInfo = new LocalTessellationCacheThreadInfo( id );
	}
      threadInfo = localThreadInfo;

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
	      size_t cached_64bit_root = lazyBuildPatch(patchIndex,commitCounter,(SubdivPatch1*)accel,scene,threadInfo);
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

		SharedLazyTessellationCache::sharedLazyTessellationCache.unlockThread(threadInfo->id);
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

      LocalTessellationCacheThreadInfo *threadInfo = NULL;
      if (unlikely(!localThreadInfo))
	{
	  const unsigned int id = SharedLazyTessellationCache::sharedLazyTessellationCache.getNextRenderThreadID();
	  localThreadInfo = new LocalTessellationCacheThreadInfo( id );
	}
      threadInfo = localThreadInfo;

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
	  size_t cached_64bit_root = lazyBuildPatch(patchIndex,commitCounter,(SubdivPatch1*)accel,scene,threadInfo);
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

	  SharedLazyTessellationCache::sharedLazyTessellationCache.unlockThread(threadInfo->id);
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

      LocalTessellationCacheThreadInfo *threadInfo = NULL;
      if (unlikely(!localThreadInfo))
	{
	  const unsigned int id = SharedLazyTessellationCache::sharedLazyTessellationCache.getNextRenderThreadID();
	  localThreadInfo = new LocalTessellationCacheThreadInfo( id );
	}
      threadInfo = localThreadInfo;

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
	  size_t cached_64bit_root = lazyBuildPatch(patchIndex,commitCounter,(SubdivPatch1*)accel,scene,threadInfo);
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
