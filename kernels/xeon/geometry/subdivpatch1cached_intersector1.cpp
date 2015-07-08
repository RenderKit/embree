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
 
#include "subdivpatch1cached_intersector1.h"
#include "../bvh4/bvh4.h"
#include "../bvh4/bvh4_intersector1.h"

#define TIMER(x) 
#define DBG(x) 

namespace embree
{
  namespace isa
  {  
    //////////////////////////////////////////////////////////////////////////////////////////////////////

    void verifySubTreeBVH(const BVH4::NodeRef ref)
    {
      assert(ref != BVH4::invalidNode );

      /* this is a leaf node */
      if (unlikely(ref.isLeaf()))
        return;
      
      const BVH4::Node* node = ref.node();
      
      for (size_t i=0;i<4;i++)
	{
	  assert(node->child(i) != BVH4::emptyNode);
	  
	  BBox3fa bounds = node->bounds(i);

	  assert( std::isfinite(bounds.lower.x) );
	  assert( std::isfinite(bounds.lower.y) );
	  assert( std::isfinite(bounds.lower.z) );

	  assert( std::isfinite(bounds.upper.x) );
	  assert( std::isfinite(bounds.upper.y) );
	  assert( std::isfinite(bounds.upper.z) );

	  verifySubTreeBVH(node->child(i));
	}
    }

    size_t countBlocks(const BVH4::NodeRef ref, const size_t range0, const size_t range1)
    {
      
      size_t t = (size_t)ref;

      assert(range0 <= t);
      assert(t <= range1);

      /* this is a leaf node */
      if (unlikely(ref.isLeaf()))
        return 3;
      
      const BVH4::Node* node = ref.node();
      
      size_t size = 0;
      for (size_t i=0;i<4;i++)
        if (node->child(i) != BVH4::emptyNode)
          size += countBlocks(node->child(i),range0,range1);

      return 2 + size;
    }

    void updateBVH4Refs(const BVH4::NodeRef &ref, const size_t old_ptr, const size_t new_ptr)
    {
      if (unlikely(ref == BVH4::emptyNode))
        return;

      assert(ref != BVH4::invalidNode);

      /* this is a leaf node */
      if (unlikely(ref.isLeaf()))
        return;

      const BVH4::Node* node = ref.node();
      
      for (size_t i=0;i<4;i++)
        {
          const BVH4::NodeRef &child = node->child(i);
          if (node->child(i) != BVH4::emptyNode)
            {
              if (child.isNode())
                updateBVH4Refs(child,old_ptr,new_ptr);

              const size_t dest_offset = (size_t)&child - old_ptr;              
              const size_t new_ref     = (size_t)child - old_ptr + new_ptr;
              size_t *ptr = (size_t*)((char*)new_ptr + dest_offset);
              *ptr = new_ref;    
            }
        }
    }

    /* build lazy subtree over patch */
    size_t SubdivPatch1CachedIntersector1::lazyBuildPatch(Precalculations &pre,
							  SubdivPatch1Cached* const subdiv_patch, 
							  const Scene* scene)
    {
      /* unlock previous patch */
      if (pre.current_patch)
      {
        assert(SharedLazyTessellationCache::sharedLazyTessellationCache.isLocked(pre.t_state));
        SharedLazyTessellationCache::sharedLazyTessellationCache.unlockThread(pre.t_state);
      }

      while(1)
      {

        SharedLazyTessellationCache::sharedLazyTessellationCache.lockThreadLoop(pre.t_state);
       
        const size_t globalTime = scene->commitCounter;
        if (void* ptr = SharedLazyTessellationCache::lookup(&subdiv_patch->root_ref,globalTime))
            return (size_t) ptr;
        
        SharedLazyTessellationCache::sharedLazyTessellationCache.unlockThread(pre.t_state);		  

        if (subdiv_patch->try_write_lock())
        {
          if (!SharedLazyTessellationCache::validTag(subdiv_patch->root_ref,globalTime)) 
          {
            /* generate vertex grid, lock and allocate memory in the cache */
            size_t new_root_ref = (size_t)buildSubdivPatchTreeCompact(*subdiv_patch,pre.t_state,scene->getSubdivMesh(subdiv_patch->geom));                                
            
            /* get current commit index */
            //const size_t commitIndex = SharedLazyTessellationCache::sharedLazyTessellationCache.getCurrentIndex();
            const size_t combinedTime = SharedLazyTessellationCache::sharedLazyTessellationCache.getTime(globalTime);

            __memory_barrier();
            /* write new root ref */
            subdiv_patch->root_ref = SharedLazyTessellationCache::Tag((void*)new_root_ref,combinedTime);
            
#if _DEBUG
            const size_t patchIndex = subdiv_patch - pre.array;
            assert(patchIndex < pre.numPrimitives);
            CACHE_STATS(SharedTessellationCacheStats::incPatchBuild(patchIndex,pre.numPrimitives));
#endif
            assert(SharedLazyTessellationCache::sharedLazyTessellationCache.isLocked(pre.t_state));

            /* unlock current patch */
            subdiv_patch->write_unlock();

            /* memory region still locked, forward progress guaranteed */
            return new_root_ref;
          }
          /* unlock current patch */
          subdiv_patch->write_unlock();
        }
      }
    }
    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    
    
    BVH4::NodeRef SubdivPatch1CachedIntersector1::buildSubdivPatchTreeCompact(const SubdivPatch1Cached &patch,
                                                                              ThreadWorkState *t_state,
									      const SubdivMesh* const geom, BBox3fa* bounds_o)
    {      
      assert( patch.grid_size_simd_blocks >= 1 );

      const size_t array_elements = patch.grid_size_simd_blocks * 8;
 
#if !defined(_MSC_VER) || defined(__INTEL_COMPILER)
      __aligned(64) float local_grid_u[array_elements+16]; 
      __aligned(64) float local_grid_v[array_elements+16];
      __aligned(64) float local_grid_x[array_elements+16];
      __aligned(64) float local_grid_y[array_elements+16];
      __aligned(64) float local_grid_z[array_elements+16];
#else
#define MAX_GRID_SIZE 64*64
      __aligned(64) float tmp_grid_u[MAX_GRID_SIZE];
      __aligned(64) float tmp_grid_v[MAX_GRID_SIZE];
      __aligned(64) float tmp_grid_x[MAX_GRID_SIZE];
      __aligned(64) float tmp_grid_y[MAX_GRID_SIZE];
      __aligned(64) float tmp_grid_z[MAX_GRID_SIZE];

      float *local_grid_u = tmp_grid_u;
      float *local_grid_v = tmp_grid_v;
      float *local_grid_x = tmp_grid_x;
      float *local_grid_y = tmp_grid_y;
      float *local_grid_z = tmp_grid_z;
      if (unlikely(array_elements >= MAX_GRID_SIZE))
      { 
        local_grid_u = (float*)_mm_malloc((array_elements + 16)*sizeof(float),64);
        local_grid_v = (float*)_mm_malloc((array_elements + 16)*sizeof(float),64);
        local_grid_x = (float*)_mm_malloc((array_elements + 16)*sizeof(float),64);
        local_grid_y = (float*)_mm_malloc((array_elements + 16)*sizeof(float),64);
        local_grid_z = (float*)_mm_malloc((array_elements + 16)*sizeof(float),64);
      }
#endif   

      /* compute vertex grid (+displacement) */
      evalGrid(patch,0,0,patch.grid_u_res,patch.grid_v_res,local_grid_x,local_grid_y,local_grid_z,local_grid_u,local_grid_v,geom);

      /* lock the cache */
      SharedLazyTessellationCache::sharedLazyTessellationCache.lockThreadLoop(t_state);

      /* allocate memory in cache and get current commit index */
      void *const lazymem = SharedLazyTessellationCache::sharedLazyTessellationCache.allocLoop(t_state,64*patch.grid_subtree_size_64b_blocks);

      /* copy temporary data to tessellation cache */
      const size_t grid_offset = patch.grid_bvh_size_64b_blocks * 16;

      float *const grid_x  = (float*)lazymem + grid_offset + 0 * array_elements;
      float *const grid_y  = (float*)lazymem + grid_offset + 1 * array_elements;
      float *const grid_z  = (float*)lazymem + grid_offset + 2 * array_elements;
      int   *const grid_uv = (int*)  lazymem + grid_offset + 3 * array_elements;
      assert( patch.grid_subtree_size_64b_blocks * 16 >= grid_offset + 4 * array_elements);

      memcpy(grid_x ,local_grid_x ,array_elements*sizeof(float));
      memcpy(grid_y ,local_grid_y ,array_elements*sizeof(float));
      memcpy(grid_z ,local_grid_z ,array_elements*sizeof(float));

      for (size_t i=0;i<array_elements;i++)
        grid_uv[i] = (((int)(local_grid_v[i] * 65535.0f/2.0f)) << 16) | ((int)(local_grid_u[i] * 65535.0f/2.0f)); 
      
      /* build bvh tree */
      BVH4::NodeRef subtree_root = 0;
      unsigned int currentIndex = 0;
      BBox3fa bounds = createSubTreeCompact( subtree_root,
					     (float*)lazymem,
					     patch,
					     grid_x,
					     array_elements,
					     GridRange(0,patch.grid_u_res-1,0,patch.grid_v_res-1),
					     currentIndex);
      if (bounds_o) *bounds_o = bounds;

      assert( std::isfinite(bounds.lower.x) );
      assert( std::isfinite(bounds.lower.y) );
      assert( std::isfinite(bounds.lower.z) );

      assert( std::isfinite(bounds.upper.x) );
      assert( std::isfinite(bounds.upper.y) );
      assert( std::isfinite(bounds.upper.z) );
      
      assert(currentIndex == patch.grid_bvh_size_64b_blocks);

#if defined(_MSC_VER) && !defined(__INTEL_COMPILER)
	
      if (array_elements >= MAX_GRID_SIZE)
      {
        _mm_free(local_grid_u);
        _mm_free(local_grid_v);
        _mm_free(local_grid_x);
        _mm_free(local_grid_y);
        _mm_free(local_grid_z);       
      }
#endif
      return subtree_root;
    }

    BBox3fa SubdivPatch1CachedIntersector1::createSubTreeCompact(BVH4::NodeRef &curNode,
								 float *const lazymem,
								 const SubdivPatch1Cached &patch,
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
	  BBox3fa bounds( empty );
	  for (size_t v = range.v_start; v<=range.v_end; v++)
	    for (size_t u = range.u_start; u<=range.u_end; u++)
	      {
		const float x = grid_x_array[ v * patch.grid_u_res + u];
		const float y = grid_y_array[ v * patch.grid_u_res + u];
		const float z = grid_z_array[ v * patch.grid_u_res + u];
		bounds.extend( Vec3fa(x,y,z) );
	      }
	  unsigned int u_start = range.u_start;
	  unsigned int v_start = range.v_start;

	  const unsigned int u_end   = range.u_end;
	  const unsigned int v_end   = range.v_end;

          if (unlikely(u_end-u_start+1 < 3)) 
	    { 
	      const unsigned int delta_u = 3 - (u_end-u_start+1);
	      if (u_start >= delta_u) 
		u_start -= delta_u; 
	      else
		u_start = 0;
	    }
          if (unlikely(v_end-v_start+1 < 3)) 
	    { 
	      const unsigned int delta_v = 3 - (v_end-v_start+1);
	      if (v_start >= delta_v) 
		v_start -= delta_v; 
	      else
		v_start = 0;
	    }

	  const unsigned int u_size = u_end-u_start+1;
	  const unsigned int v_size = v_end-v_start+1;
        
	  assert(u_size >= 1);
	  assert(v_size >= 1);
        
	  const size_t grid_offset3x3    = v_start * patch.grid_u_res + u_start;


	  size_t offset_bytes = (size_t)&grid_x_array[ grid_offset3x3 ] - (size_t)SharedLazyTessellationCache::sharedLazyTessellationCache.getDataPtr();
	  assert(offset_bytes < 0xffffffff);
	  assert((offset_bytes & 3) == 0);
          size_t value = (offset_bytes << 2) + (size_t)SharedLazyTessellationCache::sharedLazyTessellationCache.getDataPtr();
          assert( (value & 2) == 0 );
	  curNode = BVH4::encodeTypedLeaf((void*)value,2);

	  assert( std::isfinite(bounds.lower.x) );
	  assert( std::isfinite(bounds.lower.y) );
	  assert( std::isfinite(bounds.lower.z) );
	  
	  assert( std::isfinite(bounds.upper.x) );
	  assert( std::isfinite(bounds.upper.y) );
	  assert( std::isfinite(bounds.upper.z) );

	  return bounds;
	}
      
      
      /* allocate new bvh4 node */
      const size_t currentIndex = localCounter;
      
      /* 128 bytes == 2 x 64 bytes cachelines */
      localCounter += 2; 
      
      BVH4::Node *node = (BVH4::Node *)&lazymem[currentIndex*16];
      
      curNode = BVH4::encodeNode( node );
      
      node->clear();
      
      GridRange r[4];
      
      const unsigned int children = range.splitIntoSubRanges(r);
      
      /* create four subtrees */
      BBox3fa bounds( empty );
      
      for (unsigned int i=0;i<children;i++)
	{
	  BBox3fa bounds_subtree = createSubTreeCompact( node->child(i), 
							 lazymem, 
							 patch, 
							 grid_array,
							 grid_array_elements,
							 r[i],						  
							 localCounter);
	  node->set(i, bounds_subtree);
	  bounds.extend( bounds_subtree );
	}
      
      return bounds;
    }
  }
}
