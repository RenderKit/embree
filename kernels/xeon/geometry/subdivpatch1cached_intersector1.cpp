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
    
    __thread LocalTessellationCacheThreadInfo* SubdivPatch1CachedIntersector1::localThreadInfo = nullptr;

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

    // void copyTessellationCacheTag(TessellationCacheTag *dest, TessellationCacheTag *source)
    // {
    //   assert( dest->getNumBlocks() >= source->getNumBlocks() );
    //   const size_t needed_blocks = source->getNumBlocks();
      
    //   memcpy(dest->getPtr(),source->getPtr(),64*needed_blocks);
    //   size_t source_root = source->getRootRef();
    //   updateBVH4Refs(source_root,(size_t)source->getPtr(),(size_t)dest->getPtr());
    //   dest->updateRootRef( ((size_t)source_root - (size_t)source->getPtr()) + (size_t)dest->getPtr() );

    //   const size_t l1_range0 = (size_t)dest->getPtr();
    //   const size_t l1_range1 = l1_range0 + 64*needed_blocks;
    //   size_t l1_blocks = countBlocks(BVH4::NodeRef(dest->getRootRef()),l1_range0,l1_range1);
              
    //   assert(l1_blocks >= needed_blocks);              
    // }

    //////////////////////////////////////////////////////////////////////////////////////////////////////

    /* build lazy subtree over patch */
    size_t SubdivPatch1CachedIntersector1::lazyBuildPatch(Precalculations &pre,
							  SubdivPatch1Cached* const subdiv_patch, 
							  const void* geom)
    {
      /* unlock previous patch */
      if (pre.current_patch)
        SharedLazyTessellationCache::sharedLazyTessellationCache.unlockThread(pre.threadID);

      while(1)
      {
        SharedLazyTessellationCache::sharedLazyTessellationCache.lockThreadLoop(pre.threadID);
        
        static const size_t REF_TAG      = 1;
        static const size_t REF_TAG_MASK = (~REF_TAG) & 0xffffffff;
        
        /* fast path for cache hit */
        {
          CACHE_STATS(SharedTessellationCacheStats::cache_accesses++);
          const int64_t subdiv_patch_root_ref    = subdiv_patch->root_ref; 
	  
          if (likely(subdiv_patch_root_ref)) 
          {
            const size_t subdiv_patch_root = (subdiv_patch_root_ref & REF_TAG_MASK) + (size_t)SharedLazyTessellationCache::sharedLazyTessellationCache.getDataPtr();
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
          const int64_t subdiv_patch_root_ref    = subdiv_patch->root_ref;
          const size_t subdiv_patch_cache_index = subdiv_patch_root_ref >> 32;
          
          /* do we still need to create the subtree data? */
          if (subdiv_patch_root_ref == 0 || !SharedLazyTessellationCache::sharedLazyTessellationCache.validCacheIndex(subdiv_patch_cache_index))
          {	      
            size_t block_index = SharedLazyTessellationCache::sharedLazyTessellationCache.alloc(subdiv_patch->grid_subtree_size_64b_blocks);
            if (block_index == (size_t)-1)
            {
              /* cannot allocate => flush the cache */
              subdiv_patch->write_unlock();
              SharedLazyTessellationCache::sharedLazyTessellationCache.unlockThread(pre.threadID);		  
              SharedLazyTessellationCache::sharedLazyTessellationCache.resetCache();
              continue;
            }
            BVH4::Node* node = (BVH4::Node*)SharedLazyTessellationCache::sharedLazyTessellationCache.getBlockPtr(block_index);
#if COMPACT == 1
            int64_t new_root_ref = (int64_t)buildSubdivPatchTreeCompact(*subdiv_patch,node,((Scene*)geom)->getSubdivMesh(subdiv_patch->geom));                                
            
#else                
            size_t new_root_ref = (size_t)buildSubdivPatchTree(*subdiv_patch,node,((Scene*)geom)->getSubdivMesh(subdiv_patch->geom));
#endif
            void *test = (void*)new_root_ref;
            
            new_root_ref -= (int64_t)SharedLazyTessellationCache::sharedLazyTessellationCache.getDataPtr();                                
            assert( new_root_ref <= 0xffffffff );
            assert( !(new_root_ref & REF_TAG) );
            new_root_ref |= REF_TAG;
            new_root_ref |= (int64_t)SharedLazyTessellationCache::sharedLazyTessellationCache.getCurrentIndex() << 32; 
            subdiv_patch->root_ref = new_root_ref;
            
#if _DEBUG
            const size_t patchIndex = subdiv_patch - pre.array;
            assert(patchIndex < pre.numPrimitives);
            CACHE_STATS(SharedTessellationCacheStats::incPatchBuild(patchIndex,pre.numPrimitives));
            //SharedTessellationCacheStats::newDeletePatchPtr(patchIndex,pre.numPrimitives,subdiv_patch->grid_subtree_size_64b_blocks*64);
#endif
          }
        }
        subdiv_patch->write_unlock();
        SharedLazyTessellationCache::sharedLazyTessellationCache.unlockThread(pre.threadID);		  
      }
      
    }
    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    
    
    BVH4::NodeRef SubdivPatch1CachedIntersector1::buildSubdivPatchTreeCompact(const SubdivPatch1Cached &patch,
									      void *const lazymem,
									      const SubdivMesh* const geom)
    {      
      TIMER(double msec = 0.0);
      TIMER(msec = getSeconds());
      TIMER(uint64_t cycles = rdtsc());
      assert( patch.grid_size_simd_blocks >= 1 );

      const size_t array_elements = patch.grid_size_simd_blocks * 8;

#if 0
      PRINT( patch.grid_u_res );
      PRINT( patch.grid_v_res );
      PRINT( array_elements );
      PRINT( patch.grid_size_simd_blocks );
      PRINT( patch.grid_subtree_size_64b_blocks );
#endif
 
#if !defined(_MSC_VER) || defined(__INTEL_COMPILER)
      __aligned(64) float grid_u[array_elements+16]; 
      __aligned(64) float grid_v[array_elements+16];
     
#else
#define MAX_GRID_SIZE 64*64
      __aligned(64) float local_grid_u[MAX_GRID_SIZE];
      __aligned(64) float local_grid_v[MAX_GRID_SIZE];
	  float *const grid_u = (patch.grid_size_simd_blocks * 8 < MAX_GRID_SIZE) ? local_grid_u : (float*)_mm_malloc((array_elements + 16)*sizeof(float),64);
	  float *const grid_v = (patch.grid_size_simd_blocks * 8 < MAX_GRID_SIZE) ? local_grid_v : (float*)_mm_malloc((array_elements + 16)*sizeof(float),64);
#endif   
      const size_t grid_offset = patch.grid_bvh_size_64b_blocks * 16;

      float *const grid_x  = (float*)lazymem + grid_offset + 0 * array_elements;
      float *const grid_y  = (float*)lazymem + grid_offset + 1 * array_elements;
      float *const grid_z  = (float*)lazymem + grid_offset + 2 * array_elements;
      int   *const grid_uv = (int*)  lazymem + grid_offset + 3 * array_elements;

      assert( patch.grid_subtree_size_64b_blocks * 16 >= grid_offset + 4 * array_elements);

      evalGrid(patch,grid_x,grid_y,grid_z,grid_u,grid_v,geom);

      for (size_t i=0;i<array_elements;i++)
        grid_uv[i] = (((int)(grid_v[i] * 65535.0f/2.0f)) << 16) | ((int)(grid_u[i] * 65535.0f/2.0f)); 
      
      BVH4::NodeRef subtree_root = 0;
      unsigned int currentIndex = 0;
      BBox3fa bounds = createSubTreeCompact( subtree_root,
					     (float*)lazymem,
					     patch,
					     grid_x,
					     array_elements,
					     GridRange(0,patch.grid_u_res-1,0,patch.grid_v_res-1),
					     currentIndex);

      assert( std::isfinite(bounds.lower.x) );
      assert( std::isfinite(bounds.lower.y) );
      assert( std::isfinite(bounds.lower.z) );

      assert( std::isfinite(bounds.upper.x) );
      assert( std::isfinite(bounds.upper.y) );
      assert( std::isfinite(bounds.upper.z) );
      
      assert(currentIndex == patch.grid_bvh_size_64b_blocks);

      TIMER(cycles = rdtsc() - cycles);
      TIMER(msec = getSeconds()-msec);            
      TIMER(double throughput = 1.0 / (1000*msec));
      TIMER(double throughput2 = (2300*1E3) / (double)cycles);

      TIMER(PRINT(throughput));
      TIMER(PRINT(throughput2));

      TIMER(PRINT(1000*msec));
      TIMER(PRINT(patch.grid_u_res));
      TIMER(PRINT(patch.grid_v_res));
      TIMER(PRINT(patch.grid_subtree_size_64b_blocks*64));

#if defined(_MSC_VER) && !defined(__INTEL_COMPILER)
	
	  if (patch.grid_size_simd_blocks * 8 >= MAX_GRID_SIZE)
	  {
	    _mm_free(grid_u);
		_mm_free(grid_v);
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

    BVH4::NodeRef SubdivPatch1CachedIntersector1::buildSubdivPatchTree(const SubdivPatch1Cached &patch,
                                                                       void *const lazymem,
                                                                       const SubdivMesh* const geom)
    {      
      TIMER(double msec = 0.0);
      TIMER(msec = getSeconds());
        
      assert( patch.grid_size_simd_blocks >= 1 );
#if !defined(_MSC_VER) || defined(__INTEL_COMPILER)
      __aligned(64) float grid_x[(patch.grid_size_simd_blocks+1)*8]; 
      __aligned(64) float grid_y[(patch.grid_size_simd_blocks+1)*8];
      __aligned(64) float grid_z[(patch.grid_size_simd_blocks+1)*8]; 
        
      __aligned(64) float grid_u[(patch.grid_size_simd_blocks+1)*8]; 
      __aligned(64) float grid_v[(patch.grid_size_simd_blocks+1)*8];
     
#else
      const size_t array_elements = (patch.grid_size_simd_blocks + 1) * 8;
      float *const ptr = (float*)_malloca(5 * array_elements * sizeof(float) + 64);
      float *const grid_arrays = (float*)ALIGN_PTR(ptr,64);

      float *grid_x = &grid_arrays[array_elements * 0];
      float *grid_y = &grid_arrays[array_elements * 1];
      float *grid_z = &grid_arrays[array_elements * 2];
      float *grid_u = &grid_arrays[array_elements * 3];
      float *grid_v = &grid_arrays[array_elements * 4];

        
#endif   
      evalGrid(patch,grid_x,grid_y,grid_z,grid_u,grid_v,geom);
        
      BVH4::NodeRef subtree_root = BVH4::encodeNode( (BVH4::Node*)lazymem);
      unsigned int currentIndex = 0;
      BBox3fa bounds = createSubTree( subtree_root,
				      (float*)lazymem,
				      patch,
				      grid_x,
				      grid_y,
				      grid_z,
				      grid_u,
				      grid_v,
				      GridRange(0,patch.grid_u_res-1,0,patch.grid_v_res-1),
				      currentIndex,
				      geom);
        
      assert(currentIndex == patch.grid_subtree_size_64b_blocks);

      TIMER(msec = getSeconds()-msec);    
	{
         TIMER(PRINT(1000*msec));
         TIMER(PRINT(patch.grid_u_res));
         TIMER(PRINT(patch.grid_v_res));
         TIMER(PRINT(patch.grid_subtree_size_64b_blocks*64));
        }

#if defined(_MSC_VER) && !defined(__INTEL_COMPILER)
      _freea(ptr);
#endif
      return subtree_root;
    }

    
    
    
    BBox3fa SubdivPatch1CachedIntersector1::createSubTree(BVH4::NodeRef &curNode,
                                                          float *const lazymem,
                                                          const SubdivPatch1Cached &patch,
                                                          const float *const grid_x_array,
                                                          const float *const grid_y_array,
                                                          const float *const grid_z_array,
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
        
	  assert(u_size >= 1);
	  assert(v_size >= 1);
        
	  assert(u_size*v_size <= 9);
        
	  const unsigned int currentIndex = localCounter;
	  localCounter +=  (sizeof(Quad2x2)+63) / 64; 
        
	  Quad2x2 *qquad = (Quad2x2*)&lazymem[currentIndex*16];
                
	  float4 leaf_x_array[3];
	  float4 leaf_y_array[3];
	  float4 leaf_z_array[3];
	  float4 leaf_u_array[3];
	  float4 leaf_v_array[3];
        
	  for (unsigned int v=v_start;v<=v_end;v++)
	    {
	      const size_t offset = v * patch.grid_u_res + u_start;
	      const unsigned int local_v = v - v_start;
	      leaf_x_array[local_v] = loadu4f(&grid_x_array[ offset ]);
	      leaf_y_array[local_v] = loadu4f(&grid_y_array[ offset ]);
	      leaf_z_array[local_v] = loadu4f(&grid_z_array[ offset ]);
	      leaf_u_array[local_v] = loadu4f(&grid_u_array[ offset ]);
	      leaf_v_array[local_v] = loadu4f(&grid_v_array[ offset ]);            
	    }
        
	  /* set invalid grid u,v value to border elements */
	  for (unsigned int x=u_size-1;x<3;x++)
	    for (unsigned int y=0;y<3;y++)
	      {
		leaf_x_array[y][x] = leaf_x_array[y][u_size-1];
		leaf_y_array[y][x] = leaf_y_array[y][u_size-1];
		leaf_z_array[y][x] = leaf_z_array[y][u_size-1];
		leaf_u_array[y][x] = leaf_u_array[y][u_size-1];
		leaf_v_array[y][x] = leaf_v_array[y][u_size-1];
	      }
        
	  for (unsigned int y=v_size-1;y<3;y++)
	    for (unsigned int x=0;x<3;x++)
	      {
		leaf_x_array[y][x] = leaf_x_array[v_size-1][x];
		leaf_y_array[y][x] = leaf_y_array[v_size-1][x];
		leaf_z_array[y][x] = leaf_z_array[v_size-1][x];
		leaf_u_array[y][x] = leaf_u_array[v_size-1][x];
		leaf_v_array[y][x] = leaf_v_array[v_size-1][x];
	      }
        
                
	  qquad->init( leaf_x_array, 
		       leaf_y_array, 
		       leaf_z_array, 
		       leaf_u_array, 
		       leaf_v_array);
        
#if 0
	  PRINT("LEAF");
	  PRINT(u_start);
	  PRINT(v_start);
	  PRINT(u_end);
	  PRINT(v_end);
        
	  for (unsigned int y=0;y<3;y++)
	    for (unsigned int x=0;x<3;x++)
	      std::cout << y << " " << x 
			<< " ->  x = " << leaf_x_array[y][x] << " y = " << leaf_v_array[y][x] << " z = " << leaf_z_array[y][x]
			<< "   u = " << leaf_u_array[y][x] << " v = " << leaf_v_array[y][x] << std::endl;
        
	  PRINT( *qquad );
        
#endif          
        
	  BBox3fa bounds = qquad->bounds();
	  curNode = BVH4::encodeTypedLeaf(qquad,2);
        
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
	  BBox3fa bounds_subtree = createSubTree( node->child(i), 
						  lazymem, 
						  patch, 
						  grid_x_array,
						  grid_y_array,
						  grid_z_array,
						  grid_u_array,
						  grid_v_array,
						  r[i],						  
						  localCounter,
						  geom);
	  node->set(i, bounds_subtree);
	  bounds.extend( bounds_subtree );
	}
      
      return bounds;
    }



    void SubdivPatch1CachedIntersector1::createLocalThreadInfo()
    {
      localThreadInfo = new LocalTessellationCacheThreadInfo( SharedLazyTessellationCache::sharedLazyTessellationCache.getNextRenderThreadID() );	      
    }

  };
}
