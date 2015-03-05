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
#include "xeon/bvh4/bvh4.h"
#include "xeon/bvh4/bvh4_intersector1.h"

#define TIMER(x)
#define DBG(x) 


#define ENABLE_TESSELLATION_CACHE_HIERARCHY 0


#define NUM_SCRATCH_MEM_BLOCKS 1024

//#define SHARED_TESSELLATION_CACHE_ENTRIES 1024*32
#define SHARED_TESSELLATION_CACHE_ENTRIES 16
//#define SHARED_TESSELLATION_CACHE_ENTRIES 1024*128

namespace embree
{
  static SharedTessellationCache<SHARED_TESSELLATION_CACHE_ENTRIES> sharedTessellationCache;



  namespace isa
  {  
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

    void copyTessellationCacheTag(TessellationCacheTag *dest, TessellationCacheTag *source)
    {
      assert( dest->getNumBlocks() >= source->getNumBlocks() );
      const size_t needed_blocks = source->getNumBlocks();
      
      memcpy(dest->getPtr(),source->getPtr(),64*needed_blocks);
      size_t source_root = source->getRootRef();
      updateBVH4Refs(source_root,(size_t)source->getPtr(),(size_t)dest->getPtr());
      dest->updateRootRef( ((size_t)source_root - (size_t)source->getPtr()) + (size_t)dest->getPtr() );

      const size_t l1_range0 = (size_t)dest->getPtr();
      const size_t l1_range1 = l1_range0 + 64*needed_blocks;
      size_t l1_blocks = countBlocks(BVH4::NodeRef(dest->getRootRef()),l1_range0,l1_range1);
              
      assert(l1_blocks >= needed_blocks);              
    }

    //////////////////////////////////////////////////////////////////////////////////////////////////////


    __thread TessellationRefCache *SubdivPatch1CachedIntersector1::thread_cache = NULL;

    //////////////////////////////////////////////////////////////////////////////////////////////////////

    /* build lazy subtree over patch */
#if 1
    size_t SubdivPatch1CachedIntersector1::lazyBuildPatch(Precalculations &pre,
							  SubdivPatch1Cached* const subdiv_patch, 
							  const void* geom,
							  TessellationRefCache *ref_cache)
    {
      if (pre.local_tag)
	{
	  pre.local_tag->read_unlock();
	  pre.local_tag = NULL;
	}
      
      InputTagType tag = (InputTagType)subdiv_patch;        
      const unsigned int commitCounter = ((Scene*)geom)->commitCounter;
      TessellationCacheTag *s = sharedTessellationCache.getTag(tag);

      s->read_lock();
      CACHE_STATS(SharedTessellationCacheStats::cache_accesses++);
      if (likely(s->match(tag,commitCounter)))
	{
	  CACHE_STATS(SharedTessellationCacheStats::cache_hits++);
	  size_t patch_root = s->getRootRef();
	  pre.local_tag = s;
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
	      pre.local_tag = s;
	      s->upgrade_write_to_read_lock();
	      return patch_root;
	    }
	  else
	    {
	      BVH4::Node* node = (BVH4::Node*)s->getPtr();
	      const unsigned int needed_blocks = subdiv_patch->grid_subtree_size_64b_blocks;          
	      if (s->getNumBlocks() < needed_blocks)
		{
		  if (node != NULL)
		    free_tessellation_cache_mem(node);
		  node = (BVH4::Node*)alloc_tessellation_cache_mem(needed_blocks);              	      
		}
	      
	      size_t new_root = (size_t)buildSubdivPatchTree(*subdiv_patch,node,((Scene*)geom)->getSubdivMesh(subdiv_patch->geom));

	      s->set(tag,commitCounter,new_root,needed_blocks);
	      /* insert new patch data into local cache */
	      pre.local_tag = s;
	      s->upgrade_write_to_read_lock();
	      return new_root;
	    }
	}

      DBG(DBG_PRINT("FALLBACK"));
      CACHE_STATS(SharedTessellationCacheStats::cache_fallbacks++);

      const unsigned int needed_blocks = subdiv_patch->grid_subtree_size_64b_blocks;          
      ref_cache->reallocScratchMem(needed_blocks);
      BVH4::Node* node = (BVH4::Node*)ref_cache->getScratchMemPtr();
      size_t new_root = (size_t)buildSubdivPatchTree(*subdiv_patch,node,((Scene*)geom)->getSubdivMesh(subdiv_patch->geom));

      return new_root;                 
    }

#else
    size_t SubdivPatch1CachedIntersector1::lazyBuildPatch(Precalculations &pre,
							  SubdivPatch1Cached* const subdiv_patch, 
							  const void* geom,
							  TessellationRefCache *ref_cache)
    {
      /* lookup in per thread reference cache */
      InputTagType tag = (InputTagType)subdiv_patch;        
      const unsigned int commitCounter = ((Scene*)geom)->commitCounter;
      
      if (unlikely(ref_cache->needCommitCounterUpdate(commitCounter)))
	{
	  flushLocalTessellationCache();
	}

      CACHE_STATS(DistributedTessellationCacheStats::cache_accesses++);
      TessellationRefCacheTag *t = ref_cache->lookUpTag(tag,commitCounter);
      DBG(DBG_PRINT("LOCKUP TAG"));
      DBG(DBG_PRINT(t->getPrimTag()));

      if (likely(t->match(tag,commitCounter)))
	{
	  assert( !t->empty() );
	  CACHE_STATS(DistributedTessellationCacheStats::cache_hits++);
	  return t->getRootRef();
	}
      CACHE_STATS(DistributedTessellationCacheStats::cache_misses++);

      /* miss handler */

      if (!t->empty()) {
	TessellationCacheTag *old = sharedTessellationCache.getTagBy32BitID(t->getPrimTag());
	old->read_unlock();
	t->reset();
      };      

      TessellationCacheTag *s = sharedTessellationCache.getTag(tag);

      CACHE_STATS(SharedTessellationCacheStats::cache_accesses++);
      //if (s->match(tag,commitCounter))
	{
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
	}
	//CACHE_STATS(else SharedTessellationCacheStats::cache_misses++);

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
	      BVH4::Node* node = (BVH4::Node*)s->getPtr();
	      const unsigned int needed_blocks = subdiv_patch->grid_subtree_size_64b_blocks;          
	      if (s->getNumBlocks() < needed_blocks)
		{
		  //std::cout << needed_blocks << " -> " << s->getNumBlocks() << std::endl;
		  if (node != NULL)
		    free_tessellation_cache_mem(node);
		  node = (BVH4::Node*)alloc_tessellation_cache_mem(needed_blocks);              	      
		}
	      
	      size_t new_root = (size_t)buildSubdivPatchTree(*subdiv_patch,node,((Scene*)geom)->getSubdivMesh(subdiv_patch->geom));

	      s->set(tag,commitCounter,new_root,needed_blocks);
	      /* insert new patch data into local cache */
	      t->set(tag,commitCounter,new_root);

	      s->upgrade_write_to_read_lock();
	      return new_root;
	    }
	}

      CACHE_STATS(SharedTessellationCacheStats::cache_fallbacks++);

      const unsigned int needed_blocks = subdiv_patch->grid_subtree_size_64b_blocks;          
      ref_cache->reallocScratchMem(needed_blocks);
      BVH4::Node* node = (BVH4::Node*)ref_cache->getScratchMemPtr();
      size_t new_root = (size_t)buildSubdivPatchTree(*subdiv_patch,node,((Scene*)geom)->getSubdivMesh(subdiv_patch->geom));

      // if (s->try_write_lock())
      // 	{
      // 	  PING;
      // 	  CACHE_STATS(SharedTessellationCacheStats::cache_updates_successful++);
      // 	  BVH4::Node* dest = (BVH4::Node*)s->getPtr();
      // 	  const unsigned int needed_blocks = subdiv_patch->grid_subtree_size_64b_blocks;          
      // 	  if (s->getNumBlocks() < needed_blocks)
      // 	    {
      // 	      if (node != NULL)
      // 		free_tessellation_cache_mem(node);
      // 	      dest = (BVH4::Node*)alloc_tessellation_cache_mem(needed_blocks);              	      
      // 	    }
      // 	  memcpy(dest,node,64*needed_blocks);
      // 	  size_t source_root = new_root;
      // 	  updateBVH4Refs(source_root,(size_t)node,(size_t)dest);
      // 	  s->set(tag,commitCounter,(size_t)dest,needed_blocks);
      // 	  s->updateRootRef( ((size_t)source_root - (size_t)node) + (size_t)dest );
	  	  
      // 	  s->upgrade_write_to_read_lock();
      // 	}
      
      return new_root;      
    }
#endif
    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    
    
    
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
                
	  ssef leaf_x_array[3];
	  ssef leaf_y_array[3];
	  ssef leaf_z_array[3];
	  ssef leaf_u_array[3];
	  ssef leaf_v_array[3];
        
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
	  DBG_PRINT("LEAF");
	  DBG_PRINT(u_start);
	  DBG_PRINT(v_start);
	  DBG_PRINT(u_end);
	  DBG_PRINT(v_end);
        
	  for (unsigned int y=0;y<3;y++)
	    for (unsigned int x=0;x<3;x++)
	      std::cout << y << " " << x 
			<< " ->  x = " << leaf_x_array[y][x] << " y = " << leaf_v_array[y][x] << " z = " << leaf_z_array[y][x]
			<< "   u = " << leaf_u_array[y][x] << " v = " << leaf_v_array[y][x] << std::endl;
        
	  DBG_PRINT( *qquad );
        
#endif          
        
	  BBox3fa bounds = qquad->bounds();
	  curNode = BVH4::encodeLeaf(qquad,2);
        
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

    std::size_t getThreadID()
    {
      const std::thread::id id = std::this_thread::get_id();
      static std::size_t index = 0;
      static std::mutex mutex;
      static std::map<std::thread::id, std::size_t> ids;
      std::lock_guard<std::mutex> lock(mutex);
      if(ids.find(id) == ids.end())
	ids[id] = index++;
      return ids[id];
    }

    void SubdivPatch1CachedIntersector1::createTessellationCache()
    {
      TessellationRefCache *cache = new TessellationRefCache(  NUM_SCRATCH_MEM_BLOCKS  );
      
      DBG_PRINT((size_t)getThreadID());
      thread_cache = cache;

    }
    
  };
}
