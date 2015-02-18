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


#define ENABLE_TESSELLATION_CACHE_HIERARCHY 1

#define SHARED_TESSELLATION_CACHE_ENTRIES 256
#define PRE_ALLOC_BLOCKS 32

namespace embree
{
  namespace isa
  {  
    
    __thread PerThreadTessellationCache *SubdivPatch1CachedIntersector1::thread_cache = NULL;
    SharedTessellationCache<SHARED_TESSELLATION_CACHE_ENTRIES,PRE_ALLOC_BLOCKS> sharedTessellationCache;

    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

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
    
    void SubdivPatch1CachedIntersector1::secondLevelTessellationCacheMissHandler(TessellationCacheTag *firstLevelTag, const SubdivPatch1Cached* const subdiv_patch, const void* geom)
    {
      const unsigned int commitCounter = ((Scene*)geom)->commitCounter;
      InputTagType tag = (InputTagType)subdiv_patch;        
      const unsigned int blocks = subdiv_patch->grid_subtree_size_64b_blocks;
      
      TessellationCacheTag *t = sharedTessellationCache.getTag(tag);

      /* read lock */
      t->read_lock();

      CACHE_STATS(SharedTessellationCacheStats::cache_accesses++);
      
      if (unlikely(!t->match(tag,commitCounter)))
        {
          /* data not in second level cache */
          CACHE_STATS(SharedTessellationCacheStats::cache_misses++);

          subdiv_patch->prefetchData();
          
          /* upgrade read to write lock */
          t->upgrade_read_to_write_lock();
 
          const unsigned int needed_blocks = subdiv_patch->grid_subtree_size_64b_blocks;          
          BVH4::Node* node = (BVH4::Node*)t->getPtr();
          
          if (t->getNumBlocks() < needed_blocks)
            {
              CACHE_STATS(SharedTessellationCacheStats::cache_evictions++);                            
              if (node != NULL)
                  free_tessellation_cache_mem(node,t->getNumBlocks());                 
              node = (BVH4::Node*)alloc_tessellation_cache_mem(needed_blocks);              
              t->set(tag,commitCounter,(size_t)node,needed_blocks);
            }
          else
            {
              t->update(tag,commitCounter);             
              assert(node != NULL);
            }

          size_t new_root = (size_t)buildSubdivPatchTree(*subdiv_patch,node,((Scene*)geom)->getSubdivMesh(subdiv_patch->geom));
          assert( new_root != BVH4::invalidNode);

          /* update root ref */
          t->updateRootRef(new_root);

          /* upgrade write to read lock */
          t->upgrade_write_to_read_lock();
        }
      else
        CACHE_STATS(SharedTessellationCacheStats::cache_hits++);              

      /* copy data from second to first level tess cache */
      copyTessellationCacheTag(firstLevelTag,t);
      
      /* read unlock */      
      t->read_unlock();
    }
    
    TessellationCacheTag *SubdivPatch1CachedIntersector1::localTessellationCacheMissHandler(PerThreadTessellationCache *local_cache, const SubdivPatch1Cached* const subdiv_patch, const void* geom)
    {
      subdiv_patch->prefetchData();
      const unsigned int commitCounter = ((Scene*)geom)->commitCounter;
      InputTagType tag = (InputTagType)subdiv_patch;        
      const unsigned int blocks = subdiv_patch->grid_subtree_size_64b_blocks;  // FIXME: need subdivpatch ptr to get #blocks       
      TessellationCacheTag *t = local_cache->request(tag,commitCounter,blocks);
#if ENABLE_TESSELLATION_CACHE_HIERARCHY == 1
      secondLevelTessellationCacheMissHandler(t,subdiv_patch,geom);      
#else      
      size_t new_root = (size_t)buildSubdivPatchTree(*subdiv_patch,(BVH4::Node*)t->getPtr(),((Scene*)geom)->getSubdivMesh(subdiv_patch->geom));
      t->updateRootRef(new_root);
      assert( new_root != BVH4::invalidNode);      
#endif      
      return t;
    }    
    
#if 0
    TessellationCacheTag *SubdivPatch1CachedIntersector1::lookupCacheHierarchy(Precalculations& pre,
									       const SubdivPatch1Cached* const subdiv_patch,
									       const void* geom)
    {
      DBG(DBG_PRINT(pre.local_cache));                                            
    
      const unsigned int commitCounter = ((Scene*)geom)->commitCounter;
      InputTagType tag = (InputTagType)subdiv_patch;

      DBG(DBG_PRINT((size_t)tag / 320));
      TessellationCacheTag *t = pre.local_cache->lookup(tag,commitCounter);
      CACHE_STATS(DistributedTessellationCacheStats::cache_accesses++);

      if (unlikely(t == NULL)) /* L1 cache miss ? */
        {
          CACHE_STATS(DistributedTessellationCacheStats::cache_misses++);

          DBG(DBG_PRINT("L1 CACHE MISS"));                                            

          /* is data in L2 */
          TessellationCacheTag *t_l2 = sharedTessellationCache.getTag(tag);
          t_l2->read_lock();
          CACHE_STATS(SharedTessellationCacheStats::cache_accesses++);
      
          if (unlikely(!t_l2->match(tag,commitCounter))) /* not in L2 either */
            {
              DBG(DBG_PRINT("L2 CACHE MISS"));                                            
              
              subdiv_patch->prefetchData();

              CACHE_STATS(SharedTessellationCacheStats::cache_misses++);
              t_l2->read_unlock();
              t_l2->write_lock();
              /* update */
              const unsigned int needed_blocks = subdiv_patch->grid_subtree_size_64b_blocks;
              DBG(DBG_PRINT(needed_blocks));                          
          
              if (t_l2->getNumBlocks() < needed_blocks)
                {
                  DBG(DBG_PRINT("EXPAND L2 CACHE ENTRY"));                                            
                  BVH4::Node* node = (BVH4::Node*)t_l2->getPtr();
                  
                  if (node != NULL) free_tessellation_cache_mem(node);                                 

                  node = (BVH4::Node*)alloc_tessellation_cache_mem(needed_blocks);
                  DBG(DBG_PRINT(node));
              
                  CACHE_STATS(SharedTessellationCacheStats::cache_evictions++);              

                  t_l2->set(tag,commitCounter,(size_t)node,needed_blocks);
                  assert(t_l2->getPtr() == node);
                }
              else
                {
                  DBG(DBG_PRINT("REUSE L2 CACHE ENTRY"));                                                              
                  t_l2->update(tag,commitCounter);              
                  assert(t_l2->getPtr() != NULL);
                }
              
              BVH4::Node* node = (BVH4::Node*)t_l2->getPtr();
              DBG(DBG_PRINT(node));
             
              size_t new_root = (size_t)buildSubdivPatchTree(*subdiv_patch,node,((Scene*)geom)->getSubdivMesh(subdiv_patch->geom));
              DBG(DBG_PRINT(new_root));

              const size_t l2_range0 = (size_t)node;
              const size_t l2_range1 = ((size_t)node) + 64*needed_blocks;

              size_t l2_blocks = countBlocks(BVH4::NodeRef(new_root),l2_range0,l2_range1);
              assert( l2_blocks == needed_blocks );

              assert( new_root != BVH4::invalidNode);          
              t_l2->updateRootRef(new_root);
              assert(t_l2->getRootRef() == new_root);
              
              /* get L1 cache tag to evict */
              TessellationCacheTag &t_l1 = pre.local_cache->request(tag,commitCounter,needed_blocks);
             
              /* copy data to L1 */
              DBG(DBG_PRINT("INIT FROM SHARED CACHE TAG"));                                                              

              t_l1.update(tag,commitCounter);

              DBG(DBG_PRINT("updateNodeRefs"));                                                              
              DBG(DBG_PRINT(t_l2->getPtr()));
              DBG(DBG_PRINT(t_l1.getPtr()));
              assert(t_l1.getNumBlocks() >= needed_blocks);

              memcpy(t_l1.getPtr(),t_l2->getPtr(),64*needed_blocks);
              size_t t_l2_root = t_l2->getRootRef();
              updateBVH4Refs(t_l2_root,(size_t)t_l2->getPtr(),(size_t)t_l1.getPtr());
              t_l1.updateRootRef( ((size_t)t_l2_root - (size_t)t_l2->getPtr()) + (size_t)t_l1.getPtr() );
              DBG(DBG_PRINT(t_l1.subtree_root));

              DBG(t_l2->print());
              DBG(t_l1.print());
              const size_t l1_range0 = (size_t)t_l1.getPtr();
              const size_t l1_range1 = l1_range0 + 64*needed_blocks;
              size_t l1_blocks = countBlocks(BVH4::NodeRef(t_l1.getRootRef()),l1_range0,l1_range1);
              
              assert(l1_blocks >= needed_blocks);

              DBG(pre.local_cache->print());
              
              /* return data from L1 */
              assert( t_l1.match(tag,commitCounter));
              BVH4::NodeRef l1_root = t_l1.getRootRef();
              DBG(DBG_PRINT(l1_root));

              /* write unlock */
              t_l2->write_unlock();

              return l1_root;          

            }
          else
            {
              DBG(DBG_PRINT("L2 CACHE HIT"));                                            
              DBG(pre.local_cache->print());
              
              CACHE_STATS(SharedTessellationCacheStats::cache_hits++);
              TessellationCacheTag &t_l1 = pre.local_cache->request(tag,commitCounter,t_l2->getNumBlocks());

              DBG(t_l1.print());
              
              /* copy data to L1 */
              DBG(DBG_PRINT("INIT FROM SHARED CACHE TAG"));
              t_l1.update(tag,commitCounter);

              DBG(DBG_PRINT("updateNodeRefs"));                                                                            
              memcpy(t_l1.getPtr(),t_l2->getPtr(),64*t_l2->getNumBlocks());
              size_t t_l2_root = t_l2->getRootRef();
              updateBVH4Refs(t_l2_root,(size_t)t_l2->getPtr(),(size_t)t_l1.getPtr());
              t_l1.updateRootRef( (size_t)t_l2_root - (size_t)t_l2->getPtr() + (size_t)t_l1.getPtr() );
              DBG(DBG_PRINT(t_l1.subtree_root));

              BVH4::NodeRef l1_root = t_l1.getRootRef();
              
              t_l2->read_unlock();

              /* return data from L1 */
              return l1_root;                        
            }
        }
      CACHE_STATS(DistributedTessellationCacheStats::cache_hits++);

      DBG(DBG_PRINT("L1 CACHE HIT"));                                            
      return root;          
    }
#endif    

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
    
    void SubdivPatch1CachedIntersector1::createTessellationCache()
    {
      PerThreadTessellationCache *cache = (PerThreadTessellationCache *)_mm_malloc(sizeof(PerThreadTessellationCache),64);
      assert( (size_t)cache % 64 == 0 );
      cache->init();	
#if defined(DEBUG) && 0
      static AtomicMutex mtx;
      mtx.lock();
      std::cout << "Enabling tessellation cache with " << cache->allocated64ByteBlocks() << " blocks = " << cache->allocated64ByteBlocks()*64 << " bytes as default size" << std::endl;
      mtx.unlock();
#endif
      thread_cache = cache;
    }
    
  };
}
