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

#include "subdivpatch1cached_intersector1.h"
#include "common/subdiv/tessellation.h"
#include "common/subdiv/tessellation_cache.h"
#include "xeon/bvh4/bvh4.h"
#include "xeon/bvh4/bvh4_intersector1.h"


namespace embree
{

#define TIMER(x) 
  

  __thread TessellationCache *thread_cache = NULL;

  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


  __noinline void evalGrid(const SubdivPatch1Cached &patch,
                              float *__restrict__ const grid_x,
                              float *__restrict__ const grid_y,
                              float *__restrict__ const grid_z,
                              float *__restrict__ const grid_u,
                              float *__restrict__ const grid_v,
                              const SubdivMesh* const geom)
  {
    gridUVTessellator(patch.level,
                      patch.grid_u_res,
                      patch.grid_v_res,
                      grid_u,
                      grid_v);

    if (unlikely(patch.needsStiching()))
      stichUVGrid(patch.level,patch.grid_u_res,patch.grid_v_res,grid_u,grid_v);

#if defined(__AVX__)

    for (size_t i=0;i<patch.grid_size_8wide_blocks;i++)
      {
        avxf uu = load8f(&grid_u[8*i]);
        avxf vv = load8f(&grid_v[8*i]);
        avx3f vtx = patch.eval8(uu,vv);

        if (unlikely(((SubdivMesh*)geom)->displFunc != NULL))
          {
            avx3f normal = patch.normal8(uu,vv);
            normal = normalize_safe(normal);
              
            ((SubdivMesh*)geom)->displFunc(((SubdivMesh*)geom)->userPtr,
                                           patch.geom,
                                           patch.prim,
                                           (const float*)&uu,
                                           (const float*)&vv,
                                           (const float*)&normal.x,
                                           (const float*)&normal.y,
                                           (const float*)&normal.z,
                                           (float*)&vtx.x,
                                           (float*)&vtx.y,
                                           (float*)&vtx.z,
                                           8);
          }

        *(avxf*)&grid_x[8*i] = vtx.x;
        *(avxf*)&grid_y[8*i] = vtx.y;
        *(avxf*)&grid_z[8*i] = vtx.z;        
        *(avxf*)&grid_u[8*i] = uu;
        *(avxf*)&grid_v[8*i] = vv;
      }
#else

    for (size_t i=0;i<patch.grid_size_8wide_blocks*2;i++) // 4-wide blocks for SSE
      {
        ssef uu = load4f(&grid_u[4*i]);
        ssef vv = load4f(&grid_v[4*i]);
        sse3f vtx = patch.eval4(uu,vv);

        if (unlikely(((SubdivMesh*)geom)->displFunc != NULL))
          {
            sse3f normal = patch.normal4(uu,vv);
            normal = normalize_safe(normal);
              
            ((SubdivMesh*)geom)->displFunc(((SubdivMesh*)geom)->userPtr,
                                           patch.geom,
                                           patch.prim,
                                           (const float*)&uu,
                                           (const float*)&vv,
                                           (const float*)&normal.x,
                                           (const float*)&normal.y,
                                           (const float*)&normal.z,
                                           (float*)&vtx.x,
                                           (float*)&vtx.y,
                                           (float*)&vtx.z,
                                           4);
          }

        *(ssef*)&grid_x[4*i] = vtx.x;
        *(ssef*)&grid_y[4*i] = vtx.y;
        *(ssef*)&grid_z[4*i] = vtx.z;        
        *(ssef*)&grid_u[4*i] = uu;
        *(ssef*)&grid_v[4*i] = vv;
      }

#endif
    
  }




  BBox3fa createSubTree(BVH4::NodeRef &curNode,
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

        float leaf_x_array[3][3];
        float leaf_y_array[3][3];
        float leaf_z_array[3][3];
        float leaf_u_array[3][3];
        float leaf_v_array[3][3];

        for (unsigned int v=v_start;v<=v_end;v++)
          for (unsigned int u=u_start;u<=u_end;u++)
            {
              const unsigned int local_v = v - v_start;
              const unsigned int local_u = u - u_start;
              leaf_x_array[local_v][local_u] = grid_x_array[ v * patch.grid_u_res + u ];
              leaf_y_array[local_v][local_u] = grid_y_array[ v * patch.grid_u_res + u ];
              leaf_z_array[local_v][local_u] = grid_z_array[ v * patch.grid_u_res + u ];
              leaf_u_array[local_v][local_u] = grid_u_array[ v * patch.grid_u_res + u ];
              leaf_v_array[local_v][local_u] = grid_v_array[ v * patch.grid_u_res + u ];
            }

        /* set invalid grid u,v value to border elements */
        for (unsigned int y=0;y<3;y++)
          for (unsigned int x=u_size-1;x<3;x++)
            {
              leaf_x_array[y][x] = leaf_x_array[y][u_size-1];
              leaf_y_array[y][x] = leaf_y_array[y][u_size-1];
              leaf_z_array[y][x] = leaf_z_array[y][u_size-1];
              leaf_u_array[y][x] = leaf_u_array[y][u_size-1];
              leaf_v_array[y][x] = leaf_v_array[y][u_size-1];
            }

        for (unsigned int x=0;x<3;x++)
          for (unsigned int y=v_size-1;y<3;y++)
            {
              leaf_x_array[y][x] = leaf_x_array[v_size-1][x];
              leaf_y_array[y][x] = leaf_y_array[v_size-1][x];
              leaf_z_array[y][x] = leaf_z_array[v_size-1][x];
              leaf_u_array[y][x] = leaf_u_array[v_size-1][x];
              leaf_v_array[y][x] = leaf_v_array[v_size-1][x];
            }


        qquad->init( (float*)leaf_x_array, 
                     (float*)leaf_y_array, 
                     (float*)leaf_z_array, 
                     (float*)leaf_u_array, 
                     (float*)leaf_v_array, 
                     0, 
                     3, 
                     6);

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


  __noinline BVH4::NodeRef buildSubdivPatchTree(const SubdivPatch1Cached &patch,
                                                void *const lazymem,
                                                const SubdivMesh* const geom)
  {

    TIMER(double msec = 0.0);
    TIMER(msec = getSeconds());

    assert( patch.grid_size_8wide_blocks >= 1 );
    __aligned(64) float grid_x[(patch.grid_size_8wide_blocks+1)*8]; 
    __aligned(64) float grid_y[(patch.grid_size_8wide_blocks+1)*8];
    __aligned(64) float grid_z[(patch.grid_size_8wide_blocks+1)*8]; 

    __aligned(64) float grid_u[(patch.grid_size_8wide_blocks+1)*8]; 
    __aligned(64) float grid_v[(patch.grid_size_8wide_blocks+1)*8];

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

    //TessellationCache::printStats(); 

    return subtree_root;
  }







  static AtomicMutex mtx;

  size_t SubdivPatch1CachedIntersector1::getSubtreeRootNode(const SubdivPatch1Cached* const subdiv_patch, 
                                                            const void* geom)
  {
    const unsigned int commitCounter = ((Scene*)geom)->commitCounter;

    TessellationCache *local_cache = NULL;

    if (unlikely(!thread_cache))
      {

        /* need thread cache to be aligned */
        thread_cache = (TessellationCache *)_mm_malloc(sizeof(TessellationCache),64);
        assert( (size_t)thread_cache % 64 == 0 );
        thread_cache->init();	

#if DEBUG
        mtx.lock();
        std::cout << "Enabling tessellation cache with " << thread_cache->allocated64ByteBlocks() << " blocks = " << thread_cache->allocated64ByteBlocks()*64 << " bytes as default size" << std::endl;
        mtx.unlock();
#endif
      }

    local_cache = thread_cache;

    SubdivPatch1Cached* tag = (SubdivPatch1Cached*)subdiv_patch;
    
    BVH4::NodeRef root = local_cache->lookup(tag,commitCounter);

    if (unlikely(root == (size_t)-1))
      {
        const unsigned int blocks = subdiv_patch->grid_subtree_size_64b_blocks;

        TessellationCache::CacheTag &t = local_cache->request(tag,commitCounter,blocks);
        BVH4::Node* node = (BVH4::Node*)local_cache->getCacheMemoryPtr(t);
        size_t new_root = (size_t)buildSubdivPatchTree(*subdiv_patch,node,((Scene*)geom)->getSubdivMesh(subdiv_patch->geom));
        assert( new_root != BVH4::invalidNode);

        local_cache->updateRootRef(t,new_root);

        assert( (size_t)local_cache->getPtr() + (size_t)t.getRootRef() == new_root );

        return new_root;
      }
    return root;   
  }

};
