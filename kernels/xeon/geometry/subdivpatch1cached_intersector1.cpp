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




#if defined(__AVX__)

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

        // DBG_PRINT(u_size);
        // DBG_PRINT(v_size);

        assert(u_size*v_size <= 9);

        const unsigned int currentIndex = localCounter;
        localCounter += 4; // 4 cachelines for Quad2x2

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
        // FIXME: use SIMD
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
                     6,
                     &patch);

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

    thread_cache->printStats(); 

    return subtree_root;
  }







#endif

  static AtomicMutex mtx;

  size_t SubdivPatch1CachedIntersector1::getSubtreeRootNode(const SubdivPatch1Cached* const subdiv_patch, 
                                                            const void* geom)
  {
#if defined(__AVX__) 

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


    if (unlikely(root == BVH4::invalidNode))
      {
        const unsigned int blocks = subdiv_patch->grid_subtree_size_64b_blocks;
        BVH4::NodeRef &new_root = local_cache->insert(tag,commitCounter,blocks);

        assert( new_root.isNode() );

        BVH4::Node* node = new_root.node(); // pointer to mem

        new_root = buildSubdivPatchTree(*subdiv_patch,node,((Scene*)geom)->getSubdivMesh(subdiv_patch->geom));
        assert( new_root != BVH4::invalidNode);
        //local_cache->printStats();
        return new_root;
      }
    return root;   
#endif    
  }



  // FIXME: REMOVE
//   void SubdivPatch1CachedIntersector1::intersect_subdiv_patch(const Precalculations& pre,
//                                                               Ray& ray,
//                                                               const Primitive& subdiv_patch,
//                                                               size_t ty,
//                                                               const void* geom,
//                                                               size_t& lazy_node) // geom == mesh or geom == scene?
//   {
//     /* only avx code for now */
// #if defined(__AVX__) 

//     /* stupid scanline-based grid intersector for debugging */
//     const float * const edge_levels = subdiv_patch.level;
//     const unsigned int grid_u_res   = subdiv_patch.grid_u_res;
//     const unsigned int grid_v_res   = subdiv_patch.grid_v_res;

// #if 0
//     DBG_PRINT(geom);
//     DBG_PRINT(subdiv_patch.grid_size_8wide_blocks);
//     DBG_PRINT(grid_u_res);
//     DBG_PRINT(grid_v_res);
//     DBG_PRINT(edge_levels[0]);
//     DBG_PRINT(edge_levels[1]);
//     DBG_PRINT(edge_levels[2]);
//     DBG_PRINT(edge_levels[3]);
// #endif

//     __aligned(64) float grid_x[(subdiv_patch.grid_size_8wide_blocks+1)*8]; // for unaligned access
//     __aligned(64) float grid_y[(subdiv_patch.grid_size_8wide_blocks+1)*8];
//     __aligned(64) float grid_z[(subdiv_patch.grid_size_8wide_blocks+1)*8];
//     __aligned(64) float grid_u[(subdiv_patch.grid_size_8wide_blocks+1)*8]; 
//     __aligned(64) float grid_v[(subdiv_patch.grid_size_8wide_blocks+1)*8];


//     evalGrid(subdiv_patch,grid_x,grid_y,grid_z,grid_u,grid_v,(SubdivMesh*)geom);

//     size_t offset_line0 = 0;
//     size_t offset_line1 = grid_u_res;

//     for (unsigned int y=0;y<grid_v_res-1;y++,offset_line0+=grid_u_res,offset_line1+=grid_u_res)
//       {
//         for (unsigned int x=0;x<grid_u_res-1;x+=8)
//           {
//             const size_t offset_v0 = offset_line0 + x + 0;
//             const size_t offset_v1 = offset_line0 + x + 1;
//             const size_t offset_v2 = offset_line1 + x + 1;
//             const size_t offset_v3 = offset_line1 + x + 0;
            
//             avxb m_active ( true );
//             if (unlikely(x + 8 > (grid_u_res-1))) 
//               {
//                 for (size_t i=(grid_u_res-1)%8;i<8;i++)
//                   m_active[i] = 0;
//               }
//             intersect1_quad8(ray,grid_x,grid_y,grid_z,
//                              grid_u,grid_v,
//                              offset_v0,offset_v1,offset_v2,offset_v3,
//                              m_active,
//                              &subdiv_patch,
//                              geom );

//           }
//       }	  

// #endif
//   }

//   static __forceinline void intersect1_tri8_precise(Ray& ray,
//                                                     const avx3f &v0_org,
//                                                     const avx3f &v1_org,
//                                                     const avx3f &v2_org,
//                                                     const float *__restrict__ const u_grid,
//                                                     const float *__restrict__ const v_grid,
//                                                     const size_t offset_v0,
//                                                     const size_t offset_v1,
//                                                     const size_t offset_v2,
//                                                     const avxb &m_active,
//                                                     const SubdivPatch1Cached *const sptr,
//                                                     const void* geom)
//   {
//     const avx3f O = ray.org;
//     const avx3f D = ray.dir;

//     const avx3f v0 = v0_org - O;
//     const avx3f v1 = v1_org - O;
//     const avx3f v2 = v2_org - O;
   
//     const avx3f e0 = v2 - v0;
//     const avx3f e1 = v0 - v1;	     
//     const avx3f e2 = v1 - v2;	     

//     /* calculate geometry normal and denominator */
//     const avx3f Ng1 = cross(e1,e0);
//     const avx3f Ng = Ng1+Ng1;
//     const avxf den = dot(Ng,D);
//     const avxf absDen = abs(den);
//     const avxf sgnDen = signmsk(den);
      
//     avxb valid = m_active;
//     /* perform edge tests */
//     const avxf U = dot(avx3f(cross(v2+v0,e0)),D) ^ sgnDen;
//     valid &= U >= 0.0f;
//     if (likely(none(valid))) return;
//     const avxf V = dot(avx3f(cross(v0+v1,e1)),D) ^ sgnDen;
//     valid &= V >= 0.0f;
//     if (likely(none(valid))) return;
//     const avxf W = dot(avx3f(cross(v1+v2,e2)),D) ^ sgnDen;
//     valid &= W >= 0.0f;
//     if (likely(none(valid))) return;
      
//     /* perform depth test */
//     const avxf T = dot(v0,Ng) ^ sgnDen;
//     valid &= (T >= absDen*ray.tnear) & (absDen*ray.tfar >= T);
//     if (unlikely(none(valid))) return;
      
//     /* perform backface culling */
// #if defined(RTCORE_BACKFACE_CULLING)
//     valid &= den > avxf(zero);
//     if (unlikely(none(valid))) return;
// #else
//     valid &= den != avxf(zero);
//     if (unlikely(none(valid))) return;
// #endif
            
//     /* calculate hit information */
//     const avxf rcpAbsDen = rcp(absDen);
//     const avxf u = U*rcpAbsDen;
//     const avxf v = V*rcpAbsDen;
//     const avxf t = T*rcpAbsDen;

// #if FORCE_TRIANGLE_UV == 0
//     const avxf _u0 = load8f(&u_grid[offset_v0]);
//     const avxf _u1 = load8f(&u_grid[offset_v1]);
//     const avxf _u2 = load8f(&u_grid[offset_v2]);
//     const avxf u_final = u * _u1 + v * _u2 + (1.0f-u-v) * _u0;

//     const avxf _v0 = load8f(&v_grid[offset_v0]);
//     const avxf _v1 = load8f(&v_grid[offset_v1]);
//     const avxf _v2 = load8f(&v_grid[offset_v2]);
//     const avxf v_final = u * _v1 + v * _v2 + (1.0f-u-v) * _v0;
// #else
//     const avxf u_final = u;
//     const avxf v_final = v;
// #endif


//     size_t i = select_min(valid,t);

//     /* update hit information */
//     ray.u = u_final[i];
//     ray.v = v_final[i];
//     ray.tfar = t[i];
//     ray.Ng.x = Ng.x[i];
//     ray.Ng.y = Ng.y[i];
//     ray.Ng.z = Ng.z[i];
//     ray.geomID = ((size_t)sptr)       & (unsigned int)-1; 
//     ray.primID = (((size_t)sptr)>>32) & (unsigned int)-1;
      
//   };

//   static __forceinline void intersect1_quad8(Ray& ray,
//                                              const float *__restrict__ const vtx_x,
//                                              const float *__restrict__ const vtx_y,
//                                              const float *__restrict__ const vtx_z,
//                                              const float *__restrict__ const u,
//                                              const float *__restrict__ const v,
//                                              const size_t offset_v0,
//                                              const size_t offset_v1,
//                                              const size_t offset_v2,
//                                              const size_t offset_v3,
//                                              const avxb &m_active,
//                                              const SubdivPatch1Cached *const sptr,
//                                              const void* geom)
//   {
//     const avx3f v0( load8f(&vtx_x[offset_v0]), load8f(&vtx_y[offset_v0]), load8f(&vtx_z[offset_v0]));
//     const avx3f v1( load8f(&vtx_x[offset_v1]), load8f(&vtx_y[offset_v1]), load8f(&vtx_z[offset_v1]));
//     const avx3f v2( load8f(&vtx_x[offset_v2]), load8f(&vtx_y[offset_v2]), load8f(&vtx_z[offset_v2]));
//     const avx3f v3( load8f(&vtx_x[offset_v3]), load8f(&vtx_y[offset_v3]), load8f(&vtx_z[offset_v3]));

//     intersect1_tri8_precise(ray,v0,v1,v3,u,v,offset_v0,offset_v1,offset_v3,m_active,sptr,geom);
//     intersect1_tri8_precise(ray,v3,v1,v2,u,v,offset_v3,offset_v1,offset_v2,m_active,sptr,geom);

//   }

};
