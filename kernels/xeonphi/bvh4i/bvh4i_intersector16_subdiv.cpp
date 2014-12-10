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

#include "bvh4i_intersector16_subdiv.h"
#include "bvh4i_leaf_intersector.h"
#include "geometry/subdivpatch1.h"
#include "common/subdiv/tessellation_cache.h"

#define TIMER(x) x
#define COMPUTE_SUBDIV_NORMALS_AFTER_PATCH_INTERSECTION 0

namespace embree
{
  namespace isa
  {
    __thread TessellationCache *tess_cache = NULL;


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
	  localCounter += 5; // u,v, x,y,z

	  mic_f &leaf_u_array = lazymem[currentIndex+0];
	  mic_f &leaf_v_array = lazymem[currentIndex+1];

	  leaf_u_array = mic_f::inf();
	  leaf_v_array = mic_f::inf();

	  for (unsigned int v=v_start;v<=v_end;v++)
	    for (unsigned int u=u_start;u<=u_end;u++)
	      {
		const unsigned int local_v = v - v_start;
		const unsigned int local_u = u - u_start;
		leaf_u_array[4 * local_v + local_u] = grid_u_array[ v * patch.grid_u_res + u ];
		leaf_v_array[4 * local_v + local_u] = grid_v_array[ v * patch.grid_u_res + u ];
	      }

	  /* set invalid grid u,v value to border elements */

	  for (unsigned int x=u_size-1;x<4;x++)
	    for (unsigned int y=0;y<4;y++)
	      {
		leaf_u_array[4 * y + x] = leaf_u_array[4 * y + u_size-1];
		leaf_v_array[4 * y + x] = leaf_v_array[4 * y + u_size-1];
	      }

	  for (unsigned int y=v_size-1;y<4;y++)
	    for (unsigned int x=0;x<4;x++)
	      {
		leaf_u_array[4 * y + x] = leaf_u_array[4 * (v_size-1) + x];
		leaf_v_array[4 * y + x] = leaf_v_array[4 * (v_size-1) + x];
	      }
	  
	  mic3f vtx = patch.eval16(leaf_u_array,leaf_v_array);

	  if (unlikely(geom->displFunc != NULL))
	    {
	      mic3f normal      = patch.normal16(leaf_u_array,leaf_v_array);
	      normal = normalize(normal);

	      geom->displFunc(geom->userPtr,
			      patch.geom,
			      patch.prim,
			      (const float*)&leaf_u_array,
			      (const float*)&leaf_v_array,
			      (const float*)&normal.x,
			      (const float*)&normal.y,
			      (const float*)&normal.z,
			      (float*)&vtx.x,
			      (float*)&vtx.y,
			      (float*)&vtx.z,
			      16);
	    }
	  const BBox3fa leafGridBounds = getBBox3fa(vtx);

	  mic_f &leaf_vtx_x = lazymem[currentIndex+2];
	  mic_f &leaf_vtx_y = lazymem[currentIndex+3];
	  mic_f &leaf_vtx_z = lazymem[currentIndex+4];

	  leaf_vtx_x = vtx.x;
	  leaf_vtx_y = vtx.y;
	  leaf_vtx_z = vtx.z;
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
      GridRange r[4];
      
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

      TIMER(double msec = 0.0);
      TIMER(msec = getSeconds());

      //assert( patch.grid_size_simd_blocks > 1 );
      __aligned(64) float u_array[(patch.grid_size_simd_blocks+1)*16]; // for unaligned access
      __aligned(64) float v_array[(patch.grid_size_simd_blocks+1)*16];

      // PING;
      // DBG_PRINT( patch.grid_u_res );
      // DBG_PRINT( patch.grid_v_res );
      // DBG_PRINT( patch.grid_size_simd_blocks );

      gridUVTessellatorMIC(patch.level,
			   patch.grid_u_res,
			   patch.grid_v_res,
			   u_array,
			   v_array);


      BVH4i::NodeRef subtree_root = 0;
      const unsigned int oldIndex = currentIndex;

      // PING;
      // DBG_PRINT( currentIndex );
      // DBG_PRINT( patch.grid_subtree_size_64b_blocks );

      BBox3fa bounds = createSubTree( subtree_root,
				      lazymem,
				      patch,
				      u_array,
				      v_array,
				      GridRange(0,patch.grid_u_res-1,0,patch.grid_v_res-1),
				      currentIndex,
				      geom);

      // DBG_PRINT( currentIndex - oldIndex );
      // DBG_PRINT( patch.grid_subtree_size_64b_blocks );

      assert(currentIndex - oldIndex == patch.grid_subtree_size_64b_blocks);
      TIMER(msec = getSeconds()-msec);    
      return subtree_root;
    }



    static AtomicMutex mtx;

    void createTessellationCache()
    {
      TessellationCache *cache = (TessellationCache *)_mm_malloc(sizeof(TessellationCache),64);
      assert( (size_t)cache % 64 == 0 );
      cache->init();	
#if DEBUG
      mtx.lock();
      std::cout << "Enabling tessellation cache with " << cache->allocated64ByteBlocks() << " blocks = " << cache->allocated64ByteBlocks()*64 << " bytes as default size" << std::endl;
      mtx.unlock();
#endif
      tess_cache = cache;
    }

    __forceinline BVH4i::NodeRef lookUpTessellationCache(TessellationCache *local_cache,
							 const unsigned int patchIndex,
							 const unsigned int commitCounter,
							 const SubdivPatch1* const patches,
							 Scene *const scene)
    {
      TessellationCache::InputTagType tag = (TessellationCache::InputTagType)patchIndex;

      BVH4i::NodeRef subtree_root = local_cache->lookup(tag,commitCounter);
      if (unlikely(subtree_root == BVH4i::invalidNode))
	{
	  const SubdivPatch1& subdiv_patch = patches[patchIndex];
		  
	  subdiv_patch.prefetchData();

	  const SubdivMesh* const geom = (SubdivMesh*)scene->get(subdiv_patch.geom); // FIXME: test flag first

	  const unsigned int blocks = subdiv_patch.grid_subtree_size_64b_blocks;

	  TessellationCache::CacheTag &t = local_cache->request(tag,commitCounter,blocks);		      
	  mic_f *local_mem = (mic_f*)local_cache->getPtr(); 

	  unsigned int currentIndex = t.getRootRef();

	  subtree_root = initLocalLazySubdivTree(subdiv_patch,currentIndex,local_mem,geom);		      
	  assert( subtree_root != BVH4i::invalidNode);

	  local_cache->updateRootRef(t,subtree_root);
	}
      return subtree_root;
    }



    template<bool ENABLE_INTERSECTION_FILTER>
    struct SubdivLeafIntersector
    {
      // ==================
      // === single ray === 
      // ==================
      static __forceinline bool intersect(BVH4i::NodeRef curNode,
					  const mic_f &dir_xyz,
					  const mic_f &org_xyz,
					  const mic_f &min_dist_xyz,
					  mic_f &max_dist_xyz,
					  Ray& ray, 
					  const void *__restrict__ const accel,
					  const Scene*__restrict__ const geometry)
      {
	unsigned int items = curNode.items();
	unsigned int index = curNode.offsetIndex();
	const SubdivPatch1 *__restrict__ const patch_ptr = (SubdivPatch1*)accel + index;
	FATAL("NOT IMPLEMENTED");

	// return SubdivPatchIntersector1<ENABLE_INTERSECTION_FILTER>::intersect1(dir_xyz,
	// 								       org_xyz,
	// 								       ray,
	// 								       *patch_ptr);	
      }

      static __forceinline bool occluded(BVH4i::NodeRef curNode,
					 const mic_f &dir_xyz,
					 const mic_f &org_xyz,
					 const mic_f &min_dist_xyz,
					 const mic_f &max_dist_xyz,
					 Ray& ray,
					 const void *__restrict__ const accel,
					 const Scene*__restrict__ const geometry)
      {
	unsigned int items = curNode.items();
	unsigned int index = curNode.offsetIndex();
	const SubdivPatch1 *__restrict__ const patch_ptr = (SubdivPatch1*)accel + index;
	FATAL("NOT IMPLEMENTED");
	// return SubdivPatchIntersector1<ENABLE_INTERSECTION_FILTER>::occluded1(dir_xyz,
	// 								      org_xyz,
	// 								      ray,
	// 								      *patch_ptr);	
      }


    };

    static unsigned int BVH4I_LEAF_MASK = BVH4i::leaf_mask; // needed due to compiler efficiency bug
    static unsigned int M_LANE_7777 = 0x7777;               // needed due to compiler efficiency bug

    // ============================================================================================
    // ============================================================================================
    // ============================================================================================


    template<typename LeafIntersector, bool ENABLE_COMPRESSED_BVH4I_NODES>
    void BVH4iIntersector16Subdiv<LeafIntersector,ENABLE_COMPRESSED_BVH4I_NODES>::intersect(mic_i* valid_i, BVH4i* bvh, Ray16& ray16)
    {
      /* near and node stack */
      __aligned(64) float   stack_dist[4*BVH4i::maxDepth+1];
      __aligned(64) NodeRef stack_node[4*BVH4i::maxDepth+1];

      TessellationCache *local_cache = NULL;

      if (!tess_cache)
	createTessellationCache();

      local_cache = tess_cache;


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

      stack_node[0] = BVH4i::invalidNode;
      long rayIndex = -1;
      while((rayIndex = bitscan64(rayIndex,toInt(m_valid))) != BITSCAN_NO_BIT_SET_64)	    
        {
	  stack_node[1] = bvh->root;
	  size_t sindex = 2;

	  const mic_f org_xyz      = loadAOS4to16f(rayIndex,ray16.org.x,ray16.org.y,ray16.org.z);
	  const mic_f dir_xyz      = loadAOS4to16f(rayIndex,ray16.dir.x,ray16.dir.y,ray16.dir.z);
	  const mic_f rdir_xyz     = loadAOS4to16f(rayIndex,rdir16.x,rdir16.y,rdir16.z);
	  const mic_f org_rdir_xyz = org_xyz * rdir_xyz;
	  const mic_f min_dist_xyz = broadcast1to16f(&ray16.tnear[rayIndex]);
	  mic_f       max_dist_xyz = broadcast1to16f(&ray16.tfar[rayIndex]);

	  const unsigned int leaf_mask = BVH4I_LEAF_MASK;

	  while (1)
	    {

	      NodeRef curNode = stack_node[sindex-1];
	      sindex--;

	      traverse_single_intersect<ENABLE_COMPRESSED_BVH4I_NODES>(curNode,
								       sindex,
								       rdir_xyz,
								       org_rdir_xyz,
								       min_dist_xyz,
								       max_dist_xyz,
								       stack_node,
								       stack_dist,
								       nodes,
								       leaf_mask);
		   

	      /* return if stack is empty */
	      if (unlikely(curNode == BVH4i::invalidNode)) break;

	      STAT3(normal.trav_leaves,1,1,1);
	      STAT3(normal.trav_prims,4,4,4);

	      //////////////////////////////////////////////////////////////////////////////////////////////////

	      const unsigned int patchIndex = curNode.offsetIndex();

	      BVH4i::NodeRef subtree_root = lookUpTessellationCache(local_cache,
								    patchIndex,
								    commitCounter,
								    (SubdivPatch1*)accel,
								    scene);
	      mic_f     * const __restrict__ lazymem     = (mic_f*)local_cache->getPtr(); /* lazymem could change to realloc */

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

		  traverse_single_intersect<ENABLE_COMPRESSED_BVH4I_NODES>(curNode,
									   sub_sindex,
									   rdir_xyz,
									   org_rdir_xyz,
									   min_dist_xyz,
									   max_dist_xyz,
									   sub_stack_node,
									   sub_stack_dist,
									   (BVH4i::Node*)lazymem,
									   leaf_mask);
		 		   

		  /* return if stack is empty */
		  if (unlikely(curNode == BVH4i::invalidNode)) break;

		  const unsigned int uvIndex = curNode.offsetIndex();

		  prefetch<PFHINT_NT>(&lazymem[uvIndex + 0]);
		  prefetch<PFHINT_NT>(&lazymem[uvIndex + 1]);
		  prefetch<PFHINT_NT>(&lazymem[uvIndex + 2]);
		  prefetch<PFHINT_NT>(&lazymem[uvIndex + 3]);
		  prefetch<PFHINT_NT>(&lazymem[uvIndex + 4]);
		  
		  const mic_m m_active = 0x777;
		  const mic_f &uu = lazymem[uvIndex + 0];
		  const mic_f &vv = lazymem[uvIndex + 1];	
		  const mic3f vtx(lazymem[uvIndex + 2],
				  lazymem[uvIndex + 3],
				  lazymem[uvIndex + 4]);
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
	  //ray16.u[rayIndex]      = (1.0f-ray16.u[rayIndex]) * subdiv_patch.u_range.x + ray16.u[rayIndex] * subdiv_patch.u_range.y;
	  //ray16.v[rayIndex]      = (1.0f-ray16.v[rayIndex]) * subdiv_patch.v_range.x + ray16.v[rayIndex] * subdiv_patch.v_range.y;
	  if (unlikely(subdiv_patch.hasDisplacement())) continue;
#if COMPUTE_SUBDIV_NORMALS_AFTER_PATCH_INTERSECTION == 1
	  const Vec3fa normal    = subdiv_patch.normal(ray16.u[rayIndex],ray16.v[rayIndex]);
	  ray16.Ng.x[rayIndex]   = normal.x;
	  ray16.Ng.y[rayIndex]   = normal.y;
	  ray16.Ng.z[rayIndex]   = normal.z;
#endif
	}

    }

    template<typename LeafIntersector,bool ENABLE_COMPRESSED_BVH4I_NODES>    
    void BVH4iIntersector16Subdiv<LeafIntersector,ENABLE_COMPRESSED_BVH4I_NODES>::occluded(mic_i* valid_i, BVH4i* bvh, Ray16& ray16)
    {
      /* near and node stack */
      __aligned(64) NodeRef stack_node[4*BVH4i::maxDepth+1];

      TessellationCache *local_cache = NULL;

      if (!tess_cache)
	createTessellationCache();

      local_cache = tess_cache;

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
	  const mic_f org_rdir_xyz = org_xyz * rdir_xyz;
	  const mic_f min_dist_xyz = broadcast1to16f(&ray16.tnear[rayIndex]);
	  const mic_f max_dist_xyz = broadcast1to16f(&ray16.tfar[rayIndex]);
	  const mic_i v_invalidNode(BVH4i::invalidNode);
	  const unsigned int leaf_mask = BVH4I_LEAF_MASK;

	  while (1)
	    {
	      NodeRef curNode = stack_node[sindex-1];
	      sindex--;

	      traverse_single_occluded< ENABLE_COMPRESSED_BVH4I_NODES >(curNode,
									sindex,
									rdir_xyz,
									org_rdir_xyz,
									min_dist_xyz,
									max_dist_xyz,
									stack_node,
									nodes,
									leaf_mask);

	      /* return if stack is empty */
	      if (unlikely(curNode == BVH4i::invalidNode)) break;

	      STAT3(shadow.trav_leaves,1,1,1);
	      STAT3(shadow.trav_prims,4,4,4);

	      /* intersect one ray against four triangles */

	      //////////////////////////////////////////////////////////////////////////////////////////////////

	      

	      const unsigned int patchIndex = curNode.offsetIndex();
	      BVH4i::NodeRef subtree_root = lookUpTessellationCache(local_cache,
								    patchIndex,
								    commitCounter,
								    (SubdivPatch1*)accel,
								    scene);
	      mic_f     * const __restrict__ lazymem     = (mic_f*)local_cache->getPtr(); /* lazymem could change to realloc */

		{
		    
		  // -------------------------------------
		  // -------------------------------------
		  // -------------------------------------

		  __aligned(64) float   sub_stack_dist[64];
		  __aligned(64) NodeRef sub_stack_node[64];
		  sub_stack_node[0] = BVH4i::invalidNode;
		  sub_stack_node[1] = subtree_root;
		  store16f(sub_stack_dist,inf);
		  size_t sub_sindex = 2;

		  ///////////////////////////////////////////////////////////////////////////////////////////////////////
		  while (1)
		    {
		      curNode = sub_stack_node[sub_sindex-1];
		      sub_sindex--;

		      traverse_single_intersect<ENABLE_COMPRESSED_BVH4I_NODES>(curNode,
									       sub_sindex,
									       rdir_xyz,
									       org_rdir_xyz,
									       min_dist_xyz,
									       max_dist_xyz,
									       sub_stack_node,
									       sub_stack_dist,
									       (BVH4i::Node*)lazymem,
									       leaf_mask);
		 		   

		      /* return if stack is empty */
		      if (unlikely(curNode == BVH4i::invalidNode)) break;

		      const unsigned int uvIndex = curNode.offsetIndex();

		      prefetch<PFHINT_NT>(&lazymem[uvIndex + 0]);
		      prefetch<PFHINT_NT>(&lazymem[uvIndex + 1]);
		      prefetch<PFHINT_NT>(&lazymem[uvIndex + 2]);
		      prefetch<PFHINT_NT>(&lazymem[uvIndex + 3]);
		      prefetch<PFHINT_NT>(&lazymem[uvIndex + 4]);
		  
		      const mic_m m_active = 0x777;
		      const mic_f &uu = lazymem[uvIndex + 0];
		      const mic_f &vv = lazymem[uvIndex + 1];		  
		      const mic3f vtx(lazymem[uvIndex + 2],
				      lazymem[uvIndex + 3],
				      lazymem[uvIndex + 4]);
		  
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
		}

	      bool hit = ray16.primID[rayIndex] != -1;
	      if (unlikely(hit)) break;
	      //////////////////////////////////////////////////////////////////////////////////////////////////

	    }


	  if (unlikely(all(toMask(terminated)))) break;
	}


      store16i(m_valid & toMask(terminated),&ray16.geomID,0);

    }

    template<typename LeafIntersector, bool ENABLE_COMPRESSED_BVH4I_NODES>
    void BVH4iIntersector1Subdiv<LeafIntersector,ENABLE_COMPRESSED_BVH4I_NODES>::intersect(BVH4i* bvh, Ray& ray)
    {
      /* near and node stack */
      __aligned(64) float   stack_dist[3*BVH4i::maxDepth+1];
      __aligned(64) NodeRef stack_node[3*BVH4i::maxDepth+1];

      /* setup */
      //const mic_m m_valid    = *(mic_i*)valid_i != mic_i(0);
      const mic3f rdir16     = rcp_safe(mic3f(mic_f(ray.dir.x),mic_f(ray.dir.y),mic_f(ray.dir.z)));
      const mic_f inf        = mic_f(pos_inf);
      const mic_f zero       = mic_f::zero();

      store16f(stack_dist,inf);

      const Node      * __restrict__ nodes = (Node    *)bvh->nodePtr();
      const Triangle1 * __restrict__ accel = (Triangle1*)bvh->triPtr();

      stack_node[0] = BVH4i::invalidNode;      
      stack_node[1] = bvh->root;

      size_t sindex = 2;

      const mic_f org_xyz      = loadAOS4to16f(ray.org.x,ray.org.y,ray.org.z);
      const mic_f dir_xyz      = loadAOS4to16f(ray.dir.x,ray.dir.y,ray.dir.z);
      const mic_f rdir_xyz     = loadAOS4to16f(rdir16.x[0],rdir16.y[0],rdir16.z[0]);
      const mic_f org_rdir_xyz = org_xyz * rdir_xyz;
      const mic_f min_dist_xyz = broadcast1to16f(&ray.tnear);
      mic_f       max_dist_xyz = broadcast1to16f(&ray.tfar);
	  
      const unsigned int leaf_mask = BVH4I_LEAF_MASK;
	  
      while (1)
	{
	  NodeRef curNode = stack_node[sindex-1];
	  sindex--;

	  traverse_single_intersect<ENABLE_COMPRESSED_BVH4I_NODES>(curNode,
								   sindex,
								   rdir_xyz,
								   org_rdir_xyz,
								   min_dist_xyz,
								   max_dist_xyz,
								   stack_node,
								   stack_dist,
								   nodes,
								   leaf_mask);            		    

	  /* return if stack is empty */
	  if (unlikely(curNode == BVH4i::invalidNode)) break;


	  /* intersect one ray against four triangles */

	  //////////////////////////////////////////////////////////////////////////////////////////////////

	  bool hit = LeafIntersector::intersect(curNode,
						dir_xyz,
						org_xyz,
						min_dist_xyz,
						max_dist_xyz,
						ray,
						accel,
						(Scene*)bvh->geometry);
	  if (hit)
	    compactStack(stack_node,stack_dist,sindex,max_dist_xyz);
	}
    }

    template<typename LeafIntersector, bool ENABLE_COMPRESSED_BVH4I_NODES>
    void BVH4iIntersector1Subdiv<LeafIntersector,ENABLE_COMPRESSED_BVH4I_NODES>::occluded(BVH4i* bvh, Ray& ray)
    {
      /* near and node stack */
      __aligned(64) NodeRef stack_node[3*BVH4i::maxDepth+1];

      /* setup */
      const mic3f rdir16      = rcp_safe(mic3f(ray.dir.x,ray.dir.y,ray.dir.z));
      const mic_f inf         = mic_f(pos_inf);
      const mic_f zero        = mic_f::zero();

      const Node      * __restrict__ nodes = (Node     *)bvh->nodePtr();
      const Triangle1 * __restrict__ accel = (Triangle1*)bvh->triPtr();

      stack_node[0] = BVH4i::invalidNode;
      stack_node[1] = bvh->root;
      size_t sindex = 2;

      const mic_f org_xyz      = loadAOS4to16f(ray.org.x,ray.org.y,ray.org.z);
      const mic_f dir_xyz      = loadAOS4to16f(ray.dir.x,ray.dir.y,ray.dir.z);
      const mic_f rdir_xyz     = loadAOS4to16f(rdir16.x[0],rdir16.y[0],rdir16.z[0]);
      const mic_f org_rdir_xyz = org_xyz * rdir_xyz;
      const mic_f min_dist_xyz = broadcast1to16f(&ray.tnear);
      const mic_f max_dist_xyz = broadcast1to16f(&ray.tfar);

      const unsigned int leaf_mask = BVH4I_LEAF_MASK;
	  
      while (1)
	{
	  NodeRef curNode = stack_node[sindex-1];
	  sindex--;
            
	  
	  traverse_single_occluded< ENABLE_COMPRESSED_BVH4I_NODES>(curNode,
								   sindex,
								   rdir_xyz,
								   org_rdir_xyz,
								   min_dist_xyz,
								   max_dist_xyz,
								   stack_node,
								   nodes,
								   leaf_mask);	    

	  /* return if stack is empty */
	  if (unlikely(curNode == BVH4i::invalidNode)) break;


	  /* intersect one ray against four triangles */

	  //////////////////////////////////////////////////////////////////////////////////////////////////

	  bool hit = LeafIntersector::occluded(curNode,
					       dir_xyz,
					       org_xyz,
					       min_dist_xyz,
					       max_dist_xyz,
					       ray,
					       accel,
					       (Scene*)bvh->geometry);

	  if (unlikely(hit))
	    {
	      ray.geomID = 0;
	      return;
	    }
	  //////////////////////////////////////////////////////////////////////////////////////////////////

	}
    }

    // ----------------------------------------------------------------------------------------------------------------
    // ----------------------------------------------------------------------------------------------------------------
    // ----------------------------------------------------------------------------------------------------------------



    typedef BVH4iIntersector16Subdiv< SubdivLeafIntersector    < true  >, false > SubdivIntersector16SingleMoellerFilter;
    typedef BVH4iIntersector16Subdiv< SubdivLeafIntersector    < false >, false > SubdivIntersector16SingleMoellerNoFilter;

    DEFINE_INTERSECTOR16   (BVH4iSubdivMeshIntersector16        , SubdivIntersector16SingleMoellerFilter);
    DEFINE_INTERSECTOR16   (BVH4iSubdivMeshIntersector16NoFilter, SubdivIntersector16SingleMoellerNoFilter);

    typedef BVH4iIntersector1Subdiv< SubdivLeafIntersector    < true  >, false > SubdivMeshIntersector1MoellerFilter;
    typedef BVH4iIntersector1Subdiv< SubdivLeafIntersector    < false >, false > SubdivMeshIntersector1MoellerNoFilter;

    DEFINE_INTERSECTOR1    (BVH4iSubdivMeshIntersector1        , SubdivMeshIntersector1MoellerFilter);
    DEFINE_INTERSECTOR1    (BVH4iSubdivMeshIntersector1NoFilter, SubdivMeshIntersector1MoellerNoFilter);

  }
}
