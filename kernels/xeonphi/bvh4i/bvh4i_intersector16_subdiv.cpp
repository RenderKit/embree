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

  __forceinline vfloat16 load4x4f_unalign(const void *__restrict__ const ptr0,
				       const void *__restrict__ const ptr1,
				       const void *__restrict__ const ptr2,
				       const void *__restrict__ const ptr3) 
  {
    vfloat16 v = vfloat16::loadu((float*)ptr0);
    v = vfloat16::loadu(v,0xf0,(float*)ptr1);
    v = vfloat16::loadu(v,0xf00,(float*)ptr2);
    v = vfloat16::loadu(v,0xf000,(float*)ptr3);
    return v;
  }


  static double msec = 0.0;

  namespace isa
  {

    //__thread LocalTessellationCacheThreadInfo* localThreadInfo = nullptr;


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
				 vfloat16 *const lazymem,
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
	  const vbool16 m_mask = ((unsigned int)1 << u_size)-1;

	  vfloat16 min_x = pos_inf;
	  vfloat16 min_y = pos_inf;
	  vfloat16 min_z = pos_inf;
	  vfloat16 max_x = neg_inf;
	  vfloat16 max_y = neg_inf;
	  vfloat16 max_z = neg_inf;

#if 0
	  for (size_t v = 0; v<v_size; v++)
	    {
	      prefetch<PFHINT_NT>(&grid_x_array[ offset ]);
	      prefetch<PFHINT_NT>(&grid_y_array[ offset ]);
	      prefetch<PFHINT_NT>(&grid_z_array[ offset ]);

	      const vfloat16 x = vfloat16::loadu(&grid_x_array[ offset ]);
	      const vfloat16 y = vfloat16::loadu(&grid_y_array[ offset ]);
	      const vfloat16 z = vfloat16::loadu(&grid_z_array[ offset ]);
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



    size_t initLocalLazySubdivTreeCompact(const SubdivPatch1 &patch,
                                          ThreadWorkState *const t_state,
                                          const SubdivMesh* const geom)
    {
      __aligned(64) float local_grid_u[(patch.grid_size_simd_blocks+1)*16]; // for unaligned access
      __aligned(64) float local_grid_v[(patch.grid_size_simd_blocks+1)*16];
      __aligned(64) float local_grid_x[(patch.grid_size_simd_blocks+1)*16]; 
      __aligned(64) float local_grid_y[(patch.grid_size_simd_blocks+1)*16];
      __aligned(64) float local_grid_z[(patch.grid_size_simd_blocks+1)*16];


      TIMER(double msec);
      TIMER(msec = getSeconds());    

      const size_t array_elements = patch.grid_size_simd_blocks * 16;



      evalGrid(patch,0,patch.grid_u_res-1,0,patch.grid_v_res-1,patch.grid_u_res,patch.grid_v_res,local_grid_x,local_grid_y,local_grid_z,local_grid_u,local_grid_v,geom);

      // ================================================================================================

      /* lock the cache */
      SharedLazyTessellationCache::sharedLazyTessellationCache.lockThreadLoop(t_state);

      /* allocate memory */
      size_t block_index = SharedLazyTessellationCache::sharedLazyTessellationCache.allocIndexLoop(t_state,patch.grid_subtree_size_64b_blocks);

      vfloat16* lazymem   = (vfloat16*)SharedLazyTessellationCache::sharedLazyTessellationCache.getBlockPtr(block_index);
      
      // ================================================================================================

      const size_t grid_offset = patch.grid_bvh_size_64b_blocks * 16;
      assert( patch.grid_subtree_size_64b_blocks * 16 >= grid_offset + 4 * array_elements);

      float *const grid_x  = (float*)lazymem + grid_offset + 0 * array_elements;
      float *const grid_y  = (float*)lazymem + grid_offset + 1 * array_elements;
      float *const grid_z  = (float*)lazymem + grid_offset + 2 * array_elements;
      int   *const grid_uv = (int*)  lazymem + grid_offset + 3 * array_elements;

      memcpy(grid_x,local_grid_x,array_elements*sizeof(float));
      memcpy(grid_y,local_grid_y,array_elements*sizeof(float));
      memcpy(grid_z,local_grid_z,array_elements*sizeof(float));

      for (size_t i=0;i<array_elements;i+=16)
	{
	  prefetch<PFHINT_L1EX>(&grid_uv[i]);
	  const vfloat16 u = vfloat16::load(&local_grid_u[i]);
	  const vfloat16 v = vfloat16::load(&local_grid_v[i]);
	  const vint16 u_i = vint16(u * 65535.0f/2.0f);
	  const vint16 v_i = vint16(v * 65535.0f/2.0f);
	  const vint16 uv_i = (v_i << 16) | u_i;
          vint16::store(&grid_uv[i],uv_i);
	}


      BVH4i::NodeRef subtree_root = 0;

      const unsigned int grid_u_blocks = (patch.grid_u_res + U_BLOCK_SIZE-2) / (U_BLOCK_SIZE-1);
      const unsigned int grid_v_blocks = (patch.grid_v_res + V_BLOCK_SIZE-2) / (V_BLOCK_SIZE-1);

      unsigned int localCounter = 0;
      BBox3fa bounds = createSubTreeCompact( subtree_root,
					     lazymem,
					     patch,
					     grid_x,
					     array_elements,
					     GridRange(0,grid_u_blocks,0,grid_v_blocks),
					     localCounter);

      assert(localCounter == patch.grid_bvh_size_64b_blocks);

      return (size_t)subtree_root + (size_t)lazymem;
    }


    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////



    __forceinline size_t lazyBuildPatch(const unsigned int patchIndex,
					const unsigned int globalTime,
					SubdivPatch1* const patches,
					Scene *const scene,
                                        ThreadWorkState *const t_state,
					BVH4i* bvh)
    {
      while(1)
      {
        SharedLazyTessellationCache::sharedLazyTessellationCache.lockThreadLoop(t_state);
        
        SubdivPatch1* subdiv_patch = &patches[patchIndex];
        
        const size_t index = SharedLazyTessellationCache::lookupIndex(&subdiv_patch->root_ref,globalTime);
        if (index != -1) return index;

        SharedLazyTessellationCache::sharedLazyTessellationCache.unlockThread(t_state);		  
        
        subdiv_patch->write_lock();
        if (!SharedLazyTessellationCache::validTag(subdiv_patch->root_ref,globalTime)) 
        {
          const SubdivMesh* const geom = (SubdivMesh*)scene->get(subdiv_patch->geom); 

          /* generate vertex grid, lock and allocate memory in the cache */
          size_t new_root_ref = initLocalLazySubdivTreeCompact(*subdiv_patch,t_state,geom);

          /* get current commit index */
          const size_t combinedTime = SharedLazyTessellationCache::sharedLazyTessellationCache.getTime(globalTime);
          __memory_barrier();
          
          CACHE_STATS(SharedTessellationCacheStats::incPatchBuild(patchIndex,bvh->numPrimitives));
          
          subdiv_patch->root_ref = SharedLazyTessellationCache::Tag((void*)new_root_ref,combinedTime);
          subdiv_patch->write_unlock();
          return SharedLazyTessellationCache::lookupIndex(&subdiv_patch->root_ref,globalTime);
        }
        subdiv_patch->write_unlock();
      }     
    }
    

    static unsigned int BVH4I_LEAF_MASK = BVH4i::leaf_mask; // needed due to compiler efficiency bug
    static unsigned int M_LANE_7777 = 0x7777;               // needed due to compiler efficiency bug

    // ============================================================================================
    // ============================================================================================
    // ============================================================================================


    void BVH4iIntersector16Subdiv::intersect(vint16* valid_i, BVH4i* bvh, Ray16& ray16)
    {
      /* near and node stack */
      __aligned(64) float   stack_dist[4*BVH4i::maxDepth+1];
      __aligned(64) NodeRef stack_node[4*BVH4i::maxDepth+1];

      /* setup */
      const vbool16 m_valid    = *(vint16*)valid_i != vint16(0);
      const Vec3vf16 rdir16     = rcp_safe(ray16.dir);
      const vfloat16 inf        = vfloat16(pos_inf);
      const vfloat16 zero       = vfloat16::zero();

      vfloat16::store(stack_dist,inf);

      const vint16 old_primID = ray16.primID;
      const vint16 old_geomID = ray16.geomID;

      ray16.primID = select(m_valid,vint16(-1),ray16.primID);

      Scene *const scene                         = (Scene*)bvh->geometry;
      const Node      * __restrict__ const nodes = (Node     *)bvh->nodePtr();
      Triangle1 * __restrict__ const accel       = (Triangle1*)bvh->triPtr();
      const unsigned int commitCounter           = scene->commitCounter;

      ThreadWorkState *const t_state = SharedLazyTessellationCache::threadState();

      stack_node[0] = BVH4i::invalidNode;
      long rayIndex = -1;
      while((rayIndex = bitscan64(rayIndex,toInt(m_valid))) != BITSCAN_NO_BIT_SET_64)	    
        {

	  STAT3(normal.travs,1,1,1);

	  stack_node[1] = bvh->root;
	  size_t sindex = 2;

	  const vfloat16 org_xyz      = loadAOS4to16f(rayIndex,ray16.org.x,ray16.org.y,ray16.org.z);
	  const vfloat16 dir_xyz      = loadAOS4to16f(rayIndex,ray16.dir.x,ray16.dir.y,ray16.dir.z);
	  const vfloat16 rdir_xyz     = loadAOS4to16f(rayIndex,rdir16.x,rdir16.y,rdir16.z);
	  //const vfloat16 org_rdir_xyz = org_xyz * rdir_xyz;
	  const vfloat16 min_dist_xyz = vfloat16::broadcast(&ray16.tnear[rayIndex]);
	  vfloat16       max_dist_xyz = vfloat16::broadcast(&ray16.tfar[rayIndex]);

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
	      const size_t cached_64bit_root = lazyBuildPatch(patchIndex,commitCounter,(SubdivPatch1*)accel,scene,t_state,bvh);
	      const BVH4i::NodeRef subtree_root = extractBVH4iNodeRef(cached_64bit_root); 
	      float *const lazyCachePtr = (float*)((size_t)SharedLazyTessellationCache::sharedLazyTessellationCache.getDataPtr() + (size_t)extractBVH4iOffset(cached_64bit_root));
              vbool16 m_quad3x5 = 0xffff; 
              if (unlikely(patch.grid_u_res < 5))
              {
                const unsigned int m_row = ((unsigned int)1 << (2*(patch.grid_u_res-1)))-1;
                m_quad3x5 = m_row | (m_row << 8);
              }
              if (unlikely(patch.grid_v_res <= 2)) { m_quad3x5 &= ~0xff00; }

	      // ----------------------------------------------------------------------------------------------------

	      STAT3(normal.trav_prims,1,1,1);

	      // -------------------------------------
	      // -------------------------------------
	      // -------------------------------------

	      float   * __restrict__ const sub_stack_dist = &stack_dist[sindex];
	      NodeRef * __restrict__ const sub_stack_node = &stack_node[sindex];
	      sub_stack_node[0] = BVH4i::invalidNode;
	      sub_stack_node[1] = subtree_root;
	      vfloat16::storeu(sub_stack_dist,inf);
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
		  quad3x5.intersect1_tri16_precise(rayIndex,dir_xyz,org_xyz,ray16,precalculations,patchIndex,m_quad3x5);		  
		}

	      SharedLazyTessellationCache::sharedLazyTessellationCache.unlockThread(t_state);

	      // -------------------------------------
	      // -------------------------------------
	      // -------------------------------------

	      compactStack(stack_node,stack_dist,sindex,vfloat16(ray16.tfar[rayIndex]));

	      // ------------------------
	    }
	}


      /* update primID/geomID and compute normals, primID was reset to -1, update only changed slots */
      const vint16 new_primID = ray16.primID;
      ray16.geomID     = old_geomID;
      ray16.primID     = old_primID;

      vbool16 m_hit = (new_primID != -1) & m_valid; 
      rayIndex = -1;
      while((rayIndex = bitscan64(rayIndex,toInt(m_hit))) != BITSCAN_NO_BIT_SET_64)	    
        {
	  const SubdivPatch1& subdiv_patch = ((SubdivPatch1*)accel)[new_primID[rayIndex]];
	  ray16.primID[rayIndex] = subdiv_patch.prim;
	  ray16.geomID[rayIndex] = subdiv_patch.geom;
	}

    }

    void BVH4iIntersector16Subdiv::occluded(vint16* valid_i, BVH4i* bvh, Ray16& ray16)
    {
      /* near and node stack */
      __aligned(64) NodeRef stack_node[4*BVH4i::maxDepth+1];

      /* setup */
      const vbool16 m_valid = *(vint16*)valid_i != vint16(0);
      const Vec3vf16 rdir16  = rcp_safe(ray16.dir);
      vbool16 terminated    = !m_valid;
      const vfloat16 inf     = vfloat16(pos_inf);
      const vfloat16 zero    = vfloat16::zero();

      Scene *const scene                   = (Scene*)bvh->geometry;
      const Node      * __restrict__ nodes = (Node     *)bvh->nodePtr();
      const Triangle1 * __restrict__ accel = (Triangle1*)bvh->triPtr();
      const unsigned int commitCounter           = scene->commitCounter;

      ThreadWorkState *const t_state = SharedLazyTessellationCache::threadState();

      stack_node[0] = BVH4i::invalidNode;
      //ray16.primID = select(m_valid,vint16(-1),ray16.primID);

      long rayIndex = -1;
      while((rayIndex = bitscan64(rayIndex,toInt(m_valid))) != BITSCAN_NO_BIT_SET_64)	    
        {
	  stack_node[1] = bvh->root;
	  size_t sindex = 2;

	  STAT3(shadow.travs,1,1,1);

	  const vfloat16 org_xyz      = loadAOS4to16f(rayIndex,ray16.org.x,ray16.org.y,ray16.org.z);
	  const vfloat16 dir_xyz      = loadAOS4to16f(rayIndex,ray16.dir.x,ray16.dir.y,ray16.dir.z);
	  const vfloat16 rdir_xyz     = loadAOS4to16f(rayIndex,rdir16.x,rdir16.y,rdir16.z);
	  //const vfloat16 org_rdir_xyz = org_xyz * rdir_xyz;
	  const vfloat16 min_dist_xyz = vfloat16::broadcast(&ray16.tnear[rayIndex]);
	  const vfloat16 max_dist_xyz = vfloat16::broadcast(&ray16.tfar[rayIndex]);
	  const vint16 v_invalidNode(BVH4i::invalidNode);
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
	      const size_t cached_64bit_root = lazyBuildPatch(patchIndex,commitCounter,(SubdivPatch1*)accel,scene,t_state,bvh);
	      const BVH4i::NodeRef subtree_root = extractBVH4iNodeRef(cached_64bit_root); 
	      float *const lazyCachePtr = (float*)((size_t)SharedLazyTessellationCache::sharedLazyTessellationCache.getDataPtr() + (size_t)extractBVH4iOffset(cached_64bit_root));
              vbool16 m_quad3x5 = 0xffff; 
              if (unlikely(patch.grid_u_res < 5))
              {
                const unsigned int m_row = ((unsigned int)1 << (2*(patch.grid_u_res-1)))-1;
                m_quad3x5 = m_row | (m_row << 8);
              }
              if (unlikely(patch.grid_v_res <= 2)) { m_quad3x5 &= ~0xff00; }

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
								 precalculations,
                                                                 m_quad3x5)))
		      {
			terminated |= (vbool16)((unsigned int)1 << rayIndex);
			break;
		      }		  
		  }

		SharedLazyTessellationCache::sharedLazyTessellationCache.unlockThread(t_state);
	      }

	      if (unlikely(terminated & (vbool16)((unsigned int)1 << rayIndex))) break;
	      //////////////////////////////////////////////////////////////////////////////////////////////////

	    }

	  if (unlikely(all(toMask(terminated)))) break;
	}


      vint16::store(m_valid & toMask(terminated),&ray16.geomID,0);

    }


    void BVH4iIntersector1Subdiv::intersect(BVH4i* bvh, Ray& ray)
    {
      /* near and node stack */
      __aligned(64) float   stack_dist[3*BVH4i::maxDepth+1];
      __aligned(64) NodeRef stack_node[3*BVH4i::maxDepth+1];

      STAT3(normal.travs,1,1,1);

      /* setup */
      const Vec3vf16 rdir16     = rcp_safe(Vec3vf16(vfloat16(ray.dir.x),vfloat16(ray.dir.y),vfloat16(ray.dir.z)));
      const vfloat16 inf        = vfloat16(pos_inf);
      const vfloat16 zero       = vfloat16::zero();

      const unsigned int oldID = ray.primID;

      vfloat16::store(stack_dist,inf);

      const Node      * __restrict__ nodes = (Node    *)bvh->nodePtr();
      const Triangle1 * __restrict__ accel = (Triangle1*)bvh->triPtr();
      Scene *const scene                   = (Scene*)bvh->geometry;
      const unsigned int commitCounter     = scene->commitCounter;

      ThreadWorkState *const t_state = SharedLazyTessellationCache::threadState();

      stack_node[0] = BVH4i::invalidNode;      
      stack_node[1] = bvh->root;

      size_t sindex = 2;

      const vfloat16 org_xyz      = loadAOS4to16f(ray.org.x,ray.org.y,ray.org.z);
      const vfloat16 dir_xyz      = loadAOS4to16f(ray.dir.x,ray.dir.y,ray.dir.z);
      const vfloat16 rdir_xyz     = loadAOS4to16f(rdir16.x[0],rdir16.y[0],rdir16.z[0]);
      //const vfloat16 org_rdir_xyz = org_xyz * rdir_xyz;
      const vfloat16 min_dist_xyz = vfloat16::broadcast(&ray.tnear);
      vfloat16       max_dist_xyz = vfloat16::broadcast(&ray.tfar);
	  
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
	  const size_t cached_64bit_root = lazyBuildPatch(patchIndex,commitCounter,(SubdivPatch1*)accel,scene,t_state,bvh);
	  const BVH4i::NodeRef subtree_root = extractBVH4iNodeRef(cached_64bit_root); 
	  float *const lazyCachePtr = (float*)((size_t)SharedLazyTessellationCache::sharedLazyTessellationCache.getDataPtr() + (size_t)extractBVH4iOffset(cached_64bit_root));
          vbool16 m_quad3x5 = 0xffff; 
          if (unlikely(patch.grid_u_res < 5))
          {
            const unsigned int m_row = ((unsigned int)1 << (2*(patch.grid_u_res-1)))-1;
            m_quad3x5 = m_row | (m_row << 8);
          }
          if (unlikely(patch.grid_v_res <= 2)) { m_quad3x5 &= ~0xff00; }

	  // ----------------------------------------------------------------------------------------------------

	  STAT3(normal.trav_prims,1,1,1);

	  // -------------------------------------
	  // -------------------------------------
	  // -------------------------------------

	  float   * __restrict__ const sub_stack_dist = &stack_dist[sindex];
	  NodeRef * __restrict__ const sub_stack_node = &stack_node[sindex];
	  sub_stack_node[0] = BVH4i::invalidNode;
	  sub_stack_node[1] = subtree_root;
	  vfloat16::storeu(sub_stack_dist,inf);
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
	      quad3x5.intersect1_tri16_precise(dir_xyz,org_xyz,ray,precalculations,patchIndex,m_quad3x5);		  
	    }

	  SharedLazyTessellationCache::sharedLazyTessellationCache.unlockThread(t_state);
	  compactStack(stack_node,stack_dist,sindex,max_dist_xyz);
	}

      if (oldID != ray.primID)
	{
	  /* update primID/geomID and compute normals */
	  const SubdivPatch1& subdiv_patch = ((SubdivPatch1*)accel)[ray.primID];
	  ray.primID = subdiv_patch.prim;
	  ray.geomID = subdiv_patch.geom;
	}

    }

    void BVH4iIntersector1Subdiv::occluded(BVH4i* bvh, Ray& ray)
    {
      /* near and node stack */
      __aligned(64) NodeRef stack_node[3*BVH4i::maxDepth+1];

      STAT3(shadow.travs,1,1,1);

      /* setup */
      const Vec3vf16 rdir16      = rcp_safe(Vec3vf16(ray.dir.x,ray.dir.y,ray.dir.z));
      const vfloat16 inf         = vfloat16(pos_inf);
      const vfloat16 zero        = vfloat16::zero();

      const Node      * __restrict__ nodes = (Node     *)bvh->nodePtr();
      const Triangle1 * __restrict__ accel = (Triangle1*)bvh->triPtr();
      Scene *const scene                   = (Scene*)bvh->geometry;
      const unsigned int commitCounter     = scene->commitCounter;

      ThreadWorkState *const t_state = SharedLazyTessellationCache::threadState();

      stack_node[0] = BVH4i::invalidNode;
      stack_node[1] = bvh->root;
      size_t sindex = 2;

      const vfloat16 org_xyz      = loadAOS4to16f(ray.org.x,ray.org.y,ray.org.z);
      const vfloat16 dir_xyz      = loadAOS4to16f(ray.dir.x,ray.dir.y,ray.dir.z);
      const vfloat16 rdir_xyz     = loadAOS4to16f(rdir16.x[0],rdir16.y[0],rdir16.z[0]);
      //const vfloat16 org_rdir_xyz = org_xyz * rdir_xyz;
      const vfloat16 min_dist_xyz = vfloat16::broadcast(&ray.tnear);
      const vfloat16 max_dist_xyz = vfloat16::broadcast(&ray.tfar);

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
	  const size_t cached_64bit_root = lazyBuildPatch(patchIndex,commitCounter,(SubdivPatch1*)accel,scene,t_state,bvh);
	  const BVH4i::NodeRef subtree_root = extractBVH4iNodeRef(cached_64bit_root); 
	  float *const lazyCachePtr = (float*)((size_t)SharedLazyTessellationCache::sharedLazyTessellationCache.getDataPtr() + (size_t)extractBVH4iOffset(cached_64bit_root));
          vbool16 m_quad3x5 = 0xffff; 
          if (unlikely(patch.grid_u_res < 5))
          {
            const unsigned int m_row = ((unsigned int)1 << (2*(patch.grid_u_res-1)))-1;
            m_quad3x5 = m_row | (m_row << 8);
          }
          if (unlikely(patch.grid_v_res <= 2)) { m_quad3x5 &= ~0xff00; }
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
							   precalculations,
                                                           m_quad3x5)))
		{
		  SharedLazyTessellationCache::sharedLazyTessellationCache.unlockThread(t_state);
		  ray.geomID = 0;
		  return;
		}		  
	    }


	  SharedLazyTessellationCache::sharedLazyTessellationCache.unlockThread(t_state);



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
