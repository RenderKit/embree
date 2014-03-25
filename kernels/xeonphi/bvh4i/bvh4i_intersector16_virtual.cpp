// ======================================================================== //
// Copyright 2009-2013 Intel Corporation                                    //
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

#include "bvh4i_intersector16_virtual.h"
#include "geometry/virtual_accel_intersector16.h"
#include "geometry/triangle1.h"
#include "geometry/virtual_accel_intersector1.h"

namespace embree
{
  namespace isa
  {
    static unsigned int BVH4I_LEAF_MASK = BVH4i::leaf_mask; // needed due to compiler efficiency bug

    template<typename VirtualGeometryIntersector16>
    void BVH4iIntersector16Virtual<VirtualGeometryIntersector16>::intersect(mic_i* valid_i, BVH4i* bvh, Ray16& ray)
    {
      /* near and node stack */
      __aligned(64) mic_f   stack_dist[3*BVH4i::maxDepth+1];
      __aligned(64) NodeRef stack_node[3*BVH4i::maxDepth+1];

      if (unlikely(bvh == NULL)) return;

      /* load ray */
      const mic_m valid0   = *(mic_i*)valid_i != mic_i(0);
      const mic3f rdir     = rcp_safe(ray.dir);
 
      const mic3f org_rdir = ray.org * rdir;
      mic_f ray_tnear      = select(valid0,ray.tnear,pos_inf);
      mic_f ray_tfar       = select(valid0,ray.tfar ,neg_inf);
      const mic_f inf      = mic_f(pos_inf);
      
      /* allocate stack and push root node */
      stack_node[0] = BVH4i::invalidNode;
      stack_dist[0] = inf;
      stack_node[1] = bvh->root;
      stack_dist[1] = ray_tnear; 
      NodeRef* __restrict__ sptr_node = stack_node + 2;
      mic_f*   __restrict__ sptr_dist = stack_dist + 2;
      
      const Node     * __restrict__ nodes  = (Node    *)bvh->nodePtr();
      const Triangle1 * __restrict__ accel = (Triangle1*)bvh->triPtr();

      if (unlikely(accel == NULL)) return;

      while (1)
      {
        /* pop next node from stack */
        NodeRef curNode = *(sptr_node-1);
        mic_f curDist   = *(sptr_dist-1);
        sptr_node--;
        sptr_dist--;
	const mic_m m_stackDist = ray_tfar > curDist;

	/* stack emppty ? */
        if (unlikely(curNode == BVH4i::invalidNode))  break;
        
        /* cull node if behind closest hit point */
        if (unlikely(none(m_stackDist))) continue;
	        
	const unsigned int leaf_mask = BVH4I_LEAF_MASK;

	traverse_chunk_intersect(curNode,
				 curDist,
				 rdir,
				 org_rdir,
				 ray_tnear,
				 ray_tfar,
				 sptr_node,
				 sptr_dist,
				 nodes,
				 leaf_mask);            		    	
        
        /* return if stack is empty */
        if (unlikely(curNode == BVH4i::invalidNode)) break;
        
        /* intersect leaf */
        const mic_m valid_leaf = ray_tfar > curDist;
        STAT3(normal.trav_leaves,1,popcnt(valid_leaf),16);
 
	//////////////////////////////////////////////////////////////////////////////////////////////////

	unsigned int items = curNode.items();
	unsigned int index = curNode.offsetIndex(); /* array of AccelSetItems */
	Primitive *accel_ptr = (Primitive*)accel + index;

        VirtualGeometryIntersector16::intersect(valid_leaf,ray,accel_ptr,items,bvh->geometry);
	//////////////////////////////////////////////////////////////////////////////////////////////////

        ray_tfar = select(valid_leaf,ray.tfar,ray_tfar);
      }
    }
    
    template<typename VirtualGeometryIntersector16>
    void BVH4iIntersector16Virtual<VirtualGeometryIntersector16>::occluded(mic_i* valid_i, BVH4i* bvh, Ray16& ray)
    {
      /* allocate stack */
      __aligned(64) mic_f    stack_dist[3*BVH4i::maxDepth+1];
      __aligned(64) NodeRef stack_node[3*BVH4i::maxDepth+1];

      if (unlikely(bvh == NULL)) return;

      /* load ray */
      const mic_m valid = *(mic_i*)valid_i != mic_i(0);
      mic_m m_terminated = !valid;
      const mic3f rdir = rcp_safe(ray.dir);
      const mic3f org_rdir = ray.org * rdir;
      mic_f ray_tnear = select(valid,ray.tnear,pos_inf);
      mic_f ray_tfar  = select(valid,ray.tfar ,neg_inf);
      const mic_f inf = mic_f(pos_inf);
      
      /* push root node */
      stack_node[0] = BVH4i::invalidNode;
      stack_dist[0] = inf;
      stack_node[1] = bvh->root;
      stack_dist[1] = ray_tnear; 
      NodeRef* __restrict__ sptr_node = stack_node + 2;
      mic_f*   __restrict__ sptr_dist = stack_dist + 2;
      
      const Node     * __restrict__ nodes = (Node    *)bvh->nodePtr();
      const Triangle1 * __restrict__ accel = (Triangle1*)bvh->triPtr();

      if (unlikely(accel == NULL)) return;

      while (1)
      {
	const mic_m m_active = !m_terminated;

        /* pop next node from stack */
        NodeRef curNode = *(sptr_node-1);
        mic_f curDist   = *(sptr_dist-1);
        sptr_node--;
        sptr_dist--;
	const mic_m m_stackDist = gt(m_active,ray_tfar,curDist);

	/* stack emppty ? */
        if (unlikely(curNode == BVH4i::invalidNode))  break;
        
        /* cull node if behind closest hit point */
        if (unlikely(none(m_stackDist))) continue;
	
	const unsigned int leaf_mask = BVH4I_LEAF_MASK;

	traverse_chunk_occluded(curNode,
				curDist,
				rdir,
				org_rdir,
				ray_tnear,
				ray_tfar,
				m_active,
				sptr_node,
				sptr_dist,
				nodes,
				leaf_mask);            		    	
        
        /* return if stack is empty */
        if (unlikely(curNode == BVH4i::invalidNode)) break;
        
        /* intersect leaf */
        mic_m valid_leaf = gt(m_active,ray_tfar,curDist);
        STAT3(shadow.trav_leaves,1,popcnt(valid_leaf),16);

	//////////////////////////////////////////////////////////////////////////////////////////////////

	unsigned int items = curNode.items();
	unsigned int index = curNode.offsetIndex(); /* array of AccelSetItems */
	Primitive *accel_ptr = (Primitive *)accel + index;

        m_terminated |= valid_leaf & VirtualGeometryIntersector16::occluded(valid_leaf,ray,accel_ptr,items,bvh->geometry);

	//////////////////////////////////////////////////////////////////////////////////////////////////

        if (unlikely(all(m_terminated))) break;
        ray_tfar = select(m_terminated,neg_inf,ray_tfar);
      }
      store16i(valid & m_terminated,&ray.geomID,0);
    }

    template<typename VirtualGeometryIntersector1>
    void BVH4iIntersector1Virtual<VirtualGeometryIntersector1>::intersect(BVH4i* bvh, Ray& ray)
    {
      /* near and node stack */
      __aligned(64) float   stack_dist[3*BVH4i::maxDepth+1];
      __aligned(64) NodeRef stack_node[3*BVH4i::maxDepth+1];

      if (unlikely(bvh == NULL)) return;

      /* setup */
      const mic3f rdir16     = rcp_safe(mic3f(mic_f(ray.dir.x),mic_f(ray.dir.y),mic_f(ray.dir.z)));
      const mic_f inf        = mic_f(pos_inf);
      const mic_f zero       = mic_f::zero();

      store16f(stack_dist,inf);

      const Node      * __restrict__ nodes = (Node    *)bvh->nodePtr();
      const Triangle1 * __restrict__ accel = (Triangle1*)bvh->triPtr();

      if (unlikely(accel == NULL)) return;

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
            	  	    
	  traverse_single_intersect(curNode,
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

	  //////////////////////////////////////////////////////////////////////////////////////////////////

	  unsigned int items = curNode.items();
	  unsigned int index = curNode.offsetIndex(); /* array of AccelSetItems */
	  Primitive *accel_ptr = (Primitive *)accel + index;

	  VirtualGeometryIntersector1::intersect(ray,accel_ptr,items,bvh->geometry);

	  if (unlikely(any(max_dist_xyz != broadcast1to16f(&ray.tfar))))
	    {
	      max_dist_xyz = broadcast1to16f(&ray.tfar);

	      /* compact the stack if size of stack >= 2 */
	      compactStack(stack_node,stack_dist,sindex,max_dist_xyz);

	    }

	  //////////////////////////////////////////////////////////////////////////////////////////////////

	}	  
    }

    template<typename VirtualGeometryIntersector1>
    void BVH4iIntersector1Virtual<VirtualGeometryIntersector1>::occluded(BVH4i* bvh, Ray& ray)
    {
      /* near and node stack */
      __aligned(64) NodeRef stack_node[3*BVH4i::maxDepth+1];

      if (unlikely(bvh == NULL)) return;

      /* setup */
      const mic3f rdir16      = rcp_safe(mic3f(ray.dir.x,ray.dir.y,ray.dir.z));
      const mic_f inf         = mic_f(pos_inf);
      const mic_f zero        = mic_f::zero();

      const Node      * __restrict__ nodes = (Node     *)bvh->nodePtr();
      const Triangle1 * __restrict__ accel = (Triangle1*)bvh->triPtr();

      if (unlikely(accel == NULL)) return;

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
            
	  
	  traverse_single_occluded(curNode,
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



	  //////////////////////////////////////////////////////////////////////////////////////////////////

	  unsigned int items = curNode.items();
	  unsigned int index = curNode.offsetIndex(); /* array of AccelSetItems */
	  Primitive *accel_ptr = (Primitive *)accel + index;

	  if (VirtualGeometryIntersector1::occluded(ray,accel_ptr,items,bvh->geometry)) {
	    ray.geomID = 0;
	    return;
	  }

	  //////////////////////////////////////////////////////////////////////////////////////////////////

	}
    }
    
    
    DEFINE_INTERSECTOR16   (BVH4iVirtualGeometryIntersector16, BVH4iIntersector16Virtual<VirtualAccelIntersector16>);
    DEFINE_INTERSECTOR1    (BVH4iVirtualGeometryIntersector1, BVH4iIntersector1Virtual<VirtualAccelIntersector1>);

  }
}
