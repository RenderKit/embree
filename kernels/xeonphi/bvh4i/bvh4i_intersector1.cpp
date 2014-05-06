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

#include "bvh4i_intersector1.h"

#include "geometry/triangle1_intersector1_moeller.h"
#include "geometry/virtual_accel_intersector1.h"

namespace embree
{
  namespace isa
  {
    static unsigned int BVH4I_LEAF_MASK = BVH4i::leaf_mask; // needed due to compiler efficiency bug

    static __aligned(64) int zlc4[4] = {0xffffffff,0xffffffff,0xffffffff,0};
    
    void BVH4iIntersector1::intersect(BVH4i* bvh, Ray& ray)
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


	  /* intersect one ray against four triangles */

	  //////////////////////////////////////////////////////////////////////////////////////////////////

	  const Triangle1* tptr  = (Triangle1*) curNode.leaf(accel);
	  const mic_i and_mask = broadcast4to16i(zlc4);

	  bool hit = Triangle1Intersector1MoellerTrumbore::intersect1(dir_xyz,
								      org_xyz,
								      min_dist_xyz,
								      max_dist_xyz,
								      and_mask,
								      ray,
								      (Scene*)bvh->geometry,
								      tptr);
	  if (hit)
	    compactStack(stack_node,stack_dist,sindex,max_dist_xyz);
	}
    }


    void BVH4iIntersector1::occluded(BVH4i* bvh, Ray& ray)
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


	  /* intersect one ray against four triangles */

	  //////////////////////////////////////////////////////////////////////////////////////////////////

	  const Triangle1* tptr  = (Triangle1*) curNode.leaf(accel);
	  const mic_i and_mask = broadcast4to16i(zlc4);

	  const mic_m m_final = Triangle1Intersector1MoellerTrumbore::occluded1(dir_xyz,
										org_xyz,
										min_dist_xyz,
										max_dist_xyz,
										and_mask,
										ray,
										(Scene*)bvh->geometry,
										tptr);


	  if (unlikely(any(m_final)))
	    {
	      ray.geomID = 0;
	      return;
	    }
	  //////////////////////////////////////////////////////////////////////////////////////////////////

	}
    }


    void BVH4mcIntersector1::intersect(BVH4i* bvh, Ray& ray)
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


	  /* intersect one ray against four triangles */

	  //////////////////////////////////////////////////////////////////////////////////////////////////

	  const Triangle1* tptr  = (Triangle1*) curNode.leaf(accel);
	  const mic_i and_mask = broadcast4to16i(zlc4);

	  // bool hit = Triangle1Intersector1MoellerTrumbore::intersect1(dir_xyz,
	  // 							      org_xyz,
	  // 							      min_dist_xyz,
	  // 							      max_dist_xyz,
	  // 							      and_mask,
	  // 							      ray,
	  // 							      (Scene*)bvh->geometry,
	  // 							      tptr);
	  // if (hit)
	  //   compactStack(stack_node,stack_dist,sindex,max_dist_xyz);
	}
    }


    void BVH4mcIntersector1::occluded(BVH4i* bvh, Ray& ray)
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


	  /* intersect one ray against four triangles */

	  //////////////////////////////////////////////////////////////////////////////////////////////////

	  const Triangle1* tptr  = (Triangle1*) curNode.leaf(accel);
	  const mic_i and_mask = broadcast4to16i(zlc4);

	  // const mic_m m_final = Triangle1Intersector1MoellerTrumbore::occluded1(dir_xyz,
	  // 									org_xyz,
	  // 									min_dist_xyz,
	  // 									max_dist_xyz,
	  // 									and_mask,
	  // 									ray,
	  // 									(Scene*)bvh->geometry,
	  // 									tptr);


	  // if (unlikely(any(m_final)))
	  //   {
	  //     ray.geomID = 0;
	  //     return;
	  //   }
	  //////////////////////////////////////////////////////////////////////////////////////////////////

	}
    }


    DEFINE_INTERSECTOR1    (BVH4iTriangle1Intersector1, BVH4iIntersector1);
    DEFINE_INTERSECTOR1    (BVH4iVirtualIntersector1, BVH4iIntersector1);
    DEFINE_INTERSECTOR1    (BVH4mcTriangle1Intersector1, BVH4iIntersector1);

  }
}
