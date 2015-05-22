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

#include "bvh4i_intersector1.h"
#include "bvh4i_leaf_intersector.h"
#include "../geometry/virtual_accel_intersector1.h"
#include "../geometry/subdiv_intersector16.h"

namespace embree
{
  namespace isa
  {
    static unsigned int BVH4I_LEAF_MASK = BVH4i::leaf_mask; // needed due to compiler efficiency bug
    
    template<typename LeafIntersector, bool ENABLE_COMPRESSED_BVH4I_NODES, bool ROBUST>
    void BVH4iIntersector1<LeafIntersector,ENABLE_COMPRESSED_BVH4I_NODES,ROBUST>::intersect(BVH4i* bvh, Ray& ray)
    {
      /* near and node stack */
      __aligned(64) float   stack_dist[3*BVH4i::maxDepth+1];
      __aligned(64) NodeRef stack_node[3*BVH4i::maxDepth+1];

      /* setup */
      //const bool16 m_valid    = *(int16*)valid_i != int16(0);
      const Vec3f16 rdir16     = rcp_safe(Vec3f16(float16(ray.dir.x),float16(ray.dir.y),float16(ray.dir.z)));
      const float16 inf        = float16(pos_inf);
      const float16 zero       = float16::zero();

      store16f(stack_dist,inf);

      const Node      * __restrict__ nodes = (Node    *)bvh->nodePtr();
      const Triangle1 * __restrict__ accel = (Triangle1*)bvh->triPtr();

      stack_node[0] = BVH4i::invalidNode;      
      stack_node[1] = bvh->root;

      size_t sindex = 2;

      const float16 org_xyz      = loadAOS4to16f(ray.org.x,ray.org.y,ray.org.z);
      const float16 dir_xyz      = loadAOS4to16f(ray.dir.x,ray.dir.y,ray.dir.z);
      const float16 rdir_xyz     = loadAOS4to16f(rdir16.x[0],rdir16.y[0],rdir16.z[0]);
      //const float16 org_rdir_xyz = org_xyz * rdir_xyz;
      const float16 min_dist_xyz = broadcast1to16f(&ray.tnear);
      float16       max_dist_xyz = broadcast1to16f(&ray.tfar);
      const Precalculations precalculations(org_xyz,rdir_xyz);
	  
      const unsigned int leaf_mask = BVH4I_LEAF_MASK;
      while (1)
	{
	  NodeRef curNode = stack_node[sindex-1];
	  sindex--;

	  traverse_single_intersect<ENABLE_COMPRESSED_BVH4I_NODES,ROBUST>(curNode,
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


	  /* intersect one ray against four triangles */

	  //////////////////////////////////////////////////////////////////////////////////////////////////

	  bool hit = LeafIntersector::intersect(curNode,
						dir_xyz,
						org_xyz,
						min_dist_xyz,
						max_dist_xyz,
						ray,
						precalculations,
						accel,
						(Scene*)bvh->geometry);
	  if (hit)
	    compactStack(stack_node,stack_dist,sindex,max_dist_xyz);
	}
    }

    template<typename LeafIntersector, bool ENABLE_COMPRESSED_BVH4I_NODES, bool ROBUST>
    void BVH4iIntersector1<LeafIntersector,ENABLE_COMPRESSED_BVH4I_NODES,ROBUST>::occluded(BVH4i* bvh, Ray& ray)
    {
      /* near and node stack */
      __aligned(64) NodeRef stack_node[3*BVH4i::maxDepth+1];

      /* setup */
      const Vec3f16 rdir16      = rcp_safe(Vec3f16(ray.dir.x,ray.dir.y,ray.dir.z));
      const float16 inf         = float16(pos_inf);
      const float16 zero        = float16::zero();

      const Node      * __restrict__ nodes = (Node     *)bvh->nodePtr();
      const Triangle1 * __restrict__ accel = (Triangle1*)bvh->triPtr();

      stack_node[0] = BVH4i::invalidNode;
      stack_node[1] = bvh->root;
      size_t sindex = 2;

      const float16 org_xyz      = loadAOS4to16f(ray.org.x,ray.org.y,ray.org.z);
      const float16 dir_xyz      = loadAOS4to16f(ray.dir.x,ray.dir.y,ray.dir.z);
      const float16 rdir_xyz     = loadAOS4to16f(rdir16.x[0],rdir16.y[0],rdir16.z[0]);
      //const float16 org_rdir_xyz = org_xyz * rdir_xyz;
      const float16 min_dist_xyz = broadcast1to16f(&ray.tnear);
      const float16 max_dist_xyz = broadcast1to16f(&ray.tfar);
      const Precalculations precalculations(org_xyz,rdir_xyz);

      const unsigned int leaf_mask = BVH4I_LEAF_MASK;
	  
      while (1)
	{
	  NodeRef curNode = stack_node[sindex-1];
	  sindex--;
            
	  
	  traverse_single_occluded< ENABLE_COMPRESSED_BVH4I_NODES,ROBUST>(curNode,
									  sindex,
									  precalculations,
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
					       precalculations,
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


    typedef BVH4iIntersector1< Triangle1LeafIntersector  < true  >, false, false > Triangle1Intersector1MoellerFilter;
    typedef BVH4iIntersector1< Triangle1LeafIntersector  < false >, false, false > Triangle1Intersector1MoellerNoFilter;
    typedef BVH4iIntersector1< Triangle1mcLeafIntersector< true  >, true, false  > Triangle1mcIntersector1MoellerFilter;
    typedef BVH4iIntersector1< Triangle1mcLeafIntersector< false >, true, false  > Triangle1mcIntersector1MoellerNoFilter;
    typedef BVH4iIntersector1< VirtualLeafIntersector    < true  >, false, false > VirtualIntersector1MoellerFilter;
    typedef BVH4iIntersector1< VirtualLeafIntersector    < false >, false, false > VirtualIntersector1MoellerNoFilter;

    typedef BVH4iIntersector1< Triangle1LeafIntersectorRobust  < true  >, false, true > Triangle1Intersector1MoellerFilterRobust;
    typedef BVH4iIntersector1< Triangle1LeafIntersectorRobust  < false >, false, true > Triangle1Intersector1MoellerNoFilterRobust;

    DEFINE_INTERSECTOR1    (BVH4iTriangle1Intersector1        , Triangle1Intersector1MoellerFilter);
    DEFINE_INTERSECTOR1    (BVH4iTriangle1Intersector1NoFilter, Triangle1Intersector1MoellerNoFilter);

    DEFINE_INTERSECTOR1    (BVH4iTriangle1mcIntersector1        , Triangle1mcIntersector1MoellerFilter);
    DEFINE_INTERSECTOR1    (BVH4iTriangle1mcIntersector1NoFilter, Triangle1mcIntersector1MoellerNoFilter);

    DEFINE_INTERSECTOR1    (BVH4iVirtualGeometryIntersector1        , VirtualIntersector1MoellerFilter);
    DEFINE_INTERSECTOR1    (BVH4iVirtualGeometryIntersector1NoFilter, VirtualIntersector1MoellerNoFilter);

    DEFINE_INTERSECTOR1    (BVH4iTriangle1Intersector1Robust        , Triangle1Intersector1MoellerFilterRobust);
    DEFINE_INTERSECTOR1    (BVH4iTriangle1Intersector1NoFilterRobust, Triangle1Intersector1MoellerNoFilterRobust);

  }
}
