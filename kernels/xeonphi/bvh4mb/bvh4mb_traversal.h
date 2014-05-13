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

#pragma once

#include "bvh4mb.h"

namespace embree
{

  __forceinline void traverse_chunk_intersect(BVH4i::NodeRef &curNode,
					      mic_f &curDist,
					      const mic3f &rdir,
					      const mic3f &org_rdir,
					      const mic_f &ray_tnear,
					      const mic_f &ray_tfar,
					      const mic_f &time,
					      BVH4i::NodeRef *__restrict__ &sptr_node,
					      mic_f *__restrict__ &sptr_dist,
					      const BVH4i::Node      * __restrict__ const nodes,
					      const unsigned int leaf_mask)
  {


    const mic_f one_time = (mic_f::one() - time);
    while (1)
      {
	/* test if this is a leaf node */
	if (unlikely(curNode.isLeaf(leaf_mask))) break;
          
	STAT3(normal.trav_nodes,1,popcnt(ray_tfar > curDist),16);
	const BVH4i::Node* __restrict__ const node = curNode.node(nodes);

	const BVH4mb::Node* __restrict__ const nodeMB = (BVH4mb::Node*)node;

	/* pop of next node */
	sptr_node--;
	sptr_dist--;
	curNode = *sptr_node; 	  
	curDist = *sptr_dist;

	prefetch<PFHINT_L1>((mic_f*)node + 0);           
	prefetch<PFHINT_L1>((mic_f*)node + 1); 
	prefetch<PFHINT_L1>((mic_f*)node + 2); 
	prefetch<PFHINT_L1>((mic_f*)node + 3); 

#pragma unroll(4)
	for (unsigned int i=0; i<4; i++)
	  {
	    const BVH4i::NodeRef child = node->lower[i].child;

	    const mic_f lower_x =  one_time * nodeMB->lower[i].x + time * nodeMB->lower_t1[i].x;
	    const mic_f lower_y =  one_time * nodeMB->lower[i].y + time * nodeMB->lower_t1[i].y;
	    const mic_f lower_z =  one_time * nodeMB->lower[i].z + time * nodeMB->lower_t1[i].z;
	    const mic_f upper_x =  one_time * nodeMB->upper[i].x + time * nodeMB->upper_t1[i].x;
	    const mic_f upper_y =  one_time * nodeMB->upper[i].y + time * nodeMB->upper_t1[i].y;
	    const mic_f upper_z =  one_time * nodeMB->upper[i].z + time * nodeMB->upper_t1[i].z;

	    if (unlikely(i >=2 && child == BVH4i::invalidNode)) break;

	    const mic_f lclipMinX = msub(lower_x,rdir.x,org_rdir.x);
	    const mic_f lclipMinY = msub(lower_y,rdir.y,org_rdir.y);
	    const mic_f lclipMinZ = msub(lower_z,rdir.z,org_rdir.z);
	    const mic_f lclipMaxX = msub(upper_x,rdir.x,org_rdir.x);
	    const mic_f lclipMaxY = msub(upper_y,rdir.y,org_rdir.y);
	    const mic_f lclipMaxZ = msub(upper_z,rdir.z,org_rdir.z);
	    
	    const mic_f lnearP = max(max(min(lclipMinX, lclipMaxX), min(lclipMinY, lclipMaxY)), min(lclipMinZ, lclipMaxZ));
	    const mic_f lfarP  = min(min(max(lclipMinX, lclipMaxX), max(lclipMinY, lclipMaxY)), max(lclipMinZ, lclipMaxZ));
	    const mic_m lhit   = le(max(lnearP,ray_tnear),min(lfarP,ray_tfar));   
	    const mic_f childDist = select(lhit,lnearP,inf);
	    const mic_m m_child_dist = lt(childDist,curDist);
	    /* if we hit the child we choose to continue with that child if it 
	       is closer than the current next child, or we push it onto the stack */

	    if (likely(any(lhit)))
	      {
		sptr_node++;
		sptr_dist++;
              
		/* push cur node onto stack and continue with hit child */
		if (any(m_child_dist))
		  {
		    *(sptr_node-1) = curNode;
		    *(sptr_dist-1) = curDist; 
		    curDist = childDist;
		    curNode = child;
		  }              
		/* push hit child onto stack*/
		else 
		  {
		    *(sptr_node-1) = child;
		    *(sptr_dist-1) = childDist; 
		  }
	      }	      
	  }
      }
  }

  __forceinline void traverse_chunk_occluded(BVH4i::NodeRef &curNode,
					     mic_f &curDist,
					     const mic3f &rdir,
					     const mic3f &org_rdir,
					     const mic_f &ray_tnear,
					     const mic_f &ray_tfar,
					     const mic_m &m_active,
					     const mic_f &time,
					     BVH4i::NodeRef *__restrict__ &sptr_node,
					     mic_f *__restrict__ &sptr_dist,
					     const BVH4i::Node      * __restrict__ const nodes,
					     const unsigned int leaf_mask)
  {
    const mic_f one_time = (mic_f::one() - time);
    while (1)
      {
	/* test if this is a leaf node */
	if (unlikely(curNode.isLeaf(leaf_mask))) break;
          
	STAT3(shadow.trav_nodes,1,popcnt(ray_tfar > curDist),16);
	const BVH4i::Node* __restrict__ const node = curNode.node(nodes);
	const BVH4mb::Node* __restrict__ const nodeMB = (BVH4mb::Node*)node;
          
	prefetch<PFHINT_L1>((mic_f*)node + 0); 
	prefetch<PFHINT_L1>((mic_f*)node + 1); 
	prefetch<PFHINT_L1>((mic_f*)node + 2); 
	prefetch<PFHINT_L1>((mic_f*)node + 3); 

	/* pop of next node */
	sptr_node--;
	sptr_dist--;
	curNode = *sptr_node; 
	curDist = *sptr_dist;
          	 
#pragma unroll(4)
	for (unsigned int i=0; i<4; i++)
          {
	    const BVH4i::NodeRef child = node->lower[i].child;

	    const mic_f lower_x =  one_time * nodeMB->lower[i].x + time * nodeMB->lower_t1[i].x;
	    const mic_f lower_y =  one_time * nodeMB->lower[i].y + time * nodeMB->lower_t1[i].y;
	    const mic_f lower_z =  one_time * nodeMB->lower[i].z + time * nodeMB->lower_t1[i].z;
	    const mic_f upper_x =  one_time * nodeMB->upper[i].x + time * nodeMB->upper_t1[i].x;
	    const mic_f upper_y =  one_time * nodeMB->upper[i].y + time * nodeMB->upper_t1[i].y;
	    const mic_f upper_z =  one_time * nodeMB->upper[i].z + time * nodeMB->upper_t1[i].z;

	    if (unlikely(i >=2 && child == BVH4i::invalidNode)) break;

            const mic_f lclipMinX = msub(lower_x,rdir.x,org_rdir.x);
            const mic_f lclipMinY = msub(lower_y,rdir.y,org_rdir.y);
            const mic_f lclipMinZ = msub(lower_z,rdir.z,org_rdir.z);
            const mic_f lclipMaxX = msub(upper_x,rdir.x,org_rdir.x);
            const mic_f lclipMaxY = msub(upper_y,rdir.y,org_rdir.y);
            const mic_f lclipMaxZ = msub(upper_z,rdir.z,org_rdir.z);

            const mic_f lnearP = max(max(min(lclipMinX, lclipMaxX), min(lclipMinY, lclipMaxY)), min(lclipMinZ, lclipMaxZ));
            const mic_f lfarP  = min(min(max(lclipMinX, lclipMaxX), max(lclipMinY, lclipMaxY)), max(lclipMinZ, lclipMaxZ));
            const mic_m lhit   = le(m_active,max(lnearP,ray_tnear),min(lfarP,ray_tfar));      
	    const mic_f childDist = select(lhit,lnearP,inf);
            const mic_m m_child_dist = childDist < curDist;
            
            /* if we hit the child we choose to continue with that child if it 
               is closer than the current next child, or we push it onto the stack */
            if (likely(any(lhit)))
	      {
		sptr_node++;
		sptr_dist++;
              
		/* push cur node onto stack and continue with hit child */
		if (any(m_child_dist))
		  {
		    *(sptr_node-1) = curNode; 
		    *(sptr_dist-1) = curDist; 
		    curDist = childDist;
		    curNode = child;
		  }
              
		/* push hit child onto stack*/
		else {
		  *(sptr_node-1) = child;
		  *(sptr_dist-1) = childDist; 
		}
	      }	      
          }
      }

  }

};
