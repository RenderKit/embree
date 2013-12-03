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

#include "bvh4i_intersector16_chunk.h"

#include "geometry/triangle1_intersector16_moeller.h"
#include "geometry/virtual_accel_intersector16.h"
#include "common/registry_intersector.h"

namespace embree
{
  namespace isa
  {
    template<typename TriangleIntersector16>
    void BVH4iIntersector16Chunk<TriangleIntersector16>::intersect(mic_i* valid_i, BVH4i* bvh, Ray16& ray)
    {
      /* near and node stack */
      __align(64) mic_f   stack_dist[3*BVH4i::maxDepth+1];
      __align(64) NodeRef stack_node[3*BVH4i::maxDepth+1];

      /* load ray */
      const mic_m valid0   = *(mic_i*)valid_i != mic_i(0);
      const mic3f rdir     = rcp_safe(ray.dir);
      // const mic3f rdir( rcp_nr(ray.dir.x), rcp_nr(ray.dir.y), rcp_nr(ray.dir.z) );

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
      
      const Node     * __restrict__ nodes = (Node    *)bvh->nodePtr();
      const Triangle * __restrict__ accel = (Triangle*)bvh->triPtr();

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
	        
        while (1)
        {
          /* test if this is a leaf node */
          if (unlikely(curNode.isLeaf())) break;
          
          STAT3(normal.trav_nodes,1,popcnt(ray_tfar > curDist),16);
          const Node* __restrict__ const node = curNode.node(nodes);

          /* pop of next node */
          sptr_node--;
          sptr_dist--;
          curNode = *sptr_node; // FIXME: this trick creates issues with stack depth
          curDist = *sptr_dist;
          
	  prefetch<PFHINT_L1>((mic_f*)node + 1); // depth first order, prefetch		

#pragma unroll(4)
          for (unsigned int i=0; i<4; i++)
          {
            //const NodeRef child = node->children[i];
	    const NodeRef child = node->lower[i].child;

            if (unlikely(child == BVH4i::emptyNode)) break;
	    
            const mic_f lclipMinX = msub(node->lower[i].x,rdir.x,org_rdir.x);
            const mic_f lclipMinY = msub(node->lower[i].y,rdir.y,org_rdir.y);
            const mic_f lclipMinZ = msub(node->lower[i].z,rdir.z,org_rdir.z);
            const mic_f lclipMaxX = msub(node->upper[i].x,rdir.x,org_rdir.x);
            const mic_f lclipMaxY = msub(node->upper[i].y,rdir.y,org_rdir.y);
            const mic_f lclipMaxZ = msub(node->upper[i].z,rdir.z,org_rdir.z);
	    
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
              assert(sptr_node - stack_node < BVH4i::maxDepth);
            }	      
          }
        }
        
        /* return if stack is empty */
        if (unlikely(curNode == BVH4i::invalidNode)) break;
        
        /* intersect leaf */
        const mic_m valid_leaf = ray_tfar > curDist;
        STAT3(normal.trav_leaves,1,popcnt(valid_leaf),16);
        size_t items; const Triangle* tri  = (Triangle*) curNode.leaf(accel,items);
        TriangleIntersector16::intersect(valid_leaf,ray,tri,items,bvh->geometry);
        ray_tfar = select(valid_leaf,ray.tfar,ray_tfar);
      }
    }
    
    template<typename TriangleIntersector16>
    void BVH4iIntersector16Chunk<TriangleIntersector16>::occluded(mic_i* valid_i, BVH4i* bvh, Ray16& ray)
    {
      /* allocate stack */
      __align(64) mic_f    stack_dist[3*BVH4i::maxDepth+1];
      __align(64) NodeRef stack_node[3*BVH4i::maxDepth+1];

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
      const Triangle * __restrict__ accel = (Triangle*)bvh->triPtr();

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
	
	const mic_m m_active = !m_terminated;

        while (1)
        {
          /* test if this is a leaf node */
          if (unlikely(curNode.isLeaf())) break;
          
          STAT3(shadow.trav_nodes,1,popcnt(ray_tfar > curDist),16);
          const Node* __restrict__ const node = curNode.node(nodes);
          
          /* pop of next node */
          sptr_node--;
          sptr_dist--;
          curNode = *sptr_node; // FIXME: this trick creates issues with stack depth
          curDist = *sptr_dist;
          	 
#pragma unroll(4)
          for (unsigned int i=0; i<4; i++)
          {
            //const NodeRef child = node->children[i];
	    const NodeRef child = node->lower[i].child;

            if (unlikely(child == BVH4i::emptyNode)) break;
            
            const mic_f lclipMinX = msub(node->lower[i].x,rdir.x,org_rdir.x);
            const mic_f lclipMinY = msub(node->lower[i].y,rdir.y,org_rdir.y);
            const mic_f lclipMinZ = msub(node->lower[i].z,rdir.z,org_rdir.z);
            const mic_f lclipMaxX = msub(node->upper[i].x,rdir.x,org_rdir.x);
            const mic_f lclipMaxY = msub(node->upper[i].y,rdir.y,org_rdir.y);
            const mic_f lclipMaxZ = msub(node->upper[i].z,rdir.z,org_rdir.z);	    

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
              assert(sptr_node - stack_node < BVH4i::maxDepth);
            }	      
          }
        }
        
        /* return if stack is empty */
        if (unlikely(curNode == BVH4i::invalidNode)) 
          break;
        
        /* intersect leaf */
        const mic_m valid_leaf = gt(m_active,ray_tfar,curDist);
        STAT3(shadow.trav_leaves,1,popcnt(valid_leaf),16);
        size_t items; const Triangle* tri  = (Triangle*) curNode.leaf(accel,items);
        m_terminated |= valid_leaf & TriangleIntersector16::occluded(valid_leaf,ray,tri,items,bvh->geometry);
        if (unlikely(all(m_terminated))) break;
        ray_tfar = select(m_terminated,neg_inf,ray_tfar);
      }
      store16i(valid & m_terminated,&ray.geomID,0);
    }
    
    // FIXME: convert intersector16 to intersector8 and intersector4
    DEFINE_INTERSECTOR16    (BVH4iTriangle1Intersector16ChunkMoeller, BVH4iIntersector16Chunk<Triangle1Intersector16MoellerTrumbore>);
    DEFINE_INTERSECTOR16    (BVH4iVirtualIntersector16, BVH4iIntersector16Chunk<VirtualAccelIntersector16>);
  }
}
