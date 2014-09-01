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

#include "bvh4_intersector4_chunk.h"

#include "geometry/bezier1_intersector4.h"
#include "geometry/bezier1i_intersector4.h"
#include "geometry/triangle1_intersector4_moeller.h"
#include "geometry/triangle4_intersector4_moeller.h"
#if defined (__AVX__)
#include "geometry/triangle8_intersector4_moeller.h"
#endif
#include "geometry/triangle1v_intersector4_pluecker.h"
#include "geometry/triangle4v_intersector4_pluecker.h"
#include "geometry/triangle4i_intersector4.h"
#include "geometry/virtual_accel_intersector4.h"
#include "geometry/triangle1v_intersector4_moeller_mb.h"

namespace embree
{
  namespace isa
  {
    /* ray/box intersection */
    __forceinline sseb intersectBox(const Ray4& ray, const ssef& ray_tfar, const sse3f& rdir, const BVH4::NodeMB* node, const int i, ssef& dist) 
    {
      const ssef lower_x = ssef(node->lower_x[i]) + ray.time * ssef(node->lower_dx[i]);
      const ssef lower_y = ssef(node->lower_y[i]) + ray.time * ssef(node->lower_dy[i]);
      const ssef lower_z = ssef(node->lower_z[i]) + ray.time * ssef(node->lower_dz[i]);
      const ssef upper_x = ssef(node->upper_x[i]) + ray.time * ssef(node->upper_dx[i]);
      const ssef upper_y = ssef(node->upper_y[i]) + ray.time * ssef(node->upper_dy[i]);
      const ssef upper_z = ssef(node->upper_z[i]) + ray.time * ssef(node->upper_dz[i]);
      
      const ssef dminx = (lower_x - ray.org.x) * rdir.x;
      const ssef dminy = (lower_y - ray.org.y) * rdir.y;
      const ssef dminz = (lower_z - ray.org.z) * rdir.z;
      const ssef dmaxx = (upper_x - ray.org.x) * rdir.x;
      const ssef dmaxy = (upper_y - ray.org.y) * rdir.y;
      const ssef dmaxz = (upper_z - ray.org.z) * rdir.z;
      
      const ssef dlowerx = min(dminx,dmaxx);
      const ssef dlowery = min(dminy,dmaxy);
      const ssef dlowerz = min(dminz,dmaxz);
      
      const ssef dupperx = max(dminx,dmaxx);
      const ssef duppery = max(dminy,dmaxy);
      const ssef dupperz = max(dminz,dmaxz);
      
      const ssef near = max(dlowerx,dlowery,dlowerz,ray.tnear);
      const ssef far  = min(dupperx,duppery,dupperz,ray_tfar );
      dist = near;
      
      return near <= far;
    }
    

    template<int types, typename PrimitiveIntersector4>
    void BVH4Intersector4Chunk<types,PrimitiveIntersector4>::intersect(sseb* valid_i, BVH4* bvh, Ray4& ray)
    {
      /* load ray */
      const sseb valid0 = *valid_i;
      const sse3f rdir = rcp_safe(ray.dir);
      const sse3f org(ray.org), org_rdir = org * rdir;
      ssef ray_tnear = select(valid0,ray.tnear,ssef(pos_inf));
      ssef ray_tfar  = select(valid0,ray.tfar ,ssef(neg_inf));
      const ssef inf = ssef(pos_inf);
      Precalculations pre(valid0,ray);
      
      /* allocate stack and push root node */
      ssef    stack_near[stackSize];
      NodeRef stack_node[stackSize];
      stack_node[0] = BVH4::invalidNode;
      stack_near[0] = inf;
      stack_node[1] = bvh->root;
      stack_near[1] = ray_tnear; 
      NodeRef* stackEnd = stack_node+stackSize;
      NodeRef* __restrict__ sptr_node = stack_node + 2;
      ssef*    __restrict__ sptr_near = stack_near + 2;
      
      while (1)
      {
        /* pop next node from stack */
        assert(sptr_node > stack_node);
        sptr_node--;
        sptr_near--;
        NodeRef curNode = *sptr_node;
        if (unlikely(curNode == BVH4::invalidNode)) {
          assert(sptr_node == stack_node);
          break;
        }
        
        /* cull node if behind closest hit point */
        ssef curDist = *sptr_near;
        if (unlikely(none(ray_tfar > curDist))) 
          continue;
        
        while (1)
        {
	  /* process normal nodes */
          if (likely((types & 0x1) && curNode.isNode()))
          {
	    const sseb valid_node = ray_tfar > curDist;
	    STAT3(normal.trav_nodes,1,popcnt(valid_node),8);
	    const Node* __restrict__ const node = curNode.node();
	    
	    /* pop of next node */
	    assert(sptr_node > stack_node);
	    sptr_node--;
	    sptr_near--;
	    curNode = *sptr_node; 
	    curDist = *sptr_near;
          
#pragma unroll(4)
	    for (unsigned i=0; i<BVH4::N; i++)
	    {
	      const NodeRef child = node->children[i];
	      if (unlikely(child == BVH4::emptyNode)) break;
	      ssef lnearP; const sseb lhit = node->intersect(i,org,rdir,org_rdir,ray_tnear,ray_tfar,lnearP);
	      	      
	      /* if we hit the child we choose to continue with that child if it 
		 is closer than the current next child, or we push it onto the stack */
	      if (likely(any(lhit)))
	      {
		assert(sptr_node < stackEnd);
		const ssef childDist = select(lhit,lnearP,inf);
		const NodeRef child = node->children[i];
		assert(child != BVH4::emptyNode);
		sptr_node++;
		sptr_near++;
		
		/* push cur node onto stack and continue with hit child */
		if (any(childDist < curDist))
		{
		  *(sptr_node-1) = curNode;
		  *(sptr_near-1) = curDist; 
		  curDist = childDist;
		  curNode = child;
		}
		
		/* push hit child onto stack */
		else {
		  *(sptr_node-1) = child;
		  *(sptr_near-1) = childDist; 
		}
	      }	      
	    }
	  }
	  /* process motion blur nodes */
          else if (likely((types & 0x10) && curNode.isNodeMB()))
	  {
	    const sseb valid_node = ray_tfar > curDist;
	    STAT3(normal.trav_nodes,1,popcnt(valid_node),8);
	    const BVH4::NodeMB* __restrict__ const node = curNode.nodeMB();
          
	    /* pop of next node */
	    assert(sptr_node > stack_node);
	    sptr_node--;
	    sptr_near--;
	    curNode = *sptr_node; 
	    curDist = *sptr_near;
	    
#pragma unroll(4)
	    for (unsigned i=0; i<BVH4::N; i++)
	    {
	      const NodeRef child = node->child(i);
	      if (unlikely(child == BVH4::emptyNode)) break;
	      ssef lnearP; const sseb lhit = node->intersect(i,org,rdir,org_rdir,ray_tnear,ray_tfar,ray.time,lnearP);
	      
	      /* if we hit the child we choose to continue with that child if it 
		 is closer than the current next child, or we push it onto the stack */
	      if (likely(any(lhit)))
	      {
		assert(sptr_node < stackEnd);
		assert(child != BVH4::emptyNode);
		const ssef childDist = select(lhit,lnearP,inf);
		sptr_node++;
		sptr_near++;
		
		/* push cur node onto stack and continue with hit child */
		if (any(childDist < curDist))
		{
		  *(sptr_node-1) = curNode;
		  *(sptr_near-1) = curDist; 
		  curDist = childDist;
		  curNode = child;
		}
		
		/* push hit child onto stack */
		else {
		  *(sptr_node-1) = child;
		  *(sptr_near-1) = childDist; 
		}
	      }	      
	    }
	  }
	  else 
	    break;
        }
        
        /* return if stack is empty */
        if (unlikely(curNode == BVH4::invalidNode)) {
          assert(sptr_node == stack_node);
          break;
        }
        
        /* intersect leaf */
        const sseb valid_leaf = ray_tfar > curDist;
        STAT3(normal.trav_leaves,1,popcnt(valid_leaf),4);
        size_t items; const Primitive* prim = (Primitive*) curNode.leaf(items);
        PrimitiveIntersector4::intersect(valid_leaf,pre,ray,prim,items,bvh->geometry);
        ray_tfar = select(valid_leaf,ray.tfar,ray_tfar);
      }
      AVX_ZERO_UPPER();
    }
    
    template<int types, typename PrimitiveIntersector4>
    void BVH4Intersector4Chunk<types,PrimitiveIntersector4>::occluded(sseb* valid_i, BVH4* bvh, Ray4& ray)
    {
      /* load ray */
      const sseb valid = *valid_i;
      sseb terminated = !valid;
      const sse3f rdir = rcp_safe(ray.dir);
      const sse3f org(ray.org), org_rdir = org * rdir;
      ssef ray_tnear = select(valid,ray.tnear,ssef(pos_inf));
      ssef ray_tfar  = select(valid,ray.tfar ,ssef(neg_inf));
      const ssef inf = ssef(pos_inf);
      Precalculations pre(valid,ray);

      /* allocate stack and push root node */
      ssef    stack_near[stackSize];
      NodeRef stack_node[stackSize];
      stack_node[0] = BVH4::invalidNode;
      stack_near[0] = inf;
      stack_node[1] = bvh->root;
      stack_near[1] = ray_tnear; 
      NodeRef* stackEnd = stack_node+stackSize;
      NodeRef* __restrict__ sptr_node = stack_node + 2;
      ssef*    __restrict__ sptr_near = stack_near + 2;
      
      while (1)
      {
        /* pop next node from stack */
        assert(sptr_node > stack_node);
        sptr_node--;
        sptr_near--;
        NodeRef curNode = *sptr_node;
        if (unlikely(curNode == BVH4::invalidNode)) {
          assert(sptr_node == stack_node);
          break;
        }
        
        /* cull node if behind closest hit point */
        ssef curDist = *sptr_near;
        if (unlikely(none(ray_tfar > curDist))) 
          continue;
        
        while (1)
        {
	  /* process normal nodes */
          if (likely((types & 0x1) && curNode.isNode()))
          {
	    const sseb valid_node = ray_tfar > curDist;
	    STAT3(normal.trav_nodes,1,popcnt(valid_node),8);
	    const Node* __restrict__ const node = curNode.node();
	    
	    /* pop of next node */
	    assert(sptr_node > stack_node);
	    sptr_node--;
	    sptr_near--;
	    curNode = *sptr_node; 
	    curDist = *sptr_near;
          
#pragma unroll(4)
	    for (unsigned i=0; i<BVH4::N; i++)
	    {
	      const NodeRef child = node->children[i];
	      if (unlikely(child == BVH4::emptyNode)) break;
	      ssef lnearP; const sseb lhit = node->intersect(i,org,rdir,org_rdir,ray_tnear,ray_tfar,lnearP);
	      
	      /* if we hit the child we choose to continue with that child if it 
		 is closer than the current next child, or we push it onto the stack */
	      if (likely(any(lhit)))
	      {
		assert(sptr_node < stackEnd);
		const ssef childDist = select(lhit,lnearP,inf);
		const NodeRef child = node->children[i];
		assert(child != BVH4::emptyNode);
		sptr_node++;
		sptr_near++;
		
		/* push cur node onto stack and continue with hit child */
		if (any(childDist < curDist))
		{
		  *(sptr_node-1) = curNode;
		  *(sptr_near-1) = curDist; 
		  curDist = childDist;
		  curNode = child;
		}
		
		/* push hit child onto stack */
		else {
		  *(sptr_node-1) = child;
		  *(sptr_near-1) = childDist; 
		}
	      }	      
	    }
	  }
	  /* process motion blur nodes */
          else if (likely((types & 0x10) && curNode.isNodeMB()))
	  {
	    const sseb valid_node = ray_tfar > curDist;
	    STAT3(normal.trav_nodes,1,popcnt(valid_node),8);
	    const BVH4::NodeMB* __restrict__ const node = curNode.nodeMB();
          
	    /* pop of next node */
	    assert(sptr_node > stack_node);
	    sptr_node--;
	    sptr_near--;
	    curNode = *sptr_node; 
	    curDist = *sptr_near;
	    
#pragma unroll(4)
	    for (unsigned i=0; i<BVH4::N; i++)
	    {
	      const NodeRef child = node->child(i);
	      if (unlikely(child == BVH4::emptyNode)) break;
	      ssef lnearP; const sseb lhit = node->intersect(i,org,rdir,org_rdir,ray_tnear,ray_tfar,ray.time,lnearP);
	      
	      /* if we hit the child we choose to continue with that child if it 
		 is closer than the current next child, or we push it onto the stack */
	      if (likely(any(lhit)))
	      {
		assert(sptr_node < stackEnd);
		assert(child != BVH4::emptyNode);
		const ssef childDist = select(lhit,lnearP,inf);
		sptr_node++;
		sptr_near++;
		
		/* push cur node onto stack and continue with hit child */
		if (any(childDist < curDist))
		{
		  *(sptr_node-1) = curNode;
		  *(sptr_near-1) = curDist; 
		  curDist = childDist;
		  curNode = child;
		}
		
		/* push hit child onto stack */
		else {
		  *(sptr_node-1) = child;
		  *(sptr_near-1) = childDist; 
		}
	      }	      
	    }
	  }
	  else 
	    break;
        }
        
        /* return if stack is empty */
        if (unlikely(curNode == BVH4::invalidNode)) {
          assert(sptr_node == stack_node);
          break;
        }
        
        /* intersect leaf */
        const sseb valid_leaf = ray_tfar > curDist;
        STAT3(shadow.trav_leaves,1,popcnt(valid_leaf),4);
        size_t items; const Primitive* prim = (Primitive*) curNode.leaf(items);
        terminated |= PrimitiveIntersector4::occluded(!terminated,pre,ray,prim,items,bvh->geometry);
        if (all(terminated)) break;
        ray_tfar = select(terminated,ssef(neg_inf),ray_tfar);
      }
      store4i(valid & terminated,&ray.geomID,0);
      AVX_ZERO_UPPER();
    }
    
    DEFINE_INTERSECTOR4(BVH4Bezier1Intersector4Chunk, BVH4Intersector4Chunk<0x1 COMMA Bezier1Intersector4>);
    DEFINE_INTERSECTOR4(BVH4Bezier1iIntersector4Chunk, BVH4Intersector4Chunk<0x1 COMMA Bezier1iIntersector4>);
    DEFINE_INTERSECTOR4(BVH4Triangle1Intersector4ChunkMoeller, BVH4Intersector4Chunk<0x1 COMMA Triangle1Intersector4MoellerTrumbore>);
    DEFINE_INTERSECTOR4(BVH4Triangle4Intersector4ChunkMoeller, BVH4Intersector4Chunk<0x1 COMMA Triangle4Intersector4MoellerTrumbore<true> >);
    DEFINE_INTERSECTOR4(BVH4Triangle4Intersector4ChunkMoellerNoFilter, BVH4Intersector4Chunk<0x1 COMMA Triangle4Intersector4MoellerTrumbore<false> >);
#if defined (__AVX__)
    DEFINE_INTERSECTOR4(BVH4Triangle8Intersector4ChunkMoeller, BVH4Intersector4Chunk<0x1 COMMA Triangle8Intersector4MoellerTrumbore<true> >);
    DEFINE_INTERSECTOR4(BVH4Triangle8Intersector4ChunkMoellerNoFilter, BVH4Intersector4Chunk<0x1 COMMA Triangle8Intersector4MoellerTrumbore<false> >);
#endif
    DEFINE_INTERSECTOR4(BVH4Triangle1vIntersector4ChunkPluecker, BVH4Intersector4Chunk<0x1 COMMA Triangle1vIntersector4Pluecker>);
    DEFINE_INTERSECTOR4(BVH4Triangle4vIntersector4ChunkPluecker, BVH4Intersector4Chunk<0x1 COMMA Triangle4vIntersector4Pluecker>);
    DEFINE_INTERSECTOR4(BVH4Triangle4iIntersector4ChunkPluecker, BVH4Intersector4Chunk<0x1 COMMA Triangle4iIntersector4Pluecker>);
    DEFINE_INTERSECTOR4(BVH4VirtualIntersector4Chunk, BVH4Intersector4Chunk<0x1 COMMA VirtualAccelIntersector4>);
    DEFINE_INTERSECTOR4(BVH4Triangle1vMBIntersector4ChunkMoeller, BVH4Intersector4Chunk<0x10 COMMA Triangle1vIntersector4MoellerTrumboreMB>);
  }
}
