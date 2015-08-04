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

#include "bvh4_intersector16_hybrid.h"
#include "bvh4_intersector16_single.h"
#include "bvh4_intersector_node.h"

#include "../geometry/triangle4.h"
#include "../geometry/triangle4v.h"
#include "../geometry/triangle8.h"
#include "../geometry/intersector_iterators.h"
#include "../geometry/triangle_intersector_moeller.h"
#include "../geometry/triangle_intersector_pluecker.h"

#define SWITCH_THRESHOLD 7
#define SWITCH_DURING_DOWN_TRAVERSAL 1

namespace embree
{
  namespace isa
  {
    template<int types, bool robust, typename PrimitiveIntersector16>
    void BVH4Intersector16Hybrid<types,robust,PrimitiveIntersector16>::intersect(int16* valid_i, BVH4* bvh, Ray16& ray)
    {
      /* verify correct input */
      bool16 valid0 = *valid_i == -1;
#if defined(RTCORE_IGNORE_INVALID_RAYS)
      valid0 &= ray.valid();
#endif
      assert(all(valid0,ray.tnear > -FLT_MIN));
      assert(!(types & BVH4::FLAG_NODE_MB) || all(valid0,ray.time >= 0.0f & ray.time <= 1.0f));
      
      /* load ray */
      Vec3f16 ray_org = ray.org;
      Vec3f16 ray_dir = ray.dir;
      float16 ray_tnear = ray.tnear, ray_tfar  = ray.tfar;
      const Vec3f16 rdir = rcp_safe(ray_dir);
      const Vec3f16 org(ray_org), org_rdir = org * rdir;
      ray_tnear = select(valid0,ray_tnear,float16(pos_inf));
      ray_tfar  = select(valid0,ray_tfar ,float16(neg_inf));
      const float16 inf = float16(pos_inf);
      Precalculations pre(valid0,ray);

      /* compute near/far per ray */
      Vec3i16 nearXYZ;
      nearXYZ.x = select(rdir.x >= 0.0f,int16(0*(int)sizeof(float4)),int16(1*(int)sizeof(float4)));
      nearXYZ.y = select(rdir.y >= 0.0f,int16(2*(int)sizeof(float4)),int16(3*(int)sizeof(float4)));
      nearXYZ.z = select(rdir.z >= 0.0f,int16(4*(int)sizeof(float4)),int16(5*(int)sizeof(float4)));

      /* allocate stack and push root node */
      float16    stack_near[stackSizeChunk];
      NodeRef stack_node[stackSizeChunk];
      stack_node[0] = BVH4::invalidNode;
      stack_near[0] = inf;
      stack_node[1] = bvh->root;
      stack_near[1] = ray_tnear; 
      NodeRef* stackEnd = stack_node+stackSizeChunk;
      NodeRef* __restrict__ sptr_node = stack_node + 2;
      float16*    __restrict__ sptr_near = stack_near + 2;
      
      while (1) pop:
      {
        /* pop next node from stack */
        assert(sptr_node > stack_node);
        sptr_node--;
        sptr_near--;
        NodeRef cur = *sptr_node;
        if (unlikely(cur == BVH4::invalidNode)) {
          assert(sptr_node == stack_node);
          break;
        }
        
        /* cull node if behind closest hit point */
        float16 curDist = *sptr_near;
        const bool16 active = curDist < ray_tfar;
        if (unlikely(none(active)))
          continue;
        
        /* switch to single ray traversal */
#if !defined(__WIN32__) || defined(__X86_64__)
        size_t bits = movemask(active);
        if (unlikely(__popcnt(bits) <= SWITCH_THRESHOLD)) {
          for (size_t i=__bsf(bits); bits!=0; bits=__btc(bits,i), i=__bsf(bits)) {
            BVH4Intersector16Single<types,robust,PrimitiveIntersector16>::intersect1(bvh, cur, i, pre, ray, ray_org, ray_dir, rdir, ray_tnear, ray_tfar, nearXYZ);
          }
          ray_tfar = min(ray_tfar,ray.tfar);
          continue;
        }
#endif

        while (1)
        {
	  /* process normal nodes */
          if (likely((types & 0x1) && cur.isNode()))
          {
	    const bool16 valid_node = ray_tfar > curDist;
	    STAT3(normal.trav_nodes,1,popcnt(valid_node),16);
	    const Node* __restrict__ const node = cur.node();
	    
	    /* pop of next node */
	    assert(sptr_node > stack_node);
	    sptr_node--;
	    sptr_near--;
	    cur = *sptr_node; 
	    curDist = *sptr_near;
	    
#pragma unroll(4)
	    for (unsigned i=0; i<BVH4::N; i++)
	    {
	      const NodeRef child = node->children[i];
	      if (unlikely(child == BVH4::emptyNode)) break;
	      float16 lnearP; const bool16 lhit = intersect16_node<robust>(node,i,org,rdir,org_rdir,ray_tnear,ray_tfar,lnearP);
	      	      
	      /* if we hit the child we choose to continue with that child if it 
		 is closer than the current next child, or we push it onto the stack */
	      if (likely(any(lhit)))
	      {
		assert(sptr_node < stackEnd);
		assert(child != BVH4::emptyNode);
		const float16 childDist = select(lhit,lnearP,inf);
		sptr_node++;
		sptr_near++;
		
		/* push cur node onto stack and continue with hit child */
		if (any(childDist < curDist))
		{
		  *(sptr_node-1) = cur;
		  *(sptr_near-1) = curDist; 
		  curDist = childDist;
		  cur = child;
		}
		
		/* push hit child onto stack */
		else {
		  *(sptr_node-1) = child;
		  *(sptr_near-1) = childDist; 
		}
	      }     
	    }
#if SWITCH_DURING_DOWN_TRAVERSAL == 1
          // seems to be the best place for testing utilization
          if (unlikely(popcnt(ray_tfar > curDist) <= SWITCH_THRESHOLD))
            {
              *sptr_node++ = cur;
              *sptr_near++ = curDist;
              goto pop;
            }
#endif
	  }
	  
	  /* process motion blur nodes */
          else if (likely((types & 0x10) && cur.isNodeMB()))
	  {
	    const bool16 valid_node = ray_tfar > curDist;
	    STAT3(normal.trav_nodes,1,popcnt(valid_node),16);
	    const BVH4::NodeMB* __restrict__ const node = cur.nodeMB();
          
	    /* pop of next node */
	    assert(sptr_node > stack_node);
	    sptr_node--;
	    sptr_near--;
	    cur = *sptr_node; 
	    curDist = *sptr_near;
	    
#pragma unroll(4)
	    for (unsigned i=0; i<BVH4::N; i++)
	    {
	      const NodeRef child = node->child(i);
	      if (unlikely(child == BVH4::emptyNode)) break;
	      float16 lnearP; const bool16 lhit = intersect_node(node,i,org,rdir,org_rdir,ray_tnear,ray_tfar,ray.time,lnearP);
	      	      
	      /* if we hit the child we choose to continue with that child if it 
		 is closer than the current next child, or we push it onto the stack */
	      if (likely(any(lhit)))
	      {
		assert(sptr_node < stackEnd);
		assert(child != BVH4::emptyNode);
		const float16 childDist = select(lhit,lnearP,inf);
		sptr_node++;
		sptr_near++;
		
		/* push cur node onto stack and continue with hit child */
		if (any(childDist < curDist))
		{
		  *(sptr_node-1) = cur;
		  *(sptr_near-1) = curDist; 
		  curDist = childDist;
		  cur = child;
		}
		
		/* push hit child onto stack */
		else {
		  *(sptr_node-1) = child;
		  *(sptr_near-1) = childDist; 
		}
	      }	      
	    }
#if SWITCH_DURING_DOWN_TRAVERSAL == 1
          // seems to be the best place for testing utilization
          if (unlikely(popcnt(ray_tfar > curDist) <= SWITCH_THRESHOLD))
            {
              *sptr_node++ = cur;
              *sptr_near++ = curDist;
              goto pop;
            }
#endif
	  }
	  else 
	    break;
	}
        
        /* return if stack is empty */
        if (unlikely(cur == BVH4::invalidNode)) {
          assert(sptr_node == stack_node);
          break;
        }
        
        /* intersect leaf */
	assert(cur != BVH4::emptyNode);
        const bool16 valid_leaf = ray_tfar > curDist;
        STAT3(normal.trav_leaves,1,popcnt(valid_leaf),16);
        size_t items; const Primitive* prim = (Primitive*) cur.leaf(items);

        size_t lazy_node = 0;
        PrimitiveIntersector16::intersect(valid_leaf,pre,ray,prim,items,bvh->scene,lazy_node);
        ray_tfar = select(valid_leaf,ray.tfar,ray_tfar);

        if (unlikely(lazy_node)) {
          *sptr_node = lazy_node; sptr_node++;
          *sptr_near = neg_inf;   sptr_near++;
        }
      }
      AVX_ZERO_UPPER();
    }

    
    template<int types, bool robust, typename PrimitiveIntersector16>
    void BVH4Intersector16Hybrid<types,robust,PrimitiveIntersector16>::occluded(int16* valid_i, BVH4* bvh, Ray16& ray)
    {
      /* verify correct input */
      bool16 valid = *valid_i == -1;
#if defined(RTCORE_IGNORE_INVALID_RAYS)
      valid &= ray.valid();
#endif
      assert(all(valid,ray.tnear > -FLT_MIN));
      assert(!(types & BVH4::FLAG_NODE_MB) || all(valid,ray.time >= 0.0f & ray.time <= 1.0f));

      /* load ray */
      bool16 terminated = !valid;
      Vec3f16 ray_org = ray.org, ray_dir = ray.dir;
      float16 ray_tnear = ray.tnear, ray_tfar  = ray.tfar;
      const Vec3f16 rdir = rcp_safe(ray_dir);
      const Vec3f16 org(ray_org), org_rdir = org * rdir;
      ray_tnear = select(valid,ray_tnear,float16(pos_inf));
      ray_tfar  = select(valid,ray_tfar ,float16(neg_inf));
      const float16 inf = float16(pos_inf);
      Precalculations pre(valid,ray);

      /* compute near/far per ray */
      Vec3i16 nearXYZ;
      nearXYZ.x = select(rdir.x >= 0.0f,int16(0*(int)sizeof(float4)),int16(1*(int)sizeof(float4)));
      nearXYZ.y = select(rdir.y >= 0.0f,int16(2*(int)sizeof(float4)),int16(3*(int)sizeof(float4)));
      nearXYZ.z = select(rdir.z >= 0.0f,int16(4*(int)sizeof(float4)),int16(5*(int)sizeof(float4)));

      /* allocate stack and push root node */
      float16    stack_near[stackSizeChunk];
      NodeRef stack_node[stackSizeChunk];
      stack_node[0] = BVH4::invalidNode;
      stack_near[0] = inf;
      stack_node[1] = bvh->root;
      stack_near[1] = ray_tnear; 
      NodeRef* stackEnd = stack_node+stackSizeChunk;
      NodeRef* __restrict__ sptr_node = stack_node + 2;
      float16*    __restrict__ sptr_near = stack_near + 2;
      
      while (1) pop:
      {
        /* pop next node from stack */
        assert(sptr_node > stack_node);
        sptr_node--;
        sptr_near--;
        NodeRef cur = *sptr_node;
        if (unlikely(cur == BVH4::invalidNode)) {
          assert(sptr_node == stack_node);
          break;
        }

        /* cull node if behind closest hit point */
        float16 curDist = *sptr_near;
        const bool16 active = curDist < ray_tfar;
        if (unlikely(none(active))) 
          continue;
        
        /* switch to single ray traversal */
#if !defined(__WIN32__) || defined(__X86_64__)
        size_t bits = movemask(active);
        if (unlikely(__popcnt(bits) <= SWITCH_THRESHOLD)) {
          for (size_t i=__bsf(bits); bits!=0; bits=__btc(bits,i), i=__bsf(bits)) {
            if (BVH4Intersector16Single<types,robust,PrimitiveIntersector16>::occluded1(bvh,cur,i,pre,ray,ray_org,ray_dir,rdir,ray_tnear,ray_tfar,nearXYZ))
              terminated |= 1 << i;
              //terminated[i] = -1;
          }
          if (all(terminated)) break;
          ray_tfar = select(terminated,float16(neg_inf),ray_tfar);
          continue;
        }
#endif
                
        while (1)
        {
	  /* process normal nodes */
          if (likely((types & 0x1) && cur.isNode()))
          {
	    const bool16 valid_node = ray_tfar > curDist;
	    STAT3(normal.trav_nodes,1,popcnt(valid_node),16);
	    const Node* __restrict__ const node = cur.node();
	    
	    /* pop of next node */
	    assert(sptr_node > stack_node);
	    sptr_node--;
	    sptr_near--;
	    cur = *sptr_node; 
	    curDist = *sptr_near;
	    
#pragma unroll(4)
	    for (unsigned i=0; i<BVH4::N; i++)
	    {
	      const NodeRef child = node->children[i];
	      if (unlikely(child == BVH4::emptyNode)) break;
	      float16 lnearP; const bool16 lhit = intersect16_node<robust>(node,i,org,rdir,org_rdir,ray_tnear,ray_tfar,lnearP);
	      	      
	      /* if we hit the child we choose to continue with that child if it 
		 is closer than the current next child, or we push it onto the stack */
	      if (likely(any(lhit)))
	      {
		assert(sptr_node < stackEnd);
		assert(child != BVH4::emptyNode);
		const float16 childDist = select(lhit,lnearP,inf);
		sptr_node++;
		sptr_near++;
		
		/* push cur node onto stack and continue with hit child */
		if (any(childDist < curDist))
		{
		  *(sptr_node-1) = cur;
		  *(sptr_near-1) = curDist; 
		  curDist = childDist;
		  cur = child;
		}
		
		/* push hit child onto stack */
		else {
		  *(sptr_node-1) = child;
		  *(sptr_near-1) = childDist; 
		}
	      }     
	    }
#if SWITCH_DURING_DOWN_TRAVERSAL == 1
          // seems to be the best place for testing utilization
          if (unlikely(popcnt(ray_tfar > curDist) <= SWITCH_THRESHOLD))
            {
              *sptr_node++ = cur;
              *sptr_near++ = curDist;
              goto pop;
            }
#endif
	  }
	  
	  /* process motion blur nodes */
          else if (likely((types & 0x10) && cur.isNodeMB()))
	  {
	    const bool16 valid_node = ray_tfar > curDist;
	    STAT3(normal.trav_nodes,1,popcnt(valid_node),16);
	    const BVH4::NodeMB* __restrict__ const node = cur.nodeMB();
          
	    /* pop of next node */
	    assert(sptr_node > stack_node);
	    sptr_node--;
	    sptr_near--;
	    cur = *sptr_node; 
	    curDist = *sptr_near;
	    
#pragma unroll(4)
	    for (unsigned i=0; i<BVH4::N; i++)
	    {
	      const NodeRef child = node->child(i);
	      if (unlikely(child == BVH4::emptyNode)) break;
	      float16 lnearP; const bool16 lhit = intersect_node(node,i,org,rdir,org_rdir,ray_tnear,ray_tfar,ray.time,lnearP);
	      
	      /* if we hit the child we choose to continue with that child if it 
		 is closer than the current next child, or we push it onto the stack */
	      if (likely(any(lhit)))
	      {
		assert(sptr_node < stackEnd);
		assert(child != BVH4::emptyNode);
		const float16 childDist = select(lhit,lnearP,inf);
		sptr_node++;
		sptr_near++;
		
		/* push cur node onto stack and continue with hit child */
		if (any(childDist < curDist))
		{
		  *(sptr_node-1) = cur;
		  *(sptr_near-1) = curDist; 
		  curDist = childDist;
		  cur = child;
		}
		
		/* push hit child onto stack */
		else {
		  *(sptr_node-1) = child;
		  *(sptr_near-1) = childDist; 
		}
	      }	      
	    }
#if SWITCH_DURING_DOWN_TRAVERSAL == 1
          // seems to be the best place for testing utilization
          if (unlikely(popcnt(ray_tfar > curDist) <= SWITCH_THRESHOLD))
            {
              *sptr_node++ = cur;
              *sptr_near++ = curDist;
              goto pop;
            }
#endif
	  }
	  else 
	    break;
	}
        
        /* return if stack is empty */
        if (unlikely(cur == BVH4::invalidNode)) {
          assert(sptr_node == stack_node);
          break;
        }

        
        /* intersect leaf */
	assert(cur != BVH4::emptyNode);
        const bool16 valid_leaf = ray_tfar > curDist;
        STAT3(shadow.trav_leaves,1,popcnt(valid_leaf),16);
        size_t items; const Primitive* prim = (Primitive*) cur.leaf(items);

        size_t lazy_node = 0;
        terminated |= PrimitiveIntersector16::occluded(!terminated,pre,ray,prim,items,bvh->scene,lazy_node);
        if (all(terminated)) break;
        ray_tfar = select(terminated,float16(neg_inf),ray_tfar);

        if (unlikely(lazy_node)) {
          *sptr_node = lazy_node; sptr_node++;
          *sptr_near = neg_inf;   sptr_near++;
        }
      }
      store16i(valid & terminated,&ray.geomID,0);
      AVX_ZERO_UPPER();
    }
    
    DEFINE_INTERSECTOR16(BVH4Triangle4Intersector16HybridMoeller, BVH4Intersector16Hybrid<0x1 COMMA false COMMA ArrayIntersector16_1<TriangleNIntersectorMMoellerTrumbore<Ray16 COMMA Triangle4 COMMA true> > >);
    DEFINE_INTERSECTOR16(BVH4Triangle4Intersector16HybridMoellerNoFilter, BVH4Intersector16Hybrid<0x1 COMMA false COMMA ArrayIntersector16_1<TriangleNIntersectorMMoellerTrumbore<Ray16 COMMA Triangle4 COMMA false> > >);
    DEFINE_INTERSECTOR16(BVH4Triangle8Intersector16HybridMoeller, BVH4Intersector16Hybrid<0x1 COMMA false COMMA ArrayIntersector16_1<TriangleNIntersectorMMoellerTrumbore<Ray16 COMMA Triangle8 COMMA true> > >);
    DEFINE_INTERSECTOR16(BVH4Triangle8Intersector16HybridMoellerNoFilter, BVH4Intersector16Hybrid<0x1 COMMA false COMMA ArrayIntersector16_1<TriangleNIntersectorMMoellerTrumbore<Ray16 COMMA Triangle8 COMMA false> > >);
    DEFINE_INTERSECTOR16(BVH4Triangle4vIntersector16HybridPluecker, BVH4Intersector16Hybrid<0x1 COMMA true COMMA ArrayIntersector16_1<TriangleNvIntersectorMPluecker<Ray16 COMMA Triangle4v COMMA true> > >);

    // FIXME: add Triangle4vMB intersector
    // FIXME: add Triangle4i intersector
  }
}
