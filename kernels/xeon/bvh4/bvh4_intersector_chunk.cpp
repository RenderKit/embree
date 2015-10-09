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

#include "bvh4_intersector_chunk.h"
#include "bvh4_intersector_node.h"

#include "../geometry/triangle.h"
#include "../geometry/trianglei.h"
#include "../geometry/trianglev.h"
#include "../geometry/trianglev_mb.h"
#include "../geometry/trianglepairsv.h"
#include "../geometry/intersector_iterators.h"
#include "../geometry/bezier1v_intersector.h"
#include "../geometry/bezier1i_intersector.h"
#include "../geometry/triangle_intersector_moeller.h"
#include "../geometry/triangle_intersector_pluecker.h"
#include "../geometry/triangle4i_intersector_pluecker.h"
#include "../geometry/object_intersector.h"
#include "../geometry/trianglepairs_intersector_moeller.h"
#include "../geometry/subdivpatch1cached_intersector1.h"

namespace embree
{
  namespace isa
  {
    template<int K, int types, bool robust, typename PrimitiveIntersectorK>
    void BVH4IntersectorKChunk<K, types, robust, PrimitiveIntersectorK>::intersect(vint<K>* __restrict__ valid_i, BVH4* __restrict__ bvh, RayK<K>& __restrict__ ray)
    {
      /* verify correct input */
      vbool<K> valid0 = *valid_i == -1;
#if defined(RTCORE_IGNORE_INVALID_RAYS)
      valid0 &= ray.valid();
#endif
      assert(all(valid0,ray.tnear > -FLT_MIN));
      assert(!(types & BVH4::FLAG_NODE_MB) || all(valid0,ray.time >= 0.0f & ray.time <= 1.0f));
      /* load ray */
      const Vec3vfK rdir = rcp_safe(ray.dir);
      const Vec3vfK org(ray.org), org_rdir = org * rdir;
      vfloat<K> ray_tnear = select(valid0,ray.tnear,pos_inf);
      vfloat<K> ray_tfar  = select(valid0,ray.tfar ,neg_inf);
      const vfloat<K> inf = vfloat<K>(pos_inf);
      Precalculations pre(valid0,ray);

      /* allocate stack and push root node */
      vfloat<K> stack_near[stackSize];
      NodeRef stack_node[stackSize];
      stack_node[0] = BVH4::invalidNode;
      stack_near[0] = inf;
      stack_node[1] = bvh->root;
      stack_near[1] = ray_tnear; 
      NodeRef* stackEnd = stack_node+stackSize;
      NodeRef* __restrict__ sptr_node = stack_node + 2;
      vfloat<K>* __restrict__ sptr_near = stack_near + 2;
      
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
        vfloat<K> curDist = *sptr_near;
        if (unlikely(none(ray_tfar > curDist))) 
          continue;
        
        while (1) 
        {
          /* process normal nodes */
          if (likely((types & 0x1) && cur.isNode()))
          {
            const vbool<K> valid_node = ray_tfar > curDist;
            STAT3(normal.trav_nodes,1,popcnt(valid_node),K);
	    const Node* __restrict__ const node = cur.node();

            /* set cur to invalid */
            cur = BVH4::emptyNode;
            curDist = pos_inf;
	    
#pragma unroll(4)
	    for (unsigned i=0; i<BVH4::N; i++)
	    {
	      const NodeRef child = node->children[i];
	      if (unlikely(child == BVH4::emptyNode)) break;
              vfloat<K> lnearP; const vbool<K> lhit = intersect_node<K,robust>(node,i,org,rdir,org_rdir,ray_tnear,ray_tfar,lnearP);
	      
	      /* if we hit the child we choose to continue with that child if it 
		 is closer than the current next child, or we push it onto the stack */
	      if (likely(any(lhit)))
	      {
		assert(sptr_node < stackEnd);
		assert(child != BVH4::emptyNode);
                const vfloat<K> childDist = select(lhit,lnearP,inf);
		
                /* push cur node onto stack and continue with hit child */
		if (any(childDist < curDist))
		{
                  if (likely(cur != BVH4::emptyNode)) {
                    *sptr_node = cur; sptr_node++;
                    *sptr_near = curDist; sptr_near++;
                  }
		  curDist = childDist;
		  cur = child;
		}
		
		/* push hit child onto stack */
		else {
		  *sptr_node = child; sptr_node++;
		  *sptr_near = childDist; sptr_near++;
		}
	      }     
	    }
            if (unlikely(cur == BVH4::emptyNode)) 
              goto pop;
	  }
	  
	  /* process motion blur nodes */
          else if (likely((types & 0x10) && cur.isNodeMB()))
	  {
            const vbool<K> valid_node = ray_tfar > curDist;
            STAT3(normal.trav_nodes,1,popcnt(valid_node),K);
	    const BVH4::NodeMB* __restrict__ const node = cur.nodeMB();
          
             /* set cur to invalid */
            cur = BVH4::emptyNode;
            curDist = pos_inf;
	    
#pragma unroll(4)
	    for (unsigned i=0; i<BVH4::N; i++)
	    {
	      const NodeRef child = node->child(i);
	      if (unlikely(child == BVH4::emptyNode)) break;
              vfloat<K> lnearP; const vbool<K> lhit = intersect_node<K>(node,i,org,rdir,org_rdir,ray_tnear,ray_tfar,ray.time,lnearP);
	      	      
	      /* if we hit the child we choose to continue with that child if it 
		 is closer than the current next child, or we push it onto the stack */
	      if (likely(any(lhit)))
	      {
		assert(sptr_node < stackEnd);
		assert(child != BVH4::emptyNode);
                const vfloat<K> childDist = select(lhit,lnearP,inf);
		
                /* push cur node onto stack and continue with hit child */
		if (any(childDist < curDist))
		{
                  if (likely(cur != BVH4::emptyNode)) {
                    *sptr_node = cur; sptr_node++;
                    *sptr_near = curDist; sptr_near++;
                  }
		  curDist = childDist;
		  cur = child;
		}
		
		/* push hit child onto stack */
		else {
		  *sptr_node = child; sptr_node++;
		  *sptr_near = childDist; sptr_near++;
		}
	      }	      
	    }
            if (unlikely(cur == BVH4::emptyNode)) 
              goto pop;
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
        const vbool<K> valid_leaf = ray_tfar > curDist;
        STAT3(normal.trav_leaves,1,popcnt(valid_leaf),K);
	size_t items; const Primitive* prim = (Primitive*) cur.leaf(items);

        size_t lazy_node = 0;
        PrimitiveIntersectorK::intersect(valid_leaf,pre,ray,prim,items,bvh->scene,lazy_node);
	ray_tfar = select(valid_leaf,ray.tfar,ray_tfar);

        if (unlikely(lazy_node)) {
          *sptr_node = lazy_node; sptr_node++;
          *sptr_near = neg_inf;   sptr_near++;
        }
      }
      AVX_ZERO_UPPER();
    }
    
    template<int K, int types, bool robust, typename PrimitiveIntersectorK>
    void BVH4IntersectorKChunk<K, types, robust, PrimitiveIntersectorK>::occluded(vint<K>* __restrict__ valid_i, BVH4* __restrict__ bvh, RayK<K>& __restrict__ ray)
    {
      /* verify correct input */
      vbool<K> valid = *valid_i == -1;
#if defined(RTCORE_IGNORE_INVALID_RAYS)
      valid &= ray.valid();
#endif
      assert(all(valid,ray.tnear > -FLT_MIN));
      assert(!(types & BVH4::FLAG_NODE_MB) || all(valid,ray.time >= 0.0f & ray.time <= 1.0f));

      /* load ray */
      vbool<K> terminated = !valid;
      const Vec3vfK rdir = rcp_safe(ray.dir);
      const Vec3vfK org(ray.org), org_rdir = org * rdir;
      vfloat<K> ray_tnear = select(valid,ray.tnear,pos_inf);
      vfloat<K> ray_tfar  = select(valid,ray.tfar ,neg_inf);
      const vfloat<K> inf = vfloat<K>(pos_inf);
      Precalculations pre(valid,ray);

      /* allocate stack and push root node */
      vfloat<K> stack_near[stackSize];
      NodeRef stack_node[stackSize];
      stack_node[0] = BVH4::invalidNode;
      stack_near[0] = inf;
      stack_node[1] = bvh->root;
      stack_near[1] = ray_tnear; 
      NodeRef* stackEnd = stack_node+stackSize;
      NodeRef* __restrict__ sptr_node = stack_node + 2;
      vfloat<K>* __restrict__ sptr_near = stack_near + 2;
      
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
        vfloat<K> curDist = *sptr_near;
        if (unlikely(none(ray_tfar > curDist))) 
          continue;
        
        while (1)
        {
	  /* process normal nodes */
          if (likely((types & 0x1) && cur.isNode()))
          {
            const vbool<K> valid_node = ray_tfar > curDist;
            STAT3(normal.trav_nodes,1,popcnt(valid_node),K);
	    const Node* __restrict__ const node = cur.node();
	    
            /* set cur to invalid */
            cur = BVH4::emptyNode;
            curDist = pos_inf;
	    
#pragma unroll(4)
	    for (unsigned i=0; i<BVH4::N; i++)
	    {
	      const NodeRef child = node->children[i];
	      if (unlikely(child == BVH4::emptyNode)) break;
              vfloat<K> lnearP; const vbool<K> lhit = intersect_node<K,robust>(node,i,org,rdir,org_rdir,ray_tnear,ray_tfar,lnearP);
	      
	      /* if we hit the child we choose to continue with that child if it 
		 is closer than the current next child, or we push it onto the stack */
	      if (likely(any(lhit)))
	      {
		assert(sptr_node < stackEnd);
		assert(child != BVH4::emptyNode);
                const vfloat<K> childDist = select(lhit,lnearP,inf);

                /* push cur node onto stack and continue with hit child */
		if (any(childDist < curDist))
		{
                  if (likely(cur != BVH4::emptyNode)) {
                    *sptr_node = cur; sptr_node++;
                    *sptr_near = curDist; sptr_near++;
                  }
		  curDist = childDist;
		  cur = child;
		}
		
		/* push hit child onto stack */
		else {
		  *sptr_node = child; sptr_node++;
		  *sptr_near = childDist; sptr_near++;
		}
	      }     
	    }
            if (unlikely(cur == BVH4::emptyNode)) 
              goto pop;
	  }
	  
	  /* process motion blur nodes */
          else if (likely((types & 0x10) && cur.isNodeMB()))
	  {
            const vbool<K> valid_node = ray_tfar > curDist;
            STAT3(normal.trav_nodes,1,popcnt(valid_node),K);
	    const BVH4::NodeMB* __restrict__ const node = cur.nodeMB();
          
            /* set cur to invalid */
            cur = BVH4::emptyNode;
            curDist = pos_inf;
	    
#pragma unroll(4)
	    for (unsigned i=0; i<BVH4::N; i++)
	    {
	      const NodeRef child = node->child(i);
	      if (unlikely(child == BVH4::emptyNode)) break;
              vfloat<K> lnearP; const vbool<K> lhit = intersect_node<K>(node,i,org,rdir,org_rdir,ray_tnear,ray_tfar,ray.time,lnearP);
	      	      
	      /* if we hit the child we choose to continue with that child if it 
		 is closer than the current next child, or we push it onto the stack */
	      if (likely(any(lhit)))
	      {
		assert(sptr_node < stackEnd);
		assert(child != BVH4::emptyNode);
                const vfloat<K> childDist = select(lhit,lnearP,inf);

                /* push cur node onto stack and continue with hit child */
		if (any(childDist < curDist))
		{
                  if (likely(cur != BVH4::emptyNode)) {
                    *sptr_node = cur; sptr_node++;
                    *sptr_near = curDist; sptr_near++;
                  }
		  curDist = childDist;
		  cur = child;
		}
		
		/* push hit child onto stack */
		else {
		  *sptr_node = child; sptr_node++;
		  *sptr_near = childDist; sptr_near++;
		}
	      }	      
	    }
            if (unlikely(cur == BVH4::emptyNode)) 
              goto pop;
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
        const vbool<K> valid_leaf = ray_tfar > curDist;
        STAT3(shadow.trav_leaves,1,popcnt(valid_leaf),K);
        size_t items; const Primitive* prim = (Primitive*) cur.leaf(items);

        size_t lazy_node = 0;
        terminated |= valid_leaf & PrimitiveIntersectorK::occluded(valid_leaf,pre,ray,prim,items,bvh->scene,lazy_node);
        if (all(terminated)) break;
        ray_tfar = select(terminated,neg_inf,ray_tfar);

        if (unlikely(lazy_node)) {
          *sptr_node = lazy_node; sptr_node++;
          *sptr_near = neg_inf;   sptr_near++;
        }
      }
      vint<K>::store(valid & terminated,&ray.geomID,0);
      AVX_ZERO_UPPER();
    }

    ////////////////////////////////////////////////////////////////////////////////
    /// Intersector4 Definitions
    ////////////////////////////////////////////////////////////////////////////////

    DEFINE_INTERSECTOR4(BVH4Bezier1vIntersector4Chunk, BVH4IntersectorKChunk<4 COMMA 0x1 COMMA false COMMA ArrayIntersectorK<4 COMMA Bezier1vIntersectorK<4> > >);
    DEFINE_INTERSECTOR4(BVH4Bezier1iIntersector4Chunk, BVH4IntersectorKChunk<4 COMMA 0x1 COMMA false COMMA ArrayIntersectorK<4 COMMA Bezier1iIntersectorK<4> > >);
    DEFINE_INTERSECTOR4(BVH4Triangle4Intersector4ChunkMoeller, BVH4IntersectorKChunk<4 COMMA 0x1 COMMA false COMMA ArrayIntersectorK<4 COMMA TriangleMIntersectorKMoellerTrumbore<4 COMMA 4 COMMA true> > >);
    DEFINE_INTERSECTOR4(BVH4Triangle4Intersector4ChunkMoellerNoFilter, BVH4IntersectorKChunk<4 COMMA 0x1 COMMA false COMMA ArrayIntersectorK<4 COMMA TriangleMIntersectorKMoellerTrumbore<4 COMMA 4 COMMA false> > >);
#if defined(__AVX__)
    DEFINE_INTERSECTOR4(BVH4Triangle8Intersector4ChunkMoeller, BVH4IntersectorKChunk<4 COMMA 0x1 COMMA false COMMA ArrayIntersectorK<4 COMMA TriangleMIntersectorKMoellerTrumbore<8 COMMA 4 COMMA true> > >);
    DEFINE_INTERSECTOR4(BVH4Triangle8Intersector4ChunkMoellerNoFilter, BVH4IntersectorKChunk<4 COMMA 0x1 COMMA false COMMA ArrayIntersectorK<4 COMMA TriangleMIntersectorKMoellerTrumbore<8 COMMA 4 COMMA false> > >);

    DEFINE_INTERSECTOR4(BVH4TrianglePairs4Intersector4ChunkMoeller, BVH4IntersectorKChunk<4 COMMA 0x1 COMMA false COMMA ArrayIntersectorK<4 COMMA TrianglePairsMIntersectorKMoellerTrumbore<4 COMMA 4 COMMA true> > >);
    DEFINE_INTERSECTOR4(BVH4TrianglePairs4Intersector4ChunkMoellerNoFilter, BVH4IntersectorKChunk<4 COMMA 0x1 COMMA false COMMA ArrayIntersectorK<4 COMMA TrianglePairsMIntersectorKMoellerTrumbore<4 COMMA 4 COMMA false> > >);
#endif
    DEFINE_INTERSECTOR4(BVH4Triangle4vIntersector4ChunkPluecker, BVH4IntersectorKChunk<4 COMMA 0x1 COMMA true COMMA ArrayIntersectorK<4 COMMA TriangleMvIntersectorKPluecker<4 COMMA 4 COMMA true> > >);
    DEFINE_INTERSECTOR4(BVH4Triangle4iIntersector4ChunkPluecker, BVH4IntersectorKChunk<4 COMMA 0x1 COMMA true COMMA ArrayIntersectorK<4 COMMA Triangle4iIntersectorKPluecker<4 COMMA true> > >);
    DEFINE_INTERSECTOR4(BVH4VirtualIntersector4Chunk, BVH4IntersectorKChunk<4 COMMA 0x1 COMMA false COMMA ArrayIntersectorK<4 COMMA ObjectIntersector4> >);

    DEFINE_INTERSECTOR4(BVH4Triangle4vMBIntersector4ChunkMoeller, BVH4IntersectorKChunk<4 COMMA 0x10 COMMA false COMMA ArrayIntersectorK<4 COMMA TriangleMvMBIntersectorKMoellerTrumbore<4 COMMA 4 COMMA true> > >);

#if !defined(__SSE4_2__)
    DEFINE_INTERSECTOR4(BVH4Subdivpatch1CachedIntersector4, BVH4IntersectorKChunk<4 COMMA 0x1 COMMA true COMMA SubdivPatch1CachedIntersector4>);
#endif

    ////////////////////////////////////////////////////////////////////////////////
    /// Intersector8 Definitions
    ////////////////////////////////////////////////////////////////////////////////

#if defined(__AVX__)
    DEFINE_INTERSECTOR8(BVH4Bezier1vIntersector8Chunk, BVH4IntersectorKChunk<8 COMMA 0x1 COMMA false COMMA ArrayIntersectorK<8 COMMA Bezier1vIntersectorK<8> > >);
    DEFINE_INTERSECTOR8(BVH4Bezier1iIntersector8Chunk, BVH4IntersectorKChunk<8 COMMA 0x1 COMMA false COMMA ArrayIntersectorK<8 COMMA Bezier1iIntersectorK<8> > >);
    DEFINE_INTERSECTOR8(BVH4Triangle4Intersector8ChunkMoeller, BVH4IntersectorKChunk<8 COMMA 0x1 COMMA false COMMA ArrayIntersectorK<8 COMMA TriangleMIntersectorKMoellerTrumbore<4 COMMA 8 COMMA true> > >);
    DEFINE_INTERSECTOR8(BVH4Triangle4Intersector8ChunkMoellerNoFilter, BVH4IntersectorKChunk<8 COMMA 0x1 COMMA false COMMA ArrayIntersectorK<8 COMMA TriangleMIntersectorKMoellerTrumbore<4 COMMA 8 COMMA false> > >);
    DEFINE_INTERSECTOR8(BVH4Triangle8Intersector8ChunkMoeller, BVH4IntersectorKChunk<8 COMMA 0x1 COMMA false COMMA ArrayIntersectorK<8 COMMA TriangleMIntersectorKMoellerTrumbore<8 COMMA 8 COMMA true> > >);
    DEFINE_INTERSECTOR8(BVH4Triangle8Intersector8ChunkMoellerNoFilter, BVH4IntersectorKChunk<8 COMMA 0x1 COMMA false COMMA ArrayIntersectorK<8 COMMA TriangleMIntersectorKMoellerTrumbore<8 COMMA 8 COMMA false> > >);
    DEFINE_INTERSECTOR8(BVH4Triangle4vIntersector8ChunkPluecker, BVH4IntersectorKChunk<8 COMMA 0x1 COMMA true COMMA ArrayIntersectorK<8 COMMA TriangleMvIntersectorKPluecker<4 COMMA 8 COMMA true> > >);
    DEFINE_INTERSECTOR8(BVH4Triangle4iIntersector8ChunkPluecker, BVH4IntersectorKChunk<8 COMMA 0x1 COMMA true COMMA ArrayIntersectorK<8 COMMA Triangle4iIntersectorKPluecker<8 COMMA true> > >);
    DEFINE_INTERSECTOR8(BVH4VirtualIntersector8Chunk, BVH4IntersectorKChunk<8 COMMA 0x1 COMMA false COMMA ArrayIntersectorK<8 COMMA ObjectIntersector8> >);

    DEFINE_INTERSECTOR8(BVH4Triangle4vMBIntersector8ChunkMoeller, BVH4IntersectorKChunk<8 COMMA 0x10 COMMA false COMMA ArrayIntersectorK<8 COMMA TriangleMvMBIntersectorKMoellerTrumbore<4 COMMA 8 COMMA true> > >);

    DEFINE_INTERSECTOR4(BVH4TrianglePairs4Intersector8ChunkMoeller, BVH4IntersectorKChunk<8 COMMA 0x1 COMMA false COMMA ArrayIntersectorK<8 COMMA TrianglePairsMIntersectorKMoellerTrumbore<4 COMMA 8 COMMA true> > >);
    DEFINE_INTERSECTOR4(BVH4TrianglePairs4Intersector8ChunkMoellerNoFilter, BVH4IntersectorKChunk<8 COMMA 0x1 COMMA false COMMA ArrayIntersectorK<8 COMMA TrianglePairsMIntersectorKMoellerTrumbore<4 COMMA 8 COMMA false> > >);
#endif

    ////////////////////////////////////////////////////////////////////////////////
    /// Intersector16 Definitions
    ////////////////////////////////////////////////////////////////////////////////

#if defined(__AVX512F__)
    DEFINE_INTERSECTOR16(BVH4Bezier1vIntersector16Chunk, BVH4IntersectorKChunk<16 COMMA 0x1 COMMA false COMMA ArrayIntersectorK<16 COMMA Bezier1vIntersectorK<16> > >);
    DEFINE_INTERSECTOR16(BVH4Bezier1iIntersector16Chunk, BVH4IntersectorKChunk<16 COMMA 0x1 COMMA false COMMA ArrayIntersectorK<16 COMMA Bezier1iIntersectorK<16> > >);
    DEFINE_INTERSECTOR16(BVH4Triangle4Intersector16ChunkMoeller, BVH4IntersectorKChunk<16 COMMA 0x1 COMMA false COMMA ArrayIntersectorK<16 COMMA TriangleMIntersectorKMoellerTrumbore<4 COMMA 16 COMMA true> > >);
    DEFINE_INTERSECTOR16(BVH4Triangle4Intersector16ChunkMoellerNoFilter, BVH4IntersectorKChunk<16 COMMA 0x1 COMMA false COMMA ArrayIntersectorK<16 COMMA TriangleMIntersectorKMoellerTrumbore<4 COMMA 16 COMMA false> > >);
    DEFINE_INTERSECTOR16(BVH4Triangle8Intersector16ChunkMoeller, BVH4IntersectorKChunk<16 COMMA 0x1 COMMA false COMMA ArrayIntersectorK<16 COMMA TriangleMIntersectorKMoellerTrumbore<8 COMMA 16 COMMA true> > >);
    DEFINE_INTERSECTOR16(BVH4Triangle8Intersector16ChunkMoellerNoFilter, BVH4IntersectorKChunk<16 COMMA 0x1 COMMA false COMMA ArrayIntersectorK<16 COMMA TriangleMIntersectorKMoellerTrumbore<8 COMMA 16 COMMA false> > >);

    DEFINE_INTERSECTOR16(BVH4TrianglePairs4Intersector16ChunkMoeller, BVH4IntersectorKChunk<16 COMMA 0x1 COMMA false COMMA ArrayIntersectorK<16 COMMA TrianglePairsMIntersectorKMoellerTrumbore<4 COMMA 16 COMMA true> > >);

    DEFINE_INTERSECTOR16(BVH4TrianglePairs4Intersector16ChunkMoellerNoFilter, BVH4IntersectorKChunk<16 COMMA 0x1 COMMA false COMMA ArrayIntersectorK<16 COMMA TrianglePairsMIntersectorKMoellerTrumbore<4 COMMA 16 COMMA false> > >);

    DEFINE_INTERSECTOR16(BVH4Triangle4vIntersector16ChunkPluecker, BVH4IntersectorKChunk<16 COMMA 0x1 COMMA true COMMA ArrayIntersectorK<16 COMMA TriangleMvIntersectorKPluecker<4 COMMA 16 COMMA true> > >);
    DEFINE_INTERSECTOR16(BVH4Triangle4iIntersector16ChunkPluecker, BVH4IntersectorKChunk<16 COMMA 0x1 COMMA true COMMA ArrayIntersectorK<16 COMMA Triangle4iIntersectorKPluecker<16 COMMA true> > >);
    DEFINE_INTERSECTOR16(BVH4VirtualIntersector16Chunk, BVH4IntersectorKChunk<16 COMMA 0x1 COMMA false COMMA ArrayIntersectorK<16 COMMA ObjectIntersector16> >);
    DEFINE_INTERSECTOR16(BVH4Triangle4vMBIntersector16ChunkMoeller, BVH4IntersectorKChunk<16 COMMA 0x10 COMMA false COMMA ArrayIntersectorK<16 COMMA TriangleMvMBIntersectorKMoellerTrumbore<4 COMMA 16 COMMA true> > >);
#endif
  }
}
