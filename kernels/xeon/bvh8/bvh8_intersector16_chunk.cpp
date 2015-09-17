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

#include "bvh8_intersector16_chunk.h"
#include "../geometry/triangle4.h"
#include "../geometry/triangle8.h"
#include "../geometry/intersector_iterators.h"
#include "../geometry/triangle_intersector_moeller.h"

#define DBG(x) 

namespace embree
{
  namespace isa
  {    
    
    template<typename PrimitiveIntersector16>    
    void BVH8Intersector16Chunk<PrimitiveIntersector16>::intersect(int16* valid_i, BVH8* bvh, Ray16& ray)
    {
#if defined(__AVX512F__)
      
      /* load ray */
      bool16 valid0 = *valid_i == -1;
#if defined(RTCORE_IGNORE_INVALID_RAYS)
      valid0 &= ray.valid();
#endif
      assert(all(valid0,ray.tnear > -FLT_MIN));
      //assert(!(types & BVH4::FLAG_NODE_MB) || all(valid0,ray.time >= 0.0f & ray.time <= 1.0f));

      const Vec3f16 rdir = rcp_safe(ray.dir);
      const Vec3f16 org_rdir = ray.org * rdir;
      float16 ray_tnear = select(valid0,ray.tnear,pos_inf);
      float16 ray_tfar  = select(valid0,ray.tfar ,neg_inf);
      const float16 inf = float16(pos_inf);
      Precalculations pre(valid0,ray);
      
      /* allocate stack and push root node */
      float16    stack_near[3*BVH8::maxDepth+1];
      NodeRef stack_node[3*BVH8::maxDepth+1];
      stack_node[0] = BVH8::invalidNode;
      stack_near[0] = inf;
      stack_node[1] = bvh->root;
      stack_near[1] = ray_tnear; 
      NodeRef* __restrict__ sptr_node = stack_node + 2;
      float16*    __restrict__ sptr_near = stack_near + 2;
      
      while (1)
      {
        /* pop next node from stack */
        sptr_node--;
        sptr_near--;
        NodeRef cur = *sptr_node;
        if (unlikely(cur == BVH8::invalidNode)) 
          break;
        
        /* cull node if behind closest hit point */
        float16 curDist = *sptr_near;
        if (unlikely(none(ray_tfar > curDist))) 
          continue;
        
        while (1)
        {
          /* test if this is a leaf node */
          if (unlikely(cur.isLeaf()))
            break;
          
          const bool16 valid_node = ray_tfar > curDist;
          STAT3(normal.trav_nodes,1,popcnt(valid_node),16);
          const Node* __restrict__ const node = (BVH8::Node*)cur.node();
          
          /* pop of next node */
          sptr_node--;
          sptr_near--;
          cur = *sptr_node; // FIXME: this trick creates issues with stack depth
          curDist = *sptr_near;
          
          for (unsigned i=0; i<BVH8::N; i++)
          {
            const NodeRef child = node->children[i];
            if (unlikely(child == BVH8::emptyNode)) break;
            
            const float16 lclipMinX = msub(node->lower_x[i],rdir.x,org_rdir.x);
            const float16 lclipMinY = msub(node->lower_y[i],rdir.y,org_rdir.y);
            const float16 lclipMinZ = msub(node->lower_z[i],rdir.z,org_rdir.z);
            const float16 lclipMaxX = msub(node->upper_x[i],rdir.x,org_rdir.x);
            const float16 lclipMaxY = msub(node->upper_y[i],rdir.y,org_rdir.y);
            const float16 lclipMaxZ = msub(node->upper_z[i],rdir.z,org_rdir.z);
            const float16 lnearP = max(max(min(lclipMinX, lclipMaxX), min(lclipMinY, lclipMaxY)), min(lclipMinZ, lclipMaxZ));
            const float16 lfarP  = min(min(max(lclipMinX, lclipMaxX), max(lclipMinY, lclipMaxY)), max(lclipMinZ, lclipMaxZ));
            const bool16 lhit   = max(lnearP,ray_tnear) <= min(lfarP,ray_tfar);      
            
            /* if we hit the child we choose to continue with that child if it 
               is closer than the current next child, or we push it onto the stack */
            if (likely(any(lhit)))
            {
              const float16 childDist = select(lhit,lnearP,inf);
              const NodeRef child = node->children[i];
              
              /* push cur node onto stack and continue with hit child */
              if (any(childDist < curDist))
              {
                *sptr_node = cur;
                *sptr_near = curDist; 
		sptr_node++;
		sptr_near++;

                curDist = childDist;
                cur = child;
              }
              
              /* push hit child onto stack*/
              else {
                *sptr_node = child;
                *sptr_near = childDist; 
		sptr_node++;
		sptr_near++;

              }
              assert(sptr_node - stack_node < BVH8::maxDepth);
            }	      
          }
        }
        
        /* return if stack is empty */
        if (unlikely(cur == BVH8::invalidNode)) 
          break;
        
        /* intersect leaf */
	assert(cur != BVH8::emptyNode);
        const bool16 valid_leaf = ray_tfar > curDist;
        STAT3(normal.trav_leaves,1,popcnt(valid_leaf),16);
        size_t items; const Triangle* tri  = (Triangle*) cur.leaf(items);

        size_t lazy_node = 0;
        PrimitiveIntersector16::intersect(valid_leaf,pre,ray,tri,items,bvh->scene,lazy_node);
        ray_tfar = select(valid_leaf,ray.tfar,ray_tfar);

        if (unlikely(lazy_node)) {
          *sptr_node = lazy_node; sptr_node++;
          *sptr_near = neg_inf;   sptr_near++;
        }
      }
      AVX_ZERO_UPPER();
#endif       
    }
    
     template<typename PrimitiveIntersector16>
    void BVH8Intersector16Chunk<PrimitiveIntersector16>::occluded(int16* valid_i, BVH8* bvh, Ray16& ray)
    {
#if defined(__AVX512F__)
      
      /* load ray */
      const bool16 valid = *valid_i == -1;
#if defined(RTCORE_IGNORE_INVALID_RAYS)
      valid &= ray.valid();
#endif
      assert(all(valid,ray.tnear > -FLT_MIN));
      //assert(!(types & BVH4::FLAG_NODE_MB) || all(valid0,ray.time >= 0.0f & ray.time <= 1.0f));

      bool16 terminated = !valid;
      const Vec3f16 rdir = rcp_safe(ray.dir);
      const Vec3f16 org_rdir = ray.org * rdir;
      float16 ray_tnear = select(valid,ray.tnear,pos_inf);
      float16 ray_tfar  = select(valid,ray.tfar ,neg_inf);
      const float16 inf = float16(pos_inf);
      Precalculations pre(valid,ray);

      /* allocate stack and push root node */
      float16    stack_near[3*BVH8::maxDepth+1];
      NodeRef stack_node[3*BVH8::maxDepth+1];
      stack_node[0] = BVH8::invalidNode;
      stack_near[0] = inf;
      stack_node[1] = bvh->root;
      stack_near[1] = ray_tnear; 
      NodeRef* __restrict__ sptr_node = stack_node + 2;
      float16*    __restrict__ sptr_near = stack_near + 2;
      
      while (1)
      {
        /* pop next node from stack */
        sptr_node--;
        sptr_near--;
        NodeRef cur = *sptr_node;
        if (unlikely(cur == BVH8::invalidNode)) 
          break;
        
        /* cull node if behind closest hit point */
        float16 curDist = *sptr_near;
        if (unlikely(none(ray_tfar > curDist))) 
          continue;
        
        while (1)
        {
          /* test if this is a leaf node */
          if (unlikely(cur.isLeaf()))
            break;
          
          const bool16 valid_node = ray_tfar > curDist;
          STAT3(shadow.trav_nodes,1,popcnt(valid_node),16);
          const Node* __restrict__ const node = (Node*)cur.node();
          
          /* pop of next node */
          sptr_node--;
          sptr_near--;
          cur = *sptr_node; // FIXME: this trick creates issues with stack depth
          curDist = *sptr_near;
          
          for (unsigned i=0; i<BVH8::N; i++)
          {
            const NodeRef child = node->children[i];
            if (unlikely(child == BVH8::emptyNode)) break;
            
            const float16 lclipMinX = msub(node->lower_x[i],rdir.x,org_rdir.x);
            const float16 lclipMinY = msub(node->lower_y[i],rdir.y,org_rdir.y);
            const float16 lclipMinZ = msub(node->lower_z[i],rdir.z,org_rdir.z);
            const float16 lclipMaxX = msub(node->upper_x[i],rdir.x,org_rdir.x);
            const float16 lclipMaxY = msub(node->upper_y[i],rdir.y,org_rdir.y);
            const float16 lclipMaxZ = msub(node->upper_z[i],rdir.z,org_rdir.z);
            const float16 lnearP = max(max(min(lclipMinX, lclipMaxX), min(lclipMinY, lclipMaxY)), min(lclipMinZ, lclipMaxZ));
            const float16 lfarP  = min(min(max(lclipMinX, lclipMaxX), max(lclipMinY, lclipMaxY)), max(lclipMinZ, lclipMaxZ));
            const bool16 lhit   = max(lnearP,ray_tnear) <= min(lfarP,ray_tfar);      
            
            /* if we hit the child we choose to continue with that child if it 
               is closer than the current next child, or we push it onto the stack */
            if (likely(any(lhit)))
            {
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
              
              /* push hit child onto stack*/
              else {
                *(sptr_node-1) = child;
                *(sptr_near-1) = childDist; 
              }
              assert(sptr_node - stack_node < BVH8::maxDepth);
            }	      
          }
        }
        
        /* return if stack is empty */
        if (unlikely(cur == BVH8::invalidNode)) 
          break;
        
        /* intersect leaf */
	assert(cur != BVH8::emptyNode);
        const bool16 valid_leaf = ray_tfar > curDist;
        STAT3(shadow.trav_leaves,1,popcnt(valid_leaf),16);
        size_t items; const Triangle* tri  = (Triangle*) cur.leaf(items);

        size_t lazy_node = 0;
        terminated |= PrimitiveIntersector16::occluded(!terminated,pre,ray,tri,items,bvh->scene,lazy_node);
        if (all(terminated)) break;
        ray_tfar = select(terminated,neg_inf,ray_tfar);

        if (unlikely(lazy_node)) {
          *sptr_node = lazy_node; sptr_node++;
          *sptr_near = neg_inf;   sptr_near++;
        }
      }
      store16i(valid & terminated,&ray.geomID,0);
      AVX_ZERO_UPPER();
#endif      
    }
    
    DEFINE_INTERSECTOR8(BVH8Triangle4Intersector16ChunkMoeller,BVH8Intersector16Chunk<ArrayIntersector16<TriangleNIntersectorMMoellerTrumbore<Ray16 COMMA Triangle4 COMMA true> > >);
    DEFINE_INTERSECTOR8(BVH8Triangle8Intersector16ChunkMoeller,BVH8Intersector16Chunk<ArrayIntersector16<TriangleNIntersectorMMoellerTrumbore<Ray16 COMMA Triangle8 COMMA true> > >);
  }
}  
