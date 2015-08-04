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

#include "bvh8_intersector8_chunk.h"
#include "../geometry/triangle4.h"
#include "../geometry/triangle8.h"
#include "../geometry/intersector_iterators.h"
#include "../geometry/triangle_intersector_moeller.h"

#define DBG(x) 

namespace embree
{
  namespace isa
  {    
    
    template<typename PrimitiveIntersector8>    
    void BVH8Intersector8Chunk<PrimitiveIntersector8>::intersect(bool8* valid_i, BVH8* bvh, Ray8& ray)
    {
#if defined(__AVX__)
      
      /* load ray */
      bool8 valid0 = *valid_i;
#if defined(RTCORE_IGNORE_INVALID_RAYS)
      valid0 &= ray.valid();
#endif
      assert(all(valid0,ray.tnear > -FLT_MIN));
      const Vec3f8 rdir = rcp_safe(ray.dir);
      const Vec3f8 org_rdir = ray.org * rdir;
      float8 ray_tnear = select(valid0,ray.tnear,pos_inf);
      float8 ray_tfar  = select(valid0,ray.tfar ,neg_inf);
      const float8 inf = float8(pos_inf);
      Precalculations pre(valid0,ray);
      
      /* allocate stack and push root node */
      float8    stack_near[3*BVH8::maxDepth+1];
      NodeRef stack_node[3*BVH8::maxDepth+1];
      stack_node[0] = BVH8::invalidNode;
      stack_near[0] = inf;
      stack_node[1] = bvh->root;
      stack_near[1] = ray_tnear; 
      NodeRef* __restrict__ sptr_node = stack_node + 2;
      float8*    __restrict__ sptr_near = stack_near + 2;
      
      while (1)
      {
        /* pop next node from stack */
        sptr_node--;
        sptr_near--;
        NodeRef cur = *sptr_node;
        if (unlikely(cur == BVH8::invalidNode)) 
          break;
        
        /* cull node if behind closest hit point */
        float8 curDist = *sptr_near;
        if (unlikely(none(ray_tfar > curDist))) 
          continue;
        
        while (1)
        {
          /* test if this is a leaf node */
          if (unlikely(cur.isLeaf()))
            break;
          
          const bool8 valid_node = ray_tfar > curDist;
          STAT3(normal.trav_nodes,1,popcnt(valid_node),8);
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
            
#if defined(__AVX2__)
            const float8 lclipMinX = msub(node->lower_x[i],rdir.x,org_rdir.x);
            const float8 lclipMinY = msub(node->lower_y[i],rdir.y,org_rdir.y);
            const float8 lclipMinZ = msub(node->lower_z[i],rdir.z,org_rdir.z);
            const float8 lclipMaxX = msub(node->upper_x[i],rdir.x,org_rdir.x);
            const float8 lclipMaxY = msub(node->upper_y[i],rdir.y,org_rdir.y);
            const float8 lclipMaxZ = msub(node->upper_z[i],rdir.z,org_rdir.z);
            const float8 lnearP = maxi(maxi(mini(lclipMinX, lclipMaxX), mini(lclipMinY, lclipMaxY)), mini(lclipMinZ, lclipMaxZ));
            const float8 lfarP  = mini(mini(maxi(lclipMinX, lclipMaxX), maxi(lclipMinY, lclipMaxY)), maxi(lclipMinZ, lclipMaxZ));
            const bool8 lhit   = maxi(lnearP,ray_tnear) <= mini(lfarP,ray_tfar);      
#else
            const float8 lclipMinX = node->lower_x[i] * rdir.x - org_rdir.x;
            const float8 lclipMinY = node->lower_y[i] * rdir.y - org_rdir.y;
            const float8 lclipMinZ = node->lower_z[i] * rdir.z - org_rdir.z;
            const float8 lclipMaxX = node->upper_x[i] * rdir.x - org_rdir.x;
            const float8 lclipMaxY = node->upper_y[i] * rdir.y - org_rdir.y;
            const float8 lclipMaxZ = node->upper_z[i] * rdir.z - org_rdir.z;
            const float8 lnearP = max(max(min(lclipMinX, lclipMaxX), min(lclipMinY, lclipMaxY)), min(lclipMinZ, lclipMaxZ));
            const float8 lfarP  = min(min(max(lclipMinX, lclipMaxX), max(lclipMinY, lclipMaxY)), max(lclipMinZ, lclipMaxZ));
            const bool8 lhit   = max(lnearP,ray_tnear) <= min(lfarP,ray_tfar);      
#endif
            
            /* if we hit the child we choose to continue with that child if it 
               is closer than the current next child, or we push it onto the stack */
            if (likely(any(lhit)))
            {
              const float8 childDist = select(lhit,lnearP,inf);
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
        const bool8 valid_leaf = ray_tfar > curDist;
        STAT3(normal.trav_leaves,1,popcnt(valid_leaf),8);
        size_t items; const Triangle* tri  = (Triangle*) cur.leaf(items);

        size_t lazy_node = 0;
        PrimitiveIntersector8::intersect(valid_leaf,pre,ray,tri,items,bvh->scene,lazy_node);
        ray_tfar = select(valid_leaf,ray.tfar,ray_tfar);

        if (unlikely(lazy_node)) {
          *sptr_node = lazy_node; sptr_node++;
          *sptr_near = neg_inf;   sptr_near++;
        }
      }
      AVX_ZERO_UPPER();
#endif       
    }
    
     template<typename PrimitiveIntersector8>
    void BVH8Intersector8Chunk<PrimitiveIntersector8>::occluded(bool8* valid_i, BVH8* bvh, Ray8& ray)
    {
#if defined(__AVX__)
      
      /* load ray */
      bool8 valid = *valid_i;
#if defined(RTCORE_IGNORE_INVALID_RAYS)
      valid &= ray.valid();
#endif
      assert(all(valid,ray.tnear > -FLT_MIN));
      bool8 terminated = !valid;
      const Vec3f8 rdir = rcp_safe(ray.dir);
      const Vec3f8 org_rdir = ray.org * rdir;
      float8 ray_tnear = select(valid,ray.tnear,pos_inf);
      float8 ray_tfar  = select(valid,ray.tfar ,neg_inf);
      const float8 inf = float8(pos_inf);
      Precalculations pre(valid,ray);

      /* allocate stack and push root node */
      float8    stack_near[3*BVH8::maxDepth+1];
      NodeRef stack_node[3*BVH8::maxDepth+1];
      stack_node[0] = BVH8::invalidNode;
      stack_near[0] = inf;
      stack_node[1] = bvh->root;
      stack_near[1] = ray_tnear; 
      NodeRef* __restrict__ sptr_node = stack_node + 2;
      float8*    __restrict__ sptr_near = stack_near + 2;
      
      while (1)
      {
        /* pop next node from stack */
        sptr_node--;
        sptr_near--;
        NodeRef cur = *sptr_node;
        if (unlikely(cur == BVH8::invalidNode)) 
          break;
        
        /* cull node if behind closest hit point */
        float8 curDist = *sptr_near;
        if (unlikely(none(ray_tfar > curDist))) 
          continue;
        
        while (1)
        {
          /* test if this is a leaf node */
          if (unlikely(cur.isLeaf()))
            break;
          
          const bool8 valid_node = ray_tfar > curDist;
          STAT3(shadow.trav_nodes,1,popcnt(valid_node),8);
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
            
#if defined(__AVX2__)
            const float8 lclipMinX = msub(node->lower_x[i],rdir.x,org_rdir.x);
            const float8 lclipMinY = msub(node->lower_y[i],rdir.y,org_rdir.y);
            const float8 lclipMinZ = msub(node->lower_z[i],rdir.z,org_rdir.z);
            const float8 lclipMaxX = msub(node->upper_x[i],rdir.x,org_rdir.x);
            const float8 lclipMaxY = msub(node->upper_y[i],rdir.y,org_rdir.y);
            const float8 lclipMaxZ = msub(node->upper_z[i],rdir.z,org_rdir.z);
            const float8 lnearP = maxi(maxi(mini(lclipMinX, lclipMaxX), mini(lclipMinY, lclipMaxY)), mini(lclipMinZ, lclipMaxZ));
            const float8 lfarP  = mini(mini(maxi(lclipMinX, lclipMaxX), maxi(lclipMinY, lclipMaxY)), maxi(lclipMinZ, lclipMaxZ));
            const bool8 lhit   = maxi(lnearP,ray_tnear) <= mini(lfarP,ray_tfar);      
#else
            const float8 lclipMinX = node->lower_x[i] * rdir.x - org_rdir.x;
            const float8 lclipMinY = node->lower_y[i] * rdir.y - org_rdir.y;
            const float8 lclipMinZ = node->lower_z[i] * rdir.z - org_rdir.z;
            const float8 lclipMaxX = node->upper_x[i] * rdir.x - org_rdir.x;
            const float8 lclipMaxY = node->upper_y[i] * rdir.y - org_rdir.y;
            const float8 lclipMaxZ = node->upper_z[i] * rdir.z - org_rdir.z;
            const float8 lnearP = max(max(min(lclipMinX, lclipMaxX), min(lclipMinY, lclipMaxY)), min(lclipMinZ, lclipMaxZ));
            const float8 lfarP  = min(min(max(lclipMinX, lclipMaxX), max(lclipMinY, lclipMaxY)), max(lclipMinZ, lclipMaxZ));
            const bool8 lhit   = max(lnearP,ray_tnear) <= min(lfarP,ray_tfar);      
#endif
            
            /* if we hit the child we choose to continue with that child if it 
               is closer than the current next child, or we push it onto the stack */
            if (likely(any(lhit)))
            {
              const float8 childDist = select(lhit,lnearP,inf);
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
        const bool8 valid_leaf = ray_tfar > curDist;
        STAT3(shadow.trav_leaves,1,popcnt(valid_leaf),8);
        size_t items; const Triangle* tri  = (Triangle*) cur.leaf(items);
        
        size_t lazy_node = 0;
        terminated |= PrimitiveIntersector8::occluded(!terminated,pre,ray,tri,items,bvh->scene,lazy_node);
        if (all(terminated)) break;
        ray_tfar = select(terminated,neg_inf,ray_tfar);

        if (unlikely(lazy_node)) {
          *sptr_node = lazy_node; sptr_node++;
          *sptr_near = neg_inf;   sptr_near++;
        }
      }
      store8i(valid & terminated,&ray.geomID,0);
      AVX_ZERO_UPPER();
#endif      
    }
    
    DEFINE_INTERSECTOR8(BVH8Triangle4Intersector8ChunkMoeller,BVH8Intersector8Chunk<ArrayIntersector8<TriangleNIntersectorMMoellerTrumbore<Ray8 COMMA Triangle4 COMMA true> > >);
    DEFINE_INTERSECTOR8(BVH8Triangle8Intersector8ChunkMoeller,BVH8Intersector8Chunk<ArrayIntersector8<TriangleNIntersectorMMoellerTrumbore<Ray8 COMMA Triangle8 COMMA true> > >);
  }
}  
