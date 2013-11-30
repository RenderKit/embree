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

#include "bvh4_intersector16_chunk.h"

#include "geometry/triangle1_intersector16_moeller.h"
//#include "geometry/triangle4_intersector16_moeller.h"
//#include "geometry/triangle1v_intersector16_pluecker.h"
//#include "geometry/triangle4v_intersector16_pluecker.h"
//#include "geometry/trianglemesh_accel_triangle4i_intersector16.h"
//#include "geometry/flat_triangle_accel_triangle4i_intersector16.h"
#include "geometry/virtual_accel_intersector16.h"

namespace embree
{
  namespace isa
  {
    template<typename TriangleIntersector16>
    void BVH4Intersector16Chunk<TriangleIntersector16>::intersect(mic_i* valid_i, BVH4* bvh, Ray16& ray)
    {
      /* load ray */
      const mic_m valid0 = *(mic_i*)valid_i != mic_i(0);
      const mic3f rdir = rcp_safe(ray.dir);
      const mic3f org_rdir = ray.org * rdir;
      mic_f ray_tnear = select(valid0,ray.tnear,pos_inf);
      mic_f ray_tfar  = select(valid0,ray.tfar ,neg_inf);
      const mic_f inf = mic_f(pos_inf);
      
      /* allocate stack and push root node */
      mic_f    stack_near[3*BVH4::maxDepth+1];
      NodeRef stack_node[3*BVH4::maxDepth+1];
      stack_node[0] = BVH4::invalidNode;
      stack_near[0] = inf;
      stack_node[1] = bvh->root;
      stack_near[1] = ray_tnear; 
      NodeRef* __restrict__ sptr_node = stack_node + 2;
      mic_f*    __restrict__ sptr_near = stack_near + 2;
      
      while (1)
      {
        /* pop next node from stack */
        sptr_node--;
        sptr_near--;
        NodeRef curNode = *sptr_node;
        if (unlikely(curNode == BVH4::invalidNode)) 
          break;
        
        /* cull node if behind closest hit point */
        mic_f curDist = *sptr_near;
        if (unlikely(none(ray_tfar > curDist))) 
          continue;
        
        while (1)
        {
          /* test if this is a leaf node */
          if (unlikely(curNode.isLeaf()))
            break;
          
          const mic_m valid_node = ray_tfar > curDist;
          STAT3(normal.trav_nodes,1,popcnt(valid_node),16);
          const Node* __restrict__ const node = curNode.node();
          
          /* pop of next node */
          sptr_node--;
          sptr_near--;
          curNode = *sptr_node; // FIXME: this trick creates issues with stack depth
          curDist = *sptr_near;
          
#pragma unroll(4)
          for (unsigned i=0; i<4; i++)
          {
            const NodeRef child = node->children[i];
            if (unlikely(child == BVH4::emptyNode)) break;
            
            const mic_f lclipMinX = msub(node->lower_x[i],rdir.x,org_rdir.x);
            const mic_f lclipMinY = msub(node->lower_y[i],rdir.y,org_rdir.y);
            const mic_f lclipMinZ = msub(node->lower_z[i],rdir.z,org_rdir.z);
            const mic_f lclipMaxX = msub(node->upper_x[i],rdir.x,org_rdir.x);
            const mic_f lclipMaxY = msub(node->upper_y[i],rdir.y,org_rdir.y);
            const mic_f lclipMaxZ = msub(node->upper_z[i],rdir.z,org_rdir.z);
            const mic_f lnearP = max(max(min(lclipMinX, lclipMaxX), min(lclipMinY, lclipMaxY)), min(lclipMinZ, lclipMaxZ));
            const mic_f lfarP  = min(min(max(lclipMinX, lclipMaxX), max(lclipMinY, lclipMaxY)), max(lclipMinZ, lclipMaxZ));
            const mic_m lhit   = max(lnearP,ray_tnear) <= min(lfarP,ray_tfar);   
            
            /* if we hit the child we choose to continue with that child if it 
               is closer than the current next child, or we push it onto the stack */
            if (likely(any(lhit)))
            {
              const mic_f childDist = select(lhit,lnearP,inf);
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
              
              /* push hit child onto stack*/
              else {
                *(sptr_node-1) = child;
                *(sptr_near-1) = childDist; 
              }
              assert(sptr_node - stack_node < BVH4::maxDepth);
            }	      
          }
        }
        
        /* return if stack is empty */
        if (unlikely(curNode == BVH4::invalidNode)) 
          break;
        
        /* intersect leaf */
        const mic_m valid_leaf = ray_tfar > curDist;
        STAT3(normal.trav_leaves,1,popcnt(valid_leaf),16);
        size_t items; const Triangle* tri  = (Triangle*) curNode.leaf(items);
        TriangleIntersector16::intersect(valid_leaf,ray,tri,items,bvh->geometry);
        ray_tfar = select(valid_leaf,ray.tfar,ray_tfar);
      }
      AVX_ZERO_UPPER();
    }
    
    template<typename TriangleIntersector16>
    void BVH4Intersector16Chunk<TriangleIntersector16>::occluded(mic_i* valid_i, BVH4* bvh, Ray16& ray)
    {
      /* load ray */
      const mic_m valid = *(mic_i*)valid_i != mic_i(0);
      mic_m terminated = !valid;
      const mic3f rdir = rcp_safe(ray.dir);
      const mic3f org_rdir = ray.org * rdir;
      mic_f ray_tnear = select(valid,ray.tnear,pos_inf);
      mic_f ray_tfar  = select(valid,ray.tfar ,neg_inf);
      const mic_f inf = mic_f(pos_inf);
      
      /* allocate stack and push root node */
      mic_f    stack_near[3*BVH4::maxDepth+1];
      NodeRef stack_node[3*BVH4::maxDepth+1];
      stack_node[0] = BVH4::invalidNode;
      stack_near[0] = inf;
      stack_node[1] = bvh->root;
      stack_near[1] = ray_tnear; 
      NodeRef* __restrict__ sptr_node = stack_node + 2;
      mic_f*    __restrict__ sptr_near = stack_near + 2;
      
      while (1)
      {
        /* pop next node from stack */
        sptr_node--;
        sptr_near--;
        NodeRef curNode = *sptr_node;
        if (unlikely(curNode == BVH4::invalidNode)) 
          break;
        
        /* cull node if behind closest hit point */
        mic_f curDist = *sptr_near;
        if (unlikely(none(ray_tfar > curDist))) 
          continue;
        
        while (1)
        {
          /* test if this is a leaf node */
          if (unlikely(curNode.isLeaf()))
            break;
          
          const mic_m valid_node = ray_tfar > curDist;
          STAT3(shadow.trav_nodes,1,popcnt(valid_node),16);
          const Node* __restrict__ const node = curNode.node();
          
          /* pop of next node */
          sptr_node--;
          sptr_near--;
          curNode = *sptr_node; // FIXME: this trick creates issues with stack depth
          curDist = *sptr_near;
          
#pragma unroll(4)
          for (unsigned i=0; i<4; i++)
          {
            const NodeRef child = node->children[i];
            if (unlikely(child == BVH4::emptyNode)) break;
            
            const mic_f lclipMinX = msub(node->lower_x[i],rdir.x,org_rdir.x);
            const mic_f lclipMinY = msub(node->lower_y[i],rdir.y,org_rdir.y);
            const mic_f lclipMinZ = msub(node->lower_z[i],rdir.z,org_rdir.z);
            const mic_f lclipMaxX = msub(node->upper_x[i],rdir.x,org_rdir.x);
            const mic_f lclipMaxY = msub(node->upper_y[i],rdir.y,org_rdir.y);
            const mic_f lclipMaxZ = msub(node->upper_z[i],rdir.z,org_rdir.z);
            const mic_f lnearP = max(max(min(lclipMinX, lclipMaxX), min(lclipMinY, lclipMaxY)), min(lclipMinZ, lclipMaxZ));
            const mic_f lfarP  = min(min(max(lclipMinX, lclipMaxX), max(lclipMinY, lclipMaxY)), max(lclipMinZ, lclipMaxZ));
            const mic_m lhit   = max(lnearP,ray_tnear) <= min(lfarP,ray_tfar);      
            
            /* if we hit the child we choose to continue with that child if it 
               is closer than the current next child, or we push it onto the stack */
            if (likely(any(lhit)))
            {
              const mic_f childDist = select(lhit,lnearP,inf);
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
              
              /* push hit child onto stack*/
              else {
                *(sptr_node-1) = child;
                *(sptr_near-1) = childDist; 
              }
              assert(sptr_node - stack_node < BVH4::maxDepth);
            }	      
          }
        }
        
        /* return if stack is empty */
        if (unlikely(curNode == BVH4::invalidNode)) 
          break;
        
        /* intersect leaf */
        const mic_m valid_leaf = ray_tfar > curDist;
        STAT3(shadow.trav_leaves,1,popcnt(valid_leaf),16);
        size_t items; const Triangle* tri  = (Triangle*) curNode.leaf(items);
        terminated |= valid_leaf & TriangleIntersector16::occluded(valid_leaf,ray,tri,items,bvh->geometry);
        if (all(terminated)) break;
        ray_tfar = select(terminated,neg_inf,ray_tfar);
      }
      store16i(valid & terminated,&ray.geomID,0);
      AVX_ZERO_UPPER();
    }
    
    // FIXME: convert intersector16 to intersector8 and intersector4
    DEFINE_INTERSECTOR16    (BVH4Triangle1Intersector16ChunkMoeller, BVH4Intersector16Chunk<Triangle1Intersector16MoellerTrumbore>);
    //DEFINE_INTERSECTOR16    (BVH4Triangle4Intersector16ChunkMoeller, BVH4Intersector16Chunk<Triangle4Intersector16MoellerTrumbore>);
    //DEFINE_INTERSECTOR16    (BVH4Triangle1vIntersector16ChunkPluecker, BVH4Intersector16Chunk<Triangle1vIntersector16Pluecker>);
    //DEFINE_INTERSECTOR16    (BVH4Triangle4vIntersector16ChunkPluecker, BVH4Intersector16Chunk<Triangle4vIntersector16Pluecker>);
    //DEFINE_INTERSECTOR16    (BVH4Triangle4iIntersector16ChunkPluecker, BVH4Intersector16Chunk<TriangleMeshAccelTriangle4iIntersector16Pluecker>);
    //DEFINE_INTERSECTOR16    (BVH4Triangle4iiIntersector16ChunkPluecker, BVH4Intersector16Chunk<FlatTriangleAccelTriangle4iIntersector16Pluecker>);
    DEFINE_INTERSECTOR16    (BVH4VirtualIntersector16, BVH4Intersector16Chunk<VirtualAccelIntersector16>);
  }
}
