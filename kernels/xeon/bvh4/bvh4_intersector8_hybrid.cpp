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

#include "bvh4_intersector8_hybrid.h"

#include "geometry/triangle4_intersector8_moeller.h"
#include "geometry/triangle8_intersector8_moeller.h"
#include "geometry/triangle4v_intersector8_pluecker.h"

#define SWITCH_THRESHOLD 3

namespace embree
{
  namespace isa
  {
    template<typename PrimitiveIntersector8>
    __forceinline void BVH4Intersector8Hybrid<PrimitiveIntersector8>::intersect1(const BVH4* bvh, NodeRef root, size_t k, Ray8& ray, 
                                                                                 avx3f ray_org, avx3f ray_dir, avx3f ray_rdir, avxf ray_tnear, avxf ray_tfar)
    {
      /*! stack state */
      StackItem stack[stackSizeSingle];  //!< stack of nodes 
      StackItem* stackPtr = stack+1;        //!< current stack pointer
      StackItem* stackEnd = stack+stackSizeSingle;
      stack[0].ptr = root;
      stack[0].dist = neg_inf;
      
      /*! offsets to select the side that becomes the lower or upper bound */
      const size_t nearX = ray_dir.x[k] >= 0.0f ? 0*sizeof(ssef) : 1*sizeof(ssef);
      const size_t nearY = ray_dir.y[k] >= 0.0f ? 2*sizeof(ssef) : 3*sizeof(ssef);
      const size_t nearZ = ray_dir.z[k] >= 0.0f ? 4*sizeof(ssef) : 5*sizeof(ssef);
      
      /*! load the ray into SIMD registers */
      const sse3f org (ray_org .x[k],ray_org .y[k],ray_org .z[k]);
      const sse3f rdir(ray_rdir.x[k],ray_rdir.y[k],ray_rdir.z[k]);
      const sse3f org_rdir(org*rdir);
      ssef rayNear(ray_tnear[k]), rayFar(ray_tfar[k]);
      
      /* pop loop */
      while (true) pop:
      {
        /*! pop next node */
        if (unlikely(stackPtr == stack)) break;
        stackPtr--;
        NodeRef cur = NodeRef(stackPtr->ptr);
        
        /*! if popped node is too far, pop next one */
        if (unlikely(stackPtr->dist > ray.tfar[k]))
          continue;
        
        /* downtraversal loop */
        while (true)
        {
          /*! stop if we found a leaf */
          if (unlikely(cur.isLeaf())) break;
          STAT3(normal.trav_nodes,1,1,1);
          
          /*! single ray intersection with 4 boxes */
          const Node* node = cur.node();
          const size_t farX  = nearX ^ 16, farY  = nearY ^ 16, farZ  = nearZ ^ 16;
#if defined (__AVX2__)
          const ssef tNearX = msub(load4f((const char*)node+nearX), rdir.x, org_rdir.x);
          const ssef tNearY = msub(load4f((const char*)node+nearY), rdir.y, org_rdir.y);
          const ssef tNearZ = msub(load4f((const char*)node+nearZ), rdir.z, org_rdir.z);
          const ssef tFarX  = msub(load4f((const char*)node+farX ), rdir.x, org_rdir.x);
          const ssef tFarY  = msub(load4f((const char*)node+farY ), rdir.y, org_rdir.y);
          const ssef tFarZ  = msub(load4f((const char*)node+farZ ), rdir.z, org_rdir.z);
#else
          const ssef tNearX = (load4f((const char*)node+nearX) - org.x) * rdir.x;
          const ssef tNearY = (load4f((const char*)node+nearY) - org.y) * rdir.y;
          const ssef tNearZ = (load4f((const char*)node+nearZ) - org.z) * rdir.z;
          const ssef tFarX  = (load4f((const char*)node+farX ) - org.x) * rdir.x;
          const ssef tFarY  = (load4f((const char*)node+farY ) - org.y) * rdir.y;
          const ssef tFarZ  = (load4f((const char*)node+farZ ) - org.z) * rdir.z;
#endif

#if defined(__SSE4_1__)
          const ssef tNear = maxi(maxi(tNearX,tNearY),maxi(tNearZ,rayNear));
          const ssef tFar  = mini(mini(tFarX ,tFarY ),mini(tFarZ ,rayFar ));
          const sseb vmask = cast(tNear) > cast(tFar);
          size_t mask = movemask(vmask)^0xf;
#else
          const ssef tNear = max(tNearX,tNearY,tNearZ,rayNear);
          const ssef tFar  = min(tFarX ,tFarY ,tFarZ ,rayFar);
          const sseb vmask = tNear <= tFar;
          size_t mask = movemask(vmask);
#endif
          
          /*! if no child is hit, pop next node */
          if (unlikely(mask == 0))
            goto pop;
          
          /*! one child is hit, continue with that child */
          size_t r = bitscan(mask); mask = __btc(mask,r);
          if (likely(mask == 0)) {
            cur = node->child(r);
            assert(cur != BVH4::emptyNode);
            continue;
          }
          
          /*! two children are hit, push far child, and continue with closer child */
          NodeRef c0 = node->child(r); const float d0 = tNear[r];
          r = bitscan(mask); mask = __btc(mask,r);
          NodeRef c1 = node->child(r); const float d1 = tNear[r];
          assert(c0 != BVH4::emptyNode);
          assert(c1 != BVH4::emptyNode);
          if (likely(mask == 0)) {
            assert(stackPtr < stackEnd); 
            if (d0 < d1) { stackPtr->ptr = c1; stackPtr->dist = d1; stackPtr++; cur = c0; continue; }
            else         { stackPtr->ptr = c0; stackPtr->dist = d0; stackPtr++; cur = c1; continue; }
          }
          
          /*! Here starts the slow path for 3 or 4 hit children. We push
           *  all nodes onto the stack to sort them there. */
          assert(stackPtr < stackEnd); 
          stackPtr->ptr = c0; stackPtr->dist = d0; stackPtr++;
          assert(stackPtr < stackEnd); 
          stackPtr->ptr = c1; stackPtr->dist = d1; stackPtr++;
          
          /*! three children are hit, push all onto stack and sort 3 stack items, continue with closest child */
          assert(stackPtr < stackEnd); 
          r = bitscan(mask); mask = __btc(mask,r);
          NodeRef c = node->child(r); float d = tNear[r]; stackPtr->ptr = c; stackPtr->dist = d; stackPtr++;
          assert(c0 != BVH4::emptyNode);
          if (likely(mask == 0)) {
            sort(stackPtr[-1],stackPtr[-2],stackPtr[-3]);
            cur = (NodeRef) stackPtr[-1].ptr; stackPtr--;
            continue;
          }
          
          /*! four children are hit, push all onto stack and sort 4 stack items, continue with closest child */
          assert(stackPtr < stackEnd); 
          r = bitscan(mask); mask = __btc(mask,r);
          c = node->child(r); d = tNear[r]; stackPtr->ptr = c; stackPtr->dist = d; stackPtr++;
          assert(c != BVH4::emptyNode);
          sort(stackPtr[-1],stackPtr[-2],stackPtr[-3],stackPtr[-4]);
          cur = (NodeRef) stackPtr[-1].ptr; stackPtr--;
        }
        
        /*! this is a leaf node */
        STAT3(normal.trav_leaves,1,1,1);
        size_t num; Primitive* prim = (Primitive*) cur.leaf(num);
        PrimitiveIntersector8::intersect(ray,k,prim,num,bvh->geometry);
        rayFar = ray.tfar[k];
      }
    }
    
    template<typename PrimitiveIntersector8>
    void BVH4Intersector8Hybrid<PrimitiveIntersector8>::intersect(avxb* valid_i, BVH4* bvh, Ray8& ray)
    {
      /* load ray */
      const avxb valid0 = *valid_i;
      avx3f ray_org = ray.org, ray_dir = ray.dir;
      avxf ray_tnear = ray.tnear, ray_tfar  = ray.tfar;
#if defined(__FIX_RAYS__)
      const avxf float_range = 1.8E19;
      ray_org = clamp(ray_org,avx3f(-float_range),avx3f(+float_range));
      ray_dir = clamp(ray_dir,avx3f(-float_range),avx3f(+float_range));
      ray_tnear = max(ray_tnear,FLT_MIN); 
      ray_tfar  = min(ray_tfar,float(inf)); 
#endif
      const avx3f rdir = rcp_safe(ray_dir);
      const avx3f org(ray_org), org_rdir = org * rdir;
      ray_tnear = select(valid0,ray_tnear,avxf(pos_inf));
      ray_tfar  = select(valid0,ray_tfar ,avxf(neg_inf));
      const avxf inf = avxf(pos_inf);

      /* allocate stack and push root node */
      avxf    stack_near[stackSizeChunk];
      NodeRef stack_node[stackSizeChunk];
      stack_node[0] = BVH4::invalidNode;
      stack_near[0] = inf;
      stack_node[1] = bvh->root;
      stack_near[1] = ray_tnear; 
      NodeRef* stackEnd = stack_node+stackSizeChunk;
      NodeRef* __restrict__ sptr_node = stack_node + 2;
      avxf*    __restrict__ sptr_near = stack_near + 2;
      
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
        avxf curDist = *sptr_near;
        const avxb active = curDist < ray_tfar;
        if (unlikely(none(active)))
          continue;
        
        /* switch to single ray traversal */
        size_t bits = movemask(active);
        if (unlikely(__popcnt(bits) <= SWITCH_THRESHOLD)) {
          for (size_t i=__bsf(bits); bits!=0; bits=__btc(bits,i), i=__bsf(bits)) {
            intersect1(bvh,curNode,i,ray,ray_org,ray_dir,rdir,ray_tnear,ray_tfar);
          }
          ray_tfar = ray.tfar;
          continue;
        }
        
        while (1)
        {
          /* test if this is a leaf node */
          if (unlikely(curNode.isLeaf()))
            break;
          
          const avxb valid_node = ray_tfar > curDist;
          STAT3(normal.trav_nodes,1,popcnt(valid_node),8);
          const Node* __restrict__ const node = curNode.node();
          
          /* pop of next node */
          assert(sptr_node > stack_node);
          sptr_node--;
          sptr_near--;
          curNode = *sptr_node;
          curDist = *sptr_near;
          
#pragma unroll(4)
          for (unsigned i=0; i<4; i++)
          {
            const NodeRef child = node->children[i];
            if (unlikely(child == BVH4::emptyNode)) break;
            
#if defined(__AVX2__)
            const avxf lclipMinX = msub(node->lower_x[i],rdir.x,org_rdir.x);
            const avxf lclipMinY = msub(node->lower_y[i],rdir.y,org_rdir.y);
            const avxf lclipMinZ = msub(node->lower_z[i],rdir.z,org_rdir.z);
            const avxf lclipMaxX = msub(node->upper_x[i],rdir.x,org_rdir.x);
            const avxf lclipMaxY = msub(node->upper_y[i],rdir.y,org_rdir.y);
            const avxf lclipMaxZ = msub(node->upper_z[i],rdir.z,org_rdir.z);
            const avxf lnearP = maxi(maxi(mini(lclipMinX, lclipMaxX), mini(lclipMinY, lclipMaxY)), mini(lclipMinZ, lclipMaxZ));
            const avxf lfarP  = mini(mini(maxi(lclipMinX, lclipMaxX), maxi(lclipMinY, lclipMaxY)), maxi(lclipMinZ, lclipMaxZ));
            const avxb lhit   = maxi(lnearP,ray_tnear) <= mini(lfarP,ray_tfar);      
#else
            const avxf lclipMinX = (node->lower_x[i] - org.x) * rdir.x;
            const avxf lclipMinY = (node->lower_y[i] - org.y) * rdir.y;
            const avxf lclipMinZ = (node->lower_z[i] - org.z) * rdir.z;
            const avxf lclipMaxX = (node->upper_x[i] - org.x) * rdir.x;
            const avxf lclipMaxY = (node->upper_y[i] - org.y) * rdir.y;
            const avxf lclipMaxZ = (node->upper_z[i] - org.z) * rdir.z;
            const avxf lnearP = max(max(min(lclipMinX, lclipMaxX), min(lclipMinY, lclipMaxY)), min(lclipMinZ, lclipMaxZ));
            const avxf lfarP  = min(min(max(lclipMinX, lclipMaxX), max(lclipMinY, lclipMaxY)), max(lclipMinZ, lclipMaxZ));
            const avxb lhit   = max(lnearP,ray_tnear) <= min(lfarP,ray_tfar);      
#endif
            
            /* if we hit the child we choose to continue with that child if it 
               is closer than the current next child, or we push it onto the stack */
            if (likely(any(lhit)))
            {
              assert(sptr_node < stackEnd);
              const avxf childDist = select(lhit,lnearP,inf);
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
        
        /* return if stack is empty */
        if (unlikely(curNode == BVH4::invalidNode)) {
          assert(sptr_node == stack_node);
          break;
        }
        
        /* intersect leaf */
        const avxb valid_leaf = ray_tfar > curDist;
        STAT3(normal.trav_leaves,1,popcnt(valid_leaf),8);
        size_t items; const Primitive* prim = (Primitive*) curNode.leaf(items);
        PrimitiveIntersector8::intersect(valid_leaf,ray,prim,items,bvh->geometry);
        ray_tfar = select(valid_leaf,ray.tfar,ray_tfar);
      }
      AVX_ZERO_UPPER();
    }

    template<typename PrimitiveIntersector8>
    __forceinline bool BVH4Intersector8Hybrid<PrimitiveIntersector8>::occluded1(const BVH4* bvh, NodeRef root, size_t k, Ray8& ray, 
                                                                                avx3f ray_org, avx3f ray_dir, avx3f ray_rdir, avxf ray_tnear, avxf ray_tfar)
    {
      /*! stack state */
      NodeRef stack[stackSizeSingle];  //!< stack of nodes that still need to get traversed
      NodeRef* stackPtr = stack+1;        //!< current stack pointer
      NodeRef* stackEnd = stack+stackSizeSingle;
      stack[0]  = root;
      
      /*! offsets to select the side that becomes the lower or upper bound */
      const size_t nearX = ray_dir.x[k] >= 0.0f ? 0*sizeof(ssef) : 1*sizeof(ssef);
      const size_t nearY = ray_dir.y[k] >= 0.0f ? 2*sizeof(ssef) : 3*sizeof(ssef);
      const size_t nearZ = ray_dir.z[k] >= 0.0f ? 4*sizeof(ssef) : 5*sizeof(ssef);
      
      /*! load the ray into SIMD registers */
      const sse3f org (ray_org .x[k],ray_org .y[k],ray_org .z[k]);
      const sse3f rdir(ray_rdir.x[k],ray_rdir.y[k],ray_rdir.z[k]);
      const sse3f norg = -org, org_rdir(org*rdir);
      const ssef rayNear(ray_tnear[k]), rayFar(ray_tfar[k]); 
      
      /* pop loop */
      while (true) pop:
      {
        /*! pop next node */
        if (unlikely(stackPtr == stack)) break;
        stackPtr--;
        NodeRef cur = (NodeRef) *stackPtr;
        
        /* downtraversal loop */
        while (true)
        {
          /*! stop if we found a leaf */
          if (unlikely(cur.isLeaf())) break;
          STAT3(shadow.trav_nodes,1,1,1);
          
          /*! single ray intersection with 4 boxes */
          const Node* node = cur.node();
          const size_t farX  = nearX ^ 16, farY  = nearY ^ 16, farZ  = nearZ ^ 16;
#if defined (__AVX2__)
          const ssef tNearX = msub(load4f((const char*)node+nearX), rdir.x, org_rdir.x);
          const ssef tNearY = msub(load4f((const char*)node+nearY), rdir.y, org_rdir.y);
          const ssef tNearZ = msub(load4f((const char*)node+nearZ), rdir.z, org_rdir.z);
          const ssef tFarX  = msub(load4f((const char*)node+farX ), rdir.x, org_rdir.x);
          const ssef tFarY  = msub(load4f((const char*)node+farY ), rdir.y, org_rdir.y);
          const ssef tFarZ  = msub(load4f((const char*)node+farZ ), rdir.z, org_rdir.z);
#else
          const ssef tNearX = (norg.x + load4f((const char*)node+nearX)) * rdir.x;
          const ssef tNearY = (norg.y + load4f((const char*)node+nearY)) * rdir.y;
          const ssef tNearZ = (norg.z + load4f((const char*)node+nearZ)) * rdir.z;
          const ssef tFarX  = (norg.x + load4f((const char*)node+farX )) * rdir.x;
          const ssef tFarY  = (norg.y + load4f((const char*)node+farY )) * rdir.y;
          const ssef tFarZ  = (norg.z + load4f((const char*)node+farZ )) * rdir.z;
#endif
          
#if defined(__SSE4_1__)
          const ssef tNear = maxi(maxi(tNearX,tNearY),maxi(tNearZ,rayNear));
          const ssef tFar  = mini(mini(tFarX ,tFarY ),mini(tFarZ ,rayFar ));
          const sseb vmask = cast(tNear) > cast(tFar);
          size_t mask = movemask(vmask)^0xf;
#else
          const ssef tNear = max(tNearX,tNearY,tNearZ,rayNear);
          const ssef tFar  = min(tFarX ,tFarY ,tFarZ ,rayFar);
          const sseb vmask = tNear <= tFar;
          size_t mask = movemask(vmask);
#endif
          
          /*! if no child is hit, pop next node */
          if (unlikely(mask == 0))
            goto pop;
          
          /*! one child is hit, continue with that child */
          size_t r = bitscan(mask); mask = __btc(mask,r);
          if (likely(mask == 0)) {
            cur = node->child(r);
            assert(cur != BVH4::emptyNode);
            continue;
          }
          
          /*! two children are hit, push far child, and continue with closer child */
          NodeRef c0 = node->child(r); const float d0 = tNear[r];
          r = bitscan(mask); mask = __btc(mask,r);
          NodeRef c1 = node->child(r); const float d1 = tNear[r];
          assert(c0 != BVH4::emptyNode);
          assert(c1 != BVH4::emptyNode);
          if (likely(mask == 0)) {
            assert(stackPtr < stackEnd);
            if (d0 < d1) { *stackPtr = c1; stackPtr++; cur = c0; continue; }
            else         { *stackPtr = c0; stackPtr++; cur = c1; continue; }
          }
          assert(stackPtr < stackEnd);
          *stackPtr = c0; stackPtr++;
          assert(stackPtr < stackEnd);
          *stackPtr = c1; stackPtr++;
          
          /*! three children are hit */
          r = bitscan(mask); mask = __btc(mask,r); cur = node->child(r); 
          assert(cur != BVH4::emptyNode);
          if (likely(mask == 0)) continue;
          assert(stackPtr < stackEnd);
          *stackPtr = cur; stackPtr++;
          
          /*! four children are hit */
          cur = node->child(3);
          assert(cur != BVH4::emptyNode);
        }
        
        /*! this is a leaf node */
        STAT3(shadow.trav_leaves,1,1,1);
        size_t num; Primitive* prim = (Primitive*) cur.leaf(num);
        if (PrimitiveIntersector8::occluded(ray,k,prim,num,bvh->geometry)) {
          ray.geomID[k] = 0;
          return true;
        }
      }
      return false;
    }
    
    template<typename PrimitiveIntersector8>
    void BVH4Intersector8Hybrid<PrimitiveIntersector8>::occluded(avxb* valid_i, BVH4* bvh, Ray8& ray)
    {
      /* load ray */
      const avxb valid = *valid_i;
      avxb terminated = !valid;
      avx3f ray_org = ray.org, ray_dir = ray.dir;
      avxf ray_tnear = ray.tnear, ray_tfar  = ray.tfar;
#if defined(__FIX_RAYS__)
      const avxf float_range = 0.1f*FLT_MAX;
      ray_org = clamp(ray_org,avx3f(-float_range),avx3f(+float_range));
      ray_dir = clamp(ray_dir,avx3f(-float_range),avx3f(+float_range));
      ray_tnear = max(ray_tnear,FLT_MIN); 
      ray_tfar  = min(ray_tfar,float(inf)); 
#endif
      const avx3f rdir = rcp_safe(ray_dir);
      const avx3f org(ray_org), org_rdir = org * rdir;
      ray_tnear = select(valid,ray_tnear,avxf(pos_inf));
      ray_tfar  = select(valid,ray_tfar ,avxf(neg_inf));
      const avxf inf = avxf(pos_inf);
      
      /* allocate stack and push root node */
      avxf    stack_near[stackSizeChunk];
      NodeRef stack_node[stackSizeChunk];
      stack_node[0] = BVH4::invalidNode;
      stack_near[0] = inf;
      stack_node[1] = bvh->root;
      stack_near[1] = ray_tnear; 
      NodeRef* stackEnd = stack_node+stackSizeChunk;
      NodeRef* __restrict__ sptr_node = stack_node + 2;
      avxf*    __restrict__ sptr_near = stack_near + 2;
      
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
        avxf curDist = *sptr_near;
        const avxb active = curDist < ray_tfar;
        if (unlikely(none(active))) 
          continue;
        
        /* switch to single ray traversal */
        size_t bits = movemask(active);
        if (unlikely(__popcnt(bits) <= SWITCH_THRESHOLD)) {
          for (size_t i=__bsf(bits); bits!=0; bits=__btc(bits,i), i=__bsf(bits)) {
            if (occluded1(bvh,curNode,i,ray,ray_org,ray_dir,rdir,ray_tnear,ray_tfar))
              terminated[i] = -1;
          }
          if (all(terminated)) break;
          ray_tfar = select(terminated,avxf(neg_inf),ray_tfar);
          continue;
        }
                
        while (1)
        {
          /* test if this is a leaf node */
          if (unlikely(curNode.isLeaf()))
            break;
          
          const avxb valid_node = ray_tfar > curDist;
          STAT3(shadow.trav_nodes,1,popcnt(valid_node),8);
          const Node* __restrict__ const node = curNode.node();
          
          /* pop of next node */
          assert(sptr_node > stack_node);
          sptr_node--;
          sptr_near--;
          curNode = *sptr_node;
          curDist = *sptr_near;
          
#pragma unroll(4)
          for (unsigned i=0; i<4; i++)
          {
            const NodeRef child = node->children[i];
            if (unlikely(child == BVH4::emptyNode)) break;
            
#if defined(__AVX2__)
            const avxf lclipMinX = msub(node->lower_x[i],rdir.x,org_rdir.x);
            const avxf lclipMinY = msub(node->lower_y[i],rdir.y,org_rdir.y);
            const avxf lclipMinZ = msub(node->lower_z[i],rdir.z,org_rdir.z);
            const avxf lclipMaxX = msub(node->upper_x[i],rdir.x,org_rdir.x);
            const avxf lclipMaxY = msub(node->upper_y[i],rdir.y,org_rdir.y);
            const avxf lclipMaxZ = msub(node->upper_z[i],rdir.z,org_rdir.z);
            const avxf lnearP = maxi(maxi(mini(lclipMinX, lclipMaxX), mini(lclipMinY, lclipMaxY)), mini(lclipMinZ, lclipMaxZ));
            const avxf lfarP  = mini(mini(maxi(lclipMinX, lclipMaxX), maxi(lclipMinY, lclipMaxY)), maxi(lclipMinZ, lclipMaxZ));
            const avxb lhit   = maxi(lnearP,ray_tnear) <= mini(lfarP,ray_tfar);      
#else
            const avxf lclipMinX = (node->lower_x[i] - org.x) * rdir.x;
            const avxf lclipMinY = (node->lower_y[i] - org.y) * rdir.y;
            const avxf lclipMinZ = (node->lower_z[i] - org.z) * rdir.z;
            const avxf lclipMaxX = (node->upper_x[i] - org.x) * rdir.x;
            const avxf lclipMaxY = (node->upper_y[i] - org.y) * rdir.y;
            const avxf lclipMaxZ = (node->upper_z[i] - org.z) * rdir.z;
            const avxf lnearP = max(max(min(lclipMinX, lclipMaxX), min(lclipMinY, lclipMaxY)), min(lclipMinZ, lclipMaxZ));
            const avxf lfarP  = min(min(max(lclipMinX, lclipMaxX), max(lclipMinY, lclipMaxY)), max(lclipMinZ, lclipMaxZ));
            const avxb lhit   = max(lnearP,ray_tnear) <= min(lfarP,ray_tfar);      
#endif
            
            /* if we hit the child we choose to continue with that child if it 
               is closer than the current next child, or we push it onto the stack */
            if (likely(any(lhit)))
            {
              assert(sptr_node < stackEnd);
              assert(child != BVH4::emptyNode);
              const avxf childDist = select(lhit,lnearP,inf);
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
        
        /* return if stack is empty */
        if (unlikely(curNode == BVH4::invalidNode)) {
          assert(sptr_node == stack_node);
          break;
        }
        
        /* intersect leaf */
        const avxb valid_leaf = ray_tfar > curDist;
        STAT3(shadow.trav_leaves,1,popcnt(valid_leaf),8);
        size_t items; const Primitive* prim = (Primitive*) curNode.leaf(items);
        terminated |= PrimitiveIntersector8::occluded(!terminated,ray,prim,items,bvh->geometry);
        if (all(terminated)) break;
        ray_tfar = select(terminated,avxf(neg_inf),ray_tfar);
      }
      store8i(valid & terminated,&ray.geomID,0);
      AVX_ZERO_UPPER();
    }
    
    DEFINE_INTERSECTOR8(BVH4Triangle4Intersector8HybridMoeller, BVH4Intersector8Hybrid<Triangle4Intersector8MoellerTrumbore>);
    DEFINE_INTERSECTOR8(BVH4Triangle8Intersector8HybridMoeller, BVH4Intersector8Hybrid<Triangle8Intersector8MoellerTrumbore>);
    DEFINE_INTERSECTOR8(BVH4Triangle4vIntersector8HybridPluecker, BVH4Intersector8Hybrid<Triangle4vIntersector8Pluecker>);

    DEFINE_INTERSECTOR8TO16(BVH4Triangle4Intersector16HybridMoeller, BVH4Intersector8Hybrid<Triangle4Intersector8MoellerTrumbore>,BVH4);
    DEFINE_INTERSECTOR8TO16(BVH4Triangle8Intersector16HybridMoeller, BVH4Intersector8Hybrid<Triangle8Intersector8MoellerTrumbore>,BVH4);
    DEFINE_INTERSECTOR8TO16(BVH4Triangle4vIntersector16HybridPluecker, BVH4Intersector8Hybrid<Triangle4vIntersector8Pluecker>,BVH4);
  }
}
