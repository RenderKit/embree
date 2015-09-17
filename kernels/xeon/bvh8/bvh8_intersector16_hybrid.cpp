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

#include "bvh8_intersector16_hybrid.h"
#include "../geometry/triangle4.h"
#include "../geometry/triangle8.h"
#include "../geometry/triangle8v.h"
#include "../geometry/intersector_iterators.h"
#include "../geometry/triangle_intersector_moeller.h"
#include "../geometry/triangle_intersector_pluecker.h"
//#include "../geometry/triangle_intersector_pluecker2.h"

#define DBG(x) 

#define SWITCH_THRESHOLD 7
#define SWITCH_DURING_DOWN_TRAVERSAL 1

namespace embree
{
  namespace isa
  {    

    template<bool robust, typename PrimitiveIntersector16>
    __forceinline void BVH8Intersector16Hybrid<robust, PrimitiveIntersector16>::intersect1(const BVH8* bvh, NodeRef root, const size_t k, Precalculations& pre, Ray16& ray,const Vec3f16 &ray_org, const Vec3f16 &ray_dir, const Vec3f16 &ray_rdir, const float16 &ray_tnear, const float16 &ray_tfar, const Vec3i16& nearXYZ)
    {
      /*! stack state */
      StackItemT<NodeRef> stack[stackSizeSingle];  //!< stack of nodes 
      StackItemT<NodeRef>* stackPtr = stack+1;        //!< current stack pointer
      StackItemT<NodeRef>* stackEnd = stack+stackSizeSingle;
      stack[0].ptr = root;
      stack[0].dist = neg_inf;
      
      /*! offsets to select the side that becomes the lower or upper bound */
      const size_t nearX = nearXYZ.x[k];
      const size_t nearY = nearXYZ.y[k];
      const size_t nearZ = nearXYZ.z[k];

      /*! load the ray into SIMD registers */
      const Vec3f8 org (ray_org .x[k],ray_org .y[k],ray_org .z[k]);
      const Vec3f8 rdir(ray_rdir.x[k],ray_rdir.y[k],ray_rdir.z[k]);
      const Vec3f8 norg = -org, org_rdir(org*rdir);
      float8 rayNear(ray_tnear[k]), rayFar(ray_tfar[k]);
     
/* pop loop */
      while (true) pop:
      {
        /*! pop next node */
        if (unlikely(stackPtr == stack)) break;
        stackPtr--;
        NodeRef cur = NodeRef(stackPtr->ptr);
        
        /*! if popped node is too far, pop next one */
        if (unlikely(*(float*)&stackPtr->dist > ray.tfar[k]))
          continue;
        
        /* downtraversal loop */
        while (true)
        {
          /*! stop if we found a leaf */
          if (unlikely(cur.isLeaf())) break;
          STAT3(normal.trav_nodes,1,1,1);
          
          /*! single ray intersection with 4 boxes */
          const Node* node = cur.node();
          const size_t farX  = nearX ^ sizeof(float8), farY  = nearY ^ sizeof(float8), farZ  = nearZ ^ sizeof(float8);
#if defined (__AVX2__)
          const float8 tNearX = msub(load8f((const char*)node+nearX), rdir.x, org_rdir.x);
          const float8 tNearY = msub(load8f((const char*)node+nearY), rdir.y, org_rdir.y);
          const float8 tNearZ = msub(load8f((const char*)node+nearZ), rdir.z, org_rdir.z);
          const float8 tFarX  = msub(load8f((const char*)node+farX ), rdir.x, org_rdir.x);
          const float8 tFarY  = msub(load8f((const char*)node+farY ), rdir.y, org_rdir.y);
          const float8 tFarZ  = msub(load8f((const char*)node+farZ ), rdir.z, org_rdir.z);
#else
          const float8 tNearX = (norg.x + load8f((const char*)node+nearX)) * rdir.x;
          const float8 tNearY = (norg.y + load8f((const char*)node+nearY)) * rdir.y;
          const float8 tNearZ = (norg.z + load8f((const char*)node+nearZ)) * rdir.z;
          const float8 tFarX  = (norg.x + load8f((const char*)node+farX )) * rdir.x;
          const float8 tFarY  = (norg.y + load8f((const char*)node+farY )) * rdir.y;
          const float8 tFarZ  = (norg.z + load8f((const char*)node+farZ )) * rdir.z;
#endif

          const float round_down = 1.0f-2.0f*float(ulp);
          const float round_up   = 1.0f+2.0f*float(ulp);

          const float8 tNear = max(tNearX,tNearY,tNearZ,rayNear);
          const float8 tFar  = min(tFarX ,tFarY ,tFarZ ,rayFar);
          //const bool8 vmask = tNear <= tFar;
          const bool8 vmask = robust ?  (round_down*tNear <= round_up*tFar) : tNear <= tFar;

          size_t mask = movemask(vmask);
          
          /*! if no child is hit, pop next node */
          if (unlikely(mask == 0))
            goto pop;
          
          /*! one child is hit, continue with that child */
          size_t r = __bscf(mask);
          if (likely(mask == 0)) {
            cur = node->child(r); cur.prefetch();
            assert(cur != BVH8::emptyNode);
            continue;
          }
          
          /*! two children are hit, push far child, and continue with closer child */
          NodeRef c0 = node->child(r); c0.prefetch(); const unsigned int d0 = ((unsigned int*)&tNear)[r];
          r = __bscf(mask);
          NodeRef c1 = node->child(r); c1.prefetch(); const unsigned int d1 = ((unsigned int*)&tNear)[r];
          assert(c0 != BVH8::emptyNode);
          assert(c1 != BVH8::emptyNode);
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
          r = __bscf(mask);
          NodeRef c = node->child(r); c.prefetch(); unsigned int d = ((unsigned int*)&tNear)[r]; stackPtr->ptr = c; stackPtr->dist = d; stackPtr++;
          assert(c != BVH8::emptyNode);
          if (likely(mask == 0)) {
            sort(stackPtr[-1],stackPtr[-2],stackPtr[-3]);
            cur = (NodeRef) stackPtr[-1].ptr; stackPtr--;
            continue;
          }
          
	  /*! four children are hit, push all onto stack and sort 4 stack items, continue with closest child */
          r = __bscf(mask);
          c = node->child(r); c.prefetch(); d = *(unsigned int*)&tNear[r]; stackPtr->ptr = c; stackPtr->dist = d; stackPtr++;
	  if (likely(mask == 0)) {
	    sort(stackPtr[-1],stackPtr[-2],stackPtr[-3],stackPtr[-4]);
	    cur = (NodeRef) stackPtr[-1].ptr; stackPtr--;
	    continue;
	  }

	  /*! fallback case if more than 4 children are hit */
	  while (1)
	  {
	    r = __bscf(mask);
	    assert(stackPtr < stackEnd);
	    c = node->child(r); c.prefetch(); d = *(unsigned int*)&tNear[r]; stackPtr->ptr = c; stackPtr->dist = d; stackPtr++;
	    if (unlikely(mask == 0)) break;
	  }
	  
	  cur = (NodeRef) stackPtr[-1].ptr; stackPtr--;
	}
        
        /*! this is a leaf node */
	assert(cur != BVH8::emptyNode);
        STAT3(normal.trav_leaves,1,1,1);
        size_t num; Triangle* prim = (Triangle*) cur.leaf(num);
        size_t lazy_node = 0;
        PrimitiveIntersector16::intersect(pre,ray,k,prim,num,bvh->scene,lazy_node);
        rayFar = ray.tfar[k];

        if (unlikely(lazy_node)) {
          stackPtr->ptr = lazy_node;
          stackPtr->dist = neg_inf;
          stackPtr++;
        }
      }
    }

    
    template<bool robust, typename PrimitiveIntersector16>    
    void BVH8Intersector16Hybrid<robust,PrimitiveIntersector16>::intersect(int16* valid_i, BVH8* bvh, Ray16& ray)
    {
#if defined(__AVX512F__)      
      /* load ray */
      bool16 valid0 = *valid_i == -1;
#if defined(RTCORE_IGNORE_INVALID_RAYS)
      valid0 &= ray.valid();
#endif
      assert(all(valid0,ray.tnear > -FLT_MIN));
      //assert(!(types & BVH4::FLAG_NODE_MB) || all(valid0,ray.time >= 0.0f & ray.time <= 1.0f));

      const Vec3f16 ray_org = ray.org;
      const Vec3f16 ray_dir = ray.dir;
      const Vec3f16 rdir = rcp_safe(ray.dir);
      const Vec3f16 org_rdir = ray.org * rdir;
      float16 ray_tnear = select(valid0,ray.tnear,pos_inf);
      float16 ray_tfar  = select(valid0,ray.tfar ,neg_inf);
      const float16 inf = float16(pos_inf);
      Precalculations pre(valid0,ray);
      
      /* compute near/far per ray */
      Vec3i16 nearXYZ;
      nearXYZ.x = select(rdir.x >= 0.0f,int16(0*(int)sizeof(float8)),int16(1*(int)sizeof(float8)));
      nearXYZ.y = select(rdir.y >= 0.0f,int16(2*(int)sizeof(float8)),int16(3*(int)sizeof(float8)));
      nearXYZ.z = select(rdir.z >= 0.0f,int16(4*(int)sizeof(float8)),int16(5*(int)sizeof(float8)));

      /* allocate stack and push root node */
      float16 stack_near[stackSizeChunk];
      NodeRef stack_node[stackSizeChunk];
      stack_node[0] = BVH8::invalidNode;
      stack_near[0] = inf;
      stack_node[1] = bvh->root;
      stack_near[1] = ray_tnear; 
      NodeRef* stackEnd = stack_node+stackSizeChunk;
      NodeRef*  __restrict__ sptr_node = stack_node + 2;
      float16*  __restrict__ sptr_near = stack_near + 2;

      while (1) pop:
      {
        /* pop next node from stack */
        sptr_node--;
        sptr_near--;
        NodeRef cur = *sptr_node;
        if (unlikely(cur == BVH8::invalidNode)) 
          break;
        
        /* cull node if behind closest hit point */
        float16 curDist = *sptr_near;
        const bool16 active = curDist < ray_tfar;
        if (unlikely(none(active)))
          continue;

        
        /* switch to single ray traversal */
        size_t bits = movemask(active);
        if (unlikely(__popcnt(bits) <= SWITCH_THRESHOLD)) {
          for (size_t i=__bsf(bits); bits!=0; bits=__btc(bits,i), i=__bsf(bits)) {
            intersect1(bvh,cur,i,pre,ray,ray_org,ray_dir,rdir,ray_tnear,ray_tfar,nearXYZ);
          }
          ray_tfar = ray.tfar;
          continue;
        }
        
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
            //const bool16 lhit   = max(lnearP,ray_tnear) <= min(lfarP,ray_tfar);      

            const float round_down = 1.0f-2.0f*float(ulp);
            const float round_up   = 1.0f+2.0f*float(ulp);

            const bool16 lhit = robust ?  (round_down*max(lnearP,ray_tnear) <= round_up*min(lfarP,ray_tfar)) : max(lnearP,ray_tnear) <= min(lfarP,ray_tfar);
            
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

    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


    template<bool robust, typename PrimitiveIntersector16>
    __forceinline bool BVH8Intersector16Hybrid<robust, PrimitiveIntersector16>::occluded1(const BVH8* bvh, NodeRef root, const size_t k, Precalculations& pre, Ray16& ray,const Vec3f16 &ray_org, const Vec3f16 &ray_dir, const Vec3f16 &ray_rdir, const float16 &ray_tnear, const float16 &ray_tfar, const Vec3i16& nearXYZ)
    {
      /*! stack state */
      NodeRef stack[stackSizeSingle];  //!< stack of nodes that still need to get traversed
      NodeRef* stackPtr = stack+1;        //!< current stack pointer
      NodeRef* stackEnd = stack+stackSizeSingle;
      stack[0]  = root;
      
      /*! offsets to select the side that becomes the lower or upper bound */
      const size_t nearX = nearXYZ.x[k];
      const size_t nearY = nearXYZ.y[k];
      const size_t nearZ = nearXYZ.z[k];
      
      /*! load the ray into SIMD registers */
      const Vec3f8 org (ray_org .x[k],ray_org .y[k],ray_org .z[k]);
      const Vec3f8 rdir(ray_rdir.x[k],ray_rdir.y[k],ray_rdir.z[k]);
      const Vec3f8 norg = -org, org_rdir(org*rdir);
      const float8 rayNear(ray_tnear[k]), rayFar(ray_tfar[k]); 

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
          const size_t farX  = nearX ^ sizeof(float8), farY  = nearY ^ sizeof(float8), farZ  = nearZ ^ sizeof(float8);
#if defined (__AVX2__)
          const float8 tNearX = msub(load8f((const char*)node+nearX), rdir.x, org_rdir.x);
          const float8 tNearY = msub(load8f((const char*)node+nearY), rdir.y, org_rdir.y);
          const float8 tNearZ = msub(load8f((const char*)node+nearZ), rdir.z, org_rdir.z);
          const float8 tFarX  = msub(load8f((const char*)node+farX ), rdir.x, org_rdir.x);
          const float8 tFarY  = msub(load8f((const char*)node+farY ), rdir.y, org_rdir.y);
          const float8 tFarZ  = msub(load8f((const char*)node+farZ ), rdir.z, org_rdir.z);
#else
          const float8 tNearX = (norg.x + load8f((const char*)node+nearX)) * rdir.x;
          const float8 tNearY = (norg.y + load8f((const char*)node+nearY)) * rdir.y;
          const float8 tNearZ = (norg.z + load8f((const char*)node+nearZ)) * rdir.z;
          const float8 tFarX  = (norg.x + load8f((const char*)node+farX )) * rdir.x;
          const float8 tFarY  = (norg.y + load8f((const char*)node+farY )) * rdir.y;
          const float8 tFarZ  = (norg.z + load8f((const char*)node+farZ )) * rdir.z;
#endif
          
          const float8 tNear = max(tNearX,tNearY,tNearZ,rayNear);
          const float8 tFar  = min(tFarX ,tFarY ,tFarZ ,rayFar);
          const float round_down = 1.0f-2.0f*float(ulp);
          const float round_up   = 1.0f+2.0f*float(ulp);

          //const bool8 vmask = tNear <= tFar;
          const bool8 vmask = robust ?  (round_down*tNear <= round_up*tFar) : tNear <= tFar;

          size_t mask = movemask(vmask);
          
          /*! if no child is hit, pop next node */
          if (unlikely(mask == 0))
            goto pop;
          
          /*! one child is hit, continue with that child */
          size_t r = __bscf(mask);
          if (likely(mask == 0)) {
            cur = node->child(r); cur.prefetch(); 
            assert(cur != BVH8::emptyNode);
            continue;
          }
          
          /*! two children are hit, push far child, and continue with closer child */
          NodeRef c0 = node->child(r); c0.prefetch(); const unsigned int d0 = ((unsigned int*)&tNear)[r];
          r = __bscf(mask);
          NodeRef c1 = node->child(r); c1.prefetch(); const unsigned int d1 = ((unsigned int*)&tNear)[r];
          assert(c0 != BVH8::emptyNode);
          assert(c1 != BVH8::emptyNode);
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
          r = __bscf(mask);
          cur = node->child(r); cur.prefetch(); *stackPtr = cur; stackPtr++;
          if (likely(mask == 0)) {
            stackPtr--;
            continue;
          }

	  /*! process more than three children */
	  while(1)
	  {
	    r = __bscf(mask);
	    NodeRef c = node->child(r); c.prefetch(); *stackPtr = c; stackPtr++;
	    if (unlikely(mask == 0)) break;
	  }
	  cur = (NodeRef) stackPtr[-1]; stackPtr--;
        }
        
        /*! this is a leaf node */
	assert(cur != BVH8::emptyNode);
        STAT3(shadow.trav_leaves,1,1,1);
        size_t num; Triangle* prim = (Triangle*) cur.leaf(num);

        size_t lazy_node = 0;
        if (PrimitiveIntersector16::occluded(pre,ray,k,prim,num,bvh->scene,lazy_node)) {
          //ray.geomID = 0;
          //break;
	  return true;
        }

        if (unlikely(lazy_node)) {
          stackPtr->ptr = lazy_node;
          stackPtr->dist = neg_inf;
          stackPtr++;
        }
      }
      return false;
    }

    
     template<bool robust, typename PrimitiveIntersector16>
     void BVH8Intersector16Hybrid<robust, PrimitiveIntersector16>::occluded(int16* valid_i, BVH8* bvh, Ray16& ray)
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
      
      while (1) pop:
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

            const float round_down = 1.0f-2.0f*float(ulp);
            const float round_up   = 1.0f+2.0f*float(ulp);

            //const bool16 lhit   = max(lnearP,ray_tnear) <= min(lfarP,ray_tfar);      
            const bool16 lhit = robust ?  (round_down*max(lnearP,ray_tnear) <= round_up*min(lfarP,ray_tfar)) : max(lnearP,ray_tnear) <= min(lfarP,ray_tfar);
            
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
    
    DEFINE_INTERSECTOR8(BVH8Triangle4Intersector16HybridMoeller,BVH8Intersector16Hybrid<false COMMA ArrayIntersector16<TriangleNIntersectorMMoellerTrumbore<Ray16 COMMA Triangle4 COMMA true> > >);

    DEFINE_INTERSECTOR8(BVH8Triangle4Intersector16HybridMoellerNoFilter,BVH8Intersector16Hybrid<false COMMA ArrayIntersector16<TriangleNIntersectorMMoellerTrumbore<Ray16 COMMA Triangle4 COMMA false> > >);

    DEFINE_INTERSECTOR8(BVH8Triangle8Intersector16HybridMoeller,BVH8Intersector16Hybrid<false COMMA ArrayIntersector16<TriangleNIntersectorMMoellerTrumbore<Ray16 COMMA Triangle8 COMMA true> > >);

    DEFINE_INTERSECTOR8(BVH8Triangle8Intersector16HybridMoellerNoFilter,BVH8Intersector16Hybrid<false COMMA ArrayIntersector16<TriangleNIntersectorMMoellerTrumbore<Ray16 COMMA Triangle8 COMMA false> > >);

    //DEFINE_INTERSECTOR8(BVH8Triangle8vIntersector16HybridPluecker, BVH8Intersector16Hybrid<true COMMA ArrayIntersector16_1<TriangleNvIntersectorMPluecker2<Ray16 COMMA Triangle8v COMMA true> > >);
    //DEFINE_INTERSECTOR8(BVH8Triangle8vIntersector16HybridPlueckerNoFilter, BVH8Intersector16Hybrid<true COMMA ArrayIntersector16_1<TriangleNvIntersectorMPluecker2<Ray16 COMMA Triangle8v COMMA false> > >);

  }
}  
