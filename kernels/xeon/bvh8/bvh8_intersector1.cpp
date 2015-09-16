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

#include "bvh8_intersector1.h"
#include "../geometry/triangle4.h"
#include "../geometry/triangle8.h"
#include "../geometry/triangle8v.h"
#include "../geometry/trianglepairs8.h"
#include "../geometry/intersector_iterators.h"
#include "../geometry/triangle_intersector_moeller.h"
#include "../geometry/triangle_intersector_pluecker.h"
//#include "../geometry/triangle_intersector_pluecker2.h"

namespace embree
{ 
  namespace isa
  {
    template<bool robust, typename PrimitiveIntersector>
    void BVH8Intersector1<robust,PrimitiveIntersector>::intersect(const BVH8* bvh, Ray& ray)
    {
      /*! perform per ray precalculations required by the primitive intersector */
      Precalculations pre(ray,bvh);

      /*! stack state */
      StackItemT<NodeRef> stack[stackSize];  //!< stack of nodes 
      StackItemT<NodeRef>* stackPtr = stack+1;        //!< current stack pointer
      StackItemT<NodeRef>* stackEnd = stack+stackSize;
      stack[0].ptr = bvh->root;
      stack[0].dist = neg_inf;

      /* filter out invalid rays */
#if defined(RTCORE_IGNORE_INVALID_RAYS)
      if (!ray.valid()) return;
#endif

      /* verify correct input */
      assert(ray.tnear > -FLT_MIN);
      //assert(!(types & BVH4::FLAG_NODE_MB) || (ray.time >= 0.0f && ray.time <= 1.0f));

      /*! load the ray into SIMD registers */
      const Vec3f8 norg(-ray.org.x,-ray.org.y,-ray.org.z);
      const Vec3fa ray_rdir = rcp_safe(ray.dir);
      const Vec3f8 rdir(ray_rdir.x,ray_rdir.y,ray_rdir.z);
      const Vec3fa ray_org_rdir = ray.org*ray_rdir;
      const Vec3f8 org_rdir(ray_org_rdir.x,ray_org_rdir.y,ray_org_rdir.z);
      const float8  ray_near(ray.tnear);
      float8 ray_far(ray.tfar);

      /*! offsets to select the side that becomes the lower or upper bound */
      const size_t nearX = ray_rdir.x >= 0.0f ? 0*sizeof(float8) : 1*sizeof(float8);
      const size_t nearY = ray_rdir.y >= 0.0f ? 2*sizeof(float8) : 3*sizeof(float8);
      const size_t nearZ = ray_rdir.z >= 0.0f ? 4*sizeof(float8) : 5*sizeof(float8);
      
      /* pop loop */
      while (true) pop:
      {
        /*! pop next node */
        if (unlikely(stackPtr == stack)) break;
        stackPtr--;
        NodeRef cur = NodeRef(stackPtr->ptr);
        
        /*! if popped node is too far, pop next one */
        if (unlikely(*(float*)&stackPtr->dist > ray.tfar))
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

#if defined(__AVX2__)
          const float8 tNear = maxi(maxi(tNearX,tNearY),maxi(tNearZ,ray_near));
          const float8 tFar  = mini(mini(tFarX ,tFarY ),mini(tFarZ ,ray_far ));
          const bool8 vmask = robust ?  (round_down*tNear > round_up*tFar) : cast(tNear) > cast(tFar);
          size_t mask = movemask(vmask)^0xff;
#else
          const float8 tNear = max(tNearX,tNearY,tNearZ,ray_near);
          const float8 tFar  = min(tFarX ,tFarY ,tFarZ ,ray_far);
          const bool8 vmask = robust ?  (round_down*tNear > round_up*tFar) : tNear <= tFar;
          size_t mask = movemask(vmask);
#endif
          
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
        size_t num; Primitive* prim = (Primitive*) cur.leaf(num);
        size_t lazy_node = 0;
        PrimitiveIntersector::intersect(pre,ray,prim,num,bvh->scene,lazy_node);
        ray_far = ray.tfar;
        if (unlikely(lazy_node)) {
          stackPtr->ptr = lazy_node;
          stackPtr->dist = inf;
          stackPtr++;
        }
      }

      AVX_ZERO_UPPER();
    }
    
    template<bool robust, typename PrimitiveIntersector>
    void BVH8Intersector1<robust,PrimitiveIntersector>::occluded(const BVH8* bvh, Ray& ray)
    {
      /*! perform per ray precalculations required by the primitive intersector */
      Precalculations pre(ray,bvh);

      /*! stack state */
      NodeRef stack[stackSize];  //!< stack of nodes that still need to get traversed
      NodeRef* stackPtr = stack+1;        //!< current stack pointer
      NodeRef* stackEnd = stack+stackSize;
      stack[0] = bvh->root;
        
      /* filter out invalid rays */
#if defined(RTCORE_IGNORE_INVALID_RAYS)
      if (!ray.valid()) return;
#endif

      /* verify correct input */
      assert(ray.tnear > -FLT_MIN);
      //assert(!(types & BVH4::FLAG_NODE_MB) || (ray.time >= 0.0f && ray.time <= 1.0f));

      /*! load the ray into SIMD registers */
      const Vec3f8 norg(-ray.org.x,-ray.org.y,-ray.org.z);
      const Vec3fa ray_rdir = rcp_safe(ray.dir);
      const Vec3f8 rdir(ray_rdir.x,ray_rdir.y,ray_rdir.z);
      const Vec3fa ray_org_rdir = ray.org*ray_rdir;
      const Vec3f8 org_rdir(ray_org_rdir.x,ray_org_rdir.y,ray_org_rdir.z);
      const float8  ray_near(ray.tnear);
      float8 ray_far(ray.tfar);
      
      /*! offsets to select the side that becomes the lower or upper bound */
      const size_t nearX = ray_rdir.x >= 0 ? 0*sizeof(float8) : 1*sizeof(float8);
      const size_t nearY = ray_rdir.y >= 0 ? 2*sizeof(float8) : 3*sizeof(float8);
      const size_t nearZ = ray_rdir.z >= 0 ? 4*sizeof(float8) : 5*sizeof(float8);

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
          
#if defined(__AVX2__)
          const float8 tNear = maxi(maxi(tNearX,tNearY),maxi(tNearZ,ray_near));
          const float8 tFar  = mini(mini(tFarX ,tFarY ),mini(tFarZ ,ray_far ));
          const bool8 vmask = cast(tNear) > cast(tFar);
          size_t mask = movemask(vmask)^0xff;
#else
          const float8 tNear = max(tNearX,tNearY,tNearZ,ray_near);
          const float8 tFar  = min(tFarX ,tFarY ,tFarZ ,ray_far);
          const bool8 vmask = tNear <= tFar;
          size_t mask = movemask(vmask);
#endif
          
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
        size_t num; Primitive* prim = (Primitive*) cur.leaf(num);
        size_t lazy_node = 0;
        if (PrimitiveIntersector::occluded(pre,ray,prim,num,bvh->scene,lazy_node)) {
          ray.geomID = 0;
          break;
        }

        if (unlikely(lazy_node)) {
          *stackPtr = (NodeRef)lazy_node;
          stackPtr++;
        }
      }
      AVX_ZERO_UPPER();
    }

    DEFINE_INTERSECTOR1(BVH8Triangle4Intersector1Moeller,BVH8Intersector1<false COMMA ArrayIntersector1<TriangleNIntersector1MoellerTrumbore<Triangle4 COMMA true> > >);
    DEFINE_INTERSECTOR1(BVH8Triangle8Intersector1Moeller,BVH8Intersector1<false COMMA ArrayIntersector1<TriangleNIntersector1MoellerTrumbore<Triangle8 COMMA true> > >);
    DEFINE_INTERSECTOR1(BVH8TrianglePairs8Intersector1Moeller,BVH8Intersector1<false COMMA ArrayIntersector1<TrianglePairsNIntersector1MoellerTrumbore<TrianglePairs8 COMMA true> > >);
    
    //DEFINE_INTERSECTOR1(BVH8Triangle8vIntersector1Pluecker,BVH8Intersector1<true COMMA ArrayIntersector1<TriangleNvIntersector1Pluecker2<Triangle8v COMMA true> > >);
  }
}
